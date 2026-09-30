/*
 * ============================================================================
 * FSAE 2026 — Formula UFMG
 * MÓDULO DE PLAUSIBILIDADE E MONITORAMENTO DO ETC (Electronic Throttle Control)
 * ============================================================================
 * Descrição: Task única de plausibilidade unificada (APPS, TPS e BSE).
 * Setpoint não-linear da borboleta implementado via interpolação em tabela.
 * Histerese de recuperação de falhas em 1 segundo.
 * ============================================================================
 */

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include <stdint.h>
#include <stdio.h>

// ============================================================================
//   CONSTANTES DE CALIBRAÇÃO E CONFIGURAÇÃO (BASEADAS EM DADOS RAW COLETADOS)
// ============================================================================

#define MIN_APPS_1      520.0f
#define MAX_APPS_1      1960.0f

#define MIN_APPS_2      500.0f 
#define MAX_APPS_2      1980.0f 

#define MIN_TPS_1       2150.0f  // Pedal Solto (Equivale a 0%)
#define MAX_TPS_1        92.0f  // Pedal no Fundo (Equivale a 100% -> Invertido)

#define MIN_TPS_2        435.0f  // Pedal Solto (Equivale a 0%)
#define MAX_TPS_2       2330.0f  // Pedal no Fundo (Equivale a 100% -> Direto)


// Garantia de intervalos absolutos positivos para as equações de conversão linear
#define INTERVALO_APPS_1 (MAX_APPS_1 - MIN_APPS_1)
#define INTERVALO_APPS_2 (MAX_APPS_2 - MIN_APPS_2)
#define INTERVALO_TPS_1  (MIN_TPS_1 - MAX_TPS_1) // Invertido para delta positivo
#define INTERVALO_TPS_2  (MAX_TPS_2 - MIN_TPS_2)

// Limites inferiores de segurança para detecção de desconexão (Circuito Aberto)
#define CIRC_ABERTO_APPS_1  300
#define CIRC_ABERTO_APPS_2  300
#define CIRC_ABERTO_TPS_1   20
#define CIRC_ABERTO_TPS_2   150

// [T.4.3.4] Limites operacionais regulamentares para o sensor BSE
#define BSE_CIRC_ABERTO_RAW  150         
#define BSE_CURTO_VCC_RAW    4095        
#define TEMPO_FALHA_MAX_MS   100         

// Tempo para o carro poder voltar após sanada a implausibilidade (1s)
#define TEMPO_HISTERESE_MS   1000

// Pinout ESP32
#define APPS1_ADC_CHANNEL    ADC_CHANNEL_3 // GPIO36
#define APPS2_ADC_CHANNEL    ADC_CHANNEL_0 // GPIO39
#define TPS1_ADC_CHANNEL     ADC_CHANNEL_4 // GPIO32
#define TPS2_ADC_CHANNEL     ADC_CHANNEL_5 // GPIO33
#define BSE_ADC_CHANNEL      ADC_CHANNEL_6 // GPIO34
#define SAIDA                GPIO_NUM_12   

#define TAG_ADC "ADC_CORE"

// Cores ANSI para formatação do terminal
#define COR_RESET   "\033[0m"
#define COR_TENSAO  "\033[36m"    
#define COR_PERC    "\033[35m"    
#define COR_AVISO   "\033[33m"    
#define COR_OK      "\033[32m"    
#define COR_ERRO    "\033[31m"    
#define COR_PISCA   "\033[5;31m"  

// Tabela de calibração não-linear para o Setpoint da Borboleta
typedef struct {
    float pedal;
    float borboleta;
} ponto_calibracao_t;

static const ponto_calibracao_t TABELA_MAPEAMENTO[] = {
    {0.0f, 0.0f},   {5.0f, 4.7f},   {10.0f, 9.8f},  {15.0f, 14.3f}, {20.0f, 19.0f},
    {25.0f, 24.5f}, {30.0f, 28.1f}, {40.0f, 36.3f}, {50.0f, 42.5f}, {60.0f, 48.3f},
    {70.0f, 58.7f}, {80.0f, 63.4f}, {90.0f, 70.0f}, {95.0f, 80.0f}, {100.0f, 100.0f}
};
#define NUM_PONTOS (sizeof(TABELA_MAPEAMENTO) / sizeof(TABELA_MAPEAMENTO[0]))

// ============================================================================
//   ESTRUTURAS E VARIÁVEIS GLOBAIS
// ============================================================================

typedef struct {
    int apps_1;
    int apps_2;
    int tps_1;
    int tps_2;
    int bse;
} leitura_ADC_t;

volatile bool g_etc_shutdown = false;
volatile bool g_bse_shutdown = false;

gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << SAIDA),
    .mode         = GPIO_MODE_OUTPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE
};

adc_oneshot_unit_handle_t adc1_handle;

// ============================================================================
//   FUNÇÕES AUXILIARES DE HARDWARE E MATEMÁTICA
// ============================================================================

static void adc_setup(void)
{
    // Como o GPIO34 (BSE) também pertence à UNIT 1, centralizamos tudo nela
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config_adc1 = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, APPS1_ADC_CHANNEL, &config_adc1));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, APPS2_ADC_CHANNEL, &config_adc1));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, TPS1_ADC_CHANNEL,  &config_adc1));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, TPS2_ADC_CHANNEL,  &config_adc1));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BSE_ADC_CHANNEL,   &config_adc1));

    ESP_LOGI(TAG_ADC, "Estrutura de leitura RAW unificada para ADC1 inicializada.");
}

static leitura_ADC_t fetch_adc(void)
{
    leitura_ADC_t data;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, APPS1_ADC_CHANNEL, &data.apps_1));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, APPS2_ADC_CHANNEL, &data.apps_2));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, TPS1_ADC_CHANNEL,  &data.tps_1));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, TPS2_ADC_CHANNEL,  &data.tps_2));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BSE_ADC_CHANNEL,   &data.bse));
    return data;
}

static void atualizar_linha_shutdown(void)
{
   if (g_etc_shutdown) {
       gpio_set_level(SAIDA, 0); // Abre a linha de segurança
    } else {
       gpio_set_level(SAIDA, 1); // Mantém operacional
    }
}

// Interpolação linear por trechos para calcular o Setpoint alvo da borboleta
static float calcular_setpoint_borboleta(float apps_med)
{
    if (apps_med <= TABELA_MAPEAMENTO[0].pedal) return TABELA_MAPEAMENTO[0].borboleta;
    if (apps_med >= TABELA_MAPEAMENTO[NUM_PONTOS - 1].pedal) return TABELA_MAPEAMENTO[NUM_PONTOS - 1].borboleta;

    for (size_t i = 0; i < NUM_PONTOS - 1; i++) {
        if (apps_med >= TABELA_MAPEAMENTO[i].pedal && apps_med <= TABELA_MAPEAMENTO[i+1].pedal) {
            float p0 = TABELA_MAPEAMENTO[i].pedal;
            float p1 = TABELA_MAPEAMENTO[i+1].pedal;
            float b0 = TABELA_MAPEAMENTO[i].borboleta;
            float b1 = TABELA_MAPEAMENTO[i+1].borboleta;
            
            // Fórmula padrão: y = y0 + ((x - x0) * (y1 - y0)) / (x1 - x0)
            return b0 + ((apps_med - p0) * (b1 - b0)) / (p1 - p0);
        }
    }
    return apps_med; // Fallback de segurança
}

// ============================================================================
//   TASK ÚNICA: PLAUSIBILIDADE E MONITORAMENTO UNIFICADO 
// ============================================================================

void plausibilidade_unificada_task(void *arg)
{
    leitura_ADC_t data_ADC;
    float tps[2], apps[2];
    float dif_tps = 0.0f, dif_apps = 0.0f, dif_tps_sp = 0.0f;

    // Timers de persistência de falha
    uint64_t t_ini_tps = 0, t_ini_apps = 0, t_ini_sp = 0, t_ini_bse = 0;
    bool comparando_tps = false, comparando_apps = false, comparando_tps_sp = false, em_falha_bse_potencial = false;
    bool tps_shutdown = false, apps_shutdown = false, setpoint_shutdown = false, bse_shutdown_local = false;

    // Timers de recuperação por histerese
    uint64_t t_ini_recuperacao_tps = 0, t_ini_recuperacao_apps = 0, t_ini_recuperacao_sp = 0, t_ini_recuperacao_bse = 0;
    bool recuperando_tps = false, recuperando_apps = false, recuperando_sp = false, recuperando_bse = false;

    static uint32_t cont_print_bloco = 0;

    esp_task_wdt_add(NULL);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        esp_task_wdt_reset();
        data_ADC = fetch_adc();
        cont_print_bloco++;

        char string_implausibilidade[128] = "Nenhuma falha ativa.";
        char string_status_carro[64]      = "SISTEMA OPERACIONAL (NORMAL)";
        char cor_status[16]               = COR_OK;

        // --- CONVERSÃO APPS ---
        apps[0] = (100.0f / INTERVALO_APPS_1) * ((float)data_ADC.apps_1 - MIN_APPS_1);
        apps[1] = (100.0f / INTERVALO_APPS_2) * ((float)data_ADC.apps_2 - MIN_APPS_2);

        // --- CONVERSÃO TPS ---
        tps[0] = (100.0f / INTERVALO_TPS_1) * (MIN_TPS_1 - (float)data_ADC.tps_1);
        tps[1] = (100.0f / INTERVALO_TPS_2) * ((float)data_ADC.tps_2 - MIN_TPS_2);

        // Limitadores de Saturação (Anti-estouro) condicionados à desconexão
        for (int i = 0; i < 2; i++) {
            // Saturação para o TPS (Apenas se não houver suspeita de circuito aberto)
            if (data_ADC.tps_1 >= CIRC_ABERTO_TPS_1 && data_ADC.tps_2 >= CIRC_ABERTO_TPS_2) {
                if (isnan(tps[i]) || tps[i] < 0.0f)   tps[i] = 0.0f;
                if (tps[i] > 100.0f)                  tps[i] = 100.0f;
            }

            // Saturação para o APPS (Apenas se não houver suspeita de circuito aberto)
            if (data_ADC.apps_1 >= CIRC_ABERTO_APPS_1 && data_ADC.apps_2 >= CIRC_ABERTO_APPS_2) {
                if (isnan(apps[i]) || apps[i] < 0.0f)  apps[i] = 0.0f;
                if (apps[i] > 100.0f)                  apps[i] = 100.0f;
            }
        }

        // Validação de circuito aberto
        bool tps_desconectado = (data_ADC.tps_1 < CIRC_ABERTO_TPS_1 || data_ADC.tps_2 < CIRC_ABERTO_TPS_2);
        if (tps_desconectado) { tps[0] = 100.0f; tps[1] = 0.0f; }

        bool apps_desconectado = (data_ADC.apps_1 < CIRC_ABERTO_APPS_1 || data_ADC.apps_2 < CIRC_ABERTO_APPS_2);
        if (apps_desconectado) { apps[0] = 100.0f; apps[1] = 0.0f; }

        bool bse_fora_de_faixa = (data_ADC.bse < BSE_CIRC_ABERTO_RAW || data_ADC.bse >= BSE_CURTO_VCC_RAW);

        float apps_med = (apps[0] + apps[1]) / 2.0f;
        float tps_med  = (tps[0]  + tps[1])  / 2.0f;
        
        // Setpoint mapeado de forma não-linear através da tabela
        float setpoint = calcular_setpoint_borboleta(apps_med);

        // --- [IC.4.4.4] MONITORAMENTO TPS1 x TPS2 ---
        dif_tps = fabs(tps[0] - tps[1]);
        if (dif_tps > 10.0f) {
            recuperando_tps = false;
            if (!comparando_tps) { t_ini_tps = esp_timer_get_time(); comparando_tps = true; } 
            else if (((esp_timer_get_time() - t_ini_tps) / 1000) > TEMPO_FALHA_MAX_MS) { tps_shutdown = true; }
        } else {
            comparando_tps = false;
            if (tps_shutdown) {
                if (!recuperando_tps) { t_ini_recuperacao_tps = esp_timer_get_time(); recuperando_tps = true; } 
                else if (((esp_timer_get_time() - t_ini_recuperacao_tps) / 1000) >= TEMPO_HISTERESE_MS) { tps_shutdown = false; recuperando_tps = false; }
            }
        }

        // --- [T.4.2.5] MONITORAMENTO APPS1 x APPS2 ---
        dif_apps = fabs(apps[0] - apps[1]);
        if (dif_apps > 10.0f) {
            recuperando_apps = false;
            if (!comparando_apps) { t_ini_apps = esp_timer_get_time(); comparando_apps = true; } 
            else if (((esp_timer_get_time() - t_ini_apps) / 1000) >= TEMPO_FALHA_MAX_MS) { apps_shutdown = true; }
        } else {
            comparando_apps = false;
            if (apps_shutdown) {
                if (!recuperando_apps) { t_ini_recuperacao_apps = esp_timer_get_time(); recuperando_apps = true; } 
                else if (((esp_timer_get_time() - t_ini_recuperacao_apps) / 1000) >= TEMPO_HISTERESE_MS) { apps_shutdown = false; recuperando_apps = false; }
            }
        }
        
        // --- [IC.4.7.2] ERRO DE SEGUIMENTO (TPSmed x Setpoint Não-Linear) ---
        dif_tps_sp = fabs(tps_med - setpoint);
        if (!apps_desconectado) {
            if (dif_tps_sp > 10.0f) {
                recuperando_sp = false;
                if (!comparando_tps_sp) { t_ini_sp = esp_timer_get_time(); comparando_tps_sp = true; } 
                else if (((esp_timer_get_time() - t_ini_sp) / 1000) > 1000) { setpoint_shutdown = true; }
            } else {
                comparando_tps_sp = false;
                if (setpoint_shutdown) {
                    if (!recuperando_sp) { t_ini_recuperacao_sp = esp_timer_get_time(); recuperando_sp = true; } 
                    else if (((esp_timer_get_time() - t_ini_recuperacao_sp) / 1000) >= TEMPO_HISTERESE_MS) { setpoint_shutdown = false; recuperando_sp = false; }
                }
            }
        }
        
        // --- [T.4.3.4] DIAGNÓSTICO DO SENSOR DE FREIO (BSE) ---
        if (bse_fora_de_faixa) {
            recuperando_bse = false;
            if (!em_falha_bse_potencial) { t_ini_bse = esp_timer_get_time(); em_falha_bse_potencial = true; }
            else if (((esp_timer_get_time() - t_ini_bse) / 1000) >= TEMPO_FALHA_MAX_MS) { bse_shutdown_local = true; }
        } else {
            em_falha_bse_potencial = false;
            if (bse_shutdown_local) {
                if (!recuperando_bse) { t_ini_recuperacao_bse = esp_timer_get_time(); recuperando_bse = true; }
                else if (((esp_timer_get_time() - t_ini_recuperacao_bse) / 1000) >= TEMPO_HISTERESE_MS) { bse_shutdown_local = false; recuperando_bse = false; }
            }
        }

        // Atualização de Flags Globais e Hardware
        g_etc_shutdown = (tps_shutdown || apps_shutdown || setpoint_shutdown);
        g_bse_shutdown = bse_shutdown_local;
        atualizar_linha_shutdown();

        // --- FORMATAÇÃO DE STRINGS DE DIAGNÓSTICO ---
        if (tps_desconectado || apps_desconectado || bse_fora_de_faixa) {
            snprintf(string_implausibilidade, sizeof(string_implausibilidade), "%sALERTA: Sensor Desconectado ou Rompimento de Fiação!%s", COR_AVISO, COR_RESET);
        } else if (dif_tps > 10.0f) {
            snprintf(string_implausibilidade, sizeof(string_implausibilidade), "%sAVISO [IC.4.4.4]: Divergência ativa entre TPS1 e TPS2 (%.1f%%)%s", COR_AVISO, dif_tps, COR_RESET);
        } else if (dif_apps > 10.0f) {
            snprintf(string_implausibilidade, sizeof(string_implausibilidade), "%sAVISO [T.4.2.5]: Divergência ativa entre APPS1 e APPS2 (%.1f%%)%s", COR_AVISO, dif_apps, COR_RESET);
        } else if (dif_tps_sp > 10.0f) {
            snprintf(string_implausibilidade, sizeof(string_implausibilidade), "%sAVISO [IC.4.7.2]: Borboleta fora do Alvo/Setpoint (Erro: %.1f%%)%s", COR_AVISO, dif_tps_sp, COR_RESET);
        }

        if (g_etc_shutdown || g_bse_shutdown) {
            snprintf(string_status_carro, sizeof(string_status_carro), "SHUTDOWN ATIVADO (SINAL DE CORTE ENVIADO)");
            snprintf(cor_status, sizeof(cor_status), COR_PISCA);
            if (tps_shutdown)      snprintf(string_implausibilidade, sizeof(string_implausibilidade), "%sFALHA CRÍTICA [IC.4.4.4]: Quebra de correlação TPS por >100ms! (Trava de 1s)%s", COR_ERRO, COR_RESET);
            if (apps_shutdown)     snprintf(string_implausibilidade, sizeof(string_implausibilidade), "%sFALHA CRÍTICA [T.4.2.5]: Quebra de correlação APPS por >100ms! (Trava de 1s)%s", COR_ERRO, COR_RESET);
            if (setpoint_shutdown) snprintf(string_implausibilidade, sizeof(string_implausibilidade), "%sFALHA CRÍTICA [IC.4.7.2]: Erro de seguimento do ETC por >1s! (Trava de 1s)%s", COR_ERRO, COR_RESET);
            //if (g_bse_shutdown)    snprintf(string_implausibilidade, sizeof(string_implausibilidade), "%sFALHA CRÍTICA [T.4.3.3]: Falha elétrica contínua no freio BSE! (Trava de 1s)%s", COR_ERRO, COR_RESET);
        }

        // Print síncrono a cada 1 segundo
        if (cont_print_bloco >= 500) {
            printf("\n--- Bloco de Diagnóstico FSAE Horeb (RAW Mode Unificado) ---\n");
            printf("%s[RAW ADC]%s APPS1: %4d | APPS2: %4d || TPS1: %4d | TPS2: %4d || BSE: %4d\n", 
                   COR_TENSAO, COR_RESET, data_ADC.apps_1, data_ADC.apps_2, data_ADC.tps_1, data_ADC.tps_2, data_ADC.bse);
            printf("%s[CONV %%]%s APPS1: %5.1f%% | APPS2: %5.1f%% || TPS1: %5.1f%% | TPS2: %5.1f%% || SETPOINT MAP: %5.1f%%\n", 
                   COR_PERC, COR_RESET, apps[0], apps[1], tps[0], tps[1], setpoint);
            printf("[AVISOS] %s\n", string_implausibilidade);
            printf("[STATUS] %s%s%s\n", cor_status, string_status_carro, COR_RESET);
            
            cont_print_bloco = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

// ============================================================================
//   MAIN
// ============================================================================

void app_main(void) 
{
    gpio_reset_pin(SAIDA);
    gpio_config(&io_conf);
    gpio_set_level(SAIDA, 1); // Ativa em low por segurança (Open-drain/Active Low design)
    
    adc_setup();

    ESP_LOGI("CORE", "Inicializando Task Única de Plausibilidade FSAE (50Hz)...");

    // Criada no Core 1 com prioridade alta para controle crítico de malha aberta/fechada
    xTaskCreatePinnedToCore(plausibilidade_unificada_task, "Plau_Unificada", 4096, NULL, 12, NULL, 1);
}