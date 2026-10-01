# 🏎️ Módulo de Plausibilidade e Monitoramento do ETC

Este repositório contém o código de firmware desenvolvido em **C / ESP-IDF** para o ecossistema de controle eletrônico de aceleração (**ETC — Electronic Throttle Control**) do protótipo da equipe **Fórmula UFMG**.
O módulo executa a leitura, conversão, monitoramento não linear e validação de segurança em malha rápida (200 Hz / task de 5 ms) para garantir total conformidade com o regulamento de segurança da **Formula SAE (FSAE)**.

---

## 📌 Principais Funcionalidades

* **Monitoramento Unificado de Sensores:** Leitura síncrona via ADC1 dos canais de pedal (**APPS1 / APPS2**), borboleta (**TPS1 / TPS2**) e sensor de pressão de freio (**BSE**).
* **Tratamento Físico e Sinal Direto/Invertido:** Suporte a sinais complementares/invertidos (ex: TPS1 invertido e TPS2 direto) com calibração baseada em dados reais e limitadores de saturação (*anti-roll*).
* **Mapeamento de Setpoint Não Linear:** Cálculo da abertura alvo da borboleta via interpolação linear por trechos (*Look-up Table*), permitindo curvas de aceleração customizadas para dinâmica do veículo.
* **Detecção de Falhas Elétricas e de Circuito:**
 
  * Circuito aberto / desconexão de sensores de pedal e borboleta.
  * Curto ao GND ou VCC no sensor de freio (**BSE**).

* **Validação de Plausibilidade Regulamentar:**
  * **[T.4.2.5] Desacordo de APPS:** Divergência $>10\%$ entre sensores por mais de $100\text{ ms}$.
  * **[IC.4.4.4] Desacordo de TPS:** Divergência $>10\%$ entre sensores por mais de $100\text{ ms}$.
  * **[IC.4.7.2] Erro de Seguimento do ETC:** Desvio $>10\%$ entre o TPS médio e o Setpoint Alvo por mais de $1.0\text{ s}$.
  * **[T.4.3.4] Falha Elétrica de BSE:** Sinal fora da faixa de operabilidade por mais de $100\text{ ms}$.

* **Histerese de Recuperação de Segurança:** Trava de corte de segurança por pelo menos $1.0\text{ s}$ ($1000\text{ ms}$) após a normalização do sinal antes de restabelecer a operação normal.
* **Interrupção de Segurança em Hardware:** Atuação direta no pino GPIO de *Shutdown* (Open-Drain / Active Low) cortando a linha de segurança em situações de implausibilidade crítica.
* **Telemetria de Diagnóstico Serial:** Log formatado via terminal ANSI colorido a cada $1.0\text{ s}$ para rápida depuração em bancada ou box.

---

## 🛠️ Arquitetura e Hardware

* **Plataforma:** ESP32 (ESP-IDF)
* **RTOS:** FreeRTOS (Task dedicada pinned no **Core 1**, Prioridade Alta `12`)
* **Frequência da Malha:** 200 Hz (Período de amostragem de $5\text{ ms}$)
* **Watchdog:** Integrado via `esp_task_wdt`
* **Mapeamento de Pinos:**
  * `GPIO36` — APPS 1 (ADC1_CH3)
  * `GPIO39` — APPS 2 (ADC1_CH0)
  * `GPIO32` — TPS 1 (ADC1_CH4)
  * `GPIO33` — TPS 2 (ADC1_CH5)
  * `GPIO34` — BSE (ADC1_CH6)
  * `GPIO12` — Linha de Shutdown (Saída Digital)

---

## 🎛️ Calibração de Sensores (Atualizado)

Após rodadas de ajuste fino e testes práticos, os limites do comparador foram recalibrados para garantir estabilidade do sinal. A atual configuração de limites brutos do ADC é a seguinte:

```
#define MIN_APPS_1      520.0f
#define MAX_APPS_1      1960.0f

#define MIN_APPS_2      500.0f 
#define MAX_APPS_2      1980.0f 

#define MIN_TPS_1       2150.0f  // Pedal Solto (Equivale a 0%)
#define MAX_TPS_1        92.0f   // Pedal no Fundo (Equivale a 100% -> Invertido)

#define MIN_TPS_2       435.0f   // Pedal Solto (Equivale a 0%)
#define MAX_TPS_2       2330.0f  // Pedal no Fundo (Equivale a 100% -> Direto)
```

*Aviso sobre a calibração do APPS:*
Os valores mínimos do APPS 1 e 2 foram intencionalmente elevados (para a faixa de 500-520) para criar uma zona morta (deadband) na base do pedal. Isso foi feito para mitigar flutuações de ruído no final da escala (ex: oscilações de 430 para 460 com o pedal em repouso), que antes estavam causando o acionamento indevido do corte de segurança e "matando" o carro.

⚠️ Orientação para Pré-Testes (Variação de Tensão):
A partir desta versão, para todas as novas calibrações pré-teste realizadas na pista ou bancada, é obrigatório anotar o valor da tensão da bateria no momento da calibração. O objetivo dessa coleta de dados é permitir uma avaliação geral posterior para determinarmos o quanto a variação da tensão nominal da bateria afeta a calibração de tensão dos sensores.

---

## 💻 Estrutura da Tabela de Calibração (Setpoint)

O código utiliza a seguinte curva não linear para converter a intenção do piloto (APPS) no Setpoint da borboleta (TPS):

| APPS (Pedal %) | TPS Alvo (Borboleta %) |
| --- | --- |
| 0.0% | 0.0% |
| 25.0% | 24.5% |
| 50.0% | 42.5% |
| 75.0% | 63.4% |
| 100.0% | 100.0% |

---
