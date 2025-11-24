# README — Atividade de Depuração no RP2040 (Embarcatech / Prof. Wilton)

## Descrição da Branch

Esta branch contém **a versão modificada do projeto original** de sensor de cor e luminosidade, adaptada **exclusivamente para a Atividade de Depuração** da aula ministrada pelo professor **Wilton**.

⚠️ **Importante:**
A branch principal (`main`) mantém o código original da atividade de “Sensor de Cor e Luminosidade” sem qualquer alteração, conforme instruções do mentor.

---

# Objetivo da Atividade

Demonstrar o uso do **Pico Debug (Picoprobe)** para depuração via **SWD**, utilizando:

* Breakpoints
* Step-by-step (Step Into / Step Over)
* Inspeção de variáveis
* Console UART para logs
* Análise da rotina crítica do projeto

O foco da depuração foi a **rotina de leitura e classificação da cor utilizando o sensor GY-33 (TCS34725)**.

---

# Modificações em Relação à Branch Principal

Nesta branch foram realizadas alterações *somente para depuração*, incluindo:

### 1. Simplificação do Projeto

* Remoção de:

  * BH1750 (sensor de luminosidade)
  * Display OLED I2C
  * Matriz WS2812
* Manutenção apenas do:

  * Sensor de Cor GY-33 (TCS34725)
  * LED indicativo (GPIO 13)

### 2. Adaptação para Debug com PicoProbe

* Saída `stdio` redirecionada para **UART0 (pinos 16/17)** via:

  ```c
  stdio_uart_init_full(uart0, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
  ```
* Desativação da USB para `stdin/stdout` no `CMakeLists.txt`:

  ```cmake
  pico_enable_stdio_usb(medidor_luz_cor 0)
  pico_enable_stdio_uart(medidor_luz_cor 1)
  ```

### 3. Inclusão de Prints e Variáveis para Inspeção

* Prints durante leitura do sensor:

  ```c
  printf("Amostra %lu -> R=%u, G=%u, B=%u, C=%u | Cor: %s\n", ...);
  ```
* Variáveis preparadas com clareza para o painel de depuração:

  * `r`, `g`, `b`, `c`
  * `cor_atual`
  * `amostra_id`

### 4. Código estruturado para demonstrar depuração passo a passo

* Linha estratégica para breakpoint:

  ```c
  gy33_read_rgbc(&r, &g, &b, &c);
  ```
* Função `nome_cor()` com múltiplas condições para Step Into.

---

# Rotina Crítica Depurada

A rotina selecionada para análise foi:

### Leitura + Classificação de Cor

Funções envolvidas:

```c
gy33_read_rgbc(&r, &g, &b, &c);
cor_atual = nome_cor(r, g, b);
```

Essas linhas permitiram:

* Inspecionar valores brutos do sensor em tempo real
* Entrar na lógica interna da classificação (`Step Into`)
* Validar thresholds e dominâncias
* Introduzir breakpoints nos pontos-chave da máquina de decisão

---

# Como Executar a Depuração (Passo a Passo)

### 1. Carregar o firmware-picoprobe.uf2 no Pico debugger

Baixar:
[https://github.com/raspberrypi/picoprobe/releases/latest/download/picoprobe.uf2](https://github.com/raspberrypi/picoprobe/releases/latest/download/picoprobe.uf2)

Gravar no Pico → vira um **Picoprobe**.

### 2. Conectar Picoprobe ao Pico alvo via SWD

| Picoprobe | Pico Alvo |
| --------- | --------- |
| GP2       | SWCLK     |
| GP3       | SWDIO     |
| GND       | GND       |

⚠️ **Pico alvo precisa estar alimentado (USB).**

### 3. Rodar Flash + Debug

No VS Code:

* `Flash Project (SWD)`
* `Start Debugging`

---

# Demonstração de Depuração

> [Vídeo de Demonstração](https://drive.google.com/file/d/11zIy-CqNO5Bkbbo9owxklHbDiZSsgOzJ/view?usp=drive_link)

O vídeo da atividade apresenta:

✔ Breakpoint em `gy33_read_rgbc()`
✔ Step Over para observar leitura RGBC
✔ Step Into em `nome_cor()`
✔ Variáveis monitoradas no painel WATCH
✔ LED travando durante breakpoints
✔ Interpretação dos valores com objetos coloridos

---

# Status da Atividade

**Concluída com sucesso.**
Depuração via Picoprobe funcionando.
Vídeo gravado conforme solicitado.
