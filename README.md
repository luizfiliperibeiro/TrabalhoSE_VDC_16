# 🎨 Sistema Inteligente de Detecção de Cores e Luminosidade com Feedback Visual e Sonoro 

Este projeto implementa um **sistema interativo de detecção de cores**, integrando sensores e atuadores na plataforma **BitDogLab + RP2040 (Raspberry Pi Pico W)**.  
O sistema identifica cores com o **sensor GY-33 (TCS34725)**, mede luminosidade ambiente com o **BH1750**, e fornece **feedback visual, sonoro e textual** via **matriz WS2812**, **LED RGB discreto**, **buzzer PWM** e **display OLED SSD1306**.

A interação é controlada por **botões físicos**, permitindo alternar modos de operação e reiniciar o dispositivo.

---

## 🧰 Componentes Utilizados

1. 🖥️ **Display OLED SSD1306** (I2C1)  
2. 🌞 **Sensor de Luminosidade BH1750** (I2C0)  
3. 🎨 **Sensor de Cores GY-33 (TCS34725)** (I2C0)  
4. 🌈 **Matriz de LEDs WS2812 (25 pixels)**  
5. 🔊 **Buzzer PWM** (pino 21)  
6. 🔘 **Botões A e BOOTSEL**  
7. 💡 **LED RGB discreto** (pinos 11, 12, 13)

---

## ⚙️ Funcionalidades

- ✅ **Detecção automática de cores** (Vermelho, Verde, Azul, Mista...)  
- 🌈 **Matriz WS2812** exibe a cor detectada com intensidade ajustada pela **iluminação ambiente (BH1750)**  
- 💡 **LED RGB discreto** pisca em ciclo de testes (5s por cor) sem travar o loop principal  
- 🔊 **Buzzer PWM** emite alertas quando:
  - O ambiente está muito escuro (lux baixo)  
  - Uma cor **vermelha intensa** é detectada  
- 🖥️ **Display OLED** mostra informações em tempo real:  
  - Modo atual  
  - Intensidade de luz (Lux)  
  - Nome da cor detectada  
  - Valores RGB + canal Clear  

- 🔘 **Botão A**: alterna entre os modos:  
  - Sensor (automático)  
  - Vermelho, Verde, Azul, Branco, Apagado  

- 🔘 **Botão BOOTSEL (B)**: reinicia a Pico W em modo de boot USB

---

## 🚀 Como Usar

1. Clone este repositório:

   ```bash
   git clone https://github.com/luizfiliperibeiro/TrabalhoSE_VDC_16.git
   ```

2. Importe o projeto na extensão **Raspberry Pi Pico** no VS Code ou outro ambiente de desenvolvimento compatível.

3. Compile o projeto.

4. Conecte a **BitDogLab** via USB.

5. Acompanhe os logs seriais e a exibição no display OLED.

---

## 📊 Demonstração Visual

Durante a execução:  
- O **OLED** exibe status do sistema.  
- O **LED RGB discreto** percorre cores de teste.  
- A **matriz WS2812** mostra a cor detectada.  
- O **buzzer** alerta em condições críticas.  

📹 Vídeo demonstrativo disponível no Google Drive:  
[🔗 Assista aqui](https://drive.google.com/drive/folders/1IDnXWWkpMzNkOhBNlhSimZaoKrxvoTON?usp=drive_link)

---

## 🔧 Estrutura do Código

- **main.c**: lógica principal do sistema  
- **ssd1306.h / font.h**: controle do OLED  
- **bh1750_light_sensor.h**: leitura de luminosidade  
- **ws2812.h**: controle da matriz de LEDs  

---

📌 Este projeto combina **sensoriamento, processamento e atuação em tempo real**, demonstrando um sistema embarcado responsivo e interativo.  
