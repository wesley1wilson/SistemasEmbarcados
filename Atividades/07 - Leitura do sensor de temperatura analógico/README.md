## Atividade 07: Leitura do sensor de temperatura analógico

**Objetivo:**

Implementar um sistema de aquisição de dados analógicos utilizando o conversor ADC do ESP32-S3, no ambiente de simulação Wokwi. Os conceitos abordados incluem: Mapeamento de pinos analógicos, Configuração de parâmetros (faixas de tensão, resolução, atenuação, taxa de amostragem) e calibração do ADC.

**Material Necessário:**

* ESP32S3
* 4 LEDs
* 2 botões (push buttons)
* 1 Buzzer (Controle por PWM)
* 1 Display LCD com interface I2C
* 1 NTC (Sensor de temperatura analógico)

**Atividades:**

1.  **Diagrama de bloco atualizado.**
2.  **Esquemático atualizado com os novos componentes.**
3.  **Desenvolvimento do Código:**

    Desenvolva um programa utilizando o ESP-IDF para implementar um DAQ para ler a temperatura por meio do NTC e gerar um alerta sonoro ao ultrapassar a temperatura de alarme (Default 25 °C).

    **Parte A - Funcionalidade dos botões:**

    * **Botão A:** a cada acionamento, deve incrementar a temperatura de alarme (padrão: +5).
    * **Botão B:** a cada acionamento, deve decrementar a temperatura de alarme (padrão: -5).

    **Parte B – PWM: Gerar o alarme sonoro**

    * Use o driver (PWM) do ESP-IDF.
    * O alerta sonoro deve ser acionado quando a temperatura registrada pelo NTC for maior que a temperatura de alarme.
    * O alerta sonoro só deve ser desligado quando a temperatura do NTC estiver abaixo da temperatura de alarme.

**Parte C – LCD I2C: Exibir a temperatura registrada no NTC e a temperatura de alarme**

* Configure o barramento I2C e inicialize o display LCD.
* Mostre na **primeira linha** o valor da temperatura no NTC.
* Mostre na **primeira linha** o valor da temperatura de alarme atual.
* O display deve ser atualizado sempre que os valores das temperaturas forem alteradas.

**Parte D – LEDs: Sinalizar a aproximação da temperatura do NTC a temperatura de alarme**

* Ligar 1 LED quando a temperatura do NTC estiver a 20 °C da temperatura de alarme.
* Ligar 2 LED quando a temperatura do NTC estiver a 15 °C da temperatura de alarme.
* Ligar 3 LED quando a temperatura do NTC estiver a 10 °C da temperatura de alarme.
* Ligar 4 LED quando a temperatura do NTC estiver a 2 °C da temperatura de alarme.
* Piscar os 4 LEDs quando a temperatura do NTC for maior ou igual a temperatura de alarme.
* Os LEDs devem continuar piscando enquanto a temperatura do NTC for maior que a temperatura de alarme.

**Obs:**

* O debounce deve ser tratado por software (Não usar delay).
* É **obrigatório** o uso de interrupções para a leitura do estado dos botões.
