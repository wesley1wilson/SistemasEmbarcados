### Atividade 06: Controle de PWM para LED e display LCD I2C

<b>Objetivo:</b><br>
Ampliar o projeto das atividades anteriores para introduzir o uso de saídas PWM e comunicação serial (UART, I2C ou SPI), utilizando o ESP32S3 e o framework ESP-IDF, no ambiente de simulação Wokwi. Os conceitos abordados incluem: geração de sinal PWM, controle de brilho de LED, e transmissão de dados via comunicação serial.<br>
<b>Material Necessário:</b><br>
- ESP32S3
- 4 LEDs (para exibição do contador binário)
- 2 botões (push buttons)
- 1 LED adicional (para controle de brilho via PWM)
- 1 Display LCD com interface I2C

<b>Passos para a atividade:</b><br>
1. Atualize o diagrama da Atividade 04/05 incluindo:
- 1 LED;
- 1 Display LCD.
2. Atualize o esquemático com os novos módulos e componentes.
3. Desenvolvimento do código:
  Desenvolva um programa utilizando o ESP-IDF para implementar um contador binário de 4 bits. O valor atual do contador deve ser exibido utilizando 4 LEDs. Além disso, o sistema deve utilizar 2 botões conectados a entradas digitais, 1 LED adicional com controle de brilho e um display LCD.<br>
  <b>Parte A - Funcionalidade dos botões:</b><br>
  ○ Botão A: a cada acionamento, deve incrementar o valor do contador conforme a unidade de incremento atual (padrão: +1).<br>
  ○ Botão B: a cada acionamento, deve decrementar o valor do contador conforme a unidade de incremento atual (padrão: -1).<br>
  <b>Parte A - PWM: Controle de Brilho do LED</b><br>
  ○ Use o driver (PWM) do ESP-IDF.<br>
  ○ O brilho do LED PWM deve ser proporcional ao valor do contador de 4 bits.<br>
  • 0x0 -> brilho mínimo<br>
  • 0xF -> brilho máximo<br>
<b>Parte B - LCD I2C: Exibir o Valor do Contador</b><br>
○ Configure o barramento I2C e inicialize o display LCD<br>
○ Mostre na primeira linha o valor do contador em formato hexadecimal.<br>
○ Mostre na primeira linha o valor do contador em formato decimal.<br>
○ O display deve ser atualizado sempre que o valor do contador for alterado.<br>
Obs.:<br>
• O contador deve ser circular (isto é, ao ultrapassar o valor máximo de 4 bits, ele retorna ao início com base no passo atual).<br>
• O contador é de 4 bits, portanto seu valor varia entre 0x0 (0 decimal o b0000) e 0xF (15 decimal ou b1111).<br>
• A unidade de incremental é 1 unidade.<br>
• O valor final do contador deve sempre estar dentro do intervalo de 4 bits (0x0 a 0xF) após cada incremento.<br>
• O debounce deve ser tratado por software (não usar delay).<br>
• É obrigatório o uso de interrupções para a leitura do estado dos botões.
