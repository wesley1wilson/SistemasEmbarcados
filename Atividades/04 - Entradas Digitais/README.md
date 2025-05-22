### Atividade 04: Controle de LED por botão
<b>Objetivo:</b><br>
Elaborar um controlador do estado de LEDs por meio de botão. Esta atividade será realizada no simulador Wokwi e deve utilizar a ESP-IDF como framework de desenvolvimento.<br>
<b>Material Necessário:</b><br>
- ESP32S3
- 4 LED
- 2 botões
- Outros componentes para implementar os drivers
- Conta no Wokwi (https://wokwi.com/)

<b>Passos para a Atividade:</b>
1. Elaborar o diagrama em bloco de um contador binário de 4 bits com dois botões de controle com os seguinte componentes:
- ESP32S3
- 4 LED
- 2 Buttons;
- Componentes para os circuitos de driver.
2. Elaborar o esquemático para o contador.
3. Configuração do Ambiente de Simulação:
- Acesse o simulador Wokwi e crie um novo projeto;
- Adicione o ESP32S3 ao projeto;
- Adicione 4 LED;
- Adicione 2 Buttons;
- Adicione os demais componentes.
4. Desenvolvimento do Código:<br>
Desenvolva um programa utilizando o ESP-IDF para implementar um contador binário de 4 bits. O valor atual do contador deve ser exibido utilizando 4 LEDs. Além disso, o sistema deve utilizar 2 botões conectados a entradas digitais, com as seguintes funcionalidades:<br>
○ Botão A: a cada acionamento, deve incrementar o valor do contador
conforme a unidade de incremento atual (padrão: +1).<br>
○ Botão B: alterna a unidade de incremento entre +1 e +2 a cada
acionamento.<br>
Obs:<br>
● O contador deve ser circular (isto é, ao ultrapassar o valor máximo de 4 bits,
ele retorna ao início com base no passo atual).<br>
● O contador é de 4 bits, portanto seu valor varia entre 0x0 (0 decimal ou b0000)
e 0xF (15 decimal ou b1111).<br>
● A unidade de incremento inicial é 1 unidade.<br>
● Ao acionar o Botão B, a unidade de incremento muda para 2 unidades. Um
novo acionamento alterna novamente para 1 unidade, e assim por diante.<br>
● A lógica de estouro (overflow) deve considerar a unidade de incremento:<br>
   &nbsp;&nbsp;&nbsp;&nbsp;  ○ Se o contador estiver em 0xF e a unidade de incremento for 1, o próximo
valor será 0x0.<br>
  &nbsp;&nbsp;&nbsp;&nbsp; ○ Se estiver em 0xE (14 decimal), o próximo valor com incremento de 2 será
0x0.<br>
  &nbsp;&nbsp;&nbsp;&nbsp; ○ Se estiver em 0xF (15 decimal) e incremento de 2, o resultado será 0x1.<br>
● O valor final do contador deve sempre estar dentro do intervalo de 4 bits (0x0
a 0xF) após cada incremento.<br>
● O debounce deve ser tratado por software (Não usar delay).
