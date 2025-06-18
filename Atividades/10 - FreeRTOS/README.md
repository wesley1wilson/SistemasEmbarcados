# Atividade 10: FreeRTOS

## Objetivo
A partir do projeto da atividade 08, utilizar o FreeRTOS para organizar a estrutura do programa.

## Material Necessário
- ESP32‑S3  
- 4 LEDs  
- 2 botões (push buttons)  
- 1 Buzzer (controle por PWM)  
- 1 Display LCD com interface I2C  
- 1 NTC (sensor de temperatura analógico)  
- 1 microSD Card  

## Atividades

### 1. Desenvolvimento do Código  
Desenvolva um programa utilizando o ESP‑IDF para salvar a leitura da temperatura adquirida por meio do NTC em um SD Card.  
Para cada parte abaixo, implemente uma _thread_ para cada funcionalidade do programa.

#### Parte A – Funcionalidade dos botões
- **Botão A**: a cada acionamento, deve incrementar a temperatura de alarme (padrão: +5).  
- **Botão B**: a cada acionamento, deve decrementar a temperatura de alarme (padrão: −5).  

#### Parte B – PWM: Gerar o alarme sonoro
- Use o driver PWM do ESP‑IDF.  
- O alerta sonoro deve ser acionado quando a temperatura registrada pelo NTC for maior que a temperatura de alarme.  
- O alerta sonoro só deve ser desligado quando a temperatura do NTC estiver abaixo da temperatura de alarme.  

#### Parte C – LCD I2C: Exibir a temperatura registrada no NTC e a temperatura de alarme
- Configure o barramento I2C e inicialize o display LCD.  
- Mostre na primeira linha o valor da temperatura no NTC.  
- Mostre na segunda linha o valor da temperatura de alarme atual.  
- Atualize o display sempre que os valores das temperaturas forem alterados.  

#### Parte D – Display 7‑Segmentos: Sinalizar a aproximação da temperatura do NTC à temperatura de alarme
- Exibir o dígito **0** quando a temperatura do NTC estiver a 20 °C da temperatura de alarme.  
- Exibir o dígito **3** quando a temperatura do NTC estiver a 15 °C da temperatura de alarme.  
- Exibir o dígito **7** quando a temperatura do NTC estiver a 10 °C da temperatura de alarme.  
- Exibir o dígito **D** quando a temperatura do NTC estiver a 2 °C da temperatura de alarme.  
- Exibir e piscar o dígito **F** quando a temperatura do NTC for maior ou igual à temperatura de alarme.  
- O display deve continuar piscando enquanto a temperatura do NTC for maior que a temperatura de alarme.  

#### Parte E – SD Card
- Salvar todas as leituras realizadas pelo ADC da temperatura do NTC no SD Card.  

> **Obs.:**  
> - O debounce deve ser tratado por software (não usar delay).  
> - É obrigatório o uso de interrupções para a leitura do estado dos botões.  
