// ----------------------------------------------------------------------------
//                      M A L H A   A B E R T A   E   F E C H A D A
// ----------------------------------------------------------------------------
// Projeto: Controle de Nível e Vazão de Água
// Autor:   Nicole Maia Argondizzi e Isaac Miranda Camargos
// Descrição: Leitura de nível (HC-SR04) + Vazão (YF-S201) 
//            - Bomba inicia em 100%
//            - Comando Serial:
//                1) "Bomba:Ligada"    => duty_cycle = 100
//                2) "Bomba:Desligada" => duty_cycle = 0
//                3) "Vazao:XX.XX"     => ajusta setpoint de vazão (L/min)
//                4) "Nivel:XX.XX"     => ajusta setpoint de nivel (cm)
//
// --------------------------------Conexões Físicas--------------------------------------------
// 
// Sensor ultrassônico HC-SR04: 
// - VCC -> 5V do Arduino
// - GND -> GND do Arduino
// - TRIG -> Pino 7 do Arduino
// - ECHO -> Pino 6 do Arduino
//
// Sensor de fluxo de água:
// - VCC (Fio Vermelho)-> 5V do Arduino
// - GND (Fio Preto) -> GND do Arduino
// - Saída do sensor (Fio Amarelo) -> pino 49 do Arduino.
//
// - Bomba d’água + Driver:
// - PWM --> Pino 3 do Arduino
// - 0V --> GND do Arduino
//
// ----------------------------------------------------------------------------


#include <NewPing.h>
#include <FreqMeasure.h>

// -------------------------- PINOS ----------------------------
#define TRIGGER_PIN 7
#define ECHO_PIN 6
#define MAX_DISTANCE 400
#define PWM_PIN 3

// -------------------------- ENUMERADOR ----------------------------
enum ModoControle { MALHA_ABERTA, MALHA_FECHADA };
ModoControle modoAtual = MALHA_ABERTA;

// -------------------------- SENSOR ULTRASSÔNICO ----------------------------
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
float dist = 37;
float medicao = 0;
float Nivel = 0;   // Nível medido (cm)
float NivelMax = 30;

// -------------------------- SENSOR VAZÃO ----------------------------
float vazao = 0.0;
float freqVazao = 0.0;
float pwmValue = 0;   

// -------------------------- MALHA ABERTA ----------------------------
float duty_cycle = 0;                                                       
float setpointVazao = 0;
float coefA = 0.36024061;
float coefB = -2.13070854;
float coefC = 11.36919872;
float coefD = 12.50493148;
float minValueFlow = 0;
float maxValueFlow = 9;

// -------------------------- MALHA FECHADA (PID) ----------------------------
float Nivel_ref = 10.0;  // Setpoint de nível (cm)
float Kc = 10.6;         // Ganho proporcional inicial (suave)
float Ti = 20.0;         // Constante de tempo integral inicial (suave)
float Td = 0.0;          // Constante de tempo derivativa inicial
float erro = 0.0;
float erroAnt = 0.0;
float P = 0.0, I = 0.0, D = 0.0, PID = 0.0;
float Ianterior = 0.0;

// -------------------------- TEMPO ----------------------------
unsigned long timer = 0;
float tempo = 0.0;


void setup() {
  Serial.begin(9600);
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  FreqMeasure.begin();
  timer = millis();
  Serial.println("Modo:MALHA_ABERTA");
}

void loop() {

  // ----- RECEBE COMANDOS ----- Não modificar ------------
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.equalsIgnoreCase("Modo:Aberta")) {
      modoAtual = MALHA_ABERTA;
      Serial.println("Modo:MALHA_ABERTA");
    } else if (comando.equalsIgnoreCase("Modo:Fechada")) {
      modoAtual = MALHA_FECHADA;
      Serial.println("Modo:MALHA_FECHADA");
    }

    // Comando universal
    else if (comando.startsWith("Calibragem:")) {
      dist = comando.substring(11).toFloat();
    }

    // Controles específicos por modo
    if (modoAtual == MALHA_ABERTA) {
      if (comando.startsWith("Bomba:")) {
        duty_cycle = comando.endsWith("Ligada") ? 100 : 0;
      } else if (comando.startsWith("Vazao:")) {
        float temp = comando.substring(6).toFloat();
        temp = constrain(temp, minValueFlow, maxValueFlow);
        setpointVazao = temp;
        duty_cycle = coefA * pow(setpointVazao, 3) + coefB * pow(setpointVazao, 2) + coefC * setpointVazao + coefD;
      } else if (comando.startsWith("Calibrar:")) {
        int idx1 = comando.indexOf(',');
        int idx2 = comando.indexOf(',', idx1 + 1);
        int idx3 = comando.indexOf(',', idx2 + 1);
        if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
          coefA = comando.substring(9, idx1).toFloat();
          coefB = comando.substring(idx1 + 1, idx2).toFloat();
          coefC = comando.substring(idx2 + 1, idx3).toFloat();
          coefD = comando.substring(idx3 + 1).toFloat();
        }
      }
    } else if (modoAtual == MALHA_FECHADA) {
      if (comando.startsWith("Nivel:")) {
        Nivel_ref = comando.substring(6).toFloat();
      } else if (comando.startsWith("Kc:")) {
        Kc = comando.substring(3).toFloat();
      } else if (comando.startsWith("Ti:")) {
        Ti = comando.substring(3).toFloat();
      } else if (comando.startsWith("Td:")) {
        Td = comando.substring(3).toFloat();
      }
    }
  }

  // ----- TEMPO -----
  unsigned long now = millis();
  tempo = (now - timer) / 1000.0;
  timer = now;

  // ----- LEITURAS -----

  // ----- NÍVEL -----
  medicao = sonar.ping_median(3);
  Nivel = dist - (medicao / 58.8);


  // ----- VAZÃO -----
  float rawCount = FreqMeasure.read();
  freqVazao = FreqMeasure.countToFrequency(rawCount);
  vazao = freqVazao / 7.5;

  // ----- CONTROLE -----
  if (modoAtual == MALHA_ABERTA) {
    pwmValue = (duty_cycle / 100.0) * 255.0;
  } else {
    erroAnt = erro;
    erro = Nivel_ref - Nivel;
    P = Kc * erro;
    I = Ianterior + (Kc / Ti) * erro * tempo;
    D = Kc * Td * (erro - erroAnt) / tempo;
    PID = P + I + D;
    PID = constrain(PID, 0, 100);
    Ianterior = I;
    pwmValue = (PID / 100.0) * 255.0;
  }


  // ----- DESLIGAR A BOMBA QUANDO ULTRAPASSAR O NIVEL DE 34 CM -----
  if (Nivel > NivelMax) {
    pwmValue = 0;
    delay(20000);
  }

  analogWrite(PWM_PIN, (int)pwmValue);


  // ----- ENVIO SERIAL ----- Não mudar -----------------------
  String statusBomba = (pwmValue > 1) ? "Ligada" : "Desligada";
  Serial.print("Bomba:");
  Serial.print(statusBomba);
  Serial.print("|Nivel:");
  Serial.print(Nivel, 2);
  Serial.print("|Vazao:");
  Serial.println(vazao, 2);
  Serial.print("|Duty:");
  Serial.println(duty_cycle, 1);

  delay(1000);
  
}
