#include <TaskScheduler.h>

// --------------------------------------------------------------------------------------
// PROTOTIPOS DE FUNCIONES
// --------------------------------------------------------------------------------------
void funcionControl();
void funcionSerial();

// --------------------------------------------------------------------------------------
// 0. Pines
// --------------------------------------------------------------------------------------
const int PIN_ENTRADA_PLANTA = 9;
const int PIN_SALIDA_PLANTA = A0;
const int PIN_POTENCIOMETRO = A1;

// --------------------------------------------------------------------------------------
// 1. Parámetros generales
// --------------------------------------------------------------------------------------
const float REFERENCIA_BAJA = 1.0;
const float REFERENCIA_ALTA = 3.5;
const int PERIODO_ONDA_CUADRADA_MS = 6000;

const float VOLTAJE_MAX_ENTRADA_ADC = 5.0;
const int RESOLUCION_ADC = 1024;
const int RESOLUCION_PWM = 256;

// --------------------------------------------------------------------------------------
// 2. Controlador PI
// --------------------------------------------------------------------------------------
const float Kp = 1.785;
const float Ki = 0.7;
const float Ts = 0.1; // segundos

// --------------------------------------------------------------------------------------
// 3. Variables Globales
// --------------------------------------------------------------------------------------
float referencia_V = REFERENCIA_BAJA;
float salida_planta_V = 0.0;
float error_k = 0.0;
float u_k = 0.0;
float u_k_saturado = 0.0;
float e_integral = 0.0;

// Onda cuadrada
bool estadoOndaCuadrada = false;
unsigned long tiempoUltimoCambioReferencia = 0;

// Modo de referencia: true = potenciómetro, false = onda cuadrada
bool modo_manual = true;

// --------------------------------------------------------------------------------------
// 4. TaskScheduler
// --------------------------------------------------------------------------------------
Scheduler runner;

Task taskControl(Ts * 1000, TASK_FOREVER, &funcionControl); // cada 100ms
Task taskSerial(500, TASK_FOREVER, &funcionSerial);         // cada 500ms

// --------------------------------------------------------------------------------------
// 5. Funciones
// --------------------------------------------------------------------------------------
void funcionControl() {
  // CAMBIO DE REFERENCIA
  if (modo_manual) {
    // Modo potenciómetro
    int lectura = analogRead(PIN_POTENCIOMETRO);
    float voltaje = lectura * (VOLTAJE_MAX_ENTRADA_ADC / (RESOLUCION_ADC - 1.0));
    referencia_V = constrain(voltaje, REFERENCIA_BAJA, REFERENCIA_ALTA);
  } else {
    // Modo onda cuadrada automática
    if (millis() - tiempoUltimoCambioReferencia >= (PERIODO_ONDA_CUADRADA_MS / 2)) {
      estadoOndaCuadrada = !estadoOndaCuadrada;
      referencia_V = estadoOndaCuadrada ? REFERENCIA_ALTA : REFERENCIA_BAJA;
      tiempoUltimoCambioReferencia = millis();
    }
  }

  // LECTURA de Vo
  int lectura_ADC = analogRead(PIN_SALIDA_PLANTA);
  salida_planta_V = lectura_ADC * (VOLTAJE_MAX_ENTRADA_ADC / (RESOLUCION_ADC - 1.0));

  // CONTROL PI
  error_k = referencia_V - salida_planta_V;
  float u_p = Kp * error_k;
  e_integral += error_k;
  float u_i = Ki * Ts * e_integral;
  u_k = u_p + u_i;
  u_k_saturado = constrain(u_k, 0.0, 5.0);

  // SALIDA PWM
  int valor_PWM = u_k_saturado * ((RESOLUCION_PWM - 1.0) / VOLTAJE_MAX_ENTRADA_ADC);
  analogWrite(PIN_ENTRADA_PLANTA, valor_PWM);
}

void funcionSerial() {
  Serial.print(referencia_V, 3);
  Serial.print("\t");
  Serial.print(salida_planta_V, 3);
  Serial.print("\t");
  Serial.println(u_k_saturado, 3);
}

// --------------------------------------------------------------------------------------
// 6. Setup y Loop
// --------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENTRADA_PLANTA, OUTPUT);
  pinMode(PIN_SALIDA_PLANTA, INPUT);
  pinMode(PIN_POTENCIOMETRO, INPUT);

  runner.addTask(taskControl);
  runner.addTask(taskSerial);
  taskControl.enable();
  taskSerial.enable();

  tiempoUltimoCambioReferencia = millis();

  Serial.println("Sistema iniciado en modo: Potenciómetro");
}

void loop() {
  runner.execute();

  // Leer cambios desde el Monitor Serial
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      modo_manual = !modo_manual;
      if (modo_manual) {
        Serial.println("Cambiado a MODO: Potenciómetro");
      } else {
        Serial.println("Cambiado a MODO: Onda Cuadrada");
        tiempoUltimoCambioReferencia = millis(); // Reiniciar el tiempo
      }
    }
  }
}
