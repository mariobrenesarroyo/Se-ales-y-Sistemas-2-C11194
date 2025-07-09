#include <TaskScheduler.h>

// --------------------------------------------------------------------------------------
// PROTOTIPOS DE FUNCIONES
// --------------------------------------------------------------------------------------
void funcionControl();
void funcionSerial();

// --------------------------------------------------------------------------------------
// 0. Definiciones de Pines
// --------------------------------------------------------------------------------------
const int PIN_ENTRADA_PLANTA = 9;   // PWM
const int PIN_SALIDA_PLANTA = A0;  // Salida de la planta
const int PIN_POTENCIOMETRO = A1;  // Entrada del potenciómetro

// --------------------------------------------------------------------------------------
// 1. Parámetros de la Referencia y del Sistema
// --------------------------------------------------------------------------------------
const float REFERENCIA_BAJA = 1.0;
const float REFERENCIA_ALTA = 3.5;
const int PERIODO_ONDA_CUADRADA_MS = 6000;

const float VOLTAJE_MAX_ENTRADA_ADC = 5.0;
const int RESOLUCION_ADC = 1024;
const int RESOLUCION_PWM = 256;

// --------------------------------------------------------------------------------------
// 2. Parámetros del Controlador PI
// --------------------------------------------------------------------------------------
const float Kp = 1.785;
const float Ki = 0.7;
const float Ts = 0.1; // 100 ms

// --------------------------------------------------------------------------------------
// 3. Variables Globales para el Control
// --------------------------------------------------------------------------------------
float referencia_V = REFERENCIA_BAJA;
float salida_planta_V = 0.0;
float error_k = 0.0;
float u_k = 0.0;
float u_k_saturado = 0.0;
float e_integral = 0.0;

// Variables de la onda cuadrada (no usadas aquí pero las dejamos por si se activan)
bool estadoOndaCuadrada = false;
unsigned long tiempoUltimoCambioReferencia = 0;

// --------------------------------------------------------------------------------------
// 4. Variables para TaskScheduler
// --------------------------------------------------------------------------------------
Scheduler runner;

Task taskControl(Ts * 1000, TASK_FOREVER, &funcionControl);
Task taskSerial(1, TASK_FOREVER, &funcionSerial); // Cada 500 ms

// --------------------------------------------------------------------------------------
// 5. Funciones
// --------------------------------------------------------------------------------------
void funcionControl() {
  // ============================================
  // USAR POTENCIÓMETRO COMO REFERENCIA
  // ============================================
  int lecturaPotenciometro = analogRead(PIN_POTENCIOMETRO);
  float voltajePotenciometro = lecturaPotenciometro * (VOLTAJE_MAX_ENTRADA_ADC / (RESOLUCION_ADC - 1.0));
  referencia_V = constrain(voltajePotenciometro, REFERENCIA_BAJA, REFERENCIA_ALTA);

  /* 
  // ============================================
  // Onda cuadrada automática (NO USADA)
  // ============================================
  if (millis() - tiempoUltimoCambioReferencia >= (PERIODO_ONDA_CUADRADA_MS / 2)) {
    estadoOndaCuadrada = !estadoOndaCuadrada;
    referencia_V = estadoOndaCuadrada ? REFERENCIA_ALTA : REFERENCIA_BAJA;
    tiempoUltimoCambioReferencia = millis();
  }
  */

  // Leer la salida de la planta
  int lectura_ADC = analogRead(PIN_SALIDA_PLANTA);
  salida_planta_V = lectura_ADC * (VOLTAJE_MAX_ENTRADA_ADC / (RESOLUCION_ADC - 1.0));

  // Calcular el error
  error_k = referencia_V - salida_planta_V;

  // Calcular el control PI
  float u_p = Kp * error_k;
  e_integral += error_k;
  float u_i = Ki * Ts * e_integral;
  u_k = u_p + u_i;

  // Saturación de la señal de control
  u_k_saturado = constrain(u_k, 0.0, 5.0);

  // Convertir a PWM y aplicar
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
// 6. Setup y Loop del Arduino
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
}

void loop() {
  runner.execute();
}
