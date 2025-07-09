#include <TaskScheduler.h>

// --------------------------------------------------------------------------------------
// PROTOTIPOS DE FUNCIONES
// Deben declararse aquí para que el compilador las conozca antes de ser usadas en 'Task'
// --------------------------------------------------------------------------------------
void funcionControl();
void funcionSerial();

// --------------------------------------------------------------------------------------
// 0. Definiciones de Pines
// --------------------------------------------------------------------------------------
const int PIN_ENTRADA_PLANTA = 9;   // Pin PWM (Digital) para la entrada de la planta (Vs)
const int PIN_SALIDA_PLANTA = A0;  // Pin Analógico para la salida de la planta (Vo)
// const int PIN_POTENCIOMETRO = A1; // Pin Analógico para el potenciómetro (si se usa como referencia)

// --------------------------------------------------------------------------------------
// 1. Parámetros de la Referencia y del Sistema (Ajustar según necesidad)
// --------------------------------------------------------------------------------------
const float REFERENCIA_BAJA = 1.0;  // Voltaje de referencia bajo para la onda cuadrada (V)
const float REFERENCIA_ALTA = 3.5;  // Voltaje de referencia alto para la onda cuadrada (V)
const int PERIODO_ONDA_CUADRADA_MS = 6000; // Periodo total de la onda cuadrada en ms (6 segundos)

const float VOLTAJE_MAX_ENTRADA_ADC = 5.0; // Voltaje máximo que el ADC puede leer (5V para Arduino UNO)
const int RESOLUCION_ADC = 1024;           // Resolución del ADC (2^10 = 1024 para 0-1023)
const int RESOLUCION_PWM = 256;            // Resolución del PWM (2^8 = 256 para 0-255)

// --------------------------------------------------------------------------------------
// 2. Parámetros del Controlador PI (Estos deben venir de tu diseño)
// --------------------------------------------------------------------------------------
const float Kp = 1.785;   // Ganancia Proporcional (Ejemplo, usar tus valores de diseño)
const float Ki = 0.7;    // Ganancia Integral (Ejemplo, usar tus valores de diseño)
const float Ts = 0.1;     // Período de muestreo del controlador en segundos (100 ms)

// --------------------------------------------------------------------------------------
// 3. Variables Globales para el Control
// --------------------------------------------------------------------------------------
float referencia_V = REFERENCIA_BAJA; // Valor de referencia actual en Voltios
float salida_planta_V = 0.0;          // Salida medida de la planta en Voltios
float error_k = 0.0;                  // Error actual
float u_k = 0.0;                      // Señal de control calculada en Voltios (antes de saturación)
float u_k_saturado = 0.0;             // Señal de control saturada (0-5V)
float e_integral = 0.0;               // Suma integral del error

// Variables para la lógica de la onda cuadrada
bool estadoOndaCuadrada = false; // true = REFERENCIA_ALTA, false = REFERENCIA_BAJA
unsigned long tiempoUltimoCambioReferencia = 0;

// --------------------------------------------------------------------------------------
// 4. Variables para TaskScheduler
// --------------------------------------------------------------------------------------
Scheduler runner;

// Tareas:
// - Control: Ejecuta el algoritmo de control en cada período de muestreo (Ts)
// - Serial: Envía datos al Monitor/Plotter Serial a una frecuencia menor
Task taskControl(Ts * 1000, TASK_FOREVER, &funcionControl); // Ts en ms
Task taskSerial(1, TASK_FOREVER, &funcionSerial); // Cada 500 ms (2 veces por segundo)

// --------------------------------------------------------------------------------------
// 5. Funciones
// --------------------------------------------------------------------------------------

void funcionControl() {
  // 1. Manejo de la Referencia (Onda Cuadrada Automática)
  // Actualizar la referencia cada mitad de período de la onda cuadrada
  if (millis() - tiempoUltimoCambioReferencia >= (PERIODO_ONDA_CUADRADA_MS / 2)) {
    estadoOndaCuadrada = !estadoOndaCuadrada; // Cambiar estado
    referencia_V = estadoOndaCuadrada ? REFERENCIA_ALTA : REFERENCIA_BAJA;
    tiempoUltimoCambioReferencia = millis(); // Reiniciar el contador de tiempo
  }
  
  /*
  // Si deseas usar un potenciómetro como referencia (descomenta y comenta lo anterior):
  // int lecturaPotenciometro = analogRead(PIN_POTENCIOMETRO);
  // float voltajePotenciometro = lecturaPotenciometro * (VOLTAJE_MAX_ENTRADA_ADC / (RESOLUCION_ADC -1.0));
  // referencia_V = constrain(voltajePotenciometro, REFERENCIA_BAJA, REFERENCIA_ALTA); // Asegura que esté entre 1V y 3.5V
  */

  // 2. Leer la Salida de la Planta
  int lectura_ADC = analogRead(PIN_SALIDA_PLANTA); // v_o se conecta a una entrada analógica
  // Convertir la lectura ADC (0-1023) a Voltios (0-5V)
  salida_planta_V = lectura_ADC * (VOLTAJE_MAX_ENTRADA_ADC / (RESOLUCION_ADC - 1.0));

  // 3. Calcular el Error
  error_k = referencia_V - salida_planta_V;

  // 4. Calcular el Control (Controlador PI Discreto - Forma de Posición)
  // Termino Proporcional
  float u_p = Kp * error_k;

  // Termino Integral
  // Suma los errores para la integración
  e_integral += error_k; 
  float u_i = Ki * Ts * e_integral; // Multiplicar por Ts para la aproximación rectangular (suma) de la integral

  u_k = u_p + u_i; // Señal de control antes de saturación

  // 5. Saturar la Señal de Control (0V a 5V)
  u_k_saturado = constrain(u_k, 0.0, 5.0);

  // 6. Escribir la Señal de Control en el PWM de la Planta
  // Convertir el voltaje saturado (0-5V) a un valor PWM (0-255)
  int valor_PWM = u_k_saturado * ((RESOLUCION_PWM - 1.0) / VOLTAJE_MAX_ENTRADA_ADC);
  analogWrite(PIN_ENTRADA_PLANTA, valor_PWM);
}

void funcionSerial() {
  // Imprimir los valores de Referencia, Salida y Control para el Serial Plotter
  // Separados por tabulaciones para que el plotter los interprete como columnas diferentes
  Serial.print(referencia_V, 3); // 3 decimales
  Serial.print("\t");            // Tabulación
  Serial.print(salida_planta_V, 3); // 3 decimales
  Serial.print("\t");            // Tabulación
  Serial.println(u_k_saturado, 3); // 3 decimales y salto de línea al final
}

// --------------------------------------------------------------------------------------
// 6. Setup y Loop del Arduino
// --------------------------------------------------------------------------------------

void setup() {
  // Inicializar comunicación Serial
  Serial.begin(115200);

  // Configurar pines
  pinMode(PIN_ENTRADA_PLANTA, OUTPUT); // Pin PWM como salida
  pinMode(PIN_SALIDA_PLANTA, INPUT);   // Pin Analógico como entrada

  // Añadir tareas al planificador
  runner.addTask(taskControl);
  runner.addTask(taskSerial);

  // Habilitar tareas
  taskControl.enable();
  taskSerial.enable();

  tiempoUltimoCambioReferencia = millis(); // Inicializar el contador de tiempo para la referencia
}

void loop() {
  // Ejecutar las tareas planificadas
  runner.execute(); // Usamos execute() para compatibilidad con tu librería TaskScheduler
}