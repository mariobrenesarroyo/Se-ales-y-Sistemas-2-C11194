// --- INCLUDES DE LIBRERIAS ---
#include <TaskScheduler.h>

// --- DECLARACIONES DE FUNCIONES (Forward Declarations) ---
// Es necesario declarar estas funciones antes de usarlas en la configuracion de Tasks
void funcionControl();
void funcionSerial();

// --- DEFINICION DE PINES ---
// Asegurate que estos pines correspondan a tus conexiones fisicas
const int PIN_SALIDA_PLANTA = A0;   // Pin analógico donde se conectará la salida de tu circuito (Vo)
const int PIN_ENTRADA_PLANTA = 9;  // Pin PWM donde se conectará la entrada de tu circuito (Vs)
// Nota: Puedes usar el pin A1 para un potenciómetro si lo necesitas como referencia manual,
// pero el código ya incluye la referencia de onda cuadrada automática.
const int PIN_POTENCIOMETRO = A1; // Pin para potencial, aunque no usado en la referencia de onda cuadrada


// --- PARAMETROS DEL CONTROLADOR ---
const float TS = 0.1; // Periodo de muestreo en segundos (0.1s)

// Coeficientes de tu controlador C(z) = (1.785z - 1.715)/(z - 1)
const float K_C = 1.785;
const float K_C_Z_1 = 1.715;

// --- VARIABLES DEL CONTROLADOR ---
// Variables para almacenar los valores actuales y anteriores para la ecuación en diferencias
float error_k = 0;   // Error actual e[k]
float error_k_menos_1 = 0; // Error anterior e[k-1]

float u_k = 0;       // Señal de control actual u[k] (en Volts)
float u_k_menos_1 = 0; // Señal de control anterior u[k-1] (en Volts)

float referencia_V = 0; // Valor de la referencia en Volts (ej. 1V a 3.5V)
float salida_planta_V = 0; // Valor de la salida de la planta en Volts

// --- VARIABLES PARA LA REFERENCIA CUADRADA AUTOMATICA ---
const unsigned long PERIODO_ONDA_CUADRADA_MS = 6000; // 6 segundos en milisegundos
const float REFERENCIA_ALTA = 3.5; // Volts
const float REFERENCIA_BAJA = 1.0; // Volts
bool estadoOndaCuadrada = false; // false para baja (1V), true para alta (3.5V)

// Contador de ejecuciones de la tarea de control para la onda cuadrada
int control_execution_counter = 0;
const int PASOS_POR_MITAD_PERIODO = (PERIODO_ONDA_CUADRADA_MS / 2) / (TS * 1000);


// --- CONFIGURACION DE TASKSCHEDULER ---
Scheduler runner;

// Tarea para la lógica de control principal, se ejecuta cada TS (0.1s)
Task controlTask(TS * 1000, TASK_FOREVER, &funcionControl);

// Tarea para enviar datos por serial, se ejecuta cada 500ms (2 veces por segundo)
const unsigned long INTERVALO_SERIAL_MS = 500;
Task serialTask(INTERVALO_SERIAL_MS, TASK_FOREVER, &funcionSerial);


void setup() {
  // Configuración de pines de E/S
  pinMode(PIN_ENTRADA_PLANTA, OUTPUT); // Pin PWM para la señal de control (Vs)
  pinMode(PIN_SALIDA_PLANTA, INPUT);   // Pin analógico para la salida de la planta (Vo)
  pinMode(PIN_POTENCIOMETRO, INPUT);   // Pin analógico para el potenciómetro (si se usa)

  // Inicializar comunicación serial a una velocidad alta para mejor monitoreo
  Serial.begin(115200);
  Serial.println("Arduino listo para control!");

  // Inicializar variables anteriores del controlador
  // Es importante que esten en 0 al inicio para evitar valores indeseados
  error_k_menos_1 = 0;
  u_k_menos_1 = 0;

  // Establecer el estado inicial de la onda cuadrada a REFERENCIA_BAJA
  estadoOndaCuadrada = false; // Inicia en 1V
  referencia_V = REFERENCIA_BAJA;

  // Añadir tareas al planificador y habilitarlas
  runner.addTask(controlTask);
  runner.addTask(serialTask);
  controlTask.enable();
  serialTask.enable();
}

void loop() {
  // Ejecutar las tareas programadas por TaskScheduler
  runner.execute();
}

// --- FUNCIONES DE LAS TAREAS ---

// Función que contiene la lógica principal del controlador
void funcionControl() {
  // 1. Manejo de la Referencia (Onda Cuadrada Automática)
  control_execution_counter++;
  if (control_execution_counter >= PASOS_POR_MITAD_PERIODO) {
    estadoOndaCuadrada = !estadoOndaCuadrada; // Cambiar estado
    control_execution_counter = 0; // Reiniciar contador
  }
  referencia_V = estadoOndaCuadrada ? REFERENCIA_ALTA : REFERENCIA_BAJA;


  // 2. Leer Salida de la Planta (Vo)
  int lectura_planta_ADC = analogRead(PIN_SALIDA_PLANTA);
  // Escalar la lectura ADC (0-1023) a Volts (0-5V)
  salida_planta_V = lectura_planta_ADC * (5.0 / 1023.0); // IMPORTANTE: Asume Arduino trabajando a 5V!

  // 3. Calcular Error e[k]
  error_k = referencia_V - salida_planta_V;

  // 4. Implementar Ecuación en Diferencias del Controlador C(z)
  // u[k] = u[k-1] + 1.785 * e[k] - 1.715 * e[k-1]
  u_k = u_k_menos_1 + (K_C * error_k) - (K_C_Z_1 * error_k_menos_1);

  // 5. Saturar la Señal de Control (0V a 5V)
  u_k = constrain(u_k, 0.0, 5.0); // Asegura que u_k esté entre 0 y 5 Volts

  // 6. Mapear u[k] (Volts) a un valor PWM (0-255) y Enviar a la Planta (Vs)
  // Multiplicamos por 100 para trabajar con más precisión con 'map' que no maneja floats.
  // Luego map(0-500, 0, 255) convierte 0.0V-5.0V a 0-255.
  int u_k_PWM = map(static_cast<int>(u_k * 100), 0, 500, 0, 255);
  analogWrite(PIN_ENTRADA_PLANTA, u_k_PWM);

  // 7. Actualizar Variables Anteriores para la proxima iteracion
  u_k_menos_1 = u_k;
  error_k_menos_1 = error_k;
}

// Función para enviar datos por el puerto serial
void funcionSerial() {
  // Enviar los valores en Volts
  Serial.print("Ref: ");
  Serial.print(referencia_V, 3); // Imprimir con 3 decimales
  Serial.print("V, Salida: ");
  Serial.print(salida_planta_V, 3);
  Serial.print("V, Control: ");
  Serial.print(u_k, 3);
  Serial.println("V");
}