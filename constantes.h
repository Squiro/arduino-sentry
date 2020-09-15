// ESTADOS
// #define SLEEPING_STATE               1
// #define SEARCHING_STATE              2
// #define TARGET_AQUIRED_STATE         3
// #define FIRING_STATE                 4
// #define TARGET_LOST_STATE            5
// #define SLEEP_MODE_ACTIVATED         6

enum states {
 SLEEPING_STATE,
 SEARCHING_STATE,
 TARGET_AQUIRED_STATE,
 FIRING_STATE, 
 TARGET_LOST_STATE,
 SLEEP_MODE_ACTIVATED

} current_state;

// ULTRASONIDO

#define ULTRASENSOR_ECHO_PIN    8    // Pin para recibir el pulso de eco
#define ULTRASENSOR_TRIGGER_PIN 9    // Pin para enviar el pulso de disparo

// ULTRASONIDO Y DISTANCIA
#define RANGO_DETECCION_CM   200    // Distancia a la que la torreta va a detectar un objetivo
#define MICROSEGUNDOS_POR_CM 29     // Constante, basada en la velocidad del sonido (340m/s), indica cuantos microsegundos hay en 1cm

// SERVO
#define SERVO_MIN_POSITION      0
#define SERVO_MIDDLE_POSITION   90
#define SERVO_MAX_POSITION      180
#define SERVO_DELTA             10 // Es el incremento que le aplicamos al servo cada vez que lo movemos
#define SERVO_PIN               6  // Pin PWN del servo

// PIR
#define PIR_PIN            2
#define TIEMPO_CALIBRACION 0.1  // Tiempo (en segundos) de calibración para el PIR (necesario para un sensor PIR de verdad)
#define PIR_MOTION_DELAY   2500 // La cantidad de (ms) que el sensor PIR tiene que estar en LOW para que asumamos que no hay mas movimiento

// TIEMPO / PAUSAS / DELAY
#define DELAY_PARPADEO     25   // Delay (en ms) del parpadeo de los LEDs después de detectar movimiento / fase de búsqueda
#define TIEMPO_DE_BUSQUEDA 2500 // Duración de la búsqueda

// LED
#define LED_PIN         12

// FOTORESISTOR
#define SENSOR_LUZ_PIN  14
#define SENSIBLIDAD_LUZ 400 // Limite por el cual el programa empieza a funcionar (detecta si es de día o de noche)

// DEBUG
#define PRINT_STATES 0