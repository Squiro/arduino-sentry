// ESTADOS
#define DAY_TIME_STATE               100
#define BOOTING_UP_STATE             150
#define SLEEPING_STATE               200
#define SEARCHING_STATE              300
#define FIRING_STATE                 400
#define SLEEP_MODE_ACTIVATED_STATE   500

// EVENTOS
#define CONTINUE_EVENT            100
#define NIGHT_TIME_EVENT          200
#define DAY_TIME_EVENT            250
#define MOVEMENT_DETECTED_EVENT   300
#define NO_MOVEMENT_EVENT         350
#define TARGET_IN_RANGE_EVENT     400
#define TARGET_OUT_OF_RANGE_EVENT 450
#define SEARCH_TIMEOUT_EVENT      500

// "ESTADOS" DE LOS SENSORES - Utilizado para la lógica de lectura de sensores
#define READ_LIGHT_SENSOR    100
#define READ_PIR_SENSOR      200
#define READ_DISTANCE_SENSOR 300
#define CHECK_SEARCH_TIMEOUT 400

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
#define TIMEOUT_BUSQUEDA   2500 // Duración de la búsqueda
#define NEXT_EVENT_TIMEOUT 25

// LED
#define LED_PIN 12

// FOTORESISTOR
#define SENSOR_LUZ_PIN  14
#define SENSIBLIDAD_LUZ 400 // Limite por el cual el programa empieza a funcionar (detecta si es de día o de noche)

// DEBUG
#define PRINT_STATES 0

// MISC
#define SERIAL_SPEED 9600