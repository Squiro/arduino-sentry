#include <Servo.h>
#include "constantes.h"
// VARIABLES GLOBALES

//---------------------------------------------------------------------
// ESTADOS Y EVENTOS
int current_state = 0;
int event = 0;
int sensor_state = 0;

//---------------------------------------------------------------------
// SERVO 

// Objeto del servo (perteneciente a la biblioteca servo.h)
Servo servo; 
// Almacena la posición del servo (seteamos el medio como la posición inicial)
int servoPosition = SERVO_MIDDLE_POSITION;
// Indica si el servo se mueve de izquierda a derecha, o derecha a izquierda
boolean leftToRight = true; 

//---------------------------------------------------------------------
// VARIABLES RELACIONADAS AL PIR 

// Tiempo en el que ocurre una transicion de HIGH a LOW
unsigned long tiempoTransicionLow;         
// Booleana que ayuda a bloquear el output del LOW 
boolean bloquearLow = true;
// Booleana para medir el tiempo de transicion a LOW solo cuando ocurra la transición
boolean medirTiempoLow;

//---------------------------------------------------------------------
// VARIABLES RELACIONADAS AL SENSOR DE ULTRA SONIDO
int rangoDeteccion = DEFALUT_RANGO_DETECCION_CM;


//---------------------------------------------------------------------
// FLAGS

// Almacena el valor arrojado por el PIR (true = movimiento detectado, false = no hay movimiento)
int movimientoDetectado = 0; 

// Para el parpadeo del LED
int estadoLED = 0;

//---------------------------------------------------------------------
// VARIABLES RELACIONADAS AL TIEMPO 

// Mide el tiempo actual
unsigned long tiempoActual = 0;
// Utilizado para la interrupción por softawre de genera_evento
unsigned long lastTimeOut = 0;
// Registra el tiempo en que se detecto movimiento
unsigned long tiempoParpadeo = 0; 
// Registra el tiempo en el que se comenzó la búsqueda
unsigned long tiempoComienzoBusqueda = 0; 

// Contadores de interrupciones y minutos
unsigned long contador_interrupt_min = 0;
unsigned long contador_interrupt_33ms = 0; 
unsigned long contador_min = 0; 

boolean minute_interrupt = true;
boolean ms_interrupt = true;

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// SETUP

void setup()
{
  // Para poder imprimir mensajes
  Serial.begin(SERIAL_SPEED);
  // Setup de los pines de los sensores/actuadores/componentes
  pinMode(PIR_PIN, INPUT);
  pinMode(SENSOR_LUZ_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RANGE_LED_PIN, OUTPUT);
  pinMode(ULTRASENSOR_ECHO_PIN, INPUT);
  pinMode(ULTRASENSOR_TRIGGER_PIN, OUTPUT); 
  servo.attach(SERVO_PIN);

  // Activamos la interrupción del botón
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), aumentarRangoDeteccion, RISING);

  // Activamos interrupciones de timer
  activateTimerInterrupt();

  // Seteamos el estado inicial  
  current_state = DAY_TIME_STATE;
  Serial.print("La torreta no se activara hasta que sea de noche.");
}

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// LOOP

void loop()
{    
  genera_evento();
  state_machine();
}

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// ACTIVATE TIMER INTERRUPTS

// Es importante notar lo siguiente: algunos timers tienen registros diferentes. Por ejemplo, el registro TCCR2B (del timer2) NO es el mismo al TCCR1B. 
// Esto es importantísimo, porque cambia la forma en que se setean tanto los prescalers como los modos de operación del timer (CTC, normal mode, etc). 
// Para ver bien cómo se conforma cada registro y qué valor utilizar para cada modo, leer el datasheet del Atmega328p.
// http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf (pág 130 en adelante)

void activateTimerInterrupt()
{
  // Control del contador del temporizador xA. 
  // TCCRxA y TCCRxB son registros diferentes, pero funcionan en el mismo temporizador, timerX. 
  // Configuran un comportamiento diferente y se ubican en registros separados, simplemente porque todos los bits no caben en un solo byte. 
  // Al setear los bits WGM10 y WGM11 en 0, evitamos que los output pins (OCxA y OCxB) estén activos. Esto es porque los timers de Arduino
  // se pueden usar para controlar la frencuencia de PWM de los pins... y es algo en lo que no nos queremos meter en este caso.
  TCCR2A = NORMAL_MODE;
  // Este registro se utiliza para setear el factor de escala (prescaler) del contador. El factor de escala nos permite aumentar
  // el tiempo en el que se dispara una interrupción. También se utiliza para setear el modo de funcionamiento del timer,
  // nosotros elegimos CTC. 
  TCCR2B = TIMER2_PRE_1024_CTC;
  // Se utilizara la interrupción timer2 por overflow.
  TIMSK2 = OVERFLOW_BIT;
  // Habilita las interrupciones globales.
  sei();
}

// El tiempo con el que se producirá esta interrupción está dictado por el prescaler (factor escala), la frencuencia del reloj de arduino,
// y el tamaño del buffer del registro contador. La fórmula general es: N = (16.000.000 * T)/fe - 1
// Como N = 255, si despejamos esa fórmula llegamos a que se producirá una interrupción por overflow cada 0.016384 segundos (16,384 ms)
ISR(TIMER2_OVF_vect)
{  
  // Cada TIEMPO_INTERRUPT_OVERFLOW_MS se produce una interrupción, entonces sumamos eso al contador de ms
  contador_interrupt_min++;
  contador_interrupt_33ms++;

  // Cada 3662 (redondeado) interrupciones pasa un minuto,
  // esto es porque... un minuto son 60000ms, y como una interrupción por overflow ocurre cada 16,384ms...
  // 60000/16,384 = 3662.1093
  if (contador_interrupt_min == CANT_INTERRUPCIONES_HASTA_MIN)
  {
    contador_interrupt_min = 0;
    contador_min++;
  }

  // Para leer el light sensor cada TIMEOUT_MINUTOS
  if (contador_min >= TIMEOUT_MINUTOS) 
  {
    contador_min = 0;
    minute_interrupt = true;
  }

  // Para leer el resto de sensores cada 33ms
  if (contador_interrupt_33ms == CANT_INTERRUPCIONES_HASTA_33ms)
  {
    contador_interrupt_33ms = 0;
    ms_interrupt = true;    
  }

  // Llevamos registro del tiempo actual
  tiempoActual += TIEMPO_INTERRUPT_OVERFLOW_MS;

  // Si el tiempo actual se acerca al máximo de unsigned long, seteamos todos los contadores en 0
  if (tiempoActual >= TIEMPO_MAX)
  {
    tiempoActual = 0;
    tiempoParpadeo = 0;
    tiempoTransicionLow = 0;
    tiempoComienzoBusqueda = 0;
  }
}

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// STATE MACHINE

void state_machine() 
{
  switch (current_state)
  {
    case DAY_TIME_STATE:
    {      
      switch (event)
      {

        case NIGHT_TIME_EVENT:
        {
          printStateMsg("DAYTIME STATE", "NIGHT_TIME EVENT", NIGHT_TIME_EVENT);
          current_state = BOOTING_UP_STATE;
        }
          break;

        case CONTINUE_EVENT: 
        {
          printStateMsg("DAYTIME STATE", "CONTINUE EVENT", CONTINUE_EVENT);
        }
          break; 
        
        default:
          //printStateMsg("DAYTIME STATE", "UNKNOWN EVENT", UNKNOWN_EVENT);                
          break;
      }
    }
      break;

    case BOOTING_UP_STATE: 
    {
      switch (event)
      {
        case CONTINUE_EVENT:
        {
          printStateMsg("BOOTING_UP STATE", "CONTINUE EVENT", CONTINUE_EVENT);
          moverTorretaHome();
          actualizarRangeLed();
          current_state = SLEEPING_STATE;
        }
          break;
        
        default:
          printStateMsg("BOOTING_UP STATE", "UNKNOWN EVENT", UNKNOWN_EVENT);
          break;
        }    
    }
      break;

    case SLEEPING_STATE: 
    {
      switch (event)
      {
        case MOVEMENT_DETECTED_EVENT: 
        {
          printStateMsg("SLEEPING STATE", "MOVEMENT DETECTED EVENT", MOVEMENT_DETECTED_EVENT);
          // Encendemos los LED
          digitalWrite(LED_PIN , HIGH);
          // Guardamos el tiempo en que se detectó movimiento          
          tiempoParpadeo = tiempoActual;    
          // Tomamos el tiempo actual para saber cuándo empezó la búsqueda del objetivo
          tiempoComienzoBusqueda = tiempoActual;
          // Cambiamos de estado al estado de búsqueda
          current_state = SEARCHING_STATE;
        }
          break;

        case DAY_TIME_EVENT: 
        {
          printStateMsg("SLEEPING STATE", "DAY_TIME EVENT", DAY_TIME_EVENT);
          current_state = DAY_TIME_STATE;
        }
          break;

        case CONTINUE_EVENT:
        {
          //printStateMsg("SLEEPING STATE", "CONTINUE EVENT", CONTINUE_EVENT); 
          current_state = SLEEPING_STATE;       
        }          
          break;
        
        default:
          //printStateMsg("SLEEPING STATE", "UNKNOWN EVENT", UNKNOWN_EVENT);
          break;
      }
    }
      break;

    case SEARCHING_STATE: 
    {
      switch (event)
      {

        case TARGET_IN_RANGE_EVENT:
        {
            printStateMsg("SEARCHING STATE", "TARGET IN RANGE EVENT", TARGET_IN_RANGE_EVENT);
            current_state = FIRING_STATE;
        }
          break;

        case TARGET_OUT_OF_RANGE_EVENT: 
        {
            printStateMsg("SEARCHING STATE", "TARGET OUT OF RANGE EVENT", TARGET_OUT_OF_RANGE_EVENT);
            buscarObjetivo(); 
            current_state = SEARCHING_STATE;
        }
          break;
        
        case SEARCH_TIMEOUT_EVENT: 
        {
            printStateMsg("SEARCHING STATE", "SERACH TIME OUT EVENT", SEARCH_TIMEOUT_EVENT);
            current_state = SLEEP_MODE_ACTIVATED_STATE;
        }
          break;

        case CONTINUE_EVENT: 
        {
            //printStateMsg("SEARCHING STATE", "CONTINUE EVENT", CONTINUE_EVENT);
            current_state = SEARCHING_STATE;
        }
          break;
        
        default:
          //printStateMsg("SEARCHING STATE", "UNKNOWN EVENT", UNKNOWN_EVENT);
          break;
      }
    }
      break;

    case FIRING_STATE: 
    {
      switch (event)
      {
        case TARGET_IN_RANGE_EVENT: 
        {
          printStateMsg("FIRING_STATE", "TARGET IN RANGE EVENT", TARGET_IN_RANGE_EVENT);
          current_state = FIRING_STATE;
        }
          break;

        case TARGET_OUT_OF_RANGE_EVENT: 
        {
          printStateMsg("FIRING_STATE", "TARGET OUT OF RANGE EVENT", TARGET_OUT_OF_RANGE_EVENT);
          current_state = SEARCHING_STATE;
        }
          break;

        case CONTINUE_EVENT: 
        {
          //printStateMsg("FIRING_STATE", "CONTINUE EVENT", CONTINUE_EVENT);
          current_state = FIRING_STATE;
        } 
          break;
      
        default:
          //printStateMsg("FIRING_STATE", "UNKNOWN EVENT", UNKNOWN_EVENT);
          break;
      }
    }
      break;
    
    case SLEEP_MODE_ACTIVATED_STATE: 
    {
      switch (event)
      {
        case CONTINUE_EVENT:
        {
          // printStateMsg("SLEEP_MODE_ACTIVATED STATE", "CONTINUE EVENT");
          moverTorretaHome();  
          // Apagamos el LED porque la torreta se va a dormir
          digitalWrite(LED_PIN, LOW);  
          current_state = SLEEPING_STATE;
        }
          break;
        
        default:
          break;
      }
    }
      break;
    
    default:
    {
      printStateMsg("UKNOWN STATE", "UNKNOWN EVENT", UNKNOWN_EVENT);
    }
      break;
  }

  // Consumimos el evento
  event = CONTINUE_EVENT;
}

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// GENERA_EVENTO

void genera_evento() 
{
  // Si se disparó la interrupción por haber llegado al TIMEOUT_MINUTOS, leemos el sensor de luz
  if (minute_interrupt)
  {
    readLightSensor();
    // Reseteamos el booleano de interrupción
    minute_interrupt = false;
  }
  // Si se disparó la interrupción por haber llegado a los 10ms, leemos el resto de sensores que nos interesan
  else if (ms_interrupt)
  {
    // La siguiente lógica provoca que un sensor diferente se lea en cada timeout. De esta forma, podemos
    // disparar varios eventos. En sí esto no es una máquina de estados de sensores, ya que no hay eventos dentro de la misma.
    // Simplemente es una lógica de código que nos permite chequear diferentes sensores y reaccionar apropiadamente acorde a las lecturas de los
    // mismos.
    switch (sensor_state)
    {
      
      case READ_PIR_SENSOR: 
      {
        readPIRSensor();
        sensor_state = READ_DISTANCE_SENSOR;
      }
        break;
      case READ_DISTANCE_SENSOR: 
      {
        readDistanceSensor();
        sensor_state = CHECK_SEARCH_TIMEOUT;
      }
        break;
      case CHECK_SEARCH_TIMEOUT: 
      {
        checkSearchTimeout();
        sensor_state = READ_PIR_SENSOR;
      }
        break;

      default: 
        sensor_state = READ_PIR_SENSOR;
        break;
    }

    // Reseteamos el booleano de interrupción
    ms_interrupt = false;
  }
  // Si no se activó ninguno de los dos... disparamos un continue event
  else 
  {
    event = CONTINUE_EVENT;
  }
}

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// SENSOR READING FUNCTIONS

void readLightSensor()
{
  if (esDeNoche())
  {
    event = NIGHT_TIME_EVENT;
  } 
  else 
  {
    event = DAY_TIME_EVENT;
  }
}

void readPIRSensor()
{
  if (detectarMovimiento())
  {
    event = MOVEMENT_DETECTED_EVENT;
  }
  else 
  {
    event = NO_MOVEMENT_EVENT;
  }
}

void readDistanceSensor()
{
  if (estaEnRango())
  {
    event = TARGET_IN_RANGE_EVENT;
  }
  else 
  {
    event = TARGET_OUT_OF_RANGE_EVENT;
  }
}

// Chequea si el tiempo de búsqueda expiró
void checkSearchTimeout()
{   
  // Si no hay movimiento y el tiempoActual supera el tiempo de busqueda
  if (!movimientoDetectado && tiempoActual > tiempoComienzoBusqueda + TIMEOUT_BUSQUEDA)
  {
    printMsg("SEARCHING_STATE - Cambiando a sleep mode");
    // Reseteamos porque vamos a cambiar de estado
    tiempoComienzoBusqueda = 0;  
    // Disparamos el evento de search time out
    event = SEARCH_TIMEOUT_EVENT;
  } 
}

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// BUSCAR OBJETIVO

// Se encarga de mover la torreta y parpadear las luces LED
void buscarObjetivo() 
{
  // Si el objetivo no está en rango, seguimos moviendo la torreta un rato para buscar
  printMsg("SEARCHING_STATE - Buscando");
  // Movemos la torreta
  moverTorreta();  
  
  // Parpadeamos los LEDS itermitentemente teniendo en cuenta el delay del parpadeo
  if (tiempoActual > tiempoParpadeo + DELAY_PARPADEO)
  {
    printMsg("SEARCHING_STATE - Parpadeando LEDs");

    estadoLED = !estadoLED;
    digitalWrite(LED_PIN, estadoLED);

    // Volvemos a setear el tiempo de parpadeo
    tiempoParpadeo = tiempoActual;  
  } 
}

//---------------------------------------------------------------------
// SENSOR RELATED FUNCTIONS

// Chequea el nivel de luz que mide el fotoresistor, si es menor que SENSIBILIDAD_LUZ, es de noche
boolean esDeNoche() 
{
  return (analogRead(SENSOR_LUZ_PIN) <= SENSIBLIDAD_LUZ);
}

// Lee el sensor PIR para detectar movimiento
boolean detectarMovimiento() 
{
  // Leemos el pin del PIR
  int pirReading = digitalRead(PIR_PIN);  

  // Si está en alto, es porque el sensor detectó movimiento
  if (pirReading == HIGH)
  {        
    if (bloquearLow)
    {  
      // Antes de volver a realizar otro output, nos fijamos de que el sensor PIR haya dejado de detectar movimiento
      // (es decir, que haya hecho una transición a LOW)
      bloquearLow = false;
      // Imprimimos un mensaje indicando en qué tiempo se detectó el movimiento            
      printSensorMsg("Movimiento detectado a los ", tiempoActual/1000, "segundos");   
    }
    medirTiempoLow = true; 
  }   

  // Si ocurrió una transición a bajo...
  if (pirReading == LOW)
  { 
    if (medirTiempoLow)
    {
      // Guardamos el tiempo en que se transicionó de HIGH a LOW
      tiempoTransicionLow = tiempoActual;
      // Nos aseguramos de solo tomarlo al principio de la transición (si no, lo estariamos tomando todo el rato)
      medirTiempoLow = false;       
    }

    // Si el sensor permaneció en LOW más tiempo que el delay, asumimos que no se va a haber más movimiento
    if (!bloquearLow && (tiempoActual > tiempoTransicionLow + PIR_MOTION_DELAY))
    {  
        // Reseteamos el valor de bloquearLow
        bloquearLow = true;
        // Imprimimos un mensaje indicando en qué tiempo se dejó de detectar movimiento
        printSensorMsg("Movimiento finalizado a los ", (tiempoActual - PIR_MOTION_DELAY)/1000, "segundos");                          
    }
  }

  // Retornamos la lecutra del PIR
  return pirReading;
}

// Detecta si lo que se movió está dentro del rango de detección de la torreta
boolean estaEnRango()
{ 
  long duracion;
  float distancia;
  boolean result = false;
  
  digitalWrite(ULTRASENSOR_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASENSOR_TRIGGER_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(ULTRASENSOR_TRIGGER_PIN, LOW);

  duracion = pulseIn(ULTRASENSOR_ECHO_PIN, HIGH);
  
  // Convertimos el tiempo en distancia
  distancia = microsegundosACentimetros(duracion);

  if (distancia <= rangoDeteccion)
  {
    result = true;
    printSensorMsg("Objeto detectado a la distancia de ", distancia, "cm");
  }  

  return result;
}

// Convierte microsegundos a centimetros, utilizado por el sensor de ultrasonido para medir la distancia
float microsegundosACentimetros(float microsegundos)
{
  // El tiempo devuelto por el sensor de ultrasonido toma todo el viaje que realiza la onda de sonido 
  // (tanto de ida, como la vuelta luego del rebote). Para obtener la distancia del objeto detectado, dividimos por 2
  return microsegundos / MICROSEGUNDOS_POR_CM / 2;
}

// Handler de la interrupción causada por el botón
void aumentarRangoDeteccion()
{
  rangoDeteccion += INCREMENTO_RANGO;

  if (rangoDeteccion > RANGO_DETECCION_MAX)
  {
    rangoDeteccion = RANGO_DETECCION_MIN;
  }

  actualizarRangeLed();
  printSensorMsg("Rango de deteccion modificado, nuevo valor: ", rangoDeteccion, "cm");
}

// Actualiza la intensidad del range LED
void actualizarRangeLed()
{
  analogWrite(RANGE_LED_PIN, map(rangoDeteccion, RANGO_DETECCION_MIN, RANGO_DETECCION_MAX, MIN_LED_PWM, MAX_LED_PWM));
}

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// SERVO FUNCTIONS

// Mueve la torreta
void moverTorreta()
{
  // Si el barrido es de izquierda a derecha...
  if (leftToRight)
  {
    // Comprobamos que el servo no esté en la posición máxima
    if (servoPosition < SERVO_MAX_POSITION)
    {
      // Incrementamos el ángulo de rotación/posición del servo
      servoPosition += SERVO_DELTA;
    }
    else
    {
      //Si está en la posición máxima, cambiamos el sentido de barrido
      leftToRight = false;
      // Decrementamos el ángulo de rotación/posición del servo
      servoPosition -= SERVO_DELTA;
    }
  }
  // En caso contrario, el barrido es de derecha a izquierda
  else 
  {
    if (servoPosition > SERVO_MIN_POSITION)
    {
      servoPosition -= SERVO_DELTA;
    }
    else
    {
      leftToRight = true;
      servoPosition += SERVO_DELTA;
    }
  } 
  // Movemos el servo
  servo.write(servoPosition);
}

// Mueve la torreta a la posición inicial (middle position)
void moverTorretaHome()
{  
  Serial.print("Moviendo el servo a la posicion inicial: ");
  Serial.print(SERVO_MIDDLE_POSITION);  
  Serial.println(" ");
  
  // Reiniciamos la posición del servo 
  servo.write(SERVO_MIDDLE_POSITION);
}

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// PRINT FUNCTIONS

void printMsg(String msg)
{
  if (PRINT_USELESS_STATES)
  {
    Serial.println(msg);
    Serial.println(" ");
  }

}

void printStateMsg(String state, String eventStr, int eventID) 
{
  if (!PRINT_USELESS_STATES && (eventID == CONTINUE_EVENT || eventID == UNKNOWN_EVENT))
  {
    return;
  }

  Serial.println("---------- STATE MESSAGE ----------");
  Serial.println(state);
  Serial.println(eventStr);
  Serial.println("----------------------------");
}

void printSensorMsg(String msg, int cant, String unit) 
{
  Serial.println("---------- SENSOR MESSAGE ----------");
  Serial.print(msg);
  Serial.print(cant);
  Serial.print(" " + unit);
  Serial.println(" ");
  Serial.println("----------------------------");
}

//---------------------------------------------------------------------