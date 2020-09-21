#include <Servo.h>
#include "constantes.h"

// VARIABLES GLOBALES

//---------------------------------------------------------------------
// ESTADOS Y EVENTOS
int current_state = 0;
int event = 0;
int sensor_state;

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
unsigned long comienzoBusqueda = 0; 

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// SETUP

void setup()
{
  // Para poder imprimir mensajes
  Serial.begin(SERIAL_SPEED);
  // Setup de los pines de los sensores/actuadores/componentes
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ULTRASENSOR_ECHO_PIN, INPUT);
  pinMode(ULTRASENSOR_TRIGGER_PIN, OUTPUT); 
  pinMode(SENSOR_LUZ_PIN, INPUT);
  servo.attach(SERVO_PIN);     

  // Seteamos el estado inicial  
  current_state = DAY_TIME_STATE;
  Serial.print("La torreta no se activara hasta que sea de noche.");
}

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// LOOP

void loop()
{     
  tiempoActual = millis();
  state_machine();
}

//---------------------------------------------------------------------

//---------------------------------------------------------------------
// STATE MACHINE

void state_machine() 
{
  genera_evento();

  switch (current_state)
  {
    case DAY_TIME_STATE:
    {      
      switch (event)
      {

        case NIGHT_TIME_EVENT:
        {
          printStateMsg("DAYTIME STATE", "NIGHT_TIME EVENT");
          current_state = BOOTING_UP_STATE;
        }
          break;

        case CONTINUE_EVENT: 
        {
          // printStateMsg("DAYTIME STATE", "CONTINUE EVENT");
        }
          break; 
        
        default:                
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
          // printStateMsg("BOOTING_UP STATE", "CONTINUE EVENT");
          moverTorretaHome();
          current_state = SLEEPING_STATE;
        }
          break;
        
        default:
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
          printStateMsg("SLEEPING STATE", "MOTION DETECTED EVENT");
          // Encendemos los LED
          digitalWrite(LED_PIN , HIGH);
          // Guardamos el tiempo en que se detectó movimiento          
          tiempoParpadeo = tiempoActual;    
          // Tomamos el tiempo actual para saber cuándo empezó la búsqueda del objetivo
          comienzoBusqueda = tiempoActual;
          // Cambiamos de estado al estado de búsqueda
          current_state = SEARCHING_STATE;
        }
          break;

        case DAY_TIME_EVENT: 
        {
          printStateMsg("SLEEPING STATE", "DAY_TIME EVENT");
          current_state = DAY_TIME_STATE;
        }
          break;

        case CONTINUE_EVENT:
        {
          // printStateMsg("SLEEPING STATE", "CONTINUE EVENT");        
        }          
          break;
        
        default:
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
            printStateMsg("SEARCHING STATE", "TARGET IN RANGE EVENT");
            current_state = FIRING_STATE;
        }
          break;

        case TARGET_OUT_OF_RANGE_EVENT: 
        {
            printStateMsg("SEARCHING STATE", "TARGET OUT OF RANGE EVENT");
            buscarObjetivo(); 
            current_state = SEARCHING_STATE;
        }
          break;
        
        case SEARCH_TIMEOUT_EVENT: 
        {
            printStateMsg("SEARCHING STATE", "SERACH TIME OUT EVENT");
            current_state = SLEEP_MODE_ACTIVATED_STATE;
        }
          break;

        case CONTINUE_EVENT: 
        {
            // printStateMsg("SEARCHING STATE", "CONTINUE EVENT");
            current_state = SEARCHING_STATE;
        }
          break;
        
        default:
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
          printStateMsg("FIRING_STATE", "TARGET IN RANGE EVENT");
          current_state = FIRING_STATE;
        }
          break;

        case TARGET_OUT_OF_RANGE_EVENT: 
        {
          printStateMsg("FIRING_STATE", "TARGET OUT OF RANGE EVENT");
          current_state = SEARCHING_STATE;
        }
          break;    
      
        default:
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
      printStateMsg("UKNOWN STATE", "UNKNOWN EVENT");
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
  // Generamos un nuevo evento. El tiempo de generación de eventos está regulado por la constante NEXT_EVENT_TIMEOUT.
  if (tiempoActual > lastTimeOut + NEXT_EVENT_TIMEOUT) 
  {
    // Seteamos al lastTimeOut en el tiempo actual
    lastTimeOut = tiempoActual; 
    
    // La siguiente lógica provoca que un sensor diferente se lea en cada ciclo. De esta forma, podemos
    // disparar varios eventos. En sí esto no es una máquina de estados de sensores, ya que no hay eventos dentro de la misma.
    // Simplemente es una lógica de código que nos permite chequear diferentes sensores y reaccionar apropiadamente acorde a las lecturas de los
    // mismos.
    switch (sensor_state)
    {
      case READ_LIGHT_SENSOR: 
      {
        readLightSensor();
        sensor_state = READ_PIR_SENSOR;
      }
        break;
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
      case CHECK_SEARCH_TIMEOUT: 
      {
        checkSearchTimeout();
        sensor_state = READ_LIGHT_SENSOR;
      }
        break;

      default: 
        sensor_state = READ_LIGHT_SENSOR;
        break;
    }
  } 
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
  if (!movimientoDetectado && tiempoActual > comienzoBusqueda + TIMEOUT_BUSQUEDA)
  {
    printState("SEARCHING_STATE - Cambiando a sleep mode");
    // Reseteamos porque cambiamos de estado
    comienzoBusqueda = 0;  
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
  printState("SEARCHING_STATE - Buscando");
  // Movemos la torreta
  moverTorreta();  
  if (tiempoActual > tiempoParpadeo + DELAY_PARPADEO)
  {
    printState("SEARCHING_STATE - Parpadeando LEDs");

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
      Serial.println("---");
      Serial.print("Movimiento detectado a los ");
      Serial.print(tiempoActual/1000);
      Serial.print(" segundos"); 
      Serial.println(" ");
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
        Serial.print("Movimiento finalizado a los ");
        Serial.print((tiempoActual - PIR_MOTION_DELAY)/1000);
        Serial.print(" segundos");
        Serial.println(" ");
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

  if (distancia <= RANGO_DETECCION_CM)
  {
    result = true;

    Serial.print("Objeto detectado a distancia de ");
    Serial.print(distancia);
    Serial.print(" cm");
    Serial.println(" ");
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

void printState(String stateMessage) 
{
  if (PRINT_STATES)
    Serial.println(stateMessage);
}

void printStateMsg(String state, String event) 
{
  Serial.println("----------------------------");
  Serial.println(state);
  Serial.println(event);
  Serial.println("----------------------------");
}

//---------------------------------------------------------------------