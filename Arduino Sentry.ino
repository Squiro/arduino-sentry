#include <Servo.h>
#include "constantes.h"

/** IMPLEMENTAR MAQUINA DE ESTADOS CON EVENTOS */

// VARIABLES GLOBALES

/* SERVO */

// Objeto del servo (perteneciente a la biblioteca servo.h)
Servo servo; 
// Almacena la posición del servo (seteamos el medio como la posición inicial)
int servoPosition = SERVO_MIDDLE_POSITION;
// Indica si el servo se mueve de izquierda a derecha, o derecha a izquierda
boolean leftToRight = true; 
 
/* VARIABLES RELACIONADAS AL PIR */

// Tiempo en el que ocurre una transicion de HIGH a LOW
unsigned long tiempoTransicionLow;         
// Booleana que ayuda a bloquear el output del LOW 
boolean bloquearLow = true;
// Booleana para medir el tiempo de transicion a LOW solo cuando ocurra la transición
boolean medirTiempoLow;

/* FLAGS */

// Almacena el valor arrojado por el PIR (true = movimiento detectado, false = no hay movimiento)
int movimientoDetectado = 0; 
// Almacena el valor arrojado por el sensor de ultrasonido, indica si el objetivo está dentro del rango de detección
boolean objetivoEnRango = false; 

/* OTRAS VARIABLES */

// Almacena el estado actual
int estadoActual = SLEEPING_STATE; 
// Para el parpadeo del LED
int estadoLED = 0;
// Variable que almacena el string del estado actual para ser mostrado por serial
String strState;

/* VARIABLES RELACIONADAS AL TIEMPO */

// Mide el tiempo actual
unsigned long tiempoActual = 0;
// Registra el tiempo en que se detecto movimiento
unsigned long tiempoDeteccion = 0; 
// Registra el tiempo en el que se comenzó la búsqueda
unsigned long comienzoBusqueda = 0; 

void setup()
{
  // Para poder imprimir mensajes
  Serial.begin(9600);
  // Setup de los pines de los sensores/actuadores/componentes
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ULTRASENSOR_ECHO_PIN, INPUT);
  pinMode(ULTRASENSOR_TRIGGER_PIN, OUTPUT); 
  pinMode(SENSOR_LUZ_PIN, INPUT);
  servo.attach(SERVO_PIN);     

  // Movemos la torreta a la posición inicial
  moverTorretaHome();    
  // Seteamos el estado inicial  
  current_state = SLEEPING_STATE;
  Serial.print("La torreta no se activará hasta que sea de noche.");
}

void loop()
{     
  tiempoActual = millis();

  if (esDeNoche()) 
  {
    // Le damos un tiempo al sensor PIR para que se calibre 
    if ((tiempoActual/1000) > TIEMPO_CALIBRACION) 
    {
      detectarMovimiento();
      checkStates();    
    } 
    else 
    {
      Serial.println('.');
    }
  }
}

// Chequea el nivel de luz que mide el fotoresistor, si es menor que SENSIBILIDAD_LUZ, es de noche
boolean esDeNoche() 
{
  return (analogRead(SENSOR_LUZ_PIN) <= SENSIBLIDAD_LUZ);
}

void detectarMovimiento() 
{
  // Leemos el pin del PIR
  movimientoDetectado = digitalRead(PIR_PIN);  

  // Si está en alto, es porque el sensor detectó movimiento
  if (movimientoDetectado == HIGH)
  {        
    if (bloquearLow)
    {  
      // Antes de volver a realizar otro output, nos fijamos de que el sensor PIR haya dejado de detectar movimiento
      // (es decir, que haya hecho una transición a LOW)
      bloquearLow = false;            
      Serial.println("---");
      Serial.print("Movimiento detectado a los ");
      Serial.print(tiempoActual/1000);
      Serial.print(" segundos"); 
      Serial.println(" ");
    }        
    medirTiempoLow = true; 
  }   

  if (movimientoDetectado == LOW)
  {     
    if (medirTiempoLow)
    {
      // Guardamos el tiempo en que se transicionó de HIGH a LOW
      tiempoTransicionLow = millis();
      // Nos aseguramos de solo tomarlo al principio de la transición (si no, lo estariamos tomando todo el rato)
      medirTiempoLow = false;       
    }

    // Si el sensor permaneció en LOW más tiempo que el delay, asumimos que no se va a haber más movimiento
    if (!bloquearLow && (tiempoActual > tiempoTransicionLow + PIR_MOTION_DELAY))
    {  
        bloquearLow = true;                        
        Serial.print("Movimiento finalizado a los ");
        Serial.print((tiempoActual - PIR_MOTION_DELAY)/1000);
        Serial.print(" segundos");
        Serial.println(" ");
    }
  }
}

void checkStates() 
{
  // Chequeamos el sensor de ultra sonido para ver si hay un objeto (o alguien) cerca
  if (estadoActual != SLEEPING_STATE)
  {
    objetivoEnRango = estaEnRango();   
  }

  switch(current_state) 
  {
    case SLEEPING_STATE: 
    {   
      sleepingState();
      break;
    }

    case SEARCHING_STATE: 
    {
      searchingState();
      break;
    }

    case TARGET_AQUIRED_STATE: 
    {
      printState("TARGET_AQUIRED_STATE - Objetivo fijado");
      estadoActual = FIRING_STATE;
      break;
    }     
    
    case FIRING_STATE:
    {    
      firingState();
      break;
    }
 
    case TARGET_LOST_STATE:
    {    
      printState("TARGET_LOST_STATE");         
      estadoActual = SEARCHING_STATE;      
      break;
    }

    case SLEEP_MODE_ACTIVATED:
    {
      sleepingMode();      
      break;
    }
      
    default:
      Serial.println("DEFAULT");
  }
}

void sleepingState() 
{
  printState("SLEEPING STATE");

  if (movimientoDetectado)
  {
    printState("SLEEPING_STATE - Movimiento detectado");
    // Encendemos el LED
    digitalWrite(LED_PIN , HIGH);
    // Guardamos el tiempo en que se detectó movimiento
    tiempoDeteccion = millis();    
    // Cambiamos de estado al estado de búsqueda
    estadoActual = SEARCHING_STATE;
  }
}

void searchingState() 
{
  // Tomamos el tiempo actual para saber cuándo empezó la búsqueda del objetivo
  if (comienzoBusqueda == 0)
    comienzoBusqueda = millis();
  
  printState("SEARCHING_STATE");
    
  if (objetivoEnRango)
  {
    printState("SEARCHING_STATE - Objetivo detectado");
    estadoActual = TARGET_AQUIRED_STATE;
    // Reseteamos porque cambiamos de estado
    comienzoBusqueda = 0; 
  }
  // Si el objetivo no está en rango, seguimos moviendo la torreta un rato para buscar
  else 
  {
    printState("SEARCHING_STATE - Buscando");

    // Figure out how much time has passed
    //tiempoActual = millis();

    moverTorreta();
    
    if (tiempoActual > tiempoDeteccion + DELAY_PARPADEO)
    {
      // Si no hay movimiento y el tiempoActual supera el tiempo de busqueda
      if (!movimientoDetectado && tiempoActual > comienzoBusqueda + TIEMPO_DE_BUSQUEDA)
      {
        printState("SEARCHING_STATE - Cambiando a sleep mode");
        estadoActual = SLEEP_MODE_ACTIVATED;
        // Reseteamos porque cambiamos de estado
        comienzoBusqueda = 0;  
      } 
      // Si todavia hay movimiento o no se supero el tiempo de busqueda, parpadeamos los LEDS
      else
      {
        printState("SEARCHING_STATE - Parpadeando LEDs");

        estadoLED = !estadoLED;
        digitalWrite(LED_PIN, estadoLED);

        tiempoDeteccion = millis();  
      }
    } 
  }
}

void firingState() 
{
  if (objetivoEnRango)
  {  
    printState("FIRING_STATE - Alerta");
  }
  else
  {
    estadoActual = TARGET_LOST_STATE;
  } 
}

void sleepingMode() {
  printState("SLEEPING_MODE_ACTIVATED - Goodnight");        
  moverTorretaHome();  
  // Apagamos el LED porque la torreta se va a dormir
  digitalWrite(LED_PIN, LOW);  
  estadoActual = SLEEPING_STATE;
}

void printState(String stateMessage) 
{
  if (PRINT_STATES)
    Serial.println(stateMessage);
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

float microsegundosACentimetros(float microsegundos)
{
  // El tiempo devuelto por el sensor de ultrasonido toma todo el viaje que realiza la onda de sonido 
  // (tanto de ida, como la vuelta luego del rebote). Para obtener la distancia del objeto detectado, dividimos por 2
  return microsegundos / MICROSEGUNDOS_POR_CM / 2;
}

void moverTorreta()
{
  if (leftToRight)
  {
    if (servoPosition < SERVO_MAX_POSITION)
    {
      servoPosition += SERVO_DELTA;
    }
    else
    {
      leftToRight = false;
      servoPosition -= SERVO_DELTA;
    }
  }
  else // Right to Left
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
  
  //Serial.println("Moviendo el servo a la posición ");
  //Serial.println(servoPosition);

  // Movemos el servo
  servo.write(servoPosition);
}

void moverTorretaHome()
{  
  Serial.print("Moviendo el servo a la posición inicial: ");
  Serial.print(SERVO_MIDDLE_POSITION);  
  Serial.println(" ");
  
  // Reiniciamos la posición del servo 
  servo.write(SERVO_MIDDLE_POSITION);
}
