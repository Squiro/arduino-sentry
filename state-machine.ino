
//----------------------------------------------
// Registros y valores de la interrupcion de tiempo ...
#define TIMER1_TCCR1A  0b00000000
#define TIMER1_TCCR1B  0b00001101
//#define TIMER1_OCR1A   156    // 10 mseg aprox
#define TIMER1_OCR1A   813    // 20 mseg
//#define TIMER1_OCR1A   3125   // 200 mseg
//----------------------------------------------

//----------------------------------------------
// Pines de los sensores ...
const int Pin_Sensor_1 = 9;
const int Pin_Sensor_2 = 8;
const int Pin_Sensor_3 = 7;
const int Pin_Sensor_4 = 6;
//----------------------------------------------

#define PIN_7_BIT_0      	2
#define PIN_7_BIT_1      	5
#define PIN_7_BIT_2      	4
#define PIN_7_BIT_3      	3
#define HUMBRAL_CM_AUTO 	50
#define MAX_CANT_SENSORES  	4

// Tipos de eventos disponibles .
//----------------------------------------------
#define TIPO_EVENTO_COCHERA_OCUPADA 1000
#define TIPO_EVENTO_COCHERA_LIBRE	2000
#define TIPO_EVENTO_TIMEOUT 		3000
#define TIPO_EVENTO_CONTINUE		4000
//----------------------------------------------

//----------------------------------------------
// Estado del embeded ...
#define ESTADO_EMBEDDED_INIT		100
#define ESTADO_EMBEDDED_IDLE		200
//----------------------------------------------

//----------------------------------------------
// Estado de un sensor ...
#define ESTADO_SENSOR_LIBRE 		108
#define ESTADO_SENSOR_OCUPADO   	666
//----------------------------------------------

//-------------------------------------------------
int estado;
int contador_autos 					= 0;
int sensor_actual  					= 0;
bool timeout       					= false;
//-------------------------------------------------

//-------------------------------------------------
struct stSensor
{
  int  pin;
  int  estado;
  long centimetros;
  long tiempo_estacionamiento;
};
stSensor sensores[MAX_CANT_SENSORES];
//-------------------------------------------------

//-------------------------------------------------
struct stEvento
{
  int tipo;
  int param;
};
stEvento evento;
//-------------------------------------------------

//----------------------------------------------------------
// Interrupcion de temporizador 1.

ISR(TIMER1_COMPA_vect)
{
  timeout = true;
}
//-------------------------------------------------

//-------------------------------------------------
void do_init()
{
  Serial.begin(9600);

  pinMode( PIN_7_BIT_0, OUTPUT);
  pinMode( PIN_7_BIT_1, OUTPUT);
  pinMode( PIN_7_BIT_2, OUTPUT);
  pinMode( PIN_7_BIT_3, OUTPUT);
  
  sensores[0].pin = Pin_Sensor_1;
  sensores[1].pin = Pin_Sensor_2;
  sensores[2].pin = Pin_Sensor_3;
  sensores[3].pin = Pin_Sensor_4;
  sensores[0].estado = ESTADO_SENSOR_LIBRE;
  sensores[1].estado = ESTADO_SENSOR_LIBRE;
  sensores[2].estado = ESTADO_SENSOR_LIBRE;
  sensores[3].estado = ESTADO_SENSOR_LIBRE;

  // Inicializo el evento inicial
  estado = ESTADO_EMBEDDED_INIT;
  
  // El pin OC1A cambia de estado tras la comparación.
  TCCR1A = TIMER1_TCCR1A;
  
  // Usa el registro OCR1A para comparar y factor escala 1024.
  TCCR1B = TIMER1_TCCR1B;

  // Para que la interrupción ocurra al segundo.
  OCR1A  = TIMER1_OCR1A;

  // Se utilizara la interrupción timer1 por comparación.
  TIMSK1 = bit(OCIE1A); //0b00000010;

  // Habilita las interrupciones globales.
  sei();
}
//-------------------------------------------------

//-------------------------------------------------
void actualizar_display( )
{
   digitalWrite( PIN_7_BIT_0, (contador_autos&0b0001)>>0);
   digitalWrite( PIN_7_BIT_1, (contador_autos&0b0010)>>1);
   digitalWrite( PIN_7_BIT_2, (contador_autos&0b0100)>>2);
   digitalWrite( PIN_7_BIT_3, (contador_autos&0b1000)>>3);  
}
//-------------------------------------------------

//-------------------------------------------------
long leer_sensor( int pin )
{
  long tiempo_pulso;
  
  pinMode( pin, OUTPUT );
  digitalWrite( pin, LOW);
  
  delayMicroseconds(2);
  digitalWrite( pin, HIGH );
  
  delayMicroseconds(5);
  digitalWrite( pin, LOW );

  pinMode( pin, INPUT );
  tiempo_pulso = pulseIn( pin, HIGH );

  // Convierto la medicion en centimetros.
  return tiempo_pulso / 29 / 2;
}
//-------------------------------------------------

// ----------------------------------------------------
bool verificarEstadoSensor( int sensor )
{
  if( sensor >= MAX_CANT_SENSORES ) return false;
  
  sensores[sensor].centimetros = leer_sensor( sensores[sensor].pin );
  
  switch( sensores[sensor].estado )
  {
    case ESTADO_SENSOR_LIBRE:
	{
		if( (sensores[sensor].centimetros <= HUMBRAL_CM_AUTO) )
		{
			evento.tipo  = TIPO_EVENTO_COCHERA_OCUPADA;
			evento.param = sensor+1;
		  
			sensores[sensor].estado = ESTADO_SENSOR_OCUPADO;
			
			return true;
		}
	}
    break;
	
    case ESTADO_SENSOR_OCUPADO:
	{
		if( (sensores[sensor].centimetros > HUMBRAL_CM_AUTO) )
		{
			evento.tipo  = TIPO_EVENTO_COCHERA_LIBRE;
			evento.param = sensor+1;
		  
			sensores[sensor].estado = ESTADO_SENSOR_LIBRE;
			
			return true;
		}
	}
    break;
	
	default:
		Serial.println("-----------------------------------------------------");
		Serial.println("Estado de un sensor no reconocido...");
		Serial.println(sensores[sensor].estado);
		Serial.println(sensor+1);
		Serial.println("-----------------------------------------------------");
	break;
  }
  
  return false;
}

//-------------------------------------------------
void genera_evento( )
{
	if( timeout )
	{
		verificarEstadoSensor(sensor_actual);
		
		if( sensor_actual < MAX_CANT_SENSORES )
		{
			sensor_actual ++;
		}
		else
		{
			sensor_actual = 0;
			
			// doy acuse de la recepcion del timeout
			timeout = false;
		}
	}
	else
	{
		evento.tipo = TIPO_EVENTO_CONTINUE;
	}
}
//-------------------------------------------------

//-------------------------------------------------
void maquina_estados( )
{
  genera_evento();
  
  switch( estado )
  {
    case ESTADO_EMBEDDED_INIT:
	{
		switch(evento.tipo)
		{
			case TIPO_EVENTO_CONTINUE:
			{
				Serial.println("-----------------------------------------------------");
				Serial.println("Estado ESTADO_EMBEDDED_INIT...");
				Serial.println("Evento TIPO_EVENTO_CONTINUE...");
				Serial.println("-----------------------------------------------------");
				
				contador_autos = 0;
			
				actualizar_display( );
				
				estado = ESTADO_EMBEDDED_IDLE;
			}
			break;
			
			default:
				Serial.println("-----------------------------------------------------");
				Serial.println("Estado ESTADO_EMBEDDED_INIT...");
				Serial.println("Evento NO RECONOCIDO...");
				Serial.println(evento.tipo);
				Serial.println("-----------------------------------------------------");
			break;
		}
	}
	break;
	
    case ESTADO_EMBEDDED_IDLE: 
	{
		switch(evento.tipo)
		{
			case TIPO_EVENTO_COCHERA_OCUPADA:
			{
				Serial.println("-----------------------------------------------------");
				Serial.println("Estado ESTADO_EMBEDDED_IDLE...");
				Serial.println("Evento TIPO_EVENTO_COCHERA_OCUPADA...");
				Serial.println(evento.param);
				Serial.println("-----------------------------------------------------");
				
				contador_autos ++;
			
				actualizar_display( );
				
				estado = ESTADO_EMBEDDED_IDLE;
			}
			break;
		
			case TIPO_EVENTO_COCHERA_LIBRE:
			{
				Serial.println("-----------------------------------------------------");
				Serial.println("Estado ESTADO_EMBEDDED_IDLE...");
				Serial.println("Evento TIPO_EVENTO_COCHERA_LIBRE...");
				Serial.println(evento.param);
				Serial.println("-----------------------------------------------------");
				
				contador_autos --;
			
				actualizar_display( ); 
				
				estado = ESTADO_EMBEDDED_IDLE;
			}
			break;
			
			case TIPO_EVENTO_CONTINUE:
			{
				//Serial.println("-----------------------------------------------------");
				//Serial.println("Estado ESTADO_EMBEDDED_IDLE...");
				//Serial.println("Evento TIPO_EVENTO_CONTINUE...");
				//Serial.println("-----------------------------------------------------");
				
				estado = ESTADO_EMBEDDED_IDLE;
			}
			break;
			
			case TIPO_EVENTO_TIMEOUT:
			{
				Serial.println("-----------------------------------------------------");
				Serial.println("Estado ESTADO_EMBEDDED_IDLE...");
				Serial.println("Evento TIPO_EVENTO_TIMEOUT...");
				Serial.println("-----------------------------------------------------");
				
				estado = ESTADO_EMBEDDED_IDLE;
			}
			break;
			
			default:
				Serial.println("-----------------------------------------------------");
				Serial.println("Estado ESTADO_EMBEDDED_IDLE...");
				Serial.println("Evento NO RECONOCIDO...");
				Serial.println(evento.tipo);
				Serial.println("-----------------------------------------------------");
			break;
		}
	}	
	break; 
  }
  
  // Consumo el evento...
  evento.tipo  = TIPO_EVENTO_CONTINUE;
  evento.param = -1;
}


// Funciones de arduino !. 

//-------------------------------------------------
void setup()
{
    do_init();
}
//-------------------------------------------------

//-------------------------------------------------
void loop()
{
	maquina_estados();
}
//-------------------------------------------------

