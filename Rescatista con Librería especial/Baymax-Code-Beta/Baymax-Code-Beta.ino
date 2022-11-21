/*
 * Universidad Tecnológica de Tijuana
 * Ingeniería en Mecatrónica
 * Robot Rescatista "Baymax"
 * TSU. Rodríguez Corona Ernesto
 * 7A
 */
 
/*
 * Créditos a sus respectivos autores con respecto al algoritmo PID para el control de linea negra del robot
 * El algoritmo de búsqueda sí fue elaborado por el programador de este código
 * 
 * Versión 0.6 (sigue en testeo) 
 *  Esta version ya incluye el algoritmo de recepcion de datos por parte de LabVIEW y la detección de coordenadas del mismo mediante un sistema de visión
 *  
 */
 
//Libreria QTR proporcionada por el fabricante

#include <QTRSensors.h>

//Definicion de variables constantes en el proyecto

#define NUM_SENSORS   8  //numero de sensores usados
#define TIMEOUT       2500  // tiempo de espera para dar resultado en uS
#define NUM_SAMPLES_PER_SENSOR  4 
#define ir  13    //pin led on
#define ledCalibracion 2
#define PWMotorA 3
#define MotorARetroceso 4
#define MotorADerecho 5
#define ParoMotores 6
#define MotorBDerecho 7
#define MotorBRetroceso 8
#define PWMotorB 9
#define botonCalibracion 12
#define IR 13

int tiempoCalibracion = 20;
////Algoritmo PID proporcionado por la página https://aprendiendofacilelectronica.blogspot.com/2015/04/modificacion-libreria-qtr-sensors-para_18.html 

QTRSensorsAnalog qtr((unsigned char[]) {A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, IR);

unsigned int sensorValues[NUM_SENSORS];
unsigned int position = 0;

/// variables para el pid
int  derivativo=0, proporcional=0, integral=0; //parametros
int  salida_pwm=0, proporcional_pasado=0;


//_______AQUI CAMBIEREMOS LOS PARAMETROS DE NUESTRO ROBOT_________________
int velocidad=15; //variable para la velocidad, el maximo es 255
float Kp = 2;
float Kd = 0.5;
float Ki = 0.01;  //constantes
//variables para el control del sensado
int linea = 0;                //  0 para lineas negra, 1 para lineas blancas
int flanco_color = 0;      // aumenta o disminuye el valor del sensado
int en_linea = 500;         //valor al que considerara si el sensor esta en linea o no
int ruido = 50;          //valor al cual el valor del sensor es considerado como ruido
//________________________________________________________________________________

//Algoritmo PID proporcionado por la página https://aprendiendofacilelectronica.blogspot.com/2015/04/modificacion-libreria-qtr-sensors-para_18.html

// variables para la comunicacion LV-Arduino
char esperandoDato;
String bufferString;
char valor;
char estadoArduino;
int allowLecturas;
bool banderaCambio;
int coordenadaPersonaX [5];
int coordenadaPersonaY [5];
int coordenadaObstaculoX [5];
int coordenadaObstaculoY [5];

//banderas de recepcion de coordenadas 
bool banderaPersonaX [5];
bool banderaPersonaY [5];
bool banderaObstaculoX [5];
bool banderaObstaculoY [5];
bool banderaCargando = false;
bool banderaPersonas = false;
bool banderaObstaculos = false;
void reinicioBanderasCoordenadas ();
void reinicioPuntosCoordenadas ();
void mostrarCoordenadas ();
int x = 0;
int y = 0;

// Variables para el algoritmo de busqueda
void busquedaPrincipal ();
int posX = 0;
int posY = -3;
int orientacion = 0;
bool banderaSensor1 = true;
bool banderaSensor2 = true;
bool banderaInicioRuta = false;
bool rutinaInicio ();
bool origen = false;


/*
 * Inicializacion del arduino
 */
void setup() {
 
  Serial.begin (9600);
  
  //Salidas
  pinMode (ledCalibracion, OUTPUT);
  pinMode (PWMotorA, OUTPUT);
  pinMode (MotorARetroceso, OUTPUT);
  pinMode (MotorADerecho, OUTPUT);
  pinMode (ParoMotores, OUTPUT);
  pinMode (MotorBDerecho, OUTPUT);
  pinMode (MotorBRetroceso, OUTPUT);
  pinMode (PWMotorB, OUTPUT);
  
  //Entradas 
  pinMode (botonCalibracion, INPUT);
 
  //calibracion de QTR
  while (true) //Se queda en bucle en espera de accionamiento de boton para calibrar
  {
    if ( Serial.available() != 0)
    {
      valor = Serial.read();
      if (valor == 'D')
      {
        break;      
      }
    }
  }
  delay(500);
  digitalWrite(ledCalibracion, HIGH); // encendemos el led que indica que entramos en el modo de calibracion
  
  /* Informacion proporcionada por el fabricante
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  */
  
  for (uint16_t i = 0; i < 70; i++)
  {
    digitalWrite(ledCalibracion, HIGH); delay (20);
    qtr.calibrate();
    digitalWrite (ledCalibracion, LOW); delay (20);
  }
  digitalWrite(ledCalibracion, LOW); // una vez terminado la calibracion el led de calibracion se apaga.
   
   //Reiniciar variables para la recepcion de coordenadas por parte de LabVIEW
  reinicioBanderasCoordenadas ();
  reinicioPuntosCoordenadas ();
  digitalWrite (ParoMotores, LOW);
}

//rutinas para el algoritmo de busqueda

bool rutinaInicio () {
       qtr.read(sensorValues); //lectura en bruto de sensor
     
     if ((sensorValues [0] > 850 && sensorValues[7] > 850) && banderaSensor1 == false)
     {
      posY++;
      banderaSensor1 = true;
     }

     if ((sensorValues [0] < 250 && sensorValues[7] < 250) && banderaInicioRuta == true)
     {
      banderaSensor1 = false;
     }

     if (posY == 0)
     {
      origen = true;
      banderaInicioRuta = false;
      digitalWrite (ParoMotores, LOW);
     } { origen = false;}

     for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }  
  Serial.println (posY);

  return origen;
}












void setMotores(int motorA, int motorB)
{
  
  if ( motorA >= 0 )  //motor izquierdo
 {
  digitalWrite(MotorADerecho, HIGH); // con high avanza
  digitalWrite(MotorARetroceso, LOW);
  analogWrite(PWMotorA,85-motorA); //se controla de manera inversa para mayor control
 }
 else
 {
  digitalWrite(MotorADerecho, LOW);
  digitalWrite(MotorARetroceso,HIGH); //con low retrocede
  motorA = motorA*(-1); //cambio de signo
  analogWrite(PWMotorA,motorA); 
 }


  if ( motorB >= 0 ) //motor derecho
 {
  digitalWrite(MotorBDerecho, HIGH); // con high avanza
  digitalWrite(MotorBRetroceso, LOW); //con low retrocede
  analogWrite(PWMotorB,85-motorB);
 }
 else
 {
  digitalWrite(MotorBDerecho, LOW);
  digitalWrite(MotorBRetroceso,HIGH); //con low retrocede
  motorB = motorB*(-1);
  analogWrite(PWMotorB, motorB);
 }
}

//Aqui empiezan las funciones necesarias para la recepcion de coordenadas por parte de labVIEW

void reinicioPuntosCoordenadas ()
{
  for (int i = 0; i < 5; i++)
  {
    coordenadaPersonaX [i] = 0;
    coordenadaPersonaY [i] = 0;
    coordenadaObstaculoX [i] = 0;
    coordenadaObstaculoY [i] = 0;
  }
}

void reinicioBanderasCoordenadas ()
{
  x = 0; y = 0;
  for (int i = 0; i < 5; i++)
  {
    banderaPersonaX [i] = false;
    banderaPersonaY [i] = false;
    banderaObstaculoX [i] = false;
    banderaObstaculoY [i] = false;
    banderaPersonas = false;
    banderaObstaculos = false;
  }
}

void recepcionDatos ()
{
  do {
    if (Serial.available())
    {
      valor = Serial.read();
      bufferString += valor; // bufferString = (char)Serial.read() + bufferString
      if (valor == '\n' && banderaCargando == false)
      {
        banderaCargando = true;
        bufferString = "";
        continue;
      }

      //Recepcion de coordenadas X para las personas a rescatar
      if (valor == '\n' && banderaPersonaX [x] ==  false && banderaPersonas == false)
      {
        coordenadaPersonaX [x] = bufferString.toInt();
        banderaPersonaX [x] = true;
        y++;
        bufferString = "";
        continue;
      }

      //Recepcion de coordenadas en Y para las personas a rescatar
      if (valor == '\n' && banderaPersonaY [y - 1] ==  false && banderaPersonas == false)
      {
        coordenadaPersonaY [y - 1] = bufferString.toInt();
        banderaPersonaY [y - 1] = true;
        x++;
        bufferString = "";
        if (x == 3 && y == 3) {
          banderaPersonas = true;
          banderaObstaculos = false;
          x = 0;
          y = 0;
        }
        continue;
      }

      //Recepcion de la coordenadas en Y para los obstaculos a evitar
      if (valor == '\n' && banderaObstaculoX [x] == false && banderaObstaculos == false)
      {
        coordenadaObstaculoX [x] = bufferString.toInt();
        banderaObstaculoX [x] = true;
        y++;
        bufferString = "";
        continue;
      }

      if (valor == '\n' && banderaObstaculoY [y - 1] == false && banderaObstaculos == false)
      {
        coordenadaObstaculoY [ y - 1] = bufferString.toInt();
        banderaObstaculoY [y - 1] = true;
        x++;
        bufferString = "";
        if (x == 3 && y == 3) {break;} else {continue;}
      }
    }
  } while (true);
  reinicioBanderasCoordenadas();
}


void loop() {
/*
 * Lista de caracteres que se envian
 * A = Para inicar la ruta del robot
 * B = Paro del robot
 * C = Recepcion de coordenadas
 */
 
  if (Serial.available () != 0)
  {
    esperandoDato = Serial.read ();
    
    if (esperandoDato == 'A')
    {
      digitalWrite (ParoMotores, HIGH);
      banderaSensor1 = false;
      banderaInicioRuta = true;
    }

    if (esperandoDato == 'B')
    {
     digitalWrite (ParoMotores, LOW);
     banderaSensor1 = true;
     banderaInicioRuta = false;
    }
    
    if (esperandoDato == 'C')
    {
      bufferString = "";
      banderaCargando = false;
      banderaPersonas = false;
      banderaObstaculos = true;
      recepcionDatos ();
      esperandoDato == "";
    }
    
  }  
      rutinaInicio();
      pid(linea, velocidad, Kp, Ki, Kd, flanco_color, en_linea, ruido); //funcion para algoritmo pid(modificado ) (tipo_linea,velocidad,kp,ki,kd,flanco_color,en_linea,ruido)         
      frenos_contorno(linea,700); //funcion para frenado en curvas tipo  //flanco de comparación va desde 0 hasta 1000 , esto para ver si esta en negro o blanco
      
}

//Algoritmo de búsqueda

void busquedaPrincipal ()
{

}

//Algoritmo PID proporcionado por la página https://aprendiendofacilelectronica.blogspot.com/2015/04/modificacion-libreria-qtr-sensors-para_18.html
 void pid(int linea, int velocidad, float Kp, float Ki, float Kd,int flanco_color, int en_linea,int ruido)
{
  position = qtr.readLine(sensorValues, QTR_EMITTERS_ON, linea,flanco_color, en_linea, ruido ); //0 para linea negra, 1 para linea blanca
  proporcional = (position) - 3500; // set point es 3500, asi obtenemos el error


  integral=integral + proporcional_pasado; //obteniendo integral (integral += proporcional_pasado)
  derivativo = (proporcional - proporcional_pasado); //obteniedo el derivativo
  if (integral>1000) integral = 1000; //limitamos la integral para no causar problemas
  if (integral<-1000) integral =- 1000;
  salida_pwm =( proporcional * Kp ) + ( derivativo * Kd )+(integral*Ki);
  
  if (  salida_pwm > velocidad )  salida_pwm = velocidad; //limitamos la salida de pwm
  if ( salida_pwm < -velocidad )  salida_pwm = -velocidad;
  
  if (salida_pwm < 0)
 {
  setMotores (velocidad + salida_pwm, velocidad);
 }
 if (salida_pwm > 0)
 {
  setMotores (velocidad, velocidad-salida_pwm);
 }
 proporcional_pasado = proporcional;  
}

void frenos_contorno(int tipo, int flanco_comparacion)
{
  
if(tipo == 0)
{
  if (position <= 500) //si se salio por la parte derecha de la linea
 {
  setMotores(20, -15); //debido a la inercia, el motor tendera a seguri girando por eso le damos para atras , para que frene lo mas rapido posible 
  while(true)  
  {    
   qtr.read(sensorValues); //lectura en bruto de sensor   
   rutinaInicio ();
   if (origen == true)
   {
    break;
   }
if (sensorValues[3] > flanco_comparacion || sensorValues[4] > flanco_comparacion) 
//asegurar que esta en linea
   {
    break;
   } 
  }
 }

 if (position >= 6500) //si se salio por la parte izquierda de la linea
 { 
  setMotores(-15, 20);
  while(true)
  {
   rutinaInicio ();
   if (origen == true)
   {
    break;
   }
   qtr.read(sensorValues);
if (sensorValues[5] > flanco_comparacion || sensorValues[4] > flanco_comparacion )
   {
    break;
   }  
  }
 }
}
}
