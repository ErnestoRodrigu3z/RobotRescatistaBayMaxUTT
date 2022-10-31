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
 * Versión 0.2 (sigue en testeo) 
 *  
 */
 
//Libreria QTR proporcionada por el fabricante

#include <QTRSensors.h>

//Definicion de variables constantes en el proyecto

#define NUM_SENSORS   8  //numero de sensores usados
#define TIMEOUT       2500  // tiempo de espera para dar resultado en uS
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

QTRSensorsRC qtr((unsigned char[]) {A7, A6, A5, A4, A3, A2, A1, A0}, NUM_SENSORS, TIMEOUT, IR);

unsigned int sensorValues[NUM_SENSORS];
unsigned int position = 0;

/// variables para el pid
int  derivativo=0, proporcional=0, integral=0; //parametros
int  salida_pwm=0, proporcional_pasado=0;


//_______AQUI CAMBIEREMOS LOS PARAMETROS DE NUESTRO ROBOT_________________
int velocidad=70; //variable para la velocidad, el maximo es 255
float Kp=0.18;
float Kd=2;
float Ki=0.01;  //constantes
//variables para el control del sensado
int linea=0;                //  0 para lineas negra, 1 para lineas blancas
int flanco_color =0;      // aumenta o disminuye el valor del sensado
int en_linea=500;         //valor al que considerara si el sensor esta en linea o no
int ruido=  50;          //valor al cual el valor del sensor es considerado como ruido
//________________________________________________________________________________

//Algoritmo PID proporcionado por la página https://aprendiendofacilelectronica.blogspot.com/2015/04/modificacion-libreria-qtr-sensors-para_18.html

// variables para la comunicacion LV-Arduino
char esperandoDato;

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
  while (botonCalibracion); //Se queda en bucle en espera de accionamiento de boton para calibrar
  delay(500);
  digitalWrite(ledCalibracion, HIGH); // encendemos el led que indica que entramos en el modo de calibracion
  
  /* Informacion proporcionada por el fabricante
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  */
  
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(ledCalibracion, LOW); // una vez terminado la calibracion el led de calibracion se apaga.
 
}

void setMotores(int motorA, int motorB)
{
  
  if ( motorA >= 0 )  //motor izquierdo
 {
  digitalWrite(MotorADerecho, HIGH); // con high avanza
  digitalWrite(MotorARetroceso, LOW);
  analogWrite(PWMotorA,255-motorA); //se controla de manera inversa para mayor control
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
  analogWrite(PWMotorA,255-motorB);
 }
 else
 {
  digitalWrite(MotorBDerecho, LOW);
  digitalWrite(MotorBRetroceso,HIGH); //con low retrocede
  motorB = motorB*(-1);
  analogWrite(PWMotorB, motorB);
 }
}

void recepcionDatos ()
{
  
}


void loop() {
/*
 * Lista de caracteres que se envian
 * 
 */
  if (Serial.available () != 0)
  {
    esperandoDato = Serial.read ();
    
    if (esperandoDato == 'A')
    {
        pid(linea, velocidad, Kp, Ki, Kd, flanco_color, en_linea, ruido); //funcion para algoritmo pid(modificado ) (tipo_linea,velocidad,kp,ki,kd,flanco_color,en_linea,ruido)

         
      frenos_contorno(linea,700); //funcion para frenado en curvas tipo  //flanco de comparación va desde 0 hasta 1000 , esto para ver si esta en negro o blanco
    }
    
  }
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
  setMotores(-80, 90); //debido a la inercia, el motor tendera a seguri girando por eso le damos para atras , para que frene lo mas rapido posible 
  while(true)  
  {
   qtr.read(sensorValues); //lectura en bruto de sensor   
if (sensorValues[0] > flanco_comparacion || sensorValues[1] > flanco_comparacion) 
//asegurar que esta en linea
   {
    break;
   } 
  }
 }

 if (position >= 6500) //si se salio por la parte izquierda de la linea
 { 
  setMotores(90, -80);
  while(true)
  {
   qtr.read(sensorValues);
if (sensorValues[7] > flanco_comparacion || sensorValues[6] > flanco_comparacion )
   {
    break;
   }  
  }
 }
}
}
