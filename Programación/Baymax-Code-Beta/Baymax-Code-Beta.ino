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

////Algoritmo PID proporcionado por la página https://aprendiendofacilelectronica.blogspot.com/2015/04/modificacion-libreria-qtr-sensors-para_18.html 

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


//_______AQUI CAMBIEREMOS LOS PARAMETROS DE NUESTRO ROBOT_________________
int velocidad = 110; //variable para la velocidad, el maximo es 255
float Kp = 0.5; //Kp = 1.5
float Kd = 4.8; // Kp = 4.8
float Ki = 0.01;  // Ki = 0.01
//variables para el control del sensado
// Data para integral 
int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;

//declaracion de variables para utilizar PID
int proporcional = 0;         // Proporcional
int integral = 0;           //Integral
int derivativo = 0;          // Derivativo
     
int PWM = 0;   // Diferencia aplicada a los motores
int ultimoProporcional;         // Última valor del proporcional (utilizado para calcular la derivada del error)
int setPoint = 4000; // Setpoint (Como utilizamos 6 sensores, la línea debe estar entre 0 y 5000, por lo que el ideal es que esté en 2500)

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
int posY = -4;
int orientacion = 0;
bool banderaSensor1 = true;
bool banderaSensor2 = true;
bool banderaInicioRuta = false;
bool banderaPosX = false;
bool banderaPosY = false;
void rutinaInicio ();

bool banderaStart = false;
int orientacionY = 0;
int orientacionX = 0;
int pR = 0; // Persona a Rescatar
bool banderaPr [5];
void rutinaInicio ();
void arribaIzquierda (); // Giro de 90° a la izquierda cuando el sensor se encuentra en direccion al eje positivo Y
void arribaDerecha (); // Giro de 90° a la derecha cuando el sensor se encuentra direccion al eje positivo Y
void abajoIzquierda (); // Giro de 90° a la izquierda cuando el sensor se encuentra en direccion al eje negativo Y
void abajoDerecha (); // Giro de 90° a la derecha cuando el sensor se encuentra en direccion al eje negativo Y
void derechaAbajo (); // Giro de 90° a la derecha cuando el sensor se encuentra en direccion al eje positivo X
void derechaArriba (); // Giro de 90° a la izquierda cuando el sensor se encuentra en direccion al eje positivo X
void izquierdaAbajo (); // Giro de 90° a la izquierda cuando el sensor se encuentra en direccion al eje negativo X
void izquierdaArriba (); // Giro de 90° a la izquierda cuando el sensor se encuentra en direccion al eje negativo X
bool prueba = false;

//variables para rutina de motores
int tiempoVuelta = 300;
int tiempoAvance = 250;
/*
 * Inicializacion del arduino
 */

 void giroDerecha ()
 {
    digitalWrite (ParoMotores, HIGH); 
    analogWrite (PWMotorA, velocidad);
    analogWrite (PWMotorB, velocidad);
    digitalWrite (MotorADerecho, HIGH); digitalWrite (MotorARetroceso, LOW);
    digitalWrite (MotorBDerecho, HIGH); digitalWrite (MotorBRetroceso, LOW);
    delay(tiempoAvance);
    digitalWrite (MotorADerecho, HIGH); digitalWrite (MotorARetroceso, LOW);
    digitalWrite (MotorBDerecho, LOW); digitalWrite (MotorBRetroceso, HIGH);
    delay(tiempoVuelta);
    digitalWrite (ParoMotores, LOW);  
 }

 void giroIzquierda ()
 {
    digitalWrite (ParoMotores, HIGH); 
    analogWrite (PWMotorA, velocidad);
    analogWrite (PWMotorB, velocidad);
    digitalWrite (MotorADerecho, HIGH); digitalWrite (MotorARetroceso, LOW);
    digitalWrite (MotorBDerecho, HIGH); digitalWrite (MotorBRetroceso, LOW);
    delay(tiempoAvance);
    digitalWrite (MotorADerecho, LOW); digitalWrite (MotorARetroceso, HIGH);
    digitalWrite (MotorBDerecho, HIGH); digitalWrite (MotorBRetroceso, LOW);
    delay(tiempoVuelta);
    digitalWrite (ParoMotores, LOW);  
 }

 void giroCompleto ()
 {
    digitalWrite (ParoMotores, HIGH); 
    analogWrite (PWMotorA, velocidad);
    analogWrite (PWMotorB, velocidad);
    digitalWrite (MotorADerecho, HIGH); digitalWrite (MotorARetroceso, LOW);
    digitalWrite (MotorBDerecho, HIGH); digitalWrite (MotorBRetroceso, LOW);
    delay(700);
    digitalWrite (ParoMotores, LOW);  
 }
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
  digitalWrite(ledCalibracion, HIGH); // encendemos el led que indica que entramos en el modo de calibracion
  
  /* Informacion proporcionada por el fabricante
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  */
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(13);

  
  for (uint16_t i = 0; i < 70; i++)
  {
    digitalWrite(ledCalibracion, HIGH); delay (20);
    qtr.calibrate();
    digitalWrite (ledCalibracion, LOW); delay (20);
  }
  digitalWrite(ledCalibracion, LOW); // una vez terminado la calibracion el led de calibracion se apaga.

   unsigned int position = 0; 
   
   //Reiniciar variables para la recepcion de coordenadas por parte de LabVIEW
  reinicioBanderasCoordenadas ();
  reinicioPuntosCoordenadas ();
  
  digitalWrite (ParoMotores, HIGH);
        banderaSensor1 = false;
      banderaInicioRuta = true;
  delay (3000);
}

//rutinas para el algoritmo de busqueda

void rutinaInicio () {
       uint16_t position = qtr.readLineWhite(sensorValues);
     
     if ((sensorValues [0] > 850 && sensorValues[7] > 850) && banderaSensor1 == false)
     {
      posY++;
      banderaSensor1 = true;
     }

     if ((sensorValues [0] < 250 && sensorValues[7] < 250) && banderaInicioRuta == true)
     {
      banderaSensor1 = false;
     }

     if (posY == 0 && prueba == false)
     {     
      prueba = true;
      banderaInicioRuta = false;
      digitalWrite (ParoMotores, LOW);
      delay (1000);
      giroDerecha ();                    
     }
}

// control PID de los motores

void motorIzquierdo (int PWM)
{
  if (PWM >= 0)
  {
    digitalWrite(MotorADerecho, HIGH); // con high avanza
    digitalWrite(MotorARetroceso, LOW);    
  } else {
    digitalWrite(MotorADerecho, LOW);
    digitalWrite(MotorARetroceso,HIGH); //con low retrocede
    PWM *= -1;
  }
  analogWrite(PWMotorA,PWM);
}

void motorDerecho (int PWM)
{
    if (PWM >= 0)
  {
    digitalWrite(MotorBDerecho, HIGH); // con high avanza
    digitalWrite(MotorBRetroceso, LOW);    
  } else {
    digitalWrite(MotorBDerecho, LOW);
    digitalWrite(MotorBRetroceso,HIGH); //con low retrocede
    PWM *= -1;
  }
  analogWrite(PWMotorB,PWM);
}
void setMotores(int motorA, int motorB)
{
  motorIzquierdo (motorA);
  motorDerecho (motorB);
}

void frenoMotores (boolean motorA, boolean motorB, int PWM)
{
  if (motorA)
  {
    digitalWrite(MotorADerecho, HIGH); // con high avanza
    digitalWrite(MotorARetroceso, HIGH);
    analogWrite(PWMotorA,PWM);
  }

  if (motorB)
  {
    digitalWrite(MotorBDerecho, HIGH); // con high avanza
    digitalWrite(MotorBRetroceso, HIGH);
    analogWrite(PWMotorB,PWM);    
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
    banderaPr [i] =  false;
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
  for (int i = 0; i < coordenadaPersonaX[0]; i++)
  {
    digitalWrite (ledCalibracion, HIGH); delay (20);
    digitalWrite (ledCalibracion, LOW); delay (20);
  }
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
      esperandoDato = "";      
    }

    if (esperandoDato == 'B')
    {
     digitalWrite (ParoMotores, LOW);
     banderaSensor1 = true;
     banderaInicioRuta = false;
     posY = -4;
     esperandoDato = "";
    }
    
    if (esperandoDato == 'C')
    {
      bufferString = "";
      banderaCargando = false;
      banderaPersonas = false;
      banderaObstaculos = true;
      recepcionDatos ();
      esperandoDato = "";
    }
    
  }  
    rutinaInicio();
    pid();            
}

//Algoritmo de búsqueda

void busquedaPrincipal ()
{

}

//Algoritmo PID proporcionado por: 
 void pid()
{
   uint16_t position = qtr.readLineWhite(sensorValues);
  proporcional = ((int)position) - 3500;


  if ( proporcional <= -setPoint )
  {
    motorDerecho(0);
    frenoMotores (true,false,velocidad);
  }
  else if ( proporcional >= setPoint )
  {
    motorIzquierdo(0);
    frenoMotores (false,true,velocidad);
  }
  
  derivativo = proporcional - ultimoProporcional; 
  integral = error1 + error2 + error3 + error4 + error5 + error6;
  ultimoProporcional = proporcional;
  
  error6 = error5;
  error5 = error4;  
  error4 = error3;  
  error3 = error2;
  error2 = error1;
  error1 = proporcional;

 
  int diferencial = ( proporcional * Kp ) + ( derivativo * Kd )+ (integral*Ki) ;
  
  if ( diferencial > velocidad ) diferencial = velocidad; 
  else if ( diferencial < -velocidad ) diferencial = -velocidad;

  ( diferencial < 0 ) ?  setMotores (velocidad + diferencial, velocidad) : setMotores (velocidad, velocidad - diferencial);
}
