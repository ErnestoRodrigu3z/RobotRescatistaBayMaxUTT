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
float Kd = 5.2; // Kp = 4.8
float Ki = 0.05;  // Ki = 0.01
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
int posY = -3;
int orientacion = 0;
bool banderaSensor1 = true;
bool banderaSensor2 = true;
bool banderaInicioRuta = false;
bool banderaPosX = false;
bool banderaPosY = false;
void rutinaInicio ();
void backStart ();



bool banderaStart = false;
int orientacionInicio = 0;
int orientacionY = 0;
int orientacionX = 0;
int orientacionBack = 0;
int orientacionBackX = 0;
int orientacionBackY = 0;
int pR = 0; // Persona a Rescatar
bool banderaPr [5];
void rutinaInicio ();
bool prueba = false;

//variables para rutina de motores
int tiempoVuelta = 300;
int tiempoAvance = 150;
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
    delay(tiempoVuelta+25);
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
  for (int i = 0; i < coordenadaPersonaY[0]; i++)
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
      banderaStart = false;
      banderaSensor1 = false;
      banderaInicioRuta = true;
      orientacionInicio = 1;
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
    if (prueba == false)
    {
     rutinaInicio(); 
    }
    if (banderaInicioRuta == true)
    {      
      busquedaPrincipal();           
    }
    pid();            
}

//Algoritmo de búsqueda
void rutinaInicio () {
       uint16_t position = qtr.readLineWhite(sensorValues);
     
     if ((sensorValues [0] > 850 && sensorValues[7] > 850) && banderaSensor1 == false)
     {
      posY++;
      banderaSensor1 = true;
     }

     if ((sensorValues [0] < 250 && sensorValues[7] < 250))
     {
      banderaSensor1 = false;
     }

     if (posY == 0 && prueba == false)
     {     
      prueba = true;
      banderaInicioRuta = false;
      digitalWrite (ParoMotores, LOW);
      delay (500);                          
     }
}

void lecturaCoordenadaX (bool ejeX) //eje X indicara la lectura de coordenada en dicho eje
{
 /*
  * ejeX = true indica que se sumara posX
  * ejeX = false indicara que se sumanra posY
  */
  uint16_t position = qtr.readLineWhite(sensorValues);
  if ((sensorValues[7] > 850 && sensorValues [0] > 850) && banderaSensor1 == false)
    {
      (ejeX) ? posX++ : posX--;
      banderaSensor1 = true;
    }
  if (sensorValues[7] < 400 && sensorValues [0] < 400)
    {
      banderaSensor1 = false;
    }
  if (posX == coordenadaPersonaX[pR])
    {
      digitalWrite (ParoMotores, LOW);       
    }
     
}

void lecturaCoordenadaY (bool ejeY) //eje Y indicara la lectura de coordenada en dicho eje
{
   uint16_t position = qtr.readLineWhite(sensorValues);
  if ((sensorValues [0] > 850 && sensorValues[7] > 850) && banderaSensor1 == false)
     {
      (ejeY) ? posY++ : posY--;
      banderaSensor1 = true;
     }

  if ((sensorValues [0] < 250 && sensorValues[7] < 250))
     {
      banderaSensor1 = false;
     }
  if (posY == coordenadaPersonaY[pR])
     {
       digitalWrite (ParoMotores, LOW);       
     }
     
}
void busquedaPrincipal (){
  /*
  * El robot tomará en cuenta que la matriz es un plano cartesiano
  * Teniendo 8 regiones de interes Las cuales se toman cómo lo siguientes:
  * R1(-2, -2); R2(2, -2); R3(2, 2); R4(-2, 2); R5(-1, -1); R6(1, -1); R7(1, 1); R8(-1, 1);
  * El programa desarrollado en LabVIEW se encarga leer la tarjeta para determinar las regiones de interes en donde se encuentran las personas a rescatar
  * y los obstaculos a evitar
  * El resultado de esa lectura, generará las coordenadas las cualés se enviarán al Arduino mediante la comunicación HC-05
  * El controlador se encargará de determinar la mejor ruta a seguir
  */

  /* OrientacionY
   * Caso 1: Va hacía la izquierda partiendo del origen
   * Caso 2: Va hacia la derecha partiendo del origen 
   */
   
   /* OrientacionX
    * Caso 1: El sensor está orientado hacia el Eje positivo Y
    * Csso 2: El sensor está orientado hacia el eje negativo Y
    */

  switch (orientacionInicio)
  {
    case 1:
      if (posX  > coordenadaPersonaX [pR])
        {
          digitalWrite (ParoMotores, HIGH);
          giroIzquierda ();
          orientacionX = 1;                                      
          orientacionInicio = 0;                      
        } 
      if (posX < coordenadaPersonaX [pR])
      {    
        digitalWrite (ParoMotores, HIGH);
        giroDerecha ();           
        orientacionX = 2;         
        orientacionInicio = 0;                 
      } 
    break; 

    case 2:
      if (posX  > coordenadaPersonaX [pR])
      {
        digitalWrite (ParoMotores, HIGH);
        giroDerecha ();
        orientacionX = 1;                                    
        orientacionInicio = 0;                      
      } 
      if (posX < coordenadaPersonaX [pR])
      {     
        digitalWrite (ParoMotores, HIGH);
        giroIzquierda ();        
        orientacionX = 2;          
        orientacionInicio = 0;                 
      }    
    break;
  }

  switch (orientacionX)
  {
   case 1:
      lecturaCoordenadaX (false);          
      if (coordenadaPersonaX [pR] == posX && posY > coordenadaPersonaY [pR])
      {      
        giroIzquierda();
        orientacionY = 2;                   
        orientacionX = 0;
        banderaPr [pR] = true;        
      } 
      
      if (coordenadaPersonaX [pR] == posX && posY < coordenadaPersonaY [pR])
      {      
        giroDerecha();
        orientacionY = 1;                                
        orientacionX = 0;
        banderaPr [pR] = true;
      }
   break;

    case 2:      
      lecturaCoordenadaX (true);
              
      if (coordenadaPersonaX [pR] == posX && posY > coordenadaPersonaY [pR])
      {        
        giroDerecha(); 
        orientacionY = 2;
        orientacionX = 0;        

      }
      
      if (coordenadaPersonaX [pR] == posX && posY < coordenadaPersonaY [pR])
      {        
        giroIzquierda(); 
        orientacionY = 1;
        orientacionX = 0;        
      }      
    break;
  }
  switch (orientacionY)
  {
    case 1:
      lecturaCoordenadaY(true);
      
      if ((posX  == coordenadaPersonaX [pR]) && posY == coordenadaPersonaY [pR])
      {        
        digitalWrite (ParoMotores, LOW);
        orientacionBack = 1;              
      }
    break;
      
    case 2:
      lecturaCoordenadaY(false);
      
      if ((posX  == coordenadaPersonaX [pR]) && posY == coordenadaPersonaY [pR])
      {        
        digitalWrite (ParoMotores, LOW); 
        orientacionBack = 2;       
      }            
    break;
   }
   if (false) backStart();
  }

void backStart ()
{
  /* OrientacionBack
   * Caso 1: el sensor se encuentra volteando hacia positivo Y
   * Caso 2: el sensor se encuentra volteando hacia negativo Y
   */

   /* OrientacionBackX
    * Caso 1:
    * Caso 2:
    */
  switch (orientacionBack)
  {
    case 1:      
      if (posX  > 0 )
      {        
        digitalWrite (ParoMotores, HIGH);
        giroIzquierda ();                  
        orientacionBackX = 1;                                          
        orientacionBack = 0;                     
      } 
      
      if (posX < 0)
      {        
        digitalWrite (ParoMotores, HIGH);
        giroDerecha ();                   
        orientacionBackX = 2;                  
        orientacionBack = 0;        
      }          
    break;
    
    case 2:    
      if (posX  > 0 )
      {        
        digitalWrite (ParoMotores, HIGH);
        giroDerecha ();                  
        orientacionBackX = 1;                                          
        orientacionBack = 0;                     
      } 
      
      if (posX < 0)
      {        
        digitalWrite (ParoMotores, HIGH);
        giroIzquierda ();                    
        orientacionBackX = 2;                  
        orientacionBack = 0;        
      } 
    break;
  }
  
  switch (orientacionBackX)
  {    
    case 1:
      lecturaCoordenadaX (false);
      if (posX == 0 && posY > 0)
      {
        giroIzquierda ();
        orientacionBackX = 0;
        orientacionBackY = 2;
      }

      if (posX == 0 && posY < 0)
      {
        giroDerecha ();
        orientacionBackX = 0;
        orientacionBackY = 1;
      }
    break;

    case 2:
      lecturaCoordenadaX (true);
  
      if (posX == 0 && posY > 0)
      {
        giroDerecha();
        orientacionBackX = 0;
        orientacionBackY = 2;      
      }
  
      if (posX == 0 && posY < 0)
      {
        giroIzquierda();
        orientacionBackX = 0;
        orientacionBackY = 1;                  
      }
    break;
  }

  switch (orientacionBackY)
  {
   case 1:
    lecturaCoordenadaY (false);
    if (posX == 0 && posY == 0)
    {
      orientacionInicio = 2;
      orientacionBackY = 0;
      pR++;
      posY = 0; posX = 0;
      //desahabilitar bandera      
    }
    break;

    case 2:
      lecturaCoordenadaY (true);
      if (posX == 0 && posY == 0)
      {
       orientacionInicio = 1;
       orientacionBackY = 0;
       pR++;
       posY = 0; posX = 0;
       //desahabilitar bandera
      }
    break;
  }

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
