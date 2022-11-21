#include <QTRSensors.h>


QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// This example is designed for use with six QTR-1A sensors or the first six sensors of a
// QTR-8A module.  These reflectance sensors should be connected to analog inputs 0 to 5.
// The QTR-8A's emitter control pin (LEDON) can optionally be connected to digital pin 2, 
// or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to 
// QTR_NO_EMITTER_PIN.

// The setup phase of this example calibrates the sensor for ten seconds and turns on
// the LED built in to the Arduino on pin 13 while calibration is going on.
// During this phase, you should expose each reflectance sensor to the lightest and 
// darkest readings they will encounter.
// For example, if you are making a line follower, you should slide the sensors across the
// line during the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is.  Improper calibration will result in poor readings.
// If you want to skip the calibration phase, you can get the raw sensor readings
// (analog voltage readings from 0 to 1023) by calling qtra.read(sensorValues) instead of
// qtra.readLine(sensorValues).

// The main loop of the example reads the calibrated sensor values and uses them to
// estimate the position of a line.  You can test this by taping a piece of 3/4" black
// electrical tape to a piece of white paper and sliding the sensor across it.  It
// prints the sensor values to the serial monitor as numbers from 0 (maximum reflectance) 
// to 1000 (minimum reflectance) followed by the estimated location of the line as a number
// from 0 to 5000.  1000 means the line is directly under sensor 1, 2000 means directly
// under sensor 2, etc.  0 means the line is directly under sensor 0 or was last seen by
// sensor 0 before being lost.  5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.



int ParoMotores = 5;
//banderas de recepcion de coordenadas 
int coordenadaPersonaX [5];
int coordenadaPersonaY [5];
int coordenadaObstaculoX [5];
int coordenadaObstaculoY [5];
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


//Variables a migrar
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

void rutinaInicio () 
{
  
}
void arribaIzquierda ()
{
  
}

void arribaDerecha () 
{
  
}

void abajoDerecha ()
{
  
}

void abajoIzquierda ()
{
  
}

void derechaAbajo ()
{
  
}

void derechaArriba ()
{
  
}

void izquierdaAbajo ()
{
  
}
void izquierdaArriba ()
{

}


void setup()
{
  pinMode (ParoMotores, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

  digitalWrite(2, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
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
    digitalWrite(2, HIGH); delay (20);
    qtr.calibrate();
    digitalWrite (2, LOW); delay (20);
  }
  digitalWrite(2, LOW); // una vez terminado la calibracion el led de calibracion se apaga.

   unsigned int position = 0; 
}


void loop()
{ 
 uint16_t position = qtr.readLineWhite(sensorValues);
  for (unsigned char i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

if ((sensorValues [0] > 850 && sensorValues[7] > 850) && banderaSensor1 == false && banderaInicioRuta == true)
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
      banderaInicioRuta = false;
      digitalWrite (ParoMotores, LOW);
     }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(posY); // comment this line out if you are using raw values
  
}

//Algoritmo de búsqueda
void lecturaCoordenadaX (bool ejeX) //eje X indicara la lectura de coordenada en dicho eje
{
  /*
   * ejeX = true indica que se sumara posX
   * ejeX = false indicara que se sumanra posY
   */
 uint16_t position = qtr.readLineWhite(sensorValues);
   
if ((sensorValues [0] > 850 && sensorValues[7] > 850) && banderaSensor1 == false)
     {
      (ejeX) ? posX++ : posX--;
      banderaSensor1 = true;
     }

     if ((sensorValues [0] < 250 && sensorValues[7] < 200))
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

     if ((sensorValues [0] < 250 && sensorValues[7] < 200))
     {
      banderaSensor1 = false;
     }
     if (posY == coordenadaPersonaY[pR])
     {
       digitalWrite (ParoMotores, LOW);
     }
     
}
void busquedaPrincipal (int valorIzquierdo, int valorDerecho){
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
   
   /*OrientacionX
    * Caso 1: El sensor está orientado hacia el Eje positivo Y
    * Csso 2: El sensor está orientado hacia el eje negativo Y
    */

    //NOTA ERNESTO, CUANDO TE DESPIERTES ENCUENTRA UNA FORMA DE AISLAR EL PID CUANDO SE ESTÉN REALIZANDO CALCULOS Y REACTIVARLO CUANDO HAYA ACABADO LA RUTINA DE VUELTA C:
  if ((posX  > coordenadaPersonaX [pR]) && banderaStart  == false)
  {
        //Realiza un giro a la izquierda
        arribaIzquierda ();
        
        // Manda un 1 a la variable indicando el caso que se habilito
        orientacionY = 1;
                                
        // se habilita una bandera para evitar conflicto entre la toma de decisiones 
        banderaStart = true;      
        

  } else if ((posX < coordenadaPersonaX [pR]) && banderaStart == false)
  {
    //Realiza un giro a la derecha
    arribaDerecha ();
      
    // manda un 1 a la variable indicando el caso que se habilito   
    orientacionY = 2;
    
   // se habilita una bandera para evitar conflicto entre la toma de decisiones 
    banderaStart = true;        
  }
  
  //ingresar switch case de posicion
  switch (orientacionY)
  {
    case 1:
      lecturaCoordenadaX (false);      
      // compara las Y y decide
      if (coordenadaPersonaX [pR] == posX && posY > coordenadaPersonaY [pR])
      {
        //Activa rutina de cambio de giro para acercarse al eje negativo de Y pero cuando el sensor está en orientacion hacia el eje negativo X
        izquierdaAbajo();
        orientacionX = 2;                
                
      } else if (coordenadaPersonaX [pR] == posX && posY < coordenadaPersonaY [pR])
      {
        //Activa rutina de cambio de giro para acercarse al eje positivo de Y pero cuando el sensor está en orientacion hacia el eje negativo X
        izquierdaArriba();
        orientacionX = 1;
        
                
      }

      if (coordenadaPersonaX [pR] == posX && coordenadaPersonaY[pR] == posY)
      {
        orientacionY = 0;
        banderaPr [pR] = true;
        pR++;     
      }
    break;

    case 2:
      //compara las Y y decide
      lecturaCoordenadaX (true);
        
      // compara las Y y decide
      if (coordenadaPersonaX [pR] == posX && posY > coordenadaPersonaY [pR])
      {
        //Activa rutina de cambio de giro para acercarse al eje negativo de Y pero cuando el sensor está en orientacion hacia el eje positivo X
        derechaAbajo();   
        orientacionX = 2;     

      } else if (coordenadaPersonaX [pR] == posX && posY < coordenadaPersonaY [pR])
      {
        //Activa rutina de cambio de giro para acercarse al eje positivo de Y pero cuando el sensor está en orientacion hacia el eje positivo X
        derechaArriba();
        orientacionX = 1;
      }
      if (coordenadaPersonaX [pR] == posX && coordenadaPersonaY[pR] == posY)
      {
        orientacionY = 0;
        banderaPr [pR] = true;
        pR++;     
      }
    break;

    default:
      //no ha pasado nada compa
    break;
  }

  if (banderaPr [pR-1] == true)
  {
      switch (orientacionX)
  {
    case 1:
    
    break;

    case 2:
      
    break;
  }
  }
  
  
}
