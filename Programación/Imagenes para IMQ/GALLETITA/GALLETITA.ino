//qtr4.0
#include <QTRSensors.h>
QTRSensors qtra;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


//Mapeo de pines

 
#define AIN1  5  // rigth  
#define AIN2  4 
#define PWMB  3 
#define PWMA  11 
#define BIN1  8// left
#define BIN2  7 
 #define STBY 6
#define LED     13     
#define BOTON   2
     


// Constantes para PID
float KP =  4;
float KD =  2;
float Ki =  0;
/*
float KP =  2;
float KD =  0.9;
float Ki =  0.006;
*/
// Regulación de la velocidad Máxima
int Velmax = 90;

// Data para intrgal 
int error1=0;
int error2=0;
int error3=0;
int error4=0;
int error5=0;
int error6=0;


// Función accionamiento motor izquierdo
void Motoriz(int value)
{
  if ( value >= 0 )
  {
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
  }
  else
  {
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
    value *= -1;
  }
  analogWrite(PWMB,value);
}

// Función accionamiento motor derecho
void Motorde(int value)
{  
  if ( value >= 0 )
  {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
  }
  else
  {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
    value *= -1;
  }    
  analogWrite(PWMA,value);
}



//Accionamiento de motores
void Motor(int left, int righ)
{ 
  Motoriz(left);
  Motorde(righ);
}



//función de freno
void freno(boolean left, boolean righ, int value)
{
  
  if ( left )
  {
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,HIGH);
    analogWrite (PWMB, value);
  }
  if ( righ )
  {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,HIGH);
    analogWrite (PWMA, value);
  }
}


void setup()
{
// Declaramos como salida los pines utilizados
  pinMode(LED   ,OUTPUT);
  pinMode(BIN2  ,OUTPUT);
 
  pinMode(BIN1  ,OUTPUT);
  pinMode(PWMB  ,OUTPUT);
  pinMode(AIN1  ,OUTPUT);
  pinMode(AIN2  ,OUTPUT);
  pinMode(PWMA  ,OUTPUT);
  pinMode(BOTON  ,INPUT);
    pinMode(  STBY ,OUTPUT);
     digitalWrite(STBY, HIGH); 
  TCCR2B = TCCR2B & B11111000 | B00000011;  
//qtr4.0
qtra.setTypeAnalog();
qtra.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
qtra.setEmitterPin(12);



// Calibracion 4.0

delay(3000);

for (uint16_t i = 0; i < 70; i++)
  { 
    digitalWrite(LED, HIGH); delay(20);
    qtra.calibrate();
    digitalWrite(LED, LOW);  delay(20);
  }
 delay(3000);
}


unsigned int position = 0; 

//declaraos variables para utilizar PID
int proporcional = 0;         // Proporcional
int integral = 0;           //Intrgral
int derivativo = 0;          // Derivativo
     



int diferencial = 0;   // Diferencia aplicada a los motores
int last_prop;         // Última valor del proporcional (utilizado para calcular la derivada del error)
int Target = 3500; // Setpoint (Como utilizamos 6 sensores, la línea debe estar entre 0 y 5000, por lo que el ideal es que esté en 2500)

void loop(){
   
 if (digitalRead(BOTON) == HIGH) { //////////// accionamos el seguidor ////////////////
    for(;;){ 
    if (digitalRead (BOTON) == HIGH){
      //////////// bucle infinito/////////////////////////
  //qtr4.0  
  uint16_t position = qtra.readLineWhite(sensorValues);
  proporcional = ((int)position) - 3500;


  if ( proporcional <= -Target )
  {
    Motorde(0);
    freno(true,false,255);
  }
  else if ( proporcional >= Target )
  {
    Motoriz(0);
    freno(false,true,255);
  }
  
  derivativo = proporcional - last_prop; 
  integral = error1+error2+error3+error4+error5+error6;
  last_prop = proporcional;
  

  error6=error5;
  error5=error4;  
  error4=error3;  
  error3=error2;
  error2=error1;
  error1=proporcional;

 
  int diferencial = ( proporcional * KP ) + ( derivativo * KD )+ (integral*Ki) ;
  
  if ( diferencial > Velmax ) diferencial = Velmax; 
  else if ( diferencial < -Velmax ) diferencial = -Velmax;

  ( diferencial < 0 ) ? 
    Motor(Velmax+diferencial, Velmax) : Motor(Velmax, Velmax-diferencial);
} else if (digitalRead (BOTON) == LOW)
  {
      freno (1,1,0);
  }
    }
    }
}
