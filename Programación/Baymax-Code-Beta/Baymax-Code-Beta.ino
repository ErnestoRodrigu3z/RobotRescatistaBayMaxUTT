/*
 * Universidad Tecnológica de Tijuana
 * Ingeniería en Mecatrónica
 * Robot Rescatista "Baymax"
 * Autor: TSU. Rodríguez Corona Ernesto
 * 7A
 */

//Libreria QTR proporcionada por el fabricante

#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//Definicion de variables constantes en el proyecto
#define velocidad 150
#define ledCalibracion 2
#define PWMotorA 3
#define MotorAIzquierdo 4
#define MotorADerecho 5
#define ParoMotores 6
#define MotorBDerecho 7
#define MotorBIzquierdo 8
#define PWMotorB 9
#define botonCalibracion 12
#define IR 13
int tiempoCalibracion = 20;

void setup() {
  Serial.begin (9600);
  //Salidas
  pinMode (ledCalibracion, OUTPUT);
  pinMode (PWMotorA, OUTPUT);
  pinMode (MotorAIzquierdo, OUTPUT);
  pinMode (MotorADerecho, OUTPUT);
  pinMode (ParoMotores, OUTPUT);
  pinMode (MotorBDerecho, OUTPUT);
  pinMode (MotorBIzquierdo, OUTPUT);
  pinMode (PWMotorB, OUTPUT);
  
  //Entradas 
  pinMode (botonCalibracion, INPUT);
 
  //Declaracion de sensores QTR-8A
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(IR);

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
  
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(ledCalibracion, LOW); // una vez terminado la calibracion el led de calibracion se apaga.
  
  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  
  Serial.println();
  Serial.println();
  delay(1000);
}

void motorAdelante ()
{
  
}

void motorRetroceder()
{
  
}

void giroDerecha ()
{
  
}

void giroIzquierda ()
{
  
}

void loop() {

}
