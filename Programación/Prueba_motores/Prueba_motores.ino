#define PWMotorA 3
#define MotorARetroceso 4
#define MotorADerecho 5
#define ParoMotores 6
#define MotorBDerecho 7
#define MotorBRetroceso 8
#define PWMotorB 9

bool bandera = false;
int tiempoVuelta = 300;
int tiempoAvance = 250;
velocidad = 110;
void setup() {
  pinMode (PWMotorA, OUTPUT);
  pinMode (MotorARetroceso, OUTPUT);
  pinMode (MotorADerecho, OUTPUT);
  pinMode (ParoMotores, OUTPUT);
  pinMode (MotorBDerecho, OUTPUT);
  pinMode (MotorBRetroceso, OUTPUT);
  pinMode (PWMotorB, OUTPUT);
  digitalWrite (ParoMotores, HIGH);
  delay (1500);
}

void loop() {
  if (bandera == false)
  {
    bandera = true;
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

}
