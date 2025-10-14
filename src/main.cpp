#include <ESP32Servo.h>
#include <Arduino.h>
#include <math.h>
// Negro = BASE = GPIO23
// Blanco = HOMBRO = GPIO22
// Violeta = CODO = GPIO 21
// Azul = MANO = GPIO 19

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

bool within(double v, double lo, double hi){ return v >= lo && v <= hi; }

enum estados
{
  IDLE,
  MOVIENDO_INTERMEDIO,
  FIN_INTERMEDIO,
  MOVIENDO_FINAL,
  FIN_FINAL,
  LEVANTANDO
};

estados estado;

struct punto
{
  double x;
  double y;
  double z;
};

punto punto_objetivo;
punto punto_intermedio;


Servo ServoBase, ServoHombro, ServoCodo, ServoMano;   // Objeto servo
int basePin = 23;  // Pin de control del servo
int hombroPin = 22;
int codoPin = 21;
int manoPin = 19;

// Rango típico de pulsos para un servo SG90 (ajustable)
int minUs = 500;
int maxUs = 2400;

// Declaracion de los parametros en cm para la cinematica directa e inversa.
const double L_A2 = 8.05;
const double L_A3 = 8.0;
const double Herramienta_X = 5.4;
const double Herramienta_Z = -1.0;
const double L_Codo = L_A3 + Herramienta_X; // 13.4 = 8 + 5.4
const double S_X = 8.5;
const double S_Y = 5.0;
const double S_Z = 6.7;



// Posiciones iniciales
int posHombro = 50; // 0 para nuestra ref
int posBase = 90; // 0 para nuestra ref
int posCodo = 180 - 20; // 180 - pos; -90 para nuestra ref
int posMano = 0; // 0 es cerrado

// Parametros de calibracion.

const int signoCodo = sgn( ((-70) - (-90)) / (180 - 160) );
const int signoBase = 1;
const int signoHombro = -1;
const int signoMano = 1;

const int offsetBase = 90;
const int offsetHombro = 50;
const int offsetCodo = 160 - signoCodo * (-90);
const int offsetMano = 0;

// Rango maximo de movimiento.

// Base va desde -90 hasta 120
// Hombro va desde -90 hasta 45
// Codo depende de Hombro, en mejores condiciones (hombro -45) va desde -10 hasta -130
// en peor condicion (hombro 45) va desde -115 hasta -130
const double minBase = -90, maxBase = 120;
const double minHombro = -90, maxHombro = 45;
const double minCodo = -130, maxCodo = -10;



int smoothedBase = posBase;
int smoothedHombro = posHombro;
int smoothedCodo = posCodo;
int smoothedMano = posMano;

void moverGradual(Servo &servo, int &posActual, int posFinal, int pasoMax) {
  int delta = posFinal - posActual;
  if (abs(delta) > pasoMax) delta = pasoMax * sgn(delta);
  posActual += delta;
  servo.write(posActual);
}



const int servoMin = 0, servoMax = 180;
const bool codoRelativoHombro = true; 

// Funciones inline de utilidad (conversion de unidades, etc)
inline double deg2rad(double grados) { return grados * M_PI / 180.0; }
inline double rad2deg(double radianes) { return radianes * 180.0 / M_PI; }
inline double clampDouble(double v, double lo, double hi) {if(v < lo) return lo; else if(v > hi) return hi; else return v;}
inline int clampInt(int v, int lo, int hi) {if(v < lo) return lo; else if(v > hi) return hi; else return v;}


// Funcion para mover el servo a una velocidad controlada.
void moverSuave(Servo &servo, int &posActual, int posFinal, int velocidad) {
  if (posActual < posFinal) {
    for (int i = posActual; i <= posFinal; i++) {
      servo.write(i);
      delay(velocidad);
    }
  } else {
    for (int i = posActual; i >= posFinal; i--) {
      servo.write(i);
      delay(velocidad);
    }
  }
  posActual = posFinal; // actualizar posición real
}

// Funciones para convertir grados a servos y viceversa
int gradosAServo_small(double grados, int offset, int signo) {
  int angulo = int(round(signo * grados + offset));
  return clampInt(angulo, servoMin, servoMax);
}

double servoAGrados_small(int servo, int offset, int signo) {
  return signo * (servo - offset);
}

int gradosAServo(double grados, Servo &servo) {
  if (&servo == &ServoBase) return gradosAServo_small(grados, offsetBase, signoBase);
  if (&servo == &ServoHombro) return gradosAServo_small(grados, offsetHombro, signoHombro);
  if (&servo == &ServoMano) return gradosAServo_small(grados, offsetMano, signoMano);
  if (&servo == &ServoCodo) {
    double gradosCodo = grados;
    if(codoRelativoHombro)
    {
      double hombroGradosActual = servoAGrados_small(posHombro, offsetHombro, signoHombro);
      gradosCodo = grados + hombroGradosActual;
    }
    return gradosAServo_small(gradosCodo, offsetCodo, signoCodo);
  }
  return 90; // valor por defecto
}

double servoAGrados(int valorServo, Servo &servo) {
  if (&servo == &ServoBase) return servoAGrados_small(valorServo, offsetBase, signoBase);
  if (&servo == &ServoHombro) return servoAGrados_small(valorServo, offsetHombro, signoHombro);
  if (&servo == &ServoMano) return servoAGrados_small(valorServo, offsetMano, signoMano);
  if (&servo == &ServoCodo) {
    double codoGrados = servoAGrados_small(valorServo, offsetCodo, signoCodo);
    if(codoRelativoHombro)
    {
      double hombroGradosActual = servoAGrados_small(posHombro, offsetHombro, signoHombro);
      codoGrados = codoGrados - hombroGradosActual;
    }
    return codoGrados;
  }
  return 0; // valor por defecto
}

// Cinematica directa, la entrada es en radianes y la salida es x,y,z en cm.
void cinematicaDirecta(double a, double b, double c, double &x, double &y, double &z){
  double d = -c-b;

  double ca = cos(a), sa = sin(a);
  double cb = cos(b), sb = sin(b); 
  double cc = cos(c), sc = sin(c); 
  double cd = cos(d), sd = sin(d);

  double temp = (-sd - 5.4 * cd)*(-cb*cc +sb*sc) 
  + (5.4*sd-cd)*(-sb*cc - cb*sc) 
  - 8.0*sb*cc - 8*cb*sc - 8.05*sb;
 
  x = 8.5 + ca * temp; 
  y = 5 + sa * temp;

  temp = (-sd -5.4*cd)*(-sb*cc-cb*sc) 
  + (5.4*sd - cd)*(-sb*sc + cb*cc) 
  - 8*sb*sc + 8*cb*cc;

  z = temp + 8.05*cb + 6.7;
}

// Cinematica inversa.
// Utilizaremos el Jacobiano transpuesto y la funcion de cinematica directa.
// Entrada x,y,z.
// Salidas a,b,c,d en radianes. Existe la opcion de dar una guess inicial.
// Devuelve true si converge, false si no converge.
bool cinematicaInversa(double xT, double yT, double zT,
                      double &a, double &b, double &c, double &d,
                      double aInitial=deg2rad(15.0), double bInitial=deg2rad(-22.5), double cInitial=deg2rad(-70.0),
                      int maxIter = 800, double tol = 0.1 /*cm*/, double alpha = 0.3){
  // Inicializacion.
  //a = atan((yT - 5)/(xT - 8.5));
  a = aInitial;
  b = bInitial; c = cInitial; d = -c-b;
  const double h = 1e-6; // paso para diferencias finitas.
  double prevError = 1e9;

  

  for(int iter = 0; iter < maxIter; iter++)
  {
    double xC, yC, zC;
    cinematicaDirecta(a, b, c, xC, yC, zC);

    // Calculo del error.
    double ex = xT - xC;
    double ey = yT - yC;
    double ez = zT - zC;
    double error = sqrt(ex*ex + ey*ey + ez*ez);
    if(error < tol) 
    {
      d = -b - c;
      return true; // Convergio.
    }

    // Calculo del Jacobiano por diferencias finitas.
    double J[3][3];
    double fx = xC, fy = yC, fz = zC;
    // Perturbacion en a
    double tx, ty, tz;
    cinematicaDirecta(a+h, b, c, tx, ty, tz);
    J[0][0] = (tx - fx) / h;
    J[1][0] = (ty - fy) / h;
    J[2][0] = (tz - fz) / h;
    // Perturbacion en b
    cinematicaDirecta(a, b+h, c, tx, ty, tz);
    J[0][1] = (tx - fx) / h;
    J[1][1] = (ty - fy) / h;
    J[2][1] = (tz - fz) / h;
    // Perturbacion en c
    cinematicaDirecta(a, b, c+h, tx, ty, tz);
    J[0][2] = (tx - fx) / h;
    J[1][2] = (ty - fy) / h;
    J[2][2] = (tz - fz) / h;

    // Jacobiano traspuesto * error -> delta angulo.
    double da = alpha * (J[0][0]*ex + J[1][0]*ey + J[2][0]*ez);
    double db = alpha * (J[0][1]*ex + J[1][1]*ey + J[2][1]*ez);
    double dc = alpha * (J[0][2]*ex + J[1][2]*ey + J[2][2]*ez);

    // Actualizacion de angulos.
    const double maxStep = deg2rad(8.0); // maximo paso de 8 grados.
    if(da > maxStep) da = maxStep; else if(da < -maxStep) da = -maxStep;
    if(db > maxStep) db = maxStep; else if(db < -maxStep) db = -maxStep;
    if(dc > maxStep) dc = maxStep; else if(dc < -maxStep) dc = -maxStep;

    a += da;
    b += db;
    c += dc;

    // Si el error empeora, reducir alpha.
    if(error >= prevError)
    { 
      alpha *= 0.6;
      if(alpha < 1e-3) alpha = 1e-3; // minimo alpha.
    } else {
      alpha = min(alpha * 1.1, 1.0); // aumentar alpha.
    }
    prevError = error;
  }

  return false; // No convergio.
 }



// Init.
void setup() {
  Serial.begin(9600);
  delay(1000); // espera inicial para ver bien los mensajes

  analogSetAttenuation(ADC_11db);

  // Habilita timers de PWM del ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  Serial.println("Iniciando servo...");

  // Configura la señal de 50Hz para servo
  ServoBase.setPeriodHertz(50);
  ServoCodo.setPeriodHertz(50);
  ServoHombro.setPeriodHertz(50);
  ServoMano.setPeriodHertz(50);

  // Intentar hacer attach al pin
  int result = ServoBase.attach(basePin, minUs, maxUs);

  if (result) {
    Serial.print("Servo de BASE adjuntado correctamente al pin ");
    Serial.println(basePin);
  } else {
    Serial.print("ERROR: No se pudo adjuntar el servo de BASE al pin ");
    Serial.println(basePin);
  }

  result = ServoCodo.attach(codoPin, minUs, maxUs);

  if (result) {
    Serial.print("Servo de CODO adjuntado correctamente al pin ");
    Serial.println(codoPin);
  } else {
    Serial.print("ERROR: No se pudo adjuntar el servo de CODO al pin ");
    Serial.println(codoPin);
  }

  result = ServoHombro.attach(hombroPin, minUs, maxUs);

  if (result) {
    Serial.print("Servo de HOMBRO adjuntado correctamente al pin ");
    Serial.println(hombroPin);
  } else {
    Serial.print("ERROR: No se pudo adjuntar el servo de HOMBRO al pin ");
    Serial.println(hombroPin);
  }

  result = ServoMano.attach(manoPin, minUs, maxUs);

  if (result) {
    Serial.print("Servo de MANO adjuntado correctamente al pin ");
    Serial.println(manoPin);
  } else {
    Serial.print("ERROR: No se pudo adjuntar el servo de MANO al pin ");
    Serial.println(manoPin);
  }


  // Mover a la posicion inicial

  estado = IDLE;

  ServoHombro.write(posHombro);
  delay(1500);
  ServoBase.write(posBase);
  delay(1500);
  ServoCodo.write(posCodo);
  delay(1500);
  ServoMano.write(posMano);
  delay(1500);

  // INVESTIGAR PID
  punto_objetivo.x = 19.5;
  punto_objetivo.y = 13;
  punto_objetivo.z = 0.5;

  punto_intermedio.x = punto_objetivo.x - 5;
  punto_intermedio.y = punto_objetivo.y - 5;
  punto_intermedio.z = 4;

  Serial.print("Punto Intermedio, X: "); Serial.println(punto_intermedio.x);
  Serial.print("Punto Intermedio, Y: "); Serial.println(punto_intermedio.y);
  Serial.print("Punto Intermedio, Z: "); Serial.println(punto_intermedio.z);

  estado = MOVIENDO_INTERMEDIO;
}

bool trigger = false;

// Nico: No bajes el delay a menos de 15 seg que se quema.
void loop() {
  if (!ServoBase.attached()) {
    Serial.println("Servo BASE no está adjunto, revisa conexiones/pin.");
    delay(2000);
    return;
  }

  if (!ServoHombro.attached()) {
    Serial.println("Servo HOMBRO no está adjunto, revisa conexiones/pin.");
    delay(2000);
    return;
  }

  if (!ServoCodo.attached()) {
    Serial.println("Servo CODO no está adjunto, revisa conexiones/pin.");
    delay(2000);
    return;
  }

  if (!ServoMano.attached()) {
    Serial.println("Servo MANO no está adjunto, revisa conexiones/pin.");
    delay(2000);
    return;
  }

  // moverSuave(ServoBase, posBase, 0, 30); // Aprox -90 


  // 30 -45 -45
  /*moverSuave(ServoBase, posBase, 90 + 30, 30);
  moverSuave(ServoHombro, posHombro, 50 + 45, 30);
  moverSuave(ServoCodo, posCodo, 180 -20, 30);

  Serial.println("Angulos: ");
  Serial.println("Base: 30"); 
  Serial.println("Hombro: -45");
  Serial.println("Codo: -45");

  double x, y, z;
  cinematicaDirecta(deg2rad(30), deg2rad(-45), deg2rad(-45), x, y, z);
  Serial.println("Posicion:");
  Serial.print("X: "); Serial.println(x);
  Serial.print("Y: "); Serial.println(y);
  Serial.print("Z: "); Serial.println(z);*/

  // x y z
  // 15 16 1
  // 59.84, -65.87, -113.88

  // 10, 8, 10
  // 63.43 97.77 -144.82
  double a,b,c,d;
  bool exito = false;

  switch (estado)
  {
  case FIN_INTERMEDIO:
    estado = MOVIENDO_FINAL;
    delay(20);
    break;

  case MOVIENDO_INTERMEDIO:
    exito = cinematicaInversa(punto_intermedio.x, punto_intermedio.y, punto_intermedio.z,a,b,c,d);
    if(exito){
    Serial.print("Angulos (grados): ");
    Serial.print(rad2deg(a)); Serial.print(", ");
    Serial.print(rad2deg(b)); Serial.print(", ");
    Serial.print(rad2deg(c)); Serial.print(", ");
    Serial.println(rad2deg(d));
    } else {
    Serial.println("No convergio.");
    }

    if(exito)
    {
      moverSuave(ServoBase, posBase, gradosAServo(rad2deg(a), ServoBase), 20);
      moverSuave(ServoHombro, posHombro,  gradosAServo(rad2deg(b), ServoHombro), 20);
      moverSuave(ServoCodo, posCodo, gradosAServo(rad2deg(c), ServoCodo), 20);
      moverSuave(ServoMano, posMano, 130, 20);

      if( abs(posBase -  gradosAServo(rad2deg(a), ServoBase)) <= 1 &&
          abs(posHombro -  gradosAServo(rad2deg(b), ServoHombro)) <= 1 &&
          abs(posCodo - gradosAServo(rad2deg(c), ServoCodo)) <= 1)
          {
            estado = FIN_INTERMEDIO;
          }
      delay(40);
    }

    break;

    case MOVIENDO_FINAL:
      exito = cinematicaInversa(punto_objetivo.x, punto_objetivo.y, punto_objetivo.z,a,b,c,d);
      if(exito){
      Serial.print("Angulos (grados): ");
      Serial.print(rad2deg(a)); Serial.print(", ");
      Serial.print(rad2deg(b)); Serial.print(", ");
      Serial.print(rad2deg(c)); Serial.print(", ");
      Serial.println(rad2deg(d));
    } else {
      Serial.println("No convergio.");
    }

    if(exito)
    {
      moverGradual(ServoBase, posBase, gradosAServo(rad2deg(a), ServoBase), 1);
      moverGradual(ServoHombro, posHombro,  gradosAServo(rad2deg(b), ServoHombro), 1);
      moverGradual(ServoCodo, posCodo, gradosAServo(rad2deg(c), ServoCodo), 1);
      if( abs(posBase -  gradosAServo(rad2deg(a), ServoBase)) <= 1 &&
          abs(posHombro -  gradosAServo(rad2deg(b), ServoHombro)) <= 1 &&
          abs(posCodo - gradosAServo(rad2deg(c), ServoCodo)) <= 1)
          {
            estado = FIN_FINAL;
          }
      delay(40);
    }
    break;

  case FIN_FINAL:
    moverGradual(ServoMano, posMano, 0, 1);
    if(abs(posMano - 0) <= 1)
    {
      estado = LEVANTANDO;
    }
    delay(20);
    break;

  case LEVANTANDO:
    moverGradual(ServoHombro, posHombro, 50, 1);
    moverGradual(ServoCodo, posCodo, offsetCodo, 1);
    delay(20);
    break;
  }




/*
  if(!trigger && exito){
  Serial.println("Moviendo base"); Serial.println(gradosAServo(rad2deg(a), ServoBase));
  moverSuave(ServoBase, posBase, gradosAServo(rad2deg(a), ServoBase), 30);
  delay(2000);
  Serial.println("Moviendo Hombro");  Serial.println(gradosAServo(rad2deg(b), ServoHombro));
  moverSuave(ServoHombro, posHombro, gradosAServo(rad2deg(b), ServoHombro), 30);
  delay(2000);
  Serial.println("Moviendo Codo");  Serial.println(gradosAServo(rad2deg(c), ServoCodo));
  moverSuave(ServoCodo, posCodo,gradosAServo(rad2deg(c), ServoCodo), 30);
  delay(2000);
  moverSuave(ServoMano, posMano, 30, 30);
  trigger=true;
  }
*/




  /*
  moverSuave(ServoBase, posBase, 90 + 35, 30); // 35 grados
  moverSuave(ServoHombro, posHombro, 50 + 45, 30); // -45 grados
  moverSuave(ServoCodo, posCodo, 180 - 20 - 45, 30); // - 90 grados

  Serial.println("Angulos: ");
  Serial.println("Base: 35"); 
  Serial.println("Hombro: -45");
  Serial.println("Codo: -90");

  double x, y, z;
  cinematicaDirecta(deg2rad(35), deg2rad(-45), deg2rad(-90), x, y, z);
  Serial.println("Posicion:");
  Serial.print("X: "); Serial.println(x);
  Serial.print("Y: "); Serial.println(y);
  Serial.print("Z: "); Serial.println(z);

  Serial.println("Cinematica inversa:");
  double a, b, c, d;
  bool exito = cinematicaInversa(x, y, z, a, b, c, d);
  if(exito){
    Serial.print("Angulos (grados): ");
    Serial.print(rad2deg(a)); Serial.print(", ");
    Serial.print(rad2deg(b)); Serial.print(", ");
    Serial.print(rad2deg(c)); Serial.print(", ");
    Serial.println(rad2deg(d));
  } else {
    Serial.println("No convergio.");
  }

  delay(15000); // espera 15 segundos
  */


  /*Serial.println("Moviendo servo de 0° a 180°...");
  for (int pos = 0; pos <= 180; pos++) {
    myServo.write(pos);
    delay(15);
  }

  Serial.println("Moviendo servo de 180° a 0°...");
  for (int pos = 180; pos >= 0; pos--) {
    myServo.write(pos);
    delay(15);
  }*/

  //moverSuave(ServoCodo, posCodo, 180 - 70, 30);

}