#include <ESP32Servo.h>

// Negro = BASE = GPIO23
// Blanco = HOMBRO = GPIO22
// Violeta = CODO = GPIO 21
// Azul = MANO = GPIO 19

Servo ServoBase, ServoHombro, ServoCodo, ServoMano;   // Objeto servo
int basePin = 23;  // Pin de control del servo
int hombroPin = 22;
int codoPin = 21;
int manoPin = 19;

// Rango típico de pulsos para un servo SG90 (ajustable)
int minUs = 500;
int maxUs = 2400;

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

void setup() {
  Serial.begin(9600);
  delay(1000); // espera inicial para ver bien los mensajes

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

  ServoHombro.write(40);
  delay(150);
  ServoBase.write(0);
  delay(150);
  ServoCodo.write(180 - 60);
  delay(150);
  ServoMano.write(0);
  delay(150);

}


int posHombro = 0; 
int posBase = 0;
int posCodo = 180; // 180 - pos
int posMano = 0; // 0 es cerrado

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