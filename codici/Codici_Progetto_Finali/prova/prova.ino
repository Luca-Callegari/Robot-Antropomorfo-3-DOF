#include <Servo.h>

// Creazione dei tre oggetti servo
Servo servo1;
Servo servo2;
Servo servo3;

// Definizione dei PIN
const int SERVO_PIN1 = 5; // Link 1 (Base)
const int SERVO_PIN2 = 6; // Link 2
const int SERVO_PIN3 = 9; // Link 3

void setup() {
  Serial.begin(9600);
  
  // Attach dei motori
  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);
  servo3.attach(SERVO_PIN3);

  // Posizionamento iniziale (0 gradi)
  servo3.write(0);
  servo1.write(0);
  servo2.write(0);
  
  pinMode(LED_BUILTIN, OUTPUT);

  delay(2000); 
}

void loop() {
  if (Serial.available() >= 4) {
    if (Serial.read() == 255) { // Cerco l'inizio del pacchetto
      int a1 = Serial.read();
      int a2 = Serial.read();
      int a3 = Serial.read();

      servo1.write(a1);
      servo2.write(a2);
      servo3.write(a3);
    }
  }
}