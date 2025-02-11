
#include <Servo.h>
int delayValue = 1000;
int delayValue1 = 1300;
Servo ESC; 
Servo ESC1;
Servo ESC2;
Servo ESC3;    


void setup() 
{
  Serial.begin(9600);
  ESC.attach(4);
  ESC1.attach(5);
  ESC2.attach(6);
  ESC3.attach(7); 
  ESC.writeMicroseconds(delayValue);
  ESC1.writeMicroseconds(delayValue);
  ESC2.writeMicroseconds(delayValue);
  ESC3.writeMicroseconds(delayValue);
  delay(2000);
}
void loop() {
    if (Serial.available() > 0) {
        int delayValue = Serial.parseInt();
        if (delayValue > 999) {
            ESC.writeMicroseconds(delayValue);
            ESC1.writeMicroseconds(delayValue);
            ESC2.writeMicroseconds(delayValue);
            ESC3.writeMicroseconds(delayValue);

            float speed = (delayValue - 1000) / 10.0;

            Serial.println();
            Serial.println("Motor speed:");
            Serial.print(" ");
            Serial.print(speed);
            Serial.println(" %");
        }
    }
}
