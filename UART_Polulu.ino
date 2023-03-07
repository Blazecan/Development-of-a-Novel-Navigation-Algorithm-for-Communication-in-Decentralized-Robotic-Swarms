// This example drives each motor on the Romi forward, then
// backward.  The yellow user LED is on when a motor should be
// running forward and off when a motor should be running
// backward.

#include <Romi32U4.h>

Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
int speed = 0;


void setup() {

  //Serial.begin(115200);
  Serial1.begin(9600);            // initialize UART with baud rate of 9600
  //Serial.println("System Online");
  
}
void loop() {
  while (Serial1.available() > 0) {
    char receivedData = Serial1.read();   // read one byte from serial buffer and save to receivedData
    if (receivedData == '1' && speed < 400) {
      //Serial.println("ON");
      speed+=10;
      ledGreen(0);
      ledYellow(1);
      motors.setSpeeds(speed, speed);
    }
    else if (receivedData == '2' && speed > -400) {
      //Serial.println("OFF");
      speed-=10;
      ledGreen(0);
      ledRed(1);
      motors.setSpeeds(speed, speed);
    }
    else{
      ledRed(0);
      ledYellow(0);
      ledGreen(1);
    }
  }
  delay(5);
}
