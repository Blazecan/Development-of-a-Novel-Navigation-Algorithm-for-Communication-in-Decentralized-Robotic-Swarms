// This example drives each motor on the Romi forward, then
// backward.  The yellow user LED is on when a motor should be
// running forward and off when a motor should be running
// backward.

#include <Romi32U4.h>

Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
int speedLeft = 0;
int speedRight = 0;
void updateMotors(){
  motors.setSpeeds(speedLeft, speedRight);
}

void setup() {

  //Serial.begin(115200);
  Serial1.begin(9600);            // initialize UART with baud rate of 9600
  //Serial.println("System Online");
  
}
void loop() {
  while (Serial1.available() > 0) {
    char receivedData = Serial1.read();   // read one byte from serial buffer and save to receivedData
    if (receivedData == '1' && speedRight < 400 && speedLeft < 400) {
      //Serial.println("ON");
      speedLeft+=10;
      speedRight+=10;
    }
    else if (receivedData == '2' && speedRight > -400 && speedLeft > -400) {
      //Serial.println("OFF");
      speedLeft-=10;
      speedRight -=10;
    }
    else if (receivedData == '3' && speedLeft < 400){
      speedLeft += 10;
    }
    else if(receivedData == '4' && speedRight < 400){
      speedRight +=10;
    }
    updateMotors();
  }
  delay(15);
}
