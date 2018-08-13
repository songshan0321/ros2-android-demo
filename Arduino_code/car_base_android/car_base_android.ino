// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

int incomingByte = 0;   // for incoming serial data

void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial.println("Setup serial port");

  // turn on motor
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);
 
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void forward(int vel, int t){
  Serial.print("Forward");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
  delay(t);
}

void backward(int vel, int t){
  Serial.print("Backward");
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
  delay(t);
}

void halt(int t){
  Serial.print("Stop");
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  delay(t);
}

void turnRight(int vel, int t){
  Serial.print("Turning Right");
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
  delay(t);
}

void turnLeft(int vel, int t){
  Serial.print("Turning Right");
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
  delay(t);
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read() - 48;
  
    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
//    Serial.write(char(incomingByte));/

    // If button 0 being pressed
    if (incomingByte == 0){
      forward(100,1000);
      halt(500);
      turnLeft(100,1500);
      halt(500);
      forward(100,1000);
      halt(500);
    }

    // If button 1 being pressed
    if (incomingByte == 1){
      forward(100,1000);
      halt(500);
      turnRight(100,1500);
      halt(500);
      forward(100,1000);
      halt(500);
    }
    
  }
}
