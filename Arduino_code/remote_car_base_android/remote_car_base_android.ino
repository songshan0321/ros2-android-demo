// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>

#define runEvery(t) for (static uint16_t _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

const int sensorPin = A5;

String inString = ""; // string to hold the input
float linearFloat;
float angFloat;

int sensorValue = 0;
int preState = 0;
int state = 0; // 0:stop; 1:turning right; 2: turning left; 3: forward; 4: backward

void setup() {
  Serial.begin(115200);           // set up Serial library at 115200 bps
//  Serial.println("Setup serial port");

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

float filter_neg(float floatIn){
  if(floatIn < 0.0){
    return floatIn * -1;
  }
  else{
    return floatIn;
  }
}

void forward(float vel){
//  Serial.print("Forward");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
}

void backward(float vel){
//  Serial.print("Backward");
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
}

void halt(){
//  Serial.print("Stop");
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void turnRight(float vel){
//  Serial.print("Turning Right");
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
}

void turnLeft(float vel){
//  Serial.print("Turning Left");
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
}

void forwardRight(float linearVel,float deltaAngVel){
//  Serial.print("Forward Right");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(linearVel - deltaAngVel);
  motor2.setSpeed(linearVel + deltaAngVel);
  motor3.setSpeed(linearVel + deltaAngVel);
  motor4.setSpeed(linearVel - deltaAngVel);
}

void forwardLeft(float linearVel,float deltaAngVel){
//  Serial.print("Forward Left");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(linearVel + deltaAngVel);
  motor2.setSpeed(linearVel - deltaAngVel);
  motor3.setSpeed(linearVel - deltaAngVel);
  motor4.setSpeed(linearVel + deltaAngVel);
}

void backwardLeft(float linearVel,float deltaAngVel){
//  Serial.print("Backward Left");
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(linearVel + deltaAngVel);
  motor2.setSpeed(linearVel - deltaAngVel);
  motor3.setSpeed(linearVel - deltaAngVel);
  motor4.setSpeed(linearVel + deltaAngVel);
}

void backwardRight(float linearVel,float deltaAngVel){
//  Serial.print("Backward Right");
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(linearVel - deltaAngVel);
  motor2.setSpeed(linearVel + deltaAngVel);
  motor3.setSpeed(linearVel + deltaAngVel);
  motor4.setSpeed(linearVel - deltaAngVel);
}

void move_base(float linearVelIn,float angVelIn){
  
  // Convert Twist values into motor speed values
  float linearVel = filter_neg(linearVelIn * 200);
  float deltaAngVel = filter_neg(angVelIn * 150);
  
  if (linearVelIn > 0.0){
    if (angVelIn == 0.0){       // forward
      forward(linearVel);
    }
    else if (angVelIn > 0.0){   // forward left
      forwardLeft(linearVel,deltaAngVel);
    }
    else {                      // forward right
      forwardRight(linearVel,deltaAngVel);
    }
  }
  else if (linearVelIn < 0.0){
    if (angVelIn == 0.0){       // backward
      backward(linearVel);
    }
    else if (angVelIn > 0.0){   // backward left
      backwardLeft(linearVel,deltaAngVel);
    }
    else {                      // backward right
      backwardRight(linearVel,deltaAngVel);
    }
  }
  else{
    if (angVelIn == 0.0){       // stop
      halt();
    }
    else if (angVelIn > 0.0){   // turn left
      turnLeft(filter_neg(angVelIn * 200));
    }
    else {                      // turn right
      turnRight(filter_neg(angVelIn * 200));
    }
  }
}

void loop() {
  runEvery(500){// Read in LDR value every 1000 millisecond
    sensorValue = map(analogRead(sensorPin),970,1017,0,100); // read the value from the sensor
    Serial.println(sensorValue); //prints the values coming from the sensor on the screen
  }
  // Update motor base
  if (Serial.available() > 0) {
    int inChar = Serial.read();
    if (inChar == ','){
      linearFloat = inString.toFloat();
      
      inString = "";
    }
    else if (inChar == '\n'){
      angFloat = inString.toFloat();
      
      inString = "";
    }
    else{
      inString += (char)inChar;
    }
//    Serial.println(linearFloat,angFloat);
    move_base(linearFloat,angFloat);
  }
}
