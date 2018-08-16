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

int sensorValue = 0;
int incomingByte = 0;   // for incoming serial data
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

void forward(int vel){
  Serial.print("Forward");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
}

void backward(int vel){
  Serial.print("Backward");
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
  Serial.print("Stop");
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void turnRight(int vel){
  Serial.print("Turning Right");
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
}

void turnLeft(int vel){
  Serial.print("Turning Left");
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(vel);
  motor2.setSpeed(vel);
  motor3.setSpeed(vel);
  motor4.setSpeed(vel);
}

void loop() {
  runEvery(1000){// Read in LDR value every 1000 millisecond
    sensorValue = map(analogRead(sensorPin),970,1017,0,100); // read the value from the sensor
    Serial.println(sensorValue); //prints the values coming from the sensor on the screen
  }
  // Change motor direction only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read() - 48;
  
    // say what you got:
//    Serial.print("I received: ");
//    Serial.println(incomingByte);
//    Serial.write(char(incomingByte));/

    // Update state & preState
    preState = state;
    state = incomingByte;
    // Check if state is within 0~4
    if(state < 0 || state > 4){
//      Serial.print("Invalid Input: ");
//      Serial.print(incomingByte);
    }

    // when state change, update movement
    if (state != preState){
      // stop
      if (state == 0){
        halt();
      }
      // Right
      if (state == 1){
        turnRight(120);
      }
      // Left
      if (state == 2){
        turnLeft(120);
      }
      // Forward
      if (state == 3){
        forward(80);
      }
      if (state == 4){
        backward(80);
      }
    }
    
  }
}
