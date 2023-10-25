#include <Servo.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define CONV_SA 3
#define CONV_SB 4
#define CONV_LA 5
#define CONV_LB 6
#define CONV_SS 9 //Pwm
#define CONV_LS 10 //Pwm

unsigned int instructionNumber = 0;


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
unsigned long currentMillis = millis();
void setServo(int ServoNo, int degree);
void setConveyor(int num, int way, int speed);

void setup() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); 
  pwm.setPWMFreq(50);  // Set the PWM frequency to 50 Hz (typical for servos)
  Serial.begin(115200); // Start serial communication at 115200 bps
  Serial.println("Ready!");
  pinMode(CONV_SA, OUTPUT);
  pinMode(CONV_SB, OUTPUT);
  pinMode(CONV_LA, OUTPUT);
  pinMode(CONV_LB, OUTPUT);
  pinMode(CONV_SS, OUTPUT);
  pinMode(CONV_LS, OUTPUT);
}


void loop() {
  // Format: 18018018018018018012551255
  // {servo0}{servo1}{servo2}{servo3}{servo4}{servo5}{conv0mode}{conv0speed}{conv1mode}{conv1speed}
  // Check 
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input from serial
    // Check if input digits are 26
    if (input.length() == 26) {
      // Check if input digits are numbers
      if (input.toInt() != 0) {
        // Check if input digits are in range
          // Set servo positions
          setServo(0, input.substring(0, 3).toInt());
          setServo(1, input.substring(3, 6).toInt());
          setServo(2, input.substring(6, 9).toInt());
          setServo(3, input.substring(9, 12).toInt());
          setServo(4, input.substring(12, 15).toInt());
          setServo(5, input.substring(15, 18).toInt());
          // Set conveyor positions and speeds
          setConveyor(0, input.substring(18, 19).toInt(), input.substring(19, 22).toInt());
          setConveyor(1, input.substring(22, 23).toInt(), input.substring(23, 26).toInt());
          Serial.println("INST" + String(instructionNumber));
          instructionNumber++;
      }else{
        Serial.println("FAILED_INVALID_NAN");
      }
    }else{
      Serial.println("FAILED_INVALID_LENGTH");
    }
  }
}

void setServo(int ServoNo, int degree){
  Serial.println("ServoNo: " + String(ServoNo) + " Degree: " + String(degree));
  int mappedPWM = map(degree, 0, 180, 102, 512);
  pwm.setPWM(ServoNo, 0, mappedPWM);
}

void setConveyor(int num, int way, int speed){
  Serial.println("Conveyor: " + String(num) + " Way: " + String(way) + " Speed: " + String(speed));
  if (num == 0){
    if (way == 0){
      digitalWrite(CONV_SA, LOW);
      digitalWrite(CONV_SB, LOW);
    }
    else if (way == 1){
      digitalWrite(CONV_SA, HIGH);
      digitalWrite(CONV_SB, LOW);
    }
    else if (way == 2){
      digitalWrite(CONV_SA, LOW);
      digitalWrite(CONV_SB, HIGH);
    }
  }
  else if (num == 1){
    if (way == 0){
      digitalWrite(CONV_LA, LOW);
      digitalWrite(CONV_LB, LOW);
    }
    else if (way == 1){
      digitalWrite(CONV_LA, HIGH);
      digitalWrite(CONV_LB, LOW);
    }
    else if (way == 2){
      digitalWrite(CONV_LA, LOW);
      digitalWrite(CONV_LB, HIGH);
    }
  }
  // Speed
  if (num == 0){
    analogWrite(CONV_SS, speed);
  }
  else if (num == 1){
    analogWrite(CONV_LS, speed);
  }
}