#include <Servo.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>


// QOL Function
String intToString(int number);

void servoControl(int ServoNo, int degree);
void smoothServoControl(int ServoNo, int degree);
void grip(bool stage);
void toggleGrip();
void setServo(int ServoNo, int degree, bool slient);
void InstructionHandle(String instruction);
void Instruction(String instructions);
void conveyor(int num, int way);
void piSerialMonitorStatus();
void piSerialDirect();
void busy(bool state);


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

unsigned long currentMillis = millis();


#define CONV_SA 3
#define CONV_SB 4
#define CONV_LA 5
#define CONV_LB 6

int sAngle[] = {0, 0, 0, 0, 0, 0};
int sLastAngle[] = {0, 0, 0, 0, 0, 0};
int Conveyor[] = {0, 0};
String sInstruction = "";
bool isBusy = false;
int timePerMove = 40;
int stepPerMove = 1;

int servoNumber = 0, servoAngle = 90, mode = 0;
unsigned long previousMillis = 0;
unsigned long pauseDuration = 0;

bool isPausing = false;

Servo myservo; // Declare Servo object
char inputBuffer[10]; // Buffer to store serial input
int bufferIndex = 0; // Index for input buffer

void setup() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); 
  pwm.setPWMFreq(50);  // Set the PWM frequency to 50 Hz (typical for servos)
  Serial.begin(115200); // Start serial communication at 115200 bps
  Serial.println("Ready!");
}

void loop() {
  piSerialDirect();
    if (isPausing && millis() - previousMillis >= pauseDuration) {
    isPausing = false;
  }
}


// Direct instruction will be in this format
//PW000S00D000S01D025S02D000S03D000S04D090S05D040C00C10
//PWXXX = Current step of work that is being doing (For now it will be 000)
//S00DXXXS01DXXXS02DXXXS03DXXXS04DXXXS05DXXX = Current servo position
//C0XC1X = Current conveyor position (0 = Stop, 1 = Forward, 2 = Backward)

void piSerialDirect() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input from serial
    if (input.startsWith("PB")){
      if(isBusy == false){
        Serial.println("POK");
      }else{
        Serial.println("PB");
      }
      return;
    }
    if(input.startsWith("PC")){
      piSerialMonitorStatus();
      return;
    }
    if(input.startsWith("PI")){
      busy(true);
      InstructionHandle(input.substring(2));
      busy(false);
      return;
    }
    if(input.startsWith("PX")){
      busy(true);
      Instruction(input.substring(2));
      busy(false);;
      return;
    }
    // PT000M000 (T = TimePerMove, M = StepPerMove)
    if(input.startsWith("PT")){
      // If no timePerMove and stepPerMove is specified, it will return the current value in the format PT000M000
      if(input.length() == 2){
        Serial.print("PT");
        Serial.print(intToString(timePerMove));
        Serial.print("M");
        Serial.println(intToString(stepPerMove));
      }
      else{
        timePerMove = input.substring(2, 5).toInt();
        stepPerMove = input.substring(6, 9).toInt();
        Serial.print("PT");
        Serial.print(intToString(timePerMove));
        Serial.print("M");
        Serial.println(intToString(stepPerMove));
      }
      return;
    }
    // Check if the input data length is valid
    if (input.length() >= 48) {
      int servoPositions[6];
      int conveyorPositions[2];
      bool useSmoothServo = false;
      busy(true);

      // If instruction starts with PF, it will use servoControl function
      // If instruction starts with PS, it will use smoothServoControl function

      if (input.startsWith("PF")) {
        Serial.println("[Direct] Using servoControl");
        useSmoothServo = false;
      }
      else if (input.startsWith("PS")) {
        Serial.println("[Direct] Using smoothServoControl");
        useSmoothServo = true;
      }
      else {
        Serial.println("[Direct] Invalid instruction: " + input);
        return;
      }
      
      // Extract servo positions
      //Ex: PS00D045S01D120S02D090S03D035S04D060S05D180C00C11
      String servoData[6];
      servoData[0] = input.substring(5, 8); //S00DXXX
      servoData[1] = input.substring(11, 15); //S01DXXX
      servoData[2] = input.substring(18, 22); //S02DXXX
      servoData[3] = input.substring(25, 29); //S03DXXX
      servoData[4] = input.substring(32, 36); //S04DXXX
      servoData[5] = input.substring(39, 43); //S05DXXX
      
      for (int i = 0; i < 6; i++) {
        servoData[i].trim(); // Remove leading/trailing spaces
        if (servoData[i].startsWith("D")) {
          servoData[i] = servoData[i].substring(1); // Skip the 'D' character
        }
        servoPositions[i] = servoData[i].toInt();
        Serial.println("[Direct] Servo " + String(i) + " at " + servoData[i]);
        if (useSmoothServo) {
          smoothServoControl(i, servoPositions[i]);
        }
        else {
          servoControl(i, servoPositions[i]);
        }
      }
      
      // Extract conveyor positions
      String conveyorData[2];
      conveyorData[0] = input.substring(43, 45);
      conveyorData[1] = input.substring(46, 48);
      
      for (int i = 0; i < 2; i++) {
        conveyorData[i].trim(); // Remove leading/trailing spaces
        if (conveyorData[i].startsWith("C")) {
          conveyorData[i] = conveyorData[i].substring(1); // Skip the 'C' character
        }
        conveyorPositions[i] = conveyorData[i].toInt();
        Serial.println("[Direct] Conveyor " + String(i) + " at " + conveyorData[i]);
        conveyor(i, conveyorPositions[i]);
      }
      
      busy(false);
      piSerialMonitorStatus();
    } else {
      Serial.println("[Direct] Invalid instruction length: " + input);
    }
  }
}


// Send current servo position to serial for python to read
// Will be in this format PW0000S00D000S01D025S02D000S03D000S04D090S05D040S06D000C00C10
//PWXXX = Current step of work that is being doing
//S00DXXXS01DXXXS02DXXXS03DXXXS04DXXXS05DXXXS06DXXX = Current servo position
//C0XC1X = Current conveyor position (0 = Stop, 1 = Forward, 2 = Backward)

void piSerialMonitorStatus(){
  String result = "";
  result += "P";
  for (int i = 0; i < 6; i++){
    result += "S0" + String(i) + "D" + intToString(sAngle[i]);
  }
  result += "C0" + String(Conveyor[0]) + "C1" + String(Conveyor[1]);
  Serial.println(result);
}

void busy(bool state){
  if (state == true){
    isBusy = true;
    Serial.println("PB");
  }
  else if (state == false){
    isBusy = false;
    Serial.println("POK");
  }
}

void servoControl(int ServoNo, int degree){
  if (degree > 180){
    degree = 180;
  }
  if (degree > 110 && ServoNo == 3) {
    degree = 110;
  }
  // servo[ServoNo].write(degree);
  setServo(ServoNo, degree, false);
  Serial.print("S ");
  Serial.print(ServoNo);
  Serial.print(" at: ");
  // Serial.println(servo[ServoNo].read());
  Serial.println(sAngle[ServoNo]); // New function to get servo angle
}


void smoothServoControl(int ServoNo, int targetDegree) {
  // int currentDegree = servo[ServoNo].read();
  int currentDegree = sAngle[ServoNo]; // New function to get servo angle

  if (targetDegree > 180) {
    targetDegree = 180;
  }

  if (currentDegree == targetDegree) {
    setServo(ServoNo, currentDegree, true);
    return;  // No need to move if already at the target degree
  }

  int direction = (currentDegree < targetDegree) ? 1 : -1;
  // Serial.print("S ");
  // Serial.print(ServoNo);
  // Serial.print(" at: ");
  while (currentDegree != targetDegree) {
    if (millis() - previousMillis >= timePerMove) {
      currentDegree += stepPerMove * direction;
      
      if ((direction == 1 && currentDegree > targetDegree) || (direction == -1 && currentDegree < targetDegree)) {
        currentDegree = targetDegree;
      }

        setServo(ServoNo, currentDegree, true);
        // Serial.print(sAngle[ServoNo]); // New function to get servo angle
        // Serial.print(" | ");
      previousMillis = millis();
    }
  }
  Serial.println("POK");

}


void grip(bool stage){
  // 0 = open
  // 1 = grip
  if (stage == false){
    servoControl(5, 40);
    sAngle[5] = 40;
  }
  else if (stage == true){
    servoControl(5, 90);
    sAngle[5] = 90;
  }
}


void toggleGrip(){
  if (sAngle[5] <= 40){
    sAngle[5] = 90;
  }
  else{
    sAngle[5] = 40;
  }
  servoControl(5, sAngle[5]);
}


void setServo(int ServoNo, int degree, bool slient = false){
  if (degree > 180){
    degree = 180;
  }
  if (degree > 110 && ServoNo == 3) {
    degree = 110;
  }
  
  int mappedPWM = map(degree, 0, 180, 102, 512);
  pwm.setPWM(ServoNo, 0, mappedPWM);
  
  sAngle[ServoNo] = degree;
  piSerialMonitorStatus();
  if (slient == false){
    Serial.print("S ");
    Serial.print(ServoNo);
    Serial.print(" at: ");
    Serial.println(sAngle[ServoNo]);
  }
}

//Array of Instruction Servo movement
// [array of instruction]
// [instruction will look like this]
// [S03D090] = Servo 3 to degree 90
// [P500] = Pause for 500ms
// [S03F090] = Force Servo 3 to degree 90
// [G] = Toggle Grip
// [GRIP] = Grip
// [OPEN] = Open
// [R] = Reset All Servo
// [END] = End of instruction array
void InstructionHandle(String instruction) {
  if (instruction.startsWith("S")) {
    int servoNo = instruction.substring(1, 3).toInt();
    int degree = instruction.substring(4, 7).toInt();
    smoothServoControl(servoNo, degree);
    Serial.println("[Instruction] Set Servo " + String(servoNo) + " to " + String(degree));
  }
  else if (instruction.startsWith("F")) {
    int servoNo = instruction.substring(1, 3).toInt();
    int degree = instruction.substring(4, 7).toInt();
    servoControl(servoNo, degree);
    Serial.println("[Instruction] Force Servo " + String(servoNo) + " to " + String(degree));
  }
  else if (instruction.startsWith("PS")) { //PSXXX
    int pauseTime = instruction.substring(2, 5).toInt();
    pauseDuration = pauseTime * 1000;
    Serial.println("[Instruction] Pause for " + String(pauseTime) + "s");
    previousMillis = millis();
    isPausing = true;
  }
  else if (instruction.startsWith("P")) { //PXXX
    int pauseTime = instruction.substring(1, 4).toInt();
    pauseDuration = pauseTime;
    Serial.println("[Instruction] Pause for " + String(pauseTime) + "ms");
    previousMillis = millis();
    isPausing = true;
  }
    else if (instruction.startsWith("DPS")) { //DPSXXX
    int pauseTime = instruction.substring(3, 6).toInt();
    Serial.println("[Instruction] [!] Delay Pause for " + String(pauseTime) + "s");
    Serial.println("[!] WARN: Don't use this in other then prepareServo] [!]");
    delay(pauseTime * 1000);
  }
  else if (instruction.startsWith("DP")) { //DPXXX
    int pauseTime = instruction.substring(2, 5).toInt();
    Serial.println("[Instruction] [!] Delay Pause for " + String(pauseTime) + "ms");
    Serial.println("[!] WARN: Don't use this in other then prepareServo] [!]");
    delay(pauseTime);
  }
  else if (instruction.startsWith("G")) {
    toggleGrip();
    Serial.println("[Instruction] Toggle Grip");
  }
  else if (instruction.startsWith("GRIP")) {
    grip(true);
    Serial.println("[Instruction] Grip");
  }
  else if (instruction.startsWith("OPEN")) {
    grip(false);
    Serial.println("[Instruction] Open");
  }
  else if (instruction.startsWith("COV")){
    int num = instruction.substring(3, 4).toInt();
    int way = instruction.substring(4, 5).toInt();

    conveyor(num, way);
    Serial.println("[Instruction] Conveyor Belt " + String(num) + " is now " + String(way));
    
  }
  else if (instruction.startsWith("END")) {
    Serial.println("[Instruction] End of instruction");
  }
  else {
    Serial.println("[Instruction] Invalid instruction " + instruction);
  }
}


void conveyor(int num, int way){
  if (num == 0){
    if (way == 0){
      digitalWrite(CONV_SA, LOW);
      digitalWrite(CONV_SB, LOW);
      Conveyor[0] = 0;
    }
    else if (way == 1){
      digitalWrite(CONV_SA, HIGH);
      digitalWrite(CONV_SB, LOW);
      Conveyor[0] = 1;
    }
    else if (way == 2){
      digitalWrite(CONV_SA, LOW);
      digitalWrite(CONV_SB, HIGH);
      Conveyor[0] = 2;
    }
  }
  else if (num == 1){
    if (way == 0){
      digitalWrite(CONV_LA, LOW);
      digitalWrite(CONV_LB, LOW);
      Conveyor[1] = 0;
    }
    else if (way == 1){
      digitalWrite(CONV_LA, HIGH);
      digitalWrite(CONV_LB, LOW);
      Conveyor[1] = 1;
    }
    else if (way == 2){
      digitalWrite(CONV_LA, LOW);
      digitalWrite(CONV_LB, HIGH);
      Conveyor[1] = 2;
    }
  }
}

void Instruction(String instructions) {
  int startIndex = 0;
  int endIndex = instructions.indexOf('>');

  while (endIndex != -1) {
    String singleInstruction = instructions.substring(startIndex, endIndex);
    InstructionHandle(singleInstruction);
    piSerialMonitorStatus();

    startIndex = endIndex + 1;
    endIndex = instructions.indexOf('>', startIndex);
  }
}


//Convert int to String with 3 digits
String intToString(int number){
  String result = "";
  if(number < 10){
    result = "00" + String(number);
  }
  else if(number < 100){
    result = "0" + String(number);
  }
  else{
    result = String(number);
  }
  return result;
}

