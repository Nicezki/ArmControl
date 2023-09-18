#include <Servo.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>


// QOL Function
String intToString(int number);

void servoControl(int ServoNo, int degree);
void serialMorniHandle();
void resetServo();
void smoothServoControl(int ServoNo, int degree, int interpolate, unsigned long delayTime);
void grip(bool stage);
void toggleGrip();
void setServo(int ServoNo, int degree, bool slient);
void InstructionHandle(String instruction);
void Instruction(String instructions);
void recordStep();
void resetStep();
void conveyor(int num, int way);
void piSerialMonitorHandle();
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
int timePerStep

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
  // serialMorniHandle();
  // piSerialMonitorHandle();
  piSerialDirect();
    if (isPausing && millis() - previousMillis >= pauseDuration) {
    isPausing = false;
  }
}

void piSerialMonitorHandle() {
  // If serial is available, read input
  //If input start with PI, it will be parsed as instruction
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input from serial
    if (input.startsWith("PB")){
      if(isBusy == false){
        Serial.println("POK");
      }else{
        Serial.println("PB");
      }
    }
    if (input.startsWith("PI")) {
      String instruction = input.substring(2);
      Instruction(instruction);
    }
    else {
      Serial.println("[PI] Invalid instruction " + input);
    }
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
          smoothServoControl(i, servoPositions[i], 1, 25);
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



// DEPRECATED from Version 1
// void serialMorniHandle() {
//   if (Serial.available() > 0) {
//     String input = Serial.readStringUntil('\n'); // Read input from serial
    
//     if (input.startsWith("S")) {
//       mode = 0; // Set mode to select servo
//       Serial.println("Selecting Servo, Type your servo number (0-5)");
//     }
//     else if (input.startsWith("D")) {
//       mode = 1; // Set mode to set angle
//       Serial.print("Selecting Degree, Enter your degree for servo ");
//       Serial.println(servoNumber);
//     }
//     else if (input.startsWith("F")) {
//       mode = 2; // Set mode to set angle
//       Serial.print("[Force] Selecting Degree, Enter your degree for servo ");
//       Serial.println(servoNumber);
//     }    
//     else if (input.startsWith("G")) {
//       // Switch Grip State
//       // [Toggle Grip] Now: GRIP

//       Serial.println("[Toggle Grip] Now: ");
//       toggleGrip();
//     }
//     else if (input.startsWith("A")) {
//       // Get servo angle
//       Serial.print("Servo ");
//       Serial.print(servoNumber);
//       Serial.print(" | Angle: ");
//       Serial.println(servoAngle);
//     }
//     else if (input.startsWith("R")) {
//       // Get servo angle
//       Serial.println("Reset All Servo ");
//       resetServo();

//     }else if (input.startsWith("X")){
//       // Record Step
//       recordStep();
//     }else if (input.startsWith("Y")){
//       // Reset Step
//       resetStep();
//     }else if (input.startsWith("[")){
//       //Minus current servo angle by 1
//       servoAngle = sAngle[servoNumber] - 1;
//       Serial.println("[-] Servo" + String(servoNumber) + " at: " + String(sAngle[servoNumber]));
//       smoothServoControl(servoNumber, servoAngle, 1, 25);

//     }else if (input.startsWith("]")){
//       //Plus current servo angle by 1
//       servoAngle = sAngle[servoNumber] + 1;
//       Serial.println("[+] Servo" + String(servoNumber) + " at: " + String(sAngle[servoNumber]));
//       smoothServoControl(servoNumber, servoAngle, 1, 25);
//     }else if (input.startsWith("O")){
//       //Plus current servo angle by 1
//       servoAngle = sAngle[servoNumber] + 10;
//       Serial.println("[+] Servo" + String(servoNumber) + " at: " + String(sAngle[servoNumber]));
//       smoothServoControl(servoNumber, servoAngle, 1, 25);
//     }else if (input.startsWith("P")){
//       //Plus current servo angle by 1
//       servoAngle = sAngle[servoNumber] - 10;
//       Serial.println("[+] Servo" + String(servoNumber) + " at: " + String(sAngle[servoNumber]));
//       smoothServoControl(servoNumber, servoAngle, 1, 25);
//     }else {
//       // Parse input
//       int value = input.toInt();
//       if (mode == 0) {
//         // Set servo number
//         servoNumber = value;
//         Serial.print("[Select] Now controlling servo ");
//         Serial.println(servoNumber);
//       }
//       else if (mode == 1) {
//         // Set servo angle
//         servoAngle = value;
//         Serial.print("[Set] Servo ");
//         Serial.print(servoNumber);
//         Serial.print(" is now set to ");
//         Serial.println(servoAngle);
//         smoothServoControl(servoNumber, servoAngle, 1, 25);
        
//       }
//       else if (mode == 2) {
//         // Set servo angle
//         servoAngle = value;
//         Serial.print("[Force] Servo ");
//         Serial.print(servoNumber);
//         Serial.print(" is now set to ");
//         Serial.println(servoAngle);
//         servoControl(servoNumber, servoAngle);
//       }
//     }
//   }
// }

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


void smoothServoControl(int ServoNo, int targetDegree, int step = 3, unsigned long delayTime = 40) {
  // int currentDegree = servo[ServoNo].read();
  int currentDegree = sAngle[ServoNo]; // New function to get servo angle

  if (targetDegree > 180) {
    targetDegree = 180;
  }

  if (targetDegree > 110 && ServoNo == 3) {
    targetDegree = 110;
  }

  if (currentDegree == targetDegree) {
    return;  // No need to move if already at the target degree
  }

  int direction = (currentDegree < targetDegree) ? 1 : -1;
  // Serial.print("S ");
  // Serial.print(ServoNo);
  // Serial.print(" at: ");
  while (currentDegree != targetDegree) {
    if (millis() - previousMillis >= delayTime) {
      currentDegree += step * direction;
      
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
  else if (instruction.startsWith("R")) {
    resetServo();
  }
  else if (instruction.startsWith("COV")){
    // Format COV<NUM><WAY>
    // NUM = 0 to 1 (0 = Short, 1 = Long)
    // WAY = 0 or 1 or 2
    // 0 = Stop
    // 1 = Forward
    // 2 = Backward
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

// //Handle Array of Instruction
// void Instruction(String instruction[]){
//   for (int i = 0; i < sizeof(instruction); i++){
//     InstructionHandle(instruction[i]);
//   }
// }





//OBSOLETE from Version 1
// void showCurrentPos(){
// // This will return current Pos of Servo 0 to 5 in this format
// //S00D000 S01D025 S02D000 ...
// //S<servoNo>D<degree>
//   String result = "";
//   for (int i = 0; i < 6; i++){
//     result += "S0" + String(i) + "D" + intToString(sAngle[i]) + " ";
//   }
//   Serial.println(result);
// }



//OBSOLETE from version 1
//This will compare current pos of servo with last step (sLastAngle)
//If it's different, it will record the step output like this
//If only one servo is different output will be like this
//S01D025
//If more than one servo is different output will be like this
//S01D025>S02D000>S03D090 (This mean servo 1, 2, 3 is different from last step)
// void recordStep(){
//   Serial.println("[Recording Step]");
//   String result = "";
//   for (int i = 0; i < 6; i++){
//     if (sAngle[i] != sLastAngle[i]){
//       result += "S0" + String(i) + "D" + intToString(sAngle[i]) + ">";
//     }
//   }
//   if (result != ""){
//     result = result.substring(0, result.length() - 1);
//     Serial.println(result);
//   }
//   for (int i = 0; i < 6; i++){
//     sLastAngle[i] = sAngle[i];
//   }

//   if (sInstruction = ""){
//     //Add all servo movement to sInstruction from S00 to S05
//     sInstruction += "S00D"+ intToString(sAngle[0]) + ">S01D"+ intToString(sAngle[1]) + ">S02D"+ intToString(sAngle[2]) + ">S03D"+ intToString(sAngle[3]) + ">S04D"+ intToString(sAngle[4]) + ">S05D"+ intToString(sAngle[5]) + ">";
//     Serial.println("[First Recorded: " + sInstruction + "]");
//   }else{
//     //Add only servo movement that is different from last step
//     if (result != ""){
//       Serial.println("[Recorded: " + result + "]");
//       Serial.println("[All step: " + sInstruction + "]");
//       sInstruction += ">" + result;
//     }else{
//       Serial.println("[No change]");
//     }
//   }
//   }


//OBSOLETE from version 1
// void resetStep(){
//   sInstruction = "";
//   Serial.println("[Reset Step]");
// }


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

//การเขียนคำสั่งแบบใหม่
// แต่ละคำสั่งคั่นด้วย > เช่น S01D090>S02D180 (ไม่มีเว้นวรรค)
// บังคับองศา Servo แบบราบรื่นทีละนิด ใช้ S<เลข Servo เช่น 01> D<องศา เช่น 090 เช่น S01D090
// บังคับองศา Servo แบบทันที ใช้ F<เลข Servo เช่น 01> D<องศา เช่น 090 เช่น F01D090
// หยุดชั่วคราวแบบกำหนดเวลา P<เวลา> เช่น 500 เช่น P00500 (หน่วงเวลา 500 มิลลิวินาที)
// เปิดปิด Gripper ใช้ G
// หยิบของใช้ GRIP
// ปล่อยของใช้ OPEN
// บังคับสายพานโดยคำสั่ง COV<เลขสายพาน เช่น 0 หรือ 1><ทิศทาง เช่น 0 หรือ 1 หรือ 2> เช่น COV00 (หยุด) COV01 (เดินไปข้างหน้า) COV02 (ถอยหลัง)
// รีเซ็ต Servo ไปตำแหน่งเริ่มใช้ R (ห้ามใช้ในฟังก์ชั่น resetServo เพราะจะเกิด Loop)
// จบคำสั่งใช้ END


// OBSOLETE from version 1
// void resetServo(){
//   Instruction("COV01>COV11>S00D080>S01D075>S02D080>S03D075>S04D000>S05D040>DPS001>END");
// }