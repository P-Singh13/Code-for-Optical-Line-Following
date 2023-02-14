#include <Wire.h>
#include <MPU6050_tockn.h>

#define I2C_SLAVE_ADDR 0x04  // 4 in hexadecimal
#define TRIG_PIN 26          // ESP32 pin GIOP26 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 25          // ESP32 pin GIOP25 connected to Ultrasonic Sensor's ECHO pin
#define LED_White 14 
#define LED_Black 17 
#define IR_L1 2  
#define IR_L2 15   
#define IR_M1 35  
#define IR_M2 34  
#define IR_R1 32  
#define IR_R2 33  

// for Callibration
float max_L1, min_L1;
float max_L2, min_L2;
float max_M1, min_M1;
float max_M2, min_M2;
float max_R1, min_R1;
float max_R2, min_R2;

//Variable to store voltages from the IR Sensor
float voltage_L1;
float voltage_L2;
float voltage_M1;
float voltage_M2;
float voltage_R1;
float voltage_R2;

// //Variable to store voltages from the IR Sensor
// float sensorValue_L1;
// float sensorValue_L2;
// float sensorValue_M1;
// float sensorValue_M2;
// float sensorValue_R1;
// float sensorValue_R2;

//Variable for PID
float Xpk1, Xpk2;
float error, old_error, sum_error;
float Kp = 3.12;
float Ki = 1.6;
float Kd = 0.75;
float u;
float k = 0.6

MPU6050 mpu6050(Wire);
float duration_us, distance_cm;

int LMS;
int RMS;
int Angle;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(TRIG_PIN, OUTPUT);  // set ESP32 pin to output mode
  pinMode(ECHO_PIN, INPUT);   // set ESP32 pin to input mode
  pinMode(LED_Black, OUTPUT);
  pinMode(LED_White, OUTPUT);
  
  // put the car over the black line 
  //Getting max reading for IR sensor
  int max_l1 = analogRead(IR_L1);
  int max_l2 = analogRead(IR_L2);
  int max_m1 = analogRead(IR_M1);
  int max_m2 = analogRead(IR_M2);
  int max_r1 = analogRead(IR_R1);
  int max_r2 = analogRead(IR_R2);


  max_L1 = (max_l1 - 4095) * -1;
  max_L2 = (max_l2 - 4095) * -1;
  max_M1 = (max_m1 - 4095) * -1;
  max_M2 = (max_m2 - 4095) * -1;
  max_R1 = (max_r1 - 4095) * -1;
  max_R2 = (max_r2 - 4095) * -1;

  digitalWrite(LED_Black, HIGH);  // turn the LED on (HIGH is 3.3)
  delay(2000);                    // wait for a second
  digitalWrite(LED_Black, LOW);   // turn the LED on (LOW is 0)

  Serial.println();
  Serial.println("max voltage");
  Serial.print("max_L1 = ");
  Serial.print(max_L1);
  Serial.print("\tmax_L2 = ");
  Serial.print(max_L2);
  Serial.print("\tmax_M1 = ");
  Serial.print(max_M1);
  Serial.println("max_M2 = ");
  Serial.print(max_M2);
  Serial.print("\tmax_R1 = ");
  Serial.print(max_R1);
  Serial.print("\tmax_R2 = ");
  Serial.print(max_R2);
// put the car over white surface
  //Getting min reading for IR sensor
  int min_l1 = analogRead(IR_L1);
  int min_l2 = analogRead(IR_L2);
  int min_m1 = analogRead(IR_M1);
  int min_m2 = analogRead(IR_M2);
  int min_r1 = analogRead(IR_R1);
  int min_r2 = analogRead(IR_R2);

  min_L1 = (min_l1 - 4095) * -1;
  min_L2 = (min_l2 - 4095) * -1;
  min_M1 = (min_m1 - 4095) * -1;
  min_M2 = (min_m2 - 4095) * -1;
  min_R1 = (min_r1 - 4095) * -1;
  min_R2 = (min_r2 - 4095) * -1;

  digitalWrite(LED_White, HIGH);  // turn the LED on (HIGH is 3.3)
  delay(2000);                    // wait for a second
  digitalWrite(LED_White, LOW);   // turn the LED on (LOW is 0)

  Serial.println();
  Serial.println("mix voltage = ");
  Serial.print("min_L1 = ");
  Serial.print(min_L1);
  Serial.print("\tmin_L2 = ");
  Serial.print(min_L2);
  Serial.print("\tmin_M1 = ");
  Serial.println(min_M1);
  Serial.print("min_M2 = ");
  Serial.print(min_M2);
  Serial.print("\tmin_R1 = ");
  Serial.print(min_R1);
  Serial.print("\tmin_R2 = ");
  Serial.print(min_R2);
}
void Optical_Line_Following() {

  //Taking reading form IR sensor
  int sensorValue_L1 = analogRead(IR_L1);
  int sensorValue_L2 = analogRead(IR_L2);
  int sensorValue_M1 = analogRead(IR_M1);
  int sensorValue_M2 = analogRead(IR_M2);
  int sensorValue_R1 = analogRead(IR_R1);
  int sensorValue_R2 = analogRead(IR_R2);

  //converting sensorValue to voltage
  voltage_L1 = (sensorValue_L1 - 4095) * -1;
  voltage_L2 = (sensorValue_L2 - 4095) * -1;
  voltage_M1 = (sensorValue_M1 - 4095) * -1;
  voltage_M2 = (sensorValue_M2 - 4095) * -1;
  voltage_R1 = (sensorValue_R1 - 4095) * -1;
  voltage_R2 = (sensorValue_R2 - 4095) * -1;

  // voltage_L1 = map(voltage_L1, 2.03, 3.3, 3.3,  0);
  // voltage_L2 = map(voltage_L2, 2.2, 3.3, 3.3, 0);
  // voltage_M1 = map(voltage_M1, 1.78, 3.3, 3.3, 0);
  // voltage_M2 = map(voltage_M2, 2.1, 3.3, 3.3, 0);
  // voltage_R1 = map(voltage_R1, 2.27, 3.3, 3.3, 0);
  // voltage_R2 = map(voltage_R2, 1.6, 3.3, 3.3, 0);

  // calibration is being applied to the IR sensors
  voltage_L1 = map(sensorValue_L1, min_L1, max_L1, 0, 3.3);
  voltage_L2 = map(sensorValue_L2, min_L2, max_L2, 0, 3.3);
  voltage_M1 = map(sensorValue_M1, min_M1, max_M1, 0, 3.3);
  voltage_M2 = map(sensorValue_M2, min_M2, max_M2, 0, 3.3);
  voltage_R1 = map(sensorValue_R1, min_R1, max_R1, 0, 3.3);
  voltage_R2 = map(sensorValue_R2, min_R2, max_R2, 0, 3.3);

  // Voltage are constrained because in case the voltages is outside the range
  voltage_L1 = constrain(voltage_L1, 0, 3.3);
  voltage_L2 = constrain(voltage_L2, 0, 3.3);
  voltage_M1 = constrain(voltage_M1, 0, 3.3);
  voltage_M2 = constrain(voltage_M2, 0, 3.3);
  voltage_R1 = constrain(voltage_R1, 0, 3.3);
  voltage_R2 = constrain(voltage_R2, 0, 3.3);

  // print out the value you read:
  Serial.println("Taking voltage reading from IR sensors");
  Serial.println("---------------------------------------------------");
  Serial.print("Voltage for L1 = ");
  Serial.print(voltage_L1);

  Serial.print("\tVoltage for L2 = ");
  Serial.println(voltage_L2);

  Serial.print("Voltage for M1 = ");
  Serial.print(voltage_M1);

  Serial.print("\tVoltage for M2 = ");
  Serial.println(voltage_M2);

  Serial.print("Voltage for R1 = ");
  Serial.print(voltage_R1);

  Serial.print("\tVoltage for R2 = ");
  Serial.println(voltage_R2);
  Serial.println("---------------------------------------------------");
}

void loop() {
  //Weighted Average Formula
  Optical_Line_Following();
  Xpk1 = ((voltage_L1 * 12) + (voltage_L2 * 25) + (voltage_M1 * 40) + (voltage_M2 * 53) + (voltage_R1 * 68) + (voltage_R2 * 81)) / (voltage_L1 + voltage_L2 + voltage_M1 + voltage_M2 + voltage_R1 + voltage_R2);
  old_error = 50 - Xpk1;
  delay(100);
  Xpk2 = ((voltage_L1 * 12) + (voltage_L2 * 25) + (voltage_M1 * 40) + (voltage_M2 * 53) + (voltage_R1 * 68) + (voltage_R2 * 81)) / (voltage_L1 + voltage_L2 + voltage_M1 + voltage_M2 + voltage_R1 + voltage_R2);
  error = 50 - Xpk2;
  sum_error += error + old_error; // sum_error = sum_error + error
  u = (Kp * error) + (Ki * sum_error) + (Kd * (error - old_error));

  Serial.println("Calculation for PID control being carried out");
  Serial.println("---------------------------------------------------");
  Serial.print("Xp1 = ");
  Serial.println(Xpk1);
  Serial.print("Xpk2 = ");
  Serial.println(Xpk2);
  Serial.print("old error = ");
  Serial.println(old_error);
  Serial.print("new error = ");
  Serial.println(error);
  Serial.print("sum error = ");
  Serial.println(sum_error);
  Serial.print("u = ");
  Serial.println(u);
  Serial.println("---------------------------------------------------");
  
// BElOW CODE CAN BE UNCOMMENTED IF YOU WANT TO USE THE BASIC LINE FOLLOWING
// high value = black surfare and low value = white surface
//   if (voltage_M1 >= 2.8 && voltage_M2 >= 2.8) {
//     moveSteering_s();  // change the angle to 90 degrees
//     goforward();
//   } 
//   else if (voltage_M1 >= 2.8 || voltage_M2 >= 2.8) {
//     if (voltage_M1 <= 3.0 && voltage_L2 >= 3.0) {
//       moveSteering_cw1();  // change the servo anagle to 45 degrees
//       TurnAntiClockwise();
//     } 
//     else if (voltage_M2 >= 2.8 && voltage_R1 >= 2.8) {
//       moveSteering_acw1();
//       TurnAntiClockwise();
//     }
//   }
//   else if (voltage_L1 >= 2.8 || voltage_L2 >= 2.8){
//     moveSteering_cw2();  // change the servo anagle to 45 degrees
//   TurnAntiClockwise();
// }
// else if (voltage_R1 >= 2.8 || voltage_R2 >= 2.8) {
//   moveSteering_acw2();
//   TurnAntiClockwise();
// }
// PID Control last step
  Angle = 94 + u;
  LMS = 170 + (k*u);
  RMS = 170 + (k*u);
transmission();
}

void transmission() {
  int x = LMS;
  int y = RMS;
  int z = Angle;

  Wire.beginTransmission(I2C_SLAVE_ADDR);

  Wire.write((byte)((x & 0x0000FF00) >> 8));
  Wire.write((byte)(x & 0x000000FF));
  Wire.write((byte)((y & 0x0000FF00) >> 8));
  Wire.write((byte)(y & 0x000000FF));
  Wire.write((byte)((z & 0x0000FF00) >> 8));
  Wire.write((byte)(z & 0x000000FF));

  Wire.endTransmission();
}

int HC_Distance() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration_us = pulseIn(ECHO_PIN, HIGH);
  distance_cm = 0.017 * duration_us;

  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  return (distance_cm);
}

int MPU_angle() {
  mpu6050.update();
  Serial.println();
  Serial.print("MPU_angle = ");
  Serial.println(mpu6050.getAngleZ());
  return (mpu6050.getAngleZ());
}
void moveSteering_s() {
  LMS = 0;     // stop the left motor
  RMS = 0;     // stop the right motor
  Angle = 90;  // servo angle = 90
}
void moveSteering_cw1() {
  LMS = 0;     // stop the left motor
  RMS = 0;     // stop the right motor
  Angle = 70;  // servo angle = 45
}
void moveSteering_acw1() {
  LMS = 0;      // stop the left motor
  RMS = 0;      // stop the right motor
  Angle = 120;  // servo angle = 45
}

void moveSteering_cw2() {
  LMS = 0;     // stop the left motor
  RMS = 0;     // stop the right motor
  Angle = 45;  // servo angle = 45
}
void moveSteering_acw2() {
  LMS = 0;      // stop the left motor
  RMS = 0;      // stop the right motor
  Angle = 135;  // servo angle = 45
}

void goforward() {
  Angle = 94;
  LMS = 130 ;
  RMS = 170 ;
}
void gobackward() {
  LMS = -160;
  RMS = -160;
  Angle = 94;
}
void TurnClockwise() {
  LMS = 130;
  RMS = 145;
  Angle = 135;
}
void TurnAntiClockwise() {
  LMS = 130;
  RMS = 145;
  Angle = 45;
}