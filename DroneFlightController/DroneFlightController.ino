#include <MadgwickAHRS.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <SPI.h>

#include "PID.h"

#define ESC_PIN1 2
#define ESC_PIN2 3
#define ESC_PIN3 4
#define ESC_PIN4 5

#define CE 9
#define CSN 10

Servo mtr1;
Servo mtr2;
Servo mtr3;
Servo mtr4;

RF24 Radio(CSN, CE);


//NRF24 Data
const byte Address[6] = "00001";


//IMU Data
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;

//PID Constent Values
PID_Solver RollPID;
PID_Solver PitchPID;
PID_Solver YawPID;

FConstant RollConst;
FConstant PitchConst;
FConstant YawConst;

//Time Data
unsigned long CurrentTime = 0;
unsigned long PreviousTime = 0;
float DeltaTime= 0;

//Filter Algorithm Values
const float Frequncy = 16.00f;

//Motors
float M1, M2, M3, M4;

//Remote Data
struct FRFData
{
  int Throttle;
  int Yaw;
  int Pitch;
  int Roll;
} RFData;

Madgwick Filter;

void InitRadio()
{
  Radio.begin();
  Radio.openReadingPipe(0, Address);
  Radio.setPALevel(RF24_PA_MAX);
  Radio.startListening();
  
  delay(200);
}

void InitMtrDriver()
{
  mtr1.attach(ESC_PIN1, 1000, 2000);
  mtr2.attach(ESC_PIN2, 1000, 2000);
  mtr3.attach(ESC_PIN3, 1000, 2000);
  mtr4.attach(ESC_PIN4, 1000, 2000);

  delay(200);
}

void InitIMU()
{
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); 
  
  delay(200);
}

void GetIMUValue()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

   Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}

void CleanIMUValues()
{
  GyroX *= DEG_TO_RAD;
  GyroY *= DEG_TO_RAD;
  GyroZ *= DEG_TO_RAD;

  float norm = sqrt(AccX*AccX + AccY*AccY + AccZ*AccZ);
  AccX /= norm;
  AccY /= norm;
  AccZ /= norm;
}

void InitPIDSolver()
{
  //RollConstants
  RollConst.Proportional  = 0.0f;
  RollConst.Integral      = 0.0f;
  RollConst.Derivative    = 0.0f;

  //PitchConstants
  PitchConst.Proportional = 0.0f;
  PitchConst.Integral     = 0.0f;
  PitchConst.Derivative   = 0.0f;

  //yawConstant
  YawConst.Proportional   = 0.0f;
  YawConst.Integral       = 0.0f;
  YawConst.Derivative     = 0.0f;

  RollPID.SetConstants(RollConst);
  PitchPID.SetConstants(PitchConst);
  YawPID.SetConstants(YawConst);
}

void setup() {

  Serial.begin(9600);

  Filter.begin(Frequncy);

  InitMtrDriver();

  InitRadio();

  InitIMU();

  InitPIDSolver();

}

void loop() {

  //TODO:
  //-ShutDown Control Signals for Motors based on Armed Booloen
  //-Get IMU Raw data
  //-Filter the IMU Raw Values
  //-Check for Radio Transsmision
  //-Calculate Target Rotations
  //-Set PID Target Rotation
  //-Calculate Drone Control Signals Using PID
  //-Send Control Signal to drones Moters

  PreviousTime = CurrentTime;
  CurrentTime = millis();
  DeltaTime = (CurrentTime - PreviousTime) / 1000.0f;

  RollPID.SetDeltaTime(DeltaTime);
  PitchPID.SetDeltaTime(DeltaTime);
  YawPID.SetDeltaTime(DeltaTime);
  
  //Get Radio Values
  if(Radio.available())
  {
    Radio.read(&RFData, sizeof(RFData));
  }

  RollPID.SetTargetValue(RFData.Roll);
  PitchPID.SetTargetValue(RFData.Pitch);
  YawPID.SetTargetValue(RFData.Yaw);

  CleanIMUValues();
  Filter.updateIMU(GyroX, GyroY, GyroZ, AccX, AccY, AccZ);

  RollPID.SetCurrentValue(Filter.getRoll());
  PitchPID.SetCurrentValue(Filter.getPitch());
  YawPID.SetCurrentValue(Filter.getYaw());

  RollPID.Solve();
  PitchPID.Solve();
  YawPID.Solve();

  //Calculate values for the motors and write to the motor Drivers
  
}
