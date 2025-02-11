#include <Wire.h>

int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroX, GyroY, GyroZ;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
volatile float RatePitch, RateRoll, RateYaw;
volatile float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccZCalibration,AccYCalibration,AccXCalibration;
int RateCalibrationNumber;
int Sample_quantity = 5000;

// Variables to store the calibrated values
float calAccX, calAccY, calAccZ;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize the MPU6050 sensor
  Wire.beginTransmission(0x68); // MPU6050 address
  Wire.write(0x6B); // Power management register
  Wire.write(0x00); // Wake up MPU6050
  Wire.endTransmission(true);
  
  delay(100);
  
  // Calibration instructions
  Serial.println("Welcome to quadcopter IMU calibration");
  Serial.println("Place the quadcopter flat on a surface and press 'Space bar' and then 'Enter' to begin Gyro and AccZ calibration.");
  while (!Serial.available()) {}  // Wait for the user to press 'Enter'
  Serial.read();  // Clear the input buffer
  
  //For all axis calibration
  //calibrateGyro();
  //calibrateAcc();
 
  //Simple calibration
  calibrateGyroSimple();
  Serial.print(" GyroX: ");
  Serial.print(RateRoll);
  Serial.print(", GyroY: ");
  Serial.print(RatePitch);
  Serial.print(", GyroZ: ");
  Serial.print(RateYaw);
  Serial.print(", AccX: ");
  Serial.print(AccX);
  Serial.print(", AccY: ");
  Serial.print(AccY);
  Serial.print(", AccZ: ");
  Serial.println(AccZ);
  Serial.println(" Gyro and AccZ calibration done!");

//Gyro Calibrated Values
    Serial.println("");
  Serial.print("RateCalibrationRoll=");
  Serial.print(RateCalibrationRoll);
  Serial.println(";");
  Serial.print("RateCalibrationPitch=");
  Serial.print(RateCalibrationPitch);
  Serial.println(";");
  Serial.print("RateCalibrationYaw=");
  Serial.print(RateCalibrationYaw);
  Serial.println(";");
  Serial.print("AccXCalibration=");
  Serial.print(AccXCalibration);
  Serial.println(";");
  Serial.print("AccYCalibration=");
  Serial.print(AccYCalibration);
  Serial.println(";");
  Serial.print("AccZCalibration=");
  Serial.print(AccZCalibration);
  Serial.println(";");
  
  Serial.println("Press 'Space bar' and then 'Enter' to print Gyro and Acc continuosuly");
  while (!Serial.available()) {} 
  Serial.read();  

}
void loop() {



  gyro_signals();

RateRoll -= RateCalibrationRoll;
RatePitch -= RateCalibrationPitch;
RateYaw -= RateCalibrationYaw;

AccX -= AccXCalibration ;
AccY -= AccYCalibration ;
AccZ -= AccZCalibration;

 AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29; // Convert to degrees
 AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29; // Convert to degrees

  //Print the accelerometer and gyroscope values
  
  Serial.print("AccX: ");
  Serial.print(AccX);
  Serial.print(", AccY: ");
  Serial.print(AccY);
  Serial.print(", AccZ: ");
  Serial.print(AccZ);
  
  Serial.print(" | GyroX: ");
  Serial.print(RateRoll);
  Serial.print(", GyroY: ");
  Serial.print(RatePitch);
  Serial.print(", GyroZ: ");
  Serial.println(RateYaw);

  Serial.print(" | AngleRoll: ");
  Serial.print(AngleRoll);
  Serial.print(", AnglePitch: ");
  Serial.println(AnglePitch);
  

  delay(1); // Delay for 100ms before reading again
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); // Set accelerometer filters
  Wire.write(0x03);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Set gyroscope filters
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start reading accelerometer data
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6); // Request 6 bytes of accelerometer data
  AccXLSB = Wire.read() << 8 | Wire.read();
  AccYLSB = Wire.read() << 8 | Wire.read();
  AccZLSB = Wire.read() << 8 | Wire.read();

  // Set gyroscope data to read
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Set gyroscope settings
  Wire.write(0x08);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Read gyroscope data
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6); // Request 6 bytes of gyroscope data
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();
  
  // Convert raw gyro values to degrees per second (dps)
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  
  // Convert raw accelerometer values to g (acceleration due to gravity)
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  
  // // Calculate Roll and Pitch angles using accelerometer data
  // AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29; // Convert to degrees
  // AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29; // Convert to degrees
}


void calibrateAcc()
{
    //AccY calc
  Serial.println("Place the quadcopter on its Right side with nose forward and press 'Space bar' and then 'Enter'.");
  while (!Serial.available()) {}  // Wait for the user to press 'Enter'
  Serial.read();  // Clear the input buffer
  gyro_signals(); 
  Serial.print("AccX: ");
  Serial.print(AccX);
  Serial.print(", AccY: ");
  Serial.print(AccY);
  Serial.print(", AccZ: ");
  Serial.println(AccZ);
  AccYCalibration = AccY - 1;  
  
  //Acc X calc
  Serial.println("Place the quadcopter nose up and press 'Space bar' and then 'Enter'.");
  while (!Serial.available()) {}  // Wait for the user to press 'Enter'
  Serial.read();  // Clear the input buffer
  gyro_signals(); // Record X axis values
  Serial.print("AccX: ");
  Serial.print(AccX);
  Serial.print(", AccY: ");
  Serial.print(AccY);
  Serial.print(", AccZ: ");
  Serial.println(AccZ);
  AccXCalibration = AccX - 1;  
}


void calibrateGyro()
{
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 1000; RateCalibrationNumber++)
  {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    AccZCalibration +=  AccZ;

    delay(1);
  }
  RateCalibrationRoll /= 1000;
  RateCalibrationPitch /= 1000;
  RateCalibrationYaw /= 1000;
  AccZCalibration /=1000;
  AccZCalibration = AccZCalibration-1 ;
}

void calibrateGyroSimple()
{
    for (RateCalibrationNumber = 0; RateCalibrationNumber < Sample_quantity; RateCalibrationNumber++)
  {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    AccXCalibration +=  AccX;
    AccYCalibration +=  AccY;
    AccZCalibration +=  AccZ;

    delay(1);
  }
  RateCalibrationRoll /= Sample_quantity;
  RateCalibrationPitch /= Sample_quantity;
  RateCalibrationYaw /= Sample_quantity;
  AccXCalibration /=Sample_quantity;
  AccYCalibration /=Sample_quantity;
  AccZCalibration /=Sample_quantity;
  AccZCalibration = AccZCalibration-1 ;
}