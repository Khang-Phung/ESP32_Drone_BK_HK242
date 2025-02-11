
#include <Wire.h>
#include<Servo.h>
 
Servo ESC1, ESC2, ESC3, ESC4;




float Kp_roll, Ki_roll , Kd_roll, Kp_pitch, Ki_pitch, Kd_pitch;
float RateRoll_set, RatePitch_set ,RateYaw_set;
float Rate_Roll, Rate_Pitch ,Rate_Yaw;
float pre_error_roll, pre_error_pitch, pre_error_yaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
uint32_t LoopTimer;
float Input_RateRoll, Input_RatePitch, Input_RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
float error, pre_error, Kp, Ki, Kd, dt, integral_roll, integral_pitch;
float Input_motor1, Input_motor2, Input_motor3, Input_motor4;
int throttleValue = 1000;
int commandValue;

String inputString = ""; // Chuỗi nhận dữ liệu
bool stringComplete = false; // Kiểm tra chuỗi hoàn tất
unsigned long pretime;

// lấy giá trị và tính góc IMU
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  Rate_Roll=(float)GyroX/65.5;
  Rate_Pitch=(float)GyroY/65.5;
  Rate_Yaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}
void calibrateGyro(float &RateCalibrationRoll, float &RateCalibrationPitch, float &RateCalibrationYaw) {
  RateCalibrationRoll = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw = 0;

  for (int RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals(); // Hàm đọc tín hiệu gyro
    RateCalibrationRoll += Rate_Roll;
    RateCalibrationPitch += Rate_Pitch;
    RateCalibrationYaw += Rate_Yaw;
    delay(1);
  }
  
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
}

// Bộ lọc kalman
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

// Hàm PID classic
float computePID(float setPoint, float currentValue, float &pre_error, float Kp, float Ki, float Kd, float dt, float &integral) {
  float error = setPoint - currentValue;
  
  // Tỉ lệ (P)

  float proportional = Kp * error;

  // Tích phân (I)
  integral += error * dt;
  float integrative = Ki * integral;

  // Vi phân (D)
  float derivative = (error - pre_error) / dt;
  float differential = Kd * derivative;
  // Cập nhật lỗi trước cho lần tính toán tiếp theo
    pre_error = error;
  // Tổng đầu ra
  return proportional + integrative + differential;
}

// Hàm PID cách 2
float CalculatePID2(float setPoint, float currentValue, float &pre_error, float Kp, float Ki, float Kd, float dt, float &integral) {
  float error = setPoint - currentValue;
  
  // Tỉ lệ (P)

  float proportional = Kp * error;

  // Tích phân (I)
  float integrative = integral + (Ki * dt / 2) * (error + pre_error);
  integral = integrative;

  // Vi phân (D)
  float differential = (Kd / dt) * (error - pre_error);
  // Cập nhật lỗi trước cho lần tính toán tiếp theo
    pre_error = error;
  // Tổng đầu ra
  return proportional + integrative + differential;
}




// cấu hình MMA
void computeMotorInputs(
  float Throttle, 
  float Input_RateRoll, 
  float Input_RatePitch, 
  float Input_RateYaw
) {
  

/*    Motor 1 (M1): Phải-trước 5
     Motor 2 (M2): Phải-sau số 9
      Motor 3 (M3): Trái-sau số 6
       Motor 4 (M4): Trái-trước số 3

 
*/  

  Input_motor1 = Throttle + Input_RateRoll + Input_RatePitch + Input_RateYaw;
  Input_motor2 = Throttle + Input_RateRoll - Input_RatePitch - Input_RateYaw;
  Input_motor3 = Throttle - Input_RateRoll - Input_RatePitch + Input_RateYaw;
  Input_motor4 = Throttle - Input_RateRoll + Input_RatePitch - Input_RateYaw;
  /*Serial.println(Input_RateRoll);
  Serial.println(".........");*/ 
}




void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT); // Đèn L tích hợp trên pin 13
  digitalWrite(13, LOW);  // Tắt LED
  Serial.begin(9600);
  inputString.reserve(200);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.println("calibrateGyro!!!");
  calibrateGyro(RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw);
  LoopTimer=micros();
  Serial.println("calibrateGyro_Done!!!");
  ESC1.attach(5);
  ESC2.attach(9);
  ESC3.attach(6);
  ESC4.attach(3);
  ESC1.writeMicroseconds(throttleValue);
  ESC2.writeMicroseconds(throttleValue);
  ESC3.writeMicroseconds(throttleValue);
  ESC4.writeMicroseconds(throttleValue);
  Kp_roll = 3.9;
  Ki_roll = 0.1;
  Kd_roll = 0.55;
  Serial.println("calibrateESC!!!");
  delay(1000);
  Serial.println("setup done!!!");
  digitalWrite(13, HIGH); // Bật LED
}



void loop() {

  gyro_signals();
    Rate_Roll -= RateCalibrationRoll;
    Rate_Pitch -= RateCalibrationPitch;
    Rate_Yaw -= RateCalibrationYaw;

    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, Rate_Roll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, Rate_Pitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    unsigned long current_time = micros();
    dt = (current_time - LoopTimer) / 1e6; // Đổi thành giây
    LoopTimer = current_time;
    
  
    float output_roll = computePID(RateRoll_set, KalmanAngleRoll, pre_error_roll, Kp_roll, Ki_roll, Kd_roll, dt, integral_roll);
    
    float output_pitch = computePID(RatePitch_set, KalmanAnglePitch, pre_error_pitch, Kp_pitch, Ki_pitch, Kd_pitch, dt, integral_pitch);
   // float output_pitch = 0 ;
  computeMotorInputs(throttleValue, output_roll, output_pitch, 0); // 
  Input_motor1 = constrain(Input_motor1, 1000, 1650); 
  Input_motor2 = constrain(Input_motor2, 1000, 1650); 
  Input_motor3 = constrain(Input_motor3, 1000, 1650); 
  Input_motor4 = constrain(Input_motor4, 1000, 1650);
  if (millis()-pretime>10){
    Serial.print(Input_motor1);
  Serial.print(" ");

  Serial.print(Input_motor2);
  Serial.print(" ");

  Serial.print(Input_motor3);
  Serial.print(" ");
 
  Serial.println(Input_motor4); 

  Serial.print(Kp_roll);
  Serial.print(" ");
  Serial.print(Ki_roll);
  Serial.print(" ");
  Serial.println(Kd_roll);

  Serial.println(KalmanAngleRoll);
  Serial.println(KalmanAnglePitch);
  pretime= millis();
  }
  
  
  
  ESC1.writeMicroseconds(Input_motor1);
  ESC2.writeMicroseconds(Input_motor2);
  ESC3.writeMicroseconds(Input_motor3);
  ESC4.writeMicroseconds(Input_motor4);

  

   // Kiểm tra chuỗi hoàn tất
if (stringComplete) {
  parseInput(inputString);  // Phân tích dữ liệu nhận được
  if (commandValue == 10) {
    if (throttleValue < 1900) { throttleValue = throttleValue + 50; }
  } else if (commandValue == 11) {
    if (throttleValue > 1100) { throttleValue = throttleValue - 50; }
  } else if (commandValue == 12) {
    throttleValue = 1050; 

  } else if (commandValue == 13) {digitalWrite(13, LOW);  // Tắt LED
    calibrateGyro(RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw);
  LoopTimer=micros();
  Serial.println("calibrateGyro_Done!!!");
  ESC1.attach(5);
  ESC2.attach(9);
  ESC3.attach(6);
  ESC4.attach(3);
  throttleValue = 1000;
  ESC1.writeMicroseconds(throttleValue);
  ESC2.writeMicroseconds(throttleValue);
  ESC3.writeMicroseconds(throttleValue);
  ESC4.writeMicroseconds(throttleValue);
  Serial.println("calibrateESC!!!");
  delay(1000);
  digitalWrite(13, HIGH);  // Tắt LED
  }
  commandValue = 0;
  inputString = "";        // Reset chuỗi sau khi xử lý xong
  stringComplete = false;  // Đặt cờ chuỗi hoàn tất về false
}
  //Kp_roll = Kp;
  //Ki_roll = Ki;
  //Kd_roll = Kd;

  Kp_pitch = Kp;
  Ki_pitch = Ki;
  Kd_pitch = Kd;

  while (micros() - LoopTimer < 2500);
  LoopTimer=micros();
}

// Hàm nhận dữ liệu từ UART
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read(); // Đọc từng ký tự
    if (inChar == '\n') { // Kết thúc chuỗi với ký tự xuống dòng
      stringComplete = true;
      break;
    } else {
      inputString += inChar; // Thêm ký tự vào chuỗi
    }
  }
}

// Hàm phân tích chuỗi nhận được
void parseInput(String data) {
  if (data.startsWith("Command Value: ")) {
    commandValue = data.substring(15).toInt(); // Lấy giá trị Command
    Serial.print("Command updated: ");
    Serial.println(commandValue);
  } else if (data.startsWith("Kp value: ")) {
    Kp = data.substring(10).toFloat(); // Lấy giá trị Kp
   
  } else if (data.startsWith("Ki value: ")) {
    Ki = data.substring(10).toFloat(); // Lấy giá trị Ki
    
  } else if (data.startsWith("Kd value: ")) {
    Kd = data.substring(10).toFloat(); // Lấy giá trị Kd
   // Serial.print("Kd updated: ");
   // Serial.println(kd);
    }else if (data.startsWith("Throttle Value: ")) {
    int temp_throttleValue = data.substring(16).toInt(); // Lấy giá trị Throttle
    if (temp_throttleValue> 999 && temp_throttleValue < 1900){throttleValue=temp_throttleValue;}
  } else {
    Serial.println("Unknown data received!");
  }
}
