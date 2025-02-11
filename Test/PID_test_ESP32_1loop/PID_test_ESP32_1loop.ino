#include <WiFi.h>
#include <ESP32Servo.h>  // Change to the standard Servo library for ESP32
#include <Wire.h>
// Khai báo thông tin Access Point
const char *ssid = "ESP32_AccessPoint";
const char *password = "12345678";

// Trạng thái LED
bool ledState = false;
// Thời gian của bộ điều khiển
uint32_t LoopTimer;
// Thời gian để định thời tuỳ chọn
unsigned long pretime;



// đại lượng đo về từ IMU
float Rate_Roll, Rate_Pitch, Rate_Yaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
// dùng cho bộ lọc
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };
// dùng cho PID
float error, pre_error, Kp, Ki, Kd, dt, integral_roll, integral_pitch, integral_roll_gyro, integral_pitch_gyro;
float Kp_roll, Ki_roll, Kd_roll, Kp_pitch, Ki_pitch, Kd_pitch;
float Kp_roll_gyro, Ki_roll_gyro, Kd_roll_gyro, Kp_pitch_gyro, Ki_pitch_gyro, Kd_pitch_gyro;
float RateRoll_set, RatePitch_set, RateYaw_set;
float Roll_set, Pitch_set, Yaw_set;
float output_roll, output_pitch, output_roll_rate, output_pitch_rate;

float pre_error_roll, pre_error_pitch, pre_error_yaw;
float pre_error_roll_gyro, pre_error_pitch_gyro, pre_error_yaw_gyro;

// dùng cho MMA
float Input_RateRoll, Input_RatePitch, Input_RateYaw;
float Input_motor1, Input_motor2, Input_motor3, Input_motor4;
int throttleValue = 1000;
volatile char commandValue;
const int mot1_pin = 12;
const int mot2_pin = 27;
const int mot3_pin = 14;  //is 14 for some designed FC on perforated baords
const int mot4_pin = 13;
int ESCfreq = 250;
Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;



// HTML của trang web
String generateHTML() {
  String html = "<!DOCTYPE html>";
  html += "<html><head><title>ESP32 DRONE Control</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>";
  // Thiết lập chung
  html += "body {text-align: center; font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f8f9fa;}";
  html += "h1 {color: #343a40; margin-bottom: 20px;}";
  html += "h2 {color: #495057; margin-top: 30px;}";
  html += ".button-group {display: flex; justify-content: center; gap: 15px; flex-wrap: nowrap;}";  // Đảm bảo các button nằm ngang
  html += "button {font-size: 16px; padding: 10px 15px; background: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; transition: all 0.3s ease;}";
  html += "button:hover {background: #0056b3;}";
  html += "input {font-size: 16px; padding: 10px; width: 80%; max-width: 10px; margin: 5px 0; border: 1px solid #ccc; border-radius: 5px;}";
  html += "input:focus {border-color: #007bff; outline: none;}";
  html += "label {font-size: 16px; color: #495057; margin-right: 5px;}";
  html += ".form-container {max-width: 400px; margin: 0 auto; padding: 15px; background: white; border-radius: 8px; box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1);}";
  html += "hr {margin: 20px 0; border: 1px solid #dee2e6;}";

  // Media queries cho smartphone
  html += "@media (max-width: 600px) {";
  html += "h1 {font-size: 22px;}";
  html += "h2 {font-size: 18px;}";
  html += "button {font-size: 14px; padding: 8px 12px;}";
  html += "input {font-size: 14px; padding: 8px; max-width: 100%;}";
  html += ".form-container {width: 95%; padding: 10px;}";
  html += "}";

  html += "</style>";
  html += "</head><body>";

  // Tiêu đề
  html += "<h1>ESP32 DRONE Control</h1>";

  // Nhóm nút điều khiển
  html += "<div class='form-container'>";
  html += "<div class='button-group'>";
  html += "<form action='/off'><button>Pause</button></form>";
  html += "<form action='/UP'><button>Increase</button></form>";
  html += "<form action='/DOWN'><button>Decrease</button></form>";
  html += "<form action='/RESET'><button>Start/Reset</button></form>";
  html += "</div><hr>";

  // Input Throttle
  html += "<h2>Throttle Input</h2>";
  html += "<form action='/custom'>";
  html += "<input type='text' name='value' placeholder='Enter value'>";
  html += "<input type='submit' value='Send'>";
  html += "</form><hr>";

  // Slider điều chỉnh throttle
  html += "<div class='slider-container'>";
  html += "<label for='throttle'>Throttle Value:</label>";
  html += "<input type='range' id='throttle' name='throttle' class='slider' min='1000' max='2000' value='1200' step='1' oninput='updateThrottle(this.value)'>";
  html += "<span id='throttleValue'>1500</span>";
  html += "</div>";

  // Script để cập nhật slider
  html += "<script>";
  html += "function updateThrottle(value) {";
  html += "  document.getElementById('throttleValue').innerText = value;";
  html += "  fetch('/throttle?value=' + value).then(response => console.log('Throttle updated to: ' + value));";
  html += "}";
  html += "</script>";

  // Nhóm PID Parameters
  html += "<h2>Adjust PID Parameters</h2>";
  html += "<form action='/set_pid'>";
  html += "<div class='pid-group'>";
  html += "<label for='kp'>Kp:</label><input type='text' id='kp' name='kp' placeholder='Kp'>";
  html += "<label for='ki'>Ki:</label><input type='text' id='ki' name='ki' placeholder='Ki'>";
  html += "<label for='kd'>Kd:</label><input type='text' id='kd' name='kd' placeholder='Kd'>";
  html += "</div>";
  html += "<button type='submit'>Set PID</button>";
  html += "</form>";
  html += "</div>";

  html += "</body></html>";
  return html;
}

WiFiServer server(80);  // Tạo server ở cổng 80

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
  Wire.requestFrom(0x68, 6);
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
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  Rate_Roll = (float)GyroX / 65.5;
  Rate_Pitch = (float)GyroY / 65.5;
  Rate_Yaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

/*void calibrateGyro(float &RateCalibrationRoll, float &RateCalibrationPitch, float &RateCalibrationYaw) {
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
}*/
// Bộ lọc kalman
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
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
  float Input_RateYaw) {
  /*    Motor 1 (M1): Phải-trước 12 CW
     Motor 2 (M2): Phải-sau số 27 CWW
      Motor 3 (M3): Trái-sau số 14 CW
       Motor 4 (M4): Trái-trước số 13 CWW
*/
  Input_motor1 = Throttle - Input_RateRoll - Input_RatePitch + Input_RateYaw;
  Input_motor2 = Throttle - Input_RateRoll + Input_RatePitch - Input_RateYaw;
  Input_motor3 = Throttle + Input_RateRoll + Input_RatePitch + Input_RateYaw;
  Input_motor4 = Throttle + Input_RateRoll - Input_RatePitch - Input_RateYaw;
  /*Serial.println(Input_RateRoll);
  Serial.println(".........");*/
}

void setup() {
  // Cài đặt GPIO2 làm OUTPUT
  pinMode(2, OUTPUT);

  digitalWrite(2, LOW);  // Tắt LED ban đầu

  // Bắt đầu Access Point
  WiFi.softAP(ssid, password);
  Serial.begin(115200);
  Serial.println("Access Point đã khởi tạo!");
  Serial.print("SSID: ");
  Serial.println(ssid);

  // Bắt đầu server
  server.begin();
  Serial.println("Server đã khởi động!");

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.println("calibrateGyro!!!");
  //calibrateGyro(RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw);
  RateCalibrationRoll = 0.79;
  RateCalibrationPitch = -0.97;
  RateCalibrationYaw = 1.41;


  //Serial.println("calibrateGyro_Done!!!");

  mot1.attach(mot1_pin, 1000, 2000);
  delay(1000);
  mot1.setPeriodHertz(ESCfreq);
  delay(100);
  mot2.attach(mot2_pin, 1000, 2000);
  delay(1000);
  mot2.setPeriodHertz(ESCfreq);
  delay(100);
  mot3.attach(mot3_pin, 1000, 2000);
  delay(1000);
  mot3.setPeriodHertz(ESCfreq);
  delay(100);
  mot4.attach(mot4_pin, 1000, 2000);
  delay(1000);
  mot4.setPeriodHertz(ESCfreq);
  delay(100);

  mot1.writeMicroseconds(throttleValue);
  mot2.writeMicroseconds(throttleValue);
  mot3.writeMicroseconds(throttleValue);
  mot4.writeMicroseconds(throttleValue);
  delay(1000);
  //Kp_roll = 3.9;
  //Ki_roll = 0.1;
  //Kd_roll = 0.55;
  Kp_roll = 3.9;
  Ki_roll = 0.1;
  Kd_roll = 0.58;
  Kp_pitch = Kp_roll;
  Ki_pitch = Ki_roll;
  Kd_pitch = Kd_roll;
  Kp_roll_gyro = 2;
  Ki_roll_gyro = 0.01;
  Kd_roll_gyro = 0.03;
  Kp_pitch_gyro = Kp_roll_gyro;
  Ki_pitch_gyro = Ki_roll_gyro;
  Kd_pitch_gyro = Kd_roll_gyro;
  Serial.println("calibrateESC!!!");
  Serial.println("setup done!!!");
  digitalWrite(2, LOW);
  digitalWrite(2, HIGH);
  commandValue = 0;
  LoopTimer = micros();
}

void loop() {
  //unsigned long start = micros();
  WiFiClient client = server.available();  // Lắng nghe kết nối
  if (client) {
    String request = client.readStringUntil('\r');  // Đọc yêu cầu HTTP
    client.flush();

    // Kiểm tra yêu cầu
    // Xử lý yêu cầu
    if (request.indexOf("GET /off") != -1) {
      digitalWrite(2, LOW);  // Tắt LED
      Serial.println("Command Value: " + String(12));
      commandValue = 1;
      throttleValue = 1000;
      mot1.writeMicroseconds(throttleValue);
      mot2.writeMicroseconds(throttleValue);
      mot3.writeMicroseconds(throttleValue);
      mot4.writeMicroseconds(throttleValue);

    } else if (request.indexOf("GET /UP") != -1) {
      digitalWrite(2, HIGH);  // Bật LED
      Serial.println("Command Value: " + String(10));
      throttleValue = throttleValue + 50;
    } else if (request.indexOf("GET /RESET") != -1) {
      Serial.println("Command Value: " + String(13));
      output_roll = 0;
      output_pitch = 0;
      integral_roll = 0;
      integral_pitch = 0;
      integral_roll_gyro = 0;
      integral_pitch_gyro = 0;
      output_roll_rate = 0;
      output_pitch_rate = 0;
      pre_error_roll = 0;
      pre_error_pitch = 0;
      pre_error_yaw = 0;
      pre_error_roll_gyro = 0;
      pre_error_pitch_gyro = 0;
      pre_error_yaw_gyro = 0;
      commandValue = 0;

    } else if (request.indexOf("GET /DOWN") != -1) {
      digitalWrite(2, LOW);  // Tắt LED
      Serial.println("Command Value: " + String(11));
      throttleValue = throttleValue - 50;
    } else if (request.indexOf("GET /throttle") != -1) {
      // Lấy giá trị của throttle từ query string
      int valueIndex = request.indexOf("value=");
      if (valueIndex != -1) {
        String throttleInput = request.substring(valueIndex + 6);
        throttleInput = throttleInput.substring(0, throttleInput.indexOf(' '));
        throttleValue = throttleInput.toInt();
        Serial.println("Throttle Value: " + throttleInput);
      }
    } else if (request.indexOf("GET /custom") != -1) {
      // Xử lý giá trị nhập từ client
      int valueIndex = request.indexOf("value=");
      if (valueIndex != -1) {
        String customValue = request.substring(valueIndex + 6);            // Lấy giá trị sau "value="
        customValue = customValue.substring(0, customValue.indexOf(' '));  // Loại bỏ phần dư
        throttleValue = customValue.toInt();
        Serial.println("Throttle Value: " + customValue);

        // Xử lý giá trị nhận được từ input (nếu cần)
      }
    } else if (request.indexOf("GET /set_pid") != -1) {
      // Xử lý thông số PID
      String kp = "";
      String ki = "";
      String kd = "";

      // In ra yêu cầu nhận được để kiểm tra
      //Serial.println("Request for PID: " + request);

      // Lấy giá trị Kp
      int kpIndex = request.indexOf("kp=");
      if (kpIndex != -1) {
        kp = request.substring(kpIndex + 3);
        int endIndex = kp.indexOf('&');
        if (endIndex != -1) {
          kp = kp.substring(0, endIndex);  // Lấy giá trị Kp
          Kp = kp.toFloat();
          Kp_roll = Kp;
          Kp_pitch = Kp;
        }
        Serial.println("Kp value: " + kp);  // Debug để xem giá trị nhận được



      } else {
        Serial.println("Kp not found in request");  // Nếu không có giá trị Kp
      }

      // Lấy giá trị Ki
      int kiIndex = request.indexOf("ki=");
      if (kiIndex != -1) {
        ki = request.substring(kiIndex + 3);
        int endIndex = ki.indexOf('&');
        if (endIndex != -1) {
          ki = ki.substring(0, endIndex);  // Lấy giá trị Ki
          Ki = ki.toFloat();

          Ki_roll = Ki;

          Ki_pitch = Ki;
        }
        Serial.println("Ki value: " + ki);  // Debug để xem giá trị nhận được



      } else {
        Serial.println("Ki not found in request");  // Nếu không có giá trị Ki
      }

      // Lấy giá trị Kd
      int kdIndex = request.indexOf("kd=");
      if (kdIndex != -1) {
        kd = request.substring(kdIndex + 3);
        int endIndex = kd.indexOf(' ');
        if (endIndex != -1) {
          kd = kd.substring(0, endIndex);  // Lấy giá trị Kd
          Kd = kd.toFloat();

          Kd_roll = Kd;


          Kd_pitch = Kd;
        }
        Serial.println("Kd value: " + kd);  // Debug để xem giá trị nhận được



      } else {
        Serial.println("Kd not found in request");  // Nếu không có giá trị Kd
      }

      // Sau khi nhận giá trị PID, bạn có thể sử dụng chúng để cài đặt hoặc xử lý thêm
    }

    // Gửi phản hồi cho client
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println();
    client.println(generateHTML());
    client.stop();
  }
  if (commandValue != 1) {

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
    /*KalmanAngleRoll = 0.991 * (KalmanAngleRoll + Rate_Roll * 0.004) + (1 - 0.991) * AngleRoll;
    KalmanAnglePitch = 0.991 * (KalmanAnglePitch + Rate_Pitch * 0.004) + (1 - 0.991) * AnglePitch;*/

    KalmanAngleRoll = (KalmanAngleRoll > 25) ? 25 : ((KalmanAngleRoll < -25) ? -25 : KalmanAngleRoll);
    KalmanAnglePitch = (KalmanAnglePitch > 25) ? 25 : ((KalmanAnglePitch < -25) ? -25 : KalmanAnglePitch);

    unsigned long current_time = micros();
    dt = (current_time - LoopTimer) / 1e6;  // Đổi thành giây



    output_roll = CalculatePID2(Roll_set, KalmanAngleRoll, pre_error_roll, Kp_roll, Ki_roll, Kd_roll, dt, integral_roll);

    output_pitch = CalculatePID2(Pitch_set, KalmanAnglePitch, pre_error_pitch, Kp_pitch, Ki_pitch, Kd_pitch, dt, integral_pitch);

    output_roll = (output_roll > 200) ? 200 : ((output_roll < -400) ? -400 : output_roll);
    output_pitch = (output_pitch > 200) ? 200 : ((output_pitch < -400) ? -400 : output_pitch);
    // float output_pitch = 0 ;
    computeMotorInputs(throttleValue, output_roll, 0, 0);  //
    //computeMotorInputs(throttleValue, output_roll, output_pitch, 0); //
    Input_motor1 = (Input_motor1 < 1000) ? 1000 : (Input_motor1 > 1650) ? 1650
                                                                        : Input_motor1;
    Input_motor2 = (Input_motor2 < 1000) ? 1000 : (Input_motor2 > 1650) ? 1650
                                                                        : Input_motor2;
    Input_motor3 = (Input_motor3 < 1000) ? 1000 : (Input_motor3 > 1650) ? 1650
                                                                        : Input_motor3;
    Input_motor4 = (Input_motor4 < 1000) ? 1000 : (Input_motor4 > 1650) ? 1650
                                                                        : Input_motor4;
    /*if (millis() - pretime > 10) {
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
      pretime = millis();
    }*/



    mot1.writeMicroseconds(Input_motor1);
    mot2.writeMicroseconds(Input_motor2);
    mot3.writeMicroseconds(Input_motor3);
    mot4.writeMicroseconds(Input_motor4);

    //unsigned long elapsed = micros() - start;
    //Serial.println(elapsed); // In thời gian thực thi (micro giây)
    while (micros() - LoopTimer < 4000)
      ;
    LoopTimer = micros();
  }
}
