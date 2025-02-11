
#include <Servo.h>
int delayValue = 1000;
int data;
float Kp ;
float Ki ;
float Kd ;

String inputString = ""; // Chuỗi nhận dữ liệu
bool stringComplete = false; // Kiểm tra chuỗi hoàn tất


Servo ESC; 
Servo ESC1;
Servo ESC2;
Servo ESC3;    



void setup() 
{
   Serial.begin(9600);
  //inputString.reserve(200); // Dự trữ bộ nhớ cho chuỗi nhận
  ESC.attach(5);
  ESC1.attach(9);
  ESC2.attach(6);
  ESC3.attach(3); 
  ESC.writeMicroseconds(delayValue);
  ESC1.writeMicroseconds(delayValue);
  ESC2.writeMicroseconds(delayValue);
  ESC3.writeMicroseconds(delayValue);
  delay(2000);
}
void loop() {


  /*  if (stringComplete) {
    parseInput(inputString); // Phân tích dữ liệu nhận được
    inputString = ""; // Reset chuỗi sau khi xử lý xong
    stringComplete = false; // Đặt cờ chuỗi hoàn tất về false
  }
} */



    if (Serial.available() > 0) {
       
        int data = Serial.parseInt();
        if (data == 10 ){
          if (delayValue < 1900){delayValue = delayValue + 50;}
        }
        if (data == 11 ){
          if (delayValue > 1100){delayValue = delayValue - 50;}
        }
        if (data == 12){delayValue = 1000;}
        if (data > 999 && data < 2000) {
          delayValue = data ;
        }
            ESC.writeMicroseconds(delayValue);
            ESC1.writeMicroseconds(delayValue);
            ESC2.writeMicroseconds(delayValue);
            ESC3.writeMicroseconds(delayValue);
            float speed = (delayValue - 1000) / 10.0;

            Serial.println();
            Serial.println("Motor speed:");
            Serial.print(" ");                                                                       
            Serial.print(speed);
            Serial.println(" %");
        
    }
}
/*// Hàm nhận dữ liệu từ UART
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
  if (data.startsWith("Throttle Value: ")) {
    throttleValue = data.substring(16).toInt(); // Lấy giá trị Throttle
    Serial.print("Throttle updated: ");
    Serial.println(throttleValue);
  } else if (data.startsWith("Command Value: ")) {
    commandValue = data.substring(15).toInt(); // Lấy giá trị Command
    Serial.print("Command updated: ");
    Serial.println(commandValue);
  } else if (data.startsWith("Kp value: ")) {
    kpValue = data.substring(10).toFloat(); // Lấy giá trị Kp
    Serial.print("Kp updated: ");
    Serial.println(kpValue);
  } else if (data.startsWith("Ki value: ")) {
    kiValue = data.substring(10).toFloat(); // Lấy giá trị Ki
    Serial.print("Ki updated: ");
    Serial.println(kiValue);
  } else if (data.startsWith("Kd value: ")) {
    kdValue = data.substring(10).toFloat(); // Lấy giá trị Kd
    Serial.print("Kd updated: ");
    Serial.println(kdValue);
  } else {
    Serial.println("Unknown data received!");
  }
}*/
