
/*
 * https://maniacbug.github.io/RF24/classRF24.html
 * 
 * VCC - 3.3v
 * GND - GND
 * CSN - 8
 * CE - 7
 * SCK - 13
 * MOSI - 11
 * MISO - 12
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
int delayValue = 1000;
int delayValue1 = 1300;

RF24 radio(7, 8); // CE, CSN
const byte diachi[6] = "12345";
Servo ESC; 
Servo ESC1;
Servo ESC2;
Servo ESC3;    
int data;
int Data0;

unsigned short wait_temp = 500;
unsigned long pretime_temp;


void setup() 
{
  Serial.begin(9600);
  
  if (!radio.begin()) 
  {
    Serial.println("Module không khởi động được...!!");
    while (1) {}
  } 
  
  radio.openReadingPipe(1, diachi);
  //Lệnh openReadingPipe có số đường truyền từ 0-5
  //Nhưng đường 0 đã được dùng cho ghi (mặc định)
  //Vì vậy chỉ dùng 1-5, nếu dùng không sẽ bị chồng lấn
  //Mở 1 kênh có địa chỉ 12345 trên đường truyền 1
  //kênh này chỉ đọc data trên địa chỉ 12345   
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(80);
  radio.setDataRate(RF24_1MBPS);
  radio.startListening();
  
  if (!radio.available())
  {
    Serial.println("Chưa kết nối được với TX...!!");
    Serial.println("CHỜ KẾT NỐI.......");
  } 
  ESC.attach(3);
  ESC1.attach(5);
  ESC2.attach(6);
  ESC3.attach(9); 
  ESC.writeMicroseconds(delayValue);
  ESC1.writeMicroseconds(delayValue);
  ESC2.writeMicroseconds(delayValue);
  ESC3.writeMicroseconds(delayValue);
  delay(2000);
}

void loop() 
{
  if ((millis() - pretime_temp) >= wait_temp) {radio.read(&data, sizeof(data));  Serial.println(data); pretime_temp = millis();}  
    
                                                           
    if (data == 1) {
  Data0 = 1000;
} else if (data == 2) {
  Data0 = 1200;
} else if (data == 3) {
  Data0 = 1300;
} else { // Trường hợp không khớp với bất kỳ giá trị nào
  Data0 = 0;
}
    

  ESC.writeMicroseconds(Data0);
  ESC1.writeMicroseconds(Data0);
  ESC2.writeMicroseconds(Data0);
  ESC3.writeMicroseconds(Data0);   
}
