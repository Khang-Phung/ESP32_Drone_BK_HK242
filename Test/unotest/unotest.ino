#include <nRF24L01.h>                                    
#include <RF24.h>    
const byte diachi[6] = "12345";                                 
RF24 radio(7, 8); //(CE, CSN)
//RF24 radio(7, 8); //(CE, CSN)
int data;                                
void setup(){
//  TCCR0A = 0;  
//  TCCR0A |= (1<<CS01)|(0<<CS00);
  Serial.begin(9600);
    if (!radio.begin()) 
  {
    Serial.println("Module không khởi động được...!!");
    while (1) {}
  } 
  radio.begin();                                        
  radio.setChannel(80);  // Channel(0 ... 127)
  radio.setDataRate     (RF24_250KBPS);  // RF24_250KBPS, RF24_1MBPS, RF24_2MBPS)
  radio.setPALevel      (RF24_PA_MIN); //RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm
  radio.openWritingPipe (diachi);  // 0x1234567890
  radio.stopListening();
  if (!radio.available())
  {
    Serial.println("Chưa kết nối được với RX...!!");
    Serial.println("CHỜ KẾT NỐI.......");
  } 
}
void loop(){
  data = 1;                            
  radio.write(&data, sizeof(data));
}