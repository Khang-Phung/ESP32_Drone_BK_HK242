#include <WiFi.h>

// Khai báo thông tin Access Point
const char* ssid = "ESP32_AccessPoint";
const char* password = "12345678";

// Trạng thái LED
bool ledState = false;

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
  html += ".button-group {display: flex; justify-content: center; gap: 15px; flex-wrap: nowrap;}"; // Đảm bảo các button nằm ngang
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
  html += "<form action='/off'><button>OFF Motor</button></form>";
  html += "<form action='/UP'><button>Increase</button></form>";
  html += "<form action='/DOWN'><button>Decrease</button></form>";
  html += "<form action='/RESET'><button>Calib_MPU</button></form>";
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




WiFiServer server(80); // Tạo server ở cổng 80

void setup() {
    // Cài đặt GPIO2 làm OUTPUT
    pinMode(2, OUTPUT);
    
    digitalWrite(2, LOW); // Tắt LED ban đầu

    // Bắt đầu Access Point
    WiFi.softAP(ssid, password);
    Serial.begin(9600);
    Serial.println("Access Point đã khởi tạo!");
    Serial.print("SSID: ");
    Serial.println(ssid);

    // Bắt đầu server
    server.begin();
    Serial.println("Server đã khởi động!");
}

void loop() {
    WiFiClient client = server.available(); // Lắng nghe kết nối
    if (client) {
        String request = client.readStringUntil('\r'); // Đọc yêu cầu HTTP
        client.flush();

        // Kiểm tra yêu cầu
         // Xử lý yêu cầu
        if (request.indexOf("GET /off") != -1) {
            digitalWrite(2, LOW); // Tắt LED
            Serial.println("Command Value: " + String(12));

        } else if (request.indexOf("GET /UP") != -1) {
            digitalWrite(2, HIGH); // Bật LED
            Serial.println("Command Value: " + String(10));

            } else if (request.indexOf("GET /RESET") != -1) {
            Serial.println("Command Value: " + String(13));

        } else if (request.indexOf("GET /DOWN") != -1) {
            digitalWrite(2, LOW); // Tắt LED
            Serial.println("Command Value: " + String(11));
            
          } else if (request.indexOf("GET /throttle") != -1) {
            // Lấy giá trị của throttle từ query string
            int valueIndex = request.indexOf("value=");
            if (valueIndex != -1) {
                String throttleValue = request.substring(valueIndex + 6);
                throttleValue = throttleValue.substring(0, throttleValue.indexOf(' '));
                Serial.println("Throttle Value: " + throttleValue);
            }
        } else if (request.indexOf("GET /custom") != -1) {
            // Xử lý giá trị nhập từ client
            int valueIndex = request.indexOf("value="); 
            if (valueIndex != -1) {
                String customValue = request.substring(valueIndex + 6); // Lấy giá trị sau "value="
                customValue = customValue.substring(0, customValue.indexOf(' ')); // Loại bỏ phần dư
                Serial.println("Throttle Value: " + customValue);
                // Xử lý giá trị nhận được từ input (nếu cần)
            }} else if (request.indexOf("GET /set_pid") != -1) {
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
                    kp = kp.substring(0, endIndex); // Lấy giá trị Kp
                }
                Serial.println("Kp value: " + kp); // Debug để xem giá trị nhận được



            } else {
                Serial.println("Kp not found in request"); // Nếu không có giá trị Kp
            }

            // Lấy giá trị Ki
            int kiIndex = request.indexOf("ki=");
            if (kiIndex != -1) {
                ki = request.substring(kiIndex + 3);
                int endIndex = ki.indexOf('&');
                if (endIndex != -1) {
                    ki = ki.substring(0, endIndex); // Lấy giá trị Ki
                }
                Serial.println("Ki value: " + ki); // Debug để xem giá trị nhận được



            } else {
                Serial.println("Ki not found in request"); // Nếu không có giá trị Ki
            }

            // Lấy giá trị Kd
            int kdIndex = request.indexOf("kd=");
            if (kdIndex != -1) {
                kd = request.substring(kdIndex + 3);
                int endIndex = kd.indexOf(' ');
                if (endIndex != -1) {
                    kd = kd.substring(0, endIndex); // Lấy giá trị Kd
                }
                Serial.println("Kd value: " + kd); // Debug để xem giá trị nhận được



            } else {
                Serial.println("Kd not found in request"); // Nếu không có giá trị Kd
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

    
}

