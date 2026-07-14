#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>          // Thư viện quản lý cấu hình WiFi qua Web

// Định nghĩa chân LED (ESP32-S2 thường dùng chân GPIO 15 làm LED onboard, hoặc bạn sửa thành chân LED trên mạch của bạn)
#define LED_PIN 15

// Khai báo phiên bản hiện tại của firmware đang chạy trong mạch
const String CURRENT_VERSION = "2.0"; 

// Đường dẫn file firmware.bin và file version.txt trên GitHub
const String URL_VERSION   = "https://raw.githubusercontent.com/pethitechcom-dotcom/esp32-ota-firmware/main/version.txt";
const String URL_FIRMWARE  = "https://raw.githubusercontent.com/pethitechcom-dotcom/esp32-ota-firmware/main/firmware.ino.bin";

// Khai báo các hàm
void checkAndPerformOTA();
void blinkLed(int times, int delayTime);

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    // Cấu hình chân LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // Tắt LED lúc khởi động

    Serial.println("\n--- ESP32-S2 OTA & WIFI MANAGER ---");
    Serial.print("Phiên bản hiện tại: v");
    Serial.println(CURRENT_VERSION);

    // Khởi tạo WiFiManager
    WiFiManager wm;

    Serial.println("Đang kiểm tra kết nối WiFi...");
    // Tự động phát AP "ESP32-S2-Setup" nếu không kết nối được WiFi cũ
    if (!wm.autoConnect("ESP32-S2-Setup")) {
        Serial.println("Cấu hình WiFi thất bại. Khởi động lại...");
        delay(3000);
        ESP.restart();
    }

    Serial.println("\nKết nối WiFi thành công!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    // Nháy LED nhanh 5 lần báo hiệu đã kết nối WiFi thành công
    blinkLed(5, 100);

    // Tiến hành kiểm tra phiên bản và cập nhật OTA
    checkAndPerformOTA();
}

void loop() {
    // Chương trình nhấp nháy LED liên tục để báo hiệu mạch đang chạy bình thường
    // Chu kỳ: Sáng 500ms, Tắt 500ms
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);

    // Kiểm tra định kỳ mỗi 5 phút (300000 ms)
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 300000) {
        lastCheck = millis();
        if (WiFi.status() == WL_CONNECTED) {
            checkAndPerformOTA();
        }
    }
}

// Hàm phụ trợ nhấp nháy LED nhanh để báo trạng thái
void blinkLed(int times, int delayTime) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(delayTime);
        digitalWrite(LED_PIN, LOW);
        delay(delayTime);
    }
}

void checkAndPerformOTA() {
    Serial.println("Đang kiểm tra phiên bản mới từ GitHub...");

    WiFiClientSecure client;
    client.setInsecure(); // Bỏ qua xác thực SSL để kết nối ổn định

    HTTPClient http;
    // 1. Kết nối để đọc file version.txt trước
    if (http.begin(client, URL_VERSION)) {
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();
            payload.trim(); // Xóa bỏ khoảng trắng thừa hoặc ký tự xuống dòng (\n)
            
            Serial.print("Phiên bản trên GitHub: v");
            Serial.println(payload);

            // So sánh phiên bản trên mạng với phiên bản hiện tại trong mạch
            if (payload != CURRENT_VERSION) { 
                Serial.println("Phát hiện phiên bản mới! Bắt đầu quá trình tải Firmware...");
                
                // Nháy LED liên tục 3 lần thật chậm để báo hiệu chuẩn bị OTA
                blinkLed(3, 500);

                // Cấu hình tiến trình cập nhật
                httpUpdate.onStart([]() {
                    Serial.println("OTA: Bắt đầu tải firmware mới...");
                });
                httpUpdate.onEnd([]() {
                    Serial.println("OTA: Tải xong! Đang nạp vào flash...");
                });
                httpUpdate.onProgress([](int cur, int total) {
                    Serial.printf("OTA: Tiến độ: %d%%\r", (cur * 100) / total);
                });
                
                // 2. Thực hiện cập nhật firmware (.bin)
                t_httpUpdate_return ret = httpUpdate.update(client, URL_FIRMWARE);

                switch (ret) {
                    case HTTP_UPDATE_FAILED:
                        Serial.printf("Cập nhật thất bại. Lỗi (%d): %s\n", 
                                      httpUpdate.getLastError(), 
                                      httpUpdate.getLastErrorString().c_str());
                        break;
                    case HTTP_UPDATE_NO_UPDATES:
                        Serial.println("Không có bản cập nhật.");
                        break;
                    case HTTP_UPDATE_OK:
                        Serial.println("Cập nhật thành công! Mạch đang tự khởi động lại...");
                        break;
                }
            } else {
                Serial.println("Bạn đang sử dụng phiên bản mới nhất. Không cần cập nhật.");
            }
        } else {
            Serial.printf("Không thể đọc file version.txt. Mã lỗi HTTP: %d\n", httpCode);
        }
        http.end();
    } else {
        Serial.println("Không thể kết nối tới máy chủ GitHub.");
    }
}