#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>      // Thư viện quản lý cấu hình WiFi qua Web
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>

// ================= CẤU HÌNH FIRMWARE & OTA =================
const String CURRENT_VERSION = "2.1";  
const String URL_VERSION  = "https://raw.githubusercontent.com/pethitechcom-dotcom/esp32-ota-firmware/main/version.txt";
const String URL_FIRMWARE = "https://raw.githubusercontent.com/pethitechcom-dotcom/esp32-ota-firmware/main/firmware.ino.bin";

// ================= CẤU HÌNH CHÂN GPIO ĐÃ CHỐT =================
#define LED_PIN         1   // LED báo trạng thái (0=Sáng, 1=Tắt)
#define BUZZER_PIN      37  // Còi báo (1=Bật, 0=Tắt)
#define POT_PIN         2   // Biến trở chỉnh tốc độ (ADC)
#define BTN_START       3   // Nút bấm chạy máy/tạm dừng
#define BTN_HOME        4   // Nút bấm đưa về Home
#define BTN_ESTOP       5   // Nút dừng khẩn cấp (Ngắt cứng)
#define SENSOR_HOME     6   // Cảm biến tiệm cận gốc Home
#define SENSOR_END      7   // Cảm biến tiệm cận giới hạn cuối

#define STEP_PUL        17  // Chân xung Step
#define STEP_DIR        16  // Chân chiều DIR
#define STEP_ENA        18  // Chân cho phép ENA
#define RELAY_CUTTER    34  // Rơ-le Contactor máy cắt

// ================= HẰNG SỐ HIỆU CHUẨN BIẾN TRỞ =================
#define POT_RAW_MIN     100
#define POT_RAW_MAX     2735
#define DEADBAND_LOW    130
#define DEADBAND_HIGH   2700

// ================= ĐỐI TƯỢNG & BIẾN =================
LiquidCrystal_I2C lcd(0x27, 20, 4); 
AccelStepper stepper(AccelStepper::DRIVER, STEP_PUL, STEP_DIR); 

enum MachineState {
  STATE_IDLE,
  STATE_DELAY_START,
  STATE_CUTTING,
  STATE_END_LIMIT,
  STATE_HOMING,
  STATE_ESTOP
};

volatile MachineState currentState = STATE_IDLE;
volatile bool eStopTriggered = false;

unsigned long delayStartTime = 0;
const unsigned long CUT_DELAY_MS = 2000; 
int currentSpeed = 0;
int targetSpeed = 0;
unsigned long lastLcdUpdate = 0;

// Khử dội phím Non-blocking
bool startBtnState = false;
bool homeBtnState  = false;
unsigned long lastStartPress = 0;
unsigned long lastHomePress  = 0;

// Còi bíp Non-blocking
unsigned long beepEndTime = 0;
bool isBeeping = false;

// Bộ lọc biến trở
const int numReadings = 15;
int readings[numReadings];      
int readIndex = 0;              
long total = 0;                 

// Khai báo hàm phụ trợ OTA
void checkAndPerformOTA();
void triggerBeep(unsigned long duration);
void handleBuzzer();
void controlLED();
void readPotentiometer();
void displayStatus();

// ================= HÀM NGẮT KHẨN CẤP =================
void IRAM_ATTR eStopISR() {
  eStopTriggered = true;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_HOME, INPUT_PULLUP);
  pinMode(BTN_ESTOP, INPUT_PULLUP);
  pinMode(SENSOR_HOME, INPUT_PULLUP);
  pinMode(SENSOR_END, INPUT_PULLUP);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_CUTTER, OUTPUT);
  
  digitalWrite(LED_PIN, LOW);      // 0 = Sáng (Báo nguồn sẵn sàng)
  digitalWrite(BUZZER_PIN, LOW);   // 0 = Tắt còi
  digitalWrite(RELAY_CUTTER, LOW); // Tắt rơ-le

  attachInterrupt(digitalPinToInterrupt(BTN_ESTOP), eStopISR, FALLING);

  analogReadResolution(12);
  for (int i = 0; i < numReadings; i++) readings[i] = 0;

  stepper.setEnablePin(STEP_ENA);
  stepper.setPinsInverted(false, false, true); 
  stepper.setMaxSpeed(4000); 
  stepper.setAcceleration(2500);
  stepper.enableOutputs();

  Wire.begin(8, 9);
  Wire.setTimeOut(100); 
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("====================");
  lcd.setCursor(0, 1); lcd.print(" KHOI KET NOI WIFI  ");
  lcd.setCursor(0, 2); lcd.print(" VUI LONG CHO...    ");
  lcd.setCursor(0, 3); lcd.print("====================");

  // 1. Khởi tạo WiFiManager kết nối mạng
  WiFiManager wm;
  if (!wm.autoConnect("ESP32-S2-Setup")) {
    Serial.println("Cấu hình WiFi thất bại. Khởi động lại...");
    delay(3000);
    ESP.restart();
  }

  lcd.setCursor(0, 1); lcd.print(" WIFI CONNECTED     ");
  triggerBeep(100); delay(150);
  triggerBeep(100); delay(150);
  delay(1000);

  // 2. Kiểm tra cập nhật OTA ngay khi khởi động thành công
  lcd.setCursor(0, 1); lcd.print(" KIEM TRA CAP NHAT  ");
  checkAndPerformOTA();

  lcd.clear();
}

// ================= VÒNG LẶP CHÍNH =================
void loop() {
  // Kiểm tra cập nhật OTA định kỳ mỗi 5 phút (300000 ms)
  static unsigned long lastOtaCheck = 0;
  if (millis() - lastOtaCheck > 300000) {
    lastOtaCheck = millis();
    if (WiFi.status() == WL_CONNECTED) {
      checkAndPerformOTA();
    }
  }

  // 1. XỬ LÝ KHẨN CẤP
  if (eStopTriggered) {
    if (currentState != STATE_ESTOP) {
      currentState = STATE_ESTOP;
      digitalWrite(RELAY_CUTTER, LOW); 
      stepper.stop();
      stepper.disableOutputs();        
      displayStatus();                 
    }
    handleBuzzer(); 
    controlLED(); // Ép LED cập nhật trạng thái lỗi
    return; 
  }

  // 2. ĐỌC BIẾN TRỞ & ĐIỀU KHIỂN ĐÈN/CÒI
  readPotentiometer();
  handleBuzzer();
  controlLED(); // LED nhấp nháy chạy hoàn toàn độc lập dưới nền

  // 3. ĐỌC NÚT NHẤN KHÔNG CHẶN CPU
  bool currentStart = (digitalRead(BTN_START) == LOW);
  bool currentHome  = (digitalRead(BTN_HOME) == LOW);
  bool triggerStart = false;
  bool triggerHome  = false;

  if (currentStart && !startBtnState && (millis() - lastStartPress > 200)) {
    startBtnState = true;
    lastStartPress = millis();
    triggerStart = true;
    triggerBeep(100); 
  } else if (!currentStart) {
    startBtnState = false; 
  }

  if (currentHome && !homeBtnState && (millis() - lastHomePress > 200)) {
    homeBtnState = true;
    lastHomePress = millis();
    triggerHome = true;
    triggerBeep(100); 
  } else if (!currentHome) {
    homeBtnState = false;
  }

  // 4. MÁY TRẠNG THÁI
  switch (currentState) {
    
    case STATE_IDLE:
      digitalWrite(RELAY_CUTTER, LOW); 
      if (triggerStart) { 
        currentState = STATE_DELAY_START;
        digitalWrite(RELAY_CUTTER, HIGH); 
        delayStartTime = millis();
      }
      if (triggerHome) { 
        currentState = STATE_HOMING;
        stepper.setSpeed(-1200); 
      }
      break;

    case STATE_DELAY_START:
      if (millis() - delayStartTime >= CUT_DELAY_MS) {
        currentState = STATE_CUTTING;
      }
      break;

    case STATE_CUTTING:
      stepper.setSpeed(currentSpeed);
      stepper.runSpeed(); 

      if (digitalRead(SENSOR_END) == LOW) {
        currentState = STATE_END_LIMIT;
        digitalWrite(RELAY_CUTTER, LOW); 
        stepper.stop();
        triggerBeep(600); 
      }

      if (triggerStart) {
        currentState = STATE_IDLE;
        digitalWrite(RELAY_CUTTER, LOW);
        stepper.stop();
      }
      break;

    case STATE_END_LIMIT:
      if (triggerHome) {
        currentState = STATE_HOMING;
        stepper.setSpeed(-1200); 
      }
      break;

    case STATE_HOMING:
      stepper.runSpeed();

      if (digitalRead(SENSOR_HOME) == LOW) {
        stepper.stop();
        stepper.setCurrentPosition(0); 
        currentState = STATE_IDLE;
        triggerBeep(600); 
      }
      break;

    case STATE_ESTOP:
      break;
  }

  // 5. CẬP NHẬT MÀN HÌNH LCD
  if (millis() - lastLcdUpdate > 300) {
    displayStatus();
    lastLcdUpdate = millis();
  }
}

// ================= CÁC HÀM XỬ LÝ PHỤ TRỢ & OTA =================

void checkAndPerformOTA() {
  Serial.println("Đang kiểm tra phiên bản mới từ GitHub...");

  WiFiClientSecure client;
  client.setInsecure(); // Bỏ qua xác thực SSL để kết nối raw.githubusercontent.com mượt mà

  HTTPClient http;
  if (http.begin(client, URL_VERSION)) {
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      payload.trim(); // Xóa khoảng trắng thừa hoặc ký tự xuống dòng
      
      Serial.print("Phiên bản trên GitHub: v");
      Serial.println(payload);

      if (payload != CURRENT_VERSION) {  
        Serial.println("Phát hiện phiên bản mới! Bắt đầu tải Firmware...");
        
        lcd.setCursor(0, 2); lcd.print(" DANG TAI PHIEN BAN  ");
        lcd.setCursor(0, 3); lcd.print(" MOI TU GITHUB...     ");

        // Cấu hình sự kiện cập nhật
        httpUpdate.onStart([]() {
          Serial.println("OTA: Bắt đầu tải firmware mới...");
        });
        httpUpdate.onEnd([]() {
          Serial.println("OTA: Tải xong! Đang nạp vào flash...");
        });
        httpUpdate.onProgress([](int cur, int total) {
          Serial.printf("OTA: Tiến độ: %d%%\r", (cur * 100) / total);
        });
        
        t_httpUpdate_return ret = httpUpdate.update(client, URL_FIRMWARE);

        switch (ret) {
          case HTTP_UPDATE_FAILED:
            Serial.printf("Cập nhật thất bại. Lỗi (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
            break;
          case HTTP_UPDATE_NO_UPDATES:
            Serial.println("Không có bản cập nhật.");
            break;
          case HTTP_UPDATE_OK:
            Serial.println("Cập nhật thành công! Mạch đang tự khởi động lại...");
            break;
        }
      } else {
        Serial.println("Mạch đang dùng phiên bản mới nhất.");
      }
    } else {
      Serial.printf("Không thể đọc file version.txt. Mã lỗi HTTP: %d\n", httpCode);
    }
    http.end();
  } else {
    Serial.println("Không thể kết nối tới máy chủ GitHub.");
  }
}

void controlLED() {
  static unsigned long lastLedToggle = 0;
  static bool ledState = false;

  if (currentState == STATE_IDLE) {
    digitalWrite(LED_PIN, LOW); // 0 = Sáng liên tục chờ lệnh
  } 
  else if (currentState == STATE_ESTOP) {
    digitalWrite(LED_PIN, HIGH); // 1 = Tắt ngóm khi lỗi
  } 
  else {
    if (millis() - lastLedToggle > 250) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? LOW : HIGH);
      lastLedToggle = millis();
    }
  }
}

void triggerBeep(unsigned long duration) {
  if (currentState == STATE_ESTOP) return; 
  digitalWrite(BUZZER_PIN, HIGH); 
  beepEndTime = millis() + duration;
  isBeeping = true;
}

void handleBuzzer() {
  if (currentState == STATE_ESTOP) {
    static unsigned long lastEstopBeep = 0;
    static bool estopBeepState = false;
    if (millis() - lastEstopBeep > 150) {
      estopBeepState = !estopBeepState;
      digitalWrite(BUZZER_PIN, estopBeepState ? HIGH : LOW);
      lastEstopBeep = millis();
    }
    return;
  }
  
  if (isBeeping && millis() >= beepEndTime) {
    digitalWrite(BUZZER_PIN, LOW); 
    isBeeping = false;
  }
}

void readPotentiometer() {
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(POT_PIN);
  total = total + readings[readIndex];
  readIndex++;
  if (readIndex >= numReadings) readIndex = 0;
  
  int averageAdc = total / numReadings;
  int speedPercent = 0;

  if (averageAdc <= DEADBAND_LOW) {
    speedPercent = 0; 
  } 
  else if (averageAdc >= DEADBAND_HIGH) {
    speedPercent = 100; 
  } 
  else {
    speedPercent = map(averageAdc, DEADBAND_LOW, DEADBAND_HIGH, 0, 100);
  }

  speedPercent = constrain(speedPercent, 0, 100);
  targetSpeed = map(speedPercent, 0, 100, 0, 3200);

  if (abs(targetSpeed - currentSpeed) > 16) { 
    currentSpeed = targetSpeed;
  }
}

void displayStatus() {
  char rowBuffer[32]; 

  switch (currentState) {
    case STATE_IDLE:        snprintf(rowBuffer, sizeof(rowBuffer), "STATE: READY        "); break; 
    case STATE_DELAY_START: snprintf(rowBuffer, sizeof(rowBuffer), "STATE: SPINNING UP  "); break; 
    case STATE_CUTTING:     snprintf(rowBuffer, sizeof(rowBuffer), "STATE: CUTTING      "); break; 
    case STATE_END_LIMIT:   snprintf(rowBuffer, sizeof(rowBuffer), "STATE: END LIMIT    "); break; 
    case STATE_HOMING:      snprintf(rowBuffer, sizeof(rowBuffer), "STATE: HOMING       "); break; 
    case STATE_ESTOP:       snprintf(rowBuffer, sizeof(rowBuffer), "STATE: EMERGENCY    "); break; 
  }
  lcd.setCursor(0, 0); lcd.print(rowBuffer);

  if (currentState == STATE_ESTOP) {
    snprintf(rowBuffer, sizeof(rowBuffer), "MOTOR: DISABLED     ");
  } else {
    int displayPercent = map(currentSpeed, 0, 3200, 0, 100);
    displayPercent = constrain(displayPercent, 0, 100);
    snprintf(rowBuffer, sizeof(rowBuffer), "SPEED: %3d%%         ", displayPercent); 
  }
  lcd.setCursor(0, 1); lcd.print(rowBuffer);

  if (digitalRead(RELAY_CUTTER) == HIGH) {
    snprintf(rowBuffer, sizeof(rowBuffer), "CUTTER: RUNNING     ");
  } else {
    snprintf(rowBuffer, sizeof(rowBuffer), "CUTTER: STOPPED     ");
  }
  lcd.setCursor(0, 2); lcd.print(rowBuffer);

  int homeState = (digitalRead(SENSOR_HOME) == LOW) ? 1 : 0;
  int endState  = (digitalRead(SENSOR_END) == LOW) ? 1 : 0;
  
  snprintf(rowBuffer, sizeof(rowBuffer), "I/O: HOME:%d  END:%d  ", homeState, endState);
  lcd.setCursor(0, 3); lcd.print(rowBuffer);
}
