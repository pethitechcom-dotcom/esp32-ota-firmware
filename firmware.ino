#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>

// ================= CẤU HÌNH FIRMWARE & OTA =================
const String CURRENT_VERSION = "2.2";  
const String URL_VERSION  = "https://raw.githubusercontent.com/pethitechcom-dotcom/esp32-ota-firmware/main/version.txt";
const String URL_FIRMWARE = "https://raw.githubusercontent.com/pethitechcom-dotcom/esp32-ota-firmware/main/firmware.ino.bin";

// ================= CẤU HÌNH CHÂN GPIO ĐÃ CHỐT =================
#define LED_PIN         1   // LED báo trạng thái (0=Sáng, 1=Tắt)
#define BUZZER_PIN      37  // Còi báo (1=Bật, 0=Tắt)
#define POT_PIN         2   // Biến trở chỉnh tốc độ (ADC)
#define BTN_START       3   // Nút bấm chạy máy/tạm dừng
#define BTN_HOME        4   // Nút bấm đưa về Home
#define BTN_ESTOP       5   // Nút dừng khẩn cấp (Ngắt cứng)
#define SENSOR_HOME     6   // Cảm biến tiệm cận gốc Home (NPN)
#define SENSOR_END      7   // Cảm biến tiệm cận giới hạn cuối (NPN)

#define STEP_PUL        17  // Chân xung Step
#define STEP_DIR        16  // Chân chiều DIR
#define STEP_ENA        18  // Chân cho phép ENA
#define RELAY_CUTTER    34  // Rơ-le Contactor máy cắt (GPIO34)

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

// Khai báo hàm phụ trợ
void checkAndPerformOTA();
void triggerBeep(unsigned long duration);
void handleBuzzer();
void controlLED();
void readPotentiometer();
void displayStatus();

// ================= NGẮT PHẦN CỨNG BƠM XUNG ĐỘNG CƠ (TIMER) =================
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  stepper.runSpeed(); // Tự động phát xung ngầm không cần CPU chờ
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ================= HÀM NGẮT KHẨN CẤP (E-STOP) =================
void IRAM_ATTR eStopISR() {
  eStopTriggered = true;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Cấu hình ngõ vào có trở kéo lên (INPUT_PULLUP)
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

  // Cài đặt ngắt E-Stop
  attachInterrupt(digitalPinToInterrupt(BTN_ESTOP), eStopISR, FALLING);

  // Bộ lọc biến trở
  analogReadResolution(12);
  for (int i = 0; i < numReadings; i++) readings[i] = 0;

  // Cấu hình Stepper (Không set tốc độ ở đây, để Timer lo)
  stepper.setEnablePin(STEP_ENA);
  stepper.setPinsInverted(false, false, true); 
  stepper.setMaxSpeed(4000); 
  stepper.setAcceleration(2500);
  stepper.enableOutputs();

  // Khởi động I2C ép xung 400kHz để màn hình in cực nhanh
  Wire.begin(8, 9);
  Wire.setClock(400000); 
  Wire.setTimeOut(100); 
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("====================");
  lcd.setCursor(0, 1); lcd.print(" KHOI KET NOI WIFI  ");
  lcd.setCursor(0, 2); lcd.print(" VUI LONG CHO...    ");
  lcd.setCursor(0, 3); lcd.print("====================");

  // Khởi tạo WiFiManager kết nối mạng
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

  // Kiểm tra OTA lần đầu
  lcd.setCursor(0, 1); lcd.print(" KIEM TRA CAP NHAT  ");
  checkAndPerformOTA();

  lcd.clear();

  // Khởi động Hardware Timer cho động cơ (Dành cho core v2)
  // (Nếu dùng core v3 báo lỗi, đổi thành: timer = timerBegin(1000000); timerAttachInterrupt(timer, &onTimer);)
  timer = timerBegin(0, 80, true);               
  timerAttachInterrupt(timer, &onTimer, true);   
  timerAlarmWrite(timer, 50, true);              // Chu kỳ 50us = Tần số 20kHz phát xung
  timerAlarmEnable(timer);                       
}

// ================= VÒNG LẶP CHÍNH =================
void loop() {
  // 1. Kiểm tra OTA nền định kỳ 5 phút
  static unsigned long lastOtaCheck = 0;
  if (millis() - lastOtaCheck > 300000) {
    lastOtaCheck = millis();
    if (WiFi.status() == WL_CONNECTED) {
      checkAndPerformOTA();
    }
  }

  // 2. XỬ LÝ KHẨN CẤP AN TOÀN TUYỆT ĐỐI
  if (eStopTriggered) {
    if (digitalRead(BTN_ESTOP) == LOW) {
      // Đang bị nhấn: Khóa chết hệ thống
      if (currentState != STATE_ESTOP) {
        currentState = STATE_ESTOP;
        digitalWrite(RELAY_CUTTER, LOW); 
        stepper.stop();
        stepper.disableOutputs();        
        displayStatus();                 
      }
      handleBuzzer(); 
      controlLED(); 
      return; // Chặn CPU không chạy tiếp
    } 
    else {
      // Đã nhả nút: Reset và mở khóa
      eStopTriggered = false;          
      currentState = STATE_IDLE;       
      stepper.enableOutputs();         
      digitalWrite(BUZZER_PIN, LOW);   
      displayStatus();                 
    }
  }

  // 3. ĐỌC BIẾN TRỞ & NGOẠI VI NỀN
  readPotentiometer();
  handleBuzzer();
  controlLED(); 

  // 4. ĐỌC NÚT NHẤN (NON-BLOCKING)
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

  // 5. MÁY TRẠNG THÁI HỆ THỐNG
  switch (currentState) {
    
    case STATE_IDLE:
      digitalWrite(RELAY_CUTTER, LOW); 
      stepper.setSpeed(0); // Dừng động cơ
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
      stepper.setSpeed(currentSpeed); // Cập nhật tốc độ mới liên tục, Timer sẽ lo việc phát xung

      if (digitalRead(SENSOR_END) == LOW) {
        currentState = STATE_END_LIMIT;
        digitalWrite(RELAY_CUTTER, LOW); 
        stepper.stop();
        triggerBeep(600); 
      }

      if (triggerStart) { // Ấn lần 2 để tạm dừng
        currentState = STATE_IDLE;
        digitalWrite(RELAY_CUTTER, LOW);
        stepper.stop();
      }
      break;

    case STATE_END_LIMIT:
      stepper.setSpeed(0);
      if (triggerHome) {
        currentState = STATE_HOMING;
        stepper.setSpeed(-1200); 
      }
      break;

    case STATE_HOMING:
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

  // 6. CẬP NHẬT MÀN HÌNH LCD
  if (millis() - lastLcdUpdate > 300) {
    displayStatus();
    lastLcdUpdate = millis();
  }
}

// ================= CÁC HÀM XỬ LÝ CHỨC NĂNG =================

void checkAndPerformOTA() {
  WiFiClientSecure client;
  client.setInsecure(); 

  HTTPClient http;
  if (http.begin(client, URL_VERSION)) {
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      payload.trim(); 
      
      if (payload != CURRENT_VERSION) {  
        lcd.setCursor(0, 2); lcd.print(" DANG TAI PHIEN BAN  ");
        lcd.setCursor(0, 3); lcd.print(" MOI TU GITHUB...    ");

        httpUpdate.update(client, URL_FIRMWARE);
        // Nếu thành công máy tự reset, không cần lệnh xử lý thêm
      }
    }
    http.end();
  }
}

void controlLED() {
  static unsigned long lastLedToggle = 0;
  static bool ledState = false;

  if (currentState == STATE_IDLE) {
    digitalWrite(LED_PIN, LOW); 
  } 
  else if (currentState == STATE_ESTOP) {
    digitalWrite(LED_PIN, HIGH); 
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
