// 改良版：4ピンファン対応＋S8050オープンコレクタ駆動用コード（ESP32 Core v3.x対応・エラー修正版）
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define LED_PIN 4
#define BUTTON_PIN 15
#define PIR_PIN 18
#define PWM_PIN 5
#define FAN_TACH 16  // TACH入力

// BLE UUIDs
#define SERVICE_UUID        "3fb5626e-27b5-4cb9-b6ff-924b70f13215"
#define CHARACTERISTIC_UUID "5dfaf517-f077-428d-b909-de9e321d1667"

unsigned long PIR_OFF_DELAY = 5000;

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ★【復元】モード管理の定義と変数
enum ModeType { MODE_BUTTON, MODE_PIR };
ModeType currentMode = MODE_BUTTON;

bool deviceConnected = false;
bool ledState = false;
bool childLock = false;
int SPEED = 43;  // 17%に相当する値
int speedPercent = 17;

const int STOP = 0;
const int PWM_FREQ = 25000;  // 4ピンファン標準の25kHz
const int PWM_RES = 8;       

// S8050で引き下げるため、255-valで正論理（100%で全開）
int pwm4pin(int val) {
  return 255 - val;
}

unsigned long lastRPMUpdate = 0;               
const unsigned long RPM_UPDATE_MS = 200;       
const unsigned long NO_PULSE_TIMEOUT = 1000;   

volatile unsigned long lastPulseTime = 0;
volatile unsigned long lastDif = 0;
float smoothRPM = 0.0f;  
int rpm = 0;             

void IRAM_ATTR tachISR() {
  unsigned long now = micros();
  unsigned long dif = now - lastPulseTime;
  if (lastPulseTime == 0 || dif >= 3000) { 
    lastDif = dif;
    lastPulseTime = now;  
  }
}

void updateRPM_fromLastDif() {
  unsigned long nowMs = millis();
  
  if (lastPulseTime != 0 && (nowMs - (lastPulseTime / 1000UL) > NO_PULSE_TIMEOUT)) {
    rpm = 0;
    smoothRPM = 0;
    return;
  }

  noInterrupts();
  unsigned long dif = lastDif;
  interrupts();

  if (dif == 0) return;

  uint32_t instantRPM = 0;
  if (dif > 0) {
    uint64_t denom = (uint64_t)dif * 2ULL;  
    instantRPM = (uint32_t)(60000000ULL / denom);
  }

  if (instantRPM < 30 || instantRPM > 3200) { 
    return;
  }

  float alpha;
  if (instantRPM > smoothRPM)
    alpha = 0.7f;  
  else
    alpha = 0.3f;  

  smoothRPM = alpha * instantRPM + (1.0f - alpha) * smoothRPM;
  rpm = (int)(smoothRPM + 0.5f);
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Mode: ");
  display.println(currentMode == MODE_BUTTON ? "BUTTON" : "PIR");

  display.setCursor(0, 12);
  display.print("Child Lock: ");
  display.println(childLock ? "ON" : "OFF");

  display.setCursor(0, 24);
  display.print("PIR duration: ");
  display.print(PIR_OFF_DELAY / 1000);
  display.println("s");

  display.setCursor(0, 36);
  display.print("Fan: ");
  display.println(ledState ? "ON" : "OFF");

  display.setCursor(0, 48);
  display.print("Speed: ");
  display.print(speedPercent);  
  display.println("%");

  display.setCursor(0, 56);
  display.print("RPM: ");
  display.println(rpm);

  display.display();
}

void fanStartup() {
  ledcWrite(PWM_PIN, pwm4pin(SPEED)); 
  delay(1000); 
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("BLE client disconnected");
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String strValue = pCharacteristic->getValue();
    Serial.print("Received BLE: ");
    Serial.println(strValue);

    if (strValue.startsWith("speed ")) {
      int percValue = strValue.substring(6).toInt();
      if (percValue >= 0 && percValue <= 100) {
        speedPercent = percValue;
        SPEED = map(percValue, 0, 100, 0, 255);
        if (ledState) {
          ledcWrite(PWM_PIN, pwm4pin(SPEED));
        }
      }
    } else if (strValue.startsWith("duration ")) {
      int dur = strValue.substring(9).toInt();
      if (dur >= 1 && dur <= 300) {
        PIR_OFF_DELAY = dur * 1000UL;
      }
    } else if (strValue == "lock on")
      childLock = true;
    else if (strValue == "lock off")
      childLock = false;
    else if (strValue == "mode pir") {
      currentMode = MODE_PIR;
    } else if (strValue == "mode button") {
      currentMode = MODE_BUTTON;
    } else if (strValue == "on") {
      ledState = true;
      fanStartup();
    } else if (strValue == "off") {
      ledState = false;
      ledcWrite(PWM_PIN, pwm4pin(STOP)); 
    }
    updateDisplay();
  }
};

void setup() {
  Serial.begin(115200);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT);
  pinMode(FAN_TACH, INPUT); 

  attachInterrupt(digitalPinToInterrupt(FAN_TACH), tachISR, FALLING);

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES); 
  ledcWrite(PWM_PIN, pwm4pin(STOP)); 

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.clearDisplay();
  updateDisplay();

  BLEDevice::init("ESP32_Fan");  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("4-Pin Fan Controller Ready...");
}

void loop() {
  unsigned long now = millis();

  if (now - lastRPMUpdate >= RPM_UPDATE_MS) {
    updateRPM_fromLastDif();
    updateDisplay();
    lastRPMUpdate = now;
  }

  if (currentMode == MODE_BUTTON) {
    bool currentButtonState = digitalRead(BUTTON_PIN);
    static bool lastButtonState = HIGH;
    if (lastButtonState == HIGH && currentButtonState == LOW && !childLock) {
      ledState = !ledState;
      if (ledState)
        fanStartup();
      else
        ledcWrite(PWM_PIN, pwm4pin(STOP));
    }
    lastButtonState = currentButtonState;
  } else if (currentMode == MODE_PIR) {
    bool pirInput = digitalRead(PIR_PIN);
    static unsigned long pirOffTime = 0;
    if (pirInput) {
      pirOffTime = 0;
      if (!ledState) {
        ledState = true;
        fanStartup();
      }
    } else if (ledState) {
      if (pirOffTime == 0) pirOffTime = millis();
      if (millis() - pirOffTime >= PIR_OFF_DELAY) {
        ledState = false;
        ledcWrite(PWM_PIN, pwm4pin(STOP));
        pirOffTime = 0;
      }
    }
  }

  delay(10);  
}
