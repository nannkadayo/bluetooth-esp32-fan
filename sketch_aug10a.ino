// 改良版：瞬間値＋適応EWMAで急加速にも追従するRPM表示
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
#define FAN_TACH 16  // TACH入力（割り込みピン）

// BLE UUIDs
#define SERVICE_UUID        "3fb5626e-27b5-4cb9-b6ff-924b70f13215"
#define CHARACTERISTIC_UUID "5dfaf517-f077-428d-b909-de9e321d1667"

unsigned long PIR_OFF_DELAY = 5000;  // 初期値5秒

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

bool deviceConnected = false;
bool ledState = false;
bool childLock = false;
int SPEED = 214;
int speedPercent = 84;  // 修正：初期値を84%に（214/255*100）

const int STOP = 0;
const int PWM_FREQ = 25000;  // 25kHz
const int PWM_RES = 8;       // 8bit
const int PWM_CHANNEL = 0;

enum ModeType { MODE_BUTTON, MODE_PIR };
ModeType currentMode = MODE_BUTTON;

// 安全な下限・上限設定
const unsigned long MIN_PULSE_MICROS = 3000;   // 約3333rpmが限界（チャタ除去）
const unsigned int MIN_VALID_RPM = 30;
const unsigned int MAX_VALID_RPM = 3200;       // P14 Max 実測上限+余裕

volatile unsigned long lastPulseTime = 0;
volatile unsigned long lastDif = 0;
float smoothRPM = 0.0f;  // 平滑化後のRPM
int rpm = 0;             // 表示用RPM

int pwm4pin(int val) {
  return 255 - val;
}

unsigned long lastRPMUpdate = 0;               // 最後にRPM更新した時刻
const unsigned long RPM_UPDATE_MS = 200;       // RPM更新周期（ミリ秒）

// 停止検出用
const unsigned long NO_PULSE_TIMEOUT = 1000;   // ms, これ以上パルスが無ければ停止と判断

void IRAM_ATTR tachISR() {
  unsigned long now = micros();
  unsigned long dif = now - lastPulseTime;
  if (lastPulseTime == 0 || dif >= MIN_PULSE_MICROS) {
    lastDif = dif;
    lastPulseTime = now;  // 最後のパルス時刻（us）
  }
}

void updateRPM_fromLastDif() {
  unsigned long nowMs = millis();
  
  // 一定時間パルスが無ければ停止扱い
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
    uint64_t denom = (uint64_t)dif * 2ULL;  // 2パルスで1回転
    instantRPM = (uint32_t)(60000000ULL / denom);
  }

  if (instantRPM < MIN_VALID_RPM || instantRPM > MAX_VALID_RPM) {
    return;
  }

  // 適応EWMA
  float alpha;
  if (instantRPM > smoothRPM)
    alpha = 0.7f;  // 加速時は追従重視
  else
    alpha = 0.3f;  // 減速/安定時は平滑化重視

  smoothRPM = alpha * instantRPM + (1.0f - alpha) * smoothRPM;
  rpm = (int)(smoothRPM + 0.5f);
}

// ------------------ OLED表示 ------------------
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
  display.print(ledState ? "ON" : "OFF");

  display.setCursor(0, 48);
  display.print("Speed: ");
  display.print(speedPercent);  // 修正：speedPercentをそのまま表示
  display.println("%");

  // RPM表示は下部
  display.setCursor(0, 56);
  display.print("RPM: ");
  display.println(rpm);

  display.display();
}

// ------------------ ファン起動 ------------------
void fanStartup() {
  ledcWrite(PWM_CHANNEL, pwm4pin(SPEED));
  delay(2000);
  ledcWrite(PWM_CHANNEL, pwm4pin(STOP));
  delay(1000);
  ledcWrite(PWM_CHANNEL, pwm4pin(SPEED));
}

// ------------------ BLEコールバック ------------------
bool lastButtonState = HIGH;

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
    std::string value = pCharacteristic->getValue();
    String strValue = String(value.c_str());
    Serial.print("Received BLE: ");
    Serial.println(strValue);

    if (strValue.startsWith("speed ")) {
      int percValue = strValue.substring(6).toInt();
      if (percValue >= 0 && percValue <= 100) {
        speedPercent = percValue;
        // 修正：speedPercentをそのまま0-255にマップ（反転なし）
        SPEED = map(percValue, 0, 100, 0, 255);
        if (ledState) {
          ledcWrite(PWM_CHANNEL, pwm4pin(SPEED));
        }
      }
    } else if (strValue.startsWith("duration ")) {
      int dur = strValue.substring(9).toInt();
      if (dur >= 1 && dur <= 300) {
        PIR_OFF_DELAY = dur * 1000UL;
        Serial.print("PIR OFF delay set to ");
        Serial.print(dur);
        Serial.println(" seconds");
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
      ledcWrite(PWM_CHANNEL, pwm4pin(STOP));
    }
    updateDisplay();
  }
};

// ------------------ setup ------------------
void setup() {
  Serial.begin(115200);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT);
  pinMode(FAN_TACH, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FAN_TACH), tachISR, FALLING);

  // PWM：既存のあなただけのledc関数を使っている前提（そのまま）
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, pwm4pin(STOP));

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.clearDisplay();
  updateDisplay();

  BLEDevice::init("ESP32_Fan");  // 名前を付けるとブラウザ検出が安定
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
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("ESP32 BLE Device ready. Waiting for connections...");
}

// ------------------ loop ------------------
void loop() {
  unsigned long now = millis();

  // RPM更新（表示間隔）
  if (now - lastRPMUpdate >= RPM_UPDATE_MS) {
    updateRPM_fromLastDif();
    updateDisplay();
    Serial.print("Fan RPM: ");
    Serial.println(rpm);
    lastRPMUpdate = now;
  }

  // ボタン/PIR制御（既存ロジック）
  if (currentMode == MODE_BUTTON) {
    bool currentButtonState = digitalRead(BUTTON_PIN);
    if (lastButtonState == HIGH && currentButtonState == LOW && !childLock) {
      ledState = !ledState;
      if (ledState)
        fanStartup();
      else
        ledcWrite(PWM_CHANNEL, pwm4pin(STOP));
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
        ledcWrite(PWM_CHANNEL, pwm4pin(STOP));
        pirOffTime = 0;
      }
    }
  }

  delay(10);  // 高速ループで割り込み/表示応答改善（必要に応じて調整）
}
