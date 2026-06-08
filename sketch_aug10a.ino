/*
 * ファンコントローラー - 最終版（全操作でOLED更新・デュアルコア対応）
 * Core 0: OLED表示専用タスク
 * Core 1: BLE/ESP-NOW/ファン制御メイン処理
 * 
 * 切り替え方法：
 *   #define IS_REMOTE 0  → 本体モード
 *   #define IS_REMOTE 1  → リモコンモード
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// ============================================================
// モード選択
// ============================================================
#define IS_REMOTE 1  // 0:本体モード, 1:リモコンモード

// ============================================================
// ピン定義
// ============================================================
#define OLED_SDA    21
#define OLED_SCL    22

#if !IS_REMOTE
  #define BUTTON_PIN 15
  #define PIR_PIN    18
  #define PWM_PIN     5
  #define FAN_TACH   16
#else
  #define ENC_CLK    16
  #define ENC_DT     17
  #define ENC_SW     18
  #define BUTTON_PIN 15
#endif

// ============================================================
// ESP-NOW設定
// ============================================================
#if IS_REMOTE
  uint8_t partnerMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // 本体のMAC
#else
  uint8_t partnerMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // リモコンのMAC
#endif

// ============================================================
// 共通構造体
// ============================================================
typedef struct {
  bool fanOn;
  bool childLock;
  uint8_t mode;
  uint8_t speedPercent;
  uint16_t rpm;
  uint16_t pirDelay;
  bool bleConnected;
} StatusPacket;

typedef struct {
  uint8_t command;  // 1=ON,2=OFF,3=SPEED_SET,6=MODE,7=LOCK,8=STATUS_REQ
  int value;
} RemoteCommand;

// ============================================================
// BLE設定（本体モードのみ）
// ============================================================
#if !IS_REMOTE
  #define SERVICE_UUID        "3fb5626e-27b5-4cb9-b6ff-924b70f13215"
  #define CHARACTERISTIC_UUID "5dfaf517-f077-428d-b909-de9e321d1667"
  BLEServer* pServer = nullptr;
  BLECharacteristic* pCharacteristic = nullptr;
  bool deviceConnected = false;
#endif

// ============================================================
// OLED表示
// ============================================================
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ============================================================
// グローバル変数
// ============================================================
bool fanOn = false;
bool childLock = false;
uint8_t currentMode = 0;
uint8_t speedPercent = 17;
uint16_t rpm = 0;
uint16_t pirDelay = 5;

StatusPacket status;
unsigned long lastSyncTime = 0;
const unsigned long SYNC_INTERVAL_MS = 200;

// デュアルコア通信用
TaskHandle_t displayTaskHandle = NULL;
volatile bool displayNeedsUpdate = true;  // 表示更新フラグ

// ============================================================
// リモコンモード用変数
// ============================================================
#if IS_REMOTE
  int localSpeedPercent = 50;
  bool localFanOn = false;
  int lastEncoded = 0;
  int encoderSteps = 0;
  unsigned long lastEncoderChange = 0;
  unsigned long lastButtonPress = 0;
  #define DEBOUNCE_MS 50
  #define ENCODER_STEPS_PER_CLICK 2
#endif

// ============================================================
// プロトタイプ宣言
// ============================================================
void updateDisplay();
void sendStatus();
void processRemoteCommand(RemoteCommand& cmd);
void displayTask(void *pvParameters);
void requestDisplayUpdate();

// 表示更新を要求する関数
void requestDisplayUpdate() {
  displayNeedsUpdate = true;
}

// ============================================================
// ファン制御（本体モードのみ）
// ============================================================
#if !IS_REMOTE
  const int STOP = 0;
  const int PWM_FREQ = 25000;
  const int PWM_RES = 8;
  
  int pwm4pin(int val) { return 255 - val; }
  
  void fanStartup() {
    int SPEED = map(speedPercent, 0, 100, 0, 255);
    ledcWrite(PWM_PIN, pwm4pin(SPEED));
    delay(1000);
    requestDisplayUpdate();
  }
  
  void setFanSpeed(int percent) {
    speedPercent = constrain(percent, 0, 100);
    int pwmVal = map(speedPercent, 0, 100, 0, 255);
    if (fanOn) ledcWrite(PWM_PIN, pwm4pin(pwmVal));
    requestDisplayUpdate();
  }
  
  void fanOff() { 
    ledcWrite(PWM_PIN, pwm4pin(STOP));
    requestDisplayUpdate();
  }
  
  volatile unsigned long lastPulseTime = 0;
  volatile unsigned long lastDif = 0;
  float smoothRPM = 0.0f;
  
  void IRAM_ATTR tachISR() {
    unsigned long now = micros();
    unsigned long dif = now - lastPulseTime;
    if (lastPulseTime == 0 || dif >= 3000) {
      lastDif = dif;
      lastPulseTime = now;
    }
  }
  
  void updateRPM() {
    unsigned long oldRpm = rpm;
    if (lastPulseTime != 0 && (millis() - (lastPulseTime / 1000UL) > 1000)) {
      rpm = 0;
      smoothRPM = 0;
    } else {
      noInterrupts();
      unsigned long dif = lastDif;
      interrupts();
      if (dif != 0) {
        uint32_t instantRPM = (uint32_t)(60000000ULL / ((uint64_t)dif * 2ULL));
        if (instantRPM >= 30 && instantRPM <= 3200) {
          float alpha = (instantRPM > smoothRPM) ? 0.7f : 0.3f;
          smoothRPM = alpha * instantRPM + (1.0f - alpha) * smoothRPM;
          rpm = (int)(smoothRPM + 0.5f);
        }
      }
    }
    if (oldRpm != rpm) requestDisplayUpdate();
  }
  
  void handleManualButton() {
    static unsigned long lastButtonState = HIGH;
    bool currentButtonState = digitalRead(BUTTON_PIN);
    if (lastButtonState == HIGH && currentButtonState == LOW && !childLock) {
      fanOn = !fanOn;
      if (fanOn) fanStartup();
      else fanOff();
      sendStatus();
      requestDisplayUpdate();
      delay(50);
    }
    lastButtonState = currentButtonState;
  }
  
  void handlePIR() {
    static unsigned long pirOffTime = 0;
    unsigned long now = millis();
    if (currentMode == 1) {
      bool pirInput = digitalRead(PIR_PIN);
      if (pirInput) {
        if (pirOffTime != 0) {
          pirOffTime = 0;
          requestDisplayUpdate();
        }
        if (!fanOn) {
          fanOn = true;
          fanStartup();
          sendStatus();
          requestDisplayUpdate();
        }
      } else if (fanOn) {
        if (pirOffTime == 0) {
          pirOffTime = now;
          requestDisplayUpdate();
        }
        if (now - pirOffTime >= (pirDelay * 1000UL)) {
          fanOn = false;
          fanOff();
          sendStatus();
          pirOffTime = 0;
          requestDisplayUpdate();
        }
      }
    }
  }
#endif

// ============================================================
// OLED表示（両コアから呼び出し可能）
// ============================================================
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
#if IS_REMOTE
   display.setCursor(0, 0);
  display.print(F("Mode: "));
  display.println(currentMode == 0 ? F("BUTTON") : F("PIR"));

  display.setCursor(0, 12);
  display.print(F("Lock: "));
  display.println(childLock ? F("ON") : F("OFF"));

  display.setCursor(0, 24);
  display.print(F("Fan: "));
  display.println(localFanOn ? F("ON") : F("OFF"));

  display.setCursor(0, 36);
  display.print(F("Speed: "));
  display.print(localSpeedPercent);
  display.println(F("%"));

  display.setCursor(0, 48);
  display.print(F("RPM: "));
  display.println(rpm);

  display.setCursor(80, 0);
 // display.print(deviceConnected ? F("BLE") : F("---"));
#else
  display.setCursor(0, 0);
  display.print(F("Mode: "));
  display.println(currentMode == 0 ? F("BUTTON") : F("PIR"));
  display.setCursor(0, 12);
  display.print(F("Lock: "));
  display.println(childLock ? F("ON") : F("OFF"));
  display.setCursor(0, 24);
  display.print(F("PIR: "));
  display.print(pirDelay);
  display.println(F("s"));
  display.setCursor(0, 36);
  display.print(F("Fan: "));
  display.println(fanOn ? F("ON") : F("OFF"));
  display.setCursor(0, 48);
  display.print(F("Speed: "));
  display.print(speedPercent);
  display.println(F("%"));
  display.setCursor(0, 56);
  display.print(F("RPM: "));
  display.println(rpm);
  display.setCursor(80, 0);
  display.print(deviceConnected ? F("BLE") : F("---"));
#endif
  
  display.display();
}

// ============================================================
// Core 0で動作するOLED表示タスク
// ============================================================
void displayTask(void *pvParameters) {
  unsigned long lastDrawTime = 0;
  
  while(1) {
    if (displayNeedsUpdate && (millis() - lastDrawTime >= 50)) {
      updateDisplay();
      displayNeedsUpdate = false;
      lastDrawTime = millis();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ============================================================
// ESP-NOW通信
// ============================================================
void sendStatus() {
#if !IS_REMOTE
  status.fanOn = fanOn;
  status.childLock = childLock;
  status.mode = currentMode;
  status.speedPercent = speedPercent;
  status.rpm = rpm;
  status.pirDelay = pirDelay;
  status.bleConnected = deviceConnected;
  esp_now_send(partnerMac, (uint8_t*)&status, sizeof(status));
#endif
}

#if !IS_REMOTE
void processRemoteCommand(RemoteCommand& cmd) {
  bool changed = false;
  switch(cmd.command) {
    case 1: 
      fanOn = true; 
      fanStartup(); 
      changed = true;
      break;
    case 2: 
      fanOn = false; 
      fanOff(); 
      changed = true;
      break;
    case 3: 
      if (speedPercent != constrain(cmd.value, 0, 100)) {
        speedPercent = constrain(cmd.value, 0, 100);
        if (fanOn) setFanSpeed(speedPercent);
        changed = true;
      }
      break;
    case 6: 
      currentMode = (currentMode == 0) ? 1 : 0; 
      changed = true;
      break;
    case 7: 
      childLock = !childLock; 
      changed = true;
      break;
    case 8: 
      sendStatus(); 
      break;
  }
  if (changed) {
    sendStatus();
    requestDisplayUpdate();
  }
}
#endif

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#if IS_REMOTE
  if (len == sizeof(StatusPacket)) {
    StatusPacket received;
    memcpy(&received, data, sizeof(received));
    bool changed = false;
    if (millis() - lastEncoderChange > 500) {
      if (localSpeedPercent != received.speedPercent) {
        localSpeedPercent = received.speedPercent;
        changed = true;
      }
      if (localFanOn != received.fanOn) {
        localFanOn = received.fanOn;
        changed = true;
      }
      if (fanOn != received.fanOn) {
        fanOn = received.fanOn;
        changed = true;
      }
      if (speedPercent != received.speedPercent) {
        speedPercent = received.speedPercent;
        changed = true;
      }
      if (rpm != received.rpm) {
        rpm = received.rpm;
        changed = true;
      }
      if (currentMode != received.mode) {
        currentMode = received.mode;
        changed = true;
      }
      if (childLock != received.childLock) {
        childLock = received.childLock;
        changed = true;
      }
      if (pirDelay != received.pirDelay) {
        pirDelay = received.pirDelay;
        changed = true;
      }
    }
    if (changed) requestDisplayUpdate();
  }
#else
  if (len == sizeof(RemoteCommand)) {
    RemoteCommand cmd;
    memcpy(&cmd, data, sizeof(cmd));
    processRemoteCommand(cmd);
  }
#endif
}

// ============================================================
// リモコンモード処理
// ============================================================
#if IS_REMOTE
void updateEncoder() {
  int msb = digitalRead(ENC_CLK);
  int lsb = digitalRead(ENC_DT);
  int encoded = (msb << 1) | lsb;
  int sum = (lastEncoded << 2) | encoded;
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderSteps++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderSteps--;
  lastEncoded = encoded;
  
  int delta = 0;
  if (encoderSteps >= ENCODER_STEPS_PER_CLICK) { delta = 1; encoderSteps = 0; }
  if (encoderSteps <= -ENCODER_STEPS_PER_CLICK) { delta = -1; encoderSteps = 0; }
  
  if (delta != 0) {
    lastEncoderChange = millis();
    int newSpeed = localSpeedPercent + (delta * 2);
    newSpeed = constrain(newSpeed, 0, 100);
    if (newSpeed != localSpeedPercent) {
      localSpeedPercent = newSpeed;
      RemoteCommand cmd;
      cmd.command = 3;
      cmd.value = localSpeedPercent;
      esp_now_send(partnerMac, (uint8_t*)&cmd, sizeof(cmd));
      requestDisplayUpdate();
    }
  }
}

void handleEncoderButton() {
  if (digitalRead(ENC_SW) == LOW) {
    unsigned long now = millis();
    if (now - lastButtonPress > DEBOUNCE_MS) {
      lastButtonPress = now;
      
      // ★ ON/OFF切り替え（モード変更から変更）
      localFanOn = !localFanOn;
      
      RemoteCommand cmd;
      cmd.command = localFanOn ? 1 : 2;  // 1=ON, 2=OFF
      cmd.value = localFanOn ? 1 : 0;
      esp_now_send(partnerMac, (uint8_t*)&cmd, sizeof(cmd));
      requestDisplayUpdate();
      
      Serial.print(F("Fan: "));
      Serial.println(localFanOn ? F("ON") : F("OFF"));
    }
    while (digitalRead(ENC_SW) == LOW) delay(5);
  }

}

void handleRemoteButton() {
  static unsigned long lastRemotePress = 0;
  if (digitalRead(BUTTON_PIN) == LOW) {
    unsigned long now = millis();
    if (now - lastRemotePress > DEBOUNCE_MS) {
      lastRemotePress = now;
      RemoteCommand cmd;
      cmd.command = 6;
      cmd.value = 0;
      esp_now_send(partnerMac, (uint8_t*)&cmd, sizeof(cmd));
      requestDisplayUpdate();
    }
    while (digitalRead(BUTTON_PIN) == LOW) delay(5);
  }
}
#endif

// ============================================================
// BLE初期化（本体モードのみ）
// ============================================================
#if !IS_REMOTE
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override { 
    deviceConnected = true; 
    sendStatus();
    requestDisplayUpdate();
  }
  void onDisconnect(BLEServer* pServer) override { 
    deviceConnected = false; 
    sendStatus();
    requestDisplayUpdate();
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String strValue = pCharacteristic->getValue();
    bool changed = false;
    
    if (strValue.startsWith("speed ")) {
      int val = strValue.substring(6).toInt();
      if (speedPercent != constrain(val, 0, 100)) {
        speedPercent = constrain(val, 0, 100);
        if (fanOn) setFanSpeed(speedPercent);
        changed = true;
      }
    } else if (strValue.startsWith("duration ")) {
      int val = constrain(strValue.substring(9).toInt(), 1, 300);
      if (pirDelay != val) {
        pirDelay = val;
        changed = true;
      }
    } else if (strValue == "lock on") {
      if (!childLock) {
        childLock = true;
        changed = true;
      }
    } else if (strValue == "lock off") {
      if (childLock) {
        childLock = false;
        changed = true;
      }
    } else if (strValue == "mode pir") {
      if (currentMode != 1) {
        currentMode = 1;
        changed = true;
      }
    } else if (strValue == "mode button") {
      if (currentMode != 0) {
        currentMode = 0;
        changed = true;
      }
    } else if (strValue == "on") {
      if (!fanOn) {
        fanOn = true;
        fanStartup();
        changed = true;
      }
    } else if (strValue == "off") {
      if (fanOn) {
        fanOn = false;
        fanOff();
        changed = true;
      }
    }
    
    if (changed) {
      sendStatus();
      requestDisplayUpdate();
    }
  }
};

void initBLE() {
  BLEDevice::init("ESP32_Fan");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}
#endif

// ============================================================
// ESP-NOW初期化
// ============================================================
// ============================================================
// ESP-NOW初期化（修正版）
// ============================================================
// initESP_NOW()関数を修正
void initESP_NOW() {
  WiFi.mode(WIFI_STA);
  
  // ★ チャンネルを固定（両方のESP32で同じ番号に）
 WiFi.setChannel(6);  // WiFiライブラリの関数
  delay(100);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("ESP-NOW init failed"));
    while (true) delay(1000);
  }
  
  esp_now_register_recv_cb(onDataRecv);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, partnerMac, 6);
  peerInfo.channel = 6;  // ★ チャンネルを明示的に指定
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(F("Peer add failed"));
  } else {
    Serial.println(F("Peer added"));
    Serial.print(F("Partner MAC: "));
    for(int i=0; i<6; i++) {
      Serial.printf("%02X", partnerMac[i]);
      if(i<5) Serial.print(":");
    }
    Serial.println();
  }
}

// ============================================================
// セットアップ
// ============================================================
// ============================================================
// セットアップ（修正版）
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  
  // ★ 1. 最初にWiFiをSTAモードで起動（MACアドレス取得のため）
  WiFi.mode(WIFI_STA);
  delay(100);  // WiFiスタック安定待ち
  
  // ★ 2. MACアドレス表示（ここで正しく取得できる）
  Serial.println(F("================================"));
  Serial.print(F("Device MAC Address: "));
  Serial.println(WiFi.macAddress());
  Serial.print(F("Mode: "));
  Serial.println(IS_REMOTE ? F("REMOTE") : F("MAIN"));
  Serial.println(F("================================"));
  
  // I2C高速化
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  
  // ピンモード設定
#if !IS_REMOTE
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(FAN_TACH, INPUT);
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  fanOff();
  attachInterrupt(digitalPinToInterrupt(FAN_TACH), tachISR, FALLING);
#else
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastEncoded = (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT);
#endif
  
  // OLED初期化
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 28);
  display.println(IS_REMOTE ? F("REMOTE MODE") : F("MAIN MODE"));
  display.display();
  delay(1000);
  
  // ★ 3. ESP-NOW初期化（WiFiは既にSTAモード）
  initESP_NOW();
  
#if !IS_REMOTE
  initBLE();
#endif
  
  // Core 0でOLED表示タスクを開始
  xTaskCreatePinnedToCore(
    displayTask,
    "DisplayTask",
    4096,
    NULL,
    1,
    &displayTaskHandle,
    0  // Core 0
  );
  
  displayNeedsUpdate = true;
}

// ============================================================
// メインループ（Core 1で実行）
// ============================================================
void loop() {
#if IS_REMOTE
  updateEncoder();
  handleEncoderButton();
  handleRemoteButton();
  
  static unsigned long lastRequest = 0;
  if (millis() - lastRequest >= 10000) {
    RemoteCommand cmd;
    cmd.command = 8;
    cmd.value = 0;
    esp_now_send(partnerMac, (uint8_t*)&cmd, sizeof(cmd));
    lastRequest = millis();
  }
#else
  static unsigned long lastRPMUpdate = 0;
  if (millis() - lastRPMUpdate >= 200) {
    updateRPM();
    lastRPMUpdate = millis();
  }
  
  handleManualButton();
  handlePIR();
  
  static unsigned long lastStatusSend = 0;
  if (millis() - lastStatusSend >= SYNC_INTERVAL_MS) {
    sendStatus();
    lastStatusSend = millis();
  }
#endif
  
  delay(5);
}
