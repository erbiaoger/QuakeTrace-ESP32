#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <stdio.h>
#include <string.h>
#include "config.h"

// 单次触发事件的数据结构：
// 记录触发时刻（ms/us）和触发/峰值 ADC，便于后续导出分析。
struct TriggerEvent {
  unsigned long localMillis;
  unsigned long localMicros;
  uint16_t triggerAdc;
  uint16_t peakAdc;
};

// 事件缓存与系统状态变量。
TriggerEvent events[MAX_EVENTS];
int eventCount = 0;
int eventWriteIndex = 0;
unsigned long totalEventCount = 0;
unsigned long lastTriggerMillis = 0;
unsigned long lastHeartbeatMillis = 0;
unsigned long lastDisplayMillis = 0;
bool triggerArmed = true;
bool oledReady = false;
bool isDumpingData = false;
bool signalStreamEnabled = SIGNAL_STREAM_DEFAULT_ON;
unsigned long signalStreamIntervalMs = SIGNAL_STREAM_DEFAULT_INTERVAL_MS;
unsigned long lastSignalStreamMs = 0;
uint16_t baselineAdc = 0;
uint16_t triggerDelta = PIEZO_TRIGGER_THRESHOLD;
uint16_t rearmDelta = PIEZO_REARM_THRESHOLD;
bool thresholdsAutoCalibrated = false;
uint8_t oledI2cAddr = 0;
uint8_t oledTextRows = 8;
uint8_t oledDriverMode = (uint8_t)OLED_DRIVER_MODE;
U8X8_SSD1306_128X64_NONAME_HW_I2C oledSsd1306_64(U8X8_PIN_NONE);
U8X8_SH1106_128X64_NONAME_HW_I2C oledSh1106_64(U8X8_PIN_NONE);
U8X8_SSD1306_128X32_UNIVISION_HW_I2C oledSsd1306_32(U8X8_PIN_NONE);
U8X8 *oled = nullptr;
bool flashLogReady = false;
unsigned long flashWriteFailCount = 0;
bool wifiConnected = false;
unsigned long lastWifiRetryMs = 0;
bool bleReady = false;
bool bleClientConnected = false;
BLEServer *bleServer = nullptr;
BLECharacteristic *bleCharacteristic = nullptr;
bool bleDumpInProgress = false;
File bleDumpFile;
unsigned long lastBleDumpNotifyMs = 0;
volatile bool bleReqHelp = false;
volatile bool bleReqStatus = false;
volatile bool bleReqDumpFlash = false;
volatile bool bleReqStopDump = false;
volatile bool bleReqClearFlash = false;
volatile bool bleReqUnknown = false;
char bleUnknownCmd[40] = {0};

static void clearFlashCsv();

// OLED 驱动模式转文本，便于日志与状态输出。
static const char *oledDriverName(uint8_t mode) {
  switch (mode) {
    case OLED_DRIVER_MODE_SSD1306_128X64: return "SSD1306_128x64";
    case OLED_DRIVER_MODE_SH1106_128X64: return "SH1106_128x64";
    case OLED_DRIVER_MODE_SSD1306_128X32: return "SSD1306_128x32";
    default: return "UNKNOWN";
  }
}

// 判断 Wi-Fi 账号配置是否有效。
static bool wifiCredentialReady() {
  if (strlen(WIFI_SSID) == 0) return false;
  if (strcmp(WIFI_SSID, "YOUR_WIFI_SSID") == 0) return false;
  return true;
}

// BLE 写入命令规范化：去前后空白并转小写。
static void normalizeBleCommand(char *s) {
  if (s == nullptr) return;

  size_t start = 0;
  while (s[start] == ' ' || s[start] == '\t' || s[start] == '\r' || s[start] == '\n') start++;
  if (start > 0) {
    size_t i = 0;
    while (s[start + i] != '\0') {
      s[i] = s[start + i];
      i++;
    }
    s[i] = '\0';
  }

  size_t len = strlen(s);
  while (len > 0 &&
         (s[len - 1] == ' ' || s[len - 1] == '\t' || s[len - 1] == '\r' || s[len - 1] == '\n')) {
    s[len - 1] = '\0';
    len--;
  }

  for (size_t i = 0; s[i] != '\0'; i++) {
    if (s[i] >= 'A' && s[i] <= 'Z') {
      s[i] = (char)(s[i] - 'A' + 'a');
    }
  }
}

// 约束信号流输出周期，避免刷屏过快或过慢。
static unsigned long clampSignalStreamInterval(unsigned long ms) {
  if (ms < SIGNAL_STREAM_MIN_INTERVAL_MS) return SIGNAL_STREAM_MIN_INTERVAL_MS;
  if (ms > SIGNAL_STREAM_MAX_INTERVAL_MS) return SIGNAL_STREAM_MAX_INTERVAL_MS;
  return ms;
}

// 信号流当前采样频率（Hz）。
static unsigned long signalStreamHz() {
  if (signalStreamIntervalMs == 0) return 0;
  return 1000UL / signalStreamIntervalMs;
}

// 开关实时信号流。
static void setSignalStream(bool on) {
  signalStreamEnabled = on;
  lastSignalStreamMs = 0;
  if (signalStreamEnabled) {
    Serial.print("Signal stream ON, interval(ms)=");
    Serial.print(signalStreamIntervalMs);
    Serial.print(",hz=");
    Serial.println(signalStreamHz());
    Serial.println("sig_ms,adc,delta,base,td,rd,armed");
  } else {
    Serial.println("Signal stream OFF");
  }
}

// 解析并处理 stream 命令。
static void handleStreamCommand(const char *cmd) {
  if (strcmp(cmd, "stream") == 0) {
    Serial.print("Signal stream: ");
    Serial.println(signalStreamEnabled ? "ON" : "OFF");
    Serial.print("interval(ms)=");
    Serial.print(signalStreamIntervalMs);
    Serial.print(",hz=");
    Serial.println(signalStreamHz());
    Serial.println("Usage: stream on | stream off | stream ms <20-200> | stream hz <1-50>");
    return;
  }

  if (strcmp(cmd, "stream on") == 0) {
    setSignalStream(true);
    return;
  }
  if (strcmp(cmd, "stream off") == 0) {
    setSignalStream(false);
    return;
  }

  unsigned int ms = 0;
  if (sscanf(cmd, "stream ms %u", &ms) == 1) {
    signalStreamIntervalMs = clampSignalStreamInterval((unsigned long)ms);
    setSignalStream(true);
    return;
  }

  unsigned int hz = 0;
  if (sscanf(cmd, "stream hz %u", &hz) == 1 && hz > 0) {
    unsigned long derivedMs = 1000UL / (unsigned long)hz;
    if (derivedMs == 0) derivedMs = 1;
    signalStreamIntervalMs = clampSignalStreamInterval(derivedMs);
    setSignalStream(true);
    return;
  }

  Serial.println("Unknown stream command. Usage: stream on|off|ms <20-200>|hz <1-50>");
}

// 连接 Wi-Fi（STA 模式）。
static bool connectWifi() {
  if (!ENABLE_WIFI) return false;

  if (!wifiCredentialReady()) {
    Serial.println("WiFi disabled: SSID not configured.");
    wifiConnected = false;
    return false;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    return true;
  }

  Serial.print("WiFi connecting: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED &&
         (millis() - startMs) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
  }

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiConnected) {
    Serial.print("WiFi connected, IP=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect timeout.");
  }
  return wifiConnected;
}

// 周期维持 Wi-Fi 连接。
static void maintainWifi() {
  if (!ENABLE_WIFI) return;
  if (!wifiCredentialReady()) return;

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    return;
  }

  wifiConnected = false;
  const unsigned long nowMs = millis();
  if ((unsigned long)(nowMs - lastWifiRetryMs) < WIFI_RETRY_INTERVAL_MS) return;
  lastWifiRetryMs = nowMs;
  connectWifi();
}

class DasBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    (void)pServer;
    bleClientConnected = true;
    Serial.println("BLE client connected.");
  }

  void onDisconnect(BLEServer *pServer) override {
    (void)pServer;
    bleClientConnected = false;
    if (bleServer != nullptr) {
      bleServer->startAdvertising();
    }
    Serial.println("BLE client disconnected.");
  }
};

class DasBleCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    if (pCharacteristic == nullptr) return;

    const std::string value = pCharacteristic->getValue();
    if (value.empty()) return;

    char cmd[48];
    size_t n = value.size();
    if (n >= sizeof(cmd)) n = sizeof(cmd) - 1;
    memcpy(cmd, value.data(), n);
    cmd[n] = '\0';
    normalizeBleCommand(cmd);
    if (cmd[0] == '\0') return;

    if (strcmp(cmd, "help") == 0) {
      bleReqHelp = true;
    } else if (strcmp(cmd, "status") == 0) {
      bleReqStatus = true;
    } else if (strcmp(cmd, "dumpflash") == 0 || strcmp(cmd, "dumpcsv") == 0) {
      bleReqDumpFlash = true;
    } else if (strcmp(cmd, "stopdump") == 0) {
      bleReqStopDump = true;
    } else if (strcmp(cmd, "clearflash") == 0) {
      bleReqClearFlash = true;
    } else {
      strncpy(bleUnknownCmd, cmd, sizeof(bleUnknownCmd) - 1);
      bleUnknownCmd[sizeof(bleUnknownCmd) - 1] = '\0';
      bleReqUnknown = true;
    }
  }
};

// 初始化 BLE：创建一个带 Notify 的特征值。
static void initBle() {
  if (!ENABLE_BLE) return;

  BLEDevice::init(BLE_DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  if (bleServer == nullptr) {
    Serial.println("BLE server create failed.");
    bleReady = false;
    return;
  }

  bleServer->setCallbacks(new DasBleServerCallbacks());
  BLEService *service = bleServer->createService(BLE_SERVICE_UUID);
  if (service == nullptr) {
    Serial.println("BLE service create failed.");
    bleReady = false;
    return;
  }

  bleCharacteristic = service->createCharacteristic(
      BLE_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR);
  if (bleCharacteristic == nullptr) {
    Serial.println("BLE characteristic create failed.");
    bleReady = false;
    return;
  }

  bleCharacteristic->setCallbacks(new DasBleCharacteristicCallbacks());
  bleCharacteristic->addDescriptor(new BLE2902());
  bleCharacteristic->setValue("DAS ready");
  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  bleReady = true;
  Serial.print("BLE advertising as ");
  Serial.println(BLE_DEVICE_NAME);
}

// 通过 BLE 发送一条状态文本（若有已连接客户端）。
static void bleNotify(const char *message) {
  if (!ENABLE_BLE || !bleReady) return;
  if (!bleClientConnected || bleCharacteristic == nullptr) return;
  if (message == nullptr) return;

  bleCharacteristic->setValue((uint8_t *)message, strlen(message));
  bleCharacteristic->notify();
}

// 开始通过 BLE 分段发送 Flash CSV。
static void startBleDumpFlash() {
  if (!ENABLE_BLE || !bleReady || !bleClientConnected) return;
  if (!ENABLE_FLASH_LOG || !flashLogReady) {
    bleNotify("ERR:flash log not ready");
    return;
  }
  if (bleDumpInProgress) {
    bleNotify("ERR:dump already running");
    return;
  }

  bleDumpFile = SPIFFS.open(FLASH_LOG_PATH, FILE_READ);
  if (!bleDumpFile) {
    bleNotify("ERR:open flash log fail");
    return;
  }
  bleDumpInProgress = true;
  lastBleDumpNotifyMs = 0;
  bleNotify("#BEGIN_FLASH_CSV");
}

// 停止 BLE Flash 导出。
static void stopBleDumpFlash(bool notifyStop) {
  if (bleDumpInProgress) {
    bleDumpFile.close();
    bleDumpInProgress = false;
  }
  if (notifyStop) {
    bleNotify("#END_FLASH_CSV");
  }
}

// 在 loop 中持续发送 Flash CSV（逐行）。
static void processBleDumpFlash() {
  if (!bleDumpInProgress) {
    if (isDumpingData) isDumpingData = false;
    return;
  }

  isDumpingData = true;
  if (!bleClientConnected) {
    stopBleDumpFlash(false);
    isDumpingData = false;
    return;
  }

  const unsigned long nowMs = millis();
  if ((unsigned long)(nowMs - lastBleDumpNotifyMs) < 20) return;
  lastBleDumpNotifyMs = nowMs;

  if (!bleDumpFile || !bleDumpFile.available()) {
    stopBleDumpFlash(true);
    isDumpingData = false;
    return;
  }

  String line = bleDumpFile.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  bleNotify(line.c_str());
}

// 处理 BLE 写命令。
static void processBleRequests() {
  if (bleReqHelp) {
    bleReqHelp = false;
    bleNotify("cmd:help|status|dumpflash|stopdump|clearflash");
  }
  if (bleReqStatus) {
    bleReqStatus = false;
    char msg[120];
    snprintf(msg, sizeof(msg), "status,ev=%lu,ram=%d,wifi=%s,flash=%s,dump=%s",
             totalEventCount, eventCount,
             (WiFi.status() == WL_CONNECTED) ? "on" : "off",
             (flashLogReady ? "on" : "off"),
             (bleDumpInProgress ? "on" : "off"));
    bleNotify(msg);
  }
  if (bleReqClearFlash) {
    bleReqClearFlash = false;
    clearFlashCsv();
    bleNotify("flash log cleared");
  }
  if (bleReqStopDump) {
    bleReqStopDump = false;
    stopBleDumpFlash(true);
  }
  if (bleReqDumpFlash) {
    bleReqDumpFlash = false;
    startBleDumpFlash();
  }
  if (bleReqUnknown) {
    bleReqUnknown = false;
    char msg[96];
    snprintf(msg, sizeof(msg), "ERR:unknown cmd=%s", bleUnknownCmd);
    bleNotify(msg);
  }
}

// 扫描 I2C 总线，返回 OLED 常用地址（0x3C/0x3D），未找到则返回 0。
static uint8_t findOledI2cAddress() {
  uint8_t oledAddr = 0;
  int foundCount = 0;

  Serial.println("I2C scan start...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission();
    if (err == 0) {
      foundCount++;
      Serial.print("I2C device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      if (addr == 0x3C || addr == 0x3D) {
        oledAddr = addr;
      }
    }
  }

  if (foundCount == 0) {
    Serial.println("I2C scan done: no device found.");
  } else {
    Serial.print("I2C scan done, total devices: ");
    Serial.println(foundCount);
  }
  return oledAddr;
}

// 激活指定 OLED 驱动，并完成基本初始化。
static bool activateOledDriver(uint8_t mode) {
  U8X8 *selected = nullptr;
  uint8_t rows = 8;

  switch (mode) {
    case OLED_DRIVER_MODE_SSD1306_128X64:
      selected = &oledSsd1306_64;
      rows = 8;
      break;
    case OLED_DRIVER_MODE_SH1106_128X64:
      selected = &oledSh1106_64;
      rows = 8;
      break;
    case OLED_DRIVER_MODE_SSD1306_128X32:
      selected = &oledSsd1306_32;
      rows = 4;
      break;
    default:
      return false;
  }

  if (oledI2cAddr == 0) return false;

  // U8x8 使用 8-bit 地址格式，因此这里左移 1 位。
  selected->setI2CAddress((uint8_t)(oledI2cAddr << 1));
  selected->begin();
  selected->setPowerSave(0);
  selected->setFlipMode(0);
  selected->setContrast(OLED_CONTRAST);
  selected->setFont(u8x8_font_chroma48medium8_r);
  selected->clearDisplay();

  oled = selected;
  oledTextRows = rows;
  oledDriverMode = mode;
  oledReady = true;
  return true;
}

// 把命令字符串转小写，便于大小写无关地解析串口命令。
static void toLowerInPlace(char *s) {
  for (size_t i = 0; s[i] != '\0'; i++) {
    if (s[i] >= 'A' && s[i] <= 'Z') {
      s[i] = (char)(s[i] - 'A' + 'a');
    }
  }
}

// 去掉命令前后空白字符，避免串口输入带空格时无法匹配。
static void trimInPlace(char *s) {
  size_t start = 0;
  while (s[start] == ' ' || s[start] == '\t') start++;

  if (start > 0) {
    size_t i = 0;
    while (s[start + i] != '\0') {
      s[i] = s[start + i];
      i++;
    }
    s[i] = '\0';
  }

  size_t len = strlen(s);
  while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t')) {
    s[len - 1] = '\0';
    len--;
  }
}

// 从串口读取一行（以换行结尾），成功返回 true。
// 用静态缓冲区累积字符，避免每轮 loop 都重新分配内存。
static bool readLine(char *out, size_t outSize) {
  static char buf[48];
  static size_t idx = 0;

  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[idx] = '\0';
      strncpy(out, buf, outSize - 1);
      out[outSize - 1] = '\0';
      idx = 0;
      return true;
    }
    if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
  return false;
}

// 计算两个无符号 16 位数的绝对差值。
static uint16_t absDiffU16(uint16_t a, uint16_t b) {
  return (a >= b) ? (a - b) : (b - a);
}

// 将“按时间顺序的第 order 条事件”映射到环形缓冲数组下标。
static int ringIndexByOrder(int order) {
  if (eventCount <= 0) return -1;
  if (order < 0 || order >= eventCount) return -1;
  int oldest = eventWriteIndex - eventCount;
  while (oldest < 0) oldest += MAX_EVENTS;
  return (oldest + order) % MAX_EVENTS;
}

// 获取最新一条事件，存在则返回 true。
static bool getLatestEvent(TriggerEvent &out) {
  if (eventCount <= 0) return false;
  int idx = eventWriteIndex - 1;
  if (idx < 0) idx += MAX_EVENTS;
  out = events[idx];
  return true;
}

// 初始化内部 Flash 日志文件（SPIFFS）。
static void initFlashLog() {
  if (!ENABLE_FLASH_LOG) return;

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS init failed. Flash logging disabled.");
    flashLogReady = false;
    return;
  }

  flashLogReady = true;
  if (!SPIFFS.exists(FLASH_LOG_PATH)) {
    File f = SPIFFS.open(FLASH_LOG_PATH, FILE_WRITE);
    if (!f) {
      Serial.println("Cannot create flash log file.");
      flashLogReady = false;
      return;
    }
    f.println("index,local_ms,local_us,trigger_adc,peak_adc");
    f.close();
  }
}

// 追加写入一条触发事件到 Flash CSV。
static void appendEventToFlash(unsigned long index, const TriggerEvent &ev) {
  if (!ENABLE_FLASH_LOG || !flashLogReady) return;

  File f = SPIFFS.open(FLASH_LOG_PATH, FILE_APPEND);
  if (!f) {
    flashWriteFailCount++;
    return;
  }

  f.print(index);
  f.print(",");
  f.print(ev.localMillis);
  f.print(",");
  f.print(ev.localMicros);
  f.print(",");
  f.print(ev.triggerAdc);
  f.print(",");
  f.println(ev.peakAdc);
  f.close();
}

// 导出 Flash CSV（全量历史）。
static void dumpFlashCsv() {
  if (!ENABLE_FLASH_LOG || !flashLogReady) {
    Serial.println("Flash log is not ready.");
    return;
  }

  File f = SPIFFS.open(FLASH_LOG_PATH, FILE_READ);
  if (!f) {
    Serial.println("Cannot open flash log file.");
    return;
  }

  isDumpingData = true;
  while (f.available()) {
    uint8_t buf[128];
    const size_t n = f.read(buf, sizeof(buf));
    if (n == 0) break;
    Serial.write(buf, n);
  }
  f.close();
  isDumpingData = false;
}

// 清空 Flash CSV，仅保留表头。
static void clearFlashCsv() {
  if (!ENABLE_FLASH_LOG || !flashLogReady) {
    Serial.println("Flash log is not ready.");
    return;
  }

  if (bleDumpInProgress) {
    stopBleDumpFlash(false);
  }

  SPIFFS.remove(FLASH_LOG_PATH);
  File f = SPIFFS.open(FLASH_LOG_PATH, FILE_WRITE);
  if (!f) {
    Serial.println("Recreate flash log file failed.");
    return;
  }
  f.println("index,local_ms,local_us,trigger_adc,peak_adc");
  f.close();
  Serial.println("Flash log cleared.");
}

// 读取 Flash 日志文件大小（字节）。
static size_t flashLogSizeBytes() {
  if (!ENABLE_FLASH_LOG || !flashLogReady) return 0;
  File f = SPIFFS.open(FLASH_LOG_PATH, FILE_READ);
  if (!f) return 0;
  const size_t s = f.size();
  f.close();
  return s;
}

// 使用固定阈值（手动模式）。
static void applyFixedThresholds() {
  triggerDelta = (uint16_t)PIEZO_TRIGGER_THRESHOLD;
  const uint16_t fixedRearm = (uint16_t)PIEZO_REARM_THRESHOLD;
  rearmDelta = (fixedRearm > triggerDelta) ? triggerDelta : fixedRearm;
  if (rearmDelta == 0) rearmDelta = 1;
  thresholdsAutoCalibrated = false;
}

// 根据噪声幅度计算动态触发/解锁阈值。
static void applyAutoThresholds(uint16_t noiseDelta) {
  uint32_t derivedTrigger =
      (uint32_t)noiseDelta * (uint32_t)AUTO_THRESHOLD_NOISE_MULTIPLIER +
      (uint32_t)AUTO_THRESHOLD_MARGIN;
  if (derivedTrigger < (uint32_t)AUTO_THRESHOLD_MIN_TRIGGER_DELTA) {
    derivedTrigger = (uint32_t)AUTO_THRESHOLD_MIN_TRIGGER_DELTA;
  }
  if (derivedTrigger > (uint32_t)ADC_MAX_VALUE) {
    derivedTrigger = (uint32_t)ADC_MAX_VALUE;
  }

  triggerDelta = (uint16_t)derivedTrigger;

  uint32_t derivedRearm =
      ((uint32_t)triggerDelta * (uint32_t)AUTO_THRESHOLD_REARM_PERCENT) / 100UL;
  if (derivedRearm == 0) derivedRearm = 1;
  if (derivedRearm > (uint32_t)triggerDelta) {
    derivedRearm = (uint32_t)triggerDelta;
  }
  rearmDelta = (uint16_t)derivedRearm;
  thresholdsAutoCalibrated = true;
}

// 上电自动标定：采样静态噪声，得到 baseline 与动态阈值。
static void calibrateThresholds() {
  const int sampleCount = (AUTO_CALIBRATION_SAMPLES > 0) ? AUTO_CALIBRATION_SAMPLES : 1;

  baselineAdc = (uint16_t)analogRead(PIN_PIEZO);
  applyFixedThresholds();

  if (!ENABLE_AUTO_CALIBRATION) return;

  Serial.println("Calibrating... keep piezo still for ~0.5s");

  uint32_t sum = 0;
  uint16_t minAdc = (uint16_t)ADC_MAX_VALUE;
  uint16_t maxAdc = 0;

  for (int i = 0; i < sampleCount; i++) {
    const uint16_t v = (uint16_t)analogRead(PIN_PIEZO);
    sum += v;
    if (v < minAdc) minAdc = v;
    if (v > maxAdc) maxAdc = v;
    if (AUTO_CALIBRATION_GAP_US > 0) {
      delayMicroseconds((uint32_t)AUTO_CALIBRATION_GAP_US);
    }
  }

  baselineAdc = (uint16_t)(sum / (uint32_t)sampleCount);
  const uint16_t noiseLow = absDiffU16(minAdc, baselineAdc);
  const uint16_t noiseHigh = absDiffU16(maxAdc, baselineAdc);
  const uint16_t noiseDelta = (noiseLow > noiseHigh) ? noiseLow : noiseHigh;
  applyAutoThresholds(noiseDelta);

  Serial.print("Calibration done. baseline=");
  Serial.print(baselineAdc);
  Serial.print(",noise_delta=");
  Serial.print(noiseDelta);
  Serial.print(",trigger_delta=");
  Serial.print(triggerDelta);
  Serial.print(",rearm_delta=");
  Serial.println(rearmDelta);
}

// 仅在“安静区间”内更新基线，避免被冲击峰值拉偏。
static void updateBaseline(uint16_t adc) {
  const uint16_t quietWindow = (triggerDelta > 10U) ? (uint16_t)(triggerDelta / 2U) : 5U;
  if (absDiffU16(adc, baselineAdc) > quietWindow) return;

  const int divisor = (BASELINE_TRACK_DIVISOR > 1) ? BASELINE_TRACK_DIVISOR : 2;
  const int32_t diff = (int32_t)adc - (int32_t)baselineAdc;
  int32_t step = diff / divisor;
  if (step == 0 && diff != 0) {
    step = (diff > 0) ? 1 : -1;
  }

  int32_t next = (int32_t)baselineAdc + step;
  if (next < 0) next = 0;
  if (next > (int32_t)ADC_MAX_VALUE) next = (int32_t)ADC_MAX_VALUE;
  baselineAdc = (uint16_t)next;
}

// 在一个短时间窗口内持续采样，捕获本次触发的峰值 ADC。
static uint16_t capturePeakAdc(uint16_t seed) {
  uint16_t peak = seed;
  const unsigned long startUs = micros();

  while ((unsigned long)(micros() - startUs) < PEAK_WINDOW_US) {
    const uint16_t v = (uint16_t)analogRead(PIN_PIEZO);
    if (v > peak) peak = v;
  }
  return peak;
}

// 触发时闪烁 LED，提供肉眼可见的反馈。
static void blinkLed() {
  digitalWrite(PIN_LED, HIGH);
  delay(20);
  digitalWrite(PIN_LED, LOW);
}

// 把毫秒时间格式化为 M:SS.mmm 方便显示。
static void formatMs(char *out, size_t outSize, unsigned long ms) {
  const unsigned long totalSec = ms / 1000UL;
  const unsigned long minutes = totalSec / 60UL;
  const unsigned long seconds = totalSec % 60UL;
  const unsigned long millisPart = ms % 1000UL;
  snprintf(out, outSize, "%lu:%02lu.%03lu", minutes, seconds, millisPart);
}

// 初始化 OLED：上电、设置字体、显示启动字样。
static void initOled() {
  if (!ENABLE_OLED) return;

  Wire.begin(21, 22);
  Wire.setClock(100000);
  oledI2cAddr = findOledI2cAddress();
  if (oledI2cAddr == 0) {
    Serial.println("OLED not found at 0x3C/0x3D. Check VCC/GND/SDA/SCL.");
    oledReady = false;
    return;
  }

  if (!activateOledDriver((uint8_t)OLED_DRIVER_MODE)) {
    Serial.println("OLED driver activate failed.");
    oledReady = false;
    return;
  }

  Serial.print("OLED initialized at I2C 0x");
  if (oledI2cAddr < 16) Serial.print("0");
  Serial.println(oledI2cAddr, HEX);
  Serial.print("OLED driver: ");
  Serial.println(oledDriverName(oledDriverMode));

  oled->drawString(0, 0, "DAS PIEZO BOX");
  if (oledTextRows > 1) {
    oled->drawString(0, 1, oledDriverName(oledDriverMode));
  }
  if (oledTextRows > 2) {
    oled->drawString(0, 2, "OLED Ready");
  }
  if (oledTextRows > 3) {
    oled->drawString(0, 3, "cmd: drv0/1/2");
  }
}

// 周期刷新 OLED 显示：
// 1) 当前 ADC 与 ARM/HOLD 状态
// 2) 触发总数
// 3) 当前运行时间
// 4) 最近一次触发时间与触发幅值
static void updateDisplay(uint16_t adc, uint16_t delta, unsigned long nowMs) {
  if (!ENABLE_OLED || !oledReady || oled == nullptr) return;
  if ((unsigned long)(nowMs - lastDisplayMillis) < OLED_REFRESH_MS) return;
  lastDisplayMillis = nowMs;

  char line[17];
  TriggerEvent latest;
  const bool hasLatest = getLatestEvent(latest);
  oled->clearLine(0);
  oled->drawString(0, 0, "DAS PIEZO BOX");

  // 128x32（4 行）模式下只显示核心信息。
  if (oledTextRows <= 4) {
    char tsLastCompact[16];
    if (hasLatest) {
      formatMs(tsLastCompact, sizeof(tsLastCompact), latest.localMillis);
    } else {
      strncpy(tsLastCompact, "--:--.---", sizeof(tsLastCompact) - 1);
      tsLastCompact[sizeof(tsLastCompact) - 1] = '\0';
    }

    snprintf(line, sizeof(line), "ADC:%4u %s", adc, triggerArmed ? "ARM" : "HOLD");
    oled->clearLine(1);
    oled->drawString(0, 1, line);

    snprintf(line, sizeof(line), "T:%s", tsLastCompact);
    oled->clearLine(2);
    oled->drawString(0, 2, line);

    if (hasLatest) {
      snprintf(line, sizeof(line), "E:%3lu Pk:%4u", totalEventCount, latest.peakAdc);
    } else {
      snprintf(line, sizeof(line), "E:%3lu Pk:----", totalEventCount);
    }
    oled->clearLine(3);
    oled->drawString(0, 3, line);
    return;
  }

  char tsNow[16];
  char tsLast[16];
  formatMs(tsNow, sizeof(tsNow), nowMs);
  if (hasLatest) {
    formatMs(tsLast, sizeof(tsLast), latest.localMillis);
  } else {
    strncpy(tsLast, "--:--.---", sizeof(tsLast) - 1);
    tsLast[sizeof(tsLast) - 1] = '\0';
  }

  snprintf(line, sizeof(line), "ADC:%4u %s", adc, triggerArmed ? "ARM" : "HOLD");
  oled->clearLine(1);
  oled->drawString(0, 1, line);

  snprintf(line, sizeof(line), "Events:%4lu", totalEventCount);
  oled->clearLine(2);
  oled->drawString(0, 2, line);

  snprintf(line, sizeof(line), "Now :%s", tsNow);
  oled->clearLine(3);
  oled->drawString(0, 3, line);

  snprintf(line, sizeof(line), "Last:%s", tsLast);
  oled->clearLine(4);
  oled->drawString(0, 4, line);

  if (hasLatest) {
    snprintf(line, sizeof(line), "Trg:%4u Pk:%4u", latest.triggerAdc, latest.peakAdc);
  } else {
    snprintf(line, sizeof(line), "Trg:---- Pk:----");
  }
  oled->clearLine(5);
  oled->drawString(0, 5, line);

  snprintf(line, sizeof(line), "Base:%4u D:%4u", baselineAdc, delta);
  oled->clearLine(6);
  oled->drawString(0, 6, line);

  snprintf(line, sizeof(line), "Td:%4u Rd:%4u", triggerDelta, rearmDelta);
  oled->clearLine(7);
  oled->drawString(0, 7, line);
}

// 保存一次触发事件到缓存。
// 使用环形缓冲保留最近 MAX_EVENTS 条，同时可写入 Flash 全量日志。
static void storeTriggerEvent(uint16_t triggerAdc) {
  TriggerEvent ev;
  ev.localMillis = millis();
  ev.localMicros = micros();
  ev.triggerAdc = triggerAdc;
  ev.peakAdc = capturePeakAdc(triggerAdc);

  events[eventWriteIndex] = ev;
  eventWriteIndex = (eventWriteIndex + 1) % MAX_EVENTS;
  if (eventCount < MAX_EVENTS) {
    eventCount++;
  }

  totalEventCount++;
  appendEventToFlash(totalEventCount - 1, ev);
  if (!bleDumpInProgress) {
    char bleMsg[96];
    snprintf(bleMsg, sizeof(bleMsg), "ev=%lu,ms=%lu,trg=%u,pk=%u",
             totalEventCount, ev.localMillis, ev.triggerAdc, ev.peakAdc);
    bleNotify(bleMsg);
  }

  blinkLed();
}

// 串口打印单条事件，格式化为 CSV 风格，便于复制存档。
static void printOneEvent(int i, const TriggerEvent &ev) {
  Serial.print("#");
  Serial.print(i);
  Serial.print(",local_ms=");
  Serial.print(ev.localMillis);
  Serial.print(",local_us=");
  Serial.print(ev.localMicros);
  Serial.print(",trigger_adc=");    // 触发 ADC：触发瞬间那一刻读到的 ADC 值（刚过阈值时的值）
  Serial.print(ev.triggerAdc);
  Serial.print(",peak_adc=");       // 峰值 ADC：触发后短窗口（你现在是 2000 us）里采到的最大 ADC 值
  Serial.println(ev.peakAdc);
}

// 导出全部事件。
static void dumpEvents() {
  Serial.println("==== Trigger Events Dump ====");
  isDumpingData = true;
  for (int i = 0; i < eventCount; i++) {
    const int idx = ringIndexByOrder(i);
    if (idx >= 0) {
      printOneEvent(i, events[idx]);
    }
  }
  isDumpingData = false;
  Serial.println("==== End Dump ====");
}

// 以标准 CSV 格式导出 RAM 中最近 MAX_EVENTS 条事件。
static void dumpEventsCsvRam() {
  isDumpingData = true;
  Serial.println("index,local_ms,local_us,trigger_adc,peak_adc");
  for (int i = 0; i < eventCount; i++) {
    const int idx = ringIndexByOrder(i);
    if (idx < 0) continue;
    const TriggerEvent &ev = events[idx];
    Serial.print(i);
    Serial.print(",");
    Serial.print(ev.localMillis);
    Serial.print(",");
    Serial.print(ev.localMicros);
    Serial.print(",");
    Serial.print(ev.triggerAdc);
    Serial.print(",");
    Serial.println(ev.peakAdc);
  }
  isDumpingData = false;
}

// 导出 CSV：优先导出 Flash 全量历史；若 Flash 未启用则导出 RAM 最近数据。
static void dumpEventsCsv() {
  if (ENABLE_FLASH_LOG && flashLogReady) {
    dumpFlashCsv();
  } else {
    dumpEventsCsvRam();
  }
}

// 清空事件缓存。
static void clearEvents() {
  eventCount = 0;
  eventWriteIndex = 0;
  Serial.println("All events cleared.");
}

// 打印当前运行状态，辅助调参和排障。
static void printStatus() {
  Serial.println("==== STATUS ====");
  Serial.print("Current ADC: ");
  Serial.println(analogRead(PIN_PIEZO));
  Serial.print("ADC max: ");
  Serial.println(ADC_MAX_VALUE);
  Serial.print("Baseline ADC: ");
  Serial.println(baselineAdc);
  Serial.print("Trigger armed: ");
  Serial.println(triggerArmed ? "YES" : "NO");
  Serial.print("Active trigger delta: ");
  Serial.println(triggerDelta);
  Serial.print("Active rearm delta: ");
  Serial.println(rearmDelta);
  Serial.print("Mode: ");
  Serial.println(thresholdsAutoCalibrated ? "AUTO_CALIBRATED" : "FIXED");
  Serial.print("Fixed trigger delta: ");
  Serial.println(PIEZO_TRIGGER_THRESHOLD);
  Serial.print("Fixed rearm delta: ");
  Serial.println(PIEZO_REARM_THRESHOLD);
  Serial.print("Deadtime(ms): ");
  Serial.println(TRIGGER_DEADTIME_MS);
  Serial.print("Peak window(us): ");
  Serial.println(PEAK_WINDOW_US);
  Serial.print("Stored RAM events: ");
  Serial.println(eventCount);
  Serial.print("Total events since boot: ");
  Serial.println(totalEventCount);
  Serial.print("Flash log ready: ");
  Serial.println((ENABLE_FLASH_LOG && flashLogReady) ? "YES" : "NO");
  Serial.print("Flash log size(bytes): ");
  Serial.println((unsigned long)flashLogSizeBytes());
  Serial.print("Flash write failures: ");
  Serial.println(flashWriteFailCount);
  Serial.print("WiFi enabled: ");
  Serial.println(ENABLE_WIFI ? "YES" : "NO");
  Serial.print("WiFi credential ready: ");
  Serial.println(wifiCredentialReady() ? "YES" : "NO");
  Serial.print("WiFi connected: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "YES" : "NO");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi IP: ");
    Serial.println(WiFi.localIP());
  }
  Serial.print("BLE enabled: ");
  Serial.println(ENABLE_BLE ? "YES" : "NO");
  Serial.print("BLE ready: ");
  Serial.println(bleReady ? "YES" : "NO");
  Serial.print("BLE client connected: ");
  Serial.println(bleClientConnected ? "YES" : "NO");
  Serial.print("Signal stream: ");
  Serial.println(signalStreamEnabled ? "ON" : "OFF");
  Serial.print("Signal stream interval(ms): ");
  Serial.println(signalStreamIntervalMs);
  Serial.print("Signal stream hz: ");
  Serial.println(signalStreamHz());
  Serial.print("OLED I2C addr: 0x");
  if (oledI2cAddr < 16) Serial.print("0");
  Serial.println(oledI2cAddr, HEX);
  Serial.print("OLED driver: ");
  Serial.println(oledDriverName(oledDriverMode));
  Serial.print("OLED rows: ");
  Serial.println(oledTextRows);
  Serial.println("Commands: dump|dumpcsv|dumpramcsv|dumpflash|clear|clearflash|clearall|status|recal|stream|wifiretry|bletest|drv0|drv1|drv2|oledtest|oledon|oledoff");
  Serial.println("drv0=SSD1306_128x64, drv1=SH1106_128x64, drv2=SSD1306_128x32");
  Serial.println("BLE write cmd: help|status|dumpflash|stopdump|clearflash");
  Serial.println("===============");
}

// 周期心跳日志：确认程序还在运行，并输出关键实时值。
static void printHeartbeat(unsigned long nowMs, uint16_t adc, uint16_t delta) {
  if (!ENABLE_DEBUG || DEBUG_HEARTBEAT_MS == 0) return;
  if (signalStreamEnabled) return;
  if (isDumpingData) return;
  if ((unsigned long)(nowMs - lastHeartbeatMillis) < DEBUG_HEARTBEAT_MS) return;

  lastHeartbeatMillis = nowMs;
  Serial.print("[alive] ms=");
  Serial.print(nowMs);
  Serial.print(",adc=");
  Serial.print(adc);
  Serial.print(",delta=");
  Serial.print(delta);
  Serial.print(",base=");
  Serial.print(baselineAdc);
  Serial.print(",td=");
  Serial.print(triggerDelta);
  Serial.print(",rd=");
  Serial.print(rearmDelta);
  Serial.print(",oled=");
  Serial.print(oledDriverName(oledDriverMode));
  Serial.print(",ram_events=");
  Serial.print(eventCount);
  Serial.print(",total_events=");
  Serial.print(totalEventCount);
  Serial.print(",wifi=");
  Serial.print((WiFi.status() == WL_CONNECTED) ? "ON" : "OFF");
  Serial.print(",ble=");
  Serial.print(bleClientConnected ? "LINKED" : (bleReady ? "ADV" : "OFF"));
  Serial.print(",armed=");
  Serial.println(triggerArmed ? "YES" : "NO");
}

// 实时信号流（用于阈值调参）。
// 格式：sig_ms,adc,delta,base,td,rd,armed
static void processSignalStream(unsigned long nowMs, uint16_t adc, uint16_t delta) {
  if (!signalStreamEnabled) return;
  if (isDumpingData || bleDumpInProgress) return;
  if ((unsigned long)(nowMs - lastSignalStreamMs) < signalStreamIntervalMs) return;

  lastSignalStreamMs = nowMs;
  Serial.print(nowMs);
  Serial.print(",");
  Serial.print(adc);
  Serial.print(",");
  Serial.print(delta);
  Serial.print(",");
  Serial.print(baselineAdc);
  Serial.print(",");
  Serial.print(triggerDelta);
  Serial.print(",");
  Serial.print(rearmDelta);
  Serial.print(",");
  Serial.println(triggerArmed ? 1 : 0);
}

// 串口切换 OLED 驱动（无需重新烧录）。
static void switchOledDriverByCommand(uint8_t mode) {
  if (!ENABLE_OLED) {
    Serial.println("OLED is disabled by config.");
    return;
  }
  if (oledI2cAddr == 0) {
    Serial.println("OLED I2C address unknown, replug and reboot first.");
    return;
  }
  if (!activateOledDriver(mode)) {
    Serial.println("Switch OLED driver failed.");
    return;
  }

  oled->drawString(0, 0, "Driver switched");
  if (oledTextRows > 1) oled->drawString(0, 1, oledDriverName(oledDriverMode));
  if (oledTextRows > 2) oled->drawString(0, 2, "OK");

  Serial.print("OLED driver switched to: ");
  Serial.println(oledDriverName(oledDriverMode));
}

// 强制 OLED 画满测试，便于确认屏幕是否真的在发光。
static void oledTestPattern() {
  if (!ENABLE_OLED || !oledReady || oled == nullptr) {
    Serial.println("OLED is not ready.");
    return;
  }

  oled->setPowerSave(0);
  oled->clearDisplay();

  const char *line = "################";
  for (uint8_t r = 0; r < oledTextRows; r++) {
    oled->drawString(0, r, line);
  }
  delay(800);
  oled->clearDisplay();

  oled->drawString(0, 0, "OLED TEST");
  if (oledTextRows > 1) oled->drawString(0, 1, oledDriverName(oledDriverMode));
  if (oledTextRows > 2) oled->drawString(0, 2, "If still black:");
  if (oledTextRows > 3) oled->drawString(0, 3, "panel issue");

  Serial.println("OLED test pattern sent.");
}

// 解析并执行串口命令：dump / clear / status。
static void handleCommand(char *cmd) {
  trimInPlace(cmd);
  toLowerInPlace(cmd);

  if (strncmp(cmd, "stream", 6) == 0 && (cmd[6] == '\0' || cmd[6] == ' ')) {
    handleStreamCommand(cmd);
    return;
  }

  if (strcmp(cmd, "dump") == 0) {
    dumpEvents();
  } else if (strcmp(cmd, "dumpcsv") == 0) {
    dumpEventsCsv();
  } else if (strcmp(cmd, "dumpramcsv") == 0) {
    dumpEventsCsvRam();
  } else if (strcmp(cmd, "dumpflash") == 0) {
    dumpFlashCsv();
  } else if (strcmp(cmd, "clear") == 0) {
    clearEvents();
  } else if (strcmp(cmd, "clearflash") == 0) {
    clearFlashCsv();
  } else if (strcmp(cmd, "clearall") == 0) {
    clearEvents();
    clearFlashCsv();
  } else if (strcmp(cmd, "status") == 0) {
    printStatus();
  } else if (strcmp(cmd, "recal") == 0) {
    calibrateThresholds();
    triggerArmed = true;
    Serial.println("Recalibration done. Trigger re-armed.");
  } else if (strcmp(cmd, "wifiretry") == 0) {
    connectWifi();
  } else if (strcmp(cmd, "bletest") == 0) {
    bleNotify("DAS BLE test message");
    Serial.println("BLE test message sent (if client connected).");
  } else if (strcmp(cmd, "drv0") == 0) {
    switchOledDriverByCommand(OLED_DRIVER_MODE_SSD1306_128X64);
  } else if (strcmp(cmd, "drv1") == 0) {
    switchOledDriverByCommand(OLED_DRIVER_MODE_SH1106_128X64);
  } else if (strcmp(cmd, "drv2") == 0) {
    switchOledDriverByCommand(OLED_DRIVER_MODE_SSD1306_128X32);
  } else if (strcmp(cmd, "oledtest") == 0) {
    oledTestPattern();
  } else if (strcmp(cmd, "oledon") == 0) {
    if (oled != nullptr) oled->setPowerSave(0);
    Serial.println("OLED power save: OFF");
  } else if (strcmp(cmd, "oledoff") == 0) {
    if (oled != nullptr) oled->setPowerSave(1);
    Serial.println("OLED power save: ON");
  } else if (cmd[0] != '\0') {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }
}

// 上电初始化流程：
// 1) 配置硬件引脚
// 2) 启动串口
// 3) 初始化 OLED
// 4) 打印初始状态
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(115200);
#if defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PIEZO, ADC_11db);
#endif

  initFlashLog();
  connectWifi();
  initBle();
  calibrateThresholds();
  initOled();
  delay(500);
  Serial.println("DAS Piezo Trigger Box Starting...");
  Serial.println("Serial ready at 115200.");
  printStatus();
}

// 主循环流程：
// 1) 采样压电 ADC
// 2) 根据阈值判断是否触发
// 3) 满足重装条件后允许下一次触发
// 4) 输出心跳、刷新 OLED
// 5) 处理串口命令
void loop() {
  processBleRequests();
  processBleDumpFlash();

  const uint16_t adc = (uint16_t)analogRead(PIN_PIEZO);
  maintainWifi();
  updateBaseline(adc);
  const uint16_t delta = absDiffU16(adc, baselineAdc);
  const unsigned long nowMs = millis();

  // ARM 状态下：达到触发阈值且超过死区时间，记录一次触发。
  if (triggerArmed) {
    if (delta >= triggerDelta &&
        (nowMs - lastTriggerMillis) >= TRIGGER_DEADTIME_MS) {
      lastTriggerMillis = nowMs;
      storeTriggerEvent(adc);
      triggerArmed = false;

      if (ENABLE_DEBUG) {
        TriggerEvent latest;
        Serial.print("Trigger captured. Count=");
        Serial.print(totalEventCount);
        Serial.print(",delta=");
        Serial.print(delta);
        Serial.print(",peak_adc=");
        if (getLatestEvent(latest)) {
          Serial.println(latest.peakAdc);
        } else {
          Serial.println(0);
        }
      }
    }
  // HOLD 状态下：信号回落到重装阈值以下，重新允许触发。
  } else if (delta <= rearmDelta) {
    triggerArmed = true;
  }

  processSignalStream(nowMs, adc, delta);
  printHeartbeat(nowMs, adc, delta);
  updateDisplay(adc, delta, nowMs);

  char cmd[48];
  if (readLine(cmd, sizeof(cmd))) {
    handleCommand(cmd);
  }
}
