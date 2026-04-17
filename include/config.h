#ifndef CONFIG_H
#define CONFIG_H

// ---------------- 引脚配置 ----------------
// 压电片模拟信号输入引脚。
// 建议接线：压电片 -> 保护电路（限压/钳位）-> GPIO34(ADC1)。
static const int PIN_PIEZO = 34;
static const int PIN_LED   = 2;

// ---------------- 触发参数 ----------------
// ESP32 默认使用 12-bit ADC，读数范围通常为 0~4095。
static const int ADC_MAX_VALUE = 4095;

// 固定阈值模式下使用的触发差值阈值（|adc - baseline|）。
static const int PIEZO_TRIGGER_THRESHOLD = 320;

// 固定阈值模式下的重触发解锁差值。
static const int PIEZO_REARM_THRESHOLD = 120;

// 是否在上电时自动标定噪声并计算动态阈值（推荐开启）。
static const bool ENABLE_AUTO_CALIBRATION = true;

// 自动标定采样点数；标定期间尽量保持压电片静止。
static const int AUTO_CALIBRATION_SAMPLES = 600;

// 自动标定采样间隔（微秒）。
static const unsigned long AUTO_CALIBRATION_GAP_US = 800;

// 动态阈值下限（防止阈值过低导致误触发）。
static const int AUTO_THRESHOLD_MIN_TRIGGER_DELTA = 80;

// 动态阈值计算：triggerDelta = noiseDelta * MULTIPLIER + MARGIN。
static const int AUTO_THRESHOLD_NOISE_MULTIPLIER = 3;
static const int AUTO_THRESHOLD_MARGIN = 20;

// 动态重触发阈值占触发阈值比例（百分比）。
static const int AUTO_THRESHOLD_REARM_PERCENT = 35;

// 基线跟踪速度：baseline += (adc - baseline) / BASELINE_TRACK_DIVISOR。
// 值越大越稳但响应越慢；建议 16~64。
static const int BASELINE_TRACK_DIVISOR = 32;

// 防抖死区时间（毫秒）。
static const unsigned long TRIGGER_DEADTIME_MS = 80;

// 触发后峰值采样窗口（微秒）。
static const unsigned long PEAK_WINDOW_US = 2000;

// 最大缓存触发记录条数。
// ESP32 内存更充裕，可根据需要继续调大。
static const int MAX_EVENTS = 512;

// 是否把每次触发同时写入 ESP32 内置 Flash（SPIFFS）。
// 开启后可脱离 USB 现场采集，回到电脑后再导出。
static const bool ENABLE_FLASH_LOG = true;

// Flash 中的日志文件路径。
static const char FLASH_LOG_PATH[] = "/events.csv";

// ---------------- 无线通信 ----------------
// Wi-Fi：用于联网（例如现场回传/远程访问）。
// 首次可先关闭，确认基础采集稳定后再开启。
static const bool ENABLE_WIFI = false;
static const char WIFI_SSID[] = "YOUR_WIFI_SSID";
static const char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 12000;
static const unsigned long WIFI_RETRY_INTERVAL_MS = 15000;

// BLE：用于手机/平板近距离查看实时触发信息。
static const bool ENABLE_BLE = true;
static const char BLE_DEVICE_NAME[] = "DAS-ESP32";
static const char BLE_SERVICE_UUID[] = "12345678-1234-1234-1234-1234567890ab";
static const char BLE_CHAR_UUID[] = "12345678-1234-1234-1234-1234567890ac";

// 是否输出详细串口调试信息。
static const bool ENABLE_DEBUG = true;

// 调试心跳输出间隔（毫秒）。
// 设为 0 可关闭周期性心跳打印。
static const unsigned long DEBUG_HEARTBEAT_MS = 1000;

// 实时信号流输出（用于调阈值）：
// 开启后按固定周期输出：ms,adc,delta,base,td,rd,armed
static const bool SIGNAL_STREAM_DEFAULT_ON = false;
static const unsigned long SIGNAL_STREAM_DEFAULT_INTERVAL_MS = 40; // 约 25Hz
static const unsigned long SIGNAL_STREAM_MIN_INTERVAL_MS = 20;     // 50Hz
static const unsigned long SIGNAL_STREAM_MAX_INTERVAL_MS = 200;    // 5Hz

// ---------------- OLED 显示 ----------------
// 是否启用 I2C OLED（SSD1306 128x64, U8x8 模式）。
static const bool ENABLE_OLED = true;

// OLED 驱动模式：
// 0 = SSD1306 128x64（常见 0.96" I2C）
// 1 = SH1106  128x64（常见 1.3" I2C）
// 2 = SSD1306 128x32（部分 0.91/0.96 模块）
#define OLED_DRIVER_MODE_SSD1306_128X64 0
#define OLED_DRIVER_MODE_SH1106_128X64  1
#define OLED_DRIVER_MODE_SSD1306_128X32 2
#define OLED_DRIVER_MODE OLED_DRIVER_MODE_SSD1306_128X32

// OLED 对比度（0~255）。
static const uint8_t OLED_CONTRAST = 255;

// OLED 刷新周期（毫秒）。
static const unsigned long OLED_REFRESH_MS = 200;

#endif
