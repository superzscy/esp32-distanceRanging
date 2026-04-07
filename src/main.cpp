#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L1X.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <BluetoothSerial.h>
#include <algorithm>

// =========================
// arguments
// =========================

bool g_enableRanging = false;
bool g_enableHttpReport = false;
bool g_enableDisplay = false;

#ifndef WIFI_SSID
#define WIFI_SSID ""
#define WIFI_SSID_FROM_ENV 0
#else
#define WIFI_SSID_FROM_ENV 1
#endif

#ifndef WIFI_PASS
#define WIFI_PASS ""
#define WIFI_PASS_FROM_ENV 0
#else
#define WIFI_PASS_FROM_ENV 1
#endif

#ifndef WIFI_CREDENTIALS_STRICT
#define WIFI_CREDENTIALS_STRICT 1
#endif

#if WIFI_SSID_FROM_ENV == 0
#warning "WIFI_SSID not injected from build flags. It is empty by default."
#endif

#if WIFI_PASS_FROM_ENV == 0
#warning "WIFI_PASS not injected from build flags. It is empty by default."
#endif

constexpr bool WIFI_SSID_CONFIGURED = (sizeof(WIFI_SSID) > 1);
constexpr bool WIFI_PASS_CONFIGURED = (sizeof(WIFI_PASS) > 1);

// I2C pins
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

// WiFi arguments (injected from build_flags + system env vars)
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;
constexpr const char* BT_DEVICE_NAME = "DR-ESP32";

// HTTP report arguments
constexpr const char* REPORT_SERVER = "http://debian.lan:8088";
constexpr const char* REPORT_API = "/api/distance";

// OLED arguments
constexpr int SCREEN_WIDTH  = 128;
constexpr int SCREEN_HEIGHT = 32;
constexpr int OLED_RESET    = -1;
constexpr uint8_t OLED_ADDR = 0x3C;

// refresh arguments
constexpr uint16_t SAMPLES_PER_SECOND = 10;
constexpr uint32_t STATS_WINDOW_MS    = 1000;
constexpr uint32_t SAMPLE_INTERVAL_MS = STATS_WINDOW_MS / SAMPLES_PER_SECOND;
constexpr uint8_t MAX_EMPTY_WINDOWS_BEFORE_RECOVER = 3;

// =========================
// globals
// =========================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();
BluetoothSerial SerialBT;
bool g_isRangingActive = false;
bool g_isWiFiActive = false;
bool g_isBluetoothActive = false;
bool g_isDisplayActive = false;

// =========================
// helper function
// =========================

void showCenteredText(const String& line1, const String& line2 = "")
{
    if (!g_isDisplayActive)
    {
        return;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(1);

    int16_t x1, y1;
    uint16_t w1, h1;
    display.getTextBounds(line1, 0, 0, &x1, &y1, &w1, &h1);
    int line1X = (SCREEN_WIDTH - static_cast<int>(w1)) / 2;
    int line1Y = line2.length() > 0 ? 6 : 12;

    display.setCursor(line1X, line1Y);
    display.println(line1);

    if (line2.length() > 0)
    {
        int16_t x2, y2;
        uint16_t w2, h2;
        display.getTextBounds(line2, 0, 0, &x2, &y2, &w2, &h2);
        int line2X = (SCREEN_WIDTH - static_cast<int>(w2)) / 2;
        int line2Y = 20;

        display.setCursor(line2X, line2Y);
        display.println(line2);
    }

    display.display();
}

void showDistanceScreen(int distanceMm)
{
    if (!g_isDisplayActive)
    {
        return;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("VL53L1X Distance");

    display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print(distanceMm);
    display.print(" mm");

    display.display();
}

void showErrorScreen(const String& msg)
{
    if (!g_isDisplayActive)
    {
        return;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.println("Error:");

    display.setCursor(0, 14);
    display.println(msg);

    display.display();
}

bool initOLED()
{
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
    {
        Serial.println("[ERROR] OLED init failed");
        return false;
    }

    display.clearDisplay();
    display.display();
    return true;
}

bool initVL53L1X()
{
    if (!vl53.begin(0x29, &Wire))
    {
        Serial.println("[ERROR] VL53L1X init failed");
        return false;
    }

    // optional options:
    // VL53L1X::Short
    // VL53L1X::Medium
    // VL53L1X::Long
    if (vl53.VL53L1X_SetDistanceMode(2) != VL53L1X_ERROR_NONE)
    {
        Serial.println("[WARN] Failed to set distance mode");
    }

    // the bigger timing budget, the stabler it is, but slower refresh rate
    // optional values: 20, 50, 100, 200, 500
    if (!vl53.setTimingBudget(100))
    {
        Serial.println("[WARN] Failed to set timing budget");
    }

    if (vl53.VL53L1X_SetInterMeasurementInMs(55) != VL53L1X_ERROR_NONE)
    {
        Serial.println("[WARN] Failed to set inter-measurement period");
    }

    if (!vl53.startRanging())
    {
        Serial.println("[ERROR] VL53L1X start ranging failed");
        return false;
    }
    Serial.println("[INFO] VL53L1X ranging started");
    return true;
}

bool recoverVL53L1X()
{
    vl53.stopRanging();
    delay(5);
    if (!vl53.startRanging())
    {
        Serial.println("[ERROR] VL53L1X restart ranging failed");
        return false;
    }
    Serial.println("[WARN] VL53L1X ranging restarted");
    return true;
}

bool readDistanceMm(int& outDistance, bool& outHadDataReady)
{
    outHadDataReady = false;

    if (!vl53.dataReady())
    {
        return false;
    }
    outHadDataReady = true;

    int distance = vl53.distance();
    uint8_t rangeStatus = 255;
    if (vl53.VL53L1X_GetRangeStatus(&rangeStatus) != VL53L1X_ERROR_NONE)
    {
        rangeStatus = 255;
    }

    vl53.clearInterrupt();

    Serial.print("[DEBUG] Distance = ");
    Serial.print(distance);
    Serial.print(" mm, RangeStatus = ");
    Serial.println(rangeStatus);

    if (rangeStatus != 0)
    {
        return false;
    }

    if (distance <= 0 || distance > 4000)
    {
        return false;
    }

    outDistance = distance;
    return true;
}

bool computeStableAverageMm(const int* samples, int count, int& outAverage)
{
    if (count <= 0)
    {
        return false;
    }

    int sorted[SAMPLES_PER_SECOND];
    for (int i = 0; i < count; ++i)
    {
        sorted[i] = samples[i];
    }
    std::sort(sorted, sorted + count);

    const int median = sorted[count / 2];
    const int threshold = max(60, median / 8); // 12.5%, minimum 60 mm

    int32_t sum = 0;
    int kept = 0;
    for (int i = 0; i < count; ++i)
    {
        if (abs(sorted[i] - median) <= threshold)
        {
            sum += sorted[i];
            ++kept;
        }
    }

    // If the number of valid points after threshold filtering is too small, it degenerates into a truncated average.
    if (kept < max(3, count / 3))
    {
        int trim = 0;
        if (count >= 10)
        {
            trim = count / 5; // abandon both ends of 20%
        }
        else if (count >= 5)
        {
            trim = 1;
        }

        const int start = trim;
        const int end = count - trim;
        if (start >= end)
        {
            return false;
        }

        sum = 0;
        kept = 0;
        for (int i = start; i < end; ++i)
        {
            sum += sorted[i];
            ++kept;
        }
    }

    if (kept <= 0)
    {
        return false;
    }

    outAverage = static_cast<int>((sum + kept / 2) / kept);
    return true;
}

String buildReportUrl()
{
    String url = REPORT_SERVER;
    String api = REPORT_API;

    if (url.endsWith("/") && api.startsWith("/"))
    {
        url.remove(url.length() - 1);
    }
    else if (!url.endsWith("/") && !api.startsWith("/"))
    {
        url += "/";
    }

    url += api;
    return url;
}

bool initWiFi()
{
    if (strlen(WIFI_SSID) == 0 || strlen(WIFI_PASS) == 0)
    {
        Serial.println("[WARN] WIFI_SSID or WIFI_PASS is empty");
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    const uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < WIFI_CONNECT_TIMEOUT_MS)
    {
        delay(200);
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("[WARN] WiFi connect timeout");
        return false;
    }

    Serial.print("[INFO] WiFi connected, IP = ");
    Serial.println(WiFi.localIP());
    return true;
}

void reportDistanceJson(bool hasValidDistance, int distanceMm, int sampleCount, int dataReadyCount, int validCount)
{
    if (!g_enableHttpReport || !g_isWiFiActive)
    {
        return;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        static uint32_t lastWarnMs = 0;
        uint32_t now = millis();
        if (now - lastWarnMs >= 5000)
        {
            Serial.println("[WARN] WiFi disconnected, skip HTTP report");
            lastWarnMs = now;
        }
        return;
    }

    String payload = "{";
    payload += "\"ts_ms\":";
    payload += millis();
    payload += ",\"has_valid_distance\":";
    payload += (hasValidDistance ? "true" : "false");
    payload += ",\"distance_mm\":";
    payload += hasValidDistance ? String(distanceMm) : "null";
    payload += ",\"sample_count\":";
    payload += sampleCount;
    payload += ",\"data_ready_count\":";
    payload += dataReadyCount;
    payload += ",\"valid_count\":";
    payload += validCount;
    payload += "}";

    HTTPClient http;
    const String url = buildReportUrl();
    if (!http.begin(url))
    {
        Serial.println("[WARN] HTTP begin failed");
        return;
    }

    http.addHeader("Content-Type", "application/json");
    const int code = http.POST(payload);
    if (code > 0)
    {
        Serial.print("[INFO] Report POST code = ");
        Serial.println(code);
    }
    else
    {
        Serial.print("[WARN] Report POST failed, code = ");
        Serial.println(code);
    }
    http.end();
}

void logCommandMessage(const String& msg)
{
    Serial.println(msg);
    if (g_isBluetoothActive && SerialBT.hasClient())
    {
        SerialBT.println(msg);
    }
}

void handleCommandLine(const String& cmdLine, const char* source)
{
    if (cmdLine == "w 0")
    {
        g_enableHttpReport = false;
        logCommandMessage(String("[CMD][") + source + "] WiFi disable requested");
    }
    else if (cmdLine == "w 1")
    {
        g_enableHttpReport = true;
        logCommandMessage(String("[CMD][") + source + "] WiFi enable requested");
    }
    else if (cmdLine == "d 0")
    {
        g_enableDisplay = false;
        logCommandMessage(String("[CMD][") + source + "] Display disable requested");
    }
    else if (cmdLine == "d 1")
    {
        g_enableDisplay = true;
        logCommandMessage(String("[CMD][") + source + "] Display enable requested");
    }
    else
    {
        logCommandMessage(String("[CMD][") + source + "] Unsupported command. Use: w 0 | w 1 | d 0 | d 1");
    }
}

void processCommandStream(Stream& stream, String& buffer, const char* source)
{
    while (stream.available() > 0)
    {
        const char ch = static_cast<char>(stream.read());
        if (ch == '\r')
        {
            continue;
        }
        if (ch != '\n')
        {
            buffer += ch;
            continue;
        }

        buffer.trim();
        if (buffer.length() > 0)
        {
            handleCommandLine(buffer, source);
        }
        buffer = "";
    }
}

void processCommands()
{
    static String serialCmd;
    static String btCmd;

    processCommandStream(Serial, serialCmd, "SERIAL");
    if (g_isBluetoothActive)
    {
        processCommandStream(SerialBT, btCmd, "BT");
    }
}

bool initBluetooth()
{
    if (!SerialBT.begin(BT_DEVICE_NAME))
    {
        Serial.println("[WARN] Bluetooth init failed");
        return false;
    }
    Serial.print("[INFO] Bluetooth SPP started: ");
    Serial.println(BT_DEVICE_NAME);
    return true;
}

void stopBluetoothIfNeeded()
{
    if (!g_isBluetoothActive)
    {
        return;
    }
    SerialBT.end();
    g_isBluetoothActive = false;
    Serial.println("[INFO] Bluetooth stopped");
}

void stopWiFiIfNeeded()
{
    if (!g_isWiFiActive)
    {
        return;
    }

    g_enableRanging = false;
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    g_isWiFiActive = false;
    Serial.println("[INFO] WiFi stopped");
}

void stopRangingIfNeeded()
{
    if (!g_isRangingActive)
    {
        return;
    }
    vl53.stopRanging();
    g_isRangingActive = false;
    Serial.println("[INFO] Ranging stopped");
}

void stopDisplayIfNeeded()
{
    if (!g_isDisplayActive)
    {
        return;
    }

    display.clearDisplay();
    display.display();
    g_isDisplayActive = false;
    Serial.println("[INFO] Display stopped");
}

// =========================
// Arduino entry
// =========================

void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.println();
    Serial.println("================================");
    Serial.println("ESP32 + VL53L1X + OLED Test");
    Serial.println("================================");

    g_isBluetoothActive = initBluetooth();

    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("[INFO] Display switch default is OFF (use d 1 to enable)");
}

void loop()
{
    processCommands();

    static uint32_t windowStartMs = 0;
    static uint32_t lastSampleMs  = 0;
    static int samples[SAMPLES_PER_SECOND];
    static int sampleCount = 0;
    static int dataReadyCount = 0;
    static int validCount = 0;
    static uint8_t emptyWindowCount = 0;
    static bool lastRangingEnabled = false;
    static bool lastHttpEnabled = false;
    static bool lastDisplayEnabled = false;
    static bool lastBtClientConnected = false;
    static uint32_t lastRangingInitAttemptMs = 0;
    static uint32_t lastWiFiAttemptMs = 0;
    static uint32_t lastDisplayInitAttemptMs = 0;
    static uint32_t lastOffLogMs = 0;

    auto resetWindow = [&]() {
        sampleCount = 0;
        dataReadyCount = 0;
        validCount = 0;
        emptyWindowCount = 0;
        windowStartMs = millis();
        lastSampleMs = windowStartMs - SAMPLE_INTERVAL_MS;
    };

    uint32_t now = millis();
    if (windowStartMs == 0)
    {
        windowStartMs = now;
        lastSampleMs = now - SAMPLE_INTERVAL_MS;
    }

    const bool btClientConnected = g_isBluetoothActive && SerialBT.hasClient();
    if (lastBtClientConnected && !btClientConnected)
    {
        if (g_enableHttpReport || g_isWiFiActive)
        {
            g_enableHttpReport = false;
            stopWiFiIfNeeded();
            Serial.println("[INFO] Bluetooth disconnected, WiFi auto disabled");
        }
    }
    lastBtClientConnected = btClientConnected;

    // Runtime WiFi on/off handling
    if (g_enableHttpReport && !g_isWiFiActive)
    {
        if (now - lastWiFiAttemptMs >= 3000)
        {
            lastWiFiAttemptMs = now;
            g_isWiFiActive = initWiFi();
            if (g_isWiFiActive)
            {
                g_enableRanging = true;
            }
            else
            {
                Serial.println("[WARN] WiFi enable requested but not ready");
            }
        }
    }
    else if (!g_enableHttpReport && g_isWiFiActive)
    {
        stopWiFiIfNeeded();
    }

    // Runtime display on/off handling
    if (g_enableDisplay && !g_isDisplayActive)
    {
        if (now - lastDisplayInitAttemptMs >= 1000)
        {
            lastDisplayInitAttemptMs = now;
            if (initOLED())
            {
                g_isDisplayActive = true;
                showCenteredText("Ranging OFF", g_isWiFiActive ? "WiFi ON" : "WiFi OFF");
            }
            else
            {
                Serial.println("[WARN] Display enable requested but init failed");
            }
        }
    }
    else if (!g_enableDisplay && g_isDisplayActive)
    {
        stopDisplayIfNeeded();
    }

    // Runtime ranging on/off handling
    if (g_enableRanging && !g_isRangingActive)
    {
        if (now - lastRangingInitAttemptMs >= 1000)
        {
            lastRangingInitAttemptMs = now;
            if (initVL53L1X())
            {
                g_isRangingActive = true;
                resetWindow();
                showCenteredText("Ranging ON", g_isWiFiActive ? "WiFi ON" : "WiFi OFF");
            }
            else
            {
                Serial.println("[WARN] Ranging enable requested but init failed");
            }
        }
    }
    else if (!g_enableRanging && g_isRangingActive)
    {
        stopRangingIfNeeded();
        resetWindow();
        showCenteredText("Ranging OFF", g_isWiFiActive ? "WiFi ON" : "WiFi OFF");
    }

    if (!g_isRangingActive)
    {
        if (g_enableRanging != lastRangingEnabled || g_enableHttpReport != lastHttpEnabled || g_enableDisplay != lastDisplayEnabled)
        {
            showCenteredText("Ranging OFF", g_isWiFiActive ? "WiFi ON" : "WiFi OFF");
            lastRangingEnabled = g_enableRanging;
            lastHttpEnabled = g_enableHttpReport;
            lastDisplayEnabled = g_enableDisplay;
        }
        if (now - lastOffLogMs >= 3000)
        {
            lastOffLogMs = now;
        }
        delay(20);
        return;
    }

    if (now - lastSampleMs >= SAMPLE_INTERVAL_MS)
    {
        lastSampleMs = now;
        int distanceMm = 0;
        bool hadDataReady = false;
        bool valid = readDistanceMm(distanceMm, hadDataReady);
        if (hadDataReady)
        {
            ++dataReadyCount;
        }
        if (valid && sampleCount < static_cast<int>(SAMPLES_PER_SECOND))
        {
            samples[sampleCount++] = distanceMm;
            ++validCount;
        }
    }

    if (now - windowStartMs >= STATS_WINDOW_MS)
    {
        int stableAverageMm = 0;
        if (computeStableAverageMm(samples, sampleCount, stableAverageMm))
        {
            showDistanceScreen(stableAverageMm);
            reportDistanceJson(true, stableAverageMm, sampleCount, dataReadyCount, validCount);
            Serial.print("[INFO] Stable Avg = ");
            Serial.print(stableAverageMm);
            Serial.print(" mm, kept = ");
            Serial.print(sampleCount);
            Serial.print(", dataReady = ");
            Serial.print(dataReadyCount);
            Serial.print(", valid = ");
            Serial.println(validCount);
            emptyWindowCount = 0;
        }
        else
        {
            showCenteredText("Measuring...", "No valid data");
            reportDistanceJson(false, 0, sampleCount, dataReadyCount, validCount);
            Serial.print("[WARN] No valid window data, kept = ");
            Serial.print(sampleCount);
            Serial.print(", dataReady = ");
            Serial.print(dataReadyCount);
            Serial.print(", valid = ");
            Serial.println(validCount);

            if (dataReadyCount == 0)
            {
                ++emptyWindowCount;
                if (emptyWindowCount >= MAX_EMPTY_WINDOWS_BEFORE_RECOVER)
                {
                    recoverVL53L1X();
                    emptyWindowCount = 0;
                }
            }
            else
            {
                emptyWindowCount = 0;
            }
        }

        sampleCount = 0;
        dataReadyCount = 0;
        validCount = 0;
        windowStartMs = now;
    }

    delay(2);
}
