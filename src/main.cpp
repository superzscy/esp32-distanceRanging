#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L1X.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <algorithm>

// =========================
// arguments
// =========================

bool g_enableRanging = true;
bool g_enableHttpReport = true;

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

#if WIFI_CREDENTIALS_STRICT
static_assert(WIFI_SSID_CONFIGURED, "WIFI_SSID is empty. Set env var WIFI_SSID before build.");
static_assert(WIFI_PASS_CONFIGURED, "WIFI_PASS is empty. Set env var WIFI_PASS before build.");
#endif

// I2C pins
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

// WiFi arguments (injected from build_flags + system env vars)
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

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

// =========================
// helper function
// =========================

void showCenteredText(const String& line1, const String& line2 = "")
{
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

    // optional options：
    // VL53L1X::Short
    // VL53L1X::Medium
    // VL53L1X::Long
    if (vl53.VL53L1X_SetDistanceMode(2) != VL53L1X_ERROR_NONE)
    {
        Serial.println("[WARN] Failed to set distance mode");
    }

    // the bigger timing budget, the stabler it is，but slower refresh rate
    // optional values: 20, 50, 100, 200, 500
    if (!vl53.setTimingBudget(100))
    {
        Serial.println("[WARN] Failed to set timing budget");
    }

    if (vl53.VL53L1X_SetInterMeasurementInMs(55) != VL53L1X_ERROR_NONE)
    {
        Serial.println("[WARN] Failed to set inter-measurement period");
    }

    vl53.startRanging();
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
    const int threshold = max(60, median / 8); // 12.5%，60mm

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
    if (!g_enableHttpReport)
    {
        return true;
    }

    if (strlen(WIFI_SSID) == 0 || strlen(WIFI_PASS) == 0)
    {
        Serial.println("[WARN] WIFI_SSID or WIFI_PASS is empty, HTTP report disabled");
        g_enableHttpReport = false;
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
        Serial.println("[WARN] WiFi connect timeout, HTTP report disabled");
        g_enableHttpReport = false;
        return false;
    }

    Serial.print("[INFO] WiFi connected, IP = ");
    Serial.println(WiFi.localIP());
    return true;
}

void reportDistanceJson(bool hasValidDistance, int distanceMm, int sampleCount, int dataReadyCount, int validCount)
{
    if (!g_enableHttpReport)
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

    Wire.begin(I2C_SDA, I2C_SCL);

    initWiFi();

    // Init OLED
    bool oledOk = initOLED();
    if (!oledOk)
    {
        // if OLED failed，use serial to report
        while (true)
        {
            Serial.println("[FATAL] OLED init failed");
            delay(1000);
        }
    }

    showCenteredText("System Booting");

    if (g_enableRanging)
    {
        Serial.println("[INFO] Ranging enabled");

        if (!initVL53L1X())
        {
            showErrorScreen("VL53L1X init fail");
            while (true)
            {
                Serial.println("[FATAL] VL53L1X init failed");
                delay(1000);
            }
        }

        showCenteredText("Ranging ON", "Init OK");
        delay(800);
    }
    else
    {
        Serial.println("[INFO] Ranging disabled by switch");
        showCenteredText("Ranging OFF");
        delay(800);
    }
}

void loop()
{
    static uint32_t windowStartMs = 0;
    static uint32_t lastSampleMs  = 0;
    static int samples[SAMPLES_PER_SECOND];
    static int sampleCount = 0;
    static int dataReadyCount = 0;
    static int validCount = 0;
    static uint8_t emptyWindowCount = 0;

    uint32_t now = millis();
    if (windowStartMs == 0)
    {
        windowStartMs = now;
        lastSampleMs = now - SAMPLE_INTERVAL_MS;
    }

    if (!g_enableRanging)
    {
        showCenteredText("Ranging OFF", "Set g_enableRanging");
        Serial.println("[INFO] Ranging is disabled");
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
