ESP32 wroom32 + 0.91' OLED + VL53L1

WiFi secrets are injected from system environment variables (not stored in source):

```powershell create a file upload.ps1, copy the following block to it
$env:WIFI_SSID="your_ssid"
$env:WIFI_PASS="your_password"
C:\Users\xxx\.platformio\penv\Scripts\platformio.exe run --target upload
```

Then build/upload in PlatformIO as usual.
