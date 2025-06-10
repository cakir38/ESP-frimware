#include <WiFi.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SHT4x.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// Pin Definitions
const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 64;
const int8_t OLED_RESET = -1;
const uint8_t I2C_ADDRESS = 0x3C;

const uint8_t SDA_PIN_1 = 8;
const uint8_t SCL_PIN_1 = 9;
const uint8_t SDA_PIN_2 = 11;
const uint8_t SCL_PIN_2 = 12;

// I2C Buses
TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);

// Display Object
// Ekran nesnesi, I2C_1 üzerinde kurulur
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_1, OLED_RESET);

WebServer server(80);
HTTPUpdateServer httpUpdater;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 10800, 60000);

// SHT4x Sensor Objects
// İki SHT4x sensör nesnesi tanımlanır
Adafruit_SHT4x sht45_1 = Adafruit_SHT4x();
Adafruit_SHT4x sht45_2 = Adafruit_SHT4x();

// Sensör bağlantı durumları ve değerleri
bool sht45_1Connected = false, sht45_2Connected = false;
float temp1 = NAN, hum1 = NAN, temp2 = NAN, hum2 = NAN;

// Kalibrasyon Ofsetleri
float temp1Offset = 0.0, hum1Offset = 0.0;
float temp2Offset = 0.0, hum2Offset = 0.0;

// AP Modu Bilgileri
const char* apSSID = "CAKIR ELKTRONIK";
const char* apPassword = "password";

// Zamanlama Değişkenleri
unsigned long previousMillisOLED = 0;
unsigned long previousMillisSensors = 0;
unsigned long previousMillisWiFiCheck = 0;
const long intervalOLED = 500; // OLED ekran güncelleme aralığı
const long intervalSensors = 1500; // Sensör okuma aralığı
const long intervalWiFiCheck = 30000; // WiFi bağlantı kontrol aralığı
const unsigned long shtReconnectInterval = 30000; // SHT yeniden bağlantı deneme aralığı
unsigned long lastSht1Attempt = 0;
unsigned long lastSht2Attempt = 0;

// OLED Ekran Durum Yönetimi
enum DisplayMode {
  MODE_AP, // AP modu
  MODE_WIFI_CONNECTED // WiFi bağlı modu
};
DisplayMode currentDisplayMode = MODE_AP;

enum APDisplayState {
  AP_INFO, // AP bilgisi ekranı
  AP_SENSOR_FULL // AP modunda tam sensör ekranı
};
APDisplayState currentAPDisplayState = AP_INFO;

enum WiFiDisplayState {
  WIFI_INFO_COMPACT, // WiFi bağlı modunda özet bilgi ekranı
  WIFI_SENSOR_FULL // WiFi bağlı modunda tam sensör ekranı
};
WiFiDisplayState currentWiFiDisplayState = WIFI_INFO_COMPACT;

unsigned long lastScreenSwitchTime = 0; // Son ekran geçiş zamanı
const unsigned long AP_INFO_DURATION = 8000; // AP bilgi ekranı süresi
const unsigned long WIFI_INFO_DURATION = 8000; // WiFi bilgi ekranı süresi
const unsigned long SENSOR_PAGE_DURATION = 30000; // Sensör sayfası süresi

// Tercihler (kalibrasyon için)
Preferences preferences;

// Fonksiyon Prototipleri
String getSignalStars();
bool initialize_sht45_1();
bool initialize_sht45_2();
void drawAPInfoScreen();
void drawWiFiInfoScreen();
void drawFullScreenSensorData();
void handleOLEDDisplay();
void readSensors();
void checkWiFiConnection();
void scanI2CBus(TwoWire &i2c, const char* busName);
void loadCalibrationOffsets();
void saveCalibrationOffsets();
String getCalibrationPage();

// WiFi Sinyal Gücü Yıldız Olarak
String getSignalStars() {
  if (WiFi.status() == WL_CONNECTED) {
    long rssi = WiFi.RSSI();
    if (rssi >= -55) return "*****";
    else if (rssi >= -65) return "****";
    else if (rssi >= -75) return "***";
    else if (rssi >= -85) return "**";
    else return "*";
  } else {
    return "-----";
  }
}

// SHT45-1 Başlat/Yeniden Bağlan
bool initialize_sht45_1() {
  if (sht45_1Connected) return true;

  Serial.println(F("SHT45-1 (I2C_1) başlatılmaya çalışılıyor..."));
  if (sht45_1.begin(&I2C_1)) {
    sht45_1Connected = true;
    sht45_1.setPrecision(SHT4X_HIGH_PRECISION);
    sht45_1.setHeater(SHT4X_NO_HEATER);
    Serial.println(F("SHT45-1 (I2C_1) bağlandı."));
    return true;
  }
  Serial.println(F("SHT45-1 (I2C_1) bağlantısı başarısız!"));
  sht45_1Connected = false;
  temp1 = NAN; hum1 = NAN;
  return false;
}

// SHT45-2 Başlat/Yeniden Bağlan
bool initialize_sht45_2() {
  if (sht45_2Connected) return true;

  Serial.println(F("SHT45-2 (I2C_2) başlatılmaya çalışılıyor..."));
  if (sht45_2.begin(&I2C_2)) {
    sht45_2Connected = true;
    sht45_2.setPrecision(SHT4X_HIGH_PRECISION);
    sht45_2.setHeater(SHT4X_NO_HEATER);
    Serial.println(F("SHT45-2 (I2C_2) bağlandı."));
    return true;
  }
  Serial.println(F("SHT45-2 (I2C_2) bağlantısı başarısız!"));
  sht45_2Connected = false;
  temp2 = NAN; hum2 = NAN;
  return false;
}

// I2C Bus Tarama Fonksiyonu
void scanI2CBus(TwoWire &i2c, const char* busName) {
  Serial.print(F("I2C bus taranıyor: "));
  Serial.println(busName);
  for (uint8_t address = 1; address < 127; address++) {
    i2c.beginTransmission(address);
    if (i2c.endTransmission() == 0) {
      Serial.print(F("I2C cihaz adresi bulundu: 0x"));
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
}

// Kalibrasyon Ofsetlerini Yükle
void loadCalibrationOffsets() {
  preferences.begin("calibration", true);
  temp1Offset = preferences.getFloat("temp1Offset", 0.0);
  hum1Offset = preferences.getFloat("hum1Offset", 0.0);
  temp2Offset = preferences.getFloat("temp2Offset", 0.0);
  hum2Offset = preferences.getFloat("hum2Offset", 0.0);
  preferences.end();
}

// Kalibrasyon Ofsetlerini Kaydet
void saveCalibrationOffsets() {
  preferences.begin("calibration", false);
  preferences.putFloat("temp1Offset", temp1Offset);
  preferences.putFloat("hum1Offset", hum1Offset);
  preferences.putFloat("temp2Offset", temp2Offset);
  preferences.putFloat("hum2Offset", hum2Offset);
  preferences.end();
}

// OLED Çizim Fonksiyonları
void drawAPInfoScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("AP MODU ")); // AP Mode Active
  display.print(F("SSID: "));
  display.println(apSSID);
  display.print(F("IP: "));
  IPAddress apIP = WiFi.softAPIP();
  display.println(apIP);

  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print(F("S1:"));
  if (sht45_1Connected && !isnan(temp1)) {
    display.print(temp1 + temp1Offset, 1); display.print((char)248); display.print("C ");
    display.print(hum1 + hum1Offset, 1); display.print("%");
  } else {
    display.print(F("Bagli Degil")); // Not Connected
  }

  display.setCursor(0, 50);
  display.print(F("S2:"));
  if (sht45_2Connected && !isnan(temp2)) {
    display.print(temp2 + temp2Offset, 1); display.print((char)248); display.print("C ");
    display.print(hum2 + hum2Offset, 1); display.print("%");
  } else {
    display.print(F("Bagli Degil")); // Not Connected
  }
  display.display();
}

void drawWiFiInfoScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  if (timeClient.isTimeSet()) {
    display.print(timeClient.getFormattedTime());
  } else {
    display.print(F("Zaman Bekleniyor..")); // Time Pending..
  }

  String signalText = getSignalStars();
  display.setCursor(SCREEN_WIDTH - (signalText.length() * 6) - 1, 0);
  display.print(signalText);

  display.setCursor(0, 10);
  display.print(F("Wifi: ")); // Net:
  display.println(WiFi.SSID());

  display.setCursor(0, 20);
  display.print(F("IP: "));
  display.println(WiFi.localIP());

  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print(F("S1:"));
  if (sht45_1Connected && !isnan(temp1)) {
    display.print(temp1 + temp1Offset, 1); display.print((char)248); display.print("C ");
    display.print(hum1 + hum1Offset, 1); display.print("%");
  } else {
    display.print(F("Bagli Degil")); // Not Connected
  }

  display.setCursor(0, 50);
  display.print(F("S2:"));
  if (sht45_2Connected && !isnan(temp2)) {
    display.print(temp2 + temp2Offset, 1); display.print((char)248); display.print("C ");
    display.print(hum2 + hum2Offset, 1); display.print("%");
  } else {
    display.print(F("Bagli Degil")); // Not Connected
  }
  display.display();
}

// Kullanıcının resmine göre güncellenmiş tam ekran sensör verisi çizim fonksiyonu
void drawFullScreenSensorData() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Başlıklar: SICAKLIK | NEM
  display.setTextSize(1);

  // Metinlerin genişliğini hesaplayarak ortalama
  int16_t x1_temp, y1_temp;
  uint16_t w_temp, h_temp;
  display.getTextBounds("SICAKLIK", 0, 0, &x1_temp, &y1_temp, &w_temp, &h_temp);
  display.setCursor((SCREEN_WIDTH / 2 - w_temp) / 2, 0); // SICAKLIK başlığını sol yarıda ortala
  display.print(F("SICAKLIK"));

  int16_t x1_hum, y1_hum;
  uint16_t w_hum, h_hum;
  display.getTextBounds("NEM", 0, 0, &x1_hum, &y1_hum, &w_hum, &h_hum);
  display.setCursor(SCREEN_WIDTH / 2 + (SCREEN_WIDTH / 2 - w_hum) / 2, 0); // NEM başlığını sağ yarıda ortala
  display.print(F("NEM"));

  // Dikey ayırıcı çizgi
  display.drawLine(SCREEN_WIDTH / 2, 0, SCREEN_WIDTH / 2, SCREEN_HEIGHT, SSD1306_WHITE);

  // Sensör 1 verileri
  display.setTextSize(2); // Daha büyük yazı tipi
  if (sht45_1Connected && !isnan(temp1)) {
    char tempStr1[7]; // örn: 26.9
    char humStr1[5];  // örn: 48%
    dtostrf(temp1 + temp1Offset, 4, 1, tempStr1); // Sıcaklık için 1 ondalık hane
    dtostrf(hum1 + hum1Offset, 3, 0, humStr1);   // Nem için 0 ondalık hane

    // Sıcaklık 1 (Ortalanmış)
    int16_t x_temp1_val, y_temp1_val;
    uint16_t w_temp1_val, h_temp1_val;
    display.getTextBounds(tempStr1, 0, 0, &x_temp1_val, &y_temp1_val, &w_temp1_val, &h_temp1_val);
    // Derece sembolü için ekstra 10 piksel boşluk bırak
    display.setCursor((SCREEN_WIDTH / 2 - (w_temp1_val + 10)) / 2, 18);
    display.print(tempStr1);
    display.setTextSize(1); // Derece sembolü için küçük yazı tipi
    display.cp437(true);
    display.write(248); // Derece sembolü
    display.setTextSize(2); // Yeniden büyük yazı tipi

    // Nem 1 (Ortalanmış)
    int16_t x_hum1_val, y_hum1_val;
    uint16_t w_hum1_val, h_hum1_val;
    display.getTextBounds(humStr1, 0, 0, &x_hum1_val, &y_hum1_val, &w_hum1_val, &h_hum1_val);
    // Yüzde sembolü için ekstra 6 piksel boşluk bırak
    display.setCursor(SCREEN_WIDTH / 2 + (SCREEN_WIDTH / 2 - (w_hum1_val + 6)) / 2, 18);
    display.print(humStr1);
    display.print(F("%"));
  } else {
    display.setTextSize(1);
    int16_t x1_notcon, y1_notcon;
    uint16_t w_notcon, h_notcon;
    display.getTextBounds("S1 Bagli Degil", 0, 0, &x1_notcon, &y1_notcon, &w_notcon, &h_notcon);
    display.setCursor((SCREEN_WIDTH - w_notcon) / 2, 25); // Tüm ekranın ortasına yakın
    display.println(F("S1 Bağli Değil")); // S1 Not Connected
  }

  // Yatay ayırıcı çizgi
  display.drawLine(0, SCREEN_HEIGHT / 2, SCREEN_WIDTH, SCREEN_HEIGHT / 2, SSD1306_WHITE);

  // Sensör 2 verileri
  display.setTextSize(2); // Daha büyük yazı tipi
  if (sht45_2Connected && !isnan(temp2)) {
    char tempStr2[7];
    char humStr2[5];
    dtostrf(temp2 + temp2Offset, 4, 1, tempStr2); // Sıcaklık için 1 ondalık hane
    dtostrf(hum2 + hum2Offset, 3, 0, humStr2);   // Nem için 0 ondalık hane

    // Sıcaklık 2 (Ortalanmış)
    int16_t x_temp2_val, y_temp2_val;
    uint16_t w_temp2_val, h_temp2_val;
    display.getTextBounds(tempStr2, 0, 0, &x_temp2_val, &y_temp2_val, &w_temp2_val, &h_temp2_val);
    display.setCursor((SCREEN_WIDTH / 2 - (w_temp2_val + 10)) / 2, 42); // Derece sembolü için boşluk bırak
    display.print(tempStr2);
    display.setTextSize(1); // Derece sembolü için küçük yazı tipi
    display.cp437(true);
    display.write(248); // Derece sembolü
    display.setTextSize(2); // Yeniden büyük yazı tipi

    // Nem 2 (Ortalanmış)
    int16_t x_hum2_val, y_hum2_val;
    uint16_t w_hum2_val, h_hum2_val;
    display.getTextBounds(humStr2, 0, 0, &x_hum2_val, &y_hum2_val, &w_hum2_val, &h_hum2_val);
    display.setCursor(SCREEN_WIDTH / 2 + (SCREEN_WIDTH / 2 - (w_hum2_val + 6)) / 2, 42); // % sembolü için boşluk bırak
    display.print(humStr2);
    display.print(F("%"));
  } else {
    display.setTextSize(1);
    int16_t x2_notcon, y2_notcon;
    uint16_t w2_notcon, h2_notcon;
    display.getTextBounds("S2 Bagli Degil", 0, 0, &x2_notcon, &y2_notcon, &w2_notcon, &h2_notcon);
    display.setCursor((SCREEN_WIDTH - w2_notcon) / 2, 50); // Tüm ekranın ortasına yakın
    display.println(F("S2 Bagli Degil")); // S2 Not Connected
  }
  display.display();
}


// Sensör Verilerini Oku
void readSensors() {
  sensors_event_t humidity_event, temp_event;

  // Sensor 1
  if (sht45_1Connected || (millis() - lastSht1Attempt > shtReconnectInterval)) {
    if (!sht45_1Connected) {
      Serial.println(F("SHT45-1 (I2C_1) yeniden bağlanmaya çalışılıyor..."));
      initialize_sht45_1();
      lastSht1Attempt = millis();
    }

    if (sht45_1Connected) {
      if (sht45_1.getEvent(&humidity_event, &temp_event)) {
        if (isnan(temp1) || abs(temp_event.temperature - temp1) < 10.0) {
          temp1 = temp_event.temperature;
        }
        if (isnan(hum1) || abs(humidity_event.relative_humidity - hum1) < 20.0) {
          hum1 = humidity_event.relative_humidity;
        }
      } else {
        Serial.println(F("SHT45-1 okuma hatası!"));
        sht45_1Connected = false;
        temp1 = NAN; hum1 = NAN;
        lastSht1Attempt = millis();
      }
    }
  }

  // Sensor 2
  if (sht45_2Connected || (millis() - lastSht2Attempt > shtReconnectInterval)) {
    if (!sht45_2Connected) {
      Serial.println(F("SHT45-2 (I2C_2) yeniden bağlanmaya çalışılıyor..."));
      initialize_sht45_2();
      lastSht2Attempt = millis();
    }

    if (sht45_2Connected) {
      if (sht45_2.getEvent(&humidity_event, &temp_event)) {
        if (isnan(temp2) || abs(temp_event.temperature - temp2) < 10.0) {
          temp2 = temp_event.temperature;
        }
        if (isnan(hum2) || abs(humidity_event.relative_humidity - hum2) < 20.0) {
          hum2 = humidity_event.relative_humidity;
        }
      } else {
        Serial.println(F("SHT45-2 okuma hatası!"));
        sht45_2Connected = false;
        temp2 = NAN; hum2 = NAN;
        lastSht2Attempt = millis();
      }
    }
  }
}

// Periyodik WiFi Bağlantı Kontrolü
void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED && (millis() - previousMillisWiFiCheck >= intervalWiFiCheck)) {
    Serial.println(F("WiFi bağlantısı kesildi. Yeniden bağlanmaya çalışılıyor..."));
    WiFi.begin(); // Kayıtlı kimlik bilgileriyle yeniden bağlanma denemesi
    previousMillisWiFiCheck = millis();
  }
}

// OLED Ekranı Yönet
void handleOLEDDisplay() {
  unsigned long currentMillis = millis();

  switch (currentDisplayMode) {
    case MODE_AP:
      // AP modunda ekran geçiş mantığı
      if (currentAPDisplayState == AP_INFO && (currentMillis - lastScreenSwitchTime >= AP_INFO_DURATION)) {
        currentAPDisplayState = AP_SENSOR_FULL;
        lastScreenSwitchTime = currentMillis;
      } else if (currentAPDisplayState == AP_SENSOR_FULL && (currentMillis - lastScreenSwitchTime >= SENSOR_PAGE_DURATION)) {
        currentAPDisplayState = AP_INFO;
        lastScreenSwitchTime = currentMillis;
      }

      if (currentAPDisplayState == AP_INFO) {
        drawAPInfoScreen();
      } else {
        drawFullScreenSensorData();
      }
      break;

    case MODE_WIFI_CONNECTED:
      // WiFi bağlı modunda ekran geçiş mantığı
      if (currentWiFiDisplayState == WIFI_INFO_COMPACT && (currentMillis - lastScreenSwitchTime >= WIFI_INFO_DURATION)) {
        currentWiFiDisplayState = WIFI_SENSOR_FULL;
        lastScreenSwitchTime = currentMillis;
      } else if (currentWiFiDisplayState == WIFI_SENSOR_FULL && (currentMillis - lastScreenSwitchTime >= SENSOR_PAGE_DURATION)) {
        currentWiFiDisplayState = WIFI_INFO_COMPACT;
        lastScreenSwitchTime = currentMillis;
      }

      if (currentWiFiDisplayState == WIFI_INFO_COMPACT) {
        drawWiFiInfoScreen();
      } else {
        drawFullScreenSensorData();
      }
      break;
  }
}

// HTML Sayfası (PROGMEM'de depolanır)
const char htmlPage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <meta name="description" content="Çakır Kuluçka sıcaklık ve nem takip sistemi - Geliştirilmiş Sürüm">
  <title>ÇAKIR KULUÇKA v1.1</title>
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css" crossorigin="anonymous" referrerpolicy="no-referrer" />
  <style>
    body {
      font-family: 'Segoe UI', Arial, sans-serif;
      background: linear-gradient(135deg, #e9ecef, #dee2e6);
      color: #343a40;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
      padding: 5px;
      margin: 0;
      box-sizing: border-box;
    }
    .header h1 {
      font-size: 2.8em;
      color: #0056b3;
      margin-bottom: 20px;
      text-align: center;
      font-weight: 600;
      text-shadow: 1px 1px 2px rgba(0,0,0,0.1);
    }
    .sensor-group {
      display: flex;
      flex-direction: row;
      justify-content: center;
      align-items: flex-start;
      margin: 10px 0;
      width: 100%;
      max-width: 1000px;
      flex-wrap: wrap;
    }
    .gauge-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      margin: 10px;
      padding: 15px;
      box-sizing: border-box;
      width: 100%;
      max-width: 460px;
      background-color: #ffffff;
      border-radius: 12px;
      box-shadow: 0 4px 12px rgba(0,0,0,0.1);
      transition: transform 0.2s ease-in-out;
    }
    .gauge-container:hover {
      transform: translateY(-3px);
    }
    .gauge-icon {
      font-size: 1.6em;
      margin-top: 8px;
      margin-left: auto;
      margin-right: auto;
      display: block;
      text-align: center;
      color: #555;
    }
    .gauge-icon i {
      display: block;
      margin-bottom: 4px;
    }
    .temp-icon i { color: #dc3545; }
    .humidity-icon i { color: #007bff; }
    #temperatureGauge1, #humidityGauge1, #temperatureGauge2, #humidityGauge2 {
      width: 100%;
      height: auto;
      max-height: 300px;
      margin-bottom: 10px;
      aspect-ratio: 400 / 280;
    }
    .footer {
      margin-top: 30px;
      padding: 18px 25px;
      background-color: #f8f9fa;
      border-radius: 12px;
      text-align: center;
      color: #495057;
      font-size: 1.1em;
      line-height: 1.7;
      width: 100%;
      max-width: 1000px;
      box-sizing: border-box;
      box-shadow: 0 -2px 8px rgba(0,0,0,0.05);
    }
    .footer a {
      color: #0056b3;
      text-decoration: none;
      font-weight: 600;
      transition: color 0.3s ease;
    }
    .footer a:hover {
      color: #003875;
      text-decoration: underline;
    }
    .update-btn-container {
      margin-top: 15px;
      text-align: center;
    }
    .update-btn {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      padding: 12px 25px;
      background: #28a745;
      color: white;
      border-radius: 50px;
      text-decoration: none;
      transition: background 0.3s ease, transform 0.2s ease;
      font-weight: bold;
      box-shadow: 0 3px 8px rgba(0,0,0,0.15);
      white-space: nowrap;
    }
    .update-btn:hover {
      background: #218838;
      transform: scale(1.03);
    }
    .update-icon {
      margin-right: 10px;
      font-size: 1.2em;
      animation: spin 2s linear infinite;
    }
    @keyframes spin {
      from { transform: rotate(0deg); }
      to { transform: rotate(360deg); }
    }
    .developer-info {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      font-style: italic;
      color: #6c757d;
      margin-top: 12px;
      font-weight: 500;
      text-align: center;
    }
    .last-update {
      text-align: center;
      margin: 10px 0;
      font-size: 0.9em;
      color: #6c757d;
    }
    .error-message {
      color: #721c24;
      background-color: #f8d7da;
      border: 1px solid #f5c6cb;
      padding: 10px 15px;
      border-radius: 8px;
      margin: 15px auto;
      text-align: center;
      width: 90%;
      max-width: 800px;
      display: none;
      font-size: 0.95em;
    }
    @media (max-width: 768px) {
      body { padding: 5px; }
      .header h1 { font-size: 2em; margin-bottom: 15px; }
      .sensor-group {
        margin: 5px 0;
        flex-direction: column;
        align-items: center;
      }
      .gauge-container {
        margin: 8px 0;
        padding: 10px;
        max-width: 95%;
      }
      .gauge-icon { font-size: 1.3em; margin-top: 5px; }
      #temperatureGauge1, #humidityGauge1, #temperatureGauge2, #humidityGauge2 {
        margin-bottom: 5px;
        max-height: 250px;
        aspect-ratio: 350 / 220;
      }
      .footer { font-size: 0.9em; padding: 12px; margin-top: 20px; }
      .update-btn { padding: 10px 20px; font-size: 0.95em; }
      .update-icon { font-size: 1em; margin-right: 6px; }
    }
    @media (min-width: 769px) and (max-width: 1024px) {
      .header h1 { font-size: 2.4em; margin-bottom: 18px; }
      .gauge-container { max-width: 45%; }
      #temperatureGauge1, #humidityGauge1, #temperatureGauge2, #humidityGauge2 {
        max-height: 280px;
        aspect-ratio: 380 / 250;
      }
    }
    @media (min-width: 1025px) {
      .gauge-container {
        width: calc(50% - 40px);
        max-width: 460px;
      }
      .sensor-group {
        justify-content: space-evenly;
      }
    }
  </style>
</head>
<body>
  <div class="header">
    <h1>ÇAKIR KULUÇKA</h1>
  </div>

  <div id="errorDisplay" class="error-message"></div>
  <div class="last-update">Son Güncelleme: <span id="lastUpdateTime">Bilinmiyor</span></div>

  <div class="sensor-group">
    <div class="gauge-container">
      <canvas id="temperatureGauge1" aria-label="Birinci sensör sıcaklık göstergesi"></canvas>
      <div class="gauge-icon temp-icon"><i class="fas fa-thermometer-half"></i>Sıcaklık 1</div>
    </div>
    <div class="gauge-container">
      <canvas id="humidityGauge1" aria-label="Birinci sensör nem göstergesi"></canvas>
      <div class="gauge-icon humidity-icon"><i class="fas fa-tint"></i>Nem 1</div>
    </div>
  </div>
  <div class="sensor-group">
    <div class="gauge-container">
      <canvas id="temperatureGauge2" aria-label="İkinci sensör sıcaklık göstergesi"></canvas>
      <div class="gauge-icon temp-icon"><i class="fas fa-thermometer-half"></i>Sıcaklık 2</div>
    </div>
    <div class="gauge-container">
      <canvas id="humidityGauge2" aria-label="İkinci sensör nem göstergesi"></canvas>
      <div class="gauge-icon humidity-icon"><i class="fas fa-tint"></i>Nem 2</div>
    </div>
  </div>
  <div class="footer">
    <div>📞 <a href="tel:05363289962">0536 328 9962</a></div>
    <div>✉️ <a href="mailto:myfavoritess@gmail.com">myfavoritess@gmail.com</a></div>
    <div class="developer-info">Mehmet ÇAKIR | Sürüm: v1.1 (Geliştirilmiş)</div>
    <div class="update-btn-container">
      <a href="/update" class="update-btn"><i class="fas fa-sync-alt update-icon"></i>OTA Güncellemesi</a>
      <a href="/calibration" style="color: inherit; text-decoration: none; margin-left: 10px;">⚙</a>
    </div>
  </div>
  <script>
    const DEBUG_MODE = false;

    function logger(message, ...optionalParams) {
      if (DEBUG_MODE) {
        console.log(message, ...optionalParams);
      }
    }

    let esp32IP = window.location.hostname;
    logger('ESP32 IP:', esp32IP);

    const canvases = {
      temp1: document.getElementById('temperatureGauge1'),
      hum1: document.getElementById('humidityGauge1'),
      temp2: document.getElementById('temperatureGauge2'), // Corrected: was humidityGauge2
      hum2: document.getElementById('humidityGauge2')
    };

    const contexts = {};
    for (const key in canvases) {
      if (canvases[key]) {
        contexts[key] = canvases[key].getContext('2d');
      } else {
        console.error(`Canvas öğesi bulunamadı: ${key}`);
      }
    }

    let gaugeGlobalDimensions = {};
    let resizeObserver = null;

    const tempColors = [
      { temp: 0, color: 'rgb(100,200,255)' },
      { temp: 15, color: 'rgb(128,255,0)' },
      { temp: 28, color: 'rgb(255,255,0)' },
      { temp: 37.0, color: 'rgb(255,200,0)' },
      { temp: 38.5, color: 'rgb(255,165,0)' },
      { temp: 42, color: 'rgb(255,69,0)' },
      { temp: 50, color: 'rgb(200,0,0)' }
    ];
    const maxTemp = 50;

    const humidityColors = [
      { humidity: 0, color: 'rgb(255,220,170)' },
      { humidity: 30, color: 'rgb(173,216,230)' },
      { humidity: 50, color: 'rgb(100,180,220)'},
      { humidity: 65, color: 'rgb(70,130,180)' },
      { humidity: 80, color: 'rgb(0,0,205)' },
      { humidity: 100, color: 'rgb(0,0,128)' }
    ];
    const maxHumidity = 100;

    const errorDisplay = document.getElementById('errorDisplay');
    const lastUpdateTime = document.getElementById('lastUpdateTime');

    function debounce(func, wait) {
      let timeout;
      return function executedFunction(...args) {
        const later = () => {
          clearTimeout(timeout);
          func(...args);
        };
        clearTimeout(timeout);
        timeout = setTimeout(later, wait);
      };
    }

    function setupCanvasDimensions(canvas) {
      if (!canvas) return false;
      const dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      const cssWidth = rect.width;
      const cssHeight = rect.height;

      if (cssWidth === 0 || cssHeight === 0) {
        logger(`Canvas ${canvas.id} boyutları sıfır, atlanıyor.`);
        return false;
      }
      canvas.width = cssWidth * dpr;
      canvas.height = cssHeight * dpr;
      const ctx = canvas.getContext('2d');
      ctx.scale(dpr, dpr);
      return true;
    }

    function calculateGaugeDimensions(canvas) {
      if (!canvas || !canvas.clientWidth) {
        logger("calculateGaugeDimensions için geçersiz canvas.");
        return { centerX: 100, centerY: 80, radius: 70, fontSize: 18, lineWidth: 12, valid: false };
      }
      const canvasWidth = canvas.clientWidth;
      const canvasHeight = canvas.clientHeight;
      let config;
      const windowWidth = window.innerWidth;

      if (windowWidth <= 768) {
        config = { radiusFactor: 0.72, fontSizeFactor: 0.12, lineWidthFactor: 0.1, centerYOffsetFactor: 0.82 };
      } else if (windowWidth <= 1024) {
        config = { radiusFactor: 0.78, fontSizeFactor: 0.1, lineWidthFactor: 0.09, centerYOffsetFactor: 0.85 };
      } else {
        config = { radiusFactor: 0.8, fontSizeFactor: 0.09, lineWidthFactor: 0.08, centerYOffsetFactor: 0.88 };
      }

      const radius = (Math.min(canvasWidth, canvasHeight * 1.7)) * config.radiusFactor / 2;

      return {
        centerX: canvasWidth / 2,
        centerY: canvasHeight * config.centerYOffsetFactor,
        radius: Math.max(10, radius),
        fontSize: Math.max(10, canvasWidth * config.fontSizeFactor),
        lineWidth: Math.max(4, canvasWidth * config.lineWidthFactor),
        valid: true
      };
    }

    function setupAllCanvasesAndDimensions() {
      let allCanvasesValid = true;
      for (const key in canvases) {
        if (canvases[key]) {
          if (!setupCanvasDimensions(canvases[key])) {
            allCanvasesValid = false;
          }
        } else {
          allCanvasesValid = false;
        }
      }

      const anyValidCanvas = Object.values(canvases).find(c => c && c.clientWidth > 0);
      if (anyValidCanvas) {
        gaugeGlobalDimensions = calculateGaugeDimensions(anyValidCanvas);
        if (!gaugeGlobalDimensions.valid) {
          logger("Gösterge boyutları hesaplanamadı, ancak canvaslar mevcut. Çizim başarısız olabilir.");
        }
      } else {
        gaugeGlobalDimensions = { valid: false };
        logger("Geçerli canvas öğesi bulunamadı veya boyutları sıfır.");
      }

      return allCanvasesValid && gaugeGlobalDimensions.valid;
    }

    function getColorForValue(value, colors, valueKey) {
      if (isNaN(value) || value === null) return 'rgb(200,200,200)';

      const firstColorPoint = colors[0];
      const lastColorPoint = colors[colors.length - 1];

      if (value <= firstColorPoint[valueKey]) return firstColorPoint.color;
      if (value >= lastColorPoint[valueKey]) return lastColorPoint.color;

      for (let i = 0; i < colors.length - 1; i++) {
        const p1 = colors[i];
        const p2 = colors[i + 1];
        if (value >= p1[valueKey] && value <= p2[valueKey]) {
          const ratio = (p2[valueKey] === p1[valueKey]) ? 1 : (value - p1[valueKey]) / (p2[valueKey] - p1[valueKey]);
          const c1 = p1.color.match(/\d+/g).map(Number);
          const c2 = p2.color.match(/\d+/g).map(Number);
          const r = Math.round(c1[0] + ratio * (c2[0] - c1[0]));
          const g = Math.round(c1[1] + ratio * (c2[1] - c1[1]));
          const b = Math.round(c1[2] + ratio * (c2[2] - c1[2]));
          return `rgb(${r},${g},${b})`;
        }
      }
      return lastColorPoint.color;
    }

    function drawHalfCircleGauge(ctx, value, maxValue, colors, valueKey, sensorConnected = true, isLoading = false) {
      if (!ctx) {
        logger("Canvas bağlamı bulunamadı!", ctx);
        return;
      }

      const { centerX, centerY, radius, fontSize, lineWidth, valid: dimValid } = gaugeGlobalDimensions;
      const cssWidth = ctx.canvas.clientWidth;
      const cssHeight = ctx.canvas.clientHeight;
      ctx.clearRect(0, 0, cssWidth, cssHeight);

      if (!dimValid || radius <= 0) {
        logger("Geçersiz gösterge boyutları veya sıfır yarıçap.", gaugeGlobalDimensions);
        ctx.font = `bold ${Math.min(16, cssWidth / 15)}px Arial`;
        ctx.fillStyle = '#dc3545';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText("Boyut Hatası", cssWidth / 2, cssHeight / 2);
        return;
      }

      const startAngle = Math.PI;
      const endAngle = 0;
      const textYPos = centerY - (fontSize * 0.1);

      ctx.beginPath();
      ctx.arc(centerX, centerY, radius, startAngle, endAngle, false);
      ctx.lineWidth = lineWidth;
      ctx.strokeStyle = '#e9ecef';
      ctx.lineCap = 'round';
      ctx.stroke();

      if (isLoading) {
        ctx.font = `italic ${fontSize * 0.85}px Arial`;
        ctx.fillStyle = '#6c757d';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText("Yükleniyor...", centerX, textYPos);
        return;
      }

      if (!sensorConnected) {
        ctx.font = `bold ${fontSize * 0.7}px Arial`;
        ctx.fillStyle = '#6c757d';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        const line1 = "Sensör";
        const line2 = "Bağlı Değil";
        const textHeight = fontSize * 0.7;
        ctx.fillText(line1, centerX, textYPos - textHeight * 0.7);
        ctx.fillText(line2, centerX, textYPos + textHeight * 0.7);
        return;
      }

      if (isNaN(value) || value === null) {
        ctx.font = `bold ${fontSize}px Arial`;
        ctx.fillStyle = '#343a40';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText("--", centerX, textYPos);
        return;
      }

      const angleRange = Math.PI;
      const valueRatio = Math.min(Math.max(value, 0), maxValue) / maxValue;
      const drawAngle = angleRange * valueRatio;

      ctx.beginPath();
      ctx.arc(centerX, centerY, radius, startAngle, startAngle + drawAngle, false);
      ctx.strokeStyle = getColorForValue(value, colors, valueKey);
      ctx.stroke();

      ctx.font = `bold ${fontSize}px Arial`;
      ctx.fillStyle = '#343a40';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      const unit = valueKey === 'temp' ? ' °C' : ' %';
      ctx.fillText(value.toFixed(1) + unit, centerX, textYPos);
    }

    let lastSensorData = null;
    let fetchIntervalId = null;

    function drawLoadingStateForAllGauges() {
      if (!gaugeGlobalDimensions.valid) {
        if (!setupAllCanvasesAndDimensions()) {
          logger("Canvas kurulumu başarısız, yükleme durumu çizilemez.");
          return;
        }
      }
      if (!gaugeGlobalDimensions.valid) {
        logger("Boyutlar hala geçersiz, yükleme durumu çizilemez.");
        return;
      }

      for (const key in contexts) {
        if (contexts[key]) {
          const isTemp = key.startsWith('temp');
          drawHalfCircleGauge(contexts[key], NaN, isTemp ? maxTemp : maxHumidity, isTemp ? tempColors : humidityColors, isTemp ? 'temp' : 'humidity', true, true);
        }
      }
    }

    async function fetchSensorData() {
      if (!gaugeGlobalDimensions.valid) {
        if (!setupAllCanvasesAndDimensions()) {
          logger("Canvas kurulumu başarısız, veri çekme ertelendi.");
          if (errorDisplay) {
            errorDisplay.textContent = "Gösterge öğeleri düzgün yüklenemedi. Lütfen sayfayı yenileyin.";
            errorDisplay.style.display = 'block';
          }
          return;
        }
      }
      if (!gaugeGlobalDimensions.valid) {
        logger("Boyutlar hala geçersiz, veri çekme atlanıyor.");
        return;
      }

      drawLoadingStateForAllGauges();

      try {
        logger(`Veri çekiliyor: http://${esp32IP}/data`);
        if (errorDisplay) errorDisplay.style.display = 'none';

        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 7500);

        const response = await fetch(`http://${esp32IP}/data`, {
          method: 'GET',
          headers: { 'Accept': 'application/json' },
          signal: controller.signal
        });
        clearTimeout(timeoutId);

        if (!response.ok) {
          throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
        const data = await response.json();
        lastSensorData = data;
        logger("Sensör verisi alındı:", JSON.stringify(data, null, 2));
        updateGauges(data);
        lastUpdateTime.textContent = new Date().toLocaleString('tr-TR');

      } catch (error) {
        logger("Sensör verisi çekilemedi:", error.message, error.stack);
        if (errorDisplay) {
          errorDisplay.textContent = `Sensör verileri alınamadı (${error.name === 'AbortError' ? 'Zaman Aşımı' : error.message}). ${lastSensorData ? 'Son bilinen veriler gösteriliyor.' : 'Test verileri kullanılıyor.'}`;
          errorDisplay.style.display = 'block';
        }

        if (lastSensorData) {
          logger("Alma hatası nedeniyle son bilinen iyi veriler kullanılıyor.");
          updateGauges(lastSensorData);
        } else {
          logger("Alma hatası ve önceki veri olmadığı için test verileri kullanılıyor.");
          const mockData = {
            sht45_1Connected: false, temp1: NaN, hum1: NaN,
            sht45_2Connected: false, temp2: NaN, hum2: NaN
          };
          updateGauges(mockData, true);
        }
      }
    }

    function updateGauges(data, isMockDataOnError = false) {
      if (!gaugeGlobalDimensions.valid) {
        logger("updateGauges: Boyutlar geçersiz, çizim atlanıyor.");
        return;
      }
      const s1Connected = data.sht45_1Connected !== undefined ? data.sht45_1Connected : (isMockDataOnError ? false : true);
      const s2Connected = data.sht45_2Connected !== undefined ? data.sht45_2Connected : (isMockDataOnError ? false : true);

      const temp1Value = s1Connected && Number.isFinite(parseFloat(data.temp1)) ? parseFloat(data.temp1) : NaN;
      const hum1Value = s1Connected && Number.isFinite(parseFloat(data.hum1)) ? parseFloat(data.hum1) : NaN;
      const temp2Value = s2Connected && Number.isFinite(parseFloat(data.temp2)) ? parseFloat(data.temp2) : NaN;
      const hum2Value = s2Connected && Number.isFinite(parseFloat(data.hum2)) ? parseFloat(data.hum2) : NaN;

      logger(`Sensör 1: sıcaklık=${temp1Value}, nem=${hum1Value}, bağlı=${s1Connected}`);
      logger(`Sensör 2: sıcaklık=${temp2Value}, nem=${hum2Value}, bağlı=${s2Connected}`);

      if (contexts.temp1) drawHalfCircleGauge(contexts.temp1, temp1Value, maxTemp, tempColors, 'temp', s1Connected);
      if (contexts.hum1) drawHalfCircleGauge(contexts.hum1, hum1Value, maxHumidity, humidityColors, 'humidity', s1Connected);
      if (contexts.temp2) drawHalfCircleGauge(contexts.temp2, temp2Value, maxTemp, tempColors, 'temp', s2Connected);
      if (contexts.hum2) drawHalfCircleGauge(contexts.hum2, hum2Value, maxHumidity, humidityColors, 'humidity', s2Connected);
    }

    function startDataRefresh() {
      logger("Veri yenileme döngüsü başlatılıyor...");
      if (fetchIntervalId) clearInterval(fetchIntervalId);
      fetchSensorData();
      fetchIntervalId = setInterval(fetchSensorData, 2500);
    }

    const debouncedResizeHandler = debounce(() => {
      logger("Pencere yeniden boyutlandırıldı (debounced), canvaslar yeniden ayarlanıyor ve göstergeler tekrar çiziliyor...");
      if (setupAllCanvasesAndDimensions()) {
        if (lastSensorData) {
          updateGauges(lastSensorData);
        } else {
          drawLoadingStateForAllGauges();
        }
      } else {
        logger("Yeniden boyutlandırma sonrası canvas kurulumu başarısız.");
        if (errorDisplay) {
          errorDisplay.textContent = "Ekran yeniden boyutlandırılırken bir sorun oluştu. Lütfen sayfayı yenileyin.";
          errorDisplay.style.display = 'block';
        }
      }
    }, 300);

    function setupResizeObserver() {
      if (resizeObserver) resizeObserver.disconnect();

      resizeObserver = new ResizeObserver(entries => {
        let needsRedraw = false;
        for (let entry of entries) {
          if (Object.values(canvases).includes(entry.target)) {
            logger(`Canvas ${entry.target.id} yeniden boyutlandırıldı.`);
            needsRedraw = true;
          }
        }
        if (needsRedraw) {
          debouncedResizeHandler();
        }
      });

      for (const key in canvases) {
        if (canvases[key]) {
          resizeObserver.observe(canvases[key]);
        }
      }
      logger("ResizeObserver kurulumu tamamlandı.");
    }

    window.addEventListener('load', () => {
      logger("Sayfa yüklendi, başlatılıyor...");
      let allContextsAvailable = true;
      for (const key in contexts) {
        if (!contexts[key]) {
          allContextsAvailable = false;
          console.error(`Canvas bağlamı ${key} yüklenemedi.`);
        }
      }

      if (!allContextsAvailable) {
        console.error("Bir veya daha fazla canvas bağlamı bulunamadı. Göstergeler çalışmayabilir.");
        if (errorDisplay) {
          errorDisplay.textContent = "Kritik Hata: Gösterge öğeleri yüklenemedi. Geliştiriciyle iletişime geçin.";
          errorDisplay.style.display = 'block';
        }
        return;
      }

      if (setupAllCanvasesAndDimensions()) {
        setupResizeObserver();
        startDataRefresh();
      } else {
        logger("İlk canvas kurulumu başarısız.");
        if (errorDisplay) {
          errorDisplay.textContent = "Göstergeler başlatılamadı. İnternet bağlantınızı kontrol edin ve yenileyin.";
          errorDisplay.style.display = 'block';
        }
      }
    });

    window.addEventListener('beforeunload', () => {
      logger("Sayfa kapanıyor, aralıklar ve gözlemciler temizleniyor...");
      if (fetchIntervalId) clearInterval(fetchIntervalId);
      if (resizeObserver) resizeObserver.disconnect();
    });
  </script>
</body>
</html>
)=====";

// Kalibrasyon Sayfası
String getCalibrationPage() {
  String page = R"=====(
<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Sensör Kalibrasyonu</title>
  <!-- Tailwind CSS CDN -->
  <script src="https://cdn.tailwindcss.com"></script>
  <!-- Inter Font -->
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700&display=swap" rel="stylesheet">
  <!-- Font Awesome -->
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css" crossorigin="anonymous" referrerpolicy="no-referrer" />
  <style>
    body { font-family: 'Inter', sans-serif; }
  </style>
</head>
<body class="bg-gray-100 min-h-screen flex flex-col items-center justify-center p-4">
  <div class="bg-white p-8 rounded-xl shadow-lg w-full max-w-md">
    <h1 class="text-3xl font-bold text-blue-700 mb-6 text-center">Sensör Kalibrasyonu</h1>
    <p class="text-gray-600 mb-6 text-center text-sm">
      Sensör okumalarında küçük ayarlamalar yapmak için aşağıdaki ofset değerlerini girin.
      Değerleri ondalık olarak girebilirsiniz (örn: 0.5 veya -1.2).
    </p>
    <form action="/calibrate" method="POST" class="space-y-4">
      <!-- Sensor 1 Offsets -->
      <div class="border-b pb-4 border-gray-200">
        <h2 class="text-xl font-semibold text-gray-800 mb-3">Sensör 1</h2>
        <div>
          <label for="temp1Offset" class="block text-gray-700 font-medium mb-1">Sıcaklık Ofseti (°C):</label>
          <input type="number" step="0.1" id="temp1Offset" name="temp1Offset" value=")=====" + String(temp1Offset) + R"=====("
                 class="w-full p-2 border border-gray-300 rounded-md focus:ring-blue-500 focus:border-blue-500 transition duration-200"
                 aria-label="Sensör 1 Sıcaklık Ofseti">
        </div>
        <div class="mt-4">
          <label for="hum1Offset" class="block text-gray-700 font-medium mb-1">Nem Ofseti (%):</label>
          <input type="number" step="0.1" id="hum1Offset" name="hum1Offset" value=")=====" + String(hum1Offset) + R"=====("
                 class="w-full p-2 border border-gray-300 rounded-md focus:ring-blue-500 focus:border-blue-500 transition duration-200"
                 aria-label="Sensör 1 Nem Ofseti">
        </div>
      </div>

      <!-- Sensor 2 Offsets -->
      <div class="pt-4">
        <h2 class="text-xl font-semibold text-gray-800 mb-3">Sensör 2</h2>
        <div>
          <label for="temp2Offset" class="block text-gray-700 font-medium mb-1">Sıcaklık Ofseti (°C):</label>
          <input type="number" step="0.1" id="temp2Offset" name="temp2Offset" value=")=====" + String(temp2Offset) + R"=====("
                 class="w-full p-2 border border-gray-300 rounded-md focus:ring-blue-500 focus:border-blue-500 transition duration-200"
                 aria-label="Sensör 2 Sıcaklık Ofseti">
        </div>
        <div class="mt-4">
          <label for="hum2Offset" class="block text-gray-700 font-medium mb-1">Nem Ofseti (%):</label>
          <input type="number" step="0.1" id="hum2Offset" name="hum2Offset" value=")=====" + String(hum2Offset) + R"=====("
                 class="w-full p-2 border border-gray-300 rounded-md focus:ring-blue-500 focus:border-blue-500 transition duration-200"
                 aria-label="Sensör 2 Nem Ofseti">
        </div>
      </div>

      <button type="submit"
              class="w-full bg-green-600 hover:bg-green-700 text-white font-bold py-3 px-4 rounded-lg shadow-md transition duration-300 transform hover:scale-105">
        Kalibrasyonu Kaydet
      </button>
    </form>
    <div class="text-center mt-6">
      <a href="/" class="text-blue-600 hover:text-blue-800 font-medium transition duration-200 flex items-center justify-center">
        <i class="fas fa-arrow-left mr-2"></i> Ana Sayfaya Dön
      </a>
    </div>
  </div>
</body>
</html>
)=====";
  return page;
} // <-- Bu kapanış parantezi eklendi

void setup() {
  Serial.begin(115200);
  Serial.println(F("\nCihaz Başlatılıyor...")); // Device Starting...

  I2C_1.begin(SDA_PIN_1, SCL_PIN_1, 100000);
  I2C_2.begin(SDA_PIN_2, SCL_PIN_2, 100000);

  // I2C Bus Taraması
  scanI2CBus(I2C_1, "I2C_1");
  scanI2CBus(I2C_2, "I2C_2");

  // OLED Başlat
  if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS, true)) {
    Serial.println(F("SSD1306 OLED başlatma başarısız!")); // SSD1306 OLED initialization failed!
    while (1);
  }
  Serial.println(F("OLED başlatıldı.")); // OLED initialized.

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println(F("CAKIR"));
  display.setCursor(0, 30);
  display.println(F("ELEKTRONIK"));
  display.setTextSize(1);
  display.setCursor(0, 55);
  display.println(F("TEL:05363289962"));
  display.display();
  delay(3000);

  initialize_sht45_1();
  initialize_sht45_2();
  readSensors();

  currentDisplayMode = MODE_AP;
  currentAPDisplayState = AP_INFO;
  lastScreenSwitchTime = millis();
  handleOLEDDisplay();

  WiFiManager wifiManager;
  wifiManager.setConnectTimeout(20);
  wifiManager.setConfigPortalTimeout(180);

  wifiManager.setAPCallback([](WiFiManager *myWiFiManager) {
    Serial.print(F("WiFi Manager AP Modunda: ")); // WiFi Manager in AP Mode:
    Serial.println(myWiFiManager->getConfigPortalSSID());
    Serial.print(F("AP IP adresi: ")); // AP IP address:
    Serial.println(WiFi.softAPIP());
    currentDisplayMode = MODE_AP;
    currentAPDisplayState = AP_INFO;
    lastScreenSwitchTime = millis();
    handleOLEDDisplay();
  });

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
    Serial.println(F("WiFi Olayı: STA_GOT_IP")); // WiFi Event: STA_GOT_IP
    Serial.print(F("Bağlı IP: ")); // Connected IP:
    Serial.println(WiFi.localIP());
    currentDisplayMode = MODE_WIFI_CONNECTED;
    currentWiFiDisplayState = WIFI_INFO_COMPACT;
    lastScreenSwitchTime = millis();
    timeClient.begin();
    Serial.println(F("NTP istemcisi başlatıldı.")); // NTP client started.
    handleOLEDDisplay();
  }, ARDUINO_EVENT_WIFI_STA_GOT_IP);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
    Serial.println(F("WiFi Olayı: STA_DISCONNECTED")); // WiFi Event: STA_DISCONNECTED
    currentDisplayMode = MODE_AP;
    currentAPDisplayState = AP_INFO;
    lastScreenSwitchTime = millis();
    handleOLEDDisplay();
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  if (!wifiManager.autoConnect(apSSID, apPassword)) {
    Serial.println(F("WiFi bağlantı zaman aşımına uğradı veya yapılandırma tamamlanmadı. AP modunda bekleniyor.")); // WiFi connection timed out or configuration not completed. Waiting in AP mode.
  } else {
    Serial.println(F("WiFi bağlandı.")); // WiFi connected.
    Serial.print(F("SSID: ")); Serial.println(WiFi.SSID());
    Serial.print(F("IP Adresi: ")); Serial.println(WiFi.localIP());
  }

  // OTA Güncelleme Kurulumu
  httpUpdater.setup(&server, "/update", "admin", "password");

  // Kök Sayfa
  server.on("/", HTTP_GET, []() { server.send(200, "text/html", FPSTR(htmlPage)); });

  // Veri Uç Noktası
  server.on("/data", HTTP_GET, []() {
    DynamicJsonDocument doc(512);
    doc["sht45_1Connected"] = sht45_1Connected;
    if (sht45_1Connected && !isnan(temp1)) doc["temp1"] = temp1 + temp1Offset;
    else doc["temp1"] = nullptr;
    if (sht45_1Connected && !isnan(hum1)) doc["hum1"] = hum1 + hum1Offset;
    else doc["hum1"] = nullptr;

    doc["sht45_2Connected"] = sht45_2Connected;
    if (sht45_2Connected && !isnan(temp2)) doc["temp2"] = temp2 + temp2Offset;
    else doc["temp2"] = nullptr;
    if (sht45_2Connected && !isnan(hum2)) doc["hum2"] = hum2 + hum2Offset;
    else doc["hum2"] = nullptr;

    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
  });

  // Kalibrasyon Sayfası
  server.on("/calibration", HTTP_GET, []() {
    server.send(200, "text/html", getCalibrationPage());
  });

  // Kalibrasyonu Kaydet
  server.on("/calibrate", HTTP_POST, []() {
    if (server.hasArg("temp1Offset")) temp1Offset = server.arg("temp1Offset").toFloat();
    if (server.hasArg("hum1Offset")) hum1Offset = server.arg("hum1Offset").toFloat();
    if (server.hasArg("temp2Offset")) temp2Offset = server.arg("temp2Offset").toFloat();
    if (server.hasArg("hum2Offset")) hum2Offset = server.arg("hum2Offset").toFloat();
    saveCalibrationOffsets();
    server.send(200, "text/html", "<h1>Kalibrasyon Kaydedildi</h1><a href='/'>Ana Sayfaya Dön</a>"); // Calibration Saved, Return to Home
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "Sayfa Bulunamadı"); // Page Not Found
  });

  server.begin();
  Serial.println(F("Web sunucusu başlatıldı.")); // Web server started.

  // Kalibrasyon Ofsetlerini Yükle
  loadCalibrationOffsets();
}

void loop() {
  unsigned long currentMillis = millis();
  server.handleClient();

  if (WiFi.status() == WL_CONNECTED) {
    static unsigned long lastNtpUpdateTime = 0;
    if (currentMillis - lastNtpUpdateTime > 3600000 || !timeClient.isTimeSet()) {
      if (timeClient.update()) {
        Serial.println("NTP Güncellendi: " + timeClient.getFormattedTime()); // NTP Updated
      } else {
        Serial.println("NTP Güncelleme Başarısız"); // NTP Update Failed
      }
      lastNtpUpdateTime = currentMillis;
    }
  } else {
    checkWiFiConnection();
  }

  if (currentMillis - previousMillisSensors >= intervalSensors) {
    previousMillisSensors = currentMillis;
    readSensors();
  }

  if (currentMillis - previousMillisOLED >= intervalOLED) {
    previousMillisOLED = currentMillis;
    handleOLEDDisplay();
  }
}
