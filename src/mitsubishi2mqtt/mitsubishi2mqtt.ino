/*
  mitsubishi2mqtt - Mitsubishi Heat Pump to MQTT control for Home Assistant.
  Copyright (c) 2019 gysmo38, dzungpv, shampeon, endeavour, jascdk, chrdavis, alekslyse, maxmacSTN.  All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "FS.h" // SPIFFS for store config
#ifdef ESP32
#include <WiFi.h> // WIFI for ESP32
#include <WiFiUdp.h>
#include <ESPmDNS.h>   // mDNS for ESP32
#include <WebServer.h> // webServer for ESP32
#include "SPIFFS.h"    // ESP32 SPIFFS for store config
#include <esp_task_wdt.h> // Watchdog
WebServer server(80);     // ESP32 web

#else
#include <ESP8266WiFi.h> // WIFI for ESP8266
#include <WiFiClient.h>
#include <ESP8266mDNS.h>      // mDNS for ESP8266
#include <ESP8266WebServer.h> // webServer for ESP8266
ESP8266WebServer server(80); // ESP8266 web
#endif

#include "logger.h"
#include <ArduinoJson.h>       // json to process MQTT: ArduinoJson 6.11.4
#include <PubSubClient.h>      // MQTT: PubSubClient 2.8.0
#include <DNSServer.h>         // DNS for captive portal
#include <math.h>              // for rounding to Fahrenheit values
#include <ArduinoOTA.h>        // for OTA
#include <HeatPump.h>          // SwiCago library: https://github.com/SwiCago/HeatPump
#include "config.h"            // config file
#include "html_common.h"       // common code HTML (like header, footer)
#include "javascript_common.h" // common code javascript (like refresh page)
#include "html_init.h"         // code html for initial config
#include "html_menu.h"         // code html for menu
#include "html_pages.h"        // code html for pages

#define TAG "mainApp"

/* Multi Reset Detector for ESP8266*/
#ifdef ESP8266
#define ESP8266_MRD_USE_RTC false // true
#define ESP_MRD_USE_LITTLEFS false
#define ESP_MRD_USE_SPIFFS false
#define ESP_MRD_USE_EEPROM true
// These definitions must be placed before #include <ESP_MultiResetDetector.h> to be used
// Otherwise, default values (MRD_TIMES = 3, MRD_TIMEOUT = 10 seconds and MRD_ADDRESS = 0) will be used
// Number of subsequent resets during MRD_TIMEOUT to activate
#define MRD_TIMES 5 // Press reset button for 5 times
// Number of seconds after reset during which a
// subsequent reset will be considered a multi reset.
#define MRD_TIMEOUT 10
// RTC/EEPROM Memory Address for the MultiResetDetector to use
#define MRD_ADDRESS 0
#include <ESP_MultiResetDetector.h> //https://github.com/khoih-prog/ESP_MultiResetDetector
MultiResetDetector *mrd;
#endif

// Special setup for ESP8266 & ESP32 Variants
#ifdef ESP8266
HardwareSerial *acSerial(&Serial);

#endif
#ifdef ESP32
HardwareSerial *acSerial(&Serial0);
// Set stack size to 16KB (8KB is likely to crash).
SET_LOOP_TASK_STACK_SIZE(16 * 1024); // 16KB

enum btnAction
{
  noPress,
  shortPress,
  longPress,
  longLongPress
};
volatile unsigned long BTNPresedTime = 0;
volatile bool btnPressed = false;
uint8_t btnAction = noPress;
#endif

// wifi, mqtt and heatpump client instances
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

// Captive portal variables, only used for config page
const byte DNS_PORT = 53;
IPAddress apIP(8, 8, 8, 8);
IPAddress netMsk(255, 255, 255, 0);
DNSServer dnsServer;

boolean captive = false;
boolean mqtt_config = false;
boolean wifi_config = false;

// HVAC
HeatPump hp;
unsigned long lastUpdate ;
unsigned long lastCommandSend;
unsigned long lastMqttRetry;
unsigned long lastHpSync;
unsigned int hpConnectionRetries;
unsigned int hpConnectionTotalRetries;
float energy = 0; // kWh
bool previousCMDisPower = true;

// Local state
StaticJsonDocument<JSON_OBJECT_SIZE(14)> rootInfo;
// StaticJsonDocument<256>  rootInfo;

// Web OTA
int uploaderror = 0;

// Prototypes
void wifiFactoryReset();
void testMode();
String getId();
void setDefaults();
bool loadOthers();
bool loadUnit();
bool initWifi();
bool loadWifi();
void handleInitSetup();
void handleSaveInit();
void handleSaveWifi();
void handleReboot();
void handleNotFound();
void mqttConnect();
void mqttCallback(char *topic, byte *payload, unsigned int length);
bool connectWifi();
bool checkLogin();
float convertCelsiusToLocalUnit(float temperature, bool isFahrenheit);
float convertLocalUnitToCelsius(float temperature, bool isFahrenheit);
heatpumpSettings change_states(heatpumpSettings settings);
String getTemperatureScale();
bool is_authenticated();
String hpGetMode(heatpumpSettings hvacSettings);
void hpStatusChanged(heatpumpStatus currentStatus);
void readHPstate();
void playBeep(Buzzer_preset buzzer_preset);
void updateUnitSettings();

#ifdef ESP8266
// Check multiple reset detector.
void checkMRD()
{
  mrd = new MultiResetDetector(MRD_TIMEOUT, MRD_ADDRESS);
  if (mrd->detectMultiReset())
  {
    wifiFactoryReset();
    delay(1000);
    ESP.restart();
  }
}
#endif

void testMode()
{

#ifdef ESP8266

  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED_ACT, HIGH);
    delay(100);
    digitalWrite(LED_ACT, LOW);
    delay(100);
  }
  if (!hp.connect(acSerial))
  {
    while (1)
    {
      digitalWrite(LED_ACT, HIGH);
      delay(100);
      digitalWrite(LED_ACT, LOW);
      delay(100);
    }
  }
  digitalWrite(LED_ACT, HIGH);
  hp.setModeSetting("FAN");
  hp.setPowerSetting("ON");
  hp.update();
  delay(1000);
  hp.setPowerSetting("OFF");
  hp.update();

  SPIFFS.format();

  // Serial.println("format_done");

  while (1)
  {
    digitalWrite(LED_ACT, millis() / 1000 % 2);
    if (acSerial->available())
    {
      String cmd = Serial.readStringUntil('\n');
      if (cmd == "mac")
      {
        acSerial->println("mac");
        acSerial->println(WiFi.macAddress());
      }
      if (cmd == "wlan")
      {
        int numberOfNetworks = WiFi.scanNetworks();
        String wlan_list = "";
        for (int i = 0; i < numberOfNetworks; i++)
        {
          wlan_list += WiFi.SSID(i) + "\t" + String(WiFi.RSSI(i)) + "\n";
        }
        acSerial->println("wlan");
        acSerial->println(wlan_list);
      }
    }
  }

#endif

#ifdef ESP32

  acSerial->begin(115200);

  delay(3000);
  String res = "";
  bool wifireset = false;
  bool wifiScanning = false;
  if (acSerial->available() > 0)
  {
    res = acSerial->readStringUntil('\n');
    if (res.indexOf("TESTMODE") == -1)
    {
      return;
    }
  }
  else
  {
    acSerial->end();
    delay(1000);
    return;
  }
  acSerial->println("OK");

  Serial.begin(115200);

  Serial.println("-- Enter test mode --");

  playBeep(ON);

  while (1)
  {

    if (acSerial->available() > 0)
    {
      res = acSerial->readStringUntil('\n');

      Serial.println("HWSERIAL <" + res);

      if (res.indexOf("INIT") != -1)
      {
        wifiFactoryReset();
        acSerial->println("OK");
      }

      else if (res.indexOf("SWVERSION?") != -1)
      {
        acSerial->println(String("Mitsubishi2MQTT - " + String(m2mqtt_version)));
      }

      else if (res.indexOf("HWVERSION?") != -1)
      {
        acSerial->println(hardware_version);
      }


      else if (res.indexOf("VOLTAGE") != -1)
      {
        acSerial->println("OK");
        delay(50);
        acSerial->end();
        pinMode(PIN_AC_TX, OUTPUT);
        pinMode(PIN_AC_RX, OUTPUT);

        for (int i = 0; i < 10; i++)
        {
          digitalWrite(PIN_AC_TX, 0);
          digitalWrite(PIN_AC_RX, 0);
          delay(200);
          digitalWrite(PIN_AC_TX, 1);
          digitalWrite(PIN_AC_RX, 1);
          delay(200);
        }
        delay(50);
        acSerial->begin(115200);

      }

      else if (res.indexOf("mac?") != -1)
      {
        acSerial->println(WiFi.macAddress());
      }
      else if (res.indexOf("wlan?") != -1)
      {
        int numberOfNetworks = WiFi.scanNetworks();
        String wlan_list = "";
        for (int i = 0; i < numberOfNetworks; i++)
        {
          wlan_list += WiFi.SSID(i) + "\t" + String(WiFi.RSSI(i)) + "\t";
        }
        acSerial->print("wlan:");
        acSerial->println(wlan_list);
      

      }
      else if (res.indexOf("END") != -1)
      {
        playBeep(OFF);

        while (1)
        { 

            if (acSerial->available() > 0){
              res = acSerial->readStringUntil('\n');
              Serial.println("HWSERIAL <" + res);
              if (res.indexOf("CONNECTED?")!= -1){ 
                acSerial->println("OK");
              }
            }



          digitalWrite(LED_ACT, digitalRead(BTN_1));
          digitalWrite(LED_ON, digitalRead(BTN_1));
        }
      }
    }

}

#endif
}

void readHPstate(heatpumpSettings hpSettings, heatpumpStatus hpStatus)
{
  Log.ln(TAG, "** AC Status *****************************");
  Log.ln(TAG, "\tPower: " + String(hpSettings.power));
  Log.ln(TAG, "\tMode: " + String(hpSettings.mode) + "(" + String(hpStatus.operating ? "active" : "idle") + ")");
  float degc = hpSettings.temperature;
  float degf = degc * 1.8 + 32.0;
  Log.ln(TAG, "\tTarget: " + String(degc, 1));
  Log.ln(TAG, "\tFan: " + String(hpSettings.fan));
  Log.ln(TAG, "\tSwing: H:" + String(hpSettings.wideVane) + " V:" + String(hpSettings.vane));
  Log.ln(TAG, "\tInside: " + String(hpStatus.roomTemperature, 1));
  // Log.ln(TAG, "\tOutside: " + String(this->currentStatus.outsideTemperature, 1));
  // Log.ln(TAG, "\tCoil: " + String(this->currentStatus.coilTemperature, 1));
  Log.ln(TAG, "\tCompressor Freq: " + String(hpStatus.compressorFrequency) + " Hz");
  Log.ln(TAG, "\tPower: " + String(hpStatus.power) + " W");
  Log.ln(TAG, "\tEnergy: " + String(energy) + " kWh");

  Log.ln(TAG, "******************************************\n");
}

void wifiFactoryReset()
{
  for (int i = 0; i < 20; i++)
  {
    digitalWrite(LED_ACT, LED_OFF);
    delay(100);
    digitalWrite(LED_ACT, LED_ON);
    delay(100);
  }

  SPIFFS.format();


}

/*
  void tick()
  {
  //toggle state
  int state = digitalRead(LED_ACT);  // get the current state of GPIO2 pin
  digitalWrite(LED_ACT, !state);     // set pin to the opposite state
  }*/

bool loadWifi()
{
  ap_ssid = "";
  ap_pwd = "";
  if (!SPIFFS.exists(wifi_conf))
  {
    // Serial.println(F("Wifi config file not exist!"));
    return false;
  }
  File configFile = SPIFFS.open(wifi_conf, "r");
  if (!configFile)
  {
    // Serial.println(F("Failed to open wifi config file"));
    return false;
  }
  size_t size = configFile.size();
  if (size > 1024)
  {
    // Serial.println(F("Wifi config file size is too large"));
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);
  configFile.readBytes(buf.get(), size);
  const size_t capacity = JSON_OBJECT_SIZE(4) + 130;
  DynamicJsonDocument doc(capacity);
  deserializeJson(doc, buf.get());
  hostname = doc["hostname"].as<String>();
  ap_ssid = doc["ap_ssid"].as<String>();
  ap_pwd = doc["ap_pwd"].as<String>();
  // prevent ota password is "null" if not exist key
  if (doc.containsKey("ota_pwd"))
  {
    ota_pwd = doc["ota_pwd"].as<String>();
  }
  else
  {
    ota_pwd = "";
  }
  return true;
}

void playBeep(Buzzer_preset buzzer_p)
{
#ifdef ESP32
  if (!beep)
    return;

  switch (buzzer_p)
  {
  case ON:
    ledcWriteTone(0, BUZZER_FREQ);
    delay(100);
    ledcWriteTone(0, 0);
    delay(100);
    ledcWriteTone(0, BUZZER_FREQ);
    delay(100);
    ledcWriteTone(0, 0);
    break;

  case OFF:
    ledcWriteTone(0, BUZZER_FREQ);
    delay(400);
    ledcWriteTone(0, 0);
    break;

  case SET:
    ledcWriteTone(0, BUZZER_FREQ);
    delay(100);
    ledcWriteTone(0, 0);
  default:
    break;
  }
#endif
}

void saveMqtt(String mqttFn, String mqttHost, String mqttPort, String mqttUser,
              String mqttPwd, String mqttTopic)
{

  const size_t capacity = JSON_OBJECT_SIZE(6) + 400;
  DynamicJsonDocument doc(capacity);
  // if mqtt port is empty, we use default port
  if (mqttPort[0] == '\0')
    mqttPort = "1883";
  doc["mqtt_fn"] = mqttFn;
  doc["mqtt_host"] = mqttHost;
  doc["mqtt_port"] = mqttPort;
  doc["mqtt_user"] = mqttUser;
  doc["mqtt_pwd"] = mqttPwd;
  doc["mqtt_topic"] = mqttTopic;
  File configFile = SPIFFS.open(mqtt_conf, "w");
  if (!configFile)
  {
    // Serial.println(F("Failed to open config file for writing"));
  }
  serializeJson(doc, configFile);
  configFile.close();
}

void saveUnit(String tempUnit, String supportMode, String updateInterval, String loginPassword, String minTemp, String maxTemp, String tempStep, String beep, String ledEnabled)
{
  StaticJsonDocument<128> doc;
  // if temp unit is empty, we use default celcius
  if (tempUnit.isEmpty())
    tempUnit = "cel";
  doc["unit_tempUnit"] = tempUnit;
  // if minTemp is empty, we use default 16
  if (minTemp.isEmpty())
    minTemp = 16;
  doc["min_temp"] = minTemp;
  // if maxTemp is empty, we use default 31
  if (maxTemp.isEmpty())
    maxTemp = 31;
  doc["max_temp"] = maxTemp;
  // if tempStep is empty, we use default 1
  if (tempStep.isEmpty())
    tempStep = 1;
  doc["temp_step"] = tempStep;
  // if updateInterval is empty, we use default 15
  if (updateInterval.isEmpty())
    updateInterval = 15;
  doc["update_int"] = updateInterval;
  // if support mode is empty, we use default all mode
  if (supportMode.isEmpty())
    supportMode = "all";
  doc["support_mode"] = supportMode;
  // if login password is empty, we use empty
  if (loginPassword.isEmpty())
    loginPassword = "";
  if (beep.isEmpty())
    beep = "1";
  doc["beep"] = beep;
  if (ledEnabled.isEmpty())
    ledEnabled = "1";
  doc["ledEnabled"] = ledEnabled;

  doc["login_password"] = loginPassword;
  File configFile = SPIFFS.open(unit_conf, "w");
  if (!configFile)
  {
    // Serial.println(F("Failed to open config file for writing"));
  }
  serializeJson(doc, configFile);
  configFile.close();
}

void saveWifi(String apSsid, String apPwd, String hostName, String otaPwd)
{
  const size_t capacity = JSON_OBJECT_SIZE(4) + 130;
  DynamicJsonDocument doc(capacity);
  doc["ap_ssid"] = apSsid;
  doc["ap_pwd"] = apPwd;
  doc["hostname"] = hostName;
  doc["ota_pwd"] = otaPwd;
  File configFile = SPIFFS.open(wifi_conf, "w");
  if (!configFile)
  {
    // Serial.println(F("Failed to open wifi file for writing"));
  }
  serializeJson(doc, configFile);
  delay(10);
  configFile.close();
}

void saveEnergy(float cur_energy)
{
  const size_t capacity = JSON_OBJECT_SIZE(1);
  DynamicJsonDocument doc(capacity);
  doc["cur_energy"] = cur_energy;
  File energyFile = SPIFFS.open(energy_file, "w");
  if (!energyFile)
  {
    Serial.println(F("Failed to save energy file for writing"));
  }
  serializeJson(doc, energyFile);
  delay(10);
  energyFile.close();
}

void saveOthers(String haa, String haat, String availability_report, String debug)
{
  const size_t capacity = JSON_OBJECT_SIZE(4) + 130;
  DynamicJsonDocument doc(capacity);
  doc["haa"] = haa;
  doc["haat"] = haat;
  doc["avail_report"] = availability_report;
  doc["debug"] = debug;
  File configFile = SPIFFS.open(others_conf, "w");
  if (!configFile)
  {
    // Serial.println(F("Failed to open wifi file for writing"));
  }
  serializeJson(doc, configFile);
  delay(10);
  configFile.close();
}

void saveUnitFeedback(bool beepEnabled, bool ledEnabled){
  saveUnit(useFahrenheit?"fah":"cel",  supportHeatMode?"all":"nht", String(update_int/1000), login_password, String(min_temp), String(max_temp), temp_step, beep?"1":"0", ledEnabled?"1":"0");
}

// Initialize captive portal page
void initCaptivePortal()
{
  // Serial.println(F("Starting captive portal"));
  server.on("/", handleInitSetup);
  server.on("/save", handleSaveInit);
  server.on("/reboot", handleReboot);
  server.onNotFound(handleNotFound);
  server.begin();
  captive = true;
}

void initMqtt()
{
  mqtt_client.setServer(mqtt_server.c_str(), atoi(mqtt_port.c_str()));
  mqtt_client.setCallback(mqttCallback);
  mqtt_client.setKeepAlive(120);
  mqttConnect();
}

// Enable OTA only when connected as a client.
void initOTA()
{
  // write_log("Start OTA Listener");
  ArduinoOTA.setHostname(hostname.c_str());
  if (ota_pwd.length() > 0)
  {
    ArduinoOTA.setPassword(ota_pwd.c_str());
  }
  ArduinoOTA.onStart([]()
                     {
                       // write_log("Start");
                     });
  ArduinoOTA.onEnd([]()
                   {
                     // write_log("\nEnd");
                   });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
                          //    write_log("Progress: %u%%\r", (progress / (total / 100)));
                        });
  ArduinoOTA.onError([](ota_error_t error)
                     {
                       //    write_log("Error[%u]: ", error);
                       // if (error == OTA_AUTH_ERROR) Serial.println(F("Auth Failed"));
                       // else if (error == OTA_BEGIN_ERROR) Serial.println(F("Begin Failed"));
                       // else if (error == OTA_CONNECT_ERROR) Serial.println(F("Connect Failed"));
                       // else if (error == OTA_RECEIVE_ERROR) Serial.println(F("Receive Failed"));
                       // else if (error == OTA_END_ERROR) Serial.println(F("End Failed"));
                     });
  ArduinoOTA.begin();
}

bool loadMqtt()
{
  if (!SPIFFS.exists(mqtt_conf))
  {
    // Serial.println(F("MQTT config file not exist!"));
    return false;
  }
  // write_log("Loading MQTT configuration");
  File configFile = SPIFFS.open(mqtt_conf, "r");
  if (!configFile)
  {
    // write_log("Failed to open MQTT config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024)
  {
    // write_log("Config file size is too large");
    return false;
  }
  std::unique_ptr<char[]> buf(new char[size]);

  configFile.readBytes(buf.get(), size);
  const size_t capacity = JSON_OBJECT_SIZE(6) + 400;
  DynamicJsonDocument doc(capacity);
  deserializeJson(doc, buf.get());
  mqtt_fn = doc["mqtt_fn"].as<String>();
  mqtt_server = doc["mqtt_host"].as<String>();
  mqtt_port = doc["mqtt_port"].as<String>();
  mqtt_username = doc["mqtt_user"].as<String>();
  mqtt_password = doc["mqtt_pwd"].as<String>();
  mqtt_topic = doc["mqtt_topic"].as<String>();

  // write_log("=== START DEBUG MQTT ===");
  // write_log("Friendly Name" + mqtt_fn);
  // write_log("IP Server " + mqtt_server);
  // write_log("IP Port " + mqtt_port);
  // write_log("Username " + mqtt_username);
  // write_log("Password " + mqtt_password);
  // write_log("Topic " + mqtt_topic);
  // write_log("=== END DEBUG MQTT ===");

  mqtt_config = true;
  return true;
}

bool loadUnit()
{
  if (!SPIFFS.exists(unit_conf))
  {
    // Serial.println(F("Unit config file not exist!"));
    return false;
  }
  File configFile = SPIFFS.open(unit_conf, "r");
  if (!configFile)
  {
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024)
  {
    return false;
  }
  std::unique_ptr<char[]> buf(new char[size]);

  configFile.readBytes(buf.get(), size);
  StaticJsonDocument<256> doc;
  deserializeJson(doc, buf.get());
  // unit
  String unit_tempUnit = doc["unit_tempUnit"].as<String>();
  if (unit_tempUnit == "fah")
    useFahrenheit = true;
  min_temp = doc["min_temp"].as<uint8_t>();
  max_temp = doc["max_temp"].as<uint8_t>();
  temp_step = doc["temp_step"].as<String>();
  update_int = doc["update_int"].as<uint8_t>() * 1000;
  // mode
  String supportMode = doc["support_mode"].as<String>();
  if (supportMode == "nht")
    supportHeatMode = false;
  // prevent login password is "null" if not exist key
  if (doc.containsKey("login_password"))
  {
    login_password = doc["login_password"].as<String>();
  }
  else
  {
    login_password = "";
  }
  String beepStr = doc["beep"].as<String>();
  beep = beepStr == "1";

  String ledEnabledStr = doc["ledEnabled"].as<String>();
  ledEnabled = ledEnabledStr == "1";
  return true;
}

bool loadEnergy()
{
  if (!SPIFFS.exists(energy_file))
  {
    // Serial.println(F("Energy storage file not exist!"));
    return false;
  }
  File energyFile = SPIFFS.open(energy_file, "r");
  if (!energyFile)
  {
    return false;
  }

  size_t size = energyFile.size();
  if (size > 1024)
  {
    return false;
  }
  std::unique_ptr<char[]> buf(new char[size]);

  energyFile.readBytes(buf.get(), size);
  const size_t capacity = JSON_OBJECT_SIZE(1);
  DynamicJsonDocument doc(capacity);
  deserializeJson(doc, buf.get());
  // energy
  energy = doc["cur_energy"].as<float>();
  return true;
}

bool loadOthers()
{
  if (!SPIFFS.exists(others_conf))
  {
    // Serial.println(F("Others config file not exist!"));
    return false;
  }
  File configFile = SPIFFS.open(others_conf, "r");
  if (!configFile)
  {
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024)
  {
    return false;
  }
  std::unique_ptr<char[]> buf(new char[size]);

  configFile.readBytes(buf.get(), size);
  const size_t capacity = JSON_OBJECT_SIZE(3) + 200;
  DynamicJsonDocument doc(capacity);
  deserializeJson(doc, buf.get());
  // unit
  String unit_tempUnit = doc["unit_tempUnit"].as<String>();
  if (unit_tempUnit == "fah")
    useFahrenheit = true;
  others_haa_topic = doc["haat"].as<String>();
  String avail_report = doc["avail_report"].as<String>();
  String haa = doc["haa"].as<String>();
  String debug = doc["debug"].as<String>();

  if (strcmp(haa.c_str(), "OFF") == 0)
  {
    others_haa = false;
  }
  if (strcmp(avail_report.c_str(), "OFF") == 0)
  {
    others_avail_report = false;
  }
  if (strcmp(debug.c_str(), "ON") == 0)
  {
    _debugMode = true;
  }

  return true;
}

void setDefaults()
{
  ap_ssid = "";
  ap_pwd = "";
  others_haa = true;
  others_avail_report = true;
  others_haa_topic = "homeassistant";
}

boolean initWifi()
{
  bool connectWifiSuccess = true;
  if (ap_ssid[0] != '\0')
  {
    connectWifiSuccess = wifi_config = connectWifi();
    if (connectWifiSuccess)
    {
      digitalWrite(LED_ACT, LED_OFF);
      return true;
    }
    else
    {
      digitalWrite(LED_ACT, LED_ON);
      // reset hostname back to default before starting AP mode for privacy
      hostname = hostnamePrefix;
      hostname += getId();
    }
  }

  Log.ln(TAG, "Starting in AP mode");
  WiFi.mode(WIFI_AP);
  wifi_timeout = millis() + WIFI_RETRY_INTERVAL_MS;
  WiFi.persistent(false); // fix crash esp32 https://github.com/espressif/arduino-esp32/issues/2025
  WiFi.softAPConfig(apIP, apIP, netMsk);
  if (!connectWifiSuccess and login_password != "")
  {
    // Set AP password when falling back to AP on fail
    WiFi.softAP(hostname.c_str(), login_password.c_str());
  }
  else
  {
    // First time setup does not require password
    WiFi.softAP(hostname.c_str());
  }
  delay(2000); // VERY IMPORTANT

  Log.ln(TAG, "IP address: " + WiFi.softAPIP().toString());
  wifi_config = false;
  return false;
}

// Handler webserver response

void sendWrappedHTML(String content)
{
  String headerContent = FPSTR(html_common_header);
  String footerContent = FPSTR(html_common_footer);
  String toSend = headerContent + content + footerContent;
  toSend.replace(F("_UNIT_NAME_"), hostname);
  toSend.replace(F("_VERSION_"), m2mqtt_version);
  server.send(200, F("text/html"), toSend);
}

void handleNotFound()
{
  if (captive)
  {
    handleInitSetup();
  }
  else
  {
    server.sendHeader("Location", "/");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(302);
    return;
  }
}

void handleSaveWifi()
{
  if (!checkLogin())
    return;

  // Serial.println(F("Saving wifi config"));
  if (server.method() == HTTP_POST)
  {
    saveWifi(server.arg("ssid"), server.arg("psk"), server.arg("hn"), server.arg("otapwd"));
  }
  String initSavePage = FPSTR(html_init_save);
  initSavePage.replace("_TXT_INIT_REBOOT_MESS_", FPSTR(txt_init_reboot_mes));
  sendWrappedHTML(initSavePage);
  delay(500);
  ESP.restart();
}

void handleSaveInit()
{
  if (!checkLogin())
    return;

  // Serial.println(F("Saving wifi config"));
  if (server.method() == HTTP_POST)
  {
    saveWifi(server.arg("ssid"), server.arg("psk"), server.arg("hn"), server.arg("otapwd"));
    saveMqtt(server.arg("fn"), server.arg("mh"), server.arg("ml"), server.arg("mu"), server.arg("mp"), server.arg("mt"));
  }
  String initSavePage = FPSTR(html_init_save);
  initSavePage.replace("_TXT_INIT_REBOOT_MESS_", FPSTR(txt_init_reboot_mes));
  sendWrappedHTML(initSavePage);
  delay(500);
  ESP.restart();
}

void handleReboot()
{
  if (!checkLogin())
    return;

  String initRebootPage = FPSTR(html_init_reboot);
  initRebootPage.replace("_TXT_INIT_REBOOT_", FPSTR(txt_init_reboot));
  sendWrappedHTML(initRebootPage);
  delay(500);
  ESP.restart();
}

void handleRoot()
{
  if (!checkLogin())
    return;

  if (server.hasArg("REBOOT"))
  {
    String rebootPage = FPSTR(html_page_reboot);
    String countDown = FPSTR(count_down_script);
    rebootPage.replace("_TXT_M_REBOOT_", FPSTR(txt_m_reboot));
    sendWrappedHTML(rebootPage + countDown);
    delay(500);
#ifdef ESP32
    ESP.restart();
#else
    ESP.reset();
#endif
  }
  else
  {
    String menuRootPage = FPSTR(html_menu_root);
    menuRootPage.replace("_SHOW_LOGOUT_", (String)(login_password.length() > 0));
    // not show control button if hp not connected
    menuRootPage.replace("_SHOW_CONTROL_", (String)(hp.isConnected()));
    menuRootPage.replace("_TXT_CONTROL_", FPSTR(txt_control));
    menuRootPage.replace("_TXT_SETUP_", FPSTR(txt_setup));
    menuRootPage.replace("_TXT_STATUS_", FPSTR(txt_status));
    menuRootPage.replace("_TXT_LOGGING", FPSTR(txt_logging));
    menuRootPage.replace("_TXT_FW_UPGRADE_", FPSTR(txt_firmware_upgrade));
    menuRootPage.replace("_TXT_REBOOT_", FPSTR(txt_reboot));
    menuRootPage.replace("_TXT_LOGOUT_", FPSTR(txt_logout));
    sendWrappedHTML(menuRootPage);
  }
}

void handleInitSetup()
{
  String initSetupPage = FPSTR(html_init_setup);
  initSetupPage.replace("_TXT_INIT_TITLE_", FPSTR(txt_init_title));
  initSetupPage.replace("_TXT_INIT_HOST_", FPSTR(txt_wifi_hostname));
  initSetupPage.replace("_TXT_INIT_SSID_", FPSTR(txt_wifi_SSID));
  initSetupPage.replace("_TXT_INIT_PSK_", FPSTR(txt_wifi_psk));
  initSetupPage.replace("_TXT_INIT_OTA_", FPSTR(txt_wifi_otap));
  initSetupPage.replace("_TXT_SAVE_", FPSTR(txt_save));
  initSetupPage.replace("_TXT_REBOOT_", FPSTR(txt_reboot));

  initSetupPage.replace("_TXT_MQTT_TITLE_", FPSTR(txt_mqtt_title));
  initSetupPage.replace("_TXT_MQTT_FN_", FPSTR(txt_mqtt_fn));
  initSetupPage.replace("_TXT_MQTT_HOST_", FPSTR(txt_mqtt_host));
  initSetupPage.replace("_TXT_MQTT_PORT_", FPSTR(txt_mqtt_port));
  initSetupPage.replace("_TXT_MQTT_USER_", FPSTR(txt_mqtt_user));
  initSetupPage.replace("_TXT_MQTT_PASSWORD_", FPSTR(txt_mqtt_password));
  initSetupPage.replace("_TXT_MQTT_TOPIC_", FPSTR(txt_mqtt_topic));
  initSetupPage.replace(F("_MQTT_FN_"), hostname);
  initSetupPage.replace(F("_MQTT_HOST_"), "");
  initSetupPage.replace(F("_MQTT_PORT_"), "");
  initSetupPage.replace(F("_MQTT_USER_"), "");
  initSetupPage.replace(F("_MQTT_PASSWORD_"), "");
  initSetupPage.replace(F("_MQTT_TOPIC_"), mqtt_topic);

  sendWrappedHTML(initSetupPage);
}

void handleSetup()
{
  if (!checkLogin())
    return;

  if (server.hasArg("RESET"))
  {
    String pageReset = FPSTR(html_page_reset);
    String ssid = hostnamePrefix;
    ssid += getId();
    pageReset.replace("_TXT_M_RESET_", FPSTR(txt_m_reset));
    pageReset.replace("_SSID_", ssid);
    sendWrappedHTML(pageReset);
    SPIFFS.format();
    delay(500);
#ifdef ESP32
    ESP.restart();
#else
    ESP.reset();
#endif
  }
  else
  {
    String menuSetupPage = FPSTR(html_menu_setup);
    menuSetupPage.replace("_TXT_MQTT_", FPSTR(txt_MQTT));
    menuSetupPage.replace("_TXT_WIFI_", FPSTR(txt_WIFI));
    menuSetupPage.replace("_TXT_UNIT_", FPSTR(txt_unit));
    menuSetupPage.replace("_TXT_OTHERS_", FPSTR(txt_others));
    menuSetupPage.replace("_TXT_RESET_", FPSTR(txt_reset));
    menuSetupPage.replace("_TXT_BACK_", FPSTR(txt_back));
    menuSetupPage.replace("_TXT_RESETCONFIRM_", FPSTR(txt_reset_confirm));
    sendWrappedHTML(menuSetupPage);
  }
}

void rebootAndSendPage()
{
  String saveRebootPage = FPSTR(html_page_save_reboot);
  String countDown = FPSTR(count_down_script);
  saveRebootPage.replace("_TXT_M_SAVE_", FPSTR(txt_m_save));
  sendWrappedHTML(saveRebootPage + countDown);
  delay(500);
  ESP.restart();
}

void handleOthers()
{
  if (!checkLogin())
    return;

  if (server.method() == HTTP_POST)
  {
    saveOthers(server.arg("HAA"), server.arg("haat"), server.arg("AVAIL_REPORT"), server.arg("Debug"));
    rebootAndSendPage();
  }
  else
  {
    String othersPage = FPSTR(html_page_others);
    othersPage.replace("_TXT_SAVE_", FPSTR(txt_save));
    othersPage.replace("_TXT_BACK_", FPSTR(txt_back));
    othersPage.replace("_TXT_F_ON_", FPSTR(txt_f_on));
    othersPage.replace("_TXT_F_OFF_", FPSTR(txt_f_off));
    othersPage.replace("_TXT_OTHERS_TITLE_", FPSTR(txt_others_title));
    othersPage.replace("_TXT_OTHERS_HAAUTO_", FPSTR(txt_others_haauto));
    othersPage.replace("_TXT_OTHERS_HATOPIC_", FPSTR(txt_others_hatopic));
    othersPage.replace("_TXT_OTHERS_AVAILABILITY_REPORT_", FPSTR(txt_others_availability_report));
    othersPage.replace("_TXT_OTHERS_DEBUG_", FPSTR(txt_others_debug));

    othersPage.replace("_HAA_TOPIC_", others_haa_topic);
    if (others_haa)
    {
      othersPage.replace("_HAA_ON_", "selected");
    }
    else
    {
      othersPage.replace("_HAA_OFF_", "selected");
    }

    if (others_avail_report)
    {
      othersPage.replace("_HA_AVAIL_REPORT_ON_", "selected");
    }
    else
    {
      othersPage.replace("_HA_AVAIL_REPORT_OFF_", "selected");
    }

    if (_debugMode)
    {
      othersPage.replace("_DEBUG_ON_", "selected");
    }
    else
    {
      othersPage.replace("_DEBUG_OFF_", "selected");
    }
    sendWrappedHTML(othersPage);
  }
}

void handleMqtt()
{
  if (!checkLogin())
    return;

  if (server.method() == HTTP_POST)
  {
    saveMqtt(server.arg("fn"), server.arg("mh"), server.arg("ml"), server.arg("mu"), server.arg("mp"), server.arg("mt"));
    rebootAndSendPage();
  }
  else
  {
    String mqttPage = FPSTR(html_page_mqtt);
    mqttPage.replace("_TXT_SAVE_", FPSTR(txt_save));
    mqttPage.replace("_TXT_BACK_", FPSTR(txt_back));
    mqttPage.replace("_TXT_MQTT_TITLE_", FPSTR(txt_mqtt_title));
    mqttPage.replace("_TXT_MQTT_FN_", FPSTR(txt_mqtt_fn));
    mqttPage.replace("_TXT_MQTT_HOST_", FPSTR(txt_mqtt_host));
    mqttPage.replace("_TXT_MQTT_PORT_", FPSTR(txt_mqtt_port));
    mqttPage.replace("_TXT_MQTT_USER_", FPSTR(txt_mqtt_user));
    mqttPage.replace("_TXT_MQTT_PASSWORD_", FPSTR(txt_mqtt_password));
    mqttPage.replace("_TXT_MQTT_TOPIC_", FPSTR(txt_mqtt_topic));
    mqttPage.replace(F("_MQTT_FN_"), mqtt_fn);
    mqttPage.replace(F("_MQTT_HOST_"), mqtt_server);
    mqttPage.replace(F("_MQTT_PORT_"), String(mqtt_port));
    mqttPage.replace(F("_MQTT_USER_"), mqtt_username);
    mqttPage.replace(F("_MQTT_PASSWORD_"), mqtt_password);
    mqttPage.replace(F("_MQTT_TOPIC_"), mqtt_topic);
    sendWrappedHTML(mqttPage);
  }
}

void handleUnit()
{
  if (!checkLogin())
    return;

  if (server.method() == HTTP_POST)
  {
    saveUnit(server.arg("tu"), server.arg("md"), server.arg("update_int"), server.arg("lpw"), (String)convertLocalUnitToCelsius(server.arg("min_temp").toInt(), useFahrenheit), (String)convertLocalUnitToCelsius(server.arg("max_temp").toInt(), useFahrenheit), server.arg("temp_step"), server.arg("beep"), server.arg("led"));
    rebootAndSendPage();
  }
  else
  {
    String unitPage = FPSTR(html_page_unit);
    unitPage.replace("_TXT_SAVE_", FPSTR(txt_save));
    unitPage.replace("_TXT_BACK_", FPSTR(txt_back));
    unitPage.replace("_TXT_UNIT_TITLE_", FPSTR(txt_unit_title));
    unitPage.replace("_TXT_UNIT_TEMP_", FPSTR(txt_unit_temp));
    unitPage.replace("_TXT_UNIT_MINTEMP_", FPSTR(txt_unit_mintemp));
    unitPage.replace("_TXT_UNIT_MAXTEMP_", FPSTR(txt_unit_maxtemp));
    unitPage.replace("_TXT_UNIT_STEPTEMP_", FPSTR(txt_unit_steptemp));
    unitPage.replace("_TXT_UNIT_MODES_", FPSTR(txt_unit_modes));
    unitPage.replace("_TXT_UNIT_UPDATE_INTERVAL_", FPSTR(txt_unit_update_interval));
    unitPage.replace("_TXT_UNIT_PASSWORD_", FPSTR(txt_unit_password));
    unitPage.replace("_TXT_UNIT_BEEP_", FPSTR(txt_unit_beep));
    unitPage.replace("_TXT_UNIT_LED_", FPSTR(txt_unit_led));
    unitPage.replace("_TXT_F_CELSIUS_", FPSTR(txt_f_celsius));
    unitPage.replace("_TXT_F_FH_", FPSTR(txt_f_fh));
    unitPage.replace("_TXT_F_ALLMODES_", FPSTR(txt_f_allmodes));
    unitPage.replace("_TXT_F_NOHEAT_", FPSTR(txt_f_noheat));
    unitPage.replace("_TXT_F_5_S", FPSTR(txt_f_5s));
    unitPage.replace("_TXT_F_15_S", FPSTR(txt_f_15s));
    unitPage.replace("_TXT_F_30_S", FPSTR(txt_f_30s));
    unitPage.replace("_TXT_F_45_S", FPSTR(txt_f_45s));
    unitPage.replace("_TXT_F_60_S", FPSTR(txt_f_60s));
    unitPage.replace("_TXT_F_BEEP_ON_", FPSTR(txt_f_beep_on));
    unitPage.replace("_TXT_F_BEEP_OFF_", FPSTR(txt_f_beep_off));
    unitPage.replace("_TXT_F_LED_ON_", FPSTR(txt_f_led_on));
    unitPage.replace("_TXT_F_LED_OFF_", FPSTR(txt_f_led_off));
    unitPage.replace(F("_MIN_TEMP_"), String(convertCelsiusToLocalUnit(min_temp, useFahrenheit)));
    unitPage.replace(F("_MAX_TEMP_"), String(convertCelsiusToLocalUnit(max_temp, useFahrenheit)));
    unitPage.replace(F("_TEMP_STEP_"), String(temp_step));
    // temp
    if (useFahrenheit)
      unitPage.replace(F("_TU_FAH_"), F("selected"));
    else
      unitPage.replace(F("_TU_CEL_"), F("selected"));
    // mode
    if (supportHeatMode)
      unitPage.replace(F("_MD_ALL_"), F("selected"));
    else
      unitPage.replace(F("_MD_NONHEAT_"), F("selected"));
    unitPage.replace(F("_LOGIN_PASSWORD_"), login_password);

    // beep
    if (beep)
      unitPage.replace(F("_BEEP_ON_"), F("selected"));
    else
      unitPage.replace(F("_BEEP_OFF_"), F("selected"));

    // led
    if (ledEnabled)
      unitPage.replace(F("_LED_ON_"), F("selected"));
    else
      unitPage.replace(F("_LED_OFF_"), F("selected"));

    switch (update_int)
    {
    case (5000):
      unitPage.replace(F("_UPDATE_5S_"), F("selected"));
      break;
    case (15000):
      unitPage.replace(F("_UPDATE_15S_"), F("selected"));
      break;
    case (30000):
      unitPage.replace(F("_UPDATE_30S_"), F("selected"));
      break;
    case (45000):
      unitPage.replace(F("_UPDATE_45S_"), F("selected"));
      break;
    case (60000):
      unitPage.replace(F("_UPDATE_60S_"), F("selected"));
      break;
    }

    sendWrappedHTML(unitPage);
  }
}

void handleWifi()
{
  if (!checkLogin())
    return;

  if (server.method() == HTTP_POST)
  {
    saveWifi(server.arg("ssid"), server.arg("psk"), server.arg("hn"), server.arg("otapwd"));
    rebootAndSendPage();
#ifdef ESP32
    ESP.restart();
#else
    ESP.reset();
#endif
  }
  else
  {
    String wifiPage = FPSTR(html_page_wifi);
    String str_ap_ssid = ap_ssid;
    String str_ap_pwd = ap_pwd;
    String str_ota_pwd = ota_pwd;
    str_ap_ssid.replace("'", F("&apos;"));
    str_ap_pwd.replace("'", F("&apos;"));
    str_ota_pwd.replace("'", F("&apos;"));
    wifiPage.replace("_TXT_SAVE_", FPSTR(txt_save));
    wifiPage.replace("_TXT_BACK_", FPSTR(txt_back));
    wifiPage.replace("_TXT_WIFI_TITLE_", FPSTR(txt_wifi_title));
    wifiPage.replace("_TXT_WIFI_HOST_", FPSTR(txt_wifi_hostname));
    wifiPage.replace("_TXT_WIFI_SSID_", FPSTR(txt_wifi_SSID));
    wifiPage.replace("_TXT_WIFI_PSK_", FPSTR(txt_wifi_psk));
    wifiPage.replace("_TXT_WIFI_OTAP_", FPSTR(txt_wifi_otap));
    wifiPage.replace(F("_SSID_"), str_ap_ssid);
    wifiPage.replace(F("_PSK_"), str_ap_pwd);
    wifiPage.replace(F("_OTA_PWD_"), str_ota_pwd);
    sendWrappedHTML(wifiPage);
  }
}

void handleStatus()
{
  if (!checkLogin())
    return;

  String statusPage = FPSTR(html_page_status);
  statusPage.replace("_TXT_BACK_", FPSTR(txt_back));
  statusPage.replace("_TXT_STATUS_TITLE_", FPSTR(txt_status_title));
  statusPage.replace("_TXT_STATUS_HVAC_", FPSTR(txt_status_hvac));
  statusPage.replace("_TXT_STATUS_MQTT_", FPSTR(txt_status_mqtt));
  statusPage.replace("_TXT_STATUS_WIFI_", FPSTR(txt_status_wifi));
  statusPage.replace("_TXT_RETRIES_HVAC_", FPSTR(txt_retries_hvac));

  if (server.hasArg("mrconn"))
    mqttConnect();

  String connected = F("<span style='color:#47c266'><b>");
  connected += FPSTR(txt_status_connect);
  connected += F("</b><span>");

  String disconnected = F("<span style='color:#d43535'><b>");
  disconnected += FPSTR(txt_status_disconnect);
  disconnected += F("</b></span>");

  if (hp.isConnected())
    statusPage.replace(F("_HVAC_STATUS_"), connected);
  else
    statusPage.replace(F("_HVAC_STATUS_"), disconnected);
  if (mqtt_client.connected())
    statusPage.replace(F("_MQTT_STATUS_"), connected);
  else
    statusPage.replace(F("_MQTT_STATUS_"), disconnected);
  statusPage.replace(F("_HVAC_RETRIES_"), String(hpConnectionTotalRetries));
  statusPage.replace(F("_MQTT_REASON_"), String(mqtt_client.state()));
  statusPage.replace(F("_WIFI_STATUS_"), String(WiFi.RSSI()));
  sendWrappedHTML(statusPage);
}

void handleControl()
{
  if (!checkLogin())
    return;

  // not connected to hp, redirect to status page
  if (!hp.isConnected())
  {
    server.sendHeader("Location", "/status");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(302);
    return;
  }
  heatpumpSettings settings = hp.getSettings();
  settings = change_states(settings);
  String controlPage = FPSTR(html_page_control);
  String headerContent = FPSTR(html_common_header);
  String footerContent = FPSTR(html_common_footer);
  // write_log("Enter HVAC control");
  headerContent.replace("_UNIT_NAME_", hostname);
  footerContent.replace("_VERSION_", m2mqtt_version);
  controlPage.replace("_TXT_BACK_", FPSTR(txt_back));
  controlPage.replace("_UNIT_NAME_", hostname);
  controlPage.replace("_RATE_", "60");
  controlPage.replace("_ROOMTEMP_", String(convertCelsiusToLocalUnit(hp.getRoomTemperature(), useFahrenheit)));
  controlPage.replace("_USE_FAHRENHEIT_", (String)useFahrenheit);
  controlPage.replace("_TEMP_SCALE_", getTemperatureScale());
  controlPage.replace("_HEAT_MODE_SUPPORT_", (String)supportHeatMode);
  controlPage.replace(F("_MIN_TEMP_"), String(convertCelsiusToLocalUnit(min_temp, useFahrenheit)));
  controlPage.replace(F("_MAX_TEMP_"), String(convertCelsiusToLocalUnit(max_temp, useFahrenheit)));
  controlPage.replace(F("_TEMP_STEP_"), String(temp_step));
  controlPage.replace("_TXT_CTRL_CTEMP_", FPSTR(txt_ctrl_ctemp));
  controlPage.replace("_TXT_CTRL_TEMP_", FPSTR(txt_ctrl_temp));
  controlPage.replace("_TXT_CTRL_TITLE_", FPSTR(txt_ctrl_title));
  controlPage.replace("_TXT_CTRL_POWER_", FPSTR(txt_ctrl_power));
  controlPage.replace("_TXT_CTRL_MODE_", FPSTR(txt_ctrl_mode));
  controlPage.replace("_TXT_CTRL_FAN_", FPSTR(txt_ctrl_fan));
  controlPage.replace("_TXT_CTRL_VANE_", FPSTR(txt_ctrl_vane));
  controlPage.replace("_TXT_CTRL_WVANE_", FPSTR(txt_ctrl_wvane));
  controlPage.replace("_TXT_F_ON_", FPSTR(txt_f_on));
  controlPage.replace("_TXT_F_OFF_", FPSTR(txt_f_off));
  controlPage.replace("_TXT_F_AUTO_", FPSTR(txt_f_auto));
  controlPage.replace("_TXT_F_HEAT_", FPSTR(txt_f_heat));
  controlPage.replace("_TXT_F_DRY_", FPSTR(txt_f_dry));
  controlPage.replace("_TXT_F_COOL_", FPSTR(txt_f_cool));
  controlPage.replace("_TXT_F_FAN_", FPSTR(txt_f_fan));
  controlPage.replace("_TXT_F_QUIET_", FPSTR(txt_f_quiet));
  controlPage.replace("_TXT_F_SPEED_", FPSTR(txt_f_speed));
  controlPage.replace("_TXT_F_SWING_", FPSTR(txt_f_swing));
  controlPage.replace("_TXT_F_POS_", FPSTR(txt_f_pos));

  if (strcmp(settings.power, "ON") == 0)
  {
    controlPage.replace("_POWER_ON_", "selected");
  }
  else if (strcmp(settings.power, "OFF") == 0)
  {
    controlPage.replace("_POWER_OFF_", "selected");
  }

  if (strcmp(settings.mode, "HEAT") == 0)
  {
    controlPage.replace("_MODE_H_", "selected");
  }
  else if (strcmp(settings.mode, "DRY") == 0)
  {
    controlPage.replace("_MODE_D_", "selected");
  }
  else if (strcmp(settings.mode, "COOL") == 0)
  {
    controlPage.replace("_MODE_C_", "selected");
  }
  else if (strcmp(settings.mode, "FAN") == 0)
  {
    controlPage.replace("_MODE_F_", "selected");
  }
  else if (strcmp(settings.mode, "AUTO") == 0)
  {
    controlPage.replace("_MODE_A_", "selected");
  }

  if (strcmp(settings.fan, "AUTO") == 0)
  {
    controlPage.replace("_FAN_A_", "selected");
  }
  else if (strcmp(settings.fan, "QUIET") == 0)
  {
    controlPage.replace("_FAN_Q_", "selected");
  }
  else if (strcmp(settings.fan, "1") == 0)
  {
    controlPage.replace("_FAN_1_", "selected");
  }
  else if (strcmp(settings.fan, "2") == 0)
  {
    controlPage.replace("_FAN_2_", "selected");
  }
  else if (strcmp(settings.fan, "3") == 0)
  {
    controlPage.replace("_FAN_3_", "selected");
  }
  else if (strcmp(settings.fan, "4") == 0)
  {
    controlPage.replace("_FAN_4_", "selected");
  }

  controlPage.replace("_VANE_V_", settings.vane);
  if (strcmp(settings.vane, "AUTO") == 0)
  {
    controlPage.replace("_VANE_A_", "selected");
  }
  else if (strcmp(settings.vane, "1") == 0)
  {
    controlPage.replace("_VANE_1_", "selected");
  }
  else if (strcmp(settings.vane, "2") == 0)
  {
    controlPage.replace("_VANE_2_", "selected");
  }
  else if (strcmp(settings.vane, "3") == 0)
  {
    controlPage.replace("_VANE_3_", "selected");
  }
  else if (strcmp(settings.vane, "4") == 0)
  {
    controlPage.replace("_VANE_4_", "selected");
  }
  else if (strcmp(settings.vane, "5") == 0)
  {
    controlPage.replace("_VANE_5_", "selected");
  }
  else if (strcmp(settings.vane, "SWING") == 0)
  {
    controlPage.replace("_VANE_S_", "selected");
  }

  controlPage.replace("_WIDEVANE_V_", settings.wideVane);
  if (strcmp(settings.wideVane, "<<") == 0)
  {
    controlPage.replace("_WVANE_1_", "selected");
  }
  else if (strcmp(settings.wideVane, "<") == 0)
  {
    controlPage.replace("_WVANE_2_", "selected");
  }
  else if (strcmp(settings.wideVane, "|") == 0)
  {
    controlPage.replace("_WVANE_3_", "selected");
  }
  else if (strcmp(settings.wideVane, ">") == 0)
  {
    controlPage.replace("_WVANE_4_", "selected");
  }
  else if (strcmp(settings.wideVane, ">>") == 0)
  {
    controlPage.replace("_WVANE_5_", "selected");
  }
  else if (strcmp(settings.wideVane, "<>") == 0)
  {
    controlPage.replace("_WVANE_6_", "selected");
  }
  else if (strcmp(settings.wideVane, "SWING") == 0)
  {
    controlPage.replace("_WVANE_S_", "selected");
  }
  controlPage.replace("_TEMP_", String(convertCelsiusToLocalUnit(hp.getTemperature(), useFahrenheit)));

  // We need to send the page content in chunks to overcome
  // a limitation on the maximum size we can send at one
  // time (approx 6k).
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", headerContent);
  server.sendContent(controlPage);
  server.sendContent(footerContent);
  // Signal the end of the content
  server.sendContent("");
  // delay(100);
}

// login page, also called for logout
void handleLogin()
{
  bool loginSuccess = false;
  String msg;
  String loginPage = FPSTR(html_page_login);
  loginPage.replace("_TXT_LOGIN_TITLE_", FPSTR(txt_login_title));
  loginPage.replace("_TXT_LOGIN_PASSWORD_", FPSTR(txt_login_password));
  loginPage.replace("_TXT_LOGIN_", FPSTR(txt_login));

  if (server.hasArg("USERNAME") || server.hasArg("PASSWORD") || server.hasArg("LOGOUT"))
  {
    if (server.hasArg("LOGOUT"))
    {
      // logout
      server.sendHeader("Cache-Control", "no-cache");
      server.sendHeader("Set-Cookie", "M2MSESSIONID=0");
      loginSuccess = false;
    }
    if (server.hasArg("USERNAME") && server.hasArg("PASSWORD"))
    {
      if (server.arg("USERNAME") == "admin" && server.arg("PASSWORD") == login_password)
      {
        server.sendHeader("Cache-Control", "no-cache");
        server.sendHeader("Set-Cookie", "M2MSESSIONID=1");
        loginSuccess = true;
        msg = F("<span style='color:#47c266;font-weight:bold;'>");
        msg += FPSTR(txt_login_sucess);
        msg += F("<span>");
        loginPage += F("<script>");
        loginPage += F("setTimeout(function () {");
        loginPage += F("window.location.href= '/';");
        loginPage += F("}, 3000);");
        loginPage += F("</script>");
        // Log in Successful;
      }
      else
      {
        msg = F("<span style='color:#d43535;font-weight:bold;'>");
        msg += FPSTR(txt_login_fail);
        msg += F("</span>");
        // Log in Failed;
      }
    }
  }
  else
  {
    if (is_authenticated() or login_password.length() == 0)
    {
      server.sendHeader("Location", "/");
      server.sendHeader("Cache-Control", "no-cache");
      // use javascript in the case browser disable redirect
      String redirectPage = F("<html lang=\"en\" class=\"\"><head><meta charset='utf-8'>");
      redirectPage += F("<script>");
      redirectPage += F("setTimeout(function () {");
      redirectPage += F("window.location.href= '/';");
      redirectPage += F("}, 1000);");
      redirectPage += F("</script>");
      redirectPage += F("</body></html>");
      server.send(302, F("text/html"), redirectPage);
      return;
    }
  }
  loginPage.replace(F("_LOGIN_SUCCESS_"), (String)loginSuccess);
  loginPage.replace(F("_LOGIN_MSG_"), msg);
  sendWrappedHTML(loginPage);
}

void handleUpgrade()
{
  if (!checkLogin())
    return;

  uploaderror = 0;
  String upgradePage = FPSTR(html_page_upgrade);
  upgradePage.replace("_TXT_B_UPGRADE_", FPSTR(txt_upgrade));
  upgradePage.replace("_TXT_BACK_", FPSTR(txt_back));
  upgradePage.replace("_TXT_UPGRADE_TITLE_", FPSTR(txt_upgrade_title));
  upgradePage.replace("_TXT_UPGRADE_INFO_", FPSTR(txt_upgrade_info));
  upgradePage.replace("_TXT_UPGRADE_START_", FPSTR(txt_upgrade_start));

  sendWrappedHTML(upgradePage);
}

void handleUploadDone()
{
  Log.ln(TAG,"Upload done");
  bool restartflag = false;
  String uploadDonePage = FPSTR(html_page_upload);
  String content = F("<div style='text-align:center;'><b>Upload ");
  if (uploaderror)
  {
    content += F("<span style='color:#d43535'>failed</span></b><br/><br/>");
    if (uploaderror == 1)
    {
      content += FPSTR(txt_upload_nofile);
    }
    else if (uploaderror == 2)
    {
      content += FPSTR(txt_upload_filetoolarge);
    }
    else if (uploaderror == 3)
    {
      content += FPSTR(txt_upload_fileheader);
    }
    else if (uploaderror == 4)
    {
      content += FPSTR(txt_upload_flashsize);
    }
    else if (uploaderror == 5)
    {
      content += FPSTR(txt_upload_buffer);
    }
    else if (uploaderror == 6)
    {
      content += FPSTR(txt_upload_failed);
    }
    else if (uploaderror == 7)
    {
      content += FPSTR(txt_upload_aborted);
    }
    else
    {
      content += FPSTR(txt_upload_error);
      content += String(uploaderror);
    }
    if (Update.hasError())
    {
      content += FPSTR(txt_upload_code);
      content += String(Update.getError());
    }
  }
  else
  {
    content += F("<span style='color:#47c266; font-weight: bold;'>");
    content += FPSTR(txt_upload_sucess);
    content += F("</span><br/><br/>");
    content += FPSTR(txt_upload_refresh);
    content += F("<span id='count'>10s</span>...");
    content += FPSTR(count_down_script);
    restartflag = true;
  }
  content += F("</div><br/>");
  uploadDonePage.replace("_UPLOAD_MSG_", content);
  uploadDonePage.replace("_TXT_BACK_", FPSTR(txt_back));
  sendWrappedHTML(uploadDonePage);
  if (restartflag)
  {
    delay(500);
#ifdef ESP32
    ESP.restart();
#else
    ESP.reset();
#endif
  }
}

void handleUploadLoop()
{
  if (!checkLogin())
    return;

  // Log.ln(TAG, "Upload Loop");
  // Based on ESP8266HTTPUpdateServer.cpp uses ESP8266WebServer Parsing.cpp and Cores Updater.cpp (Update)
  // char log[200];
  digitalWrite(LED_ACT, (millis() % 1000 / 100 % 2));

  if (uploaderror)
  {
    Update.end();
    return;
  }
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    // Log.ln(TAG, "Upload Start");
    if (upload.filename.c_str()[0] == 0)
    {
      uploaderror = 1;
      return;
    }
    // save cpu by disconnect/stop retry mqtt server
    if (mqtt_client.state() == MQTT_CONNECTED)
    {
      mqtt_client.disconnect();
      lastMqttRetry = millis();
    }

    // Serial.printl(log);
    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (!Update.begin(maxSketchSpace))
    { // start with max available size
      // Log.ln(TAG, "Upload: Error, not enough storage");
      uploaderror = 2;
      return;
    }
  }
  else if (!uploaderror && (upload.status == UPLOAD_FILE_WRITE))
  {
    // Log.ln(TAG, "Upload Write");
    if (upload.totalSize == 0)
    {
      if (upload.buf[0] != 0xE9)
      {
        // Log.ln(TAG, "Upload: File magic header does not start with 0xE9" );
        uploaderror = 3;
        return;
      }
      uint32_t bin_flash_size = ESP.magicFlashChipSize((upload.buf[3] & 0xf0) >> 4);
#ifdef ESP32
      if (bin_flash_size > ESP.getFlashChipSize())
      {
#else
      if (bin_flash_size > ESP.getFlashChipRealSize())
      {
#endif
        // Log.ln(TAG, "Upload: File flash size is larger than device flash size" );
        uploaderror = 4;
        return;
      }
      if (ESP.getFlashChipMode() == 3)
      {
        upload.buf[2] = 3; // DOUT - ESP8285
      }
      else
      {
        upload.buf[2] = 2; // DIO - ESP8266
      }
    }
    // Log.ln(TAG, "Update Write");
    if (!uploaderror && (Update.write(upload.buf, upload.currentSize) != upload.currentSize))
    {
      // Update.printError(Serial);
      uploaderror = 5;
      return;
    }
  }
  else if (!uploaderror && (upload.status == UPLOAD_FILE_END))
  {
    // Log.ln(TAG, "Update END");
    if (Update.end(true))
    { // true to set the size to the current progress
      // Serial.printl(log)
    }
    else
    {
      // Update.printError(Serial);
      uploaderror = 6;
      return;
    }
  }
  else if (upload.status == UPLOAD_FILE_ABORTED)
  {
    // Log.ln(TAG, "Upload: Upload: Update was aborted");
    uploaderror = 7;
    Update.end();
  }

  #ifdef ESP32
    esp_task_wdt_reset();
  #else
    delay(0);
  #endif
}

void handleLogging()
{
  if (!checkLogin())
    return;

  String othersPage = FPSTR(html_page_logging);
  othersPage.replace("_TXT_LOGGING_TITLE_", FPSTR(txt_logging_title));
  othersPage.replace("_TXT_BACK_", FPSTR(txt_back));

  sendWrappedHTML(othersPage);
}

void handleAPILogs()
{
  if (!checkLogin())
    return;

  if (server.method() == HTTP_GET)
  {
    server.send(200, F("text/html"), Log.getLogs());
  }
}

void write_log(String log)
{
  File logFile = SPIFFS.open(console_file, "a");
  logFile.println(log);
  logFile.close();
}

heatpumpSettings change_states(heatpumpSettings settings)
{
  if (server.hasArg("CONNECT"))
  {
    hp.connect(acSerial);
  }
  else
  {
    bool update = false;
    if (server.hasArg("POWER"))
    {
      settings.power = strdup(server.arg("POWER").c_str());
      Log.ln(TAG, "Power = " + String(settings.power));
      update = true;
      previousCMDisPower = true;
    }
    if (server.hasArg("MODE"))
    {
      settings.mode = strdup(server.arg("MODE").c_str());
      Log.ln(TAG, "Mode = " + String(settings.mode));
      update = true;
    }
    if (server.hasArg("TEMP"))
    {
      settings.temperature = convertLocalUnitToCelsius(server.arg("TEMP").toInt(), useFahrenheit);
      Log.ln(TAG, "Temp = " + String(settings.temperature));
      update = true;
    }
    if (server.hasArg("FAN"))
    {
      settings.fan = strdup(server.arg("FAN").c_str());
      Log.ln(TAG, "Fan = " + String(settings.fan));
      update = true;
    }
    if (server.hasArg("VANE"))
    {
      settings.vane = strdup(server.arg("VANE").c_str());
      Log.ln(TAG, "Vane = " + String(settings.vane));
      update = true;
    }
    if (server.hasArg("WIDEVANE"))
    {
      settings.wideVane = strdup(server.arg("WIDEVANE").c_str());
      Log.ln(TAG, "WideVane = " + String(settings.wideVane));
      update = true;
    }
    if (update)
    {
      playBeep(SET);
      hp.setSettings(settings);
      lastCommandSend = millis();
    }
  }
  return settings;
}

void readHeatPumpSettings()
{
  heatpumpSettings currentSettings = hp.getSettings();

  rootInfo.clear();
  rootInfo["temperature"] = convertCelsiusToLocalUnit(currentSettings.temperature, useFahrenheit);
  rootInfo["fan"] = currentSettings.fan;
  rootInfo["vane"] = currentSettings.vane;
  rootInfo["wideVane"] = currentSettings.wideVane;
  rootInfo["mode"] = hpGetMode(currentSettings);
}

void hpSettingsChanged()
{
  // Log.ln(TAG, "hpSettingsChanged");
  // send room temp, operating info and all information


  // if ((millis() > (lastUpdate + update_int)) && (millis() > (lastCommandSend + POLL_DELAY_AFTER_SET_MS))) { // only send the temperature every update_int interval and not just sent command to A/C.
  if ((millis() - lastUpdate > update_int) && (millis()  - lastCommandSend >  ((previousCMDisPower) ? 30000 : POLL_DELAY_AFTER_SET_MS ))) { // only send the temperature every update_int interval and not just sent command to A/C.

    readHeatPumpSettings();

    String mqttOutput;
    serializeJson(rootInfo, mqttOutput);

    // if (!mqtt_client.publish(ha_settings_topic.c_str(), mqttOutput.c_str(), true)) {
    //   if (_debugMode) mqtt_client.publish(ha_debug_topic.c_str(), (char*)("Failed to publish hp settings"));
    // }

    hpStatusChanged(hp.getStatus());
    
  }
  
}

String hpGetMode(heatpumpSettings hpSettings)
{
  // Map the heat pump state to one of HA's HVAC_MODE_* values.
  // https://github.com/home-assistant/core/blob/master/homeassistant/components/climate/const.py#L3-L23

  String hppower = String(hpSettings.power);
  if (hppower.equalsIgnoreCase("off"))
  {
    return "off";
  }

  String hpmode = String(hpSettings.mode);
  hpmode.toLowerCase();

  if (hpmode == "fan")
    return "fan_only";
  else if (hpmode == "auto")
    return "heat_cool";
  else
    return hpmode; // cool, heat, dry
}

String hpGetAction(heatpumpStatus hpStatus, heatpumpSettings hpSettings)
{
  // Map heat pump state to one of HA's CURRENT_HVAC_* values.
  // https://github.com/home-assistant/core/blob/master/homeassistant/components/climate/const.py#L80-L86

  String hppower = String(hpSettings.power);
  if (hppower.equalsIgnoreCase("off"))
  {
    return "off";
  }

  String hpmode = String(hpSettings.mode);
  hpmode.toLowerCase();

  if (hpmode == "fan")
    return "fan";
  else if (!hpStatus.operating)
    return "idle";
  else if (hpmode == "auto")
    return "idle";
  else if (hpmode == "cool")
    return "cooling";
  else if (hpmode == "heat")
    return "heating";
  else if (hpmode == "dry")
    return "drying";
  else
    return hpmode; // unknown
}

void calculateEnergy(heatpumpStatus currentStatus)
{
  static unsigned long lastUpdate = millis();
  static float lastEnergySavedValue = 0;
  static unsigned long lastEnergySavedTime = 0;

  int currentPower = currentStatus.power;
  int secondSyncUpdate = (millis() - lastUpdate) / 1000;
  float sectionEnergy = currentPower * (secondSyncUpdate / 3600.0); // Wh
  energy += sectionEnergy / 1000;                                   // kWh

  if (energy - lastEnergySavedValue > ENERGY_SAVE_THRESHOLD && millis() - lastEnergySavedTime > (ENERGY_SAVE_INTERVAL * 60000))
  {
    Log.ln(TAG, "Save energy to SPIFFS");
    saveEnergy(energy);
    lastEnergySavedTime = millis();
    lastEnergySavedValue = energy;
  }

  lastUpdate = millis();
}

void hpStatusChanged(heatpumpStatus currentStatus)
{


  // if ((millis() > (lastTempSend + update_int)) && (millis() > (lastCommandSend + POLL_DELAY_AFTER_SET_MS))) { // only send the temperature every update_int interval and not just sent command to A/C.
  if ((millis() - lastUpdate > update_int) && (millis()  - lastCommandSend >  ((previousCMDisPower) ? 30000 : POLL_DELAY_AFTER_SET_MS ))) { // only send the temperature every update_int interval and not just sent command to A/C.

    // send room temp, operating info and all information
    heatpumpSettings currentSettings = hp.getSettings();

    calculateEnergy(currentStatus);

    if (currentStatus.roomTemperature == 0)
      return;

    rootInfo.clear();
    rootInfo["roomTemperature"] = convertCelsiusToLocalUnit(currentStatus.roomTemperature, useFahrenheit);
    rootInfo["temperature"] = convertCelsiusToLocalUnit(currentSettings.temperature, useFahrenheit);
    rootInfo["fan"] = currentSettings.fan;
    rootInfo["vane"] = currentSettings.vane;
    rootInfo["wideVane"] = currentSettings.wideVane;
    rootInfo["mode"] = hpGetMode(currentSettings);
    rootInfo["action"] = hpGetAction(currentStatus, currentSettings);
    rootInfo["compressorFrequency"] = currentStatus.compressorFrequency;
    rootInfo["power"] = currentStatus.power;
    rootInfo["energy"] = roundf(energy * 100) / 100;
    String mqttOutput;
    serializeJson(rootInfo, mqttOutput);

    if (!mqtt_client.publish_P(ha_state_topic.c_str(), mqttOutput.c_str(), false))
    {
      if (_debugMode)
        mqtt_client.publish(ha_debug_topic.c_str(), (char *)("Failed to publish hp status change"));
    }

    readHPstate(currentSettings, hp.getStatus());

    #ifdef ESP32
    Log.ln(TAG, "PSRAM size:\t" + String(ESP.getPsramSize()));
    Log.ln(TAG, "PSRAM Free:\t" + String(ESP.getFreePsram()));
    Log.ln(TAG, "Heap left:\t" + String(esp_get_free_heap_size()));
    Log.ln(TAG, "Free Stack Space:\t" + String(uxTaskGetStackHighWaterMark(NULL)));
    #endif

    //Update unit setting (Beep & LED to MQTT as well)
    updateUnitSettings(); 
    lastUpdate = millis();
  }
}

void updateUnitSettings(){
    String mqttOutput;
    StaticJsonDocument<32> doc;
    doc["led"] = ledEnabled?"ON":"OFF";
    doc["beep"] = beep?"ON":"OFF";
    serializeJson(doc,mqttOutput);

    if (!mqtt_client.publish_P(ha_unit_settings_topic.c_str(), mqttOutput.c_str(), false))
    {
      if (_debugMode)
        mqtt_client.publish(ha_debug_topic.c_str(), (char *)("Failed to publish hp status change"));
    }
}

void hpPacketDebug(byte *packet, unsigned int length, const char *packetDirection)
{
  if (_debugMode)
  {
    String message;
    for (unsigned int idx = 0; idx < length; idx++)
    {
      if (packet[idx] < 16)
      {
        message += "0"; // pad single hex digits with a 0
      }
      message += String(packet[idx], HEX) + " ";
    }

    const size_t bufferSize = JSON_OBJECT_SIZE(10);
    StaticJsonDocument<bufferSize> root;

    root[packetDirection] = message;
    String mqttOutput;
    serializeJson(root, mqttOutput);
    if (!mqtt_client.publish(ha_debug_topic.c_str(), mqttOutput.c_str()))
    {
      mqtt_client.publish(ha_debug_topic.c_str(), (char *)("Failed to publish to heatpump/debug topic"));
    }
  }
}

// Used to send a dummy packet in state topic to validate action in HA interface
void hpSendLocalState()
{

  // Send dummy MQTT state packet before unit update
  String mqttOutput;
  serializeJson(rootInfo, mqttOutput);
  if (!mqtt_client.publish_P(ha_state_topic.c_str(), mqttOutput.c_str(), false))
  {
    if (_debugMode)
      mqtt_client.publish(ha_debug_topic.c_str(), (char *)("Failed to publish dummy hp status change"));
  }

  // Restart counter for waiting enought time for the unit to update before sending a state packet
  lastUpdate = millis();
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  bool hvacControl = false;
  // Copy payload into message buffer
  char message[length + 1];
  for (unsigned int i = 0; i < length; i++)
  {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';

  // HA topics
  // Receive power topic
  if (strcmp(topic, ha_power_set_topic.c_str()) == 0)
  {
    String modeUpper = message;
    modeUpper.toUpperCase();
    if (modeUpper == "OFF")
    {
      playBeep(OFF);
      hp.setPowerSetting("OFF");
      hvacControl = true;
      previousCMDisPower = true;
    }
    else if (modeUpper == "ON")
    {
      playBeep(ON);
      hp.setPowerSetting("ON");
      hvacControl = true;
      previousCMDisPower = true;
    }
  }
  else if (strcmp(topic, ha_mode_set_topic.c_str()) == 0)
  {
    String modeUpper = message;
    modeUpper.toUpperCase();
    if (modeUpper == "OFF")
    {
      playBeep(OFF);
      rootInfo["mode"] = "off";
      rootInfo["action"] = "off";
      hpSendLocalState();
      hp.setPowerSetting("OFF");
      hvacControl = true;
    }
    else
    {
      playBeep(ON);
      if (modeUpper == "HEAT_COOL")
      {
        rootInfo["mode"] = "heat_cool";
        rootInfo["action"] = "idle";
        modeUpper = "AUTO";
      }
      else if (modeUpper == "HEAT")
      {
        rootInfo["mode"] = "heat";
        rootInfo["action"] = "heating";
      }
      else if (modeUpper == "COOL")
      {
        rootInfo["mode"] = "cool";
        rootInfo["action"] = "cooling";
      }
      else if (modeUpper == "DRY")
      {
        rootInfo["mode"] = "dry";
        rootInfo["action"] = "drying";
      }
      else if (modeUpper == "FAN_ONLY")
      {
        rootInfo["mode"] = "fan_only";
        rootInfo["action"] = "fan";
        modeUpper = "FAN";
      }
      else
      {
        return;
      }
      hpSendLocalState();
      hp.setPowerSetting("ON");
      hp.setModeSetting(modeUpper.c_str());
      hvacControl = true;
      previousCMDisPower = true;
    }
  }
  else if (strcmp(topic, ha_temp_set_topic.c_str()) == 0)
  {
    float temperature = strtof(message, NULL);
    float temperature_c = convertLocalUnitToCelsius(temperature, useFahrenheit);
    temperature_c = int(temperature_c * 10) / 10; //remove decimal point
    if (temperature_c < min_temp || temperature_c > max_temp)
    {
      temperature_c = 23;
      rootInfo["temperature"] = convertCelsiusToLocalUnit(temperature_c, useFahrenheit);
    }
    else
    {
      rootInfo["temperature"] = int(temperature * 10) / 10;  //remove decimal point
    }
    playBeep(SET);
    hpSendLocalState();
    hp.setTemperature(temperature_c);
    hvacControl = true;
  }
  else if (strcmp(topic, ha_fan_set_topic.c_str()) == 0)
  {
    rootInfo["fan"] = (String)message;
    playBeep(SET);
    hpSendLocalState();
    hp.setFanSpeed(message);
    hvacControl = true;
  }
  else if (strcmp(topic, ha_vane_set_topic.c_str()) == 0)
  {
    rootInfo["vane"] = (String)message;
    playBeep(SET);
    hpSendLocalState();
    hp.setVaneSetting(message);
    hvacControl = true;
  }
  else if (strcmp(topic, ha_wideVane_set_topic.c_str()) == 0)
  {
    rootInfo["wideVane"] = (String)message;
    playBeep(SET);
    hpSendLocalState();
    hp.setWideVaneSetting(message);
    hvacControl = true;
  }
  else if (strcmp(topic, ha_remote_temp_set_topic.c_str()) == 0)
  {
    float temperature = strtof(message, NULL);
    playBeep(SET);
    hp.setRemoteTemperature(convertLocalUnitToCelsius(temperature, useFahrenheit));
    hvacControl = true;
  }
  else if (strcmp(topic, ha_debug_set_topic.c_str()) == 0)
  { // if the incoming message is on the heatpump_debug_set_topic topic...
    if (strcmp(message, "on") == 0)
    {
      _debugMode = true;
      mqtt_client.publish(ha_debug_topic.c_str(), (char *)("Debug mode enabled"));
    }
    else if (strcmp(message, "off") == 0)
    {
      _debugMode = false;
      mqtt_client.publish(ha_debug_topic.c_str(), (char *)("Debug mode disabled"));
    }
  }
  else if (strcmp(topic, ha_custom_packet.c_str()) == 0)
  { // send custom packet for advance user
    String custom = message;

    // copy custom packet to char array
    char buffer[(custom.length() + 1)]; // +1 for the NULL at the end
    custom.toCharArray(buffer, (custom.length() + 1));

    byte bytes[20]; // max custom packet bytes is 20
    int byteCount = 0;
    char *nextByte;

    // loop over the byte string, breaking it up by spaces (or at the end of the line - \n)
    nextByte = strtok(buffer, " ");
    while (nextByte != NULL && byteCount < 20)
    {
      bytes[byteCount] = strtol(nextByte, NULL, 16); // convert from hex string
      nextByte = strtok(NULL, "   ");
      byteCount++;
    }

    // dump the packet so we can see what it is. handy because you can run the code without connecting the ESP to the heatpump, and test sending custom packets
    hpPacketDebug(bytes, byteCount, "customPacket");
    playBeep(SET);
    hp.sendCustomPacket(bytes, byteCount);
    hvacControl = true;
  }
  else if (strcmp(topic, ha_button_energy_set_topic.c_str()) == 0)
  {
    Log.ln(TAG, "Energy set");
    String messageStr = message;
    float newEnergyVal = messageStr.toFloat();
    Log.ln(TAG, "Set energy to " + String(newEnergyVal) + " kWh");
    energy = newEnergyVal;
    saveEnergy(energy);
    rootInfo["energy"] = energy;
    hpSendLocalState();
  }else if (strcmp(topic, ha_switch_unit_led_set_topic.c_str()) == 0){
    ledEnabled = strcmp(message,"ON") == 0;
    updateUnitSettings();
    saveUnitFeedback(beep,ledEnabled);
  }
  else if (strcmp(topic, ha_switch_unit_beep_set_topic.c_str()) == 0){
    beep = strcmp(message,"ON") == 0;
    updateUnitSettings();
    saveUnitFeedback(beep,ledEnabled);
  }
  else
  {
    mqtt_client.publish(ha_debug_topic.c_str(), strcat((char *)"heatpump: wrong mqtt topic: ", topic));
  }

  if (hvacControl){
    lastCommandSend = millis();
    hp.setInfoModeIndex(0);
  }

}

void addMQTTDeviceInfo(DynamicJsonDocument *JsonDocument)
{
  JsonObject haConfigDevice = JsonDocument->createNestedObject("device");

  haConfigDevice["ids"] = mqtt_fn;
  haConfigDevice["name"] = mqtt_fn;
  haConfigDevice["sw"] = "Mitsubishi2MQTT " + String(m2mqtt_version);
  haConfigDevice["mdl"] = "HVAC MITSUBISHI";
  haConfigDevice["mf"] = "MITSUBISHI ELECTRIC";
  haConfigDevice["hw"] = hardware_version;
  haConfigDevice["cu"] = "http://" + WiFi.localIP().toString();
}

void haConfig()
{

  // send HA config packet
  // setup HA payload device
  const size_t capacityClimateConfig = JSON_ARRAY_SIZE(7) + 2 * JSON_ARRAY_SIZE(6) + JSON_ARRAY_SIZE(7) + JSON_OBJECT_SIZE(30) + 2048;
  DynamicJsonDocument haClimateConfig(capacityClimateConfig);

  haClimateConfig["name"] = nullptr;
  haClimateConfig["unique_id"] = getId();
  haClimateConfig["icon"] = HA_AC_icon;

  JsonArray haConfigModes = haClimateConfig.createNestedArray("modes");
  haConfigModes.add("heat_cool"); // native AUTO mode
  haConfigModes.add("cool");
  haConfigModes.add("dry");
  if (supportHeatMode)
  {
    haConfigModes.add("heat");
  }
  haConfigModes.add("fan_only"); // native FAN mode
  haConfigModes.add("off");

  haClimateConfig["mode_cmd_t"] = ha_mode_set_topic;
  haClimateConfig["mode_stat_t"] = ha_state_topic;
  haClimateConfig["mode_stat_tpl"] = F("{{ value_json.mode if (value_json is defined and value_json.mode is defined and value_json.mode|length) else 'off' }}"); // Set default value for fix "Could not parse data for HA"
  haClimateConfig["temp_cmd_t"] = ha_temp_set_topic;
  haClimateConfig["temp_stat_t"] = ha_state_topic;

  if (others_avail_report)
  {
    haClimateConfig["avty_t"] = ha_availability_topic;          // MQTT last will (status) messages topic
    haClimateConfig["pl_not_avail"] = mqtt_payload_unavailable; // MQTT offline message payload
    haClimateConfig["pl_avail"] = mqtt_payload_available;       // MQTT online message payload
  }
  // Set default value for fix "Could not parse data for HA"
  String temp_stat_tpl_str = F("{% if (value_json is defined and value_json.temperature is defined) %}{% if (value_json.temperature|int > ");
  temp_stat_tpl_str += (String)convertCelsiusToLocalUnit(min_temp, useFahrenheit) + " and value_json.temperature|int < ";
  temp_stat_tpl_str += (String)convertCelsiusToLocalUnit(max_temp, useFahrenheit) + ") %}{{ value_json.temperature }}";
  temp_stat_tpl_str += "{% elif (value_json.temperature|int < " + (String)convertCelsiusToLocalUnit(min_temp, useFahrenheit) + ") %}" + (String)convertCelsiusToLocalUnit(min_temp, useFahrenheit) + "{% elif (value_json.temperature|int > " + (String)convertCelsiusToLocalUnit(max_temp, useFahrenheit) + ") %}" + (String)convertCelsiusToLocalUnit(max_temp, useFahrenheit) + "{% endif %}{% else %}" + (String)convertCelsiusToLocalUnit(22, useFahrenheit) + "{% endif %}";
  haClimateConfig["temp_stat_tpl"] = temp_stat_tpl_str;
  haClimateConfig["curr_temp_t"] = ha_state_topic;
  String curr_temp_tpl_str = F("{{ value_json.roomTemperature if (value_json is defined and value_json.roomTemperature is defined and value_json.roomTemperature|int > ");
  curr_temp_tpl_str += (String)convertCelsiusToLocalUnit(1, useFahrenheit) + ") else '" + (String)convertCelsiusToLocalUnit(26, useFahrenheit) + "' }}"; // Set default value for fix "Could not parse data for HA"
  haClimateConfig["curr_temp_tpl"] = curr_temp_tpl_str;
  haClimateConfig["min_temp"] = convertCelsiusToLocalUnit(min_temp, useFahrenheit);
  haClimateConfig["max_temp"] = convertCelsiusToLocalUnit(max_temp, useFahrenheit);
  haClimateConfig["temp_step"] = temp_step;
  haClimateConfig["pow_cmd_t"] = ha_power_set_topic;
  haClimateConfig["temperature_unit"] = useFahrenheit ? "F" : "C";
  String curr_pwr_tpl_str = "{{ value_json.power if (value_json is defined and value_json.power is defined and value_json.power|int >= 0) else '' }}";      // Set default value for fix "Could not parse data for HA"
  String cur_energy_tpl_str = "{{ value_json.energy if (value_json is defined and value_json.energy is defined and value_json.energy|int >= 0) else '' }}"; // Set default value for fix "Could not parse data for HA"

  JsonArray haConfigFan_modes = haClimateConfig.createNestedArray("fan_modes");
  haConfigFan_modes.add("AUTO");
  haConfigFan_modes.add("QUIET");
  haConfigFan_modes.add("1");
  haConfigFan_modes.add("2");
  haConfigFan_modes.add("3");
  haConfigFan_modes.add("4");

  haClimateConfig["fan_mode_cmd_t"] = ha_fan_set_topic;
  haClimateConfig["fan_mode_stat_t"] = ha_state_topic;
  haClimateConfig["fan_mode_stat_tpl"] = F("{{ value_json.fan if (value_json is defined and value_json.fan is defined and value_json.fan|length) else 'AUTO' }}"); // Set default value for fix "Could not parse data for HA"

  JsonArray haConfigSwing_modes = haClimateConfig.createNestedArray("swing_modes");
  haConfigSwing_modes.add("AUTO");
  haConfigSwing_modes.add("1");
  haConfigSwing_modes.add("2");
  haConfigSwing_modes.add("3");
  haConfigSwing_modes.add("4");
  haConfigSwing_modes.add("5");
  haConfigSwing_modes.add("SWING");

  haClimateConfig["swing_mode_cmd_t"] = ha_vane_set_topic;
  haClimateConfig["swing_mode_stat_t"] = ha_state_topic;
  haClimateConfig["swing_mode_stat_tpl"] = F("{{ value_json.vane if (value_json is defined and value_json.vane is defined and value_json.vane|length) else 'AUTO' }}"); // Set default value for fix "Could not parse data for HA"
  haClimateConfig["action_topic"] = ha_state_topic;
  haClimateConfig["action_template"] = F("{{ value_json.action if (value_json is defined and value_json.action is defined and value_json.action|length) else 'idle' }}"); // Set default value for fix "Could not parse data for HA"

  addMQTTDeviceInfo(&haClimateConfig);

  String mqttOutput;
  serializeJson(haClimateConfig, mqttOutput);
  mqtt_client.beginPublish(ha_climate_config_topic.c_str(), mqttOutput.length(), true);
  mqtt_client.print(mqttOutput);
  mqtt_client.endPublish();

  // Room Temperature config
  const size_t capacityRoomTempConfig = JSON_OBJECT_SIZE(7) + JSON_OBJECT_SIZE(8) + 2048;
  ;
  DynamicJsonDocument haRoomTempSensorConfig(capacityRoomTempConfig);
  haRoomTempSensorConfig["name"] = "Room temperature";
  haRoomTempSensorConfig["unique_id"] = getId() + "_room_temp";
  haRoomTempSensorConfig["icon"] = HA_thermometer_icon;
  haRoomTempSensorConfig["unit_of_measurement"] = useFahrenheit ? "°F" : "°C";
  haRoomTempSensorConfig["device_class"] = "Temperature";
  haRoomTempSensorConfig["state_topic"] = ha_state_topic;
  haRoomTempSensorConfig["value_template"] = curr_temp_tpl_str;

  addMQTTDeviceInfo(&haRoomTempSensorConfig);

  mqttOutput.clear();
  serializeJson(haRoomTempSensorConfig, mqttOutput);
  mqtt_client.beginPublish(ha_sensor_room_temp_config_topic.c_str(), mqttOutput.length(), true);
  mqtt_client.print(mqttOutput);
  mqtt_client.endPublish();

  // Power sensor config
  const size_t capacityPowerSensorConfig = JSON_OBJECT_SIZE(7) + JSON_OBJECT_SIZE(8) + 2048;
  ;
  DynamicJsonDocument haPowerSensorConfig(capacityPowerSensorConfig);
  haPowerSensorConfig["name"] = "Power";
  haPowerSensorConfig["unique_id"] = getId() + "_power";
  haPowerSensorConfig["icon"] = HA_lightning_bolt;
  haPowerSensorConfig["unit_of_measurement"] = "W";
  haPowerSensorConfig["device_class"] = "power";
  haPowerSensorConfig["state_topic"] = ha_state_topic;
  haPowerSensorConfig["value_template"] = curr_pwr_tpl_str;

  addMQTTDeviceInfo(&haPowerSensorConfig);

  mqttOutput.clear();
  serializeJson(haPowerSensorConfig, mqttOutput);
  mqtt_client.beginPublish(ha_sensor_power_config_topic.c_str(), mqttOutput.length(), true);
  mqtt_client.print(mqttOutput);
  mqtt_client.endPublish();

  // Energy sensor config
  const size_t capacityEnergySensorConfig = JSON_OBJECT_SIZE(9) + JSON_OBJECT_SIZE(8) + 2048;
  ;
  DynamicJsonDocument haEnergySensorConfig(capacityEnergySensorConfig);
  haEnergySensorConfig["name"] = "Energy";
  haEnergySensorConfig["unique_id"] = getId() + "_energy";
  haEnergySensorConfig["icon"] = HA_counter;
  haEnergySensorConfig["unit_of_measurement"] = "kWh";
  haEnergySensorConfig["device_class"] = "energy";
  haEnergySensorConfig["state_topic"] = ha_state_topic;
  haEnergySensorConfig["value_template"] = cur_energy_tpl_str;
  haEnergySensorConfig["state_class"] = "total_increasing";
  haEnergySensorConfig["suggested_display_precision"] = 1;

  addMQTTDeviceInfo(&haEnergySensorConfig);

  mqttOutput.clear();
  serializeJson(haEnergySensorConfig, mqttOutput);
  mqtt_client.beginPublish(ha_sensor_energy_config_topic.c_str(), mqttOutput.length(), true);
  mqtt_client.print(mqttOutput);
  mqtt_client.endPublish();

  // Energy reset button config
  const size_t capacityEnergyResetButtonConfig = JSON_OBJECT_SIZE(6) + JSON_OBJECT_SIZE(8) + 2048;
  ;
  DynamicJsonDocument haEnergyResetButtonConfig(capacityEnergyResetButtonConfig);
  haEnergyResetButtonConfig["name"] = "Energy Reset";
  haEnergyResetButtonConfig["unique_id"] = getId() + "_energy_reset";
  haEnergyResetButtonConfig["icon"] = HA_restart;
  haEnergyResetButtonConfig["command_topic"] = ha_button_energy_set_topic;
  haEnergyResetButtonConfig["entity_category"] = "config";
  haEnergyResetButtonConfig["payload_press"] = "0";

  addMQTTDeviceInfo(&haEnergyResetButtonConfig);

  mqttOutput.clear();
  serializeJson(haEnergyResetButtonConfig, mqttOutput);
  mqtt_client.beginPublish(ha_button_reset_energy_config_topic.c_str(), mqttOutput.length(), true);
  mqtt_client.print(mqttOutput);
  mqtt_client.endPublish();

  // Vane vertical config
  const size_t capacityVaneVerticalConfig = JSON_ARRAY_SIZE(8) + JSON_OBJECT_SIZE(7) + JSON_OBJECT_SIZE(8) + 2048;
  DynamicJsonDocument haVaneVerticalConfig(capacityVaneVerticalConfig);
  haVaneVerticalConfig["name"] = "Vane Vertical";
  haVaneVerticalConfig["unique_id"] = getId() + "_vane_vertical";
  haVaneVerticalConfig["icon"] = HA_vane_vertical_icon;
  haVaneVerticalConfig["state_topic"] = ha_state_topic;
  haVaneVerticalConfig["value_template"] = F("{{ value_json.vane if (value_json is defined and value_json.vane is defined and value_json.vane|length) else 'AUTO' }}"); // Set default value for fix "Could not parse data for HA"
  haVaneVerticalConfig["command_topic"] = ha_vane_set_topic;
  JsonArray haConfighVaneVerticalOptions = haVaneVerticalConfig.createNestedArray("options");
  haConfighVaneVerticalOptions.add("AUTO");
  haConfighVaneVerticalOptions.add("1");
  haConfighVaneVerticalOptions.add("2");
  haConfighVaneVerticalOptions.add("3");
  haConfighVaneVerticalOptions.add("4");
  haConfighVaneVerticalOptions.add("5");
  haConfighVaneVerticalOptions.add("SWING");

  addMQTTDeviceInfo(&haVaneVerticalConfig);

  mqttOutput.clear();
  serializeJson(haVaneVerticalConfig, mqttOutput);
  mqtt_client.beginPublish(ha_select_vane_vertical_config_topic.c_str(), mqttOutput.length(), true);
  mqtt_client.print(mqttOutput);
  mqtt_client.endPublish();

  // Vane horizontal config
  const size_t capacityVaneHorizontalConfig = JSON_ARRAY_SIZE(7) + JSON_OBJECT_SIZE(7) + JSON_OBJECT_SIZE(8) + 2048;
  DynamicJsonDocument haVaneHorizontalConfig(capacityVaneHorizontalConfig);
  haVaneHorizontalConfig["name"] = "Vane Horizontal";
  haVaneHorizontalConfig["unique_id"] = getId() + "_vane_horizontal";
  haVaneHorizontalConfig["icon"] = HA_vane_horizontal_icon;
  haVaneHorizontalConfig["state_topic"] = ha_state_topic;
  haVaneHorizontalConfig["value_template"] = F("{{ value_json.wideVane if (value_json is defined and value_json.wideVane is defined and value_json.wideVane|length) else 'SWING' }}"); // Set default value for fix "Could not parse data for HA"
  haVaneHorizontalConfig["command_topic"] = ha_wideVane_set_topic;
  JsonArray haConfigVaneHorizontalOptions = haVaneHorizontalConfig.createNestedArray("options");
  haConfigVaneHorizontalOptions.add("<<");
  haConfigVaneHorizontalOptions.add("<");
  haConfigVaneHorizontalOptions.add("|");
  haConfigVaneHorizontalOptions.add(">");
  haConfigVaneHorizontalOptions.add(">>");
  haConfigVaneHorizontalOptions.add("SWING");
  addMQTTDeviceInfo(&haVaneHorizontalConfig);

  mqttOutput.clear();
  serializeJson(haVaneHorizontalConfig, mqttOutput);
  mqtt_client.beginPublish(ha_select_vane_horizontal_config_topic.c_str(), mqttOutput.length(), true);
  mqtt_client.print(mqttOutput);
  mqtt_client.endPublish();

  #ifdef ESP32
  // LED Switch config
  const size_t capacityLEDSwitchConfig = JSON_OBJECT_SIZE(7) + JSON_OBJECT_SIZE(8) + 2048;
  DynamicJsonDocument haLEDSwitchConfig(capacityLEDSwitchConfig);
  haLEDSwitchConfig["name"] = "LED";
  haLEDSwitchConfig["unique_id"] = getId() + "_unit_led";
  haLEDSwitchConfig["icon"] = HA_led;
  haLEDSwitchConfig["command_topic"] = ha_switch_unit_led_set_topic;
  haLEDSwitchConfig["entity_category"] = "config";
  haLEDSwitchConfig["state_topic"] = ha_unit_settings_topic;
  haLEDSwitchConfig["value_template"] = F("{{ value_json.led if (value_json is defined and value_json.led is defined and value_json.led|length) else 'ON' }}"); 

  addMQTTDeviceInfo(&haLEDSwitchConfig);
  mqttOutput.clear();
  serializeJson(haLEDSwitchConfig, mqttOutput);
  mqtt_client.beginPublish(ha_switch_unit_led_config_topic.c_str(), mqttOutput.length(), true);
  mqtt_client.print(mqttOutput);
  mqtt_client.endPublish();

  // beep config
  const size_t capacityBeepSwitchConfig = JSON_OBJECT_SIZE(7) + JSON_OBJECT_SIZE(8) + 2048;
  DynamicJsonDocument haBeepSwitchConfig(capacityBeepSwitchConfig);
  haBeepSwitchConfig["name"] = "Beep";
  haBeepSwitchConfig["unique_id"] = getId() + "_unit_beep";
  haBeepSwitchConfig["icon"] = HA_beep;
  haBeepSwitchConfig["command_topic"] = ha_switch_unit_beep_set_topic;
  haBeepSwitchConfig["entity_category"] = "config";
  haBeepSwitchConfig["state_topic"] = ha_unit_settings_topic;
  haBeepSwitchConfig["value_template"] = F("{{ value_json.beep if (value_json is defined and value_json.beep is defined and value_json.beep|length) else 'ON' }}"); 

  addMQTTDeviceInfo(&haBeepSwitchConfig);
  mqttOutput.clear();
  serializeJson(haBeepSwitchConfig, mqttOutput);
  mqtt_client.beginPublish(ha_switch_unit_beep_config_topic.c_str(), mqttOutput.length(), true);
  mqtt_client.print(mqttOutput);
  mqtt_client.endPublish();

  #endif

}

void mqttConnect()
{
  // Loop until we're reconnected
  int attempts = 0;
  while (!mqtt_client.connected())
  {
    // Attempt to connect
    mqtt_client.connect(mqtt_client_id.c_str(), mqtt_username.c_str(), mqtt_password.c_str(), ha_availability_topic.c_str(), 1, true, mqtt_payload_unavailable);
    // If state < 0 (MQTT_CONNECTED) => network problem we retry 5 times and then waiting for MQTT_RETRY_INTERVAL_MS and retry reapeatly
    if (mqtt_client.state() < MQTT_CONNECTED)
    {
      if (attempts == 5)
      {
        lastMqttRetry = millis();
        return;
      }
      else
      {
        delay(10);
        attempts++;
      }
    }
    // If state > 0 (MQTT_CONNECTED) => config or server problem we stop retry
    else if (mqtt_client.state() > MQTT_CONNECTED)
    {
      return;
    }
    // We are connected
    else
    {
      mqtt_client.subscribe(ha_debug_set_topic.c_str());
      mqtt_client.subscribe(ha_power_set_topic.c_str());
      mqtt_client.subscribe(ha_mode_set_topic.c_str());
      mqtt_client.subscribe(ha_fan_set_topic.c_str());
      mqtt_client.subscribe(ha_temp_set_topic.c_str());
      mqtt_client.subscribe(ha_vane_set_topic.c_str());
      mqtt_client.subscribe(ha_wideVane_set_topic.c_str());
      mqtt_client.subscribe(ha_remote_temp_set_topic.c_str());
      mqtt_client.subscribe(ha_custom_packet.c_str());
      mqtt_client.subscribe(ha_button_energy_set_topic.c_str());
      mqtt_client.subscribe(ha_switch_unit_led_set_topic.c_str());
      mqtt_client.subscribe(ha_switch_unit_beep_set_topic.c_str());
      mqtt_client.publish(ha_availability_topic.c_str(), mqtt_payload_available, true); // publish status as available
      if (others_haa)
      {
        haConfig();
      }
      updateUnitSettings();
    }
  }
}

bool connectWifi()
{

  if (WiFi.getMode() != WIFI_STA)
  {
    WiFi.mode(WIFI_STA);
    delay(10);
  }
#ifdef ESP32
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
#endif

#ifdef ESP32
  WiFi.setHostname(hostname.c_str());
#else
  WiFi.hostname(hostname.c_str());
#endif

  WiFi.begin(ap_ssid.c_str(), ap_pwd.c_str());
  wifi_timeout = millis() + 30000;
  while (WiFi.status() != WL_CONNECTED && millis() < wifi_timeout)
  {
    Serial.write('.');
    // Serial.print(WiFi.status());
    //  wait 500ms, flashing the blue LED to indicate WiFi connecting...
    digitalWrite(LED_ACT, LED_ON);
    delay(250);
    digitalWrite(LED_ACT, LED_OFF);
    delay(250);
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    Log.ln(TAG, "Failed to connect to wifi");
    return false;
  }
  Log.ln(TAG, "Connected to " + ap_ssid);
  Log.ln(TAG, "Ready");
  while (WiFi.localIP().toString() == "0.0.0.0" || WiFi.localIP().toString() == "")
  {
    Serial.write('.');
    delay(500);
  }
  if (WiFi.localIP().toString() == "0.0.0.0" || WiFi.localIP().toString() == "")
  {
    Log.ln(TAG, "Failed to get IP Address");
    return false;
  }
  Log.ln(TAG, "IP address: " + WiFi.localIP().toString());
  // ticker.detach(); // Stop blinking the LED because now we are connected:)
  // keep LED off (For Wemos D1-Mini)
  digitalWrite(LED_ACT, LED_OFF);
  return true;
}

// temperature helper functions
float toFahrenheit(float fromCelcius)
{
  return round(1.8 * fromCelcius + 32.0);
}

float toCelsius(float fromFahrenheit)
{
  return (fromFahrenheit - 32.0) / 1.8;
}

float convertCelsiusToLocalUnit(float temperature, bool isFahrenheit)
{
  if (isFahrenheit)
  {
    return toFahrenheit(temperature);
  }
  else
  {
    return temperature;
  }
}

float convertLocalUnitToCelsius(float temperature, bool isFahrenheit)
{
  if (isFahrenheit)
  {
    return toCelsius(temperature);
  }
  else
  {
    return temperature;
  }
}

String getTemperatureScale()
{
  if (useFahrenheit)
  {
    return "F";
  }
  else
  {
    return "C";
  }
}

String getId()
{
#ifdef ESP32
  String lastMac = WiFi.macAddress();
  lastMac.remove(0, 9);
  lastMac.replace(":", "");
  return lastMac;
#else
  uint32_t chipID = ESP.getChipId();
  return String(chipID, HEX);
#endif
}

// Check if header is present and correct
bool is_authenticated()
{
  if (server.hasHeader("Cookie"))
  {
    // Found cookie;
    String cookie = server.header("Cookie");
    if (cookie.indexOf("M2MSESSIONID=1") != -1)
    {
      // Authentication Successful
      return true;
    }
  }
  // Authentication Failed
  return false;
}

bool checkLogin()
{
  if (!is_authenticated() and login_password.length() > 0)
  {
    server.sendHeader("Location", "/login");
    server.sendHeader("Cache-Control", "no-cache");
    // use javascript in the case browser disable redirect
    String redirectPage = F("<html lang=\"en\" class=\"\"><head><meta charset='utf-8'>");
    redirectPage += F("<script>");
    redirectPage += F("setTimeout(function () {");
    redirectPage += F("window.location.href= '/login';");
    redirectPage += F("}, 1000);");
    redirectPage += F("</script>");
    redirectPage += F("</body></html>");
    server.send(302, F("text/html"), redirectPage);
    return false;
  }
  return true;
}

#ifdef ESP32
void safeMode()
{
  Serial.begin(115200); // USB CDC
  delay(1000);

  digitalWrite(LED_ACT, LOW);

  Log.ln(TAG, "Safemode entered");

  for (int i = 0; i < 60; i++)
  {
    digitalWrite(LED_PWR, !digitalRead(LED_PWR));
    delay(1000);
  }
  ESP.restart();
}


void handleButton()
{
  if (millis() > 10000)
  {
    unsigned long pressedTime = millis() - BTNPresedTime;
    if (pressedTime > 15000 && btnPressed)
    {
      btnAction = longLongPress;
    }

    switch (btnAction)
    {

    case (shortPress):
      Log.ln(TAG, "Handle Short press");
      if (hp.isConnected())
      {
        digitalWrite(LED_ACT, LED_ON);
        if (hp.getPowerSetting() == "OFF")
        {
          hp.setPowerSetting("ON");
        }
        else
        {
          hp.setPowerSetting("OFF");
        }
        digitalWrite(LED_ACT, LED_OFF);
      }
      else
      {
        for (int i = 0; i < 4; i++)
        {
          digitalWrite(LED_ACT, LED_ON);
          delay(200);
          digitalWrite(LED_ACT, LED_OFF);
          delay(200);
        }
      }
      btnAction = noPress;
      break;

    case (longPress):
      Log.ln(TAG, "Handle Long press");
      ESP.restart();
      break;

    case (longLongPress):
      Log.ln(TAG, "Handle Long Long press");
      wifiFactoryReset();
      delay(1000);
      ESP.restart();
      break;
    }
  }
}

void IRAM_ATTR InterruptBTN()
{

  if (millis() < 10000)
  {
    return;
  }

  btnPressed = !btnPressed;

  if (btnPressed)
  {
    Log.ln(TAG, "Pressed");
    BTNPresedTime = millis();
  }
  else
  {
    Log.ln(TAG, "Released");
    unsigned long pressedTime = millis() - BTNPresedTime;
    // digitalWrite(LED_ACT,0);
    if (pressedTime > 50 && pressedTime < 500)
    {
      btnAction = shortPress;
    }
    else if (pressedTime > 5000 && pressedTime <= 15000)
    {
      btnAction = longPress;
    }
    else if (pressedTime > 15000)
    {
      btnAction = longLongPress;
    }
    else
    {
      btnAction = noPress;
    }
  }
}

#endif

void setup()
{

#ifdef ESP8266
  pinMode(LED_ACT, OUTPUT);
  digitalWrite(LED_ACT, LED_ON);
  checkMRD();
  delay(2000);
  mrd->stop();

  // Start serial for debug before HVAC connect to serial
  acSerial->begin(9600);
  // Serial.println(F("Starting Mitsubishi2MQTT"));
  // Mount SPIFFS filesystem

  delay(1000);

  Log.ln(TAG, "----Starting Mitsubishi2MQTT----");
  Log.ln(TAG, "FW Version:\t" + String(m2mqtt_version));
  Log.ln(TAG, "HW Version:\t" + String(hardware_version));
  Log.ln(TAG, "MAC Address:\t" + WiFi.macAddress());

  if (SPIFFS.begin())
  {
    // Serial.println(F("Mounted file system"));
  }
  else
  {
    SPIFFS.format();
  }

  // set test mode
  pinMode(0, INPUT_PULLUP);
  delay(1000);
  if (!digitalRead(0))
  {
    testMode();
  }
#endif

#ifdef ESP32
  pinMode(LED_ACT, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_ACT, LED_ON);
  digitalWrite(LED_PWR, LED_ON);
  ledcAttachPin(BUZZER, 0);
  ledcWriteTone(0, 0);

  attachInterrupt(BTN_1, InterruptBTN, CHANGE);
  pinMode(BTN_1, INPUT_PULLUP);
  delay(1000);
  if (!digitalRead(BTN_1))
  {
    wifiFactoryReset();
    delay(1000);
    ESP.restart();
  }
  testMode();

  // Start serial for debug before HVAC connect to serial
  acSerial->begin(9600);
  Serial.begin(115200);
  delay(1000);

  Log.ln(TAG, "----Starting Mitsubishi2MQTT----");
  Log.ln(TAG, "FW Version:\t" + String(m2mqtt_version));
  Log.ln(TAG, "HW Version:\t" + String(hardware_version));
  Log.ln(TAG, "ESP Chip Model:\t" + String(ESP.getChipModel()));
  Log.ln(TAG, "ESP PSRam Size:\t" + String(ESP.getPsramSize() / 1000) + " Kb");
  Log.ln(TAG, "MAC Address:\t" + WiFi.macAddress());

  if (esp_reset_reason() == ESP_RST_TASK_WDT)
  {
    esp_task_wdt_init(60, true);
    safeMode();
  }

  // Mount SPIFFS filesystem
  if (SPIFFS.begin())
  {
    Log.ln(TAG, "SPIFFS Mount OK");
  }
  else
  {
    Log.ln(TAG, "SPIFFS Mount Failed. Formatting...");
    if(SPIFFS.format()){
      Log.ln(TAG, "Formatting Completed");
      SPIFFS.begin();
    }else{
      Log.ln(TAG, "Format Failed. The system may not work properly!");
    }
  }
  #endif

  // Define hostname
  hostname += hostnamePrefix;
  hostname += getId();
  mqtt_client_id = hostname;
#ifdef ESP32
  WiFi.setHostname(hostname.c_str());
#else
  WiFi.hostname(hostname.c_str());
#endif

  setDefaults();
  wifi_config_exists = loadWifi();
  loadOthers();
  loadUnit();
  loadEnergy();
  if (initWifi())
  {
    if (SPIFFS.exists(console_file))
    {
      SPIFFS.remove(console_file);
    }
    // write_log("Starting Mitsubishi2MQTT");
    // Web interface
    server.on("/", handleRoot);
    server.on("/control", handleControl);
    server.on("/setup", handleSetup);
    server.on("/mqtt", handleMqtt);
    server.on("/wifi", handleWifi);
    server.on("/unit", handleUnit);
    server.on("/status", handleStatus);
    server.on("/others", handleOthers);
    server.on("/logging", handleLogging);
    server.on("/api/logs", handleAPILogs);
    server.onNotFound(handleNotFound);
    if (login_password.length() > 0)
    {
      server.on("/login", handleLogin);
      // here the list of headers to be recorded, use for authentication
      const char *headerkeys[] = {"User-Agent", "Cookie"};
      size_t headerkeyssize = sizeof(headerkeys) / sizeof(char *);
      // ask server to track these headers
      server.collectHeaders(headerkeys, headerkeyssize);
    }
    server.on("/upgrade", handleUpgrade);
    server.on("/upload", HTTP_POST, handleUploadDone, handleUploadLoop);

    server.begin();
    lastMqttRetry = 0;
    lastHpSync = 0;
    hpConnectionRetries = 0;
    hpConnectionTotalRetries = 0;
    if (loadMqtt())
    {
      Log.ln(TAG, "Starting MQTT");
      // setup HA topics
      ha_power_set_topic = mqtt_topic + "/" + mqtt_fn + "/power/set";
      ha_mode_set_topic = mqtt_topic + "/" + mqtt_fn + "/mode/set";
      ha_temp_set_topic = mqtt_topic + "/" + mqtt_fn + "/temp/set";
      ha_remote_temp_set_topic = mqtt_topic + "/" + mqtt_fn + "/remote_temp/set";
      ha_fan_set_topic = mqtt_topic + "/" + mqtt_fn + "/fan/set";
      ha_vane_set_topic = mqtt_topic + "/" + mqtt_fn + "/vane/set";
      ha_wideVane_set_topic = mqtt_topic + "/" + mqtt_fn + "/wideVane/set";
      ha_settings_topic = mqtt_topic + "/" + mqtt_fn + "/settings";
      ha_unit_settings_topic = mqtt_topic + "/" + mqtt_fn + "/unitSettings";
      ha_state_topic = mqtt_topic + "/" + mqtt_fn + "/state";
      ha_debug_topic = mqtt_topic + "/" + mqtt_fn + "/debug";
      ha_debug_set_topic = mqtt_topic + "/" + mqtt_fn + "/debug/set";
      ha_custom_packet = mqtt_topic + "/" + mqtt_fn + "/custom/send";
      ha_button_energy_set_topic = mqtt_topic + "/" + mqtt_fn + "/energy/set";
      ha_availability_topic = mqtt_topic + "/" + mqtt_fn + "/availability";
      ha_switch_unit_led_set_topic = mqtt_topic + "/" + mqtt_fn + "/led/set";
      ha_switch_unit_beep_set_topic = mqtt_topic + "/" + mqtt_fn + "/beep/set";

      if (others_haa)
      {
        ha_climate_config_topic = others_haa_topic + "/climate/" + mqtt_fn + "/config";
        ha_sensor_room_temp_config_topic = others_haa_topic + "/sensor/" + mqtt_fn + "/room_temp/config";
        ha_sensor_power_config_topic = others_haa_topic + "/sensor/" + mqtt_fn + "/power/config";
        ha_sensor_energy_config_topic = others_haa_topic + "/sensor/" + mqtt_fn + "/energy/config";
        ha_button_reset_energy_config_topic = others_haa_topic + "/button/" + mqtt_fn + "/energy_reset/config";
        ha_select_vane_vertical_config_topic = others_haa_topic + "/select/" + mqtt_fn + "/vane_vertical/config";
        ha_select_vane_horizontal_config_topic = others_haa_topic + "/select/" + mqtt_fn + "/vane_horizontal/config";
        ha_switch_unit_led_config_topic = others_haa_topic + "/switch/" + mqtt_fn + "/led/config";
        ha_switch_unit_beep_config_topic = others_haa_topic + "/switch/" + mqtt_fn + "/beep/config";
      }
      // startup mqtt connection
      initMqtt();
    }
    else
    {
      // write_log("Not found MQTT config go to configuration page");
    }

    hp.setSettingsChangedCallback(hpSettingsChanged);
    hp.setStatusChangedCallback(hpStatusChanged);
    hp.setPacketCallback(hpPacketDebug);
    // Allow Remote/Panel
    // hp.enableExternalUpdate();
    hp.disableAutoUpdate();
    if (hp.connect(acSerial))
    {
      Log.ln(TAG, "HVAC connected!");
    }
    else
    {
      Log.ln(TAG, "HVAC connection failed!");
    }
    heatpumpStatus currentStatus = hp.getStatus();
    heatpumpSettings currentSettings = hp.getSettings();
    rootInfo["roomTemperature"] = convertCelsiusToLocalUnit(currentStatus.roomTemperature, useFahrenheit);
    rootInfo["temperature"] = convertCelsiusToLocalUnit(currentSettings.temperature, useFahrenheit);
    rootInfo["fan"] = currentSettings.fan;
    rootInfo["vane"] = currentSettings.vane;
    rootInfo["wideVane"] = currentSettings.wideVane;
    rootInfo["mode"] = hpGetMode(currentSettings);
    rootInfo["action"] = hpGetAction(currentStatus, currentSettings);
    rootInfo["compressorFrequency"] = currentStatus.compressorFrequency;
    rootInfo["power"] = currentStatus.power;
    rootInfo["energy"] = roundf(energy * 100) / 100;

    lastUpdate = millis();
  }
  else
  {
    dnsServer.start(DNS_PORT, "*", apIP);
    initCaptivePortal();
  }
  initOTA();

#ifdef ESP32
  Log.ln(TAG, "---Setup completed---");

  digitalWrite(LED_ACT, LED_OFF);
  // Enable watchdog
  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);
#endif
}

void loop()
{
  bool mqttOK = false;
  server.handleClient();
  ArduinoOTA.handle();
#ifdef ESP32
  esp_task_wdt_reset();
#endif

  // reset board to attempt to connect to wifi again if in ap mode or wifi dropped out and time limit passed
  if (WiFi.getMode() == WIFI_STA and WiFi.status() == WL_CONNECTED)
  {
    wifi_timeout = millis() + WIFI_RETRY_INTERVAL_MS;
  }
  else if (wifi_config_exists and millis() > wifi_timeout)
  {
    ESP.restart();
  }

  if (!captive)
  {

    // Sync HVAC UNIT
    if (!hp.isConnected())
    {
      #ifdef ESP32
      digitalWrite(LED_PWR, millis() / 1000 %2);
      #endif
      // Use exponential backoff for retries, where each retry is double the length of the previous one.
      unsigned long timeNextSync = (1 << hpConnectionRetries) * HP_RETRY_INTERVAL_MS + lastHpSync;
      if (((millis() > timeNextSync) or lastHpSync == 0))
      {
        lastHpSync = millis();
        // If we've retried more than the max number of tries, keep retrying at that fixed interval, which is several minutes.
        hpConnectionRetries = min(hpConnectionRetries + 1u, HP_MAX_RETRIES);
        hpConnectionTotalRetries++;
        Log.ln(TAG, "HVAC is NOT connected, connecting...");
        hp.sync();
        if (hp.isConnected())
        {
          Log.ln(TAG, "HVAC connected!");
        }
        else
        {
          Log.ln(TAG, "HVAC connection failed!");
        }
      }
    }
    else
    {
      
      #ifdef ESP32
      digitalWrite(LED_PWR, ledEnabled? LED_ON: LED_OFF);
      #endif
      hpConnectionRetries = 0;

        // Log.ln(TAG,"Sync");
        hp.sync();
        delay(1000);
        // Log.ln(TAG,"Sync done");
        // currentSettings = ac.getSettings();
        // currentStatus = ac.getStatus();
        // if (hp.isConnected()){
        //   readHPstate(hp.getSettings(), hp.getStatus());
        // }
        // Log.ln(TAG, "PSRAM size:\t" + String(ESP.getPsramSize()));
        // Log.ln(TAG, "PSRAM Free:\t" + String(ESP.getFreePsram()));
        // Log.ln(TAG, "Heap left:\t" + String(esp_get_free_heap_size()));
        // Log.ln(TAG, "Free Stack Space:\t" + String(uxTaskGetStackHighWaterMark(NULL)));
    }

    if (mqtt_config)
    {
      // MQTT failed retry to connect
      if (mqtt_client.state() < MQTT_CONNECTED)
      {
        mqttOK = false;

        if ((millis() - lastMqttRetry > MQTT_RETRY_INTERVAL_MS) or lastMqttRetry == 0)
        {
          mqttConnect();
        }
      }
      // MQTT config problem on MQTT do nothing
      else if (mqtt_client.state() > MQTT_CONNECTED)
        return;
      // MQTT connected send status
      else
      {
        mqttOK = true;
        hpStatusChanged(hp.getStatus());
        mqtt_client.loop();
      }
    }
  }
  else
  {
    dnsServer.processNextRequest();
    digitalWrite(LED_ACT, LED_ON);
  }

  //Handle ACT LED
  if (hp.sendPending() || !mqttOK ){
    digitalWrite(LED_ACT,LED_ON);
  }else{
    digitalWrite(LED_ACT,LED_OFF);
  }

#ifdef ESP32
  handleButton();
#endif
}
