#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8g2lib.h>
#include <Wire.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>

#include <WiFiManager.h>

#include <PubSubClient.h>

#include <LittleFS.h>
#include <Ticker.h>

#include <ArduinoJson.h>

#include "icons.h"

U8G2_SSD1306_64X32_1F_F_HW_I2C display(DISP_ROT, U8X8_PIN_NONE);
Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

WiFiManager wifiManager;
WiFiClient client;
char mqtt_server[40] = "grafana.local";
char room[40] = "";

PubSubClient mqtt(client);

const static uint8_t resetButton = 0;  //GPIO 0
const static uint8_t portalButton = 2; //GPIO 2

//flag for saving data
bool shouldSaveConfig = false;

void setupOTA();
void delayWhileCheckingButtons(uint32_t time);
void saveConfigCallback();
void checkButtons();
void loadWLANConfig();
void saveWLANConfig();
void setupWLAN();
void displayPrintCenterln(const char *text, uint8_t y);
void createInfluxMessage(char *dst, uint8_t len, const char *topic, float value, uint8_t precision);
void displayMessage(uint16_t duration, const uint8_t *icon, const char *message1, const char *message2 = "");
void displayMeasurement(uint16_t duration, const uint8_t *icon, const char *measurement, const char *unit);

void setup()
{
  pinMode(resetButton, INPUT);
  pinMode(portalButton, INPUT);

  // start the serial connection
  Serial.begin(115200);
  // wait for serial monitor to open
  while (!Serial)
    ;

  if (!display.begin())
  {
    Serial.println(F("SSD1306 allocation failed"));
    while (1)
      ;
  }
  display.clearBuffer();

  loadWLANConfig();
  setupWLAN();

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveWLANConfig();
  }

  setupOTA();

  mqtt.setServer(mqtt_server, 1883);

  if (!bme.begin(BME280_ADDRESS_ALTERNATE))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    displayMessage(5000, warningIcon, "BME280 ERR", "restarting");
    display.clearBuffer();
    ESP.reset();
    delay(5000);
  }
  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
}

void loop()
{

  if (WiFi.status() != WL_CONNECTED)
  {
    displayMessage(5000, connectIcon, "disconnected", "resetting");
    ESP.reset();
  }

  if (!mqtt.connected())
  {
    String clientId = String("AtmoNode-") + room;
    if (!mqtt.connect(clientId.c_str()))
    {
      displayMessage(5000, connectIcon, "MQTT failed", "retrying");
      return;
    }
  }

  Serial.print("Sensing for room ");
  Serial.println(room);
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  temp_event.temperature = temp_event.temperature - BME_WARMING_OFFSET;

  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Humidity = "));
  Serial.print(humidity_event.relative_humidity);
  Serial.println(" %");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");
  yield();

  // streaming sensor data
  String baseTopic = String("atmonode/") + room + "/";
  mqtt.publish(String(baseTopic + "temperature").c_str(), String(temp_event.temperature, 1).c_str());
  mqtt.publish(String(baseTopic + "humidity").c_str(), String(humidity_event.relative_humidity, 1).c_str());
  mqtt.publish(String(baseTopic + "pressure").c_str(), String(pressure_event.pressure, 0).c_str());
  yield();

  // messages for storing the data in influxdb
  const char *persistentTopic = "atmonode";
  char messageBuffer[50] = {0};
  createInfluxMessage(messageBuffer, 50, "temperature", temp_event.temperature, 1);
  mqtt.publish(persistentTopic, messageBuffer);

  createInfluxMessage(messageBuffer, 50, "humidity", humidity_event.relative_humidity, 1);
  mqtt.publish(persistentTopic, messageBuffer);

  createInfluxMessage(messageBuffer, 50, "pressure", pressure_event.pressure, 0);
  mqtt.publish(persistentTopic, messageBuffer);

  yield();
  displayMeasurement(3000, humidityIcon, String(humidity_event.relative_humidity, 1).c_str(), "Rh");
  delayWhileCheckingButtons(2000);

  yield();
  displayMeasurement(3000, temperatureIcon, String(temp_event.temperature, 1).c_str(), "°C");
  delayWhileCheckingButtons(2000);

  yield();
  displayMeasurement(3000, pressureIcon, String(pressure_event.pressure, 0).c_str(), "hPa");
  delayWhileCheckingButtons(2000);
}

void setupOTA()
{
  ArduinoOTA.onStart([]() {
    displayMessage(1, warningIcon, "update in", "progress");
  });
  ArduinoOTA.onEnd([]() {
    displayMessage(1000, warningIcon, "done", "restarting");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));

    displayMessage(0, warningIcon, "Progress", String(String(progress / (total / 100), 10) + "%").c_str());
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void delayWhileCheckingButtons(uint32_t time)
{
  uint32_t start = millis();
  while (millis() - start < time)
  {
    // Handle OTA update server
    ArduinoOTA.handle();

    checkButtons();
    delay(5);
  }
}

void checkButtons()
{
  if (!digitalRead(resetButton))
  {
    ESP.reset();
  }

  if (!digitalRead(portalButton))
  {
    wifiManager.resetSettings();
    ESP.reset();
  }
}

//callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void loadWLANConfig()
{
  if (LittleFS.begin())
  {
    Serial.println("mounted file system");
    if (LittleFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(size);
        auto error = deserializeJson(doc, buf.get());
        serializeJson(doc, Serial);
        if (error)
        {
          Serial.println("failed to load json config");
          Serial.println(error.c_str());
        }
        else
        {
          Serial.println("\nparsed json");
          if (doc.containsKey("mqtt_server"))
          {
            strcpy(mqtt_server, doc["mqtt_server"]);
          }
          strcpy(room, doc["room"]);
        }
        configFile.close();
      }
    }
    else
    {
      wifiManager.resetSettings();
    }
  }
  else
  {
    Serial.println("failed to mount FS");
    wifiManager.resetSettings();
  }
}

void saveWLANConfig()
{
  Serial.println("saving config");
  DynamicJsonDocument doc(512);
  doc["mqtt_server"] = mqtt_server;
  doc["room"] = room;

  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile)
  {
    Serial.println("failed to open config file for writing");
  }

  serializeJson(doc, Serial);
  serializeJson(doc, configFile);
  configFile.close();
}

void setupWLAN()
{
  // auto-manage wifi configuration
  WiFiManagerParameter mqtt_server_param("MQTT Server", "IP or hostname", mqtt_server, 32);
  WiFiManagerParameter room_param("Room", "room", room, 32);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&mqtt_server_param);
  wifiManager.addParameter(&room_param);

  // create a unique SSID
  String apSSID = String("AtmoNode") + String(random(10, 100), DEC);
  Serial.print("Random accesspoint SSID is: ");
  Serial.println(apSSID);

  // create a random numeric 4-digit password
  String apPasscode = String(random(1000, 10000), DEC);
  apPasscode += String(random(1000, 10000), DEC);
  Serial.print("Random accesspoint password is: ");
  Serial.println(apPasscode);

  if (!MDNS.begin("atmo"))
  {
    Serial.println("Error setting up mDNS responder!");
    displayMessage(2000, wlanIcon, "mDNS fail");
  }

  displayMessage(5000, wlanIcon, apSSID.c_str(), apPasscode.c_str());
  wifiManager.setTimeout(60);
  bool success = wifiManager.autoConnect(apSSID.c_str(), apPasscode.c_str());

  if (!success)
  {
    Serial.println("failed to connect and hit timeout");

    displayMessage(5000, wlanIcon, "WiFi failed", "restarting");
    display.clearBuffer();
    ESP.reset();
    delay(5000);
  }

  //read updated parameters
  strcpy(mqtt_server, mqtt_server_param.getValue());
  strcpy(room, room_param.getValue());

  Serial.println("connected...");
  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  displayMessage(2000, wlanIcon, "connected");
  displayMessage(2000, wlanIcon, WiFi.SSID().c_str());

  display.clearBuffer();
}

void createInfluxMessage(char *dst, uint8_t len, const char *topic, float value, uint8_t precision)
{
  uint8_t pos = 0;
  strncpy(dst + pos, topic, len - pos);
  pos += strlen(topic);
  strncpy(dst + pos, ",site=", len - pos);
  pos += 6;
  strncpy(dst + pos, room, len - pos);
  pos += strlen(room);
  strncpy(dst + pos, " value=", len - pos);
  pos += 7;
  String val_str = String(value, precision);
  strncpy(dst + pos, val_str.c_str(), len - pos);
}

void displayMessage(uint16_t duration, const uint8_t *icon, const char *message1, const char *message2)
{
  display.setFont(u8g2_font_t0_11_tf);
  display.clearBuffer();
  display.drawXBMP(
      (display.getDisplayWidth() - iconWidth) / 2, 0,
      iconWidth, iconHeight, icon);
  displayPrintCenterln(message1, iconHeight + 5);
  displayPrintCenterln(message2, iconHeight + 13);
  display.sendBuffer();
  delay(duration);
}

void displayMeasurement(uint16_t duration, const uint8_t *icon, const char *measurement, const char *unit)
{
  display.setFont(u8g2_font_t0_11_tf);
  display.clearBuffer();
  display.drawXBMP(
      ((display.getDisplayWidth() / 2) - iconWidth) / 2, 0,
      iconWidth, iconHeight, icon);
  display.drawStr((display.getDisplayWidth() / 2), 16 - 5, unit);

  display.setFont(u8g2_font_10x20_tn);
  displayPrintCenterln(measurement, iconHeight + 16);
  display.sendBuffer();
  delay(duration);
}

void displayPrintCenterln(const char *text, uint8_t y)
{
  uint8_t width = display.getStrWidth(text);
  display.drawStr((display.getDisplayWidth() - width) / 2, y, text);
}
