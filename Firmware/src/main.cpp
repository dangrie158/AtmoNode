#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8g2lib.h>
#include <Wire.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>

#include <WiFiManager.h>

#include <AdafruitIO_WiFi.h>

#include <LittleFS.h>
#include <Ticker.h>

#include <ArduinoJson.h>

#include "icons.h"

U8G2_SSD1306_64X32_1F_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

WiFiManager wifiManager;
WiFiClient client;
char adafruit_io_username[40] = "";
char adafruit_io_key[40] = "";
char room[40] = "";

AdafruitIO_WiFi *io;
AdafruitIO_Feed *humidity;
AdafruitIO_Feed *temperature;
AdafruitIO_Feed *pressure;

const static uint8_t resetButton = 0;  //GPIO 0
const static uint8_t portalButton = 2; //GPIO 2

//flag for saving data
bool shouldSaveConfig = false;

void delayWhileCheckingButtons(uint32_t time);
void saveConfigCallback();
void checkButtons();
void loadWLANConfig();
void saveWLANConfig();
void setupWLAN();
void displayPrintCenterln(const char *text, uint8_t y);
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

  io = new AdafruitIO_WiFi(adafruit_io_username, adafruit_io_key, "", "");

  Serial.println("Connecting to Adafruit IO");
  displayMessage(2000, connectIcon, "AdafruitIO", "connecting");
  io->connect();

  // wait for a connection
  aio_status_t lastStatus = AIO_IDLE, newStatus = AIO_IDLE;
  while (io->status() < AIO_CONNECTED)
  {
    aio_status_t newStatus = io->status();
    if (newStatus != lastStatus)
    {
      lastStatus = newStatus;
      auto statusText = io->statusText();
      Serial.println(io->statusText());
      displayMessage(0, connectIcon, String(statusText).c_str());
    }
  }

  String room_str = String(room);
  humidity = io->feed(String(room_str + ".Humidity").c_str());
  temperature = io->feed(String(room_str + ".Temperature").c_str());
  pressure = io->feed(String(room_str + ".Pressure").c_str());

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
  // io->run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io->adafruit.com, and processes any incoming data.
  io->run();

  Serial.print("Sensing for room ");
  Serial.println(room);
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Humidity = "));
  Serial.print(humidity_event.relative_humidity);
  Serial.println(" %");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");

  temperature->save(temp_event.temperature);
  humidity->save(humidity_event.relative_humidity);
  pressure->save(pressure_event.pressure);
  for (uint8_t i = 0; i < 6; i++)
  {
    displayMeasurement(3000, humidityIcon, String(humidity_event.relative_humidity, 1).c_str(), "Rh");
    delayWhileCheckingButtons(2000);
    displayMeasurement(3000, temperatureIcon, String(temp_event.temperature, 1).c_str(), "°C");
    delayWhileCheckingButtons(2000);
    displayMeasurement(3000, pressureIcon, String(pressure_event.pressure, 0).c_str(), "hPa");
    delayWhileCheckingButtons(2000);
  }
}

void delayWhileCheckingButtons(uint32_t time)
{
  uint32_t start = millis();
  while (millis() - start < time)
  {
    checkButtons();
    delay(10);
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

          strcpy(adafruit_io_username, doc["adafruit_io_username"]);
          strcpy(adafruit_io_key, doc["adafruit_io_key"]);
          strcpy(room, doc["room"]);
        }
        configFile.close();
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }
}

void saveWLANConfig()
{
  Serial.println("saving config");
  DynamicJsonDocument doc(512);
  doc["adafruit_io_username"] = adafruit_io_username;
  doc["adafruit_io_key"] = adafruit_io_key;
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
  WiFiManagerParameter adafruit_io_username_param("Adafruit IO Username", "username", adafruit_io_username, 32);
  WiFiManagerParameter adafruit_io_key_param("Adafruit IO Key", "key", adafruit_io_key, 32);
  WiFiManagerParameter room_param("Room", "room", room, 32);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&adafruit_io_username_param);
  wifiManager.addParameter(&adafruit_io_key_param);
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
  strcpy(adafruit_io_username, adafruit_io_username_param.getValue());
  strcpy(adafruit_io_key, adafruit_io_key_param.getValue());
  strcpy(room, room_param.getValue());

  Serial.println("connected...");
  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  displayMessage(2000, wlanIcon, "connected");
  displayMessage(2000, wlanIcon, WiFi.SSID().c_str());

  display.clearBuffer();
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
