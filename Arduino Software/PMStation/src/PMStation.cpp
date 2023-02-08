// board type: ESP32 Dev Module
// ESP32 arduino core == v2.0.0 (later versions do not work)
#include <Arduino.h>

#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <PMS.h>
#include <Wire.h>
#include <SparkFunHTU21D.h>
#include <Adafruit_BMP280.h>
#include <RunningAverage.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <WiFiManager.h>
#include <Preferences.h>

#include "config.h"

// Pins
#define LED_BUILTIN 23
#define FAN_PWM_PIN 25

// PM sensor UART pins
#define PMS1_RX 19
#define PMS1_TX 21
#define PMS2_RX 5
#define PMS2_TX 18
#define PMS3_RX 16
#define PMS3_TX 17

// I2C pins
#define I2C_1_SDA 13
#define I2C_1_SCL 14
#define I2C_2_SDA 26
#define I2C_2_SCL 27

// I2C adresses
#define htu21d_1_addr 0x40  // I2C 1
#define htu21d_2_addr 0x04  // I2C 2
#define bmp280_1_addr 0x76  // I2C 1

// Serial
#define PMS_BAUD 9600
SoftwareSerial pms1_ser(PMS1_RX, PMS1_TX);
SoftwareSerial pms2_ser(PMS2_RX, PMS2_TX);
SoftwareSerial pms3_ser(PMS3_RX, PMS3_TX);

WiFiClient wifi_client;
PubSubClient client(wifi_client);
bool wifi_available = false;
bool mqtt_available = false;

// WiFi manager
WiFiManager wm;
static WiFiManagerParameter mqtt_server_param("mqtt_server", "MQTT Server", "", 50);
static WiFiManagerParameter mqtt_port_param("mqtt_port", "MQTT Port", "", 6);
static WiFiManagerParameter mqtt_user_param("mqtt_user", "MQTT User", "", 50);
static WiFiManagerParameter mqtt_password_param("mqtt_password", "MQTT Password", "", 50);

Preferences prefs;
String mqtt_server = "";
int32_t mqtt_port = 0;
String mqtt_user = "";
String mqtt_password = "";

// PM sensors
PMS pms1(pms1_ser);
PMS::DATA pms1_data;

PMS pms2(pms2_ser);
PMS::DATA pms2_data;

PMS pms3(pms3_ser);
PMS::DATA pms3_data;

// I2C
TwoWire i2c_1(0);
TwoWire i2c_2(1);

// HTU21D sensors
HTU21D htu21d_1;
HTU21D htu21d_2;
const float temp_range_min = -100;
const float temp_range_max = 100;
const float humd_range_min = 0;
const float humd_range_max = 100;

// BMP280 sensors
Adafruit_BMP280 bmp280_1(&i2c_1);

// Data variables
const int nrSamples = 60;
RunningAverage pm10_1_avg(nrSamples);
RunningAverage pm10_2_avg(nrSamples);
RunningAverage pm10_3_avg(nrSamples);

RunningAverage pm25_1_avg(nrSamples);
RunningAverage pm25_2_avg(nrSamples);
RunningAverage pm25_3_avg(nrSamples);

RunningAverage pm100_1_avg(nrSamples);
RunningAverage pm100_2_avg(nrSamples);
RunningAverage pm100_3_avg(nrSamples);

RunningAverage temperature_1_avg(nrSamples);
RunningAverage temperature_2_avg(nrSamples);
RunningAverage temperature_x_avg(nrSamples);

RunningAverage humidity_1_avg(nrSamples);
RunningAverage humidity_2_avg(nrSamples);

RunningAverage pressure_avg(nrSamples);

float pm10_avg = 0;
float pm10_delta = 0;
float pm25_avg = 0;
float pm25_delta = 0;
float pm100_avg = 0;
float pm100_delta = 0;
float temperature_avg = 0;
float temperature_delta = 0;
float humidity_avg = 0;
float humidity_delta = 0;

// Char arrays
char pm10_avg_chr [20];
char pm25_avg_chr [20];
char pm100_avg_chr [20];
char temperature_avg_chr [20];
char humidity_avg_chr [20];
char pressure_avg_chr [20];

// Update sensor values interval
const unsigned long update_interval = 1 * 1000L;
unsigned long last_update_time = 0;
unsigned long last_publish_time = 0;

void save_prefs() {
  prefs.begin("mqtt");
  prefs.putString("mqtt_srv", mqtt_server);
  prefs.putUInt("mqtt_port", mqtt_port);
  prefs.putString("mqtt_user", mqtt_user);
  prefs.putString("mqtt_pass", mqtt_password);
  prefs.end();
}

void read_prefs() {
  prefs.begin("mqtt");
  mqtt_server = prefs.getString("mqtt_srv");
  mqtt_port = prefs.getUInt("mqtt_port");
  mqtt_user = prefs.getString("mqtt_user");
  mqtt_password = prefs.getString("mqtt_pass");
  prefs.end();
}

void param_callback() {
  mqtt_server = mqtt_server_param.getValue();
  mqtt_port = String(mqtt_port_param.getValue()).toInt();
  mqtt_user = mqtt_user_param.getValue();
  mqtt_password = mqtt_password_param.getValue();
  save_prefs();
}

void average_3_values(float val1, float val2, float val3, float &avg_out, float &max_err) {
  // exclude outlier and average remaining two
  float avg_all = (val1 + val2 + val3) / 3.0;
  float err1 = val1 - avg_all;
  float err2 = val2 - avg_all;
  float err3 = val3 - avg_all;

  if (err1 > err2 && err1 > err3) {
    avg_out = (val2 + val3) / 2.0;
    max_err = err1;
  }
  else if (err2 > err1 && err2 > err3) {
    avg_out = (val1 + val3) / 2.0;
    max_err = err2;
  }
  else {
    avg_out = (val1 + val2) / 2.0;
    max_err = err3;
  }
}

void average_2_values(float val1, float val2, float &avg_out, float &delta) {
  avg_out = (val1 + val2) / 2.0;
  delta = val1 - val2;
}


void print_all_data() {
  Serial.println("######## DATA ########"); Serial.println();
  Serial.println("[PM 1.0 (ug/m3)]");
  Serial.print("All sensors:"); Serial.print("\t");
  Serial.print(pm10_1_avg.getFastAverage(), 0); Serial.print("\t");
  Serial.print(pm10_2_avg.getFastAverage(), 0); Serial.print("\t");
  Serial.println(pm10_3_avg.getFastAverage(), 0);
  Serial.print("Average:"); Serial.print("\t");
  Serial.println(pm10_avg, 1);
  Serial.print("Delta:"); Serial.print("\t");
  Serial.println(pm10_delta, 1); Serial.println();

  Serial.println("[PM 2.5 (ug/m3)]");
  Serial.print("All sensors:"); Serial.print("\t");
  Serial.print(pm25_1_avg.getFastAverage(), 0); Serial.print("\t");
  Serial.print(pm25_2_avg.getFastAverage(), 0); Serial.print("\t");
  Serial.println(pm25_3_avg.getFastAverage(), 0);
  Serial.print("Average:"); Serial.print("\t");
  Serial.println(pm25_avg, 1);
  Serial.print("Delta:"); Serial.print("\t");
  Serial.println(pm25_delta, 1); Serial.println();

  Serial.println("[PM 10.0 (ug/m3)]");
  Serial.print("All sensors:"); Serial.print("\t");
  Serial.print(pm100_1_avg.getFastAverage(), 0); Serial.print("\t");
  Serial.print(pm100_2_avg.getFastAverage(), 0); Serial.print("\t");
  Serial.println(pm100_3_avg.getFastAverage(), 0);
  Serial.print("Average:"); Serial.print("\t");
  Serial.println(pm100_avg, 1);
  Serial.print("Delta:"); Serial.print("\t");
  Serial.println(pm100_delta, 1); Serial.println();

  Serial.println("[Temperature (*C)]");
  Serial.print("All sensors:"); Serial.print("\t");
  Serial.print(temperature_1_avg.getFastAverage(), 1); Serial.print("\t");
  Serial.println(temperature_2_avg.getFastAverage(), 1);
  Serial.print("Average: "); Serial.print("\t");
  Serial.println(temperature_avg, 1);
  Serial.print("Delta:"); Serial.print("\t");
  Serial.println(temperature_delta, 1); Serial.println();

  Serial.println("[Humidity (%)]");
  Serial.print("All sensors:"); Serial.print("\t");
  Serial.print(humidity_1_avg.getFastAverage(), 1); Serial.print("\t");
  Serial.println(humidity_2_avg.getFastAverage(), 1);
  Serial.print("Average:"); Serial.print("\t");
  Serial.println(humidity_avg, 1);
  Serial.print("Delta:"); Serial.print("\t");
  Serial.println(humidity_delta, 1); Serial.println();

  Serial.println("[Pressure (hPa)]");
  Serial.println(pressure_avg.getFastAverage(), 0);
  Serial.println();
}

void wifi_handler_loop() {
  static uint8_t ctrl = 0;
  static uint32_t reconnect_time = 0;
  static const uint32_t connect_timeout = 20000;
  switch (ctrl) {
    case 0: // check wifi status
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Reconnecting WiFi...");
        wifi_available = false;
        WiFi.disconnect(true);
        WiFi.begin();
        reconnect_time = millis();
        ctrl = 1;
      } else {
        wifi_available = true;
      }
      break;
    case 1: // wait for connection or timoeut
      if (WiFi.status() == WL_CONNECTED) {
        wifi_available = true;
        Serial.println("WiFi connected.");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        ctrl = 0;
      }
      else if (millis() - reconnect_time > connect_timeout) {
        ctrl = 0;
      }
      break;
  }
}

void mqtt_handler_loop() {
  static uint8_t ctrl = 0;
  static uint32_t reconnect_time = 20000;
  static const uint32_t connect_timeout = 20000;
  switch (ctrl) {
    case 0: // check wifi status
      if (wifi_available && !client.connected() && millis() - reconnect_time > connect_timeout) {
        Serial.println("Reconnecting MQTT...");
        mqtt_available = false;
        if (mqtt_user.length() == 0 && mqtt_password.length() == 0) {
          client.connect(client_id);
        } else {
          client.connect(client_id, mqtt_user.c_str(), mqtt_password.c_str());
        }
        reconnect_time = millis();
        ctrl = 1;
      }
      break;
    case 1: // wait for connection or timoeut
      if (client.connected()) {
        mqtt_available = true;
        Serial.println("MQTT connected.");
        ctrl = 0;
      }
      else if (millis() - reconnect_time > connect_timeout) {
        Serial.print("MQTT connection failed, rc=");
        Serial.println(client.state());
        ctrl = 0;
      }
      break;
  }
}

void setup() {
  esp_task_wdt_init(120, true);
  esp_task_wdt_add(NULL);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FAN_PWM_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(FAN_PWM_PIN, HIGH);

  read_prefs();
  
  Serial.begin(115200);
  Serial.println("###### PMStation Active ######");
  Serial.println();
  
  pms1_ser.begin(PMS_BAUD);
  pms2_ser.begin(PMS_BAUD);
  pms3_ser.begin(PMS_BAUD);

  i2c_1.setPins(I2C_1_SDA, I2C_1_SCL);
  i2c_2.setPins(I2C_2_SDA, I2C_2_SCL);

  htu21d_1.begin(i2c_1);
  htu21d_2.begin(i2c_2);

  bmp280_1.begin(bmp280_1_addr);
  /* Default settings from datasheet. */
  bmp280_1.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                       Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                       Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                       Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  mqtt_server_param.setValue(mqtt_server.c_str(), 50);
  mqtt_port_param.setValue(String(mqtt_port).c_str(), 6);
  mqtt_user_param.setValue(mqtt_user.c_str(), 50);
  mqtt_password_param.setValue(mqtt_password.c_str(), 50);
  wm.addParameter(&mqtt_server_param);
  wm.addParameter(&mqtt_port_param);
  wm.addParameter(&mqtt_user_param);
  wm.addParameter(&mqtt_password_param);
  wm.setTitle("PMS Station");
  wm.setSaveParamsCallback(param_callback);

  bool ret;
  ret = wm.autoConnect("PMS Station", "password");
  if (!ret) ESP.restart();
  
  wifi_client.setTimeout(1000);
  client.setSocketTimeout(2);
  client.setServer(mqtt_server.c_str(), mqtt_port);
  client.setBufferSize(1024);
}

void loop() {

  wifi_handler_loop();
  mqtt_handler_loop();
  client.loop();

  bool r1, r2, r3;
  r1 = pms1.read(pms1_data);
  r2 = pms2.read(pms2_data);
  r3 = pms3.read(pms3_data);
  
  if (millis() - last_update_time >= update_interval) {
    last_update_time = millis();
    esp_task_wdt_reset();

    // PM 1.0 um
    pm10_1_avg.addValue(pms1_data.PM_AE_UG_1_0);
    pm10_2_avg.addValue(pms2_data.PM_AE_UG_1_0);
    pm10_3_avg.addValue(pms3_data.PM_AE_UG_1_0);
    average_3_values(pm10_1_avg.getFastAverage(), pm10_2_avg.getFastAverage(), pm10_3_avg.getFastAverage(),
                      pm10_avg, pm10_delta);

    // PM 2.5 um
    pm25_1_avg.addValue(pms1_data.PM_AE_UG_2_5);
    pm25_2_avg.addValue(pms2_data.PM_AE_UG_2_5);
    pm25_3_avg.addValue(pms3_data.PM_AE_UG_2_5);
    average_3_values(pm25_1_avg.getFastAverage(), pm25_2_avg.getFastAverage(), pm25_3_avg.getFastAverage(),
                      pm25_avg, pm25_delta);

    // PM 10.0 um
    pm100_1_avg.addValue(pms1_data.PM_AE_UG_10_0);
    pm100_2_avg.addValue(pms2_data.PM_AE_UG_10_0);
    pm100_3_avg.addValue(pms3_data.PM_AE_UG_10_0);
    average_3_values(pm100_1_avg.getFastAverage(), pm100_2_avg.getFastAverage(), pm100_3_avg.getFastAverage(),
                      pm100_avg, pm100_delta);

    // Temperature
    float temp_1 = htu21d_1.readTemperature();
    float temp_2 = htu21d_2.readTemperature();
    if (temp_1 > temp_range_min && temp_1 < temp_range_max) {
      temperature_1_avg.addValue(temp_1);
    }
    if (temp_2 > temp_range_min && temp_2 < temp_range_max) {
      temperature_2_avg.addValue(temp_2);
    }
    average_2_values(temperature_1_avg.getFastAverage(), temperature_2_avg.getFastAverage(),
                      temperature_avg, temperature_delta);
    float temp_x = bmp280_1.readTemperature();
    temperature_x_avg.addValue(temp_x);
    
    // Humidity
    float humidity_1 = htu21d_1.readHumidity();
    float humidity_2 = htu21d_2.readHumidity();
    if (humidity_1 > humd_range_min && humidity_1 < humd_range_max) {
      humidity_1_avg.addValue(humidity_1);
    }
    if (humidity_2 > humd_range_min && humidity_2 < humd_range_max) {
      humidity_2_avg.addValue(humidity_2);
    }
    average_2_values(humidity_1_avg.getFastAverage(), humidity_2_avg.getFastAverage(),
                      humidity_avg, humidity_delta);

    // Pressure
    float pressure = bmp280_1.readPressure() / 100;
    pressure_avg.addValue(pressure);
  }

  if (millis() - last_publish_time >= publish_interval) {
    last_publish_time = millis();
    print_all_data();

    if (mqtt_available) {
      dtostrf(pm10_avg, 3, 0, pm10_avg_chr);
      dtostrf(pm25_avg, 3, 0, pm25_avg_chr);
      dtostrf(pm100_avg, 3, 0, pm100_avg_chr);
      client.publish(pm10_topic, pm10_avg_chr);
      client.publish(pm25_topic, pm25_avg_chr);
      client.publish(pm100_topic, pm100_avg_chr);
      
      dtostrf(temperature_avg, 5, 1, temperature_avg_chr);
      dtostrf(humidity_avg, 4, 1, humidity_avg_chr);
      dtostrf(pressure_avg.getFastAverage(), 6, 1, pressure_avg_chr);
      client.publish(temperature_topic, temperature_avg_chr);
      client.publish(humidity_topic, humidity_avg_chr);
      client.publish(pressure_topic, pressure_avg_chr);
  
      // Publish JSON
  
      StaticJsonDocument<1024> doc;
      doc["PM1.0_1"] = pm10_1_avg.getFastAverage();
      doc["PM1.0_2"] = pm10_2_avg.getFastAverage();
      doc["PM1.0_3"] = pm10_3_avg.getFastAverage();
      doc["PM1.0_avg"] = pm10_avg;
      doc["PM1.0_delta"] = pm10_delta;
      doc["PM2.5_1"] = pm25_1_avg.getFastAverage();
      doc["PM2.5_2"] = pm25_2_avg.getFastAverage();
      doc["PM2.5_3"] = pm25_3_avg.getFastAverage();
      doc["PM2.5_avg"] = pm25_avg;
      doc["PM2.5_delta"] = pm25_delta;
      doc["PM10.0_1"] = pm100_1_avg.getFastAverage();
      doc["PM10.0_2"] = pm100_2_avg.getFastAverage();
      doc["PM10.0_3"] = pm100_3_avg.getFastAverage();
      doc["PM10.0_avg"] = pm100_avg;
      doc["PM10.0_delta"] = pm100_delta;
      doc["temperature_1"] = temperature_1_avg.getFastAverage();
      doc["temperature_2"] = temperature_2_avg.getFastAverage();
      doc["temperature_x"] = temperature_x_avg.getFastAverage();
      doc["temperature_avg"] = temperature_avg;
      doc["temperature_delta"] = temperature_delta;
      doc["humidity_1"] = humidity_1_avg.getFastAverage();
      doc["humidity_2"] = humidity_2_avg.getFastAverage();
      doc["humidity_avg"] = humidity_avg;
      doc["humidity_delta"] = humidity_delta;
      doc["pressure"] = pressure_avg.getFastAverage();
  
      char buffer[1024];
      size_t n = serializeJson(doc, buffer);
      client.publish(raw_topic, (uint8_t*)buffer, n, false);
    }
    char buffer[40];
    sprintf(buffer, "MIN FREE HEAP: %d", ESP.getMinFreeHeap());
    Serial.println(buffer);
    if (ESP.getMinFreeHeap() < 20000UL) {
      ESP.restart();
    }
  }
  
}