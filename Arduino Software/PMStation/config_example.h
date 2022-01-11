// WiFi
const char* ssid = "ssid";
const char* password = "password";

// MQTT
const char* mqtt_server = "192.168.1.123";
const char* client_id = "PMStation1";
const unsigned long publish_interval = 30 * 1000L;
unsigned long last_publish_time = 0;

// Topics
const char* pm10_topic = "home/PMStation/PM1.0";
const char* pm25_topic = "home/PMStation/PM2.5";
const char* pm100_topic = "home/PMStation/PM10.0";
const char* temperature_topic = "home/PMStation/temperature";
const char* humidity_topic = "home/PMStation/humidity";
const char* pressure_topic = "home/PMStation/pressure";
const char* raw_topic = "home/PMStation/raw";
