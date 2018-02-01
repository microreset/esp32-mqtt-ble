///////////////////////////////////////////////////////////////////////////
//  CONFIGURATION - SOFTWARE
///////////////////////////////////////////////////////////////////////////
#define NB_OF_BLE_TRACKED_DEVICES 1
BLETrackedDevice BLETrackedDevices[NB_OF_BLE_TRACKED_DEVICES] = {
  {"ff:ff:ff:ff:ff:ff", NULL, 0, 0, false, 0, false, false, {0}}
};

#define BLE_SCANNING_PERIOD   5
#define MAX_NON_ADV_PERIOD    10000
#define LOW_MEMORY_THRESHOLD  50000

// Location of the BLE scanner
#define LOCATION ""
#define TX_POWER 72
#define FILTER_FACTOR 60  //Between 0 and 100. 0 favors oldest values (slower/smoother), 100 favors the more recent ones (faster/more dynamic)

// Debug output
// #define DEBUG_SERIAL

// Wi-Fi credentials
#define WIFI_SSID     ""
#define WIFI_PASSWORD ""
// #define WIFI_KEEP_ON

// Syslog server connection info
// #define SYSLOG          // uncomment to activate syslog
#define SYSLOG_SERVER ""
#define SYSLOG_PORT 514
// This device info
#define DEVICE_HOSTNAME LOCATION
#define APP_NAME "blebeacon"

// Over-the-Air update
// Not implemented yet
//#define OTA
//#define OTA_HOSTNAME  ""    // hostname esp8266-[ChipID] by default
//#define OTA_PASSWORD  ""    // no password by default
//#define OTA_PORT      8266  // port 8266 by default

// MQTT
#define MQTT_USERNAME     ""
#define MQTT_PASSWORD     ""
#define MQTT_SERVER       ""
#define MQTT_SERVER_PORT  1883

#define MQTT_CONNECTION_TIMEOUT 5000 // [ms]

// MQTT availability: available/unavailable
#define MQTT_AVAILABILITY_TOPIC_TEMPLATE  "stat/%s/POWER"

// MQTT binary sensor: presence/<LOCATION>, example: presence/livingroom
#define MQTT_SENSOR_TOPIC_TEMPLATE        "presence/%s"

// Example: {"id":"e0:92:e5:69:30:ef","name":"vívoactive HR\u0000","rssi":-81,"distance":10.918119754009897}
#define MQTT_SENSOR_PAYLOAD_TEMPLATE     "{\"id\":\"%s\",\"rssi\":%s,\"distance\":%s}"

#define MQTT_PAYLOAD_AVAILABLE    "ON"
#define MQTT_PAYLOAD_UNAVAILABLE  "OFF"
