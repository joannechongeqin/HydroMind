#include <Arduino.h>
//  CREDITS:
// This code is modified based on the code by:
// - Jonas Byström @ https://github.com/jonasbystrom/ESP-Now-Sensor-system-with-WiFi/blob/main/espnow_sensor/espnow_sensor.ino

// ------------------------------------------------------------------------------------------
// CONFIG VARIABLES THAT NEED TO BE EDITED
// ------------------------------------------------------------------------------------------
// ESP-NOW SYSTEM CONFIGS - Gateway_Mac
// ESP-NOW GATEWAY CONFIGS - SOFTAP_SSID, SOFTAP_PASS, UNITS
// SETUP FOR BLYNK - BLYNK_AUTH_TOKEN

// ------------------------------------------------------------------------------------------
// ESP-NOW SYSTEM CONFIGS
// ------------------------------------------------------------------------------------------
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiMulti.h>
// This is the MAC address to be installed (sensors shall then send to this MAC address)
uint8_t GatewayMac[] = {0x58, 0xCF, 0x79, 0xE4, 0x4E, 0x60};

// ESP-Now message format. Sensor data is transmitted using this struct.
typedef struct sensor_data_t
{           // Sensor data format for sending on ESP-Now to Gateway
  int unit; // Unit no to identy which sensor is sending
  float flowRate;
  int totalMilliLitres;
  unsigned long updated; // Epoch time when received by Gateway. Set by gateway/receiver. (Not used by sensor, but part of struct for convenience reasons.)
} sensor_data_t;

// ------------------------------------------------------------------------------------------
// ESP-NOW GATEWAY CONFIGS
// ------------------------------------------------------------------------------------------
// Router WiFi Credentials (runnning on 2.4GHz and Channel=1)
// ****if using hotspot (go to settings) -> configure -> advanced -> broadcast channel -> 1 **** //
#define SOFTAP_SSID "---- ENTER YOUR WIFI SSID ----"
#define SOFTAP_PASS "---- ENTER YOUR WIFI PASSWORD ----"

#define UNITS 1 // No of esp-now sensor units supported to receive from. ESP-Now has a maximum of 20

// ------------------------------------------------------------------------------------------
// SETUP FOR BLYNK
// ------------------------------------------------------------------------------------------
#include <BlynkSimpleEsp32.h>
#define BLYNK_TEMPLATE_ID "TMPL6l7fxKSS1"
#define BLYNK_TEMPLATE_NAME "HydroMind"
#define BLYNK_AUTH_TOKEN "1SPrH3zBG9UokY7ost1UdVTryrrIS5C_"
#define BLYNK_PRINT Serial
#define VPIN_totalFlow_ml V4

// ------------------------------------------------------------------------------------------
// GLOBALS
// ------------------------------------------------------------------------------------------
WiFiMulti wifiMulti;

sensor_data_t bufSensorData;         // buffer for incoming data
sensor_data_t sensorData[UNITS + 1]; // buffer for all sensor data

// ------------------------------------------------------------------------------------------
// ESP-NOW functions
// ------------------------------------------------------------------------------------------

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  char macStr[24];
  snprintf(macStr, sizeof(macStr), " %02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("\nData received from: ");
  Serial.println(macStr);
  memcpy(&bufSensorData, data, sizeof(bufSensorData));

  // Print data
  Serial.println("");
  Serial.print("Unit: ");
  Serial.print(bufSensorData.unit);
  Serial.print("   flowRate: ");
  Serial.print(bufSensorData.flowRate);
  Serial.print("   totalMilliLitres: ");
  Serial.print(bufSensorData.totalMilliLitres);
  Serial.println("");

  // Store data
  int i = bufSensorData.unit;
  if ((i >= 1) && (i <= UNITS))
  {
    memcpy(&sensorData[i], data, sizeof(bufSensorData));
  };
}

// ------------------------------------------------------------------------------------
void setup()
// ------------------------------------------------------------------------------------
{ // Init Serial
  Serial.begin(115200);

  // Connect to WiFi ------------------------------
  Serial.print("Connecting to WiFi ");

  // Set device in AP mode to begin with
  WiFi.mode(WIFI_AP_STA); // AP _and_ STA is required (!IMPORTANT)

  wifiMulti.addAP(SOFTAP_SSID, SOFTAP_PASS); // I use wifiMulti ... just by habit, i guess ....
  while (wifiMulti.run() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  // Come here - we are connected
  Serial.println(" Done");

  // Print WiFi data
  Serial.println("Set as AP_STA station.");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("Channel: ");
  Serial.println(WiFi.channel());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  delay(1000);

  // Initialize ESP-Now ---------------------------

  // Config gateway AP - set SSID and channel
  int channel = WiFi.channel();
  if (WiFi.softAP(SOFTAP_SSID, SOFTAP_PASS, channel, 1))
  {
    Serial.println("AP Config Success. AP SSID: " + String(SOFTAP_SSID));
  }
  else
  {
    Serial.println("AP Config failed.");
  }

  // Print MAC addresses
  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());

// Init ESP-Now
#define ESPNOW_SUCCESS ESP_OK

  if (esp_now_init() == ESPNOW_SUCCESS)
  {
    Serial.println("ESP - Now Init Success");
  }
  else
  {
    Serial.println("ESP - Now Init Failed");
    ESP.restart(); // just restart if we cant init ESP-Now
  }

  // ESP-Now is now initialized. Register a callback fcn for when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // Start BLYNK
  Blynk.begin(BLYNK_AUTH_TOKEN, SOFTAP_SSID, SOFTAP_PASS);

}

// ------------------------------------------------------------------------------------
void loop()
// ------------------------------------------------------------------------------------
{
  Blynk.run();
  double totalLitres = bufSensorData.totalMilliLitres / 1000.0;
  Blynk.virtualWrite(V4, totalLitres);
  Blynk.virtualWrite(V5, totalLitres);
}