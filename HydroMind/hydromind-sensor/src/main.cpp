#include <Arduino.h>
//  CREDITS:
//  This code is modified based on the code by:
// - Jonas Byström @ https://github.com/jonasbystrom/ESP-Now-Sensor-system-with-WiFi/blob/main/espnow_sensor/espnow_sensor.ino
// - IdrisCytron @ https://gist.github.com/IdrisCytron/1b23265ede09577c19ef6acc27ee848d

// ------------------------------------------------------------------------------------------
// CONFIG VARIABLES THAT NEED TO BE EDITED
// ------------------------------------------------------------------------------------------
// ESP-NOW SYSTEM CONFIGS - Gateway_Mac
// ESP SENSOR CONFIGS - UNIT

// ------------------------------------------------------------------------------------------
// ESP-NOW SYSTEM CONFIGS
// ------------------------------------------------------------------------------------------
#include <WiFi.h>
#include <esp_now.h>
#define WIFI_CHANNEL 1 // Must be 1. (!IMPORTANT)
                       // ESP-Now can work on other channels, but the receiving ESP Gateway must operate on
                       // channel 1 in order to also work for TCP/IP WiFi.
                       // It has been reported to work also for other Channels, but I have only succeeded on ch 1.

uint8_t Gateway_Mac[] = {0x58, 0xCF, 0x79, 0xE4, 0x4E, 0x60};
// MAC Address of the remote ESP Gateway we send to.
// This is the "system address" all Sensors send to and the Gateway listens on.
// (The ESP Gateway will set this "soft" MAC address in its HW. See Gateway code for info.)

typedef struct sensor_data_t
{                           // Sensor data format for sending on ESP-Now to Gateway
  int unit;                 // Unit no to identy which sensor is sending
  float flowRate;
  int totalMilliLitres;
  unsigned long updated;    // Epoch time when received by Gateway. Set by gateway/receiver. (Not used by sensor, but part of struct for convenience reasons.)
  int test_num;
} sensor_data_t;

// -----------------------------------------------------------------------------------------
// ESP SENSOR CONFIGS
// -----------------------------------------------------------------------------------------
#define UNIT 1                // Sensor unit ID to identify THIS unit for the receiving gateway. Recommended to use [1 -20]

#define DEBUG_LOG             // Enable (uncomment) to print debug info. Disabling (comment) debug output saves some 4-5 ms ...

#define SLEEP_SECS 5 * 60 - 8 // [sec] Sleep time between wake up and readings. Will be 5 mins +/- 8 secs. Varying to avoid transmit collusions.
#define MAX_WAKETIME_MS 1000  // [ms]  Timeout until forced gotosleep if no sending success

// -----------------------------------------------------------------------------------------
// GLOBALS
// -----------------------------------------------------------------------------------------
sensor_data_t sensorData;
volatile boolean messageSent; // flag to tell when message is sent out and we can safely goto sleep
esp_now_peer_info_t gateway;

// -----------------------------------------------------------------------------------------
// SETUP FOR YF-S201C FLOW SENSOR
// -----------------------------------------------------------------------------------------
#define SENSOR 2
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
float calibrationFactor = 5.2725; // COULD BE CALIBRATED FURTHER BASED ON EXPERIMENTAL RESULTS
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
int test_num;

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

// -----------------------------------------------------------------------------------------
// SETUP FOR LED
// -----------------------------------------------------------------------------------------
#include <Adafruit_NeoPixel.h>
#define PIN 8
#define ONBOARDPIXEL 0
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);
long starttime = 0;
int timetoturnyellow = 5000; // TO BE UPDATED BASED ON OUTPUT OF ML MODEL
int timetoturnred = 10000;   // TO BE UPDATED BASED ON OUTPUT OF ML MODEL

// -----------------------------------------------------------------------------------------
// FUNCTIONS TO BE USED
// -----------------------------------------------------------------------------------------
void gotoSleep();
void setup_espnow();
void setup_sensor();
void read_flow();
void send_msg();
void setupLED();
void lightLED();

// -----------------------------------------------------------------------------------------
void setup()
// -----------------------------------------------------------------------------------------
{
  // Disable WiFi until we shall use it, to save energy
  WiFi.persistent(false); // Dont save WiFi info to Flash - to save time

  Serial.begin(115200);

  sensorData.unit = UNIT;

  // Set up flow sensor ---------------------------
  setup_sensor();

  // Set up espnow ---------------------------
  setup_espnow();

  // Set up LED ---------------------------
  setupLED();
}

// -----------------------------------------------------------------------------------------
void loop()
// -----------------------------------------------------------------------------------------
{
  // Read flow sensor and update values -----------------------------------
  read_flow();

  // Send message -----------------------------------
  send_msg();

  lightLED();

  delay(1000);
}

// -----------------------------------------------------------------------------------------
void gotoSleep()
// -----------------------------------------------------------------------------------------
{
  int sleepSecs;
  sleepSecs = SLEEP_SECS + ((uint8_t)esp_random() / 16); // add random time to avoid traffic jam collisions

  esp_sleep_enable_timer_wakeup(sleepSecs * 1000000);
  // esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, LOW);
  esp_deep_sleep_start();

  // Never return here - ESP will be reset after deep sleep
}

void setup_espnow()
{
  // Set up ESP-Now link ---------------------------
  WiFi.mode(WIFI_STA); // Station mode for esp-now sensor node
  WiFi.disconnect();
#ifdef DEBUG_LOG
  Serial.printf("My HW mac: %s", WiFi.macAddress().c_str());
  Serial.println("");
  Serial.printf("Sending to MAC: %02x:%02x:%02x:%02x:%02x:%02x", Gateway_Mac[0], Gateway_Mac[1], Gateway_Mac[2], Gateway_Mac[3], Gateway_Mac[4], Gateway_Mac[5]);
  Serial.printf(", on channel: %i\n", WIFI_CHANNEL);
#endif

  // Initialize ESP-now ----------------------------
  if (esp_now_init() != 0)
  {
#ifdef DEBUG_LOG
    Serial.println("*** ESP_Now init failed. Going to sleep");
#endif
    delay(100);
    gotoSleep();
  }

  memcpy(gateway.peer_addr, Gateway_Mac, 6);
  gateway.channel = WIFI_CHANNEL;
  gateway.encrypt = false; // no encryption
  esp_now_add_peer(&gateway);
}

void send_msg()
{
  esp_now_register_send_cb([](const uint8_t *mac, esp_now_send_status_t sendStatus)
                           {
                             // callback for message sent out
                             messageSent = true; // flag message is sent out - we can now safely go to sleep ...
#ifdef DEBUG_LOG
                             Serial.printf("Message sent out, sendStatus = %i\n", sendStatus);
#endif
                           });
  messageSent = false;

#ifdef DEBUG_LOG
  Serial.println("Message Data: " +
                 String(sensorData.unit) + ", flowRate:" +
                 String(sensorData.flowRate) + "L/min, totalFlow: " +
                 String(sensorData.totalMilliLitres) + "ml, test_num: " +
                 String(sensorData.test_num));
#endif
  uint8_t sendBuf[sizeof(sensorData)]; // create a send buffer for sending sensor data (safer)
  memcpy(sendBuf, &sensorData, sizeof(sensorData));

  const uint8_t *peer_addr = gateway.peer_addr;
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&sensorData, sizeof(sensorData));

#ifdef DEBUG_LOG
  Serial.print("Sending result: ");
  Serial.println(result);
#endif
}

void setup_sensor()
{
  pinMode(SENSOR, INPUT_PULLUP);
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;
  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
}

// -----------------------------------------------------------------------------------------
void read_flow()
// -----------------------------------------------------------------------------------------
{
  currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {

    pulse1Sec = pulseCount;
    pulseCount = 0;

    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();

    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;

    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
    test_num++;

    sensorData.flowRate = flowRate;
    sensorData.totalMilliLitres = totalMilliLitres;
  }
}

// -----------------------------------------------------------------------------------------
void setupLED()
// -----------------------------------------------------------------------------------------
{
  pixels.begin(); // This initializes the NeoPixel library.
  pixels.clear();
}

// -----------------------------------------------------------------------------------------
void lightLED()
// -----------------------------------------------------------------------------------------
{
  if (flowRate > 0 && starttime == 0)
  {
    starttime = currentMillis;
  }
  else if (flowRate == 0)
  {
    starttime = 0;
  }

  if (starttime == 0)
  {
    pixels.clear();
  }
  else if (millis() - starttime >= timetoturnred)
  {
    pixels.setPixelColor(ONBOARDPIXEL, pixels.Color(10, 0, 0));
  }
  else if (millis() - starttime >= timetoturnyellow)
  {
    pixels.setPixelColor(ONBOARDPIXEL, pixels.Color(10, 10, 0));
  }
  else if (starttime != 0)
  {
    pixels.setPixelColor(ONBOARDPIXEL, pixels.Color(0, 10, 0));
  }
  pixels.show();
}