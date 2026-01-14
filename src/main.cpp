#include <Arduino.h>
#include "driver/twai.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

/* ================== PIN CONFIG ================== */
#define CAN_TX GPIO_NUM_5
#define CAN_RX GPIO_NUM_4

#define DHTPIN 27 
#define DHTTYPE DHT11

#define TRIG_PIN 18
#define ECHO_PIN 19

#define OLED_SDA 21
#define OLED_SCL 22

#define REVERSE_LED_PIN 25

/* ================== LED BLINK CONFIG ================== */
#define LED_BLINK_SLOW_MS 500
#define LED_BLINK_NORMAL_MS 250
#define LED_BLINK_FAST_MS 100

/* ================== SENSOR TIMING ================== */
#define SENSOR_INTERVAL_MS 500 

/* ================== OBJECTS ================== */
DHT dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

/* ================== ECU STATE ================== */
bool sensorsEnabled = false; // Reverse OFF by default
bool forcedFault = false;

float temperature = 0;
float humidity = 0;
uint16_t distanceCm = 0;
uint8_t faultFlags = 0;

/* ================== TIMERS ================== */
unsigned long lastDhtRead = 0;
unsigned long lastUltrasonicRead = 0;
unsigned long lastLedToggle = 0;

/* ================== LED STATE ================== */
bool ledState = false;

uint16_t readDistance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 40000);
  if (duration == 0)
    return 0;

  return duration / 58; // cm
}

void updateReverseLed()
{
  if (!sensorsEnabled || distanceCm == 0)
  {
    digitalWrite(REVERSE_LED_PIN, LOW);
    return;
  }

  unsigned long now = millis();
  unsigned long interval;

  if (distanceCm <= 15)
    interval = LED_BLINK_FAST_MS;
  else if (distanceCm <= 30)
    interval = LED_BLINK_NORMAL_MS;
  else
  {
    digitalWrite(REVERSE_LED_PIN, HIGH); // safe distance
    return;
  }

  if (now - lastLedToggle >= interval)
  {
    ledState = !ledState;
    digitalWrite(REVERSE_LED_PIN, ledState);
    lastLedToggle = now;
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(REVERSE_LED_PIN, OUTPUT);
  digitalWrite(REVERSE_LED_PIN, LOW);

  dht.begin();

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  Serial.println("ESP32 CAN ECU STARTED (DHT always ON)");
}

/* ================== LOOP ================== */
void loop()
{

  unsigned long now = millis();

  /* ---------- CAN RX ---------- */
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK)
  {
    if (rx.identifier == 0x200 && rx.data_length_code >= 2)
    {
      if (rx.data[0] == 0x01)
        sensorsEnabled = rx.data[1]; // Reverse ON/OFF
      if (rx.data[0] == 0x02)
        forcedFault = rx.data[1];
    }
  }

  if (now - lastDhtRead >= SENSOR_INTERVAL_MS)
  {
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (!isnan(t) && !isnan(h))
    {
      temperature = t;
      humidity = h;
    }
    lastDhtRead = now;
  }

  if (sensorsEnabled && (now - lastUltrasonicRead >= SENSOR_INTERVAL_MS))
  {
    distanceCm = readDistance();
    lastUltrasonicRead = now;
  }

  updateReverseLed();

  faultFlags = 0;
  if (isnan(temperature))
    faultFlags |= (1 << 0);
  if (sensorsEnabled && distanceCm == 0)
    faultFlags |= (1 << 1);
  if (temperature >= 90)
    faultFlags |= (1 << 2);
  if (sensorsEnabled && distanceCm <= 30)
    faultFlags |= (1 << 3);
  if (forcedFault)
    faultFlags |= (1 << 7);

  /* ---------- CAN TX ---------- */
  twai_message_t tx{};
  tx.identifier = 0x100;
  tx.data_length_code = 8;

  tx.data[0] = 50;
  tx.data[1] = (int8_t)temperature;
  tx.data[2] = (uint8_t)humidity;
  tx.data[3] = faultFlags;
  tx.data[4] = sensorsEnabled;
  tx.data[5] = forcedFault;

  twai_transmit(&tx, pdMS_TO_TICKS(20));

  /* ---------- OLED DISPLAY ---------- */
  display.clearDisplay();
  display.setTextColor(WHITE);

  if (!sensorsEnabled)
  {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("ECU STATUS");

    display.setTextSize(1);
    display.setCursor(0, 22);
    display.print("Temp: ");
    display.print(temperature);
    display.println(" C");

    display.setCursor(0, 34);
    display.print("Hum : ");
    display.print(humidity);
    display.println(" %");

    display.setCursor(0, 50);
    display.println("MODE: NORMAL");
  }
  else
  {
    const char *status = "SAFE";
    if (distanceCm <= 15)
      status = "DANGER";
    else if (distanceCm <= 30)
      status = "CAUTION";
    else if (distanceCm <= 100)
      status = "NEAR";

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("REVERSE ASSIST");

    display.setCursor(0, 20);
    display.setTextSize(2);
    display.print(distanceCm);
    display.println("cm");

    display.setCursor(0, 40);
    display.setTextSize(1);
    display.print("Status: ");
    display.println(status);
    display.setCursor(0, 50);
    display.print("T:");
    display.print(temperature);
    display.print("C ");

    display.print("H:");
    display.print(humidity);
    display.print("%");
  }

  display.display();
  delay(10);
}
