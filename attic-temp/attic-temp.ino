#include <EspMQTTClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

#include "secrets.h"

// I2C. Use pins D1 (SCL), D2 (SDA)

EspMQTTClient client(
  SECRET_WIFI_AP_NAME,
  SECRET_WIFI_PASSWORD,
  "192.168.1.67",  // MQTT Broker server ip
  SECRET_MQTT_USER,
  SECRET_MQTT_PASSWORD,
  "recirculation-pump"
);

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();


int ledState = LOW;

unsigned long previousMillis = 0;
const long interval = 100;

void setup() {
  Serial.begin(115200);
  client.enableDebuggingMessages(true);
  pinMode(LED_BUILTIN, OUTPUT);


  if (!bme.begin()) {
    Serial.println("Couldn't find BME280 sensor, check wiring!");
    while (1) delay(10);
  }
}

void loop() {
  
  client.loop();

  if (client.isConnected()) {
    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

    size_t maxSize = 1000;
    char topic[maxSize];
    char message[maxSize];
    // Temperature
    
    snprintf(topic, maxSize, "milton/sensors/attic-temperature-0");
    snprintf(message, maxSize, "{\"temperatureF\": %3.2f}", temp_event.temperature);
    client.publish(topic, message);
    // hPa
    snprintf(topic, maxSize, "milton/sensors/attic-pressure-0");
    snprintf(message, maxSize, "{\"pressure\": %4.2f}", pressure_event.pressure);
    client.publish(topic, message);

    snprintf(topic, maxSize, "milton/sensors/attic-humidity-0");
    snprintf(message, maxSize, "{\"relative\": %4.2f}", humidity_event.relative_humidity);
    client.publish(topic, message);

    delay(1000);
  } else {
    // If we're not connected, blink the LED
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      digitalWrite(LED_BUILTIN, ledState);
    }
    delay(10);
  }
  
}

void onConnectionEstablished() {
  Serial.println("Connection established to MQTT");
  // Turn the LED on to indicate we're in a good state.
  digitalWrite(LED_BUILTIN, HIGH);
  client.subscribe("attic1/serial-messages", [] (const String &payload)  {
    Serial.println(payload);
  });

}
