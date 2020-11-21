#include <OneWire.h>
#include <DallasTemperature.h>
#include <EspMQTTClient.h>

#include "secrets.h"

EspMQTTClient client(
  SECRET_WIFI_AP_NAME,
  SECRET_WIFI_PASSWORD,
  "192.168.1.67",  // MQTT Broker server ip
  SECRET_MQTT_USER,
  SECRET_MQTT_PASSWORD,
  "recirculation-pump"
);

/** One wire Temp sensor **/
const int oneWireBus = D2; 
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

/** Pressure sensor config **/
const int analogInPin = A0;

int ledState = LOW;

unsigned long previousMillis = 0;
const long interval = 100;

void setup() {
  Serial.begin(115200);
  sensors.begin();
  client.enableDebuggingMessages(true);
  pinMode(LED_BUILTIN, OUTPUT);

}

typedef struct deviceData {
  float temp;
  DeviceAddress address;
} DeviceData;

void loop() {
  sensors.requestTemperatures(); 
  int deviceCount = sensors.getDeviceCount();

  DeviceData data[deviceCount];
  for (int i = 0; i < deviceCount; i++) {
    data[i].temp = sensors.getTempFByIndex(i);
    if (!sensors.getAddress(&data[i].address[0], i)) {
      Serial.println("Failed to read device address");
    }
  }
  
  int sensorValue = analogRead(analogInPin);  
  // map it to the range of the PWM out
  int outputValue = map(sensorValue, 0, 1023, 0, 255);
  
  client.loop();

  if (client.isConnected()) {
    size_t maxSize = 1000;
    char topic[maxSize];
    char message[maxSize];
    for (int i = 0; i < deviceCount; i++) {
      snprintf(topic, maxSize, "recirculation-pump/sensors/temperature-%llx", *(uint64_t*)&data[i].address);
      snprintf(message, maxSize, "{\"temperatureF\": %2.2f}", data[i].temp);
      client.publish(topic, message);
    }
    snprintf(topic, maxSize, "recirculation-pump/sensors/pressure-0");
    snprintf(message, maxSize, "{\"raw\": %d, \"scaled\": %d}", outputValue, sensorValue);
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
  client.subscribe("recirculation-pump/serial-messages", [] (const String &payload)  {
    Serial.println(payload);
  });

}
