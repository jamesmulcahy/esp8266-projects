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
  "recirculation-pump"      // Client name that uniquely identify your device
);

/** One wire Temp sensor **/
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

/** Pressure sensor config **/
const int analogInPin = A0;

void setup() {
  Serial.begin(115200);
  sensors.begin();
  client.enableDebuggingMessages(true);
}

void loop() {
  sensors.requestTemperatures(); 
  
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.print("DS18B20 Temp\n");
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");
  
  int sensorValue = analogRead(analogInPin);
  
  // map it to the range of the PWM out
  int outputValue = map(sensorValue, 0, 1023, 0, 255);
  
  // print the readings in the Serial Monitor
  Serial.print("Pressure Sensor\n sensorValue =");
  Serial.print(sensorValue);
  Serial.print("\t output = ");
  Serial.println(outputValue);
  

  client.publish("recirculation-pump/test", "This is a message");

  client.loop();
  delay(1000);
}

void onConnectionEstablished() {
  Serial.println("Connection established to MQTT");
  client.subscribe("recirculation-pump/serial-messages", [] (const String &payload)  {
    Serial.println(payload);
  });

}
