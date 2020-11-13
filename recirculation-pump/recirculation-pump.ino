#include <OneWire.h>
#include <DallasTemperature.h>

/** One wire Temp sensor **/
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

/** Pressure sensor config **/
const int analogInPin = A0;

void setup() {
  Serial.begin(115200);
  sensors.begin();
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

  delay(5000);
}
