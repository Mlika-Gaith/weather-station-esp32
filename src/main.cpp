/****************************************
 * Include Libraries
 ****************************************/
#include <Arduino.h>
#include <DHT.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_SI1145.h"
#include <Adafruit_BMP085.h>
#include <WiFi.h>
#include "UbidotsEsp32Mqtt.h"
/****************************************
 * Define Constants
 ****************************************/
#define DHTPIN 4 // for DHT sensor
#define DHTTYPE 22
#define rainAnalog 35
#define rainDigital 34
#define windAnalog 33
const char *UBIDOTS_TOKEN = "BBFF-SIWc6Kj79JulfzayasC6qEDeK6aMo1";  // Ubidots TOKEN
const char *WIFI_SSID = "TOPNET_69F8";      // Wi-Fi SSID
const char *WIFI_PASS = "9k0cvzo2q6";      // Wi-Fi password
const char *DEVICE_LABEL = "ghaith-s-esp32";   // Device label
const char *TEMPERATURE_LABEL = "temperature"; // Put here your Variable label to which data  will be published
const char *HUMIDITY_LABEL = "Humidity"; // Put here your Variable label to which data  will be published
const char *TEMPERATURE_F_LABEL = "Temperature_f"; // Put here your Variable label to which data  will be published
const char *HEAT_INDEX_C_LABEL = "Heat_Index_c"; // Put here your Variable label to which data  will be published
const char *HEAT_INDEX_F_LABEL = "Heat_Index_f"; // Put here your Variable label to which data  will be published
const char *WIND_SPEED_LABEL = "Wind_Speed"; // Put here your Variable label to which data  will be published
const char *UVINDEX_LABEL = "UVIndex"; // Put here your Variable label to which data  will be published
const char *VISIBLE_LIGHT_LABEL = "Visible_light"; // Put here your Variable label to which data  will be published
const char *INFRARED_LIGHT_LABEL = "Infrared_light"; // Put here your Variable label to which data  will be published
const char *PRESSURE_LABEL = "Pressure"; // Put here your Variable label to which data  will be published
const char *ALTITUDE_LABEL = "Altitude"; // Put here your Variable label to which data  will be published
const char *PRESSURE_SEA_LEVEL_LABEL = "Pressure_Sea_Level"; // Put here your Variable label to which data  will be published
const char *RAIN_STATUS_LABEL = "Rain_Status"; // Put here your Variable label to which data  will be published
const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds
unsigned long timer;

DHT dht(DHTPIN, DHTTYPE);
Adafruit_SI1145 uv = Adafruit_SI1145();
Adafruit_BMP085 bmp;
Ubidots ubidots(UBIDOTS_TOKEN);

/****************************************
 * Auxiliar Functions
 ****************************************/

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
/*==============================================*/
/*======= Temperature ==========================*/
/*==============================================*/
void readTemperature(){
  Serial.println(F("DHT Sensor Test !"));
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true); // read temp in fahrenheit

  // check if any reads failed
  if (isnan(h) || isnan(t) || isnan(f)){
    Serial.println(F("Failed to read from DHT sensor !!"));
    return;
  }

  float hif = dht.computeHeatIndex(f, h);// Temperature in °F
  float hic = dht.computeHeatIndex(t, h, false);// feels like to the human body
  Serial.print(F("Humidity: ")); 
  Serial.print(h);
  Serial.print(F(" %  Temperature: "));
  Serial.print(t);
  Serial.print(F(" °C "));
  Serial.print(f);
  Serial.print(F(" °F  Heat index: "));
  Serial.print(hic);
  Serial.print(F(" °C "));
  Serial.print(hif);
  Serial.println(F(" °F"));
  float time = millis() - timer;
  if (abs(time) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    ubidots.add(TEMPERATURE_LABEL, t); // Insert your variable Labels and the value to be sent
    ubidots.add(HUMIDITY_LABEL,h); // Insert your variable Labels and the value to be sent
    ubidots.add(TEMPERATURE_F_LABEL, f); // Insert your variable Labels and the value to be sent
    ubidots.add(HEAT_INDEX_C_LABEL, hic); // Insert your variable Labels and the value to be sent
    ubidots.add(HEAT_INDEX_F_LABEL, hif); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }

}

/*==============================================*/
/*======= Uv light ==========================*/
/*==============================================*/

void readUV(){
  Serial.println(F("UV light Sensor Test !"));
  int vis = uv.readVisible(); // visible light level
  int ir = uv.readIR(); // Infrared light level
  float UVindex = uv.readUV();
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!
  UVindex /= 100.0;
  Serial.print(F("Visible Light level: ")); 
  Serial.print(vis);
  Serial.print(F(" Infrared light level: "));
  Serial.print(ir);
  Serial.print(F(" UVindex: "));
  Serial.print(UVindex);
  Serial.println(F(""));
  float time = millis() - timer;
  if (abs(time) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    ubidots.add(UVINDEX_LABEL, UVindex); // Insert your variable Labels and the value to be sent
    ubidots.add(VISIBLE_LIGHT_LABEL, vis); // Insert your variable Labels and the value to be sent
    ubidots.add(INFRARED_LIGHT_LABEL, UVindex); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }
  delay(2000);
}

/*==============================================*/
/*======= Pressure ==========================*/
/*==============================================*/

void readPressure(){
  Serial.println(F("Pressure Sensor Test !"));
  int pressure = bmp.readPressure();
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  float altitude = bmp.readAltitude();
  // sea level pressure
  int pressureSeaLevel = bmp.readSealevelPressure();
  Serial.print(F("Pressure: ")); 
  Serial.print(pressure);
  Serial.print(F(" Pa  Altitude: "));
  Serial.print(altitude);
  Serial.print(F(" meters  Pressure at Sea Level: "));
  Serial.print(pressureSeaLevel);
  Serial.print(F(" Pa "));
  Serial.println(F(""));
  float time = millis() - timer;
  if (abs(time) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    ubidots.add(PRESSURE_LABEL, pressure); // Insert your variable Labels and the value to be sent
    ubidots.add(ALTITUDE_LABEL,altitude); // Insert your variable Labels and the value to be sent
    ubidots.add(PRESSURE_SEA_LEVEL_LABEL, pressureSeaLevel); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }
}

/*==============================================*/
/*======= Rain yes or no ==========================*/
/*==============================================*/

void readRain(){
  Serial.println(F("Rain Sensor Test !"));
  int rainAnalogVal = analogRead(windAnalog);
  int rainDigitalVal = digitalRead(rainDigital); // 0 if it's raining
  Serial.print(F("Rain analog: ")); 
  Serial.print(rainAnalogVal);
  Serial.print(F("  rainDigitalVal: "));
  Serial.print(rainDigitalVal);
  if(rainDigitalVal == 0){
    Serial.print(F("  it's raining !!"));
  }
  Serial.println(F(""));
  float time = millis() - timer;
  if (abs(time) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    ubidots.add(RAIN_STATUS_LABEL, rainDigitalVal); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }
}

/*==============================================*/
/*======= Wind Speed ==========================*/
/*==============================================*/

void readWindSpeed(){
  Serial.println(F("Wind Speed Sensor Test !"));
  int windAnalogVal = analogRead(windAnalog);
  float voltage= windAnalogVal * (5.0 / 1023.0);
  float windSpeed = (voltage / 0.01);
  float min_windSpeed = (voltage / 0.01*0.9);
  float max_windSpeed = (voltage / 0.01*0.9);
  Serial.print(F("Voltage: ")); 
  Serial.print(voltage);
  Serial.print(F(" V Wind Speed: "));
  Serial.print(windSpeed);
  Serial.print(F(" Km/h Min Wind Speed:"));
  Serial.print(min_windSpeed);
  Serial.print(F(" Km/h  Max Wind Speed: "));
  Serial.print(max_windSpeed);
  Serial.print(F(" Km/h "));
  Serial.println(F(""));
  float time = millis() - timer;
  if (abs(time) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    ubidots.add(WIND_SPEED_LABEL,windSpeed); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }
}

/****************************************
 * Main Functions
 ****************************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if (!bmp.begin()) {
    Serial.println("Could not find a valid Pressure Sensor, check wiring!");
    while (1);
  }
  if (! uv.begin()) {
    Serial.println("Didn't find A UV light Sensor, check wiring!");
    while (1);
  }
  
  dht.begin();
  pinMode(rainDigital,INPUT);

  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();

  timer = millis();
  
}

void loop() {
  // put your main code here, to run repeatedly:
   if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  readTemperature();
  readUV();
  readPressure();
  readRain();
  readWindSpeed();
  ubidots.loop();
  delay(2000);
}