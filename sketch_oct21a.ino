#include "DHT.h"
#include "SparkFun_SGP30_Arduino_Library.h"
#include <Wire.h>
#include "MQ131.h"
#include <SoftwareSerial.h> 
#include <TinyGPS++.h> 

// int RXPin = 2; //Digital Pin - 2 (Rx)
// int TXPin = 3; //Digital Pin - 3 (Tx)

SoftwareSerial gpsSerial(4,5); //Digital Pin - 4 & 5 (Rx and Tx)

float lat = 19.021624, lon = 72.870855; // Initial GPS Co-ordinates

SGP30 mySensor; 
long t1, t2;

int c = 0; //Initial CO2 concentration
int h2 = 0; //Initial H2 concentration
int voc = 0; //Inital VOC concentration
int eth = 0; //Initial Ethanol concentration
int o3 = 0; //Initial O3 concentration        

int sensorValueMQ135;

unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

int sensorValue;
int digitalValue;

#define measurePin A5 //Analog Pin - 5 (PM 2.5)

#define ledPower 8 //Digital Pin - 8 (PM 2.5)

#define DHTPIN 7 //Digital Pin - 7 

#define DHTTYPE DHT22

#define MQ135Pin A3 //Analog Pin - 3 (MQ135)

#define MQ131Pin A4 //Digital Pin - 6 (MQ131)

DHT dht(DHTPIN, DHTTYPE);

TinyGPSPlus gps;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
    while (1);
  }
  mySensor.initAirQuality();
  t1 = millis();
  dht.begin();
  MQ131.begin(MQ131Pin, A0, LOW_CONCENTRATION, 1000000);  //
  MQ131.setTimeToRead(20);
  MQ131.setR0(9000);
  gpsSerial.begin(9600);
  pinMode(ledPower,OUTPUT);
}

void loop() {
  delay(1000);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  digitalWrite(ledPower,LOW);
  delayMicroseconds(280);
  voMeasured = analogRead(measurePin);
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(sleepTime);       
  calcVoltage = voMeasured * (5.0 / 1024);
  dustDensity = 0.17 * calcVoltage - 0.1;
  if (dustDensity < 0) {
    dustDensity = 0.00;
  }
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT Sensor!"));
    return;
  }
  t2 = millis();
  if (t2 >= t1 + 1000) {
    t1 = t2;
    mySensor.measureAirQuality();
    c = mySensor.CO2;
    voc = mySensor.TVOC;
    mySensor.measureRawSignals();
    h2 = mySensor.H2;
    eth = mySensor.ethanol;
  }
  o3 = MQ131.getO3(PPB);
  sensorValueMQ135 = analogRead(MQ135Pin);
  while(gpsSerial.available() > 0)
  {
  if(gps.encode(gpsSerial.read())) 
  {  
    if (gps.location.isValid())
    {
      lat = gps.location.lat();
      lon = gps.location.lng();
    }
  }
  }
  String StringToSend = String(h) + "," + String(t) + "," + String(c) + "," + String(dustDensity) + "," + String(voc) + "," + String(h2) + "," + String(eth) + "," + String(o3) + "," + String(sensorValueMQ135)+ "," + String(lat)+ "," + String(lon);
  Serial.println(StringToSend);
}