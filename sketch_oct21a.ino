#include "DHT.h"
#include "Adafruit_SGP30.h"
#include <Wire.h>
#include "MQ131.h"
#include <SoftwareSerial.h> 
#include <TinyGPS++.h> 


//Connect Tx - 4 !Important   
// SoftwareSerial gpsSerial(4,5); //Digital Pin - 4 & 5 (Rx and Tx)

float lat = 19.021624;
float lon = 72.870855; // Initial GPS Co-ordinates

Adafruit_SGP30 sgp;

float c = 0; //Initial CO2 concentration
float h2 = 0; //Initial H2 concentration
float voc = 0; //Inital VOC concentration
float eth = 0; //Initial Ethanol concentration
float o3 = 0; //Initial O3 concentration        

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

#define MQ131Pin 2 //Digital Pin - 2 (MQ131)

DHT dht(DHTPIN, DHTTYPE);

TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  // Serial1.begin(115200);  
  Serial1.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  dht.begin();
  MQ131.begin(MQ131Pin, A1, LOW_CONCENTRATION, 1000000);  //
  MQ131.calibrate();
  // gpsSerial.begin(9600);
  pinMode(ledPower,OUTPUT);
  // Serial1.println("TVOC TEST");
    if (!sgp.begin()) {  
        Serial1.println("Sensor not found");
        while (1);
    }
    // Serial1.println("\nInitialization...");
}

void loop() {
  int counter = 0;
  while(counter<=45){
    if (!sgp.IAQmeasure()) {
        // Serial1.println("Measurement failed");
    }
  if (! sgp.IAQmeasureRaw()) {
    // Serial1.println("Raw Measurement failed");
  }
  voc = sgp.TVOC;
  c = sgp.eCO2;
  h2 = sgp.rawH2;
  eth = sgp.rawEthanol;
  counter++;
  delay(500);
  }
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  int counter1 = 0;
  while(counter1 <= 45){
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
    counter1++;
    delay(500);
  }
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT Sensor!"));
    return;
  }
  MQ131.sample();
  o3 = MQ131.getO3(PPB);
  sensorValueMQ135 = analogRead(MQ135Pin);

  int counter2 = 0;
  while(counter2<=45)
    {
      if (Serial1.available() > 0)
      {
      gps.encode(Serial1.read());
      if (gps.location.isUpdated())
      { 
      lat = gps.location.lat();
      lon = gps.location.lng();
      }
      }
      counter2++;
      delay(500);
    } 
  String StringToSend = String(h) + "," + String(t) + "," + String(c) + "," + String(dustDensity) + "," + String(voc) + "," + String(h2) + "," + String(eth) + "," + String(o3) + "," + String(sensorValueMQ135)+ "," + String(lat)+ "," + String(lon);
  Serial.println(StringToSend);
  delay(1000);
}