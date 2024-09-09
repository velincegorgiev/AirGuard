#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 5); // RX, TX
SoftwareSerial proba(2, 3); // RX, TX

#include"AirQuality.h"
#include"Arduino.h"
AirQuality airqualitysensor;
int current_quality =-1;
// PM10 Sensor
const int pm10SensorPin = A1; // Analog input for PM10 sensor

#include "DFRobot_OxygenSensor.h"

float uvIndex;
int lightValue;
int ozoneConcentration;

#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  10             
DFRobot_OxygenSensor oxygen;

#include <dht.h>

dht DHT;

#define DHT11_PIN 6

int odvlaznuvac=8;
int navlaznuvac=12;
int procistuvac=9;
int proc=7;
int lade=10;
int greac=11;

#define SIGNAL_PIN A2
int value = 0; // variable to store the sensor value

void setup()
{
  pinMode(odvlaznuvac, OUTPUT);
  pinMode(navlaznuvac, OUTPUT);
  pinMode(procistuvac, OUTPUT);
  pinMode(proc, OUTPUT);
  pinMode(lade, OUTPUT);
  pinMode(greac, OUTPUT);
    Serial.begin(9600);
    mySerial.begin(9600);
    proba.begin(9600);
    while(!oxygen.begin(Oxygen_IICAddress)){
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("I2c connect success !");
    airqualitysensor.init(A0);
}
void loop()
{

  value = analogRead(SIGNAL_PIN); // read the analog value from sensor
  Serial.print("Whater level: ");
  Serial.println(value);

  int chk = DHT.read11(DHT11_PIN);
  Serial.print("Temperature = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);

  float oxygenData = oxygen.getOxygenData(COLLECT_NUMBER);
  Serial.print(" oxygen concentration is ");
  Serial.print(oxygenData);
  Serial.println(" %vol");
  
  // Read the analog value from the PM10 sensor
  int pm10SensorValue = analogRead(pm10SensorPin);

  // Convert the analog value to PM10 concentration using the formula
  float pm10Concentration = map(pm10SensorValue, 0, 1023, 0, 100); // Adjust 100 to match the upper limit of your PM10 sensor


  // Print the PM10 and PM2.5 concentrations to the Serial Monitor
  Serial.print("PM10 Concentration: ");
  Serial.print(pm10Concentration);
  Serial.println(" ug/m3");

  // Interpret air quality based on PM10 concentration and print the quality
  if (pm10Concentration <= 12) {
    Serial.println("PM10 Air Quality: Good");
  } else if (pm10Concentration <= 35.4) {
    Serial.println("PM10 Air Quality: Moderate");
  } else if (pm10Concentration <= 55.4) {
    Serial.println("PM10 Air Quality: Unhealthy for Sensitive Groups");
  } else if (pm10Concentration <= 150.4) {
    
    Serial.println("PM10 Air Quality: Unhealthy");
  } else if (pm10Concentration <= 250.4) {
    Serial.println("PM10 Air Quality: Very Unhealthy");
  } else {
    Serial.println("PM10 Air Quality: Hazardous");
  }
    int vrednost=analogRead(A0);
    current_quality=airqualitysensor.slope();
    if (current_quality >= 0)// if a valid data returned.
    {
        if (current_quality==0)
            Serial.println("High pollution! Force signal active");
        else if (current_quality==1)
            Serial.println("High pollution!");
        else if (current_quality==2)
            Serial.println("Low pollution!");
        else if (current_quality ==3)
            Serial.println("Fresh air");
    }

  if(DHT.temperature<20)
  {
    digitalWrite(greac, HIGH);
  }
  else
  {
    digitalWrite(greac, LOW);
  }
  if(DHT.temperature>30)
  {
    digitalWrite(lade, HIGH);
  }
  else
  {
    digitalWrite(lade, LOW);
  }
  if((DHT.humidity>49)&&(value<400))
  {
    digitalWrite(odvlaznuvac, HIGH);
  }
  else{
    digitalWrite(odvlaznuvac, LOW);
  }
  if(DHT.humidity<40)
  {
    digitalWrite(navlaznuvac, HIGH);
  }
  else{
    digitalWrite(odvlaznuvac, LOW);
  }
  
  if((pm10Concentration >=55.4)||(current_quality==2))
  {
    digitalWrite(procistuvac, HIGH);
    digitalWrite(proc, HIGH);
  }
  else{
    digitalWrite(procistuvac, LOW);
    digitalWrite(proc, LOW);
  }
  if (proba.available() > 0) {
    String receivedData = proba.readStringUntil('\n');
//      sender.print("O:");
//  sender.print(ozoneConcentration);
//  sender.print("L:");
//  sender.print(lightValue);
//  sender.print("U:");
//  sender.println(uvIndex);

    int ozone = receivedData.indexOf("O:");
    int light = receivedData.indexOf("L:");
    int uv = receivedData.indexOf("U:");
    ozoneConcentration= receivedData.substring(ozone + 2).toInt();
    lightValue=receivedData.substring(light + 2).toInt();
    uvIndex=receivedData.substring(uv + 2).toFloat();
    Serial.println(ozoneConcentration);
    Serial.println(lightValue);
    Serial.println(uvIndex);
  }

  mySerial.print("W:");
  mySerial.print(value);
  mySerial.print("T:");
  mySerial.print(DHT.temperature);
  mySerial.print("H:");
  mySerial.print(DHT.humidity);
  mySerial.print("O:");
  mySerial.print(oxygenData);
  mySerial.print("P:");
  mySerial.print(pm10Concentration);
  mySerial.print("Z:");
  mySerial.print(ozoneConcentration);
  mySerial.print("L:");
  mySerial.print(lightValue);
  mySerial.print("U:");
  mySerial.print(uvIndex);
  mySerial.print("Q:");
  mySerial.println(vrednost);

  delay(1000);
}
ISR(TIMER2_OVF_vect)
{
    if(airqualitysensor.counter==122)//set 2 seconds as a detected duty
    {
        airqualitysensor.last_vol=airqualitysensor.first_vol;
        airqualitysensor.first_vol=analogRead(A0);
        airqualitysensor.counter=0;
        airqualitysensor.timer_index=1;
        PORTB=PORTB^0x20;
    }
    else
    {
        airqualitysensor.counter++;
    }
}
