#include <SoftwareSerial.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <RTClib.h>

char ssid[] = "5BEAFC2";    // your network SSID (name)
char pass[] = "12345678"; // your network password
int status = WL_IDLE_STATUS;     // the WiFi radio's status
WiFiClient client;

SoftwareSerial mySerial(4, 5); // RX, TX
SoftwareSerial prima(2, 3); // RX, TX
SoftwareSerial stm(12, 13); // RX, TX

RTC_DS3231 rtc;
char t[32];

int nivoNavodata;
int temperatura;
int vlaznost;
float kislorod;
float zagaduvanje;
int ozon;
int svetlina;
float uvsvetlina;
int kvalitet;

#define TdsSensorPin A1
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;

int liquidLevel = 0;

const int moistureSensorPin = A0; // Analog pin to which the moisture sensor is connected

const int flowSensorPin = 7; // The digital pin to which the flow sensor is connected
volatile int pulseCount = 0; // Count of pulses from the sensor
float flowRate = 0.0; // Flow rate in liters per minute (L/min)
unsigned int flowMilliLiters = 0; // Flow volume in milliliters (mL)
unsigned long totalMilliLiters = 0; // Total flow volume in milliliters (mL)
unsigned long oldTime = 0;

const unsigned long interval = 6 * 24 * 60 * 60 * 1000; // 6 days in milliseconds
unsigned long previousMillis = 0;

int water = 10;

void pulseCounter() {
  pulseCount++;
}

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    mySerial.begin(9600);
    prima.begin(9600);
    stm.begin(9600);

  if (!rtc.begin()) 
  {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) 
  {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
    pinMode(TdsSensorPin,INPUT);

    pinMode(water, OUTPUT);

     pinMode(flowSensorPin, INPUT_PULLUP); // Set the sensor pin as an input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING); // Attach the interrupt to the pin

    pinMode(9, INPUT);

  // attempt to connect to WiFi network
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection
    delay(10000);
  }
  Serial.println("Connected to wifi");
}

void loop()
{
  DateTime now = rtc.now();
    // Collect RTC values
  int currentYear = now.year();
  int currentMonth = now.month();
  int currentDay = now.day();
  int currentHour = now.hour();
  int currentMinute = now.minute();
  int currentSecond = now.second();
  // Send values to the other Arduino
//  mySerial.print("Y:");
//  mySerial.print(currentYear);
//  mySerial.print("M:");
//  mySerial.print(currentMonth);
//  mySerial.print("D:");
//  mySerial.print(currentDay);
//  mySerial.print("H:");
//  mySerial.print(currentHour);
//  mySerial.print("O:");
//  mySerial.print(currentMinute);
//  mySerial.print("S:");
//  mySerial.println(currentSecond);

  unsigned long currentTime = millis();

  // Calculate the time elapsed since the last calculation
  unsigned long elapsedTime = currentTime - oldTime;

  // Calculate the flow rate (L/min)
  flowRate = (1000.0 / (elapsedTime)) * pulseCount;

  // Calculate the flow volume (mL)
  flowMilliLiters = (flowRate / 60) * elapsedTime;

  // Add the flow volume to the total
  totalMilliLiters += flowMilliLiters;

  // Reset the pulse count and time
  pulseCount = 0;
  oldTime = currentTime;

  
// Read the moisture level from the sensor
  int moistureLevel = analogRead(moistureSensorPin);

  // Map the moisture level to a percentage (0% dry, 100% wet)
  int percentage = map(moistureLevel, 0, 1023, 0, 100);



  liquidLevel = digitalRead(9);
  

   
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
   }

   unsigned long currentMillis = millis();
   if ((currentMillis - previousMillis >= interval)&&(tdsValue<400)&&(liquidLevel>0)&&(percentage<25)&&(now.hour()>19||now.hour()<6)&&(totalMilliLiters<360)&&(temperatura<30)) {
   
     digitalWrite(water, HIGH);
//
//    // Reset the timer
//    previousMillis = currentMillis;
  }
  else{
    digitalWrite(water, LOW);
    totalMilliLiters=0;
     // Reset the timer
    previousMillis = currentMillis;
  }

  if (prima.available() > 0) {
    String receivedData = prima.readStringUntil('\n');
    int valuee = receivedData.indexOf("W:");
    int temperaturee = receivedData.indexOf("T:");
    int humidityy = receivedData.indexOf("H:");
    int oxygen = receivedData.indexOf("O:");
    int concentration = receivedData.indexOf("P:");
    int ozconcentration = receivedData.indexOf("Z:");
    int light = receivedData.indexOf("L:");
    int uvIndex = receivedData.indexOf("U:");
    int vrednost = receivedData.indexOf("Q:");

    nivoNavodata= receivedData.substring(valuee + 2).toInt();
    temperatura= receivedData.substring(temperaturee + 2).toInt();
    vlaznost= receivedData.substring(humidityy + 2).toInt();
    kislorod= receivedData.substring(oxygen + 2).toInt();
    zagaduvanje= receivedData.substring(concentration + 2).toInt();
    ozon= receivedData.substring(ozconcentration + 2).toInt();
    svetlina= receivedData.substring(light + 2).toInt();
    uvsvetlina= receivedData.substring(uvIndex + 2).toInt();
    kvalitet= receivedData.substring(vrednost + 2).toInt();

    
    Serial.println(nivoNavodata);
    stm.print("N:");
    stm.print(nivoNavodata);
    Serial.println(temperatura);
    stm.print("T:");
    stm.print(temperatura);
    Serial.println(vlaznost);
    stm.print("V:");
    stm.print(vlaznost);
    Serial.println(kislorod);
    stm.print("K:");
    stm.print(kislorod);
    Serial.println(zagaduvanje);
    stm.print("Z:");
    stm.print(zagaduvanje);
    Serial.println(ozon);
    stm.print("O:");
    stm.print(ozon);
    Serial.println(svetlina);
    stm.print("S:");
    stm.print(svetlina);
    Serial.println(uvsvetlina);
    stm.print("U:");
    stm.print(uvsvetlina);
    Serial.println(kvalitet);
    stm.print("Q:");
    stm.print(kvalitet);
    Serial.print(currentYear);
    stm.print("Y:");
    stm.print(currentYear);
    Serial.print('/');
    Serial.print(currentMonth);
    stm.print("M:");
    stm.print(currentMonth);
    Serial.print('/');
    Serial.print(currentDay);
    stm.print("D:");
    stm.print(currentDay);
    Serial.print(" ");
    Serial.print(currentHour);
    stm.print("H:");
    stm.print(currentHour);
    Serial.print(':');
    Serial.print(currentMinute);
    stm.print("E:");
    stm.print(currentMinute);
    Serial.print(':');
    Serial.println(currentSecond);
    stm.print("C:");
    stm.print(currentSecond);
    Serial.println(flowRate);
    stm.print("F:");
    stm.print(flowRate);
    Serial.println(totalMilliLiters);
    stm.print("W:");
    stm.print(totalMilliLiters);
    Serial.print(percentage);
    stm.print("P:");
    stm.print(percentage);
    Serial.println("%");
    Serial.print("liquidLevel= "); 
    Serial.println(liquidLevel, DEC);
    stm.print("L:");
    stm.print(liquidLevel);
    Serial.print("TDS Value:");
    Serial.print(tdsValue,0);
    stm.print("X:");
    stm.println(tdsValue);
    Serial.println("ppm");

  }

  if (WiFi.status() == WL_CONNECTED) {
    // your webhook URL
    const char* server = "webhook.site";
    
    // create a string to hold the data
    String postData = "";
    
    // Add the data to the postData string
    postData += "nivoNavodata=";
    postData += String(nivoNavodata);
    postData += "&temperatura=";
    postData += String(temperatura);
    postData += "&vlaznost=";
    postData += String(vlaznost);
    postData += "&kislorod=";
    postData += String(kislorod);
    postData += "&zagaduvanje=";
    postData += String(zagaduvanje);
    postData += "&ozon=";
    postData += String(ozon);
    postData += "&svetlina=";
    postData += String(svetlina);
    postData += "&uvsvetlina=";
    postData += String(uvsvetlina);
    postData += "&kvalitet=";
    postData += String(kvalitet);
    postData += "&currentDateTime=";
    postData += String(currentYear) + "/" + String(currentMonth) + "/" + String(currentDay) + " " +
                 String(currentHour) + ":" + String(currentMinute) + ":" + String(currentSecond);
    postData += "&flowRate=";
    postData += String(flowRate);
    postData += "&totalMilliLiters=";
    postData += String(totalMilliLiters);
    postData += "&percentage=";
    postData += String(percentage);
    postData += "&liquidLevel=";
    postData += String(liquidLevel);
    postData += "&tdsValue=";
    postData += String(tdsValue);

    // establish connection to the server
    if (client.connect(server, 80)) {
      Serial.println("Connected to server");
      // make the HTTP POST request
      client.println("POST /85e60b52-dcf9-4781-9382-e8fa866cb26e HTTP/1.1");
      client.println("Host: webhook.site");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.println(postData.length());
      client.println();
      client.println(postData);
    } else {
      Serial.println("Connection failed");
    }

    // wait for the server to respond
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.print(c);
      }
    }
    // close the connection
    client.stop();
    Serial.println("\nDisconnected from server");
    
    delay(1000);
}
}
