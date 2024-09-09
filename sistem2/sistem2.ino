#include <SoftwareSerial.h>
#include <ThreeWire.h>  
#include <RtcDS1302.h>

ThreeWire myWire(4,5,2); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

SoftwareSerial mySerial(2, 3); // RX, TX
SoftwareSerial sender(4,5);
#include <Wire.h>               // Include the Wire library for I2C communication
#include <DHT.h>  // Include the DHT library
#include <Stepper.h>

const int stepsPerRevolution = 150;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);


const int svetlo = 7;

const int potrebnasvetlina = 6;

const int uvSensorPin = A0;
const int lightSensorPin = A1;   // Analog pin where the light sensor is connected

// Define the light level thresholds
const int darkThreshold = 100;         // Adjust this value based on your environment
const int lowLightThreshold = 300;     // Adjust this value based on your environment
const int mediumLightThreshold = 600;  // Adjust this value based on your environment
const int highLightThreshold = 800;    // Adjust this value based on your environment
const int veryHighLightThreshold = 900; // Adjust this value based on your environment

//      String yearr;
//      int monthh ;
//      int dayy ;
//      int hourr ;
//      int minutee ;
//      int secondd ;
//      int yearInt;
//
//struct DateTime {
//    int year;
//    int month;
//    int day;
//    int hour;
//    int minute;
//    int second;
//        // Custom comparison operators
//    bool operator>=(const DateTime &other) const {
//        if (year > other.year) return true;
//        if (year < other.year) return false;
//        if (month > other.month) return true;
//        if (month < other.month) return false;
//        if (day > other.day) return true;
//        if (day < other.day) return false;
//        if (hour > other.hour) return true;
//        if (hour < other.hour) return false;
//        if (minute > other.minute) return true;
//        if (minute < other.minute) return false;
//        return second >= other.second;
//    }
//
//    bool operator<=(const DateTime &other) const {
//        if (year < other.year) return true;
//        if (year > other.year) return false;
//        if (month < other.month) return true;
//        if (month > other.month) return false;
//        if (day < other.day) return true;
//        if (day > other.day) return false;
//        if (hour < other.hour) return true;
//        if (hour > other.hour) return false;
//        if (minute < other.minute) return true;
//        if (minute > other.minute) return false;
//        return second <= other.second;
//    }
//};

//state on motro
int motorState=1;

// Set the LCD address and dimensions
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2


#define DHTPIN 6      // Pin where the DHT11 is connected
#define DHTTYPE DHT11 // DHT11 sensor type
DHT dht(DHTPIN, DHTTYPE);

unsigned long startTime = 0;
unsigned long activeTime = 0;
unsigned long previousMillis = 0;
unsigned long interval = 1000;  // Adjust this interval as needed (1 second in this example)
unsigned long dailyInterval = 86400000;  // 24 hours in milliseconds
int receivedYear, receivedMonth, receivedDay, receivedHour, receivedMinute, receivedSecond;



#include "DFRobot_OzoneSensor.h"

#define COLLECT_NUMBER   20              // collect number, the collection range is 1-100
#define Ozone_IICAddress OZONE_ADDRESS_3

DFRobot_OzoneSensor Ozone;

void setup() {
  Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    printDateTime(compiled);

    if (!Rtc.IsDateTimeValid()) 
    {
        // Common Causes:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing

        Serial.println("RTC lost confidence in the DateTime!");
        Rtc.SetDateTime(compiled);
    }

    if (Rtc.GetIsWriteProtected())
    {
        Serial.println("RTC was write protected, enabling writing now");
        Rtc.SetIsWriteProtected(false);
    }

    if (!Rtc.GetIsRunning())
    {
        Serial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) 
    {
        Serial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        Serial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled) 
    {
        Serial.println("RTC is the same as compile time! (not expected but all is fine)");
    }
  // set the speed at 20 rpm:
  myStepper.setSpeed(20);
    Serial.begin(9600);       // Initialize serial communication
    sender.begin(9600);
    mySerial.begin(9600);
    Wire.begin();
    dht.begin();
    startTime = millis();
    pinMode(svetlo, OUTPUT);

    while(!Ozone.begin(Ozone_IICAddress)) {
    Serial.println("I2c device number error !");
    delay(1000);
  }  Serial.println("I2c connect success !");
/*   Set iic mode, active mode or passive mode
       MEASURE_MODE_AUTOMATIC            // active  mode
       MEASURE_MODE_PASSIVE              // passive mode
*/
    Ozone.setModes(MEASURE_MODE_PASSIVE);
}

void loop() {
  RtcDateTime now = Rtc.GetDateTime();

    printDateTime(now);
    Serial.println();

    if (!now.IsValid())
    {
        // Common Causes:
        //    1) the battery on the device is low or even missing and the power line was disconnected
        Serial.println("RTC lost confidence in the DateTime!");
    }
    int lightValue = analogRead(lightSensorPin); // Read analog value
//    
//if (mySerial.available() > 0) {
//    String receivedData = mySerial.readStringUntil('\n');
////    Serial.println(receivedData);
//    // Parse the received string to extract sensor values
//    int currentYear = receivedData.indexOf("Y:");
//    int currentMonth = receivedData.indexOf("M:");
//    int currentDay = receivedData.indexOf("D:");
//    int currentHour = receivedData.indexOf("H:");
//    int currentMinute = receivedData.indexOf("O:");
//    int currentSecond = receivedData.indexOf("S:");
//      yearr = receivedData.substring(currentYear + 2,currentMonth);
//      monthh = receivedData.substring(currentMonth + 2).toInt();
//      dayy = receivedData.substring(currentDay + 2).toInt();
//      hourr = receivedData.substring(currentHour + 2).toInt();
//      minutee = receivedData.substring(currentMinute + 2).toInt();
//      secondd = receivedData.substring(currentSecond + 2).toInt();
//      yearInt = yearr.toInt();
//      Serial.println(yearr);
//       Serial.println(monthh);
//        Serial.println(dayy);
//        Serial.println(hourr);
//        Serial.println(minutee);
//        Serial.println(secondd);
//
//     
//  }


 float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    // Check if any reading failed and exit early if it did
    if (isnan(temperature) || isnan(humidity)) {
        Serial.println("Failed to read data from DHT11 sensor.");
        return;
    }


    

     int sensorValue = analogRead(uvSensorPin);  // Read analog value from UV sensor

    // Calculate UV intensity based on the sensor's formula
    float uvIntensity = sensorValue * (5.0 / 1024.0);  // Convert analog value to voltage
    uvIntensity *= 307.0;  // Conversion factor for GUVA-S12SD sensor
    uvIntensity /= 10.0;   // Convert to mW/cm²

    // Convert UV intensity to UV index using WHO scale
    float uvIndex = uvIntensity / 40.0;  // UV index formula for GUVA-S12SD sensor

    // Print sensor value, calculated UV intensity, and UV index to the Serial Monitor
//    Serial.print("Sensor Value: ");
//    Serial.print(sensorValue);
//    Serial.print("\tUV Intensity: ");
//    Serial.print(uvIntensity);
//    Serial.print(" mW/cm²\t");
    

    // Interpret UV index level based on WHO scale
//    if (uvIndex <= 2.9) {
//        Serial.println("Low UV Index: No protection required.");
//    } else if (uvIndex <= 5.9) {
//        Serial.println("Moderate UV Index: Protection recommended.");
//    } else if (uvIndex <= 7.9) {
//        Serial.println("High UV Index: Protection essential.");
//    } else if (uvIndex <= 10.9) {
//        Serial.println("Very High UV Index: Extra protection required.");
//    } else {
//        Serial.println("Extreme UV Index: Avoid sun exposure.");
//    }
//4-21
RtcDateTime pocni(now.Year(), now.Month(), now.Day(), 4, 30, 0);
RtcDateTime zavrsi(now.Year(), now.Month(), now.Day(), 20, 30, 0);
unsigned long currentMillis = millis();
 if ((lightValue>300)&&(now >= pocni && now <= zavrsi)) {
    activeTime += currentMillis - previousMillis;
    
  }
  float activeHours = activeTime / 3600000.0; // Convert milliseconds to hours
    Serial.print("Sensor active hours today: ");
    Serial.println(activeHours, 2); // Print with 2 decimal places
  if ((now.Hour()==4)&&(now.Minute()==0)) {
    activeTime = 0;
    startTime = currentMillis;
  }

  previousMillis = currentMillis;

if (((now.Hour() >= 21 && now.Hour() <= 23) || (now.Hour() >= 0 && now.Hour() < 3))&&(potrebnasvetlina-activeTime / 3600000.0)>0) {
   activeTime += currentMillis - previousMillis;
   Serial.println(activeTime/ 3600000.0);
   digitalWrite(svetlo, HIGH);
}
else
{
  digitalWrite(svetlo, LOW);
}
previousMillis = currentMillis;
if((motorState==1)&&((activeTime / 3600000.0)>7)||((uvIndex>6)||(temperature>37)))
{
  myStepper.step(stepsPerRevolution);
  motorState=0;
  
}
else if((motorState==0)&&(uvIndex<5)&&(temperature<35))
{
  myStepper.step(-stepsPerRevolution);
  motorState=1;
}
int16_t ozoneConcentration = Ozone.readOzoneData(COLLECT_NUMBER);
  Serial.print("Ozone concentration is ");
  Serial.print(ozoneConcentration);
  Serial.println(" PPB.");

  sender.print("O:");
  sender.print(ozoneConcentration);
  sender.print("L:");
  sender.print(lightValue);
  sender.print("U:");
  sender.println(uvIndex);
    delay(1000); // Delay for better readability
}
// Define a function to create a DateTime object
//DateTime createDateTime(int year, int month, int day, int hour, int minute, int second) {
//    DateTime dt;
//    dt.year = year;
//    dt.month = month;
//    dt.day = day;
//    dt.hour = hour;
//    dt.minute = minute;
//    dt.second = second;
//    return dt;
//}
#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
}
