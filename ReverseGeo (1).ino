/* Original Code by Kenton Harris 11/12/2012 edited by Kyle 02/29/16

This program unlocks a box that has reached a certain GPS location.

The device uses the following products from Adafruit Industries:
Arduino Nano
Ultimate GPS (version1): http://www.adafruit.com/products/746
16x2 Character LCD: https://www.adafruit.com/products/181
TPro Micro Servo SG90: https://www.adafruit.com/products/169

Tutorials for these products found on learn.adafruit.com helped with much of this.
Copyright (c) 2012, Adafruit Industries
All rights reserved.

Thanks to bnordlund9 for much of the code. This is  simplified verison of his Geobox found here:
http://www.youtube.com/watch?v=g0060tcuofg
Credit to Mikal Hart of http://arduiniana.org/ for the original idea of Reverse Geocache.
*/

#include <math.h>
#include <LiquidCrystal.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//GPS 
SoftwareSerial mySerial(9,10);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO false                             //make true to debug GPS
boolean usingInterrupt = false;
void useInterrupt(boolean);

//Servo
#include <Servo.h>
Servo servoLatch;

//Declarations
const float deg2rad = 0.01745329251994;
const float rEarth = 6370.0;                                             // can replace with 3958.75 mi, 6370.0 km, 3440.06 NM, or 6371000.0m
float range = 3000;                                                      // distance from HERE to THERE
String here;                                                             // read from GPS

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(0, 1, 5, 4, 3, 2);                                     //Pins here are 0, 1 for the nano which are tx and rx

int gpsWasFixed = HIGH;                                                  // did the GPS have a fix?
int ledFix = 7;                                                          // pin for fix LED (not implemented on V1
int servoPin = 6;                                                        // pin for servo
int servoLock = 50;                                                      // angle (deg) of "locked" servo
int servoUnlock = 160;                                                   // angle (deg) of "unlocked" servo

//String there = "N38 51.409, W077 01.328";                              //Desired Location goes here. Make sure you use the same syntax and number of characters
                                                                         //google maps outputs this 37°57'37.8"N 121°59'14.1"W, should be this N37 57.378, W121 59.141
String there = "N34 07.131, W118 18.023";                                //Sun dial at griffith observatory, LA

//String there = "N37 57.430, W121 59.175";                              // Near Kyles house at Wharton Way, Concord, CA 



void setup()
{
  servoLatch.attach(servoPin);
  servoLatch.write(servoLock);
  delay(500);
  servoLatch.detach();
  lcd.begin(16, 2);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);                          // RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);                             // 1 Hz update rate can be faster but there is no need with this application
  useInterrupt(true);                                                    // reads the steaming data in a background
  
  lcd.begin(16, 2);
  lcd.setCursor(6,0);
  lcd.println("Hello     ");
  lcd.setCursor(5,1);
  lcd.println("Matthew           ");
  delay(5000);

}

void loop(){
  // Parse GPS and recalculate RANGE
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))                                      // also sets the newNMEAreceived() flag to false
      return;                                                            // We can fail to parse a sentence in which case we should just wait for another
  }
    if (GPS.fix) {
    gpsWasFixed = HIGH;
    digitalWrite(ledFix, HIGH);

  
    here = gps2string ((String) GPS.lat, GPS.latitude, (String) GPS.lon, GPS.longitude);
    range = haversine(string2lat(here), string2lon(here), string2lat(there), string2lon(there));
  
    /*
    Serial.print("Here: ");                                        //for GPS debug
    Serial.print(here);
    Serial.print("There: ");
    Serial.println(there);
    Serial.print("Range: ");
    Serial.print(range);
    Serial.println("km");
    */

    lcd.clear();                                                     //Displays range information on LCD screen
    lcd.print("Range: ");
    lcd.print(range);
    lcd.print("km");

    delay(1000);
  }
  else {                                                              //No GPS fix- take box outside
    
    lcd.setCursor(0,0);
    lcd.print("Take me outside!");
    lcd.setCursor(0,1);
    lcd.print("Locking Sat...");   
    delay(200);
  }
  
  if (range < 0.10){
    servoLatch.attach(servoPin);
    servoLatch.write(servoUnlock);
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("You found it! ");
    servoLatch.detach();
    delay(100000);
  }
}


SIGNAL(TIMER0_COMPA_vect) {
  // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  char c = GPS.read();
  if (GPSECHO)
    if (c) UDR0 = c;  
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

String int2fw (int x, int n) {
  // returns a string of length n (fixed-width)
  String s = (String) x;
  while (s.length() < n) {
    s = "0" + s;
  }
  return s;
}

String gps2string (String lat, float latitude, String lon, float longitude) {
  // returns "Ndd mm.mmm, Wddd mm.mmm";
  int dd = (int) latitude/100;
  int mm = (int) latitude % 100;
  int mmm = (int) round(1000 * (latitude - floor(latitude)));
  String gps2lat = lat + int2fw(dd, 2) + " " + int2fw(mm, 2) + "." + int2fw(mmm, 3);
  dd = (int) longitude/100;
  mm = (int) longitude % 100;
  mmm = (int) round(1000 * (longitude - floor(longitude)));
  String gps2lon = lon + int2fw(dd, 3) + " " + int2fw(mm, 2) + "." + int2fw(mmm, 3);
  String myString = gps2lat + ", " + gps2lon;
  return myString;
};
/*
float string2radius (String myString) {
  // returns a floating-point number: e.g. String myString = "Radius: 005.1 NM";
  float r = ((myString.charAt(8) - '0') * 100.0) + ((myString.charAt(9) - '0') * 10.0) + ((myString.charAt(10) - '0') * 1.0) + ((myString.charAt(12) - '0') * 0.10);
  return r;
};*/

float string2lat (String myString) {
  // returns radians: e.g. String myString = "N38 58.892, W076 29.177";
  float lat = ((myString.charAt(1) - '0') * 10.0) + (myString.charAt(2) - '0') * 1.0 + ((myString.charAt(4) - '0') / 6.0) + ((myString.charAt(5) - '0') / 60.0) + ((myString.charAt(7) - '0') / 600.0) + ((myString.charAt(8) - '0') / 6000.0) + ((myString.charAt(9) - '0') / 60000.0);
  //Serial.print("float lat: ");
  //Serial.println(lat);
  lat *= deg2rad;
  if (myString.charAt(0) == 'S')
    lat *= -1;                                                           // Correct for hemisphere
  return lat;
};

float string2lon (String myString) {
  // returns radians: e.g. String myString = "N38 58.892, W076 29.177";
  float lon = ((myString.charAt(13) - '0') * 100.0) + ((myString.charAt(14) - '0') * 10.0) + (myString.charAt(15) - '0') * 1.0 + ((myString.charAt(17) - '0') / 6.0) + ((myString.charAt(18) - '0') / 60.0) + ((myString.charAt(20) - '0') / 600.0) + ((myString.charAt(21) - '0') / 6000.0) + ((myString.charAt(22) - '0') / 60000.0);
  //Serial.print("float lon: ");
  //Serial.println(lon);
  lon *= deg2rad;
  if (myString.charAt(12) == 'W')
    lon *= -1;                                                           // Correct for hemisphere
  return lon;
};


float haversine (float lat1, float lon1, float lat2, float lon2) {
  // returns the great-circle distance between two points (radians) on a sphere
  float h = sq((sin((lat1 - lat2) / 2.0))) + (cos(lat1) * cos(lat2) * sq((sin((lon1 - lon2) / 2.0))));
  float d = 2.0 * rEarth * asin (sqrt(h)); 
  //Serial.println(d);
  return d;
  
};
