// -----------------------------------------------------------------------------------------
// |                                                                                       |
// |                               GPS                                                     |
// |                                                                                       |
// -----------------------------------------------------------------------------------------
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>


#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);

#define PeriodUpdateGPS 2000 //en ms


#define GPSECHO  false //pour ne pas avoir les données brutes

boolean usingInterrupt = true;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

// -----------------------------------------------------------------------------------------
// |                                                                                       |
// |                                 TX                                                    |
// |                                                                                       |
// -----------------------------------------------------------------------------------------

#include <Wire.h>
#include <arduinoUtils.h>

// Include the SX1272 and SPI library:
#include "sx1272_INSAT.h"
#include <SPI.h>

#define freq_centrale CH_868v1
#define BW BW_125
#define CR CR_5
#define SF SF_12
#define OutPower POW_14
#define PreambLong 12
#define TX_Addr 2
#define MessageLong 12
#define MaxNbRetries 3
#define PeriodTransmission 5000 //en ms
#define WaitTxMax 2000 //en ms


void setup() {
  // -----------------------------------------------------------------------------------------
  // |                                                                                       |
  // |                               GPS                                                     |
  // |                                                                                       |
  // -----------------------------------------------------------------------------------------
  // put your setup code here, to run once:
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // autorisation des interruptions.
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  Serial.println(PMTK_Q_RELEASE);  //marche.
  Serial.println(ARDUINO); 
}

void loop() {
  // -----------------------------------------------------------------------------------------
  // |                                                                                       |
  // |                               GPS                                                     |
  // |                                                                                       |
  // -----------------------------------------------------------------------------------------
  // put your main code here, to run repeatedly:
    // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();


  // approximately every PeriodUpdateGPS ms or so, print out the current stats
  if (millis() - timer > PeriodUpdateGPS) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    //Uncomment the loop to display NMEA frame only when GPS module is locked on satellites (GPS fix).
    //if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
   // }
  }
}
