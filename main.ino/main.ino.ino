
// #define GPS_ENABLE
// #define TX_ENABLE
#define RX_ENABLE


// -----------------------------------------------------------------------------------------
// |                                                                                       |
// |                               GPS                                                     |
// |                                                                                       |
// -----------------------------------------------------------------------------------------

#ifdef GPS_ENABLE
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

#endif

// -----------------------------------------------------------------------------------------
// |                                                                                       |
// |                                 TX                                                    |
// |                                                                                       |
// -----------------------------------------------------------------------------------------

#ifdef TX_ENABLE
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


uint8_t tx_address = TX_Addr;
char Message[MessageLong] = "4IR-SC/INSA";
float waitPeriod = 5000; //en ms
float TOA = 0; //en ms

// status variables
int8_t e;

#ifndef CONFIG_CREATED
boolean ConfigOK = true; //passe à false si problème d'allumage, de config de la fréquence ou de la puissance de sortie
#define CONFIG_CREATED
#endif

#endif

// -----------------------------------------------------------------------------------------
// |                                                                                       |
// |                               Rx                                                      |
// |                                                                                       |
// -----------------------------------------------------------------------------------------

#ifdef RX_ENABLE
  
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
#define RX_Addr 8
#define MaxNbRetries 3
#define WaitRxMax 7000 //en ms


uint8_t rx_address = RX_Addr;


#ifndef CONFIG_CREATED
boolean ConfigOK = true; //passe à false si problème d'allumage, de config de la fréquence ou de la puissance de sortie
#define CONFIG_CREATED
#endif

// status variables
int8_t e_rx;

#endif

void setup() {
  // -----------------------------------------------------------------------------------------
  // |                                                                                       |
  // |                               GPS                                                     |
  // |                                                                                       |
  // -----------------------------------------------------------------------------------------

  #ifdef GPS_ENABLE
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

#endif
  // -----------------------------------------------------------------------------------------
  // |                                                                                       |
  // |                               TX                                                      |
  // |                                                                                       |
  // -----------------------------------------------------------------------------------------

#ifdef TX_ENABLE
  
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  // Print a start message
  Serial.println(F("SX1272 module configuration in Arduino"));

  // Power ON the module
  e = sx1272.ON();
  if (e == 0)
  {
    Serial.println(F("SX1272 Module on"));
  }
  else
  {
    Serial.println(F("Problem of activation of SX1272 Module !"));
    ConfigOK = false;
  }

  // Select frequency channel
  e = sx1272.setChannel(freq_centrale);
  Serial.print(F("Frequency channel "));
  Serial.print(freq_centrale,HEX);
  if (e == 0)
  {
    Serial.println(F(" has been successfully set."));
  }
  else
  {
    Serial.println(F(" has not been set !"));
    ConfigOK = false;
  }

  // Select output power
  e = sx1272.setPower(OutPower);
  Serial.print(F("Output power "));
  Serial.print(OutPower,HEX);
  if (e == 0)
  {
    Serial.println(F(" has been successfully set."));
  }
  else
  {
    Serial.println(F(" has not been set !"));
    ConfigOK = false;
  }
  
  if (ConfigOK == true) {
    // Set header
    e = sx1272.setHeaderON();
    // Set transmission mode
    e = sx1272.setCR(CR_5);    // CR = 4/5
    e = sx1272.setSF(SF);   // SF = 12
    e = sx1272.setBW(BW_125);    // BW = 125 KHz
    // Set CRC
    e = sx1272.setCRC_ON();
    // Set the node address
    e = sx1272.setNodeAddress(tx_address);
    // Set the length of preamble
    e = sx1272.setPreambleLength(PreambLong);
    // Set the number of transmission retries
    sx1272._maxRetries = MaxNbRetries; 

    //parameters display 
    Serial.println(F("#Verification of parameters:#"));
    Serial.print(F("  Node address: "));
    Serial.println(sx1272._nodeAddress,DEC);  
    Serial.print(F("  Bandwidth: "));
    Serial.println(sx1272._bandwidth,DEC);  
    Serial.print(F("  Coding rate: "));
    Serial.println(sx1272._codingRate,DEC);
    Serial.print(F("  Spreading factor: "));
    Serial.println(sx1272._spreadingFactor,DEC);  
    Serial.print(F("  Header mode: "));
    Serial.println(sx1272._header,DEC); 
    Serial.print(F("  CRC field: "));
    Serial.println(sx1272._CRC,DEC); 
    Serial.print(F("  BW: "));
    Serial.println(sx1272._bandwidth,DEC); 
    Serial.println(F("SX1272 successfully configured !"));
  }
  else
  {
    Serial.println(F("SX1272 initialization failed !")); 
  }

  if (ConfigOK == true) {
    delay(1000);  
    Serial.println(F("  "));
    Serial.println(F("----------------------------------------"));
    Serial.println(F("Continuous transmission in broadcast mode.")); 
    Serial.println(F("----------------------------------------")); 
  } 

  TOA = sx1272.timeOnAir();
  waitPeriod = PeriodTransmission-TOA;
  Serial.print(F("  TOA (s): "));
  Serial.println(TOA,DEC); 
  if (waitPeriod < 0) {
    Serial.println(F("TOA longer than transmission period !"));
    waitPeriod = 5000;
  }

  #endif
  // -----------------------------------------------------------------------------------------
  // |                                                                                       |
  // |                               RX                                                      |
  // |                                                                                       |
  // -----------------------------------------------------------------------------------------

  #ifdef RX_ENABLE
   // Open serial communications and wait for port to open:
  Serial.begin(115200);

  // Print a start message
  Serial.println(F("SX1272 module configuration in Arduino"));

  // Power ON the module
  e_rx = sx1272.ON();
  if (e_rx == 0)
  {
    Serial.println(F("SX1272 Module on"));
  }
  else
  {
    Serial.println(F("Problem of activation of SX1272 Module !"));
    ConfigOK = false;
  }

  // Select frequency channel
  e_rx = sx1272.setChannel(freq_centrale);
  Serial.print(F("Frequency channel "));
  Serial.print(freq_centrale,HEX);
  if (e_rx == 0)
  {
    Serial.println(F(" has been successfully set."));
  }
  else
  {
    Serial.println(F(" has not been set !"));
    ConfigOK = false;
  }

  // Select output power
  e_rx = sx1272.setPower(OutPower);
  Serial.print(F("Output power "));
  Serial.print(OutPower,HEX);
  if (e_rx == 0)
  {
    Serial.println(F(" has been successfully set."));
  }
  else
  {
    Serial.println(F(" has not been set !"));
    ConfigOK = false;
  }
  
  if (ConfigOK == true) {
    // Set header
    e_rx = sx1272.setHeaderON();
    // Set transmission mode
    e_rx = sx1272.setCR(CR_5);    // CR = 4/5
    e_rx = sx1272.setSF(SF_12);   // SF = 12
    e_rx = sx1272.setBW(BW_125);    // BW = 125 KHz
    // Set CRC
    e_rx = sx1272.setCRC_ON();
    // Set the node address
    e_rx = sx1272.setNodeAddress(rx_address);
    // Set the length of preamble
    e_rx = sx1272.setPreambleLength(PreambLong);
    // Set the number of transmission retries
    sx1272._maxRetries = MaxNbRetries; 

    //parameters display 
    Serial.println(F("#Verification of parameters:#"));
    Serial.print(F("  Node address: "));
    Serial.println(sx1272._nodeAddress,DEC);  
    Serial.print(F("  Bandwidth: "));
    Serial.println(sx1272._bandwidth,DEC);  
    Serial.print(F("  Coding rate: "));
    Serial.println(sx1272._codingRate,DEC);
    Serial.print(F("  Spreading factor: "));
    Serial.println(sx1272._spreadingFactor,DEC);  
    Serial.print(F("  Header mode: "));
    Serial.println(sx1272._header,DEC); 
    Serial.print(F("  CRC field: "));
    Serial.println(sx1272._CRC,DEC); 
    Serial.print(F("  BW: "));
    Serial.println(sx1272._bandwidth,DEC); 
    Serial.println(F("SX1272 successfully configured !"));
  }
  else
  {
    Serial.println(F("SX1272 initialization failed !")); 
  }

  if (ConfigOK == true) {
    //affichage entête
    //statut (correct = 1 or bad = 0 or non received = 2) 
    Serial.println(F("\n "));
    Serial.println(F("Module ready for reception ! "));
    Serial.println(F("Packet status ; Packet number ; Received data ; RSSI packet (dBm) ; source address"));
    Serial.println(F("\n "));
  } 

  #endif
}

void loop() {
  // -----------------------------------------------------------------------------------------
  // |                                                                                       |
  // |                               GPS                                                     |
  // |                                                                                       |
  // -----------------------------------------------------------------------------------------
  #ifdef GPS_ENABLE
  
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

  #endif

  // -----------------------------------------------------------------------------------------
  // |                                                                                       |
  // |                               Tx                                                      |
  // |                                                                                       |
  // -----------------------------------------------------------------------------------------
  #ifdef TX_ENABLE

  uint8_t dest_address = BROADCAST_ADDR;  //adresse du destinataire
  int i = 0;  //compteur de paquets transmis

  if (ConfigOK == true) {
    e = sx1272.sendPacketTimeout(dest_address,Message,WaitTxMax);   

    if (e == 0) {
      Serial.print(F("\n Packet number "));
      Serial.print(i,DEC);
      Serial.println(F("\n sent."));  
      i++;
    }
    else {
      Serial.println(F("\n Trasmission problem !"));  
    }
    delay(waitPeriod); //on met une durée tampon pour émettre toutes les 5 secondes  
  }

  #endif

  // -----------------------------------------------------------------------------------------
  // |                                                                                       |
  // |                               Rx                                                      |
  // |                                                                                       |
  // -----------------------------------------------------------------------------------------
  #ifdef RX_ENABLE
 
 char StatusRXMessage;



  if (ConfigOK == true) {
    e_rx = sx1272.receivePacketTimeout(WaitRxMax);
    //paquet reçu, correct ou non
    if (e_rx == 0) {
      if (sx1272._reception == CORRECT_PACKET) {
        StatusRXMessage = '1';    
      }
      else {
        StatusRXMessage = '0';
      }  
    }
    //pas de réception --> paquet perdu
    else {
      StatusRXMessage = '2';  
    }  
  
    //écriture de la ligne de résultat

    Serial.print(F("Status Message: "));
    Serial.print(StatusRXMessage);
    Serial.print(F(" ;\nPacket Number: "));
    Serial.print(sx1272.packet_received.packnum,DEC);
    Serial.print(F(" ;\nData: "));
    for (uint8_t i =0; i < sx1272.packet_received.length; i++) {
      Serial.print(sx1272.packet_received.data[i]);  
    }
    Serial.print(F(" ;\nRSSI: "));
    e_rx = sx1272.getRSSIpacket();
    Serial.print(sx1272._RSSIpacket, DEC); 
    Serial.print(F(" ;\nAddress: "));
    Serial.print(sx1272.packet_received.src,DEC);
     
    Serial.print(F(" ;\n\n\n"));
  }

  #endif
}
