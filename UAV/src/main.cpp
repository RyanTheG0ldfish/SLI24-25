#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
static const int RXPin = 0, TXPin = 1; //CHECK IF THIS IS CORRECT
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

Adafruit_BMP3XX bmp = Adafruit_BMP3XX();
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_Mosi 11
#define BMP_CS 4
#define SEALEVELPRESSURE_HPA (1013.25)
//VERIFY ALL OF THESE ARE RIGHT OR WRONG

RH_RF95 driver(3, 20); // CHECK IF CORRRECT

RHReliableDatagram manager(driver, CLIENT_ADDRESS);

void setup() {
Serial.begin(115200);
ss.begin(GPSBaud);

if (!manager.init())
    Serial.println("Radio init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 2 to 20 dBm:
//  driver.setTxPower(20, false);
  // If you are using Modtronix inAir4 or inAir9, or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for 0 to 15 dBm and with useRFO true. 
  // Failure to do that will result in extremely low transmit powers.
//  driver.setTxPower(14, true);

  // You can optionally require this module to wait until Channel Activity
  // Detection shows no activity on the channel before transmitting by setting
  // the CAD timeout to non-zero:
//  driver.setCADTimeout(10000);

//if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  
   // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

/* WHATS THIS FOR
  pinMode(22, OUTPUT);          
    digitalWrite(22, HIGH); 
  */
}

uint8_t data[] = "Success";
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void loop() {
  // This sketch displays information every time a new sentence is correctly encoded.

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

    if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
  }

  /* SERVER SIDE RF95
 Serial.println("Sending to rf95_reliable_datagram_server");
if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      // Send a reply back to the originator client
      if (!manager.sendtoWait(data, sizeof(data), from))
        Serial.println("sendtoWait failed");
    }
  }
  */



  /*      CLIENT SIDE RF95
  // Send a message to manager_server
  if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
      Serial.print("got reply from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("No reply, is rf95_reliable_datagram_server running?");
    }
  }
  else
    Serial.println("sendtoWait failed");
  delay(500);
  */

}



/* ENVISIONED FLOW PATTERN OF THIS CODE IN REAL LIFE ----------------------------------------------------------------------------------------------
1) Power on - GPS Signal Acquire
2) Establish radio communication on the ground with handheld controller - Handheld controller receives the green light
 * in this time the code will allow the drone to start up and then it will shut off the drone so that the motors believe they are in idle mode.
3) Wait for launch to complete - May lose communication halfway through flight as we exit our antennae range
4) Landed - Regained Communication between controller (Wait for command to be received)
5) Receive "Separate" command from handheld controller
6) Power Stepper motors from Position X to Position Y
7) Send "Success" Message to handheld controller for successful separation
8) Enable UAV Power for startup
9) Send "Success" Message for correct UAV Startup communication (radio established and GPS communication established)
7) Wait for command from handheld controller
8) Receive "LAUNCH" command from handheld controller
9) Move stepper motors from position X to position Y
10) Send "Launch" Command to UAV with position data of the Rocket
11) UAV takes off - This codes mission is ALMOST done.
12) Continue sending GPS coordinate and altitude data to UAV WHEN REQUESTED by the UAV. 
13) STOP sending data after UAV has landed and "STOP" command is given.
*/