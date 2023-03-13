#include <Arduino.h>
#include <adafruit_ST7789.h>
#include <Adafruit_GFX.h>
//#include <Adafruit_GPS.h>
#include <Adafruit_BMP3XX.h>
#include <AccelStepper.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_Sensor.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

static const int RXPin = 0, TXPin = 1;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Singleton instance of the radio driver
RH_RF95 driver(3, 20);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// Initialize Adafruit ST7789 TFT library
Adafruit_ST7789 tft = Adafruit_ST7789(10, 6, 5);
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 4

#define SEALEVELPRESSURE_HPA (1013.25)

AccelStepper motor1(1, 8, 9);
AccelStepper motor2(1, 14, 15);

void setup()
{
  Serial.begin(115200);
    ss.begin(GPSBaud);

Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();

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

    // set up screen
   tft.init(172, 320);
   tft.setRotation(3);

   tft.setCursor(0, 12);
   tft.setTextColor(0xffff, 0x0000);
tft.setTextSize(2);
   tft.setTextWrap(true);

   tft.fillRect(0, 0, 320, 172, 0x0000);

   motor1.setAcceleration(20);
   motor1.setMaxSpeed(400);
    motor2.setAcceleration(20);
    motor2.setMaxSpeed(400);

    pinMode(22, OUTPUT);
    digitalWrite(22, HIGH);
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());

    Serial.print("Temperature = ");
  Serial.print((bmp.temperature * 1.8) + 32);
  Serial.println(" *F");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  }
  else
  {
    Serial.print(F("INVALID"));
  }




// TFT STUFF
    tft.setCursor(0, 12); //clear screen
  tft.println(F("Location: ")); 
  if (gps.location.isValid())
  {
    tft.print(gps.location.lat(), 6);
    tft.print(F(" N , "));
    tft.print(gps.location.lng(), 6);
    tft.println(F(" W"));
  }
  else
  {
    tft.print(F("INVALID"));
  }

  tft.println(F("Date/Time: "));
  if (gps.date.isValid())
  {
    tft.print(gps.date.month());
    tft.print(F("/"));
    tft.print(gps.date.day());
    tft.print(F("/"));
    tft.print(gps.date.year());
  }
  else
  {
    tft.print(F("INVALID"));
  }

  tft.print(F(" "));  
if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) tft.print(F("0"));
    tft.print(gps.time.hour());
    tft.print(F(":"));
    if (gps.time.minute() < 10) tft.print(F("0"));
   tft.print(gps.time.minute());
   tft.print(F(":"));
    if (gps.time.second() < 10) tft.print(F("0"));
    tft.print(gps.time.second());
    tft.print(F("."));
    if (gps.time.centisecond() < 10) tft.print(F("0"));
    tft.println(gps.time.centisecond());



    tft.print("Temperature = ");
  tft.print((bmp.temperature * 1.8) + 32);
  tft.println(" *F");

  tft.print("Pressure = ");
  tft.print(bmp.pressure / 100.0);
  tft.println(" hPa");

  tft.print("Approx. Altitude = ");
  tft.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  tft.println(" m");

  tft.println();
  }
  else
  {
    tft.print(F("INVALID"));
  }

  Serial.println();
}

uint8_t data[] = "Hello World!";
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void loop()
{

// This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
    motor1.setSpeed(200);
    motor1.runSpeed();

    motor2.setSpeed(200);
    motor2.runSpeed();

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




/* Stepper Motor Control Information


Position Based Control
mystepper.moveTo(targetPosition);
Move the motor to a new absolute position. This returns immediately. Actual movement is caused by the run() function.

mystepper.move(distance);
Move the motor (either positive or negative) relative to its current position. This returns immediately. Actual movement is caused by the run() function.

mystepper.currentPosition();
Read the motor's current absolution position.

mystepper.distanceToGo();
Read the distance the motor is from its destination position. This can be used to check if the motor has reached its final position.

mystepper.run();
Update the motor. This must be called repetitively to make the motor move.

mystepper.runToPosition();
Update the motor, and wait for it to reach its destination. This function does not return until the motor is stopped, so it is only useful if no other motors are moving.




Speed Based Control
mystepper.setSpeed(stepsPerSecond);
Set the speed, in steps per second. This function returns immediately. Actual motion is caused by called runSpeed().

mystepper.runSpeed();
Update the motor. This must be called repetitively to make the motor move.

*/