#include <Arduino.h>
#include <adafruit_ST7789.h>
#include <Adafruit_GFX.h>
#include <Adafruit_BMP3XX.h>
#include <AccelStepper.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <Servo.h>

// GPS
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
static const int RXPin = 0, TXPin = 1;
SoftwareSerial ss(RXPin, TXPin);

// Radio
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
RH_RF95 rf95(3, 20);                              // Singleton instance of the radio driver
//RHReliableDatagram manager(driver, CLIENT_ADDRESS); // Class to manage message delivery and receipt, using the driver declared above

// Display
Adafruit_ST7789 tft = Adafruit_ST7789(10, 6, 5); // Initialize Adafruit ST7789 TFT library

// BMP 3XX
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 4
#define SEALEVELPRESSURE_HPA (1013.25)

// Stepper Motors
AccelStepper motor1(1, 8, 9);
AccelStepper motor2(1, 14, 15);

// Untested Servo Control
Servo servo1; // create servo object #1 to control servo 1
Servo servo2; // create servo object #2 to control servo 2

bool separate = false;

void setup()
{
    // Serial
    Serial.begin(115200);

    // GPS
    ss.begin(GPSBaud);

    // Radio
    if (!rf95.init())
        Serial.println("Radio init failed"); // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on...The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 2 to 20 dBm:
    //  driver.setTxPower(20, false);
    // You can optionally require this module to wait until Channel Activity Detection shows no activity on the channel before transmitting by setting the CAD timeout to non-zero:
    //  driver.setCADTimeout(10000);

    // BMP 3XX
    if (!bmp.begin_SPI(BMP_CS))
    { // hardware SPI mode
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X); // Set up oversampling and filter initialization
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Display
    tft.init(172, 320);
    tft.setRotation(3);
    tft.setCursor(0, 12);
    tft.setTextColor(0xffff, 0x0000);
    tft.setTextSize(2);
    tft.setTextWrap(true);
    tft.fillRect(0, 0, 320, 172, 0x0000);

    // Stepper Motors
    motor1.setAcceleration(20);
    motor1.setMaxSpeed(400);
    motor2.setAcceleration(20);
    motor2.setMaxSpeed(400);
    pinMode(22, OUTPUT);
    digitalWrite(22, HIGH);

    // Untested Servo Stuff
    // servo1.attach(9);  // attaches the servo on pin 9 to the servo object
    // servo2.attach(5); // attaches the servo on pin 9 to the servo object
}

void Separate()
{
    /*
    5) Receive "Separate" command from handheld controller
    6) Power Stepper motors from Position X to Position Y

      //Stepper Motors
      motor1.setSpeed(200);
      motor1.runSpeed();
      motor2.setSpeed(200);
      motor2.runSpeed();

    7) Send "Success" Message to handheld controller for successful separation

    9) Send "Success" Message for correct UAV Startup communication (radio established and GPS communication established)

    7) Wait for command from handheld controller
    */
}

void Launch()
{
    /*
    8) Receive "LAUNCH" command from handheld controller

    9) Move servo motors from position X to position Y
        servo1.write(180); //values are from 0 to 180
        servo2.write(180); //values for position are from 0 to 180
    10) Send "Launch" Command to UAV with position data of the Rocket

    */
}

void displayInfo()
{
    // GPS
    Serial.print(F("Location: "));
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
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

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    }

    // BMP 3XX
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

    // DISPLAY
    tft.setCursor(0, 12); // clear screen
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
        if (gps.time.hour() < 10)
            tft.print(F("0"));
        tft.print(gps.time.hour());
        tft.print(F(":"));
        if (gps.time.minute() < 10)
            tft.print(F("0"));
        tft.print(gps.time.minute());
        tft.print(F(":"));
        if (gps.time.second() < 10)
            tft.print(F("0"));
        tft.print(gps.time.second());
        tft.print(F("."));
        if (gps.time.centisecond() < 10)
            tft.print(F("0"));
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

        Serial.println();
    }
}


void loop()
{

    // This sketch displays information every time a new sentence is correctly encoded.
    while (ss.available() > 0)
        if (gps.encode(ss.read()))
            displayInfo();

    // BMP 3XX
    if (!bmp.performReading())
    {
        Serial.println("Failed to perform reading :(");
    }

    if (rf95.available())
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len))
        {
            Serial.println((char*)buf);
            uint8_t data[] = "reply";
            rf95.send(data, sizeof(data));
        }
    }
    

    // if "separate" message detected
        // set separate to true
        // set a counter to current time (0)
        // set stepper target position
    
    // if separating
        // check the time to the start (done?)
        // check stepper position (done?)

        // if done
            // set servos
            // send done message

    // if "get position" message
        // send current reading over radio (include gps position, altitude)

    // run the stepper
        

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

/* ENVISIONED FLOW PATTERN OF THIS CODE IN REAL LIFE ----------------------------------------------------------------------------------------------
2) Establish radio communication on the ground with handheld controller - Handheld controller receives the green light
3) Wait for launch to complete - May lose communication halfway through flight as we exit our antennae range
4) Landed - Regained Communication between controller (Wait for command to be received)
5) Receive "Separate" command from handheld controller
6) Power Stepper motors from Position X to Position Y
7) Send "Success" Message to handheld controller for successful separation
9) Send "Success" Message for correct UAV Startup communication (radio established and GPS communication established)
7) Wait for command from handheld controller
8) Receive "LAUNCH" command from handheld controller
9) Move servo motors from position X to position Y
10) Send "Launch" Command to UAV with position data of the Rocket
11) UAV takes off - This codes mission is ALMOST done.
12) Continue sending GPS coordinate and altitude data to UAV WHEN REQUESTED by the UAV.
13) STOP sending data after UAV has landed and "STOP" command is given.
*/