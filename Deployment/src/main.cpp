#include <Arduino.h>
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
static const uint32_t GPSBaud = 9600;   //GPS Baud rate
TinyGPSPlus gps;   //GPS Driver
static const int RXPin = 0, TXPin = 1;  //This defines the pins that are being used for RX and TX (UART)
SoftwareSerial ss(RXPin, TXPin);    //Shouldn't need to be using Software Serial - am on hardwhere serial pins.

// Radio
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
RH_RF95 rf95(3, 2);        // Singleton instance of the radio driver

// BMP 3XX
Adafruit_BMP3XX bmp = Adafruit_BMP3XX(); //Initializes the driver
#define BMP_SCK 13      //Clock Pin
#define BMP_MISO 12     //MISO (Motherboard In - Signal Out)
#define BMP_MOSI 11     //MOSI (Motherboard Out - Signal In)
#define BMP_CS 4        //Chip Select Pin
#define SEALEVELPRESSURE_HPA (1013.25)  //Sea Level Pressure

// Stepper Motors
AccelStepper motor1(1, 8, 9);         //(1, 8, 9) where 1 = driver board, 8 = pinStep, and 9 = pinDirection
AccelStepper motor2(1, 14, 15);       //Defining related pins for the motor driver

// Untested Servo Control
Servo servo1; // create servo object #1 to control servo 1
Servo servo2; // create servo object #2 to control servo 2

float currentmillis = 0;    //Variable used to run different functions when a command is given
bool separate = false; //Variable used to run different functions when a command is given
bool part2 = false; //Variable used to run different functions when a command is given
bool position = false;  //Variable used to run different functions when a command is given

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
    if (!bmp.begin_SPI(BMP_CS)) // hardware SPI mode
    { 
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X); // Set up oversampling and filter initialization
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Stepper Motors
  motor1.setMaxSpeed(3000);
  motor1.setAcceleration(100);
  motor2.setMaxSpeed(3000);
  motor2.setAcceleration(100);

  pinMode(6, OUTPUT); //M0 - Setting the step pin to high to initialize full step control
  digitalWrite(6, LOW);
  pinMode(7, OUTPUT); //M1 - Setting the step pin to high to initialize full step control
  digitalWrite(7, LOW);

  //Servo Stuff
  servo1.attach(18);   // attaches the servo on pin 9 to the servo object
  servo2.attach(19);   // attaches the servo on pin 9 to the servo object
  servo1.write(180); // Set Servos - Values are from 0 to 180
  servo2.write(180); // Set Servos - Values are from 0 to 180
}


void loop()
{
    while (ss.available() > 0) { //GPS
      gps.encode(ss.read());  }
    
    bmp.performReading(); //Altimeter

    if (rf95.available())
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) // if "separate" message detected
        {
            Serial.println((char*)buf);
            uint8_t data[] = "Received"; //sending a reply
            rf95.send(data, sizeof(data));

            if ((char*)buf == "Separate")
            {
                separate = true;  // set separate to true
                currentmillis = millis(); // set a counter to current time (0)
                motor1.moveTo(1000);
                motor2.moveTo(2000);
            }

            if ((char*)buf == "Position")
            {
                position = true;  // set position to true (The Drone wants your position)
            }
        }
    }
   
    if(separate == true) // if separating
    {             
        if((motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) && ((millis()-currentmillis) >= 10000))  // Checks if Stepper Motors & Time are at their respective endpoints
            {
             
             separate == false; //Stops this function from running
             part2 == true;     //Makes a different function start running    
            }  
    }

    if(part2 == true) // if separation is done
    {
        servo1.write(0); // Set Servos - Values are from 0 to 180
        servo2.write(0);  //Set Servos - Values are from 0 to 180
        uint8_t data[] = "Separated"; // Prepare Done Message
        rf95.send(data, sizeof(data));  //Send Done Message
        part2 == false; //stops this command from running again
    }

    if(position == true)  // if "get position" message is received
    {
      Serial.println("Postrue");
      uint8_t data[24];
       String lat = String(gps.location.lat(), 8);

        for (int i = 0; i < lat.length(); i++)
        {
            data[i] = lat.charAt(i);
        }
        rf95.send(data, lat.length());
        
        delay(500);

        String lng = String(gps.location.lng(), 8);
        for (int i = 0; i < lng.length(); i++)
        {
            data[i] = lng.charAt(i);
        }
        rf95.send(data, lng.length());
    }
  
  motor1.run(); //run the stepper
  motor2.run(); //run the stepper
}

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