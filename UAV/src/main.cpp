#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <Adafruit_LSM6DSO32.h>
#include <servo.h>

#define LSM_CS 9  // For SPI mode, we need a CS pin
Adafruit_LSM6DSO32 dso32;

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

Servo escFL;
Servo escFR;
Servo escBL;
Servo escBR;

int output = 1488;

uint32_t printTime = 0;
uint32_t motorTime = 0;
uint32_t armTime = 0;
uint32_t lastTime = 0;

double velocityX;
double velocityY;
double velocityZ;
double gyroPitch;
double gyroRoll;
double gyroYaw;

enum MotorMode
{
    Arm,
    Operate,
};

MotorMode motorMode = Arm;


double pidCalculate(double input, double setpoint, double p, double i, double d, double &prevError, double &errorSum)
{
    double error = setpoint - input;
}


void updateGyro()
{

}

void printData(sensors_event_t temp, sensors_event_t accel, sensors_event_t gyro)
{
        Serial.println();
        Serial.print("Temp: ");
        Serial.print((bmp.temperature * 1.8) + 32);
        Serial.println(" F");
        Serial.print("Pres.: ");
        Serial.print(bmp.pressure / 100.0);
        Serial.println(" hPa");
        Serial.print("Alt: ");
        Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084);
        Serial.println(" ft");

        Serial.print("Temperature ");
        Serial.print((temp.temperature) * 1.8 + 32);
        Serial.println(" F");

        /* Display the results (acceleration is measured in m/s^2) */
        Serial.print("Accel X: ");
        Serial.print(accel.acceleration.x);
        Serial.print("\tY: ");
        Serial.print(accel.acceleration.y);
        Serial.print("\tZ: ");
        Serial.print(accel.acceleration.z);
        Serial.println(" m/s^2");

        /* Display the results (rotation is measured in rad/s) */
        Serial.print("Gyro X: ");
        Serial.print(gyro.gyro.x);
        Serial.print("\tY: ");
        Serial.print(gyro.gyro.y);
        Serial.print("\tZ: ");
        Serial.print(gyro.gyro.z);
        Serial.println(" radians/s ");

        Serial.print("Pitch: ");
        Serial.print(gyroPitch);
        Serial.print("\tRoll: ");
        Serial.println(gyroRoll);

        Serial.println(output);
}

void setMotor(Servo motor, double percentOutput)
{

}

void setup() {
Serial.begin(115200); //serial for everything

ss.begin(GPSBaud); //Intiialize GPS

if (! bmp.begin_SPI(BMP_CS)) {  // Initialize BMP3XX
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
  // Set up oversampling and filter initialization -- Initialize BMP3XX
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

if (!dso32.begin_SPI(LSM_CS)) { //Initialize Gyro
     Serial.println("Failed to find LSM6DSO32 chip");
  }
  // Set up Ranges for Gyro
  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  dso32.setAccelDataRate(LSM6DS_RATE_52_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

  //Custom Motor Control Stuff
  escFL.attach(2);
  escFR.attach(3);
  escBL.attach(4);
  escBR.attach(5);

  //More Custom Motor Control Stuff
  escFL.writeMicroseconds(output);
  escFR.writeMicroseconds(output);
  escBL.writeMicroseconds(output);
  escBR.writeMicroseconds(output);

  //More of Ethan's Stuff
  uint32_t time = millis();
  lastTime = time;
  armTime = time;
  printTime = time;


if (!manager.init())  //Initialize Radio
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

}

//This is for the radio
uint8_t data[] = "Success";
// Dont put this on the stack: (Also for Radio)
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void loop() {
  // This currently displays information every time a new sentence is correctly encoded.
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
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

 uint32_t time = micros();

    if (Serial.available())
    {
        char serialInput = Serial.read();
        if (strcmp(&serialInput, "s") == 0)
        {
            output = 1488;
        }

        if (strcmp(&serialInput, "a") == 0)
        {
            output -= 1;
        }

        if (strcmp(&serialInput, "d") == 0)
        {
            output += 1;
        }
    }
    
    if (!bmp.performReading())
    {
        Serial.println("Failed to perform altimeter reading");
    }

    //  /* Get a new normalized sensor event */
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    if (!dso32.getEvent(&accel, &gyro, &temp))
    {
        Serial.println("Failed to perform gyro reading");
    }

    gyroRoll=-atan(accel.acceleration.x/sqrt(accel.acceleration.y*accel.acceleration.y+accel.acceleration.z*accel.acceleration.z))*1/(3.142/180);
    gyroPitch=atan(accel.acceleration.y/sqrt(accel.acceleration.x*accel.acceleration.x+accel.acceleration.z*accel.acceleration.z))*1/(3.142/180);

    if (time - motorTime > 10000)
    {
        motorTime = time;

        if (millis() - armTime > 10000)
        {
            motorMode = Operate;
        }
    }

    escFL.writeMicroseconds(output);
    escFR.writeMicroseconds(output);
    escBL.writeMicroseconds(output);
    escBR.writeMicroseconds(output);
    
    if (time - printTime > 1000000)
    {
        printTime = time;
        printData(temp, accel, gyro);
    }

    lastTime = time;

}

/* ENVISIONED FLOW PATTERN OF THIS UAV CODE IN REAL LIFE ----------------------------------------------------------------------------------------------
1) Power on - Set motors to 0 - power off (getting powered off by other board)
2) Power on once landed (done by other board
3) Receive data communication from other board - either wired or wirelessly
  - This is GPS, altitude, etc. data.
4) Get command to launch from other board
  - Launch once all systems are ready to go
5) Level off @ 10 feet off the ground using garmin lidar
6) Use GPS heading information to geolocate where to go to handheld controller
7) Using GPS, subtract the two points to get a target location to fly to - and fly to it.
8) Once uav reaches handheld controller - wait for handheld controller to give signal to go back.
9) Target the location of the Deployment GPS, maintaining X distance away from the handheld controller
10) Go to X feet to the right of the rocket and hover
11) land once land command is given
*/