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

TinyGPSPlus gps;                         // GPS Driver
RH_RF95 rf95(3, 2);                      // Singleton instance of the radio driver
Adafruit_BMP3XX bmp = Adafruit_BMP3XX(); // Initializes the bmp3xx driver
#define BMP_CS 4                         // Chip Select Pin
#define SEALEVELPRESSURE_HPA (1013.25)   // Sea Level Pressure

// Stepper Motors
AccelStepper motor1(AccelStepper::DRIVER, 8, 9); //8 = pinStep and 9 = pinDirection
AccelStepper motor2(AccelStepper::DRIVER, 14, 15);

Servo servo1; // create servo object #1 to control servo 1
Servo servo2; // create servo object #2 to control servo 2

float currentmillis = 0; // Variable used to run different functions when a command is given
bool separate = false;   // Variable used to run different functions when a command is given

void sendposition()
{
    Serial.println("Postrue");
    uint8_t data[24];

    String lat = String("hlat") + String(gps.location.lat(), 8);

    for (int i = 0; i < lat.length(); i++)
    {
        data[i] = lat.charAt(i);
    }
    rf95.send(data, lat.length());

    delay(5);

    String lng = String("hlng") + String(gps.location.lng(), 8);
    for (int i = 0; i < lng.length(); i++)
    {
        data[i] = lng.charAt(i);
    }
    rf95.send(data, lng.length());

    delay(5);

    String alt = String("halt") + String(bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084, 8);
    for (int i = 0; i < lng.length(); i++)
    {
        data[i] = alt.charAt(i);
    }
    rf95.send(data, alt.length());

    delay(5);
}

void setup()
{
    // Serial
    Serial.begin(9600);
    Serial1.begin(9600); // GPS

    // Radio
    if (!rf95.init())
        Serial.println("Radio init failed"); // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on...The default transmitter power is 13dBm, using PA_BOOST.
    // rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128); // You can change the modulation parameters with eg
    // rf95.setTxPower(20, false); //The default transmitter power is 13dBm, using PA_BOOST. - If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 2 to 20 dBm:

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
    motor1.setMaxSpeed(4000);
    motor1.setAcceleration(100);
    motor2.setMaxSpeed(4000);
    motor2.setAcceleration(100);

    pinMode(6, OUTPUT); // M0 - Setting the step pin to high to initialize full step control
    digitalWrite(6, LOW);
    pinMode(7, OUTPUT); // M1 - Setting the step pin to high to initialize full step control
    digitalWrite(7, LOW);

    // Servo Stuff
    servo1.attach(18); // attaches the servo on pin 9 to the servo object
    servo2.attach(19); // attaches the servo on pin 9 to the servo object
    servo1.write(180); // Set Servos - Values are from 0 to 180
    servo2.write(0);   // Set Servos - Values are from 0 to 180

    motor1.moveTo(-100000);
    motor2.moveTo(-100000);
}

void loop()
{
    while (Serial1.available() > 0)
    {
        gps.encode(Serial1.read());
    }

    bmp.performReading(); // Altimeter

    if (rf95.available())
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) // if "separate" message detected
        {
            if (strcmp((char *)buf, "separate") == 0)
            {
                motor1.moveTo(1000);
                motor2.moveTo(2000);
                separate = true;          // set separate to true
                currentmillis = millis(); // set a counter to current time (0)
            }
            if (strcmp((char *)buf, "rocketpos") == 0)
            {
                sendposition();
            }
            Serial.println((char *)buf);
        }
    }

    if (separate == true) // if separating
    {
        if ((motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) && ((millis() - currentmillis) >= 10000)) // Checks if Stepper Motors & Time are at their respective endpoints
        {
            separate = false;  // Stops this function from running
            servo1.write(0);   // Set Servos - Values are from 0 to 180
            servo2.write(180); // Set Servos - Values are from 0 to 180
        }
    }

    motor1.run(); // run the stepper
    motor2.run(); // run the stepper
}