#include <Arduino.h>
#include <adafruit_ST7789.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BMP3XX.h>
#include <AccelStepper.h>

// Initialize Adafruit ST7789 TFT library
Adafruit_ST7789 tft = Adafruit_ST7789(10, 6, 5);
Adafruit_GPS gps = Adafruit_GPS(&Serial1);
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();

AccelStepper motor1(1, 8, 9);
AccelStepper motor2(1, 14, 15);

void setup()
{
    // start cereal
    Serial.begin(115200);

    // set up screen
    tft.init(172, 320);
    tft.setRotation(3);

    tft.setCursor(0, 12);
    tft.setTextColor(0xffff, 0x0000);
    tft.setTextSize(2);
    tft.setTextWrap(true);

    tft.fillRect(0, 0, 320, 172, 0x0000);

    // start gps
    Serial1.begin(9600);

    // actually get data
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    
    // update every second
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 

    motor1.setAcceleration(20);
    motor1.setMaxSpeed(400);
    motor2.setAcceleration(20);
    motor2.setMaxSpeed(400);

    pinMode(22, OUTPUT);
    digitalWrite(22, HIGH);
}

int color = 0;

void loop()
{
    // clear screen
    tft.setCursor(0, 12);

    // read gps
    gps.read();

    if (gps.newNMEAreceived())
    {
        gps.parse(gps.lastNMEA());
    }

    if (gps.fix)
    {
        tft.fillRect(50, 50, 50, 50, 0x0f80);
    }
    else
    {
        tft.fillRect(50, 50, 50, 50, 0xf800);
    }

    tft.print("sad: ");
    tft.println(gps.satellites);

    tft.print("year: ");
    tft.println(gps.year);

    //bmp.readTemperature();

    //tft.print("bmp: ");
    //tft.println(bmp.temperature);
    
    motor1.setSpeed(200);
    motor1.runSpeed();

    motor2.setSpeed(200);
    motor2.runSpeed();

    //delay(1);
}
