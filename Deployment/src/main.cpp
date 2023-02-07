#include <Arduino.h>
#include <adafruit_ST7789.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BMP3XX.h>

// Initialize Adafruit ST7789 TFT library
Adafruit_ST7789 tft = Adafruit_ST7789(10, 6, 5);
Adafruit_GPS gps = Adafruit_GPS(&Serial1);
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();

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

    if (!bmp.begin_I2C(0x77, &Wire))
    {
        tft.println("no bmp :(");
        //while (1) {}
    }

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
}

int color = 0;

void loop()
{
    // clear screen
    tft.setCursor(0, 12);

    // read gps
    char c = gps.read();

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
    
    delay(10);
}