#include <Arduino.h>
#include <adafruit_ST7789.h>
#include <Adafruit_GFX.h>

// Initialize Adafruit ST7789 TFT library
Adafruit_ST7789 tft = Adafruit_ST7789(10, 6, 5);

void setup()
{
    // put your setup code here, to run once:
    tft.init(172, 320);
    tft.setRotation(3);

    tft.setCursor(0, 12);
    tft.setTextColor(0xffff, 0x0000);
    tft.setTextSize(2);
    tft.setTextWrap(true);

    tft.fillRect(0, 0, 320, 172, 0x0000);

    tft.println("Dislpay Init");
    tft.println("1");
    tft.println("2");
    tft.println("3");
    tft.println("4");
    tft.println("5");
    tft.println("6");
    tft.println("7");
    tft.println("8");
    tft.println("9");
    
}

void loop()
{
    // put your main code here, to run repeatedly:
}