#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP3XX.h>
//#define BMP_CS 6
//#define SEALEVELPRESSURE_HPA (1021.3)
//Adafruit_BMP3XX bmp; // bmp390
RH_RF95 rf95(10, 8); // Singleton instance of the radio driver
TinyGPSPlus gps;
bool position = false;
void setup()
{
    Serial.begin(9600);
    Serial1.begin(9600);
    if (!rf95.init())
    {
        Serial.println("init failed");
    }                           // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
   // if (!bmp.begin_SPI(BMP_CS)) // hardware SPI mode
   // {
   //     Serial.println("Could not find a valid BMP3 sensor, check wiring!");
   //     while (true)
   //         ;
   // }
    // Set up oversampling and filter initialization
   // bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
   // bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
   // bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
   // bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    // rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128); // You can change the modulation parameters with eg
    // rf95.setTxPower(20, false); //The default transmitter power is 13dBm, using PA_BOOST. - If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 2 to 20 dBm:
}

void loop()
{

 //   if (!bmp.performReading())
 //   {
  //      Serial.println("Failed to perform altimeter reading");
 //   }
    while (Serial1.available() > 0)
    {
        gps.encode(Serial1.read());
    }

    if (Serial.available())
    {
        char serialInput = Serial.read();

        if (strcmp(&serialInput, "k"))
        {
            //Serial.println("detected key k");
            uint8_t data[] = "k";
            rf95.send(data, sizeof(data));
            rf95.waitPacketSent();
            Serial.println("Sending to rf95_server");
        }
/*
        if (strcmp(&serialInput, "s") == 0)
        {
            uint8_t data[] = "separate";
            rf95.send(data, sizeof(data));
            rf95.waitPacketSent();
            Serial.println("Sending to rf95_server");
        }

        if (strcmp(&serialInput, "l") == 0)
        {
            uint8_t data[] = "launch";
            rf95.send(data, sizeof(data));
            rf95.waitPacketSent();
            Serial.println("Sending to rf95_server");
        }
    }

    if (rf95.available())
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) // if "separate" message detected
        {
            if (strcmp((char *)buf, "handpos") == 0)
            {
                position = true; // set separate to true
            }
            Serial.println((char *)buf);
        }
    }

    if (position == true)
    {
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

        position = false;
    }
*/
        Serial.println(serialInput);
        delay(400);
    }
}
