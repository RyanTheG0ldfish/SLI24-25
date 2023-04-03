#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPSPlus.h>

RH_RF95 rf95(10, 8); // Singleton instance of the radio driver
TinyGPSPlus gps;

bool position = false;

void setup() 
{
  Serial.begin(9600);
  if (!rf95.init()) // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  Serial.println("init failed");
  Serial1.begin(9600);

  //rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128); // You can change the modulation parameters with eg
  //rf95.setTxPower(20, false); //The default transmitter power is 13dBm, using PA_BOOST. - If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 2 to 20 dBm:
}

void loop()
{
  char serialInput = Serial.read();
  Serial.println("waiting");
  
  while(Serial1.available() > 0) 
  {
  gps.encode(Serial1.read());
  } 

{
 if (strcmp(&serialInput, "k") == 0)
     {
  uint8_t data[] = "k";
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
  Serial.println("Sending to rf95_server");
     }

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
/*
 if (strcmp(&serialInput, "g") == 0)
     {
  uint8_t data[] = "g";
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
  Serial.println("Sending to rf95_server");
    }
    */
   /*
 if (strcmp(&serialInput, "w") == 0)
     {
  uint8_t data[] = "w";
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
  Serial.println("Sending to rf95_server");
     }
  */
 }

if(rf95.available())
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) // if "separate" message detected
        {
            if (strcmp((char*)buf, "handpos") == 0)
            {
            position = true;  // set separate to true
            }          
            Serial.println((char*)buf);
        }
    }

if(position == true)
{
uint8_t data[24];
       String lat = String(gps.location.lat(), 8);

        for (int i = 0; i < lat.length(); i++)
        {
            data[i] = lat.charAt(i);
        }
        rf95.send(data, lat.length());
        
        delay(5);

        String lng = String(gps.location.lng(), 8);
        for (int i = 0; i < lng.length(); i++)
        {
            data[i] = lng.charAt(i);
        }
        rf95.send(data, lng.length());
    
        delay(5);

        //Send  Altitude from bmp3xx - still missing
    
    }

Serial.println(serialInput);

delay(400);
}