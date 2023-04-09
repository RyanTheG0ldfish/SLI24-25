#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPSPlus.h>

RH_RF95 rf95(10, 7); // Singleton instance of the radio driver

#define BMP_CS 6
#define SEALEVELPRESSURE_HPA (1021.3)

#define LSM_CS 9 // For SPI mode, we need a CS pin

Adafruit_BMP3XX bmp; // bmp390
Adafruit_LSM6DSO32 dso32;

Servo escFL;
Servo escFR;
Servo escBL;
Servo escBR;

int output = 1488;

uint32_t printTime;
uint32_t motorTime;
uint32_t armTime;
uint32_t lastTime;

TinyGPSPlus gps;

double altitudeSetpoint = 0.0;
double pitchSetpoint = 0.0;
double rollSetpoint = 0.0;

double targetLat = 0.0;
double targetLng = 0.0;
double targetHeading = 0.0;

double velocityX;
double velocityY;
double velocityZ;
double gyroPitch;
double gyroRoll;
double gyroYaw;
bool ReadySwitch = false;
double currentvalue = 0.00;
double maxvalue = 0.00;

enum MotorMode
{
    Arm,
    Disabled,
    Hold,
    NavigateHands,
    NavigateRocket,
    Land,
    Starting
};

MotorMode motorMode = Arm;

double rollSum;
double rollPrev;
double pitchSum = 0.0;
double pitchPrev = 0.0;
double yawSum;
double yawPrev;
double altSum;
double altPrev;

     double PUltimate = 0.0017; 
     double PPeriod = 0.8; 
//double pitchOutput = pidCalculate(gyroPitch, pitchSetpoint, (0.6*PUltimate), (1.2*(PUltimate/PPeriod)), ((3*PUltimate*PPeriod)/40), &pitchPrev, &pitchSum, 1, timeDiff);
    
     double RUltimate = 0.0022;
     double RPeriod = 0.50;
//double rollOuput   = pidCalculate(gyroRoll,  rollSetpoint,  (0.6*RUltimate), (1.2*(RUltimate/RPeriod)), ((3*RUltimate*RPeriod)/40), &rollPrev,  &rollSum,  1, timeDiff);

     double YUltimate = 0.0017;
     double YPeriod = 2.5;
//double yawOutput   = pidCalculate(gyroYaw,   0.0,   (0.6*YUltimate), (1.2*(YUltimate/YPeriod)), ((3*YUltimate*YPeriod)/40),    &yawPrev,   &yawSum,   1, timeDiff);

     double AUltimate = 0.0099;
     double APeriod = 0;

     double constanthovering = 0.40;


double pidCalculate(double input, double setpoint, double p, double i, double d, double* const &prevError, double* const &errorSum, double outputRange, double timeDiff)
{
    double error = setpoint - input;

    *errorSum += error * timeDiff;

    if (i != 0.0)
    { 
        double maxSum = outputRange * p / i;
        *errorSum = (*errorSum < -maxSum) ? -maxSum : (*errorSum > maxSum) ? maxSum : *errorSum;
    }

    double pTerm = p * error;
    double iTerm = i * *errorSum;
    double dTerm = d * (error - *prevError) * timeDiff;

    double output = pTerm + iTerm + dTerm;

    if (output < -outputRange && outputRange >= 0.0)
    {
        output = -outputRange;
    }

    if (output > outputRange && outputRange >= 0.0)
    {
        output = outputRange;
    }

    return output;
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
    Serial.print(" ft ");
    Serial.println(altitudeSetpoint * 3.28084);

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
    Serial.print("\tYaw: ");
    Serial.println(gyroYaw);

    Serial.println(gps.location.lat(), 8);
    Serial.println(gps.location.lng(), 8);
    Serial.println(gps.satellites.value());

    switch (motorMode)
    {
    case Arm:
        Serial.println(output);
        break;

    case Disabled:
        Serial.println("Disabled");
        break;

    case Hold:
        Serial.println("Hold");
        break;

    case NavigateHands:
        Serial.println("Navigate");
        break;

    case Land:
        Serial.println("Land");
        break;

    case Starting:
        Serial.println("Starting");
        break;

    default:
        break;
    }
}

void setMotor(Servo motor, double percentOutput)
{
    // 1488 - 1832
    percentOutput = (percentOutput > 1.0) ? 1.0 : (percentOutput < -1.0) ? -1.0
                                                                         : percentOutput;
    motor.writeMicroseconds((int)(1488 + percentOutput * 300) - (percentOutput < 0 ? 30 : 0));
}

void setMotor(Servo motor, double percentOutput, boolean invert)
{
    if (percentOutput < 0.0)
    {
        percentOutput = 0.0;
    }

    if (invert)
    {
        percentOutput *= -1;
    }

    setMotor(motor, percentOutput);
}

void setup()
{
    Serial.begin(9600);
    Serial.println("Initialized");

    Serial1.begin(9600);

    if (!rf95.init())
        Serial.println("Radio init failed");

    if (!bmp.begin_SPI(BMP_CS)) // hardware SPI mode
    {
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
        while (true)
            ;
    }

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    if (!dso32.begin_SPI(LSM_CS))
    {
        Serial.println("Failed to find LSM6DSO32 chip");
        while (true)
        {
        }
    }

    dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
    dso32.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    dso32.setAccelDataRate(LSM6DS_RATE_52_HZ);
    dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

    escFL.attach(4);
    escFR.attach(18);
    escBL.attach(2);
    escBR.attach(3);

    escFL.writeMicroseconds(output);
    escFR.writeMicroseconds(output);
    escBL.writeMicroseconds(output);
    escBR.writeMicroseconds(output);

    armTime = millis();
    printTime = micros();
    lastTime = micros();
}

void loop()
{
    uint32_t time = micros();
    double timeDiff = (double)(time - lastTime) / 1000000.0;
    lastTime = time;

    pitchSetpoint /= 1.008;
    rollSetpoint /= 1.008;

    if (rf95.available())
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) // if "separate" message detected
        {
            if (strcmp((char *)buf, "key:k") == 0)
            {
                if (motorMode != Arm && motorMode != Disabled)
                {
                    motorMode = Land;
                    currentvalue = 0;
                }
                else if (motorMode == Disabled && motorMode != Land)
                {
                    motorMode = Starting;
                }
            }

/* MotorModes
    Arm,
    Disabled,
    Hold,
    NavigateHands,
    NavigateRocket,
    Land,
    Starting
*/

//double altOutput   = pidCalculate(alt, altitudeSetpoint,    (0.6*AUltimate), (1.2*(AUltimate/APeriod)), ((3*AUltimate*APeriod)/40),    &altPrev,   &altSum,   0.0, timeDiff) + 0.565;

            if (strcmp((char *)buf, "key: ") == 0)
            {
                motorMode = Disabled;
                currentvalue = 0;
                maxvalue = 0;
            }

            if (strcmp((char *)buf, "key:r") == 0)
            {
               constanthovering += 0.001;
             uint8_t data[24];

        String Ahovering = String("Hover") + String(constanthovering, 8);
        for (int i = 0; i < Ahovering.length(); i++)
        {
            data[i] = Ahovering.charAt(i);
        }
        rf95.send(data, Ahovering.length());
            }            

            if (strcmp((char *)buf, "key:f") == 0)
            {
                constanthovering -= 0.001;
             uint8_t data[24];

        String Ahovering = String("Hover") + String(constanthovering, 8);
        for (int i = 0; i < Ahovering.length(); i++)
        {
            data[i] = Ahovering.charAt(i);
        }
        rf95.send(data, Ahovering.length());
            }   
            
            
            
            
            if (strcmp((char *)buf, "key:q") == 0)
            {
                altitudeSetpoint += 0.003048;
                  uint8_t data[24];

        String altitude = String("Altitude") + String((altitudeSetpoint*3.280839895), 8);
        for (int i = 0; i < altitude.length(); i++)
        {
            data[i] = altitude.charAt(i);
        }
        rf95.send(data, altitude.length());
            }

            if (strcmp((char *)buf, "key:a") == 0)
            {
                altitudeSetpoint -= 0.003048;
                  uint8_t data[24];

        String altitude = String("Altitude") + String((altitudeSetpoint*3.280839895), 8);
        for (int i = 0; i < altitude.length(); i++)
        {
            data[i] = altitude.charAt(i);
        }
        rf95.send(data, altitude.length());
            }

            if (strcmp((char *)buf, "key:w") == 0)
            {
                AUltimate += 0.00005;
                  uint8_t data[24];

        String Ultimate = String("Ultimate") + String(AUltimate, 8);
        for (int i = 0; i < Ultimate.length(); i++)
        {
            data[i] = Ultimate.charAt(i);
        }
        rf95.send(data, Ultimate.length());
            }

            if (strcmp((char *)buf, "key:s") == 0)
            {
                AUltimate -= 0.00005;
                  uint8_t data[24];

        String Ultimate = String("Ultimate") + String(AUltimate, 8);
        for (int i = 0; i < Ultimate.length(); i++)
        {
            data[i] = Ultimate.charAt(i);
        }
        rf95.send(data, Ultimate.length());
            }


            if (strcmp((char *)buf, "key:e") == 0)
            {
                APeriod += 0.05;
                  uint8_t data[24];

        String Period = String("Period") + String(APeriod, 8);
        for (int i = 0; i < Period.length(); i++)
        {
            data[i] = Period.charAt(i);
        }
        rf95.send(data, Period.length());
            }

            if (strcmp((char *)buf, "key:d") == 0)
            {
                APeriod -= 0.05;
             uint8_t data[24];

        String Period = String("Period") + String(APeriod, 8);
        for (int i = 0; i < Period.length(); i++)
        {
            data[i] = Period.charAt(i);
        }
        rf95.send(data, Period.length());
            }                     
        }
    }

    while (Serial1.available() > 0)
    {
        gps.encode(Serial1.read());
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

    gyroRoll = -atan(accel.acceleration.x / sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 1 / (3.142 / 180);
    gyroPitch = atan(accel.acceleration.y / sqrt(accel.acceleration.x * accel.acceleration.x + accel.acceleration.z * accel.acceleration.z)) * 1 / (3.142 / 180);
    gyroYaw += (gyro.gyro.z + 0.008) * timeDiff / (3.142 / 180);

    double alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    if (motorMode == Arm && millis() - armTime > 10000)
    {
        altitudeSetpoint = alt + 0.91; // meter
        motorMode = Disabled;
    }

    if (motorMode != Arm && accel.acceleration.z < 0.0)
    {
        motorMode = Disabled;

    }


    /*  Ziegler-Nichols PID Tuning Method
        U = P value when starting to go unstable (Oscillating)
        T = Oscillation period
        Equation : 
        P = 0.6 * U
        I = 1.2 * (U/T)
        D = (3*U*T) / 40

        PITCH U=0.0012 because P oscillated then during testing
        T = ~1 Second   
   
   
   
   //(0.6*AUltimate), (1.2*(AUltimate/APeriod)), ((3*AUltimate*APeriod)/40)
   
    */

    //                                current,   sp,            p,      i,        d,       prev,        sum,     range,  dt
    double pitchOutput = pidCalculate(gyroPitch, pitchSetpoint, (0.6*PUltimate), (1.2*(PUltimate/PPeriod)), ((3*PUltimate*PPeriod)/40), &pitchPrev, &pitchSum, 1, timeDiff);
    double rollOuput   = pidCalculate(gyroRoll,  rollSetpoint,  (0.6*RUltimate), (1.2*(RUltimate/RPeriod)), ((3*RUltimate*RPeriod)/40), &rollPrev,  &rollSum,  1, timeDiff);
    double yawOutput   = pidCalculate(gyroYaw,   0.0,   0, 0, 0,    &yawPrev,   &yawSum,   1, timeDiff);
    double altOutput   = pidCalculate(alt, altitudeSetpoint,  AUltimate, 0, 0,    &altPrev,   &altSum,   1, timeDiff) + constanthovering; //0.565 is hover speed -- 0.46 is sit on ground and spin speed
    double FLout = altOutput + rollOuput + pitchOutput + yawOutput;
    double FRout = altOutput - rollOuput + pitchOutput - yawOutput;
    double BLout = altOutput + rollOuput - pitchOutput - yawOutput;
    double BRout = altOutput - rollOuput - pitchOutput + yawOutput;
    double Wingup = 0.3;

  //  if ((alt >= (altitudeSetpoint + 0.5)) && motorMode == Hold) 
  //  {
  //    setMotor(escFL, 0.5, true);
  //    setMotor(escFR, 0.5, false);
  //    setMotor(escBL, 0.5, false);
  //    setMotor(escBR, 0.5, true);  
  //    Serial.println("DROPPING");
  //  }

    if ((motorMode == Hold) && (alt <= (altitudeSetpoint + 0.5)))
    { 
        //         IF PID greater than wingup true : false    && FLout < maxvalue
        setMotor(escFL, FLout > Wingup ? (FLout < maxvalue ? FLout : maxvalue) : Wingup, true);
        setMotor(escFR, FRout > Wingup ? (FRout < maxvalue ? FRout : maxvalue) : Wingup, false);
        setMotor(escBL, (BLout > Wingup ? (BLout < maxvalue ? BLout : maxvalue) : Wingup) / 1.391, false);
        setMotor(escBR, BRout > Wingup ? (BRout < maxvalue ? BRout : maxvalue) : Wingup, true);
        
        if (maxvalue <= 0.99)
        {
            maxvalue = (maxvalue + 0.002);
        }
        else
        {
            maxvalue = 1;
        }
    }

    
    if (motorMode == Starting)
    {
    Serial.println("Starting");
    Serial.println(currentvalue);
        if (currentvalue <= 0.29)
        {
            currentvalue = (currentvalue + 0.001);
        }
        else
        {
            currentvalue = 0.3;
            maxvalue = 0.3;
            motorMode = Hold;
        }
       setMotor(escFL, currentvalue, true);
       setMotor(escFR, currentvalue, false);
       setMotor(escBL, currentvalue / 1.391, false);
       setMotor(escBR, currentvalue, true); 
    }

    if (motorMode == Arm || motorMode == Disabled)
    {
        setMotor(escFL, 0.0);
        setMotor(escFR, 0.0);
        setMotor(escBL, 0.0);
        setMotor(escBR, 0.0);
    }

if (motorMode == Land)
{
    Serial.println("Landing");
    Serial.println(maxvalue);
//           IF PID greater than wingup true : false    && FLout < maxvalue
        setMotor(escFL, FLout > Wingup ? (FLout < maxvalue ? FLout : maxvalue) : (Wingup > maxvalue ? Wingup : maxvalue), true);
        setMotor(escFR, FRout > Wingup ? (FRout < maxvalue ? FRout : maxvalue) : (Wingup > maxvalue ? Wingup : maxvalue), false);
        setMotor(escBL, (BLout > Wingup ? (BLout < maxvalue ? BLout : maxvalue) : (Wingup > maxvalue ? Wingup : maxvalue)) / 1.391, false);
        setMotor(escBR, BRout > Wingup ? (BRout < maxvalue ? BRout : maxvalue) : (Wingup > maxvalue ? Wingup : maxvalue), true);
        
        if (maxvalue >= 0.24)
        {
            maxvalue = (maxvalue - 0.0005);
        }
        else
        {
            maxvalue = 0;
            motorMode = Disabled;
        }
}


/*   // Arm,
  //  Disabled,
    Hold,
    NavigateHands,
    NavigateRocket,
    Land,
   // Starting */



    if (time - printTime > 1000000)
    {
        printTime = time;
        Serial.print(timeDiff, 6);
        printData(temp, accel, gyro);
    }

    lastTime = time;
}