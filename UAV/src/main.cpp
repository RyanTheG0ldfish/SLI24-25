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

uint32_t printTime = 0;
uint32_t motorTime = 0;
uint32_t armTime = 0;
uint32_t lastTime = 0;

TinyGPSPlus gps;

double altitudeSetpoint = 0.0;
double pitchSetpoint = 0.0;

double targetLat = 0.0;
double targetLng = 0.0;
double targetHeading = 0.0;

double velocityX;
double velocityY;
double velocityZ;
double gyroPitch;
double gyroRoll;
double gyroYaw;

enum MotorMode
{
    Arm,
    Disabled,
    Hold,
    NavigateHands,
    NavigateRocket,
    Land
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

double pidCalculate(double input, double setpoint, double p, double i, double d, double *const &prevError, double *const &errorSum, double timeDiff)
{
    double error = setpoint - input;

    *errorSum += error * timeDiff;

    double pTerm = p * error;
    double iTerm = i * *errorSum;
    double dTerm = d * (error - *prevError) * timeDiff;

    return pTerm + iTerm + dTerm;
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
    Serial.begin(57600);
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

    uint32_t time = millis();

    lastTime = time;
    armTime = time;
    printTime = time;
}

void loop()
{
    uint32_t time = micros();
    double timeDiff = (double)(time - lastTime) / 1000000.0;

    if (rf95.available())
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) // if "separate" message detected
        {
            if (strcmp((char *)buf, "k") == 0)
            {
                if (motorMode != Arm && motorMode != Disabled)
                {
                    motorMode = Disabled;
                }
                else if (motorMode == Disabled)
                {
                    motorMode = Hold;
                }
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
        altitudeSetpoint = alt + 0.2; // meter
        motorMode = Disabled;
    }

    if ((motorMode == NavigateHands || motorMode == NavigateRocket) && abs(gyroYaw) < 90)
    {
        pitchSetpoint = 0.5 * cos(gyroYaw * 3.14 / 180);
    }
    else
    {
        pitchSetpoint = 0.0;
    }

    double pitchOutput = pidCalculate(gyroPitch, pitchSetpoint, 0.003, 0.00, 0.00000, &pitchPrev, &pitchSum, timeDiff); // Setpoint, P, I, D
    double rollOuput = pidCalculate(gyroRoll, 0.0, 0.0025, 0.0, 0.00, &rollPrev, &rollSum, timeDiff);
    double yawOutput = pidCalculate(gyroYaw, 0.0, 0.000, 0.0, 0.0, &yawPrev, &yawSum, timeDiff);
    double altOutput = pidCalculate(alt, altitudeSetpoint, 0.000, 0.0, 0.0, &altPrev, &altSum, timeDiff) + 0.50;

    if (motorMode != Arm && motorMode != Disabled)
    {
        double Hover = 0.4;
        double FLout = altOutput + rollOuput + pitchOutput + yawOutput;
        double FRout = altOutput - rollOuput + pitchOutput - yawOutput;
        double BLout = altOutput + rollOuput - pitchOutput - yawOutput;
        double BRout = altOutput - rollOuput - pitchOutput + yawOutput;

        setMotor(escFL, FLout > Hover ? FLout : Hover, true);
        setMotor(escFR, FRout > Hover ? FRout : Hover, false);
        setMotor(escBL, BLout > Hover ? BLout : Hover, false);
        setMotor(escBR, BRout > Hover ? BRout : Hover, true);
    }
    else
    {
        setMotor(escFL, 0.0);
        setMotor(escFR, 0.0);
        setMotor(escBL, 0.0);
        setMotor(escBR, 0.0);
    }

    if (time - printTime > 1000000)
    {
        printTime = time;
        printData(temp, accel, gyro);
    }

    lastTime = time;
}