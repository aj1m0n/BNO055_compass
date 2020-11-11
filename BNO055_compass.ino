#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <tuple>


uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
double yaw = 0;
int i = 0;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

std::tuple<double, double, double> compass(double _roll, double _pitch, double _mgx, double _mgy, double _mgz)
{
    double Dx, Dy, Dz;
    Dx = cos(_pitch) * _mgx + sin(_pitch) * _mgy + sin(_pitch) * cos(_roll) * _mgz;
    Dy = cos(_roll) * _mgy - sin(_roll) * _mgz;
    Dz = sin(_pitch) *_mgx + cos(_pitch) * sin(_roll) * _mgy + cos(_pitch) * cos(_roll) * _mgz;
    return std::forward_as_tuple(Dx, Dy, Dz);
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Orientation Sensor Test");
    Serial.println("");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    delay(1000);
}

void loop(void)
{
    int8_t boardTemp = bno.getTemp();
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    double ax,ay, az, mx, my, mz, Dx, Dy, Dz;

    ax = accelerometerData.acceleration.x;
    ay = accelerometerData.acceleration.y;
    az = accelerometerData.acceleration.z;

    //magnetic
    mx = magnetometerData.magnetic.x;
    mx = mx - (-0.71934013);
    my = magnetometerData.magnetic.y;
    my = my - (-0.42655574);
    mz = magnetometerData.magnetic.z;
    mz = mz - (1.44813255);

    std::tie(Dx, Dy, Dz) = compass(roll(ay,az), pitch(ax, ay, az), mx, my, mz);
    
    yaw = atan2(-Dy,Dx) * 180 / M_PI + yaw;

    if(i == 10){
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");
    Serial.println(yaw/10);
    i = 0;
    yaw = 0;
    }
    i++;
}

double roll(double _ay, double _az)
{
    return atan2(_ay, _az);
}

double pitch(double _ax, double _ay, double _az)
{
    return atan2(_ax, sqrt(pow(_ay, 2) + pow(_az, 2)));
}
