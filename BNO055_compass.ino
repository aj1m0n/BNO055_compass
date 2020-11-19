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
    Dz = sin(_pitch) * _mgx + cos(_pitch) * sin(_roll) * _mgy + cos(_pitch) * cos(_roll) * _mgz;
    return std::forward_as_tuple(Dx, Dy, Dz);
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Orientation Sensor Test");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }
    std::tie(mgx, mgy, mgz) = spherical_fitting();
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

    double ax, ay, az, mx, my, mz, Dx, Dy, Dz;

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

    std::tie(Dx, Dy, Dz) = compass(roll(ay, az), pitch(ax, ay, az), mx, my, mz);

    yaw = atan2(-Dy, Dx) * 180 / M_PI + yaw;

    if (i == 10)
    {
        Serial.print(ax);
        Serial.print(",");
        Serial.print(ay);
        Serial.print(",");
        Serial.print(az);
        Serial.print(",");
        Serial.println(yaw / 10);
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

std::tuple<float, float, float> spherical_fitting()
{
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    int num_len = 360 * 3;
    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    float mxl[360*3] = {9999}, myl[360*3] = {9999}, mzl[360*3] = {9999};
    int cntx = 0, cnty = 0, cntz = 0;
    while (1)
    {
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

        int degx = orientationData.orientation.x;
        int degy = orientationData.orientation.y;
        if (degy < 0)
        {
            degy = 90 - degy;
        }
        int degz = orientationData.orientation.z;
        if (degz < 0)
        {
            degy = 180 + degy;
            degz = 180 - degz;
        }

        if (mxl[degx] != 9999)
        {
            mxl[degx] = magnetometerData.magnetic.x;
            myl[degx] = magnetometerData.magnetic.y; 
            mzl[degx] = magnetometerData.magnetic.z;
            cntx++;
        }
        if (myl[degy + 360] != 9999)
        {
            mxl[degy + 360] = magnetometerData.magnetic.x;
            myl[degx + 360] = magnetometerData.magnetic.y; 
            mzl[degx + 360] = magnetometerData.magnetic.z;
            cnty++;
        }
        if (mzl[degz + 360 * 2] != 9999)
        {
            mxl[degz + 360 * 2] = magnetometerData.magnetic.x;
            myl[degz + 360 * 2] = magnetometerData.magnetic.y; 
            mzl[degz + 360 * 2] = magnetometerData.magnetic.z;
            cntz++;
        }
        Serial.print("x:");
        Serial.print(cntx);
        Serial.print(",");
        Serial.print("y:");
        Serial.print(cnty);
        Serial.print(",");
        Serial.print("z:");
        Serial.println(cntz);

        if (cntx >= 324 && cnty >= 324 && cntz >= 324)
        {
            break;
        }

    }
    int j = 0;
    float datax[(cntx + cnty + cntz)]; 
    float datay[(cntx + cnty + cntz)];  
    float dataz[(cntx + cnty + cntz)];  
    
    for (int* it = std::begin(mxl); it != std::end(mxl); ++it){
        if(mxl[it] != 9999){
            datax[j] = mxl[it];
            datay[j] = myl[it];
            dataz[j] = mzl[it];
            j++;
    }

    
   for (int* it = std::begin(datax); it != std::end(datax); ++it){
       x2[it] = pow(datax[it],2);
       y2[it] = pow(datay[it],2);
       z2[it] = pow(dataz[it],2);

       xy[it] = datax[it] * datay[it];
       xz[it] = datax[it] * dataz[it];
       yz[it] = datay[it] * dataz[it];

       ee[it] = -datax[it] * (x2[it] + y2[it] + z[it]);
       ff[it] = -datay[it] * (x2[it] + y2[it] + z[it]);
       gg[it] = -dataz[it] * (x2[it] + y2[it] + z[it]);
       hh[it] = -(x2[it] + y2[it] + z[it]); 
   }

    
    std::accumulate(array, array + SIZE_OF_ARRAY(array), 0.0) / cntx;
    std::accumulate(array, array + SIZE_OF_ARRAY(array), 0.0) / cnty;
    std::accumulate(array, array + SIZE_OF_ARRAY(array), 0.0) / cntz;

    return std::forward_as_tuple(mxl, myl, mzl);
}
