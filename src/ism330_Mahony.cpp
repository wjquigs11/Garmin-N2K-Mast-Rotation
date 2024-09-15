/*

  Mahony fusion filter for ISM330DHCX 6DOF sensor,
  S. J. Remington 4/2023

  tested with Adafruit Feather ESP32_S2 TFT

  Device library:
		https://github.com/sparkfun/SparkFun_6DoF_ISM330DHCX_Arduino_Library

*/



#include "windparse.h"
#include "compass.h"

SparkFun_ISM330DHCX ISM330;

// Structs for X,Y,Z data
sfe_ism_data_t acc;
sfe_ism_data_t gyr;


// if gyro is to calibrated upon startup, define the following:
//#define CAL_GYRO
//float gxyz_offsets[3] = {0};
// otherwise, define the gyro offsets explicitly
//float gxyz_offsets[3] = {131.0, -579.0, -183.0};
float gxyz_offsets[3] = { -80.7, -112.9, -433.8 };

// library returns milliDPS
float RadiansPerSecond = M_PI / 180000.0; //conversion factor for Mahony filter
float Axyz[3] = {0}, Gxyz[3] = {0}; //arrays for Mahony filter

float Kp = 50.0, Ki = 0.0; //PID constants for Mahony filter
float q[4] = {1.0, 0.0, 0.0, 0.0}; //initialize quaternion, global access. X points at yaw=0
static float yaw, pitch, roll; //Euler angle output
static int16_t temperature;

movingAvg avgMastYaw(20);

// example code just converts and prints Euler angles from time to time
//unsigned long print_ms = 200, now_ms = 0, last_ms = 0; //print every print_ms
#define UPDATE_SPEED 50
static unsigned long lastUpdate = 0;
#define PRINT_SPEED 0 // ms between angle prints (0 for none)
static unsigned long lastPrint = 0; // Keep track of print time

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat);

extern float mastCompassDeg;

bool setupMastIMU(TwoWire &wirePort, uint8_t deviceAddress) {

  //Wire.begin();

  //Serial.begin(115200); delay(500);
  //while (!Serial);  //wait for serial monitor to connect (can replace this with delay())

  bool initOK = ISM330.begin(wirePort, deviceAddress);
  if (!initOK) {
    Serial.println("ISM330 not detected.");
    return false;
  }

  //  Serial.println("Applying settings.");

  ISM330.setDeviceConfig();
  ISM330.setBlockDataUpdate();

  // Set the output data rate and precision of the accelerometer
  ISM330.setAccelDataRate(ISM_XL_ODR_104Hz);
  ISM330.setAccelFullScale(ISM_2g);

  // Set the output data rate and precision of the gyroscope
  ISM330.setGyroDataRate(ISM_GY_ODR_104Hz);
  ISM330.setGyroFullScale(ISM_500dps);

  /*
  	// Turn on the accelerometer's filter and apply settings.
  	ISM330.setAccelFilterLP2();
  	ISM330.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);
  */
  // Turn on the gyroscope's filter and apply settings.
  ISM330.setGyroFilterLP1();
  ISM330.setGyroLP1Bandwidth(ISM_MEDIUM);

  // reminder!
  Serial.print("Gyro offsets ");
  for (int i = 0; i < 3; i++) {
    Serial.print(gxyz_offsets[i], 1);
    Serial.print(" ");
  }
  Serial.println();
  avgMastYaw.begin();
  return true;

}  //end of setup()

// using SFE library,
// getAccel() returns milli_g, regardless of scale setting
// getGyro() returns milli_DPS, regardless

void getISMheading(void *parameter) {
  while(1) {

    static unsigned long last_us = 0, now_us = 0; //microsecond loop timing.
    unsigned long time;
    static int loop_counter = 0;

    if ((time = millis()) - lastUpdate > UPDATE_SPEED) {
      loop_counter++;
      // Check if both gyroscope and accelerometer data are available.
      if ( ISM330.checkStatus() ) {
        ISM330.getAccel(&acc);
        ISM330.getGyro(&gyr);
        now_us = micros();

        //normalize accelerometer data
        //This would be the place to apply calibration and offsets, if desired

        float amag = sqrt(acc.xData * acc.xData + acc.yData * acc.yData + acc.zData * acc.zData);

        Axyz[0] = acc.xData / amag;
        Axyz[1] = acc.yData / amag;
        Axyz[2] = acc.zData / amag;

        Gxyz[0] = (gyr.xData - gxyz_offsets[0]) * RadiansPerSecond;
        Gxyz[1] = (gyr.yData - gxyz_offsets[1]) * RadiansPerSecond;
        Gxyz[2] = (gyr.zData - gxyz_offsets[2]) * RadiansPerSecond;


        float deltat = (now_us - last_us) * 1.0e-6; //seconds since last update
        last_us = now_us;

        Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

        //roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
        //pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
        //conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
        yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
        // to degrees
        yaw   *= RADTODEG;
        if (yaw < 0) yaw += 360.0; //compass circle
        //pitch *= RADTODEG;
        //roll *= RADTODEG;

        if (PRINT_SPEED && (time - lastPrint > PRINT_SPEED)) {
          Serial.printf(">Myaw:%lu:%0.2f\n", lastPrint, yaw);
          Serial.printf(">Mavgyaw:%lu:%d\n", lastPrint, avgMastYaw.getAvg());
          lastPrint = millis(); // Update lastPrint time
        }
        lastUpdate = millis();
        mastCompassDeg = avgMastYaw.reading(yaw);      
        if (loop_counter % 10000 == 1) {
          temperature = ISM330.getTemp();
          Serial.printf("temp:%lu:%d\n", lastPrint, temperature);
        }
      }
    }
  }
}

//--------------------------------------------------------------------------------------------------
// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vector (gravity) and measured one.
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date      Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
// last update 07/09/2020 SJR minor edits
//--------------------------------------------------------------------------------------------------
// IMU algorithm update

// ***acceleration vector must be normalized***

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms

  // Estimated direction of gravity in the body frame (factor of two divided out)
  vx = q[1] * q[3] - q[0] * q[2];  //to normalize these terms, multiply each by 2.0
  vy = q[0] * q[1] + q[2] * q[3];
  vz = q[0] * q[0] - 0.5f + q[3] * q[3];

  // Error is cross product between estimated and measured direction of gravity in body frame
  // (half the actual magnitude)
  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);

  // Compute and apply to gyro term the integral feedback, if enabled
  if (Ki > 0.0f) {
    ix += Ki * ex * deltat;  // integral error scaled by Ki
    iy += Ki * ey * deltat;
    iz += Ki * ez * deltat;
    gx += ix;  // apply integral feedback
    gy += iy;
    gz += iz;
  }

  // Apply proportional feedback to gyro term
  gx += Kp * ex;
  gy += Kp * ey;
  gz += Kp * ez;

  // Integrate rate of change of quaternion, q cross gyro term
  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // renormalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}
