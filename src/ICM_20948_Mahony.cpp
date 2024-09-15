// Mahony AHRS for the ICM_20948  S.J. Remington 6/2021
// Requires the Sparkfun ICM_20948 library
// Standard sensor orientation X North (yaw=0), Y West, Z up
// magnetometer Y and Z axes are reflected to reconcile with accelerometer.

// New Mahony filter error scheme uses Up (accel Z axis) and West (= Acc cross Mag) as the orientation reference vectors
// heavily modified from http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
//
// To collect data for calibration, use the companion program ICM_20948_get_cal_data
//
/*
  Sparkfun ICM_20948
  Hardware setup: This library supports communicating with the
  ICM_20948 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
  ICM_20948 --------- Arduino
   SCL ---------- SCL (A5 on older 'Duinos')
   SDA ---------- SDA (A4 on older 'Duinos')
   VIN ------------- 5V or 3.3V
   GND ------------- GND

*/

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <movingAvg.h>
#include "compass.h"

//////////////////////////
// ICM_20948 Library Init //
//////////////////////////
// default settings for accel and magnetometer

#define WIRE_PORT Wire // desired Wire port.
#define AD0_VAL 1      // value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when
// the ADR jumper is closed the value becomes 0

ICM_20948_I2C imu; // create an ICM_20948_I2C object imu;

// VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto
//The compass will NOT work well or at all if these are not correct

//Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float Gscale = (M_PI / 180.0) * 0.00763; //250 dps scale sensitivity = 131 dps/LSB
float G_offset[3] = {-18.3, 37.2, 24.8};

//Accel scale: divide by 16604.0 to normalize
float A_B[3]
{   79.60,  -18.56,  383.31};

float A_Ainv[3][3]
{ {  1.00847,  0.00470, -0.00428},
  {  0.00470,  1.00846, -0.00328},
  { -0.00428, -0.00328,  0.99559}
};

//Mag scale divide by 369.4 to normalize
float M_B[3] = { -121.47 , 106.46 , -58.1 };

float M_Ainv[3][3] = {
{ 3.13678 , -0.11358 , -0.00561 },
{ -0.11358 , 2.91054 , -0.02742 },
{ -0.00561 , -0.02742 , 2.98084 }};

// local magnetic declination in degrees
float declination = VARIATION;

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not yet optimized (slight overshoot apparent after rapid sensor reorientations). Ki is not used.
#define Kp 50.0
#define Ki 0.0

movingAvg avgYaw(20);

unsigned long ICMnow = 0, ICMlast = 0; //micros() timers for AHRS loop
float ICMdeltat = 0;  //loop time in seconds

#define UPDATE_SPEED 50
static unsigned long lastUpdate = 0;
#define PRINT_SPEED 0 // ms between angle prints (0 for none)
static unsigned long lastPrint = 0; // Keep track of print time

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output

extern float boatCompassDeg; // magnetic heading not corrected for variation

float calculateHeading(float r, float i, float j, float k, int correction);
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

#if 0
void setup()
{
  Serial.begin(115200);
  while (!Serial); //wait for connection
  WIRE_PORT.begin();
  WIRE_PORT.setClock(100000);
  imu.begin(WIRE_PORT, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.println(F("ICM_90248 not detected"));
    while (1);
  }
  avgYaw.begin();
}
#endif

void getICMheading(void *parameter) {
  for (;;) {
    static int loop_counter = 0; //sample & update loop counter
    static float Gxyz[3], Axyz[3], Mxyz[3]; //centered and scaled gyro/accel/mag data
    unsigned long time;

    // Update the sensor values whenever new data is available
    if ( imu.dataReady() ) {

      imu.getAGMT();

      loop_counter++;
      get_scaled_IMU(Gxyz, Axyz, Mxyz);

      // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0

      Mxyz[1] = -Mxyz[1]; //reflect Y and Z
      Mxyz[2] = -Mxyz[2]; //must be done after offsets & scales applied to raw data

      ICMnow = micros();
      ICMdeltat = (ICMnow - ICMlast) * 1.0e-6; //seconds since last update
      ICMlast = ICMnow;

      //   Gxyz[0] = Gxyz[1] = Gxyz[2] = 0;
      MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                            Mxyz[0], Mxyz[1], Mxyz[2], ICMdeltat);

      if ((time = millis()) - lastUpdate > UPDATE_SPEED) {

        // Define Tait-Bryan angles. Strictly valid only for approximately level movement
        
        // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
        // this code corrects for magnetic declination.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the
        // Earth is positive, up toward the sky is negative. Roll is angle between
        // sensor y-axis and Earth ground plane, y-axis up is positive roll.
        // Tait-Bryan angles as well as Euler angles are
        // non-commutative; that is, the get the correct orientation the rotations
        // must be applied in the correct order.
        //
        // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // which has additional links.
              
        // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
        // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock
        
        //roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
        //pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
        yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
        // to degrees
        yaw   *= 180.0 / PI;
        //pitch *= 180.0 / PI;
        //roll *= 180.0 / PI;

        // http://www.ngdc.noaa.gov/geomag-web/#declination
        //conventional nav, yaw increases CW from North, corrected for local magnetic declination

        yaw = -(yaw + declination);
        if (yaw < 0) yaw += 360.0;
        if (yaw >= 360.0) yaw -= 360.0;

        if (PRINT_SPEED && (time - lastPrint > PRINT_SPEED)) {
          //     Serial.print("ypr ");
          Serial.printf(">yaw:%lu:%0.2f\n", lastPrint, yaw);
          Serial.printf(">avgyaw:%lu:%d\n", lastPrint, avgYaw.getAvg());
          loop_counter = 0;
          lastPrint = millis(); // Update lastPrint time
        }
        lastUpdate = millis();
        boatCompassDeg = avgYaw.reading(yaw);   // moving average is more stable
      }
    }
    //vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// function to subtract offsets and apply scale/correction matrices to IMU data

void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];

  Gxyz[0] = Gscale * (imu.agmt.gyr.axes.x - G_offset[0]);
  Gxyz[1] = Gscale * (imu.agmt.gyr.axes.y - G_offset[1]);
  Gxyz[2] = Gscale * (imu.agmt.gyr.axes.z - G_offset[2]);

  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;

  //apply accel offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply mag offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  //observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; // Handle div by zero

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }


  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;


 //update quaternion with integrated contribution
 // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
gx = gx * (0.5*deltat); // pre-multiply common factors
gy = gy * (0.5*deltat);
gz = gz * (0.5*deltat);
float qa = q1;
float qb = q2;
float qc = q3;
q1 += (-qb * gx - qc * gy - q4 * gz);
q2 += (qa * gx + qc * gz - q4 * gy);
q3 += (qa * gy - qb * gz + q4 * gx);
q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
