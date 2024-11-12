#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include <ITG3200.h>

#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

// Gyroscope
ITG3200 gyro;

// Accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Magnetometer
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(67890);

// Magnetometer Calibration
// Change this
const int hard_iron[3] = { 0, 0, 0 };

// Change this
const float soft_iron[3][3] = {
  { 1, 0, 0 },
  { 0, 1, 0 },
  { 0, 0, 1 }
};

void setup() {
  // Initialize the serial communication:
  Serial.begin(115200);

  /* Initialize the gyroscope */
  gyro.init();
  gyro.zeroCalibrate(200, 10);  //sample 200 times to calibrate and it will take 200*10ms

  /* Initialise the accelerometer */
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1)
      ;
  }

  /* Set the range to whatever is appropriate for your project */
  // accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  accel.setRange(ADXL345_RANGE_2_G);

  /* Initialize the magnetometer sensor */
  if (!mag.begin()) {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1)
      ;
  }

  filter.begin(25);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

void loop() {
  unsigned long microsNow;

  float gx, gy, gz;
  float ax, ay, az;
  float mx, my, mz;
  float roll, pitch, heading;

  sensors_event_t event;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    /* Gyroscope in degrees per second */
    gyro.getAngularVelocity(&gx, &gy, &gz);

    /* Accelerometer in g */
    accel.getEvent(&event);

    ax = event.acceleration.x;
    ay = event.acceleration.y;
    az = event.acceleration.z;

    /* Magnetometer Data in uT */
    mag.getEvent(&event);

    mx = event.magnetic.x;
    my = event.magnetic.y;
    mz = event.magnetic.z;

    // Hard Iron Calibration
    int hi_cal[3];
    hi_cal[0] = mx - hard_iron[0];
    hi_cal[1] = my - hard_iron[1];
    hi_cal[2] = mz - hard_iron[2];

    // Soft Iron Calibration
    float mag_data[3];
    for (int i = 0; i < 3; i++) {
      mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
    }

    // Log it to serial port (Raw: gx, gy, gz, ax, ay, az, mx, my, mz)
    Serial.print("Raw:");
    Serial.print((int)gx);
    Serial.print(",");
    Serial.print((int)gy);
    Serial.print(",");
    Serial.print((int)gz);
    Serial.print(",");

    Serial.print((int)ax);
    Serial.print(",");
    Serial.print((int)ay);
    Serial.print(",");
    Serial.print((int)az);
    Serial.print(",");

    Serial.print((int)(mag_data[0] * 10));
    Serial.print(",");
    Serial.print((int)(mag_data[1] * 10));
    Serial.print(",");
    Serial.println((int)(mag_data[2] * 10));

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}