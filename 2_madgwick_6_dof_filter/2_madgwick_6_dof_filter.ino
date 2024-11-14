#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <ITG3200.h>

#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

// Gyroscope
ITG3200 gyro;

// Accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

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

  filter.begin(100);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 100;
  microsPrevious = micros();
}

void loop() {
  unsigned long microsNow;

  float gx, gy, gz;
  float ax, ay, az;
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
    Serial.println((int)az);

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