#include <Adafruit_LSM6DS33.h>
#include <NewPing.h>
 
#define TRIG1  13
#define ECHO1  12
#define TRIG2  7
#define ECHO2  6
#define TRIG3  11
#define ECHO3  10
#define TRIG4  9
#define ECHO4  8

#define MAX_DISTANCE 200
#define SONAR_NUM 4
#define PING_INTERVAL 33

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(TRIG1, ECHO1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIG2, ECHO2, MAX_DISTANCE),
  NewPing(TRIG3, ECHO3, MAX_DISTANCE),
  NewPing(TRIG4, ECHO4, MAX_DISTANCE)
};

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

unsigned long imu_time;

// Ping frequency in Hz
unsigned int pingSpeed = 50;

Adafruit_LSM6DS33 lsm6ds33;

float yaw = 0;
float bias = 0;

void setup() {  
  Serial.begin(9600);  // start serial for output

  if (!lsm6ds33.begin_I2C()) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }
  }

  //Serial.println("LSM6DS33 Found!");
 
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

  // First few IMU readings are garbage
  for (int i = 0; i < 10; ++i) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds33.getEvent(&accel, &gyro, &temp);
    delay(100);
  }

  // Calibrate bias
  bias = 0;
  for (int i = 0; i < 10; ++i) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds33.getEvent(&accel, &gyro, &temp);
  
    bias += gyro.gyro.z;
    delay(100);
  }

  bias /= 10;
  
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) { // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }

  imu_time = millis();
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = MAX_DISTANCE;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  
  // Get IMU

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);

  yaw += (gyro.gyro.z - bias)*((millis() - imu_time)/1000.0);
  imu_time = millis();
}


void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  Serial.print(yaw);
  Serial.print(" ");
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(cm[i]);
    Serial.print(" ");
  }
  Serial.println();
}
