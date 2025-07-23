#include <Adafruit_BNO08x.h>

/*
  BNO085 Sensor Connection (SPI Mode):

  VCC  -> 3.3V
  GND  -> GND
  SCL  -> D18
  SDA  -> D19
  AD0  -> D23
  CS   -> D5
  INT  -> D16
  RST  -> D4
  PS1  -> 3.3V
  PS0  -> 3.3V
*/

#define BNO08X_CS     5
#define BNO08X_INT    16
#define BNO08X_RESET  4

#define MODE_STANDARD    0  // Standard mode of using the library directly
#define MODE_ACCELEROMETER 1
#define MODE_GYROSCOPE     2
#define MODE_MAGNETOMETER  3
#define MODE_COMPLEMENTARY 4  // Complementary filter mode

int currentMode = MODE_STANDARD;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct Quaternion {
  float w, x, y, z;
};

// Complementary filter variables
float pitch_comp = 0.0f;
float roll_comp = 0.0f;
float yaw_comp = 0.0f;
float alpha = 0.96f;  // (96% gyro, 4% accel)
unsigned long prevTime = 0;
float dt = 0.01f;

Quaternion q_calib_inv = {1, 0, 0, 0};
bool isCalibrated = false;

// Quaternion multiplication
Quaternion multiplyQuaternions(const Quaternion& q1, const Quaternion& q2) {
  Quaternion r;
  r.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
  r.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
  r.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
  r.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
  return r;
}

// Convert quaternion to Euler angles (degrees)
void quaternionToEuler(const Quaternion& q, float &roll, float &pitch, float &yaw) {
  roll  = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * 180.0f / PI;
  float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  if (sinp > 1.0f) sinp = 1.0f;
  else if (sinp < -1.0f) sinp = -1.0f;
  pitch = asinf(sinp) * 180.0f / PI;
  yaw   = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * 180.0f / PI;
}

void enableSensorMode(int mode) {
  // Disable all reports first
  bno08x.disableReport(SH2_GAME_ROTATION_VECTOR);
  bno08x.disableReport(SH2_ACCELEROMETER);
  bno08x.disableReport(SH2_GYROSCOPE_CALIBRATED);
  bno08x.disableReport(SH2_MAGNETIC_FIELD_CALIBRATED);
  
  switch (mode) {
    case MODE_STANDARD:
      if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        Serial.println("Could not enable game rotation vector!");
        while (1) delay(10);
      }
      Serial.println("Mode: Standard (Quaternion)");
      break;
      
    case MODE_ACCELEROMETER:
      if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
        Serial.println("Could not enable accelerometer!");
        while (1) delay(10);
      }
      Serial.println("Mode: Accelerometer");
      break;
      
    case MODE_GYROSCOPE:
      if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
        Serial.println("Could not enable gyroscope!");
        while (1) delay(10);
      }
      Serial.println("Mode: Gyroscope");
      break;
      
    case MODE_MAGNETOMETER:
      if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
        Serial.println("Could not enable magnetometer!");
        while (1) delay(10);
      }
      Serial.println("Mode: Magnetometer");
      break;
      
    case MODE_COMPLEMENTARY:
      if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
        Serial.println("Could not enable accelerometer!");
        while (1) delay(10);
      }
      if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
        Serial.println("Could not enable gyroscope!");
        while (1) delay(10);
      }
      if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
        Serial.println("Could not enable magnetometer!");
        while (1) delay(10);
      }
      Serial.println("Mode: Complementary Filter (Gyro+Accel for Pitch/Roll, Gyro+Mag for Yaw)");
      // Reset complementary filter angles
      pitch_comp = 0.0f;
      roll_comp = 0.0f;
      yaw_comp = 0.0f;
      prevTime = millis();
      break;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Starting BNO085...");

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) delay(10);
  }

  Serial.println("BNO085 Found!");
  
  enableSensorMode(currentMode);

  delay(200);  // Give time to stabilize
}

void loop() {
  delay(10);

  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset. Re-enabling reports...");
    enableSensorMode(currentMode);
    isCalibrated = false;
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case '0':
        currentMode = MODE_STANDARD;
        enableSensorMode(currentMode);
        isCalibrated = false;
        break;
      case '1':
        currentMode = MODE_ACCELEROMETER;
        enableSensorMode(currentMode);
        break;
      case '2':
        currentMode = MODE_GYROSCOPE;
        enableSensorMode(currentMode);
        break;
      case '3':
        currentMode = MODE_MAGNETOMETER;
        enableSensorMode(currentMode);
        break;
      case '4':
        currentMode = MODE_COMPLEMENTARY;
        enableSensorMode(currentMode);
        break;
    }
  }

  switch (currentMode) {
    case MODE_STANDARD:
      processStandardMode();
      break;
    case MODE_ACCELEROMETER:
      processAccelerometerMode();
      break;
    case MODE_GYROSCOPE:
      processGyroscopeMode();
      break;
    case MODE_MAGNETOMETER:
      processMagnetometerMode();
      break;
    case MODE_COMPLEMENTARY:
      processComplementaryMode();
      break;
  }
}

void processStandardMode() {
  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    Quaternion q_current;
    q_current.w = sensorValue.un.gameRotationVector.real;
    q_current.x = sensorValue.un.gameRotationVector.i;
    q_current.y = sensorValue.un.gameRotationVector.j;
    q_current.z = sensorValue.un.gameRotationVector.k;

    if (!isCalibrated) {
      q_calib_inv.w = q_current.w;
      q_calib_inv.x = -q_current.x;
      q_calib_inv.y = -q_current.y;
      q_calib_inv.z = -q_current.z;
      isCalibrated = true;
      Serial.println("Orientation zeroed using quaternion calibration.");
    }

    Quaternion q_zeroed = multiplyQuaternions(q_calib_inv, q_current);

    float roll, pitch, yaw;
    quaternionToEuler(q_zeroed, roll, pitch, yaw);

    // Normalize angles to (-180, 180)
    roll = wrapAngle(roll);
    pitch = wrapAngle(pitch);
    yaw = wrapAngle(yaw);

    // Serial.print("Zeroed Euler angles - Roll: ");
    // Serial.print(roll, 2);
    // Serial.print("  Pitch: ");
    // Serial.print(pitch, 2);
    // Serial.print("  Yaw: ");
    // Serial.println(yaw, 2);

    // Print to Serial as JSON
    String serialJson = "{";
    serialJson += "\"mode\":\"standard\",";
    serialJson += "\"w\":" + String(q_zeroed.w, 6) + ",";
    serialJson += "\"x\":" + String(q_zeroed.x, 6) + ",";
    serialJson += "\"y\":" + String(q_zeroed.y, 6) + ",";
    serialJson += "\"z\":" + String(q_zeroed.z, 6);
    serialJson += "}";
    Serial.println(serialJson);
  }
}

void processAccelerometerMode() {
  if (sensorValue.sensorId == SH2_ACCELEROMETER) {
    float x = sensorValue.un.accelerometer.x;
    float y = sensorValue.un.accelerometer.y;
    float z = sensorValue.un.accelerometer.z;
    
    // Calculate magnitude
    float magnitude = sqrt(x*x + y*y + z*z);
    
    // Print to Serial as JSON
    String serialJson = "{";
    serialJson += "\"mode\":\"accelerometer\",";
    serialJson += "\"x\":" + String(x, 4) + ",";
    serialJson += "\"y\":" + String(y, 4) + ",";
    serialJson += "\"z\":" + String(z, 4) + ",";
    serialJson += "\"magnitude\":" + String(magnitude, 4);
    serialJson += "}";
    Serial.println(serialJson);
  }
}

void processGyroscopeMode() {
  if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    float x = sensorValue.un.gyroscope.x;
    float y = sensorValue.un.gyroscope.y;
    float z = sensorValue.un.gyroscope.z;
    
    // Calculate magnitude
    float magnitude = sqrt(x*x + y*y + z*z);
    
    // Print to Serial as JSON
    String serialJson = "{";
    serialJson += "\"mode\":\"gyroscope\",";
    serialJson += "\"x\":" + String(x, 4) + ",";
    serialJson += "\"y\":" + String(y, 4) + ",";
    serialJson += "\"z\":" + String(z, 4) + ",";
    serialJson += "\"magnitude\":" + String(magnitude, 4);
    serialJson += "}";
    Serial.println(serialJson);
  }
}

void processMagnetometerMode() {
  if (sensorValue.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
    float x = sensorValue.un.magneticField.x;
    float y = sensorValue.un.magneticField.y;
    float z = sensorValue.un.magneticField.z;
    
    // Calculate magnitude
    float magnitude = sqrt(x*x + y*y + z*z);
    
    // Calculate heading (assuming sensor is level)
    float heading = atan2(y, x) * 180.0f / PI;
    if (heading < 0) heading += 360.0f;
    
    // Print to Serial as JSON
    String serialJson = "{";
    serialJson += "\"mode\":\"magnetometer\",";
    serialJson += "\"x\":" + String(x, 4) + ",";
    serialJson += "\"y\":" + String(y, 4) + ",";
    serialJson += "\"z\":" + String(z, 4) + ",";
    serialJson += "\"magnitude\":" + String(magnitude, 4) + ",";
    serialJson += "\"heading\":" + String(heading, 2);
    serialJson += "}";
    Serial.println(serialJson);
  }
}

void processComplementaryMode() {
  static float accel_x = 0, accel_y = 0, accel_z = 0;
  static float gyro_x = 0, gyro_y = 0, gyro_z = 0;
  static float mag_x = 0, mag_y = 0, mag_z = 0;
  static bool hasAccel = false, hasGyro = false, hasMag = false;
  
  // Update time delta
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0f;  // Convert to seconds
  prevTime = currentTime;
  
  if (sensorValue.sensorId == SH2_ACCELEROMETER) {
    accel_x = sensorValue.un.accelerometer.x;
    accel_y = sensorValue.un.accelerometer.y;
    accel_z = sensorValue.un.accelerometer.z;
    hasAccel = true;
  }
  
  if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    gyro_x = sensorValue.un.gyroscope.x;
    gyro_y = sensorValue.un.gyroscope.y;
    gyro_z = sensorValue.un.gyroscope.z;
    hasGyro = true;
  }
  
  if (sensorValue.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
    mag_x = sensorValue.un.magneticField.x;
    mag_y = sensorValue.un.magneticField.y;
    mag_z = sensorValue.un.magneticField.z;
    hasMag = true;
  }
  
  if (hasAccel && hasGyro && hasMag) {
    // Calculate angles from accelerometer (assuming sensor is relatively stable)
    float accel_pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0f / PI;
    float accel_roll = atan2(accel_y, accel_z) * 180.0f / PI;
    
    // Calculate yaw from magnetometer (assuming sensor is level)
    float mag_yaw = atan2(mag_y, mag_x) * 180.0f / PI;
    if (mag_yaw < 0) mag_yaw += 360.0f;
    
    // Integrate gyroscope data
    float gyro_pitch = pitch_comp + gyro_y * dt;
    float gyro_roll = roll_comp + gyro_x * dt;
    float gyro_yaw = yaw_comp + gyro_z * dt;
    
    // Complementary filter for pitch and roll (gyro + accel)
    pitch_comp = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
    roll_comp = alpha * gyro_roll + (1.0f - alpha) * accel_roll;
    
    // Complementary filter for yaw (gyro + magnetometer)
    // Use a different alpha for yaw fusion (typically lower to trust magnetometer more)
    float alpha_yaw = 0.90f;  // 90% gyro, 10% magnetometer
    yaw_comp = alpha_yaw * gyro_yaw + (1.0f - alpha_yaw) * mag_yaw;
    
    pitch_comp = wrapAngle(pitch_comp);
    roll_comp = wrapAngle(roll_comp);
    yaw_comp = wrapAngle(yaw_comp);
    
    // Print to Serial as JSON
    String serialJson = "{";
    serialJson += "\"mode\":\"complementary\",";
    serialJson += "\"pitch\":" + String(pitch_comp, 2) + ",";
    serialJson += "\"roll\":" + String(roll_comp, 2) + ",";
    serialJson += "\"yaw\":" + String(yaw_comp, 2) + ",";
    // serialJson += "\"accel_pitch\":" + String(accel_pitch, 2) + ",";
    // serialJson += "\"accel_roll\":" + String(accel_roll, 2) + ",";
    // serialJson += "\"mag_yaw\":" + String(mag_yaw, 2) + ",";
    // serialJson += "\"gyro_pitch\":" + String(gyro_pitch, 2) + ",";
    // serialJson += "\"gyro_roll\":" + String(gyro_roll, 2) + ",";
    // serialJson += "\"gyro_yaw\":" + String(gyro_yaw, 2) + ",";
    serialJson += "\"dt\":" + String(dt, 4);
    serialJson += "}";
    Serial.println(serialJson);
    
    hasAccel = false;
    hasGyro = false;
    hasMag = false;
  }
}

float wrapAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}
