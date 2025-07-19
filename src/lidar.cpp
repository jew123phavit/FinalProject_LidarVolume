#include <Arduino.h>
#include <thijs_rplidar.h>

#define LIDAR_RX 16
#define LIDAR_TX 17
#define MOTOR_PWM_PIN 18

HardwareSerial lidarSerial(2);
RPlidar lidar(lidarSerial);

// Manual motor control
#define PWM_CHANNEL     0
#define PWM_FREQ        500
#define PWM_RESOLUTION  8
#define PWM_DUTY        120

void setupMotorPWM() {
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, PWM_DUTY);
}

// Point batching
#define POINTS_PER_FRAME 200
String dataBuffer;
int pointCounter = 0;

void handleLidarPoint(RPlidar* lidarPtr, uint16_t dist_mm, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
  float angle_deg = fmod(angle_q6 / 64.0, 360.0);
  if (angle_deg < 0) angle_deg += 360.0;

  // Filter noise
  if (dist_mm == 0 || dist_mm > 12000 || quality == 0) return;

  // Add to buffer
  dataBuffer += String(angle_deg, 1) + "," + String(dist_mm) + "\n";
  pointCounter++;

  if (pointCounter >= POINTS_PER_FRAME) {
    Serial.println("FRAME_START");
    Serial.print(dataBuffer);
    Serial.println("FRAME_END");

    pointCounter = 0;
    dataBuffer = "";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  setupMotorPWM();

  lidar.init(LIDAR_RX, LIDAR_TX);
  lidar.postParseCallback = handleLidarPoint;

  lidar.startStandardScan(); // No debug prints
}

void loop() {
  lidar.handleData(false, true); // Only valid points with checksum
}
