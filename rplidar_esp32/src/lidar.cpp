#include "RPLidar.h"

#define LIDAR_RX        16  // UART RX
#define LIDAR_TX        17  // UART TX
#define MOTOR_PWM_PIN   18  // PWM-capable pin

HardwareSerial lidarSerial(2);
RPLidar lidar;

// PWM motor control
#define PWM_CHANNEL     0
#define PWM_FREQ        500
#define PWM_RESOLUTION  8
#define PWM_DUTY        255

void setupMotorPWM() {
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, PWM_DUTY);
}

// CSV streaming
#define POINTS_PER_FRAME 5
String dataBuffer;
int pointCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  setupMotorPWM();

  lidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  lidar.begin(lidarSerial);

  rplidar_response_device_info_t info;
  if (lidar.getDeviceInfo(info) != RESULT_OK) {
    Serial.println("LIDAR not detected.");
    while (1);
  }

  lidar.startScan(); // default scan mode
}

void loop() {
  if (lidar.waitPoint() == RESULT_OK) {
    RPLidarMeasurement point = lidar.getCurrentPoint();

    if (point.distance > 0 && point.distance < 12000 && point.quality > 0) {
      dataBuffer += String(point.angle, 1) + "," + String(point.distance, 0) + "\n";
      pointCount++;
    }

    if (pointCount >= POINTS_PER_FRAME) {
      Serial.println("FRAME_START");
      Serial.print(dataBuffer);
      Serial.println("FRAME_END");
      dataBuffer = "";
      pointCount = 0;
    }
  }
}
