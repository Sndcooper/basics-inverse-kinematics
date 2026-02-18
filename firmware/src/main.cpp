#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

// PCA9685 Setup
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41);

// Servo Constants
constexpr uint16_t HIP_MIN_US = 1000;
constexpr uint16_t HIP_MAX_US = 2300;
constexpr uint16_t KNEE_MIN_US = 1200;
constexpr uint16_t KNEE_MAX_US = 2300;
constexpr uint16_t FOOT_MIN_US = 1000;
constexpr uint16_t FOOT_MAX_US = 2050;
constexpr uint16_t SERVO_MID_US = 1650;
constexpr uint8_t SERVO_COUNT = 18;

// Geometry (mm)
constexpr float COXA_LEN = 26.0f;    // Hip joint to knee joint
constexpr float FEMUR_LEN = 57.0f;   // Knee joint to foot joint
constexpr float TIBIA_LEN = 122.0f;  // Foot joint to tip
constexpr float HIP_RADIUS = 137.5f; // Origin to hip joint radius

// Mapping index to channel
// We assume standard mapping:
// Leg 0: Hip=0, Knee=1, Ankle=2
// Leg 1: Hip=3, Knee=4, Ankle=5
// ... Leg 2: Hip=6, Knee=7, Ankle=8
// ... Leg 3: Hip=0(PCA2), Knee=1(PCA2), Ankle=2(PCA2)
// ... Leg 4: Hip=3(PCA2)...
// ... Leg 5: Hip=6(PCA2)...

// Leg Mounting Angles (radians) - Global Yaw of the Hip Joint in Body Frame
const float LEG_MOUNT_ANGLES[6] = {
    0.0f,        // Leg 0 - 0 deg
    1.04719755f, // Leg 1 - 60 deg
    2.0943951f,  // Leg 2 - 120 deg
    3.14159265f, // Leg 3 - 180 deg
    4.1887902f,  // Leg 4 - 240 deg
    5.23598776f  // Leg 5 - 300 deg
};

// Helper: Convert microseconds to PCA9685 counts (0-4095)
uint16_t usToCounts(uint16_t microseconds, uint16_t freqHz) {
  float counts =
      (static_cast<float>(microseconds) * freqHz * 4096.0f) / 1000000.0f;
  if (counts < 0.0f)
    counts = 0.0f;
  if (counts > 4095.0f)
    counts = 4095.0f;
  return static_cast<uint16_t>(counts);
}

// Helper: Set servo pulse width
void setServoUs(uint8_t servo_index, uint16_t microseconds) {
  uint16_t counts = usToCounts(microseconds, 50);
  // Map linear index 0-17 to PCA boards
  // Leg 0,1,2 (Indices 0-8) -> PCA 1
  // Leg 3,4,5 (Indices 9-17) -> PCA 2
  if (servo_index < 9) {
    pca.setPWM(servo_index, 0, counts);
  } else {
    pca2.setPWM(servo_index - 9, 0, counts);
  }
}

// Convert angle (-90 to +90 degrees relative to neutral) to microseconds
// Assumes:
// -90 deg -> min
// 0 deg   -> mid
// +90 deg -> max
uint16_t angleToUs(float angle_deg, uint16_t min_us, uint16_t max_us) {
  if (angle_deg < -90.0f)
    angle_deg = -90.0f;
  if (angle_deg > 90.0f)
    angle_deg = 90.0f;

  float us = SERVO_MID_US +
             (angle_deg * (static_cast<float>(max_us - min_us) / 180.0f));
  if (us < min_us)
    us = static_cast<float>(min_us);
  if (us > max_us)
    us = static_cast<float>(max_us);

  return static_cast<uint16_t>(us);
}

// Desired foot-tip targets in body frame (x, y, z) for each leg.
// +X forward, +Y left, +Z up. Edit these values to move the bot.
float target_tips[6][3] = {
    {180.0f, 0.0f, -80.0f},    {90.0f, 156.0f, -80.0f},
    {-90.0f, 156.0f, -80.0f},  {-180.0f, 0.0f, -80.0f},
    {-90.0f, -156.0f, -80.0f}, {90.0f, -156.0f, -80.0f},
};

void driveIK(const float tips[6][3]) {
  Serial.println("Moving to Target Tips (IK)...");

  for (int leg = 0; leg < 6; leg++) {
    float mount_angle = LEG_MOUNT_ANGLES[leg];

    float hip_x = HIP_RADIUS * cos(mount_angle);
    float hip_y = HIP_RADIUS * sin(mount_angle);

    float dx = tips[leg][0] - hip_x;
    float dy = tips[leg][1] - hip_y;
    float dz = tips[leg][2];

    // Rotate target into leg frame so +X is forward for that leg.
    float local_x = dx * cos(-mount_angle) - dy * sin(-mount_angle);
    float local_y = dx * sin(-mount_angle) + dy * cos(-mount_angle);

    float hip_yaw_rad = atan2(local_y, local_x);
    float hip_yaw_deg = degrees(hip_yaw_rad);

    float planar = sqrt(local_x * local_x + local_y * local_y);
    float r = planar - COXA_LEN;
    float dist = sqrt(r * r + dz * dz);

    float min_reach = fabs(FEMUR_LEN - TIBIA_LEN);
    float max_reach = FEMUR_LEN + TIBIA_LEN;
    if (dist < min_reach)
      dist = min_reach;
    if (dist > max_reach)
      dist = max_reach;

    float angle_a = atan2(dz, r);
    float angle_b =
        acos((FEMUR_LEN * FEMUR_LEN + dist * dist - TIBIA_LEN * TIBIA_LEN) /
             (2.0f * FEMUR_LEN * dist));
    float femur_rad = angle_a - angle_b;
    float femur_deg = degrees(femur_rad);

    float tibia_inner =
        acos((FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - dist * dist) /
             (2.0f * FEMUR_LEN * TIBIA_LEN));
    float tibia_rad = -(PI - tibia_inner);
    float tibia_deg = degrees(tibia_rad);

    int base_idx = leg * 3;

    uint16_t pwm_hip = angleToUs(hip_yaw_deg, HIP_MIN_US, HIP_MAX_US);
    uint16_t pwm_knee = angleToUs(femur_deg, KNEE_MIN_US, KNEE_MAX_US);
    uint16_t pwm_ankle = angleToUs(tibia_deg, FOOT_MIN_US, FOOT_MAX_US);

    Serial.print("Leg ");
    Serial.print(leg);
    Serial.print(": Hip=");
    Serial.print(hip_yaw_deg);
    Serial.print(" deg (");
    Serial.print(pwm_hip);
    Serial.print("us), Knee=");
    Serial.print(femur_deg);
    Serial.print(" deg (");
    Serial.print(pwm_knee);
    Serial.print("us), Foot=");
    Serial.print(tibia_deg);
    Serial.print(" deg (");
    Serial.print(pwm_ankle);
    Serial.println("us)");

    setServoUs(base_idx + 0, pwm_hip);
    setServoUs(base_idx + 1, pwm_knee);
    setServoUs(base_idx + 2, pwm_ankle);

    delay(50);
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Hexabot Servo Controller - Pose Logic");

  pca.begin();
  pca.setPWMFreq(50);
  delay(10);
  pca2.begin();
  pca2.setPWMFreq(50);
  delay(10);

  driveIK(target_tips);
}

void loop() {
  driveIK(target_tips);
  delay(1000);
}
