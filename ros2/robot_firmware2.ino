#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int32.h>
#include <rosidl_runtime_c/string_functions.h>
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

// =========== CONFIG ==============
#define DEBUG_SERIAL  true
#define PID_PERIOD_MS 50
#define IMU_PERIOD_MS 20
#define ENCODER_PPR   3025  // ปรับตาม encoder จริง (pulses per rev)

#define LEFT_PWM_FACTOR  1.0
#define RIGHT_PWM_FACTOR 1.0
#define BACKWARD_FACTOR  1.2

#define PUBLISH_TF 1 // 1 เพื่อ publish /tf จากบน firmware

// =========== PIN SETUP =============
struct MotorPins {
  uint8_t RPWM, LPWM, REN, LEN;
  uint8_t ENC_A, ENC_B;
};
MotorPins leftMotor  = {8, 9, 6, 7, 15, 14};
MotorPins rightMotor = {4, 5, 2, 3, 16, 17};

// ======= HC-SR04 (Ultrasonic) ======
#define ULTRASONIC_TRIG 10
#define ULTRASONIC_ECHO 11
// publish rate ~10 Hz
#define ULTRASONIC_PERIOD_MS 100
// =========== PID STRUCT ============
struct Wheel {
  volatile long encCount = 0;
  long lastCount = 0;
  float targetRPM = 0;
  float measuredRPM = 0;
  float filteredRPM = 0;
  float Kp, Ki, Kd;
  float integ = 0, lastError = 0;
  int pwmOut = 0;
};
Wheel left = {.Kp=1.8, .Ki=0.05, .Kd=0.3};
Wheel right = {.Kp=1.8, .Ki=0.05, .Kd=0.3};

// ========== MPU6050 ==========
MPU6050 mpu;

// ========== micro-ROS ==========
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_subscription_t sub_cmd_vel;
rcl_publisher_t pub_odom;
rcl_publisher_t pub_tf;
rcl_publisher_t pub_imu;

// --- Joint State Publisher เพิ่มตรงนี้ ---
rcl_publisher_t pub_joint_states;
sensor_msgs__msg__JointState joint_state_msg;
rosidl_runtime_c__String joint_names[2];  // <-- แก้เป็น rosidl_runtime_c__String
static double joint_positions[2] = {0.0, 0.0};

// --- Encoder raw publisher สำหรับ debug ROS2 ---
rcl_publisher_t pub_left_enc;
rcl_publisher_t pub_right_enc;
std_msgs__msg__Int32 left_enc_msg;
std_msgs__msg__Int32 right_enc_msg;

// ----------------------------------------

geometry_msgs__msg__Twist cmd_msg;
nav_msgs__msg__Odometry odom_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped tf_odom_to_base;
sensor_msgs__msg__Imu imu_msg;

unsigned long lastPidTime = 0;
unsigned long lastCmdTime = 0;
unsigned long lastImuTime = 0;
unsigned long lastRangeTime = 0;

// --- Ultrasonic publisher ---
rcl_publisher_t pub_range;
sensor_msgs__msg__Range range_msg;

// ========== ODOMETRY/ROBOT PARAM ==========
float robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;
const float WHEEL_RADIUS = 0.065;        // m
const float WHEEL_SEPARATION = 0.26;     // m

// ======== Function: encode count → radian =========
float encoder_to_angle(long encCount) {
  return ((float)encCount / ENCODER_PPR) * 2.0 * M_PI;
}

// ========== MOTOR CONTROL ==========
void setMotorPWM(const MotorPins &mp, int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (&mp == &leftMotor) pwm = pwm * LEFT_PWM_FACTOR;
  else pwm = pwm * RIGHT_PWM_FACTOR;

  if (abs(pwm) < 5) {
    analogWrite(mp.RPWM, 0);
    analogWrite(mp.LPWM, 0);
    return;
  }

  if (pwm > 0) {
    analogWrite(mp.RPWM, pwm);
    digitalWrite(mp.LPWM, LOW);
  } else {
    analogWrite(mp.LPWM, -pwm);
    digitalWrite(mp.RPWM, LOW);
  }
}

// ========== ENCODER ISR ==========
void encL_ISR() {
  bool a = digitalRead(leftMotor.ENC_A);
  bool b = digitalRead(leftMotor.ENC_B);
  if (a == b) left.encCount++;
  else left.encCount--;
}
void encR_ISR() {
  bool a = digitalRead(rightMotor.ENC_A);
  bool b = digitalRead(rightMotor.ENC_B);
  if (a == b) right.encCount++;
  else right.encCount--;
}

// ========== PID ==========
void computeWheelPID(Wheel &w, const MotorPins &mp, unsigned long dt) {
  long diff = w.encCount - w.lastCount;
  w.lastCount = w.encCount;
  float pulsesPerMin = diff * (60000.0 / dt);
  float newRPM = pulsesPerMin / ENCODER_PPR;
  w.filteredRPM = 0.8 * w.filteredRPM + 0.2 * newRPM;
  w.measuredRPM = w.filteredRPM;

  float error = w.targetRPM - w.measuredRPM;
  if (fabs(w.targetRPM) < 0.5) {
    w.integ = 0; w.lastError = 0; w.pwmOut = 0; setMotorPWM(mp, 0); return;
  }
  w.integ += error * (dt / 1000.0);
  float deriv = (error - w.lastError) / (dt / 1000.0);
  w.lastError = error;

  int pwmFF = 0;
  int maxRPM = 200;
  if (w.targetRPM > 0) pwmFF = map(w.targetRPM, 0, maxRPM, 0, 255);
  else if (w.targetRPM < 0) pwmFF = -map(-w.targetRPM, 0, maxRPM, 0, 255) * BACKWARD_FACTOR;

  int targetPWM = constrain((int)(w.Kp*error + w.Ki*w.integ + w.Kd*deriv + pwmFF), -255, 255);
  w.pwmOut = w.pwmOut + 0.5 * (targetPWM - w.pwmOut);
  setMotorPWM(mp, w.pwmOut);
}

// ========== ODOMETRY CALC ==========
void updateOdometry(float dt) {
  // Convert measuredRPM → m/s for each wheel
  float vL = (left.measuredRPM / 60.0) * 2 * M_PI * WHEEL_RADIUS;  // m/s
  float vR = (right.measuredRPM / 60.0) * 2 * M_PI * WHEEL_RADIUS; // m/s
  float vx = (vR + vL) / 2.0;         // Linear velocity (m/s)
  float vth = (vR - vL) / WHEEL_SEPARATION; // Angular velocity (rad/s)

  // Update pose
  robot_x += vx * cos(robot_theta) * dt;
  robot_y += vx * sin(robot_theta) * dt;
  robot_theta += vth * dt;
  // Normalize theta (important for quaternions)
  while (robot_theta >  M_PI) robot_theta -= 2*M_PI;
  while (robot_theta < -M_PI) robot_theta += 2*M_PI;

  // Populate odom_msg [EXACT units per REP-0147]
  odom_msg.pose.pose.position.x = robot_x;
  odom_msg.pose.pose.position.y = robot_y;
  odom_msg.pose.pose.position.z = 0.0;

  // Convert robot_theta to quaternion for 2D (yaw)
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(robot_theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(robot_theta / 2.0);

  odom_msg.twist.twist.linear.x  = vx;   // m/s
  odom_msg.twist.twist.linear.y  = 0.0;
  odom_msg.twist.twist.linear.z  = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = vth;  // rad/s

  // Set covariance (recommend: small nonzero for x, y, yaw rates)
  odom_msg.pose.covariance[0] = 1e-3;
  odom_msg.pose.covariance[7] = 1e-3;
  odom_msg.pose.covariance[35] = 1e-2;
  odom_msg.twist.covariance[0] = 1e-3;
  odom_msg.twist.covariance[7] = 1e-3;
  odom_msg.twist.covariance[35] = 1e-2;
}

// ========== IMU READ & PUBLISH ==========
void readAndPublishIMU() {
  unsigned long now = millis();
  if (now - lastImuTime < IMU_PERIOD_MS) return;
  lastImuTime = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  imu_msg.header.stamp.sec = now / 1000;
  imu_msg.header.stamp.nanosec = (now % 1000) * 1000000;
  imu_msg.header.frame_id.data = (char*)"imu_link";

  // Acceleration (m/s^2) - convert from raw to g, then to m/s^2
  float accel_scale = 9.81 / 16384.0;
  imu_msg.linear_acceleration.x = ax * accel_scale;
  imu_msg.linear_acceleration.y = ay * accel_scale;
  imu_msg.linear_acceleration.z = az * accel_scale;

  // Angular velocity (rad/s) - convert from raw to deg/s, then to rad/s
  float gyro_scale = (1.0 / 131.0) * DEG_TO_RAD;
  imu_msg.angular_velocity.x = gx * gyro_scale;
  imu_msg.angular_velocity.y = gy * gyro_scale;
  imu_msg.angular_velocity.z = gz * gyro_scale;

  // Orientation (no mag/fusion: identity quaternion)
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  // Basic covariance (ให้ ekf-node ประมวลผลเองใน ROS2)
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = (i % 4 == 0) ? 0.05 : 0.0;
    imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 1e-5 : 0.0;
    imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 1e-3 : 0.0;
  }

  rcl_publish(&pub_imu, &imu_msg, NULL);
}

// ========== ULTRASONIC (HC-SR04) READ & PUBLISH ==========
void readAndPublishRange() {
  unsigned long now = millis();
  if (now - lastRangeTime < ULTRASONIC_PERIOD_MS) return;
  lastRangeTime = now;

  // Trigger pulse
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  // Wait for echo (timeout ~30ms => ~5m)
  unsigned long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000UL);
  float distance_m;
  if (duration == 0) {
    // no echo, report max range
    distance_m = range_msg.max_range;
  } else {
    // convert microseconds to meters: duration[*us] * 0.0001715
    distance_m = duration * 0.0001715;
  }
  if (distance_m < range_msg.min_range) distance_m = range_msg.min_range;
  if (distance_m > range_msg.max_range) distance_m = range_msg.max_range;

  range_msg.header.stamp.sec = now / 1000;
  range_msg.header.stamp.nanosec = (now % 1000) * 1000000;
  range_msg.header.frame_id.data = (char*)"ultrasonic_link";
  range_msg.range = distance_m;

  rcl_publish(&pub_range, &range_msg, NULL);
}

// ========== TF PUBLISH ==========
#if PUBLISH_TF
void publishTF() {
  tf_odom_to_base.header.stamp.sec = millis() / 1000;
  tf_odom_to_base.header.stamp.nanosec = (millis() % 1000) * 1000000;
  tf_odom_to_base.header.frame_id.data = (char*)"odom";
  tf_odom_to_base.child_frame_id.data = (char*)"base_link";

  tf_odom_to_base.transform.translation.x = robot_x;
  tf_odom_to_base.transform.translation.y = robot_y;
  tf_odom_to_base.transform.translation.z = 0.0;

  float qz = sin(robot_theta/2);
  float qw = cos(robot_theta/2);
  tf_odom_to_base.transform.rotation.x = 0.0;
  tf_odom_to_base.transform.rotation.y = 0.0;
  tf_odom_to_base.transform.rotation.z = qz;
  tf_odom_to_base.transform.rotation.w = qw;

  tf_msg.transforms.data = &tf_odom_to_base;
  tf_msg.transforms.size = 1;
  tf_msg.transforms.capacity = 1;
  rcl_publish(&pub_tf, &tf_msg, NULL);
}
#else
void publishTF() {}
#endif

// ========== CMD_VEL CALLBACK ==========
void cmdVelCallback(const void *msgin) {
  lastCmdTime = millis();
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float v = msg->linear.x;
  float w = msg->angular.z;
  float vL = v - w * WHEEL_SEPARATION / 2.0;
  float vR = v + w * WHEEL_SEPARATION / 2.0;
  // convert m/s to rpm for wheels
  left.targetRPM  = (vL / (2 * M_PI * WHEEL_RADIUS)) * 60.0;
  right.targetRPM = (vR / (2 * M_PI * WHEEL_RADIUS)) * 60.0;
}

// ========== MAIN LOOP ==========
void processPIDandPublish() {
  unsigned long now = millis();
  unsigned long dt = now - lastPidTime;
  if (dt < PID_PERIOD_MS) return;
  lastPidTime = now;

  computeWheelPID(left, leftMotor, dt);
  computeWheelPID(right, rightMotor, dt);

  updateOdometry(dt / 1000.0);

  // Odometry Header
  odom_msg.header.stamp.sec = now / 1000;
  odom_msg.header.stamp.nanosec = (now % 1000) * 1000000;
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.child_frame_id.data = (char*)"base_link";

  rcl_publish(&pub_odom, &odom_msg, NULL);

  #if PUBLISH_TF
  publishTF();
  #endif

  // --- Joint State Update & Publish ---
  joint_state_msg.header.stamp.sec = now / 1000;
  joint_state_msg.header.stamp.nanosec = (now % 1000) * 1000000;
  joint_positions[0] = encoder_to_angle(left.encCount);   // left wheel
  joint_positions[1] = encoder_to_angle(right.encCount);  // right wheel
  rcl_publish(&pub_joint_states, &joint_state_msg, NULL);

  // --- เพิ่ม publish encoder raw ---
  left_enc_msg.data = left.encCount;
  right_enc_msg.data = right.encCount;
  rcl_publish(&pub_left_enc, &left_enc_msg, NULL);
  rcl_publish(&pub_right_enc, &right_enc_msg, NULL);
  // -------------------------------

  if (DEBUG_SERIAL) {
    Serial.print("X:"); Serial.print(robot_x,3);
    Serial.print(" Y:"); Serial.print(robot_y,3);
    Serial.print(" Th:"); Serial.print(robot_theta,3);
    Serial.print("  L_enc: "); Serial.print(left.encCount);
    Serial.print("  R_enc: "); Serial.print(right.encCount);
    Serial.print(" L:"); Serial.print(left.measuredRPM,1);
    Serial.print(" R:"); Serial.println(right.measuredRPM,1);
  }
}

void setupMotorPins(const MotorPins &mp) {
  pinMode(mp.RPWM, OUTPUT); pinMode(mp.LPWM, OUTPUT);
  pinMode(mp.REN, OUTPUT); pinMode(mp.LEN, OUTPUT);
  digitalWrite(mp.REN, HIGH); digitalWrite(mp.LEN, HIGH);
}
void setupEncoder(const MotorPins &mp, void (*isr)()) {
  pinMode(mp.ENC_A, INPUT_PULLUP); pinMode(mp.ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(mp.ENC_A), isr, RISING);
}

void setup() {
  Serial.begin(115200); delay(1000);
  set_microros_transports();
  while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) { delay(500); }

  analogWriteFrequency(leftMotor.RPWM, 20000);
  analogWriteFrequency(rightMotor.RPWM, 20000);
  analogWriteResolution(8);
  setupMotorPins(leftMotor); 
  setupMotorPins(rightMotor);
  setupEncoder(leftMotor, encL_ISR); 
  setupEncoder(rightMotor, encR_ISR);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "diffbot_teensy", "", &support);

  rclc_subscription_init_default(&sub_cmd_vel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_publisher_init_default(&pub_odom, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
#if PUBLISH_TF
  rclc_publisher_init_default(&pub_tf, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf");
#endif
  rclc_publisher_init_default(&pub_imu, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data_raw");

  // ---- Ultrasonic (HC-SR04) setup ----
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  // Initialize Range message defaults
  range_msg.radiation_type = 0; // ULTRASOUND
  range_msg.field_of_view = 0.26; // ~15 degrees
  range_msg.min_range = 0.02;
  range_msg.max_range = 4.0;
  range_msg.range = range_msg.max_range;
  range_msg.header.frame_id.data = (char*)"ultrasonic_link";
  rclc_publisher_init_default(&pub_range, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "/ultrasonic_range");

  // ---- JointState publisher ----
  memset(&joint_state_msg, 0, sizeof(joint_state_msg));
  // <---- แก้ตรงนี้ ---->
  joint_names[0].data = (char*)"left_wheel_joint";
  joint_names[0].size = strlen(joint_names[0].data);
  joint_names[0].capacity = joint_names[0].size + 1;
  joint_names[1].data = (char*)"right_wheel_joint";
  joint_names[1].size = strlen(joint_names[1].data);
  joint_names[1].capacity = joint_names[1].size + 1;

  joint_state_msg.name.data = joint_names;
  joint_state_msg.name.size = 2;
  joint_state_msg.name.capacity = 2;

  joint_state_msg.position.data = joint_positions;
  joint_state_msg.position.size = 2;
  joint_state_msg.position.capacity = 2;
  rclc_publisher_init_default(&pub_joint_states, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states");
  // -----------------------------

  // ========== เพิ่ม publisher encoder ==========
  rclc_publisher_init_default(&pub_left_enc, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/left_encoder");
  rclc_publisher_init_default(&pub_right_enc, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/right_encoder");
  // =============================================

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmd_vel, &cmd_msg, &cmdVelCallback, ON_NEW_DATA);

  // Initialize odom_msg fixed frame IDs
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.child_frame_id.data = (char*)"base_link";
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  processPIDandPublish();
  readAndPublishIMU();
  readAndPublishRange();
  delay(5);
}
