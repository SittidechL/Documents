include <micro_ros_arduino.h>
include <stdio.h>

include <rcl/rcl.h>
include <rcl/error_handling.h>
include <rclc/rclc.h>
include <rclc/executor.h>

include <std_msgs/msg/int32.h>
include <std_msgs/msg/float32.h>

include <geometry_msgs/msg/twist.h>
include <geometry_msgs/msg/vector3.h>

include <rmw_microros/rmw_microros.h>
include <Encoder.h>

/IMU message
include <sensor_msgs/msg/imu.h>
include <sensor_msgs/msg/magnetic_field.h>


/////////////// pin setup /////////////////

define LED_PIN 13
define r_dir1_pin 1
define r_dir2_pin 2
define r_pwm_pin 0

define l_dir1_pin 3
define l_dir2_pin 4
define l_pwm_pin 5

define l_encoder_a 7
define l_encoder_b 8

define r_encoder_a 9
define r_encoder_b 10

/IMU library
include "mpu9250.h"
fs::Mpu9250 imu(&Wire, 0x68);
/https://github.com/ElectronicCats/mpu6050
///////////////Variables/////////////////////

cl_publisher_t left_tick_pub;
td_msgs__msg__Int32 left_tick;

cl_publisher_t right_tick_pub;
td_msgs__msg__Int32 right_tick;

cl_publisher_t feedback_vel_pub;
eometry_msgs__msg__Twist feedback_vel_msg;

cl_subscription_t wr_command_sub;
td_msgs__msg__Float32 wr_command;

cl_subscription_t wl_command_sub;
td_msgs__msg__Float32 wl_command;

cl_subscription_t pid_gain_sub;
eometry_msgs__msg__Vector3 pid_gain;

/IMU publisher
cl_publisher_t imu_pub;
ensor_msgs__msg__Imu imu_msg;

cl_publisher_t mag_pub;
ensor_msgs__msg__MagneticField mag_msg;

cl_timer_t timer;
cl_node_t node;
cl_allocator_t allocator;
clc_support_t support;
clc_executor_t executor;

ool micro_ros_init_successful;

nsigned int SPIN_FREQ = 20;
loat windup_guard = 20;

/ PID control parameters
loat kp = 5;
loat ki = 0.0004;
loat kd = 0.01;

/Motor pulse parameters
loat PPR = 840 * 4;
ong pl, pr;

/Left Wheel control parameter
nsigned long curTimeL, prevTimeL, dtL;
ong curTickL, prevTickL, diffTickL;
ouble errL, prev_errL, sumErrL, dErrL, setRPML;
ouble control_outL;
ouble measuredRPML;
ouble desiredRPMR, desiredRPML;

*Right Wheel Control*/
nsigned long curTimeR, prevTimeR, dtR;
ong curTickR, prevTickR, diffTickR;
ouble errR, prev_errR, sumErrR, dErrR, setRPMR;
ouble control_outR;
ouble measuredRPMR;

/Find max value function
loat max(float num1, float num2)

 return (num1 > num2) ? num1 : num2;


/Find min value function
loat min(float num1, float num2)

 return (num1 > num2) ? num2 : num1;


/Left Motor Controller Function
oid computePIDL(double control_cmd,
                long inTick)

 int dirs = 1;  //1 for forward direction
 //int dirm = 1;  //1 for forward direction

 curTickL = inTick;
 curTimeL = millis();

 setRPML = control_cmd;
 dirs = control_cmd / abs(control_cmd);

 diffTickL = curTickL - prevTickL;
 dtL =  (curTimeL - prevTimeL);

 measuredRPML = ((diffTickL / PPR) / (dtL * 0.001)) * 60;

 errL = abs(setRPML) - abs(measuredRPML);

 sumErrL += errL * dtL;

 dErrL = (errL - prev_errL) / dtL;

 control_outL = kp * errL + ki * sumErrL + kd * dErrL;

 if (control_outL < 0) {
   control_outL = 0;
 }
 prev_errL = errL;
 prevTickL = curTickL;
 prevTimeL = curTimeL;

 if (dirs > 0.5)
 {
   digitalWrite(l_dir1_pin, HIGH);
   digitalWrite(l_dir2_pin, LOW);
   analogWrite(l_pwm_pin, control_outL);
 }
 else if (dirs < -0.5)
 {
   digitalWrite(l_dir1_pin, LOW);
   digitalWrite(l_dir2_pin, HIGH);
   analogWrite(l_pwm_pin, control_outL);

 }
 else
 {
   digitalWrite(l_dir1_pin, LOW);
   digitalWrite(l_dir2_pin, LOW);
   analogWrite(l_pwm_pin, 0);
 }

 



/Right Motor Controller Function
oid computePIDR(double control_cmd, long inTick)

 int dirs = 1;
 //int dirm = 1;

 curTickR = inTick;
 curTimeR = millis();

 setRPMR = control_cmd;
 dirs = control_cmd / abs(control_cmd);

 diffTickR = curTickR - prevTickR;
 dtR =  (curTimeR - prevTimeR);

 measuredRPMR = ((diffTickR / PPR) / (dtR * 0.001)) * 60;

 errR = abs(setRPMR) - abs(measuredRPMR);

 sumErrR += errR * dtR;

 dErrR = (errR - prev_errR) / dtR;

 control_outR = kp * errR + ki * sumErrR + kd * dErrR;

 if (control_outR < 0) {
   control_outR = 0;
 }
 prev_errR = errR;
 prevTickR = curTickR;
 prevTimeR = curTimeR;

 if (dirs > 0.5)
 {
   digitalWrite(r_dir1_pin, HIGH);
   digitalWrite(r_dir2_pin, LOW);
   analogWrite(r_pwm_pin, control_outR);
 }
 else if (dirs < -0.5)
 {
   digitalWrite(r_dir1_pin, LOW);
   digitalWrite(r_dir2_pin, HIGH);
   analogWrite(r_pwm_pin, control_outR);

 }
 else
 {
   digitalWrite(r_dir1_pin, LOW);
   digitalWrite(r_dir2_pin, LOW);
   analogWrite(r_pwm_pin, 0);
 }



oid pid_gain_callback(const void * msgin)

 const geometry_msgs__msg__Vector3 * gain_value = (const geometry_msgs__msg__Vector3 *)msgin;

 //Uncomment for tuning
 /*kp = gain_value->x;
 
 ki = gain_value->y;
 kd = gain_value->z;*/


/Right wheel command callback function
oid rightwheel_callback(const void * msgin)

 const std_msgs__msg__Float32 * power = (const std_msgs__msg__Float32 *)msgin;

 desiredRPMR = power->data;




/Left Wheel Command Callback Function
oid leftwheel_callback(const void * msgin)

 const std_msgs__msg__Float32 * power = (const std_msgs__msg__Float32 *)msgin;

 desiredRPML = power->data;




/Timer callback function
oid timer_callback(rcl_timer_t *timer)

 rcl_publish(&feedback_vel_pub, &feedback_vel_msg, NULL);
 readIMU();
 RCLC_UNUSED(timer);


oid readIMU()

 float ax,ay,az;
 float gx,gy,gz;
 float mx,my,mz;
 if (imu.Read()) {
   ax = imu.accel_x_mps2();
   ay = imu.accel_y_mps2();
   az = imu.accel_z_mps2();
   gx = imu.gyro_x_radps();
   gy = imu.gyro_y_radps();
   gz = imu.gyro_z_radps();
   mx = imu.mag_x_ut();
   my = imu.mag_y_ut();
   mz = imu.mag_z_ut();

   imu_msg.linear_acceleration.y = ax-0.45;
   imu_msg.linear_acceleration.x= ay-0.45;
   imu_msg.linear_acceleration.z = -az;
   imu_msg.angular_velocity.x = -gx;
   imu_msg.angular_velocity.y = -gy;
   imu_msg.angular_velocity.z = -gz-0.02;

   mag_msg.magnetic_field.x = mx;
   mag_msg.magnetic_field.y = my;
   mag_msg.magnetic_field.z = mz;

   imu_msg.angular_velocity_covariance[0] =  0.0001;
   imu_msg.angular_velocity_covariance[4] =  0.0001;
   imu_msg.angular_velocity_covariance[8] =  0.00001;
           
   imu_msg.linear_acceleration_covariance[0] =  0.0001;
   imu_msg.linear_acceleration_covariance[4] =  0.0001;
   imu_msg.linear_acceleration_covariance[8] =  0.00001;

   mag_msg.magnetic_field_covariance[0]= 0.0001;
   mag_msg.magnetic_field_covariance[4]= 0.0001;
   mag_msg.magnetic_field_covariance[8]= 0.0001;
   
   rcl_publish(&imu_pub, &imu_msg, NULL);
   rcl_publish(&mag_pub, &mag_msg, NULL);

 }
 


/Create entities function
ool create_entities()

 allocator = rcl_get_default_allocator();


 rclc_support_init(&support, 0, NULL, &allocator);

 rclc_node_init_default(&node, "base_control_node", "", &support);

 rclc_subscription_init_default(
   &pid_gain_sub,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
   "pid_gain");

 rclc_subscription_init_default(
   &wl_command_sub,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
   "wheel_command_left");

 rclc_subscription_init_default(
   &wr_command_sub,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
   "wheel_command_right");

 rclc_publisher_init_default(
   &left_tick_pub,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
   "left_tick");

 rclc_publisher_init_default(
   &right_tick_pub,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
   "right_tick");

 rclc_timer_init_default(
   &timer,
   &support,
   RCL_MS_TO_NS(50),
   timer_callback);

 rclc_publisher_init_default(
   &feedback_vel_pub,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
   "feedback_vel");

 rclc_publisher_init_default(
   &imu_pub,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
   "imu/raw");

 rclc_publisher_init_default(
   &mag_pub,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
   "mag/raw");
   

 rclc_executor_init(&executor, &support.context, 4, &allocator);
 rclc_executor_add_timer(&executor, &timer);
 rclc_executor_add_subscription(&executor, &pid_gain_sub, &pid_gain, &pid_gain_callback, ON_NEW_DATA);
 rclc_executor_add_subscription(&executor, &wr_command_sub, &wr_command, &rightwheel_callback, ON_NEW_DATA);
 rclc_executor_add_subscription(&executor, &wl_command_sub, &wl_command, &leftwheel_callback, ON_NEW_DATA);

 micro_ros_init_successful = true;


oid destroy_entities()

 rcl_publisher_fini(&left_tick_pub, &node);
 rcl_publisher_fini(&right_tick_pub, &node);
 rcl_publisher_fini(&feedback_vel_pub, &node);
 rcl_subscription_fini(&wr_command_sub, &node);
 rcl_subscription_fini(&wl_command_sub, &node);
 rcl_subscription_fini(&pid_gain_sub, &node);
  rcl_publisher_fini(&imu_pub, &node);
 rcl_publisher_fini(&mag_pub, &node);
 rcl_node_fini(&node);
 rcl_timer_fini(&timer);
 rclc_executor_fini(&executor);
 rclc_support_fini(&support);

 micro_ros_init_successful = false;


oid setup() {
 set_microros_transports();
 pinMode(LED_PIN, OUTPUT);
 digitalWrite(LED_PIN, HIGH);

 //PWM pin setup
 pinMode(r_dir1_pin, OUTPUT);
 pinMode(r_dir2_pin, OUTPUT);
 pinMode(r_pwm_pin, OUTPUT);
 pinMode(l_dir1_pin, OUTPUT);
 pinMode(l_dir2_pin, OUTPUT);
 pinMode(l_pwm_pin, OUTPUT);

  //IMU setup
 Wire.begin();
 Wire.setClock(400000);
 if (!imu.Begin()) {

   while(1) {}
 }
 /* Set the sample rate divider */
 if (!imu.ConfigSrd(19)) {
   while(1) {}
 }

 micro_ros_init_successful = false;



ncoder r_encoder(9, 10);
ncoder l_encoder(7, 8);
ong old_pl = -999;
ong old_pr = -999;

ong curRPM_time;
ong prevRPM_time;
ong curPl;
ong curPr;
ong prevPl;
ong prevPr;
ong diffRPM_time;
loat RPM_l, RPM_r;

oid counter_RPM(long inPl, long inPr) {

 curPl = inPl;
 curPr = inPr;
 curRPM_time = millis();
 diffRPM_time = curRPM_time - prevRPM_time ;
 RPM_l = (((curPl - prevPl) / PPR) / (diffRPM_time * 0.001)) * 60;
 RPM_r = (((curPr - prevPr) / PPR) / (diffRPM_time * 0.001)) * 60;


 prevPl = curPl;
 prevPr = curPr;
 prevRPM_time = curRPM_time;
 feedback_vel_msg.linear.x = RPM_l; // RPM_l;
 feedback_vel_msg.linear.y = RPM_r;
 feedback_vel_msg.angular.x = control_outL;
 feedback_vel_msg.angular.y = control_outR;
 feedback_vel_msg.angular.z = ki;



oid counter_tick()

 pl = l_encoder.read();

 if (pl != old_pl) {
   old_pl = pl;
   Serial.println(pl);
 }

 pr = r_encoder.read();

 if (pr != old_pr) {
   old_pr = pr;
 }

 left_tick.data = pl;
 right_tick.data = pr;

 rcl_publish(&right_tick_pub, &right_tick, NULL);
 rcl_publish(&left_tick_pub, &left_tick, NULL);




oid loop()



 uint32_t delay_rec = 100000;
 if (RMW_RET_OK == rmw_uros_ping_agent(50, 2))
 {
   delay_rec = 5000;
   if (!micro_ros_init_successful) {
     create_entities();
   } else {
     counter_tick();

     computePIDR(desiredRPMR, pr);
     computePIDL(desiredRPML, pl);
     counter_RPM(pl, pr);

     rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
   }
 }
 else if (micro_ros_init_successful)
 {
   destroy_entities();
 }

 delayMicroseconds(delay_rec);

