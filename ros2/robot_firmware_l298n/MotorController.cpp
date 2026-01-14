
#include "MotorController.h"

MotorController::MotorController(const Pins& pins, float ppr, uint16_t pwm_max)
: _pins(pins),
  _PPR(ppr),
  _pwm_max(pwm_max),
  _enc_left(pins.l_enc_a, pins.l_enc_b),
  _enc_right(pins.r_enc_a, pins.r_enc_b)
{}

void MotorController::begin() {
  // ตั้งค่า PWM resolution สำหรับ Teensy 4.0
  analogWriteResolution(12); // 0..4095

  // ตั้ง I/O
  pinMode(_pins.l_dir1, OUTPUT);
  pinMode(_pins.l_dir2, OUTPUT);
  pinMode(_pins.l_pwm,  OUTPUT);

  pinMode(_pins.r_dir1, OUTPUT);
  pinMode(_pins.r_dir2, OUTPUT);
  pinMode(_pins.r_pwm,  OUTPUT);

  // Initial states
  digitalWrite(_pins.l_dir1, LOW);
  digitalWrite(_pins.l_dir2, LOW);
  analogWrite(_pins.l_pwm, 0);

  digitalWrite(_pins.r_dir1, LOW);
  digitalWrite(_pins.r_dir2, LOW);
  analogWrite(_pins.r_pwm, 0);

  // Init timing
  _prevPID_ms = millis();
  _prevRPM_ms = millis();

  // Init encoder baselines
  _prevPl = _enc_left.read();
  _prevPr = _enc_right.read();
}

void MotorController::reset() {
  _sumErrL = 0;
  _sumErrR = 0;
  _prev_errL = 0;
  _prev_errR = 0;
}

void MotorController::readEncoders_() {
  _pl = _enc_left.read();
  _pr = _enc_right.read();
}

void MotorController::computeRPM_() {
  unsigned long now = millis();
  unsigned long dt_ms = now - _prevRPM_ms;
  if (dt_ms == 0) dt_ms = 1;
  float dt = dt_ms * 0.001f;

  long dPl = _pl - _prevPl;
  long dPr = _pr - _prevPr;

  _state.rpm_left  = ( (float)dPl / _PPR) / dt * 60.0f;
  _state.rpm_right = ( (float)dPr / _PPR) / dt * 60.0f;

  _prevPl = _pl;
  _prevPr = _pr;
  _prevRPM_ms = now;
}

void MotorController::pidStepLeft_(double set_rpm, double meas_rpm, double dt_s) {
  double err = fabs(set_rpm) - fabs(meas_rpm);

  _sumErrL += err * dt_s;
  _sumErrL = clamp(_sumErrL, -(double)_g.windup_guard, (double)_g.windup_guard);

  double dErr = (err - _prev_errL) / (dt_s > 0 ? dt_s : 1e-3);
  double u = _g.kp * err + _g.ki * _sumErrL + _g.kd * dErr;

  if (u < 0) u = 0;
  u = clamp(u, 0.0, (double)_pwm_max);

  _state.pwm_left = (float)u;
  _prev_errL = err;
}

void MotorController::pidStepRight_(double set_rpm, double meas_rpm, double dt_s) {
  double err = fabs(set_rpm) - fabs(meas_rpm);

  _sumErrR += err * dt_s;
  _sumErrR = clamp(_sumErrR, -(double)_g.windup_guard, (double)_g.windup_guard);

  double dErr = (err - _prev_errR) / (dt_s > 0 ? dt_s : 1e-3);
  double u = _g.kp * err + _g.ki * _sumErrR + _g.kd * dErr;

  if (u < 0) u = 0;
  u = clamp(u, 0.0, (double)_pwm_max);

  _state.pwm_right = (float)u;
  _prev_errR = err;
}

void MotorController::driveLeft_(float desired_rpm) {
  int dir = 0;
  if (desired_rpm > 0) dir = 1;
  else if (desired_rpm < 0) dir = -1;

  if (dir > 0) {
    digitalWrite(_pins.l_dir1, HIGH);
    digitalWrite(_pins.l_dir2, LOW);
    analogWrite(_pins.l_pwm, (uint16_t)_state.pwm_left);
  } else if (dir < 0) {
    digitalWrite(_pins.l_dir1, LOW);
    digitalWrite(_pins.l_dir2, HIGH);
    analogWrite(_pins.l_pwm, (uint16_t)_state.pwm_left);
  } else {
    // coast/brake
    digitalWrite(_pins.l_dir1, LOW);
    digitalWrite(_pins.l_dir2, LOW);
    analogWrite(_pins.l_pwm, 0);
  }
}

void MotorController::driveRight_(float desired_rpm) {
  int dir = 0;
  if (desired_rpm > 0) dir = 1;
  else if (desired_rpm < 0) dir = -1;

  if (dir > 0) {
    digitalWrite(_pins.r_dir1, HIGH);
    digitalWrite(_pins.r_dir2, LOW);
    analogWrite(_pins.r_pwm, (uint16_t)_state.pwm_right);
  } else if (dir < 0) {
    digitalWrite(_pins.r_dir1, LOW);
    digitalWrite(_pins.r_dir2, HIGH);
    analogWrite(_pins.r_pwm, (uint16_t)_state.pwm_right);
  } else {
    digitalWrite(_pins.r_dir1, LOW);
    digitalWrite(_pins.r_dir2, LOW);
    analogWrite(_pins.r_pwm, 0);
  }
}

void MotorController::update(float desiredRPM_left, float desiredRPM_right) {
  // 1) อ่าน encoder
  readEncoders_();

  // 2) คำนวณ dt สำหรับ PID
  unsigned long now = millis();
  unsigned long dt_ms = now - _prevPID_ms;
  if (dt_ms == 0) dt_ms = 1;
  double dt = dt_ms * 0.001;

  // 3) อัปเดต RPM feedback (ใช้ช่วงเวลาอีกตัวเพื่อราบรื่น)
  computeRPM_();

  // 4) คำนวณ PID left/right (setpoint = desiredRPM_*)
  pidStepLeft_(desiredRPM_left,  _state.rpm_left,  dt);
  pidStepRight_(desiredRPM_right, _state.rpm_right, dt);

  // 5) ขับ L298N ตามทิศทาง + PWM คำนวนแล้ว
  driveLeft_(desiredRPM_left);
  driveRight_(desiredRPM_right);

  // 6) อัปเดตเวลาสำหรับรอบหน้า
  _prevPID_ms = now;
}

