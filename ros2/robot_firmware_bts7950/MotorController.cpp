
#include "MotorController.h"

MotorController::MotorController(const MotorPins& left, const MotorPins& right, float ppr, uint16_t pwm_max)
: _leftPins(left),
  _rightPins(right),
  _PPR(ppr),
  _pwm_max(pwm_max),
  _enc_left(left.ENC_A, left.ENC_B),
  _enc_right(right.ENC_A, right.ENC_B)
{}

void MotorController::begin() {
  analogWriteResolution(12); // Teensy 4.0: 0..4095 (ปรับตามต้องการ)

  // Left motor pins
  pinMode(_leftPins.RPWM, OUTPUT);
  pinMode(_leftPins.LPWM, OUTPUT);
  pinMode(_leftPins.REN,  OUTPUT);
  pinMode(_leftPins.LEN,  OUTPUT);

  // Right motor pins
  pinMode(_rightPins.RPWM, OUTPUT);
  pinMode(_rightPins.LPWM, OUTPUT);
  pinMode(_rightPins.REN,  OUTPUT);
  pinMode(_rightPins.LEN,  OUTPUT);

  // Enable driver channels (เปิดใช้งานตลอด)
  digitalWrite(_leftPins.REN, HIGH);
  digitalWrite(_leftPins.LEN, HIGH);
  digitalWrite(_rightPins.REN, HIGH);
  digitalWrite(_rightPins.LEN, HIGH);

  // Coast stop initially
  analogWrite(_leftPins.RPWM, 0);
  analogWrite(_leftPins.LPWM, 0);
  analogWrite(_rightPins.RPWM, 0);
  analogWrite(_rightPins.LPWM, 0);

  // Init timings
  _prevPID_ms = millis();
  _prevRPM_ms = millis();

  // Enc baselines
  _prevPl = _enc_left.read();
  _prevPr = _enc_right.read();
}

void MotorController::reset() {
  _sumErrL = _sumErrR = 0;
  _prev_errL = _prev_errR = 0;
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

  _state.rpm_left  = ((float)dPl / _PPR) / dt * 60.0f;
  _state.rpm_right = ((float)dPr / _PPR) / dt * 60.0f;

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
  // BTS7960: direction via which half-bridge gets PWM
  if (desired_rpm > 0) {
    // forward
    analogWrite(_leftPins.RPWM, (uint16_t)_state.pwm_left);
    analogWrite(_leftPins.LPWM, 0);
  } else if (desired_rpm < 0) {
    // reverse
    analogWrite(_leftPins.RPWM, 0);
    analogWrite(_leftPins.LPWM, (uint16_t)_state.pwm_left);
  } else {
    // coast stop
    analogWrite(_leftPins.RPWM, 0);
    analogWrite(_leftPins.LPWM, 0);
  }
}

void MotorController::driveRight_(float desired_rpm) {
  if (desired_rpm > 0) {
    analogWrite(_rightPins.RPWM, (uint16_t)_state.pwm_right);
    analogWrite(_rightPins.LPWM, 0);
  } else if (desired_rpm < 0) {
    analogWrite(_rightPins.RPWM, 0);
    analogWrite(_rightPins.LPWM, (uint16_t)_state.pwm_right);
  } else {
    analogWrite(_rightPins.RPWM, 0);
    analogWrite(_rightPins.LPWM, 0);
  }
}

void MotorController::update(float desiredRPM_left, float desiredRPM_right) {
  readEncoders_();

  unsigned long now = millis();
  unsigned long dt_ms = now - _prevPID_ms;
  if (dt_ms == 0) dt_ms = 1;
  double dt = dt_ms * 0.001;

  // update RPM feedback
  computeRPM_();

  // PID
  pidStepLeft_(desiredRPM_left,  _state.rpm_left,  dt);
  pidStepRight_(desiredRPM_right, _state.rpm_right, dt);

  // drive
  driveLeft_(desiredRPM_left);
  driveRight_(desiredRPM_right);

  _prevPID_ms = now;
}
``
