
#pragma once
#include <Arduino.h>
#include <Encoder.h>

class MotorController {
public:
  struct MotorPins {
    // BTS7960 half-bridges + enables
    uint8_t RPWM, LPWM, REN, LEN;
    // Encoder
    uint8_t ENC_A, ENC_B;
  };

  struct Gains {
    float kp{5.0f};
    float ki{0.0004f};
    float kd{0.01f};
    float windup_guard{20.0f};
  };

  struct State {
    float rpm_left{0.0f};
    float rpm_right{0.0f};
    float pwm_left{0.0f};   // 0..pwm_max
    float pwm_right{0.0f};  // 0..pwm_max
  };

  MotorController(const MotorPins& left, const MotorPins& right, float ppr, uint16_t pwm_max = 4095);

  void begin();
  void update(float desiredRPM_left, float desiredRPM_right);

  long leftTicks()  const { return _pl; }
  long rightTicks() const { return _pr; }
  const State& getState() const { return _state; }

  void setGains(const Gains& g) { _g = g; }
  Gains getGains() const { return _g; }
  void reset();

private:
  template<typename T>
  static inline T clamp(T v, T lo, T hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  void readEncoders_();
  void computeRPM_();

  // ขับ BTS7960
  void driveLeft_(float desired_rpm);
  void driveRight_(float desired_rpm);

  // คำนวณ PID
  void pidStepLeft_(double set_rpm, double meas_rpm, double dt_s);
  void pidStepRight_(double set_rpm, double meas_rpm, double dt_s);

private:
  MotorPins _leftPins;
  MotorPins _rightPins;
  Gains _g;

  const float _PPR;
  const uint16_t _pwm_max;

  Encoder _enc_left;
  Encoder _enc_right;

  // ticks
  long _pl{0}, _pr{0};
  long _prevPl{0}, _prevPr{0};

  // เวลา
  unsigned long _prevPID_ms{0};
  unsigned long _prevRPM_ms{0};

  // PID states
  double _errL{0}, _prev_errL{0}, _sumErrL{0};
  double _errR{0}, _prev_errR{0}, _sumErrR{0};

  State _state;
};
