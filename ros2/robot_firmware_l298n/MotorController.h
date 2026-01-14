
#pragma once
#include <Arduino.h>
#include <Encoder.h>

// คลาสควบคุมมอเตอร์คู่ด้วย L298N + Encoder + PID
class MotorController {
public:
  struct Pins {
    // Left motor
    uint8_t l_dir1;
    uint8_t l_dir2;
    uint8_t l_pwm;
    // Right motor
    uint8_t r_dir1;
    uint8_t r_dir2;
    uint8_t r_pwm;
    // Encoders
    uint8_t l_enc_a;
    uint8_t l_enc_b;
    uint8_t r_enc_a;
    uint8_t r_enc_b;
  };

  struct Gains {
    float kp{5.0f};
    float ki{0.0004f};
    float kd{0.01f};
    float windup_guard{20.0f}; // anti-windup
  };

  struct State {
    // RPM feedback (last computed)
    float rpm_left{0.0f};
    float rpm_right{0.0f};
    // PWM output (0..pwm_max)
    float pwm_left{0.0f};
    float pwm_right{0.0f};
  };

  MotorController(const Pins& pins, float ppr, uint16_t pwm_max = 4095);

  void begin();

  // เรียกใช้งานทุกลูปเพื่ออัพเดต PID และสั่งมอเตอร์
  // desiredRPM_* เป็นค่าตั้งต้นเป็น rpm (บวก=เดินหน้า, ลบ=ถอย)
  void update(float desiredRPM_left, float desiredRPM_right);

  // อ่าน ticks ปัจจุบัน
  long leftTicks()  const { return _pl; }
  long rightTicks() const { return _pr; }

  // อ่านสถานะล่าสุด (rpm/pwm ฯลฯ)
  const State& getState() const { return _state; }

  // ปรับ Gains ระหว่างรัน
  void setGains(const Gains& g) { _g = g; }
  Gains getGains() const { return _g; }

  // รีเซ็ต Integral/สภาวะ PID
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
  void driveLeft_(float desired_rpm);
  void driveRight_(float desired_rpm);

  // PID ของซ้าย/ขวา
  void pidStepLeft_(double set_rpm, double meas_rpm, double dt_s);
  void pidStepRight_(double set_rpm, double meas_rpm, double dt_s);

private:
  Pins _pins;
  Gains _g;

  const float _PPR;
  const uint16_t _pwm_max;

  // Encoders
  Encoder _enc_left;
  Encoder _enc_right;

  // Tick counters
  long _pl{0}, _pr{0};
  long _prevPl{0}, _prevPr{0};

  // เวลาคำนวณ
  unsigned long _prevPID_ms{0};
  unsigned long _prevRPM_ms{0};

  // PID states Left
  double _errL{0}, _prev_errL{0}, _sumErrL{0};
  // PID states Right
  double _errR{0}, _prev_errR{0}, _sumErrR{0};

  // Feedback cached
  State _state;
};
