// Apollo ring stepper — control logic: command handling, velocity ramp, soft
// limits, homing, watchdog and status reporting.
//
// Nothing in here touches hardware. It talks to the board through the hal_*
// functions declared below, which ring_stepper.ino implements for the Arduino.
// A desktop harness can implement them too, which is what lets the same logic
// be tested without a board.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "ring_protocol.h"

namespace ring {

// ---- Hardware abstraction (implemented per platform) -----------------------
uint32_t hal_millis();
uint32_t hal_micros();
int hal_serial_read();                               // next byte, or -1 if none
void hal_serial_write(const uint8_t* data, uint8_t n);
bool hal_home_active();                              // raw switch, true on the negative half
void hal_set_enable(bool on);
void hal_set_step_rate(float steps_per_s);           // signed; 0 stops
int32_t hal_position();                              // signed step count since boot

const float STEPS_PER_RAD =
    MOTOR_STEPS_PER_REV * MICROSTEPS * BELT_RATIO / 6.28318531f;

class Controller {
 public:
  void begin() {
    hal_set_enable(false);
    hal_set_step_rate(0);
    lastCtlUs_ = hal_micros();
    lastStatusMs_ = hal_millis();
    sw_ = hal_home_active();
  }

  // Call from loop() as often as possible; it paces itself.
  void poll() {
    int c;
    uint8_t budget = 48;
    while (budget-- && (c = hal_serial_read()) >= 0) {
      if (parser_.feed((uint8_t)c)) handleFrame();
    }

    uint32_t nowUs = hal_micros();
    uint32_t elapsed = nowUs - lastCtlUs_;
    if (elapsed >= CONTROL_PERIOD_US) {
      lastCtlUs_ = nowUs;
      float dt = elapsed * 1e-6f;
      if (dt > 0.01f) dt = 0.01f;  // never take a huge step after a stall
      control(dt);
    }

    uint32_t nowMs = hal_millis();
    if ((uint32_t)(nowMs - lastStatusMs_) >= STATUS_PERIOD_MS) {
      lastStatusMs_ = nowMs;
      sendStatus();
    }
  }

  State state() const { return state_; }

 private:
  enum HomePhase : uint8_t { H_FAST_OFF, H_SEEK, H_APPROACH };

  void handleFrame() {
    switch (parser_.type) {
      case CMD_VEL:
        if (parser_.len == 4) {
          float v;
          memcpy(&v, parser_.payload, 4);
          if (!(v == v)) v = 0;  // NaN
          if (v > MAX_VEL_RAD_S) v = MAX_VEL_RAD_S;
          if (v < -MAX_VEL_RAD_S) v = -MAX_VEL_RAD_S;
          cmd_ = v * STEPS_PER_RAD;
          lastCmdMs_ = hal_millis();
          haveCmd_ = true;
        }
        break;
      case CMD_HOME:
        if (parser_.len == 0 && state_ != HOMING) startHoming();
        break;
      case CMD_ENABLE:
        if (parser_.len == 1) {
          if (parser_.payload[0]) {
            setEnabled(true);
          } else {
            // Releasing the motor loses the position: there is no encoder.
            setEnabled(false);
            vel_ = 0;
            hal_set_step_rate(0);
            state_ = UNHOMED;
          }
        }
        break;
      default:
        break;
    }
  }

  void setEnabled(bool on) {
    enabled_ = on;
    hal_set_enable(on);
  }

  void startHoming() {
    setEnabled(true);
    stepLoss_ = false;
    crossErr_ = 0;
    state_ = HOMING;
    enterPhase(sw_ ? H_FAST_OFF : H_SEEK);
  }

  void enterPhase(HomePhase p) {
    phase_ = p;
    phaseStart_ = hal_position();
  }

  // Returns the velocity target (steps/s) for the current homing phase.
  float homingTarget(int32_t raw) {
    const float seek = HOME_SEEK_VEL_RAD_S * STEPS_PER_RAD;
    const float approach = HOME_APPROACH_VEL_RAD_S * STEPS_PER_RAD;
    int32_t travelled = raw - phaseStart_;
    if (travelled < 0) travelled = -travelled;
    if (travelled > (int32_t)(HOME_MAX_TRAVEL_RAD * STEPS_PER_RAD)) {
      state_ = FAULT;  // switch never changed: unplugged, or the ring is jammed
      return 0;
    }
    switch (phase_) {
      case H_FAST_OFF:  // started on the negative half: run positive until clear
        if (!sw_) { enterPhase(H_SEEK); return 0; }
        return seek;
      case H_SEEK:      // run negative until the flag is found
        if (sw_) { enterPhase(H_APPROACH); return 0; }
        return -seek;
      default:          // creep positive; the release edge is zero
        if (!sw_) {
          offset_ = raw;
          state_ = READY;
          haveCmd_ = false;  // ignore any velocity that arrived while homing
          return 0;
        }
        return approach;
    }
  }

  // Keeps the ring able to stop before a soft limit: the allowed speed toward
  // a limit falls with the square root of the distance left to it.
  float clampToLimits(float target, float pos) {
    const float softMax = SOFT_LIMIT_RAD * STEPS_PER_RAD;
    const float k = 2.0f * 0.8f * MAX_ACCEL_RAD_S2 * STEPS_PER_RAD;  // 80 % of the ramp, for margin
    float toPos = softMax - pos;
    float toNeg = softMax + pos;
    float vPos = toPos > 0 ? sqrtf(k * toPos) : 0;
    float vNeg = toNeg > 0 ? sqrtf(k * toNeg) : 0;
    if (target > vPos) { target = vPos; flags_ |= F_LIMIT_POS; }
    if (target < -vNeg) { target = -vNeg; flags_ |= F_LIMIT_NEG; }
    return target;
  }

  void control(float dt) {
    int32_t raw = hal_position();
    bool prevSw = sw_;
    debounce(hal_home_active());

    flags_ = 0;
    float target = 0;

    if (state_ == HOMING) {
      target = homingTarget(raw);
    } else if (state_ == READY) {
      float pos = (float)(raw - offset_);
      bool timedOut = !haveCmd_ || (uint32_t)(hal_millis() - lastCmdMs_) > WATCHDOG_MS;
      if (timedOut) flags_ |= F_WATCHDOG;
      target = clampToLimits(timedOut ? 0 : cmd_, pos);

      // Free consistency check: every pass through centre flips the switch,
      // and the step count should read about zero when it does.
      if (sw_ != prevSw) {
        crossErr_ = pos / STEPS_PER_RAD;
        if (fabsf(crossErr_) > STEP_LOSS_THRESH_RAD) stepLoss_ = true;
      }
    }

    float maxDv = MAX_ACCEL_RAD_S2 * STEPS_PER_RAD * dt;
    float dv = target - vel_;
    if (dv > maxDv) dv = maxDv;
    if (dv < -maxDv) dv = -maxDv;
    vel_ += dv;
    if (!enabled_) vel_ = 0;
    hal_set_step_rate(vel_);

    if (sw_) flags_ |= F_HOME_SW;
    if (stepLoss_) flags_ |= F_STEP_LOSS;
    if (enabled_) flags_ |= F_ENABLED;
    statusFlags_ = flags_;
  }

  void debounce(bool rawSw) {
    if (rawSw == sw_) {
      swCount_ = 0;
    } else if (++swCount_ >= HOME_DEBOUNCE_TICKS) {
      sw_ = rawSw;
      swCount_ = 0;
    }
  }

  void sendStatus() {
    float angle = (float)(hal_position() - offset_) / STEPS_PER_RAD;
    float vel = vel_ / STEPS_PER_RAD;
    uint8_t p[STATUS_LEN];
    memcpy(p, &angle, 4);
    memcpy(p + 4, &vel, 4);
    p[8] = state_;
    p[9] = statusFlags_;
    memcpy(p + 10, &crossErr_, 4);
    uint8_t frame[MAX_FRAME];
    hal_serial_write(frame, encode(STATUS, p, STATUS_LEN, frame));
  }

  Parser parser_;
  State state_ = UNHOMED;
  HomePhase phase_ = H_SEEK;
  int32_t phaseStart_ = 0;
  int32_t offset_ = 0;      // raw step count at the home edge
  float cmd_ = 0;           // commanded velocity, steps/s
  float vel_ = 0;           // ramped velocity being output, steps/s
  float crossErr_ = 0;      // angle the step count read at the last centre crossing, rad
  uint32_t lastCmdMs_ = 0;
  uint32_t lastCtlUs_ = 0;
  uint32_t lastStatusMs_ = 0;
  uint8_t flags_ = 0;
  uint8_t statusFlags_ = 0;
  uint8_t swCount_ = 0;
  bool sw_ = false;         // debounced home switch
  bool haveCmd_ = false;
  bool enabled_ = false;
  bool stepLoss_ = false;
};

}  // namespace ring
