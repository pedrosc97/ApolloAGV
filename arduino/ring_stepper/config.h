// Apollo ring stepper — wiring and tuning.
// Boards: Arduino MKR WiFi 1010 (3.3 V logic) or Uno / Nano (5 V logic).
// Everything you are likely to change lives in this file.
//
// MKR WiFi 1010 notes:
//   - Its pins are 3.3 V and NOT 5 V tolerant. Power the TMC2209's logic
//     supply (VIO / VDD) from the MKR's VCC pin (3.3 V), never from 5 V.
//   - Power the board over USB from the Pi. VIN takes 5 V only; do not feed
//     it from the 12 V or 18 V rails.
#pragma once

// ---- Pins -----------------------------------------------------------------
// The same pin numbers work on both boards. Pins 0 and 1 (MKR) or 2 and 3
// (Uno / Nano) are left free: they are interrupt pins an encoder would need
// if one is added later.
#define PIN_STEP 4         // -> TMC2209 STEP
#define PIN_DIR 5          // -> TMC2209 DIR
#define PIN_EN 6           // -> TMC2209 EN (active low)
#define PIN_HOME 7         // home switch to GND, internal pull-up

// Uno / Nano only: STEP is toggled with direct port access inside the timer
// interrupt. These two must describe the same pin as PIN_STEP (D4 = PORTD
// bit 4). The MKR ignores them.
#define STEP_PORT PORTD
#define STEP_BIT 4

// ---- Mechanics ------------------------------------------------------------
#define MOTOR_STEPS_PER_REV 200.0f   // 1.8 deg NEMA 17
#define MICROSTEPS 8.0f              // TMC2209 standalone: MS1 = MS2 = GND -> 1/8
#define BELT_RATIO 3.0f              // ring pulley teeth / motor pulley teeth
#define DIR_INVERT 0                 // set to 1 if positive velocity turns the ring clockwise (seen from above)

// ---- Home switch ----------------------------------------------------------
// There is no encoder, so the home switch is the only absolute reference.
// The firmware expects a HALF flag: the switch reads "active" over the whole
// negative half of travel and "inactive" over the positive half, so one read
// tells it which way centre is. Zero is the active -> inactive edge, always
// approached slowly in the positive direction.
// If your flag covers the positive half instead, flip HOME_ACTIVE_LOW.
#define HOME_ACTIVE_LOW 1            // 1: pin reads LOW on the negative half
#define HOME_DEBOUNCE_TICKS 3        // consecutive 1 ms samples that must agree

// ---- Motion (all at the ring, not the motor) --------------------------------
#define SOFT_LIMIT_RAD 2.618f        // +/-150 deg (hard travel is +/-170 deg)
#define MAX_VEL_RAD_S 3.0f           // ~172 deg/s at the ring, ~86 rpm at the motor
#define MAX_ACCEL_RAD_S2 20.0f       // matches the torque-sizing example in the spec
#define HOME_SEEK_VEL_RAD_S 0.5f
#define HOME_APPROACH_VEL_RAD_S 0.1f
#define HOME_MAX_TRAVEL_RAD 3.32f    // 190 deg: give up (FAULT) if no edge within this
#define STEP_LOSS_THRESH_RAD 0.052f  // 3 deg: centre-crossing error that raises the step-loss flag

// ---- Link -----------------------------------------------------------------
// SERIAL_BAUD, WATCHDOG_MS and STATUS_PERIOD_MS are shared with the Pi, so they live in
// comms/protocol.yaml and arrive as ring:: constants through the generated ring_protocol.h.
#define CONTROL_PERIOD_US 1000       // 1 kHz ramp / limit / homing update
