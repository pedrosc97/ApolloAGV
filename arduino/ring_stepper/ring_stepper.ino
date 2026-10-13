// Apollo ring stepper — Arduino firmware.
//
// Boards: Arduino MKR WiFi 1010 (SAMD21, 3.3 V logic) and Uno / Nano
// (ATmega328P, 5 V logic). The board is picked up automatically at compile
// time; only the step-pulse timer differs between them.
//
// Drives the ring NEMA 17 through a TMC2209 (STEP/DIR/EN), open loop: there
// is no encoder, so the ring angle is the step count since the last homing.
// The Pi sends a ring velocity; this sketch ramps to it, respects the soft
// limits, homes against the centre switch, and stops by itself if the Pi goes
// quiet. Wiring and tuning are in config.h, the control logic in ring_core.h,
// the serial framing in ring_protocol.h.
//
// This file is only the hardware layer: pins, the step-pulse timer, serial.
//
// How steps are made: a 16-bit hardware timer counts up to a "top" value and
// fires an interrupt, which emits one step. The top value is the step period,
// so step timing is exact rather than quantised to a polling loop. Below
// MIN_STEP_RATE the period no longer fits in 16 bits and the ring is stopped.

#include "config.h"
#include "ring_protocol.h"
#include "ring_core.h"

static volatile int32_t g_position = 0;
static volatile int8_t g_stepDir = 1;
static volatile bool g_running = false;
static bool g_dirPositive = true;

// ---- Per-board timer layer --------------------------------------------------
// Each board provides: TIMER_HZ, MIN_STEP_RATE, timerBegin(), timerCount(),
// timerSetCount(), timerTop(), timerSetTop(), the interrupt handler, and a
// pair of macros that mask interrupts around a critical section.

#if defined(ARDUINO_ARCH_AVR)

// Timer1 at F_CPU / 64 = 250 kHz (4 us per count), CTC mode.
static const float TIMER_HZ = F_CPU / 64.0f;
static const float MIN_STEP_RATE = 4.0f;  // 0.005 rad/s at the ring

#define CRITICAL_BEGIN() uint8_t savedIrq_ = SREG; cli()
#define CRITICAL_END() SREG = savedIrq_

static void timerBegin() {
  cli();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);  // CTC on OCR1A, prescaler 64
  TCNT1 = 0;
  OCR1A = 65535;
  TIMSK1 = _BV(OCIE1A);
  sei();
}
static inline uint16_t timerCount() { return TCNT1; }
static inline void timerSetCount(uint16_t c) { TCNT1 = c; }
static inline uint16_t timerTop() { return OCR1A; }
static inline void timerSetTop(uint16_t t) { OCR1A = t; }

ISR(TIMER1_COMPA_vect) {
  if (!g_running) return;
  STEP_PORT |= _BV(STEP_BIT);
  g_position += g_stepDir;  // the 32-bit add also stretches the pulse well past the TMC2209's 100 ns minimum
  STEP_PORT &= ~_BV(STEP_BIT);
}

#elif defined(ARDUINO_ARCH_SAMD)

// TC4 in 16-bit match-frequency mode at 48 MHz / 256 = 187.5 kHz (5.3 us per
// count). TC4 is otherwise only used by analogWrite() on a few pins, which
// this sketch never calls.
static const float TIMER_HZ = F_CPU / 256.0f;
static const float MIN_STEP_RATE = 3.0f;  // 0.004 rad/s at the ring

#define CRITICAL_BEGIN() uint32_t savedIrq_ = __get_PRIMASK(); __disable_irq()
#define CRITICAL_END() __set_PRIMASK(savedIrq_)

static inline void tcSync() {
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {}
}

static void timerBegin() {
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY) {}

  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC4->COUNT16.CTRLA.bit.SWRST) {}
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV256;
  tcSync();
  TC4->COUNT16.CC[0].reg = 65535;
  tcSync();
  TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  NVIC_SetPriority(TC4_IRQn, 0);
  NVIC_EnableIRQ(TC4_IRQn);
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  tcSync();
}
static inline uint16_t timerCount() {
  TC4->COUNT16.READREQ.reg = TC_READREQ_RREQ | TC_READREQ_ADDR(0x10);  // latch COUNT for reading
  tcSync();
  return TC4->COUNT16.COUNT.reg;
}
static inline void timerSetCount(uint16_t c) {
  TC4->COUNT16.COUNT.reg = c;
  tcSync();
}
static inline uint16_t timerTop() { return TC4->COUNT16.CC[0].reg; }
static inline void timerSetTop(uint16_t t) {
  TC4->COUNT16.CC[0].reg = t;
  tcSync();
}

void TC4_Handler() {
  TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
  if (!g_running) return;
  digitalWrite(PIN_STEP, HIGH);  // digitalWrite takes about a microsecond here, which sets the pulse width
  g_position += g_stepDir;
  digitalWrite(PIN_STEP, LOW);
}

#else
#error "Unsupported board: add a timer layer for it next to the AVR and SAMD ones."
#endif

static const uint16_t MIN_PERIOD = 16;  // caps the step rate at 10 kHz or more, far above MAX_VEL

static void writeDir(bool positive) {
  digitalWrite(PIN_DIR, (positive != (bool)DIR_INVERT) ? HIGH : LOW);
}

// ---- Hardware abstraction used by ring_core.h --------------------------------
namespace ring {

uint32_t hal_millis() { return millis(); }
uint32_t hal_micros() { return micros(); }
int hal_serial_read() { return Serial.read(); }
void hal_set_enable(bool on) { digitalWrite(PIN_EN, on ? LOW : HIGH); }

// Status frames are dropped rather than queued when the Pi is not reading,
// so a stalled or absent host can never block the control loop.
void hal_serial_write(const uint8_t* data, uint8_t n) {
#if defined(ARDUINO_ARCH_SAMD)
  if (!Serial.dtr()) return;  // native USB: nobody has the port open
#else
  if (Serial.availableForWrite() < n) return;
#endif
  Serial.write(data, n);
}

bool hal_home_active() {
  return (digitalRead(PIN_HOME) == LOW) == (bool)HOME_ACTIVE_LOW;
}

int32_t hal_position() {
  CRITICAL_BEGIN();
  int32_t p = g_position;
  CRITICAL_END();
  return p;
}

void hal_set_step_rate(float stepsPerSec) {
  bool positive = stepsPerSec >= 0;
  float mag = positive ? stepsPerSec : -stepsPerSec;
  if (mag < MIN_STEP_RATE) {
    g_running = false;
    return;
  }
  float p = TIMER_HZ / mag;
  uint16_t period = p >= 65535.0f ? 65535 : (p < MIN_PERIOD ? MIN_PERIOD : (uint16_t)p);
  uint16_t top = period - 1;

  CRITICAL_BEGIN();
  if (positive != g_dirPositive || !g_running) {
    // Starting or reversing: set DIR, then give it a full period before the
    // first step so the driver's DIR setup time is always met.
    g_dirPositive = positive;
    g_stepDir = positive ? 1 : -1;
    writeDir(positive);
    timerSetCount(0);
  }
  if (top != timerTop()) {
    timerSetTop(top);
    // If the new period is already in the past, step on the next count rather
    // than letting the counter wrap all the way round (a stall of up to 0.35 s).
    if (timerCount() >= top) timerSetCount(top - 1);
  }
  g_running = true;
  CRITICAL_END();
}

}  // namespace ring

static ring::Controller controller;

void setup() {
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);  // driver off until the Pi asks for homing
  pinMode(PIN_STEP, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  pinMode(PIN_DIR, OUTPUT);
  writeDir(true);
  pinMode(PIN_HOME, INPUT_PULLUP);

  Serial.begin(ring::SERIAL_BAUD);
  timerBegin();
  controller.begin();
}

void loop() {
  controller.poll();
}
