# Firmware

Arduino code for the ring stepper. Plain Arduino, no ROS; builds with the Arduino IDE or `arduino-cli`.

```
arduino/
├── ring_stepper/          # the sketch (Arduino needs the .ino named after its folder)
│   ├── ring_stepper.ino   # hardware layer: pins, step-pulse timer, serial
│   ├── ring_core.h        # control logic: homing, ramp, soft limits, watchdog, status; no hardware access
│   ├── config.h           # wiring and tuning
│   └── ring_protocol.h    # GENERATED from ../../comms/protocol.yaml; do not edit
├── tools/                 # bench tools for the board, run on the Mac
│   ├── ring_console.html  # browser console: home, jog, live position and STATUS plots
│   └── ring_protocol.js   # GENERATED from ../../comms/protocol.yaml; do not edit
└── test/                  # host-side tests, run with pytest from the repo root
    └── test_protocol_cross.py   # C++ header vs Python module, byte for byte
```

## Protocol

Edit `comms/protocol.yaml`, then `python3 comms/gen_protocol.py`. That rewrites
`ring_stepper/ring_protocol.h`, the Python mirror in `colcon/ros2_ws/src/apollo_ring` and
`tools/ring_protocol.js`. Commit the schema and all three together.

## Ring console

```bash
bash scripts/dev.sh ring-ui        # serves arduino/tools on http://localhost:8770 and opens Chrome
```

**Connect serial** picks the Uno's USB port (Chrome or Edge only: they have Web Serial). The Uno reboots
when the port opens, so STATUS starts about 2 s later. From there:

- **Home** sends `CMD_HOME`. **Enable** and **Release motor** send `CMD_ENABLE` 1 and 0; releasing loses the homed position.
- **Jog** works once the ring is READY: hold a button or the arrow keys and the page sends `CMD_VEL` at 25 Hz.
  Let go (or switch tabs) and it stops sending, so the ring ramps to a stop. The firmware watchdog stops it too.
- The dial shows the ring from above, with the soft limits, hard stops and the home flag's half. The charts plot
  the last 30 s of angle and velocity. **Pause** freezes them for hovering; **Export CSV** saves every STATUS frame
  since connecting.

Only one program can hold the serial port: close the console before `arduino-cli upload`, and vice versa.

**Simulate** runs a stand-in for the firmware in the page, so the console works without a board. It follows
`ring_core.h` (same homing sequence, ramp, soft limits, watchdog and step-loss check), the ring stops at the
±170° hard stops, and **Slip ring 5°** fakes lost steps. It is a copy of the logic, not the firmware itself, so
if you change `ring_core.h`, update `SimLink` in `ring_console.html` to match.

## Build and flash (Uno)

```bash
arduino-cli core install arduino:avr                         # once
arduino-cli compile -b arduino:avr:uno arduino/ring_stepper
arduino-cli upload  -b arduino:avr:uno -p /dev/cu.usbmodemXXXX arduino/ring_stepper
```

## Wiring: Arduino Uno to TMC2209

Matches `ring_stepper/config.h`; if you change a pin there, change it here too. USB (D0/D1) is the link to the
Pi. The driver runs in standalone mode: the Uno only drives STEP, DIR and EN, and nothing is configured over
UART. D2 and D3 (the Uno's interrupt pins) are left free for an encoder later.

| TMC2209 | Uno | Notes |
| --- | --- | --- |
| STEP | D4 | `PIN_STEP`. Toggled by the Timer1 interrupt through PORTD bit 4 (`STEP_PORT` / `STEP_BIT` must match) |
| DIR | D5 | `PIN_DIR`. If positive velocity turns the ring clockwise seen from above, set `DIR_INVERT` to 1 |
| EN | D6 | `PIN_EN`, active low. 10 kΩ pull-up to 5 V keeps the driver off while the Uno boots |
| MS1, MS2 | GND | 1/8 microstepping in standalone mode (`MICROSTEPS 8`) |
| SPREAD | GND | StealthChop (quiet) |
| PDN_UART | GND | Standstill current reduction on. Not connected to the Uno |
| CLK | GND | Internal oscillator |
| VREF | Board potentiometer | Sets the motor current. Start at 1.0–1.2 A RMS; the VREF-to-current conversion depends on the breakout's sense resistor, so use its datasheet |
| DIAG, INDEX | Not connected | |
| VIO | 5 V | |
| GND (logic) | GND | |
| VM / GND | 18 V bus | 100 µF / 35 V+ electrolytic right at the pins |
| 1A 1B / 2A 2B | Motor coil A / coil B | Never connect or disconnect with VM live |

| Home sensor | Uno | Notes |
| --- | --- | --- |
| Signal | D7 | `PIN_HOME`, internal pull-up. Must read LOW over the whole negative half of travel and HIGH over the positive half (`HOME_ACTIVE_LOW 1`; set it to 0 if your flag covers the positive half). Zero is the LOW to HIGH edge |
| GND | GND | A switch closes to GND; an open-collector Hall sensor powered from 5 V also works |

The same pin numbers work on the MKR WiFi 1010, but its pins are 3.3 V only: power VIO from the MKR's VCC
(3.3 V), never 5 V.
