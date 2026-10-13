# Firmware

Arduino code for the ring stepper. Plain Arduino, no ROS; builds with the Arduino IDE or `arduino-cli`.

```
arduino/
├── ring_stepper/          # the sketch (Arduino needs the .ino named after its folder)
│   └── ring_protocol.h    # GENERATED from ../../comms/protocol.yaml; do not edit
└── test/                  # host-side tests, run with pytest from the repo root
    └── test_protocol_cross.py   # C++ header vs Python module, byte for byte
```

The sketch itself (`ring_stepper.ino`, `config.h`, `ring_core.h`) and the desktop simulator test from
`apollo_ring.zip` go in `ring_stepper/` and `test/`. The zip's hand-written `ring_protocol.h` is replaced
by the generated one.

## Protocol

Edit `comms/protocol.yaml`, then `python3 comms/gen_protocol.py`. That rewrites both
`ring_stepper/ring_protocol.h` and the Python mirror in `colcon/ros2_ws/src/apollo_ring`. Commit all three together.

## Build and flash (Uno)

```bash
arduino-cli core install arduino:avr                         # once
arduino-cli lib install TMCStepper                           # once
arduino-cli compile -b arduino:avr:uno arduino/ring_stepper
arduino-cli upload  -b arduino:avr:uno -p /dev/cu.usbmodemXXXX arduino/ring_stepper
```

## Wiring: Arduino Uno to TMC2209

USB (D0/D1) is the link to the Pi. The driver runs in UART mode on SoftwareSerial.

| TMC2209 | Uno | Notes |
| --- | --- | --- |
| STEP | D9 | Timer1 OC1A; step pulses come from the timer interrupt |
| DIR | D8 | |
| EN | D7 | Active low. 10 kΩ pull-up to 5 V keeps the driver off while the Uno boots |
| PDN_UART | D10 (RX) direct, D11 (TX) through 1 kΩ | Single-wire UART, 57600 baud |
| DIAG | D2 (INT0) | Optional, StallGuard |
| MS1, MS2 | GND | UART address 0 |
| CLK | GND | Internal oscillator |
| VIO | 5 V | |
| GND (logic) | GND | |
| VM / GND | 18 V bus | 100 µF / 35 V+ electrolytic right at the pins |
| 1A 1B / 2A 2B | Motor coil A / coil B | Never connect or disconnect with VM live |
| Home switch | D3 (INT1) | Hall sensor or switch on the ring |

Start at 1.0–1.2 A RMS (R_sense 0.11 Ω), 16 microsteps with interpolation, StealthChop. Configure the
driver over UART at startup only; SoftwareSerial masks interrupts while it sends.
