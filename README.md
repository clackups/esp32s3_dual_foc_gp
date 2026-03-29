# esp32s3_dual_foc_gp

USB game controller for ESP32-S3 where two axes are controlled by two
**2804 BLDC motors** (3 coil inputs, 7 pole pairs) coupled with AS5600
magnetic encoders, providing haptic detent feedback through the motor
torque.  The motors are driven by L298N dual-H-bridge ICs.  Each
motor axis has a separately configurable number of haptic steps per
full revolution.

## Hardware

| Component | Qty | Notes |
|-----------|-----|-------|
| ESP32-S3 DevKit | 1 | Any board with native USB-OTG |
| L298N module | 2 | One per BLDC motor |
| 2804 BLDC motor | 2 | 3 coil inputs (U/V/W), 7 pole pairs (14P/12N) |
| AS5600 magnetic encoder | 2 | One per motor, on separate I2C buses |
| Diametric magnet | 2 | Attached to each motor shaft |

### Default GPIO wiring

All pin assignments are defined in **`main/pin_config.h`** and can be
changed there without modifying any other file.

| Signal | GPIO | Description |
|--------|------|-------------|
| MOTOR1_ENA  | 1  | Motor 1 coil U – L298N ENA (PWM) |
| MOTOR1_IN1  | 2  | Motor 1 coil U – L298N IN1       |
| MOTOR1_IN2  | 3  | Motor 1 coil U – L298N IN2       |
| MOTOR1_ENB  | 4  | Motor 1 coil V – L298N ENB (PWM) |
| MOTOR1_IN3  | 5  | Motor 1 coil V – L298N IN3       |
| MOTOR1_IN4  | 6  | Motor 1 coil V – L298N IN4       |
| MOTOR2_ENA  | 7  | Motor 2 coil U – L298N ENA (PWM) |
| MOTOR2_IN1  | 8  | Motor 2 coil U – L298N IN1       |
| MOTOR2_IN2  | 15 | Motor 2 coil U – L298N IN2       |
| MOTOR2_ENB  | 16 | Motor 2 coil V – L298N ENB (PWM) |
| MOTOR2_IN3  | 17 | Motor 2 coil V – L298N IN3       |
| MOTOR2_IN4  | 18 | Motor 2 coil V – L298N IN4       |
| ENCODER1_SDA | 9 | AS5600 #1 – I2C SDA |
| ENCODER1_SCL | 10 | AS5600 #1 – I2C SCL |
| ENCODER2_SDA | 11 | AS5600 #2 – I2C SDA |
| ENCODER2_SCL | 12 | AS5600 #2 – I2C SCL |

USB D−/D+ use the ESP32-S3 native USB-OTG pins (GPIO 19/20) and
require no additional configuration.

### Motor wiring (2804 BLDC — 3 coil inputs)

Each 2804 motor has three coil wires (U, V, W).  A single L298N
provides two H-bridge channels, which drive two of the three coils:

```
  ENA (PWM) ─────────── speed
  IN1 / IN2 ─────────── direction  ──► H-bridge A ──► Coil U
  ENB (PWM) ─────────── speed
  IN3 / IN4 ─────────── direction  ──► H-bridge B ──► Coil V
  Coil W ──── (floating)
```

The firmware drives coils U and V with sinusoidal PWM (90° apart) to
create a rotating magnetic field.  Coil W is left unconnected.  This
two-phase drive is sufficient for haptic-detent torque control.

## Software prerequisites

* **ESP-IDF v5.x** — [installation guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/)
* The project pulls `espressif/esp_tinyusb` automatically from the
  IDF Component Registry on first build.

## Build & flash

```bash
# Activate the ESP-IDF environment (path depends on your installation)
. $HOME/esp/esp-idf/export.sh

# Set the target (only needed once)
idf.py set-target esp32s3

# Build
idf.py build

# Flash and monitor (replace /dev/ttyUSB0 with your serial port)
idf.py -p /dev/ttyUSB0 flash monitor
```

## Configuration

### Haptic steps per revolution

In **`main/main.c`**, change these defines:

```c
#define MOTOR1_STEPS  12   /* axis 1: 12 detent positions */
#define MOTOR2_STEPS  24   /* axis 2: 24 detent positions */
```

### Motor pole pairs

The 2804 BLDC motor has 7 pole pairs (14 poles, 12 slots).  The
default is set in `FOC_DEFAULT_POLE_PAIRS` in `main/foc.h`.  Override
it per-motor in `main/main.c` if you use a different motor.

### GPIO assignments

Edit **`main/pin_config.h`** — every hardware pin is a `#define` in
that one file.

## Project structure

```
├── CMakeLists.txt          Top-level ESP-IDF project file
├── sdkconfig.defaults      Default Kconfig settings (target, USB)
├── README.md               This file
└── main/
    ├── CMakeLists.txt      Component registration
    ├── idf_component.yml   IDF Component Registry dependencies
    ├── pin_config.h        *** All GPIO assignments ***
    ├── as5600.h / .c       AS5600 I2C encoder driver
    ├── l298n.h / .c        L298N LEDC-PWM motor driver
    ├── foc.h / .c          Simplified two-phase FOC
    ├── haptic.h / .c       Haptic detent engine
    ├── usb_gamepad.h / .c  USB HID gamepad (TinyUSB)
    └── main.c              Application entry point
```

## How it works

1. **Encoder reading** — Each AS5600 provides a 12-bit absolute angle
   over I2C.  Two separate I2C buses are used because the AS5600 has a
   fixed address (0x36).

2. **FOC torque control** — A simplified two-phase FOC algorithm
   converts a desired torque command into sinusoidal phase voltages for
   the 2804 motor's coils U and V.  The L298N dual-H-bridge drives
   these two phases; coil W is left floating.

3. **Haptic detents** — The haptic engine divides one full rotation
   into N equal steps and applies a spring-like restoring torque toward
   the nearest detent centre.

4. **USB HID** — The current detent index of each axis is mapped to an
   8-bit value and sent to the host as a standard USB gamepad report.

## License

This project is provided as-is for educational and prototyping
purposes.
