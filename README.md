# esp32s3_dual_foc_gp

USB game controller for ESP32-S3 where two axes are controlled by two
**2804 BLDC motors** (3 coil inputs, 7 pole pairs) coupled with AS5600
magnetic encoders, providing haptic detent feedback through the motor
torque.  The motors are driven by Mini L298N dual-H-bridge boards.  Each
motor axis has a separately configurable number of haptic steps per
full revolution.

## Hardware

| Component | Qty | Notes |
|-----------|-----|-------|
| ESP32-S3 DevKit | 1 | Any board with native USB-OTG |
| Mini L298N board | 2 | One per BLDC motor (no ENA/ENB) |
| 2804 BLDC motor | 2 | 3 coil inputs (U/V/W), 7 pole pairs (14P/12N) |
| AS5600 magnetic encoder | 2 | One per motor, on separate I2C buses |
| Diametric magnet | 2 | Attached to each motor shaft |
| Tactile button | 10 | Active-low, one per gamepad button |

### Default GPIO wiring

All pin assignments are defined in **`main/pin_config.h`** and can be
changed there without modifying any other file.

| Signal | GPIO | Description |
|--------|------|-------------|
| MOTOR1_IN1  | 1  | Motor 1 coil U – Mini L298N IN1 (PWM) |
| MOTOR1_IN2  | 2  | Motor 1 coil V – Mini L298N IN2 (PWM) |
| MOTOR1_IN3  | 3  | Motor 1 coil W – Mini L298N IN3 (PWM) |
| MOTOR2_IN1  | 5  | Motor 2 coil U – Mini L298N IN1 (PWM) |
| MOTOR2_IN2  | 6  | Motor 2 coil V – Mini L298N IN2 (PWM) |
| MOTOR2_IN3  | 7  | Motor 2 coil W – Mini L298N IN3 (PWM) |
| BUTTON1     | 4  | Game controller button 1 (active-low)  |
| BUTTON2     | 8  | Game controller button 2 (active-low)  |
| BUTTON3     | 13 | Game controller button 3 (active-low)  |
| BUTTON4     | 14 | Game controller button 4 (active-low)  |
| BUTTON5     | 15 | Game controller button 5 (active-low)  |
| BUTTON6     | 16 | Game controller button 6 (active-low)  |
| BUTTON7     | 17 | Game controller button 7 (active-low)  |
| BUTTON8     | 18 | Game controller button 8 (active-low)  |
| BUTTON9     | 21 | Game controller button 9 (active-low)  |
| BUTTON10    | 38 | Game controller button 10 (active-low) |
| ENCODER1_SDA | 9 | AS5600 #1 – I2C SDA |
| ENCODER1_SCL | 10 | AS5600 #1 – I2C SCL |
| ENCODER2_SDA | 11 | AS5600 #2 – I2C SDA |
| ENCODER2_SCL | 12 | AS5600 #2 – I2C SCL |

USB D−/D+ use the ESP32-S3 native USB-OTG pins (GPIO 19/20) and
require no additional configuration.

### Motor wiring (2804 BLDC — 3 coil inputs)

Each 2804 motor has three coil wires (U, V, W).  A single Mini L298N
drives the three motor coils via IN1–IN3:

```
  IN1 (PWM) ──► OUT1 ──► Coil U
  IN2 (PWM) ──► OUT2 ──► Coil V
  IN3 (PWM) ──► OUT3 ──► Coil W
```

The firmware drives all three coils with sinusoidal PWM (120° apart)
to create a rotating magnetic field.  This true three-phase drive
produces smoother torque than a two-phase approximation.

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

In **`main/main.c`**, change these variables (initialised at startup):

```c
static uint16_t s_motor1_steps    = HAPTIC_DEFAULT_STEPS;  /* 12 */
static uint16_t s_motor2_steps    = HAPTIC_DEFAULT_STEPS;  /* 12 */
```

### Haptic feedback strength

Peak normalised torque for the detent effect (0 – 1).  Adjust per-axis
in **`main/main.c`**:

```c
static float s_motor1_strength = HAPTIC_DEFAULT_STRENGTH;  /* 0.25 */
static float s_motor2_strength = HAPTIC_DEFAULT_STRENGTH;  /* 0.25 */
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
    ├── l298n.h / .c        Mini L298N PWM motor driver (3-phase)
    ├── foc.h / .c          Three-phase sinusoidal FOC
    ├── haptic.h / .c       Haptic detent engine
    ├── usb_gamepad.h / .c  USB HID gamepad (TinyUSB)
    └── main.c              Application entry point
```

## How it works

1. **Encoder reading** — Each AS5600 provides a 12-bit absolute angle
   over I2C.  Two separate I2C buses are used because the AS5600 has a
   fixed address (0x36).

2. **FOC torque control** — A three-phase FOC algorithm converts a
   desired torque command into sinusoidal phase voltages (120° apart)
   for the 2804 motor's coils U, V, and W.  The Mini L298N drives
   all three phases via PWM on IN1–IN3.

3. **Haptic detents** — The haptic engine divides one full rotation
   into N equal steps and applies a spring-like restoring torque toward
   the nearest detent centre.

4. **USB HID** — The current detent index of each axis is mapped to an
   8-bit value and sent to the host as a standard USB gamepad report.

## License

This project is provided as-is for educational and prototyping
purposes.
