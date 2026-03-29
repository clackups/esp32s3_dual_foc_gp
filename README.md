# esp32s3_dual_foc_gp

USB game controller for ESP32-S3 where two axes are controlled by two
BLDC motors coupled with AS5600 magnetic encoders, providing haptic
detent feedback through the motor torque.  The motors are driven by
DRV8833 dual-H-bridge ICs.  Each motor axis has a separately
configurable number of haptic steps per full revolution.

## Hardware

| Component | Qty | Notes |
|-----------|-----|-------|
| ESP32-S3 DevKit | 1 | Any board with native USB-OTG |
| DRV8833 breakout | 2 | One per BLDC motor |
| BLDC gimbal motor | 2 | Typically 7 pole pairs |
| AS5600 magnetic encoder | 2 | One per motor, on separate I2C buses |
| Diametric magnet | 2 | Attached to each motor shaft |

### Default GPIO wiring

All pin assignments are defined in **`main/pin_config.h`** and can be
changed there without modifying any other file.

| Signal | GPIO | Description |
|--------|------|-------------|
| MOTOR1_AIN1 | 1 | Motor 1 – DRV8833 AIN1 |
| MOTOR1_AIN2 | 2 | Motor 1 – DRV8833 AIN2 |
| MOTOR1_BIN1 | 3 | Motor 1 – DRV8833 BIN1 |
| MOTOR1_BIN2 | 4 | Motor 1 – DRV8833 BIN2 |
| MOTOR2_AIN1 | 5 | Motor 2 – DRV8833 AIN1 |
| MOTOR2_AIN2 | 6 | Motor 2 – DRV8833 AIN2 |
| MOTOR2_BIN1 | 7 | Motor 2 – DRV8833 BIN1 |
| MOTOR2_BIN2 | 8 | Motor 2 – DRV8833 BIN2 |
| ENCODER1_SDA | 9 | AS5600 #1 – I2C SDA |
| ENCODER1_SCL | 10 | AS5600 #1 – I2C SCL |
| ENCODER2_SDA | 11 | AS5600 #2 – I2C SDA |
| ENCODER2_SCL | 12 | AS5600 #2 – I2C SCL |

USB D−/D+ use the ESP32-S3 native USB-OTG pins (GPIO 19/20) and
require no additional configuration.

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

Adjust `MOTOR_POLE_PAIRS` in `main/main.c` (or the default in
`main/foc.h`) to match the BLDC motors you are using.

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
    ├── drv8833.h / .c      DRV8833 LEDC-PWM motor driver
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
   converts a desired torque command into sinusoidal phase voltages.
   The DRV8833 dual-H-bridge drives two motor phases; the third phase
   floats.

3. **Haptic detents** — The haptic engine divides one full rotation
   into N equal steps and applies a spring-like restoring torque toward
   the nearest detent centre.

4. **USB HID** — The current detent index of each axis is mapped to an
   8-bit value and sent to the host as a standard USB gamepad report.

## License

This project is provided as-is for educational and prototyping
purposes.
