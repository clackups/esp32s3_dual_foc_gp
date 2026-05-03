# esp32s3_dual_foc_gp

USB game controller for ESP32-S3 where two axes are controlled by two
**M5Stack Unit-Roller485** smart motor units, providing haptic detent
feedback through the motor torque.  Each Roller485 contains its own
BLDC motor, magnetic encoder and FOC controller; the ESP32-S3 talks
to each unit over a dedicated I2C bus and only sends position
commands.  Each motor axis has a separately configurable number of
haptic steps per full revolution.

## Hardware

| Component | Qty | Notes |
|-----------|-----|-------|
| ESP32-S3 DevKit            | 1 | Any board with native USB-OTG |
| M5Stack Unit-Roller485     | 2 | One per axis, each on its own I2C bus |
| Tactile button             | 10 | Active-low, one per gamepad button |

The Roller485 documentation (I2C protocol):
<https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/776/Unit-Roller485-I2C-Protocol-EN.pdf>

The Arduino driver examples we modeled the I2C usage on:
<https://github.com/m5stack/M5Unit-Roller>

Each Roller485 has a fixed default I2C address (0x64), so the two units
must live on **separate** I2C buses.

### Default GPIO wiring

User-configurable pins (the two I2C buses, the haptic step count and
the Roller485 maximum-current limit) live in the **`Dual-FOC GP`**
top-level menuconfig menu (`idf.py menuconfig`).  The fixed pins
(buttons, mode toggle, status LED) are defined in
**`main/pin_config.h`**.

| Signal        | Default GPIO | Description |
|---------------|--------------|-------------|
| ROLLER1_SDA   | 9 (Kconfig)  | Roller485 #1 - I2C SDA |
| ROLLER1_SCL   | 10 (Kconfig) | Roller485 #1 - I2C SCL |
| ROLLER2_SDA   | 11 (Kconfig) | Roller485 #2 - I2C SDA |
| ROLLER2_SCL   | 12 (Kconfig) | Roller485 #2 - I2C SCL |
| BUTTON0       | 4            | Game controller button 0 (active-low) |
| BUTTON1       | 5            | Game controller button 1 (active-low) |
| BUTTON2       | 6            | Game controller button 2 (active-low) |
| BUTTON3       | 7            | Game controller button 3 (active-low) |
| BUTTON4       | 8            | Game controller button 4 (active-low) |
| BUTTON5       | 18           | Game controller button 5 (active-low) |
| BUTTON6       | 21           | Game controller button 6 (active-low) |
| BUTTON7       | 36           | Game controller button 7 (active-low) |
| BUTTON8       | 37           | Game controller button 8 (active-low) |
| BUTTON9       | 38           | Game controller button 9 (active-low) |
| MODE_TOGGLE   | 35           | Haptic / continuous toggle (active-low) |
| STATUS_LED    | 48           | WS2812 status LED |

USB D-/D+ use the ESP32-S3 native USB-OTG pins (GPIO 19/20) and
require no additional configuration.

## Software prerequisites

* **ESP-IDF v5.x** -- [installation guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/)
* The project pulls `espressif/esp_tinyusb` and `espressif/led_strip`
  automatically from the IDF Component Registry on first build.

## Build & flash

```bash
# Activate the ESP-IDF environment (path depends on your installation)
. $HOME/esp/esp-idf/export.sh

# Set the target (only needed once)
idf.py set-target esp32s3

# (Optional) review or change the "Dual-FOC GP" options
idf.py menuconfig

# Build
idf.py build

# Flash and monitor (replace /dev/ttyUSB0 with your serial port)
idf.py -p /dev/ttyUSB0 flash monitor
```

## Configuration

All user-tunable parameters live in the top-level **`Dual-FOC GP`**
menu of `idf.py menuconfig`:

| Kconfig symbol               | Default | Purpose |
|------------------------------|---------|---------|
| `DFGP_HAPTIC_DEFAULT_STEPS`  | 21      | Number of haptic detent steps per full revolution |
| `DFGP_HAPTIC_MAX_CURRENT`    | 20000   | Maximum current sent to the Roller485 (register 0x20). The unit matches the Roller485 firmware convention (0.01 mA, so 20000 = 200 mA). Higher values give a stronger detent snap but draw more current |
| `DFGP_ROLLER1_SDA_GPIO`      | 9       | SDA GPIO for the Roller485 #1 I2C bus |
| `DFGP_ROLLER1_SCL_GPIO`      | 10      | SCL GPIO for the Roller485 #1 I2C bus |
| `DFGP_ROLLER2_SDA_GPIO`      | 11      | SDA GPIO for the Roller485 #2 I2C bus |
| `DFGP_ROLLER2_SCL_GPIO`      | 12      | SCL GPIO for the Roller485 #2 I2C bus |
| `DFGP_ROLLER_I2C_FREQ_HZ`    | 400000  | I2C clock for both Roller485 buses |

Per-axis runtime overrides (steps and max-current) are in
**`main/main.c`** at the top of the file:

```c
static uint16_t s_motor1_steps       = HAPTIC_DEFAULT_STEPS;
static uint16_t s_motor2_steps       = HAPTIC_DEFAULT_STEPS;
static int32_t  s_motor1_max_current = HAPTIC_DEFAULT_MAX_CURRENT;
static int32_t  s_motor2_max_current = HAPTIC_DEFAULT_MAX_CURRENT;
```

## Project structure

```
|-- CMakeLists.txt          Top-level ESP-IDF project file
|-- sdkconfig.defaults      Default Kconfig settings (target, USB)
|-- README.md               This file
|-- AGENTS.MD               Coding-style instructions for AI agents
`-- main/
    |-- CMakeLists.txt      Component registration
    |-- Kconfig.projbuild   "Dual-FOC GP" menuconfig options
    |-- idf_component.yml   IDF Component Registry dependencies
    |-- pin_config.h        Fixed GPIO assignments (buttons / LED / toggle)
    |-- roller485.h / .c    M5Stack Unit-Roller485 I2C driver
    |-- haptic.h / .c       Haptic detent engine (Roller485 position mode)
    |-- usb_gamepad.h / .c  USB HID gamepad (TinyUSB)
    `-- main.c              Application entry point
```

## How it works

1. **Roller485 initialisation** -- Each unit is placed in position mode
   over its dedicated I2C bus, the maximum current limit
   (`DFGP_HAPTIC_MAX_CURRENT`, register `0x20`) is configured, the
   current shaft position is captured as detent index 0, and the
   output is enabled.

2. **Haptic detents** -- The haptic engine reads the actual position
   from the Roller485 (register `0x90`), snaps it to the nearest
   detent in encoder counts (one revolution = 36000 counts), and
   writes that target back (register `0x80`) only when it changes.
   The Roller485's internal PID + current limiter pulls the rotor
   to the target with the maximum current cap, producing the
   detent "snap" feel.

3. **Continuous centering mode** -- Holding the mode-toggle button
   switches each axis from detent feedback to a constant-centering
   mode where the target is permanently the middle position.  The
   Roller485 keeps the rotor centred while still allowing the user
   to push it within the current limit; the actual position is
   reported on the HID axis.

4. **USB HID** -- The current detent index of each axis (or the
   continuous-mode position deviation) is mapped to a signed 16-bit
   value and sent to the host as a standard USB gamepad report.

## License

This project is provided as-is for educational and prototyping
purposes.
