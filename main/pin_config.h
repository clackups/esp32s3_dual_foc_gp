/*
 * pin_config.h -- GPIO assignments for the dual-FOC gamepad.
 *
 * Every pin used by the firmware is configured through the top-level
 * "Dual-FOC GP" menuconfig menu (see main/Kconfig.projbuild).  This
 * header simply re-exports the Kconfig symbols under shorter names
 * for use elsewhere in the codebase.
 */

#pragma once

#include "driver/i2c_master.h"
#include "sdkconfig.h"

/* -- Game controller buttons (active-low, internal pull-up) ---------- */
#define BUTTON_COUNT        10
#define BUTTON0_GPIO        CONFIG_DFGP_BUTTON0_GPIO
#define BUTTON1_GPIO        CONFIG_DFGP_BUTTON1_GPIO
#define BUTTON2_GPIO        CONFIG_DFGP_BUTTON2_GPIO
#define BUTTON3_GPIO        CONFIG_DFGP_BUTTON3_GPIO
#define BUTTON4_GPIO        CONFIG_DFGP_BUTTON4_GPIO
#define BUTTON5_GPIO        CONFIG_DFGP_BUTTON5_GPIO
#define BUTTON6_GPIO        CONFIG_DFGP_BUTTON6_GPIO
#define BUTTON7_GPIO        CONFIG_DFGP_BUTTON7_GPIO
#define BUTTON8_GPIO        CONFIG_DFGP_BUTTON8_GPIO
#define BUTTON9_GPIO        CONFIG_DFGP_BUTTON9_GPIO

/* -- Roller485 I2C buses -------------------------------------------- */
#define ROLLER1_SDA_GPIO    CONFIG_DFGP_ROLLER1_SDA_GPIO
#define ROLLER1_SCL_GPIO    CONFIG_DFGP_ROLLER1_SCL_GPIO
#define ROLLER1_I2C_PORT    I2C_NUM_0

#define ROLLER2_SDA_GPIO    CONFIG_DFGP_ROLLER2_SDA_GPIO
#define ROLLER2_SCL_GPIO    CONFIG_DFGP_ROLLER2_SCL_GPIO
#define ROLLER2_I2C_PORT    I2C_NUM_1

#define ROLLER_I2C_FREQ_HZ  CONFIG_DFGP_ROLLER_I2C_FREQ_HZ

/* -- Mode toggle (haptic <-> continuous, active-low, internal pull-up) - */
#define MODE_TOGGLE_GPIO    CONFIG_DFGP_MODE_TOGGLE_GPIO
