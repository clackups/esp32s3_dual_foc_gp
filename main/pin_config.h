/*
 * pin_config.h -- GPIO assignments for the dual-FOC gamepad.
 *
 * Hardware pins that are not user-configurable through Kconfig are
 * defined here.  The two Roller485 I2C buses (SDA / SCL pins, clock,
 * the haptic step count and the Roller485 max-current limit) are
 * configured via the "Dual-FOC GP" menuconfig menu instead -- see
 * main/Kconfig.projbuild.
 */

#pragma once

#include "driver/i2c_master.h"
#include "sdkconfig.h"

/* -- Game controller buttons (active-low, internal pull-up) ---------- */
#define BUTTON_COUNT        10
#define BUTTON0_GPIO        4
#define BUTTON1_GPIO        5
#define BUTTON2_GPIO        6
#define BUTTON3_GPIO        7
#define BUTTON4_GPIO        8
#define BUTTON5_GPIO        18
#define BUTTON6_GPIO        21
#define BUTTON7_GPIO        36
#define BUTTON8_GPIO        37
#define BUTTON9_GPIO        38

/* -- Roller485 I2C buses (pins come from menuconfig) ----------------- */
#define ROLLER1_SDA_GPIO    CONFIG_DFGP_ROLLER1_SDA_GPIO
#define ROLLER1_SCL_GPIO    CONFIG_DFGP_ROLLER1_SCL_GPIO
#define ROLLER1_I2C_PORT    I2C_NUM_0

#define ROLLER2_SDA_GPIO    CONFIG_DFGP_ROLLER2_SDA_GPIO
#define ROLLER2_SCL_GPIO    CONFIG_DFGP_ROLLER2_SCL_GPIO
#define ROLLER2_I2C_PORT    I2C_NUM_1

#define ROLLER_I2C_FREQ_HZ  CONFIG_DFGP_ROLLER_I2C_FREQ_HZ

/* -- Mode toggle (haptic <-> continuous, active-low, internal pull-up) - */
#define MODE_TOGGLE_GPIO    35

/* -- Status LED (WS2812 on GPIO 48) ---------------------------------- */
#define STATUS_LED_GPIO     48
