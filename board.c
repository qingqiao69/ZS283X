/*
 * Copyright (c) 2018 Actions Semiconductor Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief board init functions
 */

#include <init.h>
#include <gpio.h>
#include <soc.h>
#include "board.h"
#include <device.h>
#include <pwm.h>
#include <input_dev.h>
#include "board_version.h"
#include <nvram_config.h>
#include <string.h>

#ifdef CONFIG_ACTIONS_ABT
extern void bteg_set_bqb_mode(void);
extern uint8_t bteg_get_bqb_mode(void);
extern int bteg_set_bqb_flag(int value);
extern int bteg_get_bqb_flag(void);
#endif

static const struct acts_pin_config board_pin_config[] = {
	/* uart0 */
	{2, 7 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* UART0_RX */
	{3, 7 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* UART0_TX */

#ifdef CONFIG_UART_ACTS_PORT_1
	/* uart1 */
	{22, 0xe | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* UART1_RX */
	{7, 7 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* UART1_TX */
#endif

	/* spi0, nor flash */
	{28, 5 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(1)},	/* SPI0_SS */
	{29, 6 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(1)},	/* SPI0_SCLK */
	{30, 6 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(1)},	/* SPI0_MISO */
	{31, 3 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(1)},	/* SPI0_MOSI */

//#ifdef CONFIG_I2C_SLAVE_ACTS
//	{CONFIG_I2C_SLAVE_0_SCL_PIN,  	5 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* I2C0_SCL */
//	{CONFIG_I2C_SLAVE_0_SDA_PIN,  	5 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* I2C0_SDA */
//#endif

	{46, GPIO_CTL_AD_SELECT | GPIO_CTL_GPIO_INEN },	/* AUX0L */
	{47, GPIO_CTL_AD_SELECT | GPIO_CTL_GPIO_INEN },	/* AUX0R */

	/* sd1 */
	{8, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},			/* SD1_CLK */
#ifdef BOARD_SDCARD_USE_INTERNAL_PULLUP
	/* sd1, internal pull-up resistances in SoC */
	{9, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PULLUP | GPIO_CTL_PADDRV_LEVEL(4)},	/* SD1_CMD */
	{10, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PULLUP | GPIO_CTL_PADDRV_LEVEL(4)},	/* SD1_D0 */
#else
	/* sd1, external pull-up resistances on the board */
	{9, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},	/* SD1_CMD */
	{10, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},	/* SD1_D0 */
#endif

	/* i2s0 tx */
#ifdef CONFIG_AUDIO_OUT_I2STX0_SUPPORT
	{32, 0x9 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* mclk */
	{33, 0x9 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* bclk */
	{34, 0x9 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* lrclk */
	{35, 0x9 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* dout */
#endif

	/* i2s0 rx */
#ifdef CONFIG_AUDIO_IN_I2SRX0_SUPPORT
	{32, 0xa | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* mclk */
	{33, 0xa | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* bclk */
	{34, 0xa | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* lrclk */
	{35, 0xa | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* din */
#endif

	/* i2s1 rx */
#ifdef CONFIG_AUDIO_IN_I2SRX1_SUPPORT
	{32, 0xb | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* mclk */
	{33, 0xb | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* bclk */
	{34, 0xb | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* lrclk */
	{35, 0xb | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},  /* din */
#endif

	/* aout */
	{50,  GPIO_CTL_AD_SELECT},  /* aoutl/aoutlp */
	{52,  GPIO_CTL_AD_SELECT},  /* aoutr/aoutrp */


#ifdef CONFIG_XSPI1_NOR_ACTS
	{11, 5 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3)},     /* SPI1_MOSI */
	{10, 5 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3)},     /* SPI1_MISO */
	{ 9, 5 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3)},     /* SPI1_SCLK */
	{ 8, 5 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3)},     /* SPI1_SS */
#endif
};

#ifdef CONFIG_ACTIONS_ABT
static const struct acts_pin_config board_pin_config_bqb[] = {
	/* uart0 */
	{2, 7 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* UART0_RX */
	{3, 7 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* UART0_TX */

	/* spi0, nor flash */
	{28, 5 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(1)},	/* SPI0_SS */
	{29, 6 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(1)},	/* SPI0_SCLK */
	{30, 6 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(1)},	/* SPI0_MISO */
	{31, 3 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(1)},	/* SPI0_MOSI */

	/* sd1 */
	{8, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},			/* SD1_CLK */
#ifdef BOARD_SDCARD_USE_INTERNAL_PULLUP
	/* sd1, internal pull-up resistances in SoC */
	{9, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PULLUP | GPIO_CTL_PADDRV_LEVEL(4)},	/* SD1_CMD */
	{10, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PULLUP | GPIO_CTL_PADDRV_LEVEL(4)},	/* SD1_D0 */
#else
	/* sd1, external pull-up resistances on the board */
	{9, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},	/* SD1_CMD */
	{10, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},	/* SD1_D0 */
#endif


#if (CONFIG_BT_BQB_UART_PORT == 0)
	/* use uart0 for BQB test */
	{2, 7 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* UART0_RX */
	{3, 7 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* UART0_TX */
#elif (CONFIG_BT_BQB_UART_PORT == 1)
	/* use uart1 for BQB test */
	{7, 0x7 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* UART1_TX */
	{22, 0xe | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3) | GPIO_CTL_PULLUP},	/* UART1_RX */
#endif
};
#endif

#ifdef BOARD_SDCARD_POWER_EN_GPIO

#define SD_CARD_POWER_RESET_MS	80

#define SD0_CMD_PIN		9
#define SD0_D0_PIN		10

static int pinmux_sd0_cmd, pinmux_sd0_d0;

static void board_mmc0_pullup_disable(void)
{
	struct device *sd_gpio_dev;

	sd_gpio_dev = device_get_binding(CONFIG_GPIO_ACTS_DEV_NAME);
	if (!sd_gpio_dev)
		return;

	/* backup origin pinmux config */
	acts_pinmux_get(SD0_CMD_PIN, &pinmux_sd0_cmd);
	acts_pinmux_get(SD0_D0_PIN, &pinmux_sd0_d0);

	/* sd_cmd pin output low level to avoid leakage */
	gpio_pin_configure(sd_gpio_dev, SD0_CMD_PIN, GPIO_DIR_OUT);
	gpio_pin_write(sd_gpio_dev, SD0_CMD_PIN, 0);

	/* sd_d0 pin output low level to avoid leakage */
	gpio_pin_configure(sd_gpio_dev, SD0_D0_PIN, GPIO_DIR_OUT);
	gpio_pin_write(sd_gpio_dev, SD0_D0_PIN, 0);
}

static void board_mmc0_pullup_enable(void)
{
	/* restore origin pullup pinmux config */
	acts_pinmux_set(SD0_CMD_PIN, pinmux_sd0_cmd);
	acts_pinmux_set(SD0_D0_PIN, pinmux_sd0_d0);
}

static int board_mmc_power_gpio_reset(struct device *power_gpio_dev, int power_gpio)
{
	gpio_pin_configure(power_gpio_dev, power_gpio,
			   GPIO_DIR_OUT);

	/* 0: power on, 1: power off */
	/* card vcc power off */
	gpio_pin_write(power_gpio_dev, power_gpio, 1);

	/* disable mmc0 pull-up to avoid leakage */
	board_mmc0_pullup_disable();

	k_sleep(SD_CARD_POWER_RESET_MS);

	/* card vcc power on */
	gpio_pin_write(power_gpio_dev, power_gpio, 0);

	k_sleep(10);

	/* restore mmc0 pull-up */
	board_mmc0_pullup_enable();

	return 0;
}
#endif	/* BOARD_SDCARD_POWER_EN_GPIO */

int board_mmc_power_reset(int mmc_id)
{
	struct device *power_gpio_dev;

	if (mmc_id != 0)
		return 0;

#ifdef BOARD_SDCARD_POWER_EN_GPIO
	power_gpio_dev = device_get_binding(BOARD_SDCARD_POWER_EN_GPIO_NAME);
	if (!power_gpio_dev)
		return -EINVAL;

	board_mmc_power_gpio_reset(power_gpio_dev, BOARD_SDCARD_POWER_EN_GPIO);

#if defined(BOARD_SDCARD_DETECT_GPIO) && (BOARD_SDCARD_DETECT_GPIO == BOARD_SDCARD_POWER_EN_GPIO)
	/* switch gpio function to input for detecting card plugin */
	gpio_pin_configure(power_gpio_dev, BOARD_SDCARD_DETECT_GPIO, GPIO_DIR_IN);
#endif

#endif	/* BOARD_SDCARD_POWER_EN_GPIO */

	return 0;
}

static const struct acts_pin_config board_pin_config_mmc[] = {
	/* sd1 */
	{8, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},			/* SD1_CLK */
#ifdef BOARD_SDCARD_USE_INTERNAL_PULLUP
	/* sd1, internal pull-up resistances in SoC */
	{9, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PULLUP | GPIO_CTL_PADDRV_LEVEL(4)},	/* SD1_CMD */
	{10, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PULLUP | GPIO_CTL_PADDRV_LEVEL(4)}, /* SD1_D0 */
#else
	/* sd1, external pull-up resistances on the board */
	{9, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},	/* SD1_CMD */
	{10, 0xd | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(4)},	/* SD1_D0 */
#endif
};

static const struct acts_pin_config board_led_pin_config[] = {
#ifdef BOARD_PWM_MAP
    BOARD_PWM_MAP
#endif
};

int _board_pinmux_8_to_10_is_sd_pins(void)
{
	int i;
    uint32_t mode;

	for (i = 0; i < ARRAY_SIZE(board_pin_config_mmc); i++)
    {
        acts_pinmux_get(board_pin_config_mmc[i].pin_num, &mode);
        if(board_pin_config_mmc[i].mode != mode)
        {
            return 0;
        }
	}
    return 1;
}

int board_pinmux_8_to_10_pins(const char *module_name)
{
	if(strcmp(module_name, "mmc") == 0)
	{
		acts_pinmux_setup_pins(board_pin_config_mmc, ARRAY_SIZE(board_pin_config_mmc));
//		board_mmc_power_reset(0);
	}
	else
		acts_pinmux_setup_pins(board_led_pin_config, ARRAY_SIZE(board_led_pin_config));

	return 0;
}

#define EXTERN_PA_CTL1_PIN  21
#define EXTERN_PA_CTL2_PIN  5

void board_extern_pa_ctl(uint8_t mode)
{
#ifdef CONFIG_ACTIONS_ABT
	if (bteg_get_bqb_mode() != 1) {
#endif
	struct device *pa_gpio_dev;

	pa_gpio_dev = device_get_binding(CONFIG_GPIO_ACTS_DEV_NAME);
	if (!pa_gpio_dev)
		return;

	if(mode == 0) {
		//PA shutoff ,  pa_ctl1=0, pa_ctl2=0
		printk("%d, ext_pa shutoff!\n", __LINE__);
		gpio_pin_configure(pa_gpio_dev, EXTERN_PA_CTL1_PIN, GPIO_DIR_OUT);
		gpio_pin_write(pa_gpio_dev, EXTERN_PA_CTL1_PIN, 0);

		gpio_pin_configure(pa_gpio_dev, EXTERN_PA_CTL2_PIN, GPIO_DIR_OUT);
		gpio_pin_write(pa_gpio_dev, EXTERN_PA_CTL2_PIN, 0);

	} else if(mode == 1) {
		// open
#if (CONFIG_EXTERN_PA_CLASS == 0)
		printk("%d, ext_pa classAB!\n", __LINE__);
		gpio_pin_configure(pa_gpio_dev, EXTERN_PA_CTL1_PIN, GPIO_DIR_OUT);
		gpio_pin_write(pa_gpio_dev, EXTERN_PA_CTL1_PIN, 1);

		gpio_pin_configure(pa_gpio_dev, EXTERN_PA_CTL2_PIN, GPIO_DIR_OUT);
		gpio_pin_write(pa_gpio_dev, EXTERN_PA_CTL2_PIN, 0);
#else
		// classD mode, pa_ctl1=1,	pa_ctl2=1
		gpio_pin_configure(pa_gpio_dev, EXTERN_PA_CTL1_PIN, GPIO_DIR_OUT);
		gpio_pin_write(pa_gpio_dev, EXTERN_PA_CTL1_PIN, 1);

		gpio_pin_configure(pa_gpio_dev, EXTERN_PA_CTL2_PIN, GPIO_DIR_OUT);
		gpio_pin_write(pa_gpio_dev, EXTERN_PA_CTL2_PIN, 1);
#endif
	} else {
		printk("%d, ext_pa mode error!\n", __LINE__);
	}
#ifdef CONFIG_ACTIONS_ABT
	}
#endif
}

static int board_early_init(struct device *arg)
{
	ARG_UNUSED(arg);

	int value = 0;

	/* LRADC1: wio0  */
	value = sys_read32(WIO0_CTL);
	value = (value & (~(0x0000000F))) | (1 << 3);
	sys_write32(value, WIO0_CTL);

	/* bandgap has filter resistor  */
	value = sys_read32(BDG_CTL);
	value = (value | (1 << 6));
	sys_write32(value, BDG_CTL);

#ifdef CONFIG_PWM_ACTS
	//acts_pinmux_setup_pins(board_led_pin_config, ARRAY_SIZE(board_led_pin_config));
#endif

	acts_pinmux_setup_pins(board_pin_config, ARRAY_SIZE(board_pin_config));

	return 0;
}

void board_jtag_init(void)
{
#ifdef CONFIG_CPU0_EJTAG_ENABLE
	soc_debug_enable_jtag(SOC_JTAG_CPU0, CONFIG_CPU0_EJTAG_GROUP);
#else
	soc_debug_disable_jtag(SOC_JTAG_CPU0);
#endif

#ifdef CONFIG_DSP_EJTAG_ENABLE
	soc_debug_enable_jtag(SOC_JTAG_DSP, CONFIG_DSP_EJTAG_GROUP);
#else
	soc_debug_disable_jtag(SOC_JTAG_DSP);
#endif
}


static int key_press_cnt;
static int key_invalid;
static struct input_value key_last_val;

u8_t sys_transceive_mode = -1;
u8_t sys_transm_input_source = -1;

#ifdef CONFIG_BT_TRANSCEIVER
#if defined(CONFIG_SWITCH_KEY_FOR_TRANSCEIVER_MODE_SWITCH)
#define DETECT_SWITCH_KEY
static u8_t key_switch_val[2] = {KEY_BT_TRANSFER, KEY_BT_RECEIVER};
static u8_t last_key_switch_val = KEY_RESERVED;
#endif

#if defined(CONFIG_SWITCH_KEY_FOR_SOURCE_MODE_SWITCH)
#define DETECT_INPUT_SOURCE
static u8_t key_input_src_val[2] = {KEY_BT_SOURCE0, KEY_BT_SOURCE1};
static u8_t last_key_input_src_val = KEY_RESERVED;
#endif
#endif


static void board_key_input_notify(struct device *dev, struct input_value *val)
{
	/* any adc key is pressed? */
	if (val->code == KEY_RESERVED) {
		key_invalid = 1;
		return;
	}

#ifdef CONFIG_BT_TRANSCEIVER
#ifdef DETECT_SWITCH_KEY
    if(val->code == key_switch_val[0])
        last_key_switch_val = key_switch_val[0];
    else if(val->code == key_switch_val[1])
        last_key_switch_val = key_switch_val[1];
#endif

#ifdef DETECT_INPUT_SOURCE
    if(val->code == key_input_src_val[0])
        last_key_input_src_val = key_input_src_val[0];
    else if(val->code == key_input_src_val[1])
        last_key_input_src_val = key_input_src_val[1];
#endif
#endif

	key_press_cnt++;

	/* save last adfu key value */
	if (key_press_cnt == 1) {
		key_last_val = *val;
		return;
	}

	/* key code must be same with last code and status is pressed */
	if (key_last_val.type != val->type ||
		key_last_val.code != val->code ||
		val->code == KEY_RESERVED) {
		key_invalid = 1;
	}
}

static void check_adfu_key(void)
{
	struct device *adckey_dev;

	/* check adfu */
	adckey_dev = device_get_binding(CONFIG_INPUT_DEV_ACTS_ADCKEY_NAME);
	if (!adckey_dev) {
		printk("%s: error \n", __func__);
	}

	input_dev_enable(adckey_dev);
	input_dev_register_notify(adckey_dev, board_key_input_notify);

	/* wait adfu key */
	k_sleep(80);

	if (key_press_cnt > 0 && !key_invalid) {
		/* check again */
		k_sleep(120);

		if (key_press_cnt > 1 && !key_invalid) {
			printk("key %d detected!\n", key_last_val.code);
            if(key_last_val.code == KEY_ADFU) {
    			/* adfu key is pressed, goto adfu! */
    			printk("enter adfu!\n");
    			sys_pm_reboot(REBOOT_TYPE_GOTO_ADFU);
            }
		}
	}

#ifdef CONFIG_BT_TRANSCEIVER
    nvram_config_get("SYS_TRANSCEIVE_MODE", &sys_transceive_mode, 1);
    if( (sys_transceive_mode != 0) && (sys_transceive_mode != 1) )
    {
        sys_transceive_mode = CONFIG_DEFAULT_TRANSCEIVE_MODE;
    }
#ifndef DETECT_SWITCH_KEY
    printk("switch to %s mode!\n", sys_transceive_mode ? "transmitter": "receiver");
#endif


#ifdef DETECT_SWITCH_KEY
    if(last_key_switch_val == KEY_BT_TRANSFER)
    {
        sys_transceive_mode = TRANSFER_MODE;
    }
    else if(last_key_switch_val == KEY_BT_RECEIVER)
    {
        sys_transceive_mode = RECEIVER_MODE;
    }
    printk("switch to %s mode!\n", sys_transceive_mode ? "transmitter": "receiver");
#endif

#ifdef DETECT_INPUT_SOURCE
	k_sleep(80);

    if(last_key_input_src_val == KEY_BT_SOURCE0)
    {
        sys_transm_input_source = INPUT_SOURCE0;
    }
    else if(last_key_input_src_val == KEY_BT_SOURCE1)
    {
        sys_transm_input_source = INPUT_SOURCE1;
    }
    printk("switch to transm %s!\n", sys_transm_input_source ? "source0": "source1");
#endif
#endif

	input_dev_unregister_notify(adckey_dev, board_key_input_notify);
	input_dev_disable(adckey_dev);
}

static int board_later_init(struct device *arg)
{
	ARG_UNUSED(arg);

	check_adfu_key();

#ifdef CONFIG_ACTIONS_FCC
    extern int fcc_test(void);
    fcc_test();
#endif

#ifdef CONFIG_ACTIONS_ABT
	if(bteg_get_bqb_flag() == 1)
	{
		bteg_set_bqb_flag(0);
		bteg_set_bqb_mode();

		acts_pinmux_setup_pins(board_pin_config_bqb, ARRAY_SIZE(board_pin_config_bqb));
	}
#endif

	return 0;
}

uint32_t libboard_version_get(void)
{
    return LIBBOARD_VERSION_NUMBER;
}


SYS_INIT(board_early_init, PRE_KERNEL_1, 5);

SYS_INIT(board_later_init, POST_KERNEL, 5);
