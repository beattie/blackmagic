/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the blackpill-f4 implementation. */

#include "general.h"
#include "platform.h"
#include "usb.h"
#include "aux_serial.h"
#include "morse.h"
#include "exception.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/usb/dwc/otg_fs.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/adc.h>

jmp_buf fatal_error_jmpbuf;
volatile uint32_t magic[2] __attribute__((section(".noinit")));

#ifdef PLATFORM_HAS_POWER_SWITCH
static void adc_init(void);
#endif

void platform_init(void)
{
	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_CRC);
#if SWO_ENCODING == 1 || SWO_ENCODING == 3
	/* Make sure to power up the timer used for trace */
	rcc_periph_clock_enable(SWO_TIM_CLK);
#endif
#if SWO_ENCODING == 2 || SWO_ENCODING == 3
	/* Enable relevant USART and DMA early in platform init */
	rcc_periph_clock_enable(SWO_UART_CLK);
	rcc_periph_clock_enable(SWO_DMA_CLK);
	/* Deal with receiving on Tx pin by enabling Half-Duplex mode */
#if SWO_UART_PORT == GPIOB && SWO_UART_RX_PIN == GPIO6
	//usart_enable_halfduplex(SWO_UART);
	USART_CR3(SWO_UART) |= USART_CR3_HDSEL;
#endif
#endif

#ifndef BMD_BOOTLOADER
	/* Blackpill board has a floating button on PA0. Pull it up and use as active-low. */
	gpio_mode_setup(USER_BUTTON_KEY_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, USER_BUTTON_KEY_PIN);

	/* Check the USER button */
	if (!gpio_get(USER_BUTTON_KEY_PORT, USER_BUTTON_KEY_PIN) || (magic[0] == BOOTMAGIC0 && magic[1] == BOOTMAGIC1)) {
		magic[0] = 0;
		magic[1] = 0;
		/* Assert blue LED as indicator we are in the bootloader */
		gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_BOOTLOADER);
		gpio_set(LED_PORT, LED_BOOTLOADER);
		/*
		 * Jump to the built in bootloader by mapping System flash.
		 * As we just come out of reset, no other deinit is needed!
		 */
		rcc_periph_clock_enable(RCC_SYSCFG);
		SYSCFG_MEMRM &= ~3U;
		SYSCFG_MEMRM |= 1U;
		scb_reset_core();
	}
#endif

	/* Unmap ST MaskROM and map back Internal Flash */
	rcc_periph_clock_enable(RCC_SYSCFG);
	if ((SYSCFG_MEMRM & 3U) == 1U)
		SYSCFG_MEMRM &= ~3U;

	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[PLATFORM_CLOCK_FREQ]);

	/* Set up DM/DP pins. PA9/PA10 are not routed to USB-C. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	/* Set up TDI, TDO, TCK and TMS pins */
	gpio_mode_setup(TDI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TDI_PIN);
	gpio_mode_setup(TDO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TDO_PIN);
	gpio_mode_setup(TCK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TCK_PIN);
	gpio_mode_setup(TMS_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TMS_PIN);
	gpio_set_output_options(TDI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, TDI_PIN);
	gpio_set_output_options(TDO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, TDO_PIN);
	gpio_set_output_options(TCK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, TCK_PIN);
	gpio_set_output_options(TMS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, TMS_PIN);

	/* Pull up TRST pin */
	gpio_set(TRST_PORT, TRST_PIN);
	gpio_mode_setup(TRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, TRST_PIN);
	gpio_set_output_options(TRST_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, TRST_PIN);
	/* Pull up nRST pin */
	gpio_set(NRST_PORT, NRST_PIN);
	gpio_mode_setup(NRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, NRST_PIN);
	gpio_set_output_options(NRST_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, NRST_PIN);

	/* Set up LED pins */
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_IDLE_RUN | LED_ERROR);
	/* Set up LED_BOOTLOADER if it hasn't been set up yet in the bootloader section above */
#ifdef BMD_BOOTLOADER
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_BOOTLOADER);
#endif
	gpio_mode_setup(LED_PORT_UART, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_UART);

#ifdef PLATFORM_HAS_POWER_SWITCH
	gpio_clear(PWR_BR_PORT, PWR_BR_PIN); // Set the pin of the given GPIO port to 0.
	gpio_mode_setup(PWR_BR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_BR_PIN);
	/* Initialize ADC for VTref sensing */
	adc_init();
#endif

	platform_timing_init();
#if ENABLE_DEBUG == 1
	/* Allow vectoring to DebugMon exception handler upon semihosting breakpoints */
	SCS_DEMCR |= SCS_DEMCR_VC_MON_EN;
#endif
	blackmagic_usb_init();
	aux_serial_init();

	/* https://github.com/libopencm3/libopencm3/pull/1256#issuecomment-779424001 */
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);

	/* By default, do not drive the SWD bus too fast. */
	platform_max_frequency_set(2000000);
}

void platform_nrst_set_val(bool assert)
{
	gpio_set_val(NRST_PORT, NRST_PIN, !assert);
}

bool platform_nrst_get_val(void)
{
	return gpio_get(NRST_PORT, NRST_PIN) == 0;
}

const char *platform_target_voltage(void)
{
#ifdef PLATFORM_HAS_POWER_SWITCH
	static char voltage_string[8];
	const uint32_t voltage = platform_target_voltage_sense();

	if (voltage == 0)
		return "Absent";

	/* Format as X.XV (e.g., "3.3V") */
	const uint32_t volts = voltage / 10U;
	const uint32_t tenths = voltage % 10U;
	snprintf(voltage_string, sizeof(voltage_string), "%u.%uV", (unsigned int)volts, (unsigned int)tenths);
	return voltage_string;
#else
	return "Unknown";
#endif
}

/*
 * Write the bootloader flag and reboot.
 * The platform_init() will see this and reboot a second time into ST BootROM.
 * If BMPBootloader is enabled, then it will see this and initialize its DFU.
 */
void platform_request_boot(void)
{
	magic[0] = BOOTMAGIC0;
	magic[1] = BOOTMAGIC1;
	scb_reset_system();
}

#ifdef PLATFORM_HAS_POWER_SWITCH
bool platform_target_get_power(void)
{
	return gpio_get(PWR_BR_PORT, PWR_BR_PIN);
}

bool platform_target_set_power(const bool power)
{
	gpio_set_val(PWR_BR_PORT, PWR_BR_PIN, power);
	return true;
}

static void adc_init(void)
{
	/* Enable ADC1 clock */
	rcc_periph_clock_enable(RCC_ADC1);

	/* Configure PB0 as analog input */
	gpio_mode_setup(VTREF_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VTREF_PIN);

	/* Configure ADC1 */
	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_480CYC);
	adc_power_on(ADC1);

	/* Wait for ADC to stabilize (minimum 3us for STM32F4) */
	for (volatile size_t i = 0; i < 1000U; ++i)
		continue;
}

/*
 * VTref sensing implementation using ADC1 Channel 8 (PB0).
 * Returns voltage in tenths of a volt (so 33 means 3.3V).
 * This function is needed for implementations that allow the target
 * to be powered from the debug probe.
 */
uint32_t platform_target_voltage_sense(void)
{
	const uint8_t channel = 8; /* ADC12_IN8 = PB0 */
	adc_set_regular_sequence(ADC1, 1, &channel);

	adc_start_conversion_regular(ADC1);

	/* Wait for end of conversion */
	while (!adc_eoc(ADC1))
		continue;

	const uint32_t val = adc_read_regular(ADC1); /* 0-4095 */

	/*
	 * Assuming direct connection or 1:1 voltage divider for 3.3V max.
	 * For simple presence/absence: if ADC reading > threshold, VTref is present.
	 * Conversion: (val * 33) / 4095 gives voltage in 0.1V units
	 */
	return (val * 33U) / 4095U;
}
#endif

void platform_ospeed_update(const uint32_t frequency)
{
	const uint8_t ospeed = frequency > 2000000U ? GPIO_OSPEED_25MHZ : GPIO_OSPEED_2MHZ;

	gpio_set_output_options(TCK_PORT, GPIO_OTYPE_PP, ospeed, TCK_PIN);
	gpio_set_output_options(TMS_PORT, GPIO_OTYPE_PP, ospeed, TMS_PIN);
	gpio_set_output_options(TDI_PORT, GPIO_OTYPE_PP, ospeed, TDI_PIN);
}

void platform_target_clk_output_enable(bool enable)
{
	if (enable) {
		gpio_mode_setup(TCK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TCK_PIN);
		SWDIO_MODE_DRIVE();
	} else {
		gpio_mode_setup(TCK_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TCK_PIN);
		SWDIO_MODE_FLOAT();
	}
}

bool platform_spi_init(const spi_bus_e bus)
{
	uint32_t controller = 0;
	if (bus == SPI_BUS_INTERNAL) {
		/* Set up onboard flash SPI GPIOs: PA5/6/7 as SPI1 in AF5, PA4 as nCS output push-pull */
		gpio_mode_setup(OB_SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, OB_SPI_SCLK | OB_SPI_MISO | OB_SPI_MOSI);
		gpio_mode_setup(OB_SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OB_SPI_CS);
		gpio_set_output_options(
			OB_SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, OB_SPI_SCLK | OB_SPI_MISO | OB_SPI_MOSI | OB_SPI_CS);
		gpio_set_af(OB_SPI_PORT, GPIO_AF5, OB_SPI_SCLK | OB_SPI_MISO | OB_SPI_MOSI);
		/* Deselect the targeted peripheral chip */
		gpio_set(OB_SPI_PORT, OB_SPI_CS);

		rcc_periph_clock_enable(RCC_SPI1);
		rcc_periph_reset_pulse(RST_SPI1);
		controller = OB_SPI;
	} else if (bus == SPI_BUS_EXTERNAL) {
		/* Set up external SPI GPIOs: PB13/14/15 as SPI2 in AF5, PB12 as nCS output push-pull */
		gpio_mode_setup(EXT_SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, EXT_SPI_SCLK | EXT_SPI_MISO | EXT_SPI_MOSI);
		gpio_mode_setup(EXT_SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EXT_SPI_CS);
		gpio_set_output_options(
			EXT_SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, EXT_SPI_SCLK | EXT_SPI_MISO | EXT_SPI_MOSI | EXT_SPI_CS);
		gpio_set_af(EXT_SPI_PORT, GPIO_AF5, EXT_SPI_SCLK | EXT_SPI_MISO | EXT_SPI_MOSI);
		/* Deselect the targeted peripheral chip */
		gpio_set(EXT_SPI_PORT, EXT_SPI_CS);

		rcc_periph_clock_enable(RCC_SPI2);
		rcc_periph_reset_pulse(RST_SPI2);
		controller = EXT_SPI;
	} else
		return false;

	/* Set up hardware SPI: master, PCLK/8, Mode 0, 8-bit MSB first */
	spi_init_master(controller, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
		SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
	spi_enable(controller);
	return true;
}

bool platform_spi_deinit(const spi_bus_e bus)
{
	if (bus == SPI_BUS_INTERNAL) {
		spi_disable(OB_SPI);
		/* Gate SPI1 APB clock */
		rcc_periph_clock_disable(RCC_SPI1);
		/* Unmap GPIOs */
		gpio_mode_setup(
			OB_SPI_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, OB_SPI_SCLK | OB_SPI_MISO | OB_SPI_MOSI | OB_SPI_CS);
		return true;
	} else if (bus == SPI_BUS_EXTERNAL) {
		spi_disable(EXT_SPI);
		rcc_periph_clock_disable(RCC_SPI2);
		gpio_mode_setup(
			EXT_SPI_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, EXT_SPI_SCLK | EXT_SPI_MISO | EXT_SPI_MOSI | EXT_SPI_CS);
		return true;
	} else
		return false;
}

bool platform_spi_chip_select(const uint8_t device_select)
{
	const uint8_t device = device_select & 0x7fU;
	const bool select = !(device_select & 0x80U);
	uint32_t port;
	uint16_t pin;
	switch (device) {
	case SPI_DEVICE_INT_FLASH:
		port = OB_SPI_CS_PORT;
		pin = OB_SPI_CS;
		break;
	case SPI_DEVICE_EXT_FLASH:
		port = EXT_SPI_CS_PORT;
		pin = EXT_SPI_CS;
		break;
	default:
		return false;
	}
	gpio_set_val(port, pin, select);
	return true;
}

uint8_t platform_spi_xfer(const spi_bus_e bus, const uint8_t value)
{
	switch (bus) {
	case SPI_BUS_INTERNAL:
		return spi_xfer(OB_SPI, value);
		break;
	case SPI_BUS_EXTERNAL:
		return spi_xfer(EXT_SPI, value);
		break;
	default:
		return 0U;
	}
}

int platform_hwversion(void)
{
	return 0;
}
