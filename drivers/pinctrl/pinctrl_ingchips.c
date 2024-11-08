#include <zephyr/init.h>
#include <zephyr/drivers/pinctrl.h>
#include "peripheral_pinctrl.h"

int configure_pin_function(uint32_t pin, uint32_t pinmux, uintptr_t reg)
{
	if (pinmux < IO_SOURCE_SW_DIO) {
		PINCTRL_SetPadMux(pin,pinmux);
	} else {
		switch (pinmux) {
		    case IO_SOURCE_SPI0_CLK_IN:
		        // 处理 SPI0_CLK_IN 的情况
		        break;
		    case IO_SOURCE_SPI0_CSN_IN:
		        // 处理 SPI0_CSN_IN 的情况
		        break;
		    case IO_SOURCE_SPI0_HOLD_IN:
		        // 处理 SPI0_HOLD_IN 的情况
		        break;
		    case IO_SOURCE_SPI0_WP_IN:
		        // 处理 SPI0_WP_IN 的情况
		        break;
		    case IO_SOURCE_SPI0_MISO_IN:
		        // 处理 SPI0_MISO_IN 的情况
		        break;
		    case IO_SOURCE_SPI0_MOSI_IN:
		        // 处理 SPI0_MOSI_IN 的情况
		        break;
		    case IO_SOURCE_SPI1_CLK_IN:
		        // 处理 SPI1_CLK_IN 的情况
		        break;
		    case IO_SOURCE_SPI1_CSN_IN:
		        // 处理 SPI1_CSN_IN 的情况
		        break;
		    case IO_SOURCE_SPI1_MISO_IN:
		        // 处理 SPI1_MISO_IN 的情况
		        break;
		    case IO_SOURCE_SPI1_MOSI_IN:
		        // 处理 SPI1_MOSI_IN 的情况
		        break;
		    case IO_SOURCE_SPI1_HOLD_IN:
		        // 处理 SPI1_HOLD_IN 的情况
		        break;
		    case IO_SOURCE_SPI1_WP_IN:
		        // 处理 SPI1_WP_IN 的情况
		        break;
		    case IO_SOURCE_IR_DATA_IN:
		        // 处理 IR_DATA_IN 的情况
		        break;
		    case IO_SOURCE_I2S_BCLK_IN:
		        // 处理 I2S_BCLK_IN 的情况
		        break;
		    case IO_SOURCE_I2S_LRCLK_IN:
		        // 处理 I2S_LRCLK_IN 的情况
		        break;
		    case IO_SOURCE_I2S_DATA_IN:
		        // 处理 I2S_DATA_IN 的情况
		        break;
		    case IO_SOURCE_UART0_RXD:
		        // 处理 UART0_RXD 的情况
		        break;
		    case IO_SOURCE_UART0_CTS:
		        // 处理 UART0_CTS 的情况
		        break;
		    case IO_SOURCE_UART1_RXD:
		        // 处理 UART1_RXD 的情况
		        break;
		    case IO_SOURCE_UART1_CTS:
		        // 处理 UART1_CTS 的情况
		        break;
		    case IO_SOURCE_I2C0_SCL_IN:
		        // 处理 I2C0_SCL_IN 的情况
		        break;
		    case IO_SOURCE_I2C0_SDA_IN:
		        // 处理 I2C0_SDA_IN 的情况
		        break;
		    case IO_SOURCE_I2C1_SCL_IN:
		        // 处理 I2C1_SCL_IN 的情况
		        break;
		    case IO_SOURCE_I2C1_SDA_IN:
		        // 处理 I2C1_SDA_IN 的情况
		        break;
		    case IO_SOURCE_PDM_DMIC_IN:
		        // 处理 PDM_DMIC_IN 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_0:
		        // 处理 KEYSCN_IN_COL_0 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_1:
		        // 处理 KEYSCN_IN_COL_1 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_2:
		        // 处理 KEYSCN_IN_COL_2 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_3:
		        // 处理 KEYSCN_IN_COL_3 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_4:
		        // 处理 KEYSCN_IN_COL_4 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_5:
		        // 处理 KEYSCN_IN_COL_5 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_6:
		        // 处理 KEYSCN_IN_COL_6 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_7:
		        // 处理 KEYSCN_IN_COL_7 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_8:
		        // 处理 KEYSCN_IN_COL_8 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_9:
		        // 处理 KEYSCN_IN_COL_9 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_10:
		        // 处理 KEYSCN_IN_COL_10 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_11:
		        // 处理 KEYSCN_IN_COL_11 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_12:
		        // 处理 KEYSCN_IN_COL_12 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_13:
		        // 处理 KEYSCN_IN_COL_13 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_14:
		        // 处理 KEYSCN_IN_COL_14 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_15:
		        // 处理 KEYSCN_IN_COL_15 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_16:
		        // 处理 KEYSCN_IN_COL_16 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_17:
		        // 处理 KEYSCN_IN_COL_17 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_18:
		        // 处理 KEYSCN_IN_COL_18 的情况
		        break;
		    case IO_SOURCE_KEYSCN_IN_COL_19:
		        // 处理 KEYSCN_IN_COL_19 的情况
		        break;
		    case IO_SOURCE_SPI2AHB_SCLK:
		        // 处理 SPI2AHB_SCLK 的情况
		        break;
		    case IO_SOURCE_SPI2AHB_CS:
		        // 处理 SPI2AHB_CS 的情况
		        break;
		    case IO_SOURCE_SPI2AHB_DI:
		        // 处理 SPI2AHB_DI 的情况
		        break;
		    case IO_SOURCE_QDEC_PHASEA:
		        // 处理 QDEC_PHASEA 的情况
		        break;
		    case IO_SOURCE_QDEC_PHASEB:
		        // 处理 QDEC_PHASEB 的情况
		        break;
		    case IO_SOURCE_QDEC_INDEX:
		        // 处理 QDEC_INDEX 的情况
		        break;
		    case IO_SOURCE_QDEC_EXT_IN_CLK:
		        // 处理 QDEC_EXT_IN_CLK 的情况
		        break;
		    case IO_SOURCE_QDEC_TIMER_EXT_IN1_A:
		        // 处理 QDEC_TIMER_EXT_IN1_A 的情况
		        break;
		    case IO_SOURCE_QDEC_TIMER_EXT_IN2_A:
		        // 处理 QDEC_TIMER_EXT_IN2_A 的情况
		        break;
		    case IO_SOURCE_QDEC_TIMER_EXT_IN2_B:
		        // 处理 QDEC_TIMER_EXT_IN2_B 的情况
		        break;
		    case IO_SOURCE_PCAP0_IN:
		        // 处理 PCAP0_IN 的情况
		        break;
		    case IO_SOURCE_PCAP1_IN:
		        // 处理 PCAP1_IN 的情况
		        break;
		    case IO_SOURCE_PCAP2_IN:
		        // 处理 PCAP2_IN 的情况
		        break;
		    case IO_SOURCE_PCAP3_IN:
		        // 处理 PCAP3_IN 的情况
		        break;
		    case IO_SOURCE_PCAP4_IN:
		        // 处理 PCAP4_IN 的情况
		        break;
		    case IO_SOURCE_PCAP5_IN:
		        // 处理 PCAP5_IN 的情况
		        break;
		    default:
		        break;
		}

	}

	return 0;
}

int configure_pin_pull(uint32_t pin, uint32_t pull, uintptr_t reg)
{
	PINCTRL_Pull(pin,(const pinctrl_pull_mode_t) pull);
	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	int ret = 0;

	for (uint8_t i = 0; i < pin_cnt; i++) {
		const pinctrl_soc_pin_t *pin_now = &pins[i];

		ret = configure_pin_function(pin_now->pin, pin_now->pinmux, reg);
		if (ret) {
			return ret;
		}

		ret = configure_pin_pull(pin_now->pin, pin_now->pull, reg);
		if (ret) {
			return ret;
		}
	}

	return ret;
}
