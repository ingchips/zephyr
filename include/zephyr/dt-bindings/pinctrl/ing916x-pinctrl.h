/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_ING916_PINCTRLF1_H_
#define ZEPHYR_ING916_PINCTRLF1_H_

// Output sources
#define IO_SOURCE_GPIO                      0
#define IO_SOURCE_DEBUG_BUS                 1
#define IO_SOURCE_SW_DATA_OUT               2
#define IO_SOURCE_SPI0_CLK_OUT              3
#define IO_SOURCE_SPI0_CSN_OUT              4
#define IO_SOURCE_SPI0_HOLD_OUT             5
#define IO_SOURCE_SPI0_WP_OUT               6
#define IO_SOURCE_SPI0_MISO_OUT             7
#define IO_SOURCE_SPI0_MOSI_OUT             8
#define IO_SOURCE_SPI1_CLK_OUT              9
#define IO_SOURCE_SPI1_CSN_OUT              10
#define IO_SOURCE_SPI1_MISO_OUT             11
#define IO_SOURCE_SPI1_MOSI_OUT             12
#define IO_SOURCE_SPI1_HOLD_OUT             13
#define IO_SOURCE_SPI1_WP_OUT               14
#define IO_SOURCE_IR_WAKEUP                 15
#define IO_SOURCE_IR_DATA_OUT               16
#define IO_SOURCE_I2S_BCLK_OUT              17
#define IO_SOURCE_I2S_LRCLK_OUT             18
#define IO_SOURCE_I2S_DATA_OUT              19
#define IO_SOURCE_UART0_TXD                 20
#define IO_SOURCE_UART0_RTS                 21
#define IO_SOURCE_UART1_TXD                 22
#define IO_SOURCE_UART1_RTS                 23
#define IO_SOURCE_I2C0_SCL_OUT              24
#define IO_SOURCE_I2C0_SDA_OUT              25
#define IO_SOURCE_I2C1_SCL_OUT              26
#define IO_SOURCE_I2C1_SDA_OUT              27
// PWM outputs from TIMERs
#define IO_SOURCE_TIMER0_PWM0_A             28
#define IO_SOURCE_TIMER0_PWM0_B             29
#define IO_SOURCE_TIMER0_PWM1_A             30
#define IO_SOURCE_TIMER0_PWM1_B             31
#define IO_SOURCE_TIMER1_PWM0_A             32
#define IO_SOURCE_TIMER1_PWM0_B             33
#define IO_SOURCE_TIMER1_PWM1_A             34
#define IO_SOURCE_TIMER1_PWM1_B             35
#define IO_SOURCE_TIMER2_PWM0_A             36
#define IO_SOURCE_TIMER2_PWM0_B             37
#define IO_SOURCE_TIMER2_PWM1_A             38
#define IO_SOURCE_TIMER2_PWM1_B             39
// PWM outputs from PWM
#define IO_SOURCE_PWM0_A                    40
#define IO_SOURCE_PWM0_B                    41
#define IO_SOURCE_PWM1_A                    42
#define IO_SOURCE_PWM1_B                    43
#define IO_SOURCE_PWM2_A                    44
#define IO_SOURCE_PWM2_B                    45
// Antenna switches
#define IO_SOURCE_ANT_SW0                   46
#define IO_SOURCE_ANT_SW1                   47
#define IO_SOURCE_ANT_SW2                   48
#define IO_SOURCE_ANT_SW3                   49
#define IO_SOURCE_ANT_SW4                   50
#define IO_SOURCE_ANT_SW5                   51
#define IO_SOURCE_ANT_SW6                   52
#define IO_SOURCE_ANT_SW7                   53
// Power amplifiers
#define IO_SOURCE_PA_TXEN                   54
#define IO_SOURCE_PA_RXEN                   55
// PDM DMIC MCLK
#define IO_SOURCE_PDM_DMIC_MCLK             56
// Key scan rows
#define IO_SOURCE_KEYSCN_ROW_0              57
#define IO_SOURCE_KEYSCN_ROW_1              58
#define IO_SOURCE_KEYSCN_ROW_2              59
#define IO_SOURCE_KEYSCN_ROW_3              60
#define IO_SOURCE_KEYSCN_ROW_4              61
#define IO_SOURCE_KEYSCN_ROW_5              62
#define IO_SOURCE_KEYSCN_ROW_6              63
#define IO_SOURCE_KEYSCN_ROW_7              64
#define IO_SOURCE_KEYSCN_ROW_8              65
#define IO_SOURCE_KEYSCN_ROW_9              66
#define IO_SOURCE_KEYSCN_ROW_10             67
#define IO_SOURCE_KEYSCN_ROW_11             68
#define IO_SOURCE_KEYSCN_ROW_12             69
#define IO_SOURCE_KEYSCN_ROW_13             70
#define IO_SOURCE_KEYSCN_ROW_14             71
#define IO_SOURCE_KEYSCN_ROW_15             72
#define IO_SOURCE_KEYSCN_ROW_16             73
#define IO_SOURCE_KEYSCN_ROW_17             74
#define IO_SOURCE_KEYSCN_ROW_18             75
#define IO_SOURCE_KEYSCN_ROW_19             76
#define IO_SOURCE_KEYSCN_ROW_20             77
#define IO_SOURCE_KEYSCN_ROW_21             78
#define IO_SOURCE_KEYSCN_ROW_22             79
#define IO_SOURCE_KEYSCN_ROW_23             80
// SPI2 AHB data output
#define IO_SOURCE_SPI2AHB_DATA_OUT          77
// QDEC outputs
#define IO_SOURCE_QDEC_TIMER_EXT_OUT0_A     78
#define IO_SOURCE_QDEC_TIMER_EXT_OUT1_A     79
#define IO_SOURCE_QDEC_TIMER_EXT_OUT2_A     80
#define IO_SOURCE_QDEC_TIMER_EXT_OUT0_B     81
#define IO_SOURCE_QDEC_TIMER_EXT_OUT1_B     82
#define IO_SOURCE_QDEC_TIMER_EXT_OUT2_B     83

// Input sources
#define IO_SOURCE_SW_DIO                    84
#define IO_SOURCE_SW_CLK                    85
#define IO_SOURCE_SPI0_CLK_IN               86
#define IO_SOURCE_SPI0_CSN_IN               87
#define IO_SOURCE_SPI0_HOLD_IN              88
#define IO_SOURCE_SPI0_WP_IN                89
#define IO_SOURCE_SPI0_MISO_IN              90
#define IO_SOURCE_SPI0_MOSI_IN              91
#define IO_SOURCE_SPI1_CLK_IN               92
#define IO_SOURCE_SPI1_CSN_IN               93
#define IO_SOURCE_SPI1_MISO_IN              94
#define IO_SOURCE_SPI1_MOSI_IN              95
#define IO_SOURCE_SPI1_HOLD_IN              96
#define IO_SOURCE_SPI1_WP_IN                97
#define IO_SOURCE_IR_DATA_IN                98
#define IO_SOURCE_I2S_BCLK_IN               99
#define IO_SOURCE_I2S_LRCLK_IN              100
#define IO_SOURCE_I2S_DATA_IN               101
#define IO_SOURCE_UART0_RXD                 102
#define IO_SOURCE_UART0_CTS                 103
#define IO_SOURCE_UART1_RXD                 104
#define IO_SOURCE_UART1_CTS                 105
#define IO_SOURCE_I2C0_SCL_IN               106
#define IO_SOURCE_I2C0_SDA_IN               107
#define IO_SOURCE_I2C1_SCL_IN               108
#define IO_SOURCE_I2C1_SDA_IN               109
#define IO_SOURCE_PDM_DMIC_IN               110
#define IO_SOURCE_KEYSCN_IN_COL_0           111
#define IO_SOURCE_KEYSCN_IN_COL_1           112
#define IO_SOURCE_KEYSCN_IN_COL_2           113
#define IO_SOURCE_KEYSCN_IN_COL_3           114
#define IO_SOURCE_KEYSCN_IN_COL_4           115
#define IO_SOURCE_KEYSCN_IN_COL_5           116
#define IO_SOURCE_KEYSCN_IN_COL_6           117
#define IO_SOURCE_KEYSCN_IN_COL_7           118
#define IO_SOURCE_KEYSCN_IN_COL_8           119
#define IO_SOURCE_KEYSCN_IN_COL_9           120
#define IO_SOURCE_KEYSCN_IN_COL_10          121
#define IO_SOURCE_KEYSCN_IN_COL_11          122
#define IO_SOURCE_KEYSCN_IN_COL_12          123
#define IO_SOURCE_KEYSCN_IN_COL_13          124
#define IO_SOURCE_KEYSCN_IN_COL_14          125
#define IO_SOURCE_KEYSCN_IN_COL_15          126
#define IO_SOURCE_KEYSCN_IN_COL_16          127
#define IO_SOURCE_KEYSCN_IN_COL_17          128
#define IO_SOURCE_KEYSCN_IN_COL_18          129
#define IO_SOURCE_KEYSCN_IN_COL_19          130
#define IO_SOURCE_SPI2AHB_SCLK              131
#define IO_SOURCE_SPI2AHB_CS                132
#define IO_SOURCE_SPI2AHB_DI                133
#define IO_SOURCE_QDEC_PHASEA               134
#define IO_SOURCE_QDEC_PHASEB               135
#define IO_SOURCE_QDEC_INDEX                136
#define IO_SOURCE_QDEC_EXT_IN_CLK           137
#define IO_SOURCE_QDEC_TIMER_EXT_IN1_A      138
#define IO_SOURCE_QDEC_TIMER_EXT_IN2_A      139
#define IO_SOURCE_QDEC_TIMER_EXT_IN2_B      140
#define IO_SOURCE_PCAP0_IN                  141
#define IO_SOURCE_PCAP1_IN                  142
#define IO_SOURCE_PCAP2_IN                  143
#define IO_SOURCE_PCAP3_IN                  144
#define IO_SOURCE_PCAP4_IN                  145
#define IO_SOURCE_PCAP5_IN                  146

#define IO_SOURCE_ADC_IN		    255


#endif
