/*
 * Copyright (c) 2021 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/irq.h>
#include "ingsoc.h"
#include "peripheral_pinctrl.h"
#include "peripheral_sysctrl.h"
#include "peripheral_gpio.h"
#include "peripheral_ssp.h"
#include "spi_context.h"
#include <zephyr/logging/log.h>


#define SPI_MIC_CLK         GIO_GPIO_7
#define SPI_MIC_MOSI        GIO_GPIO_8
#define SPI_MIC_MISO        GIO_GPIO_9
#define SPI_MIC_CS          GIO_GPIO_10
#define SPI_MIC_WP          GIO_GPIO_11
#define SPI_MIC_HOLD        GIO_GPIO_12

struct spi_ingchips_config {
	uintptr_t reg;                          // 寄存器地址
	uint32_t clkid;                         // 时钟 ID
	apSSP_sDeviceControlBlock reset;                // 复位配置结构体
	const struct pinctrl_dev_config *pcfg;  // 引脚控制配置指针
#ifdef CONFIG_SPI_ingchips_DMA
	struct dma_config *dma;                 // DMA 配置
#endif
#ifdef CONFIG_SPI_ingchips_INTERRUPT
	void (*irq_configure)(void);            // 中断配置函数指针
#endif
};

struct spi_ingchips_data {
	struct spi_context ctx;                 // SPI 上下文结构体，包含锁和同步变量
};



static bool spi_ingchips_transfer_ongoing(struct spi_ingchips_data *data) {
    return 1;
}

static int spi_ingchips_configure(const struct device *dev,
			      const struct spi_config *config)
{

    SYSCTRL_ClearClkGateMulti(    (1 << SYSCTRL_ITEM_APB_SPI1)
                                  | (1 << SYSCTRL_ITEM_APB_PinCtrl));

    PINCTRL_SelSpiIn(SPI_PORT_1, SPI_MIC_CLK, SPI_MIC_CS, SPI_MIC_HOLD,
                     SPI_MIC_WP, SPI_MIC_MISO, SPI_MIC_MOSI);
    PINCTRL_SetPadMux(SPI_MIC_CLK, IO_SOURCE_SPI1_CLK_OUT);
    PINCTRL_SetPadMux(SPI_MIC_CS, IO_SOURCE_SPI1_CSN_OUT);
    PINCTRL_SetPadMux(SPI_MIC_MOSI, IO_SOURCE_SPI1_MOSI_OUT);

    apSSP_sDeviceControlBlock pParam;

    //速率选择
    pParam.eSclkDiv = SPI_INTERFACETIMINGSCLKDIV_DEFAULT_24M;
    pParam.eSCLKPolarity = SPI_CPOL_SCLK_HIGH_IN_IDLE_STATES;
    pParam.eSCLKPhase = SPI_CPHA_ODD_SCLK_EDGES;
    pParam.eLsbMsbOrder = SPI_LSB_MOST_SIGNIFICANT_BIT_FIRST;
    pParam.eDataSize = SPI_DATALEN_8_BITS;
    pParam.eMasterSlaveMode = SPI_SLVMODE_MASTER_MODE;//主从模式
    pParam.eReadWriteMode = SPI_TRANSMODE_READ_ONLY;
    pParam.eQuadMode = SPI_DUALQUAD_REGULAR_MODE;//设置spi传输使用的io模式两线三线四线
    //单次发送数据大小 fifo满
    pParam.eWriteTransCnt = 8;
    pParam.eReadTransCnt = 8;
    pParam.eAddrEn = SPI_ADDREN_DISABLE;//地址使能
    pParam.eCmdEn = SPI_CMDEN_DISABLE;//命令使能
    //中断触发fifo阈值 半满中断
    pParam.RxThres = 4;
    pParam.TxThres = 4;
    //仅数据模式，该模式仅从机模式有效，必须处于全双工模式
    pParam.SlaveDataOnly = SPI_SLVDATAONLY_ENABLE;
    //地址长度 flash型号使用3字节地址
    pParam.eAddrLen = SPI_ADDRLEN_2_BYTES;
    //中断触发掩码
//    pParam.eInterruptMask = (1 << bsSPI_INTREN_ENDINTEN)|(1<<bsSPI_INTREN_RXFIFOINTEN);
    pParam.eMOSI_Dir = SPI_MOSI_UNI_DIR_MODE;

    apSSP_DeviceParametersSet(AHB_SSP0, &pParam);
    apSSP_SetTransferControlRdTranCnt(AHB_SSP0,65535);

	return 0;
}

static int spi_ingchips_frame_exchange(const struct device *dev)
{
	return 1;
}


static int spi_ingchips_transceive_impl(const struct device *dev,
				    const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs,
				    spi_callback_t cb,
				    void *userdata)
{
    uint32_t i,buf_void;
    if(tx_bufs == NULL && rx_bufs == NULL)
        return 0;
    if(tx_bufs == NULL) {
        for(i=0;i<rx_bufs->count;i++) {
            for (i=0;i<rx_bufs->buffers[i].len;i++) {
                apSSP_WriteFIFO(AHB_SSP0, 0);
                while (!apSSP_RxFifoEmpty(AHB_SSP0));
                apSSP_ReadFIFO(AHB_SSP0, rx_bufs->buffers[i].buf);
            }
        }
        return 1;
    }
    if(rx_bufs == NULL) {
        for(i=0;i<tx_bufs->count;i++) {
            for (i=0;i<tx_bufs->buffers[i].len;i++) {
                apSSP_WriteFIFO(AHB_SSP0, *((uint32_t*)(tx_bufs->buffers[i].buf) + i));
                while (!apSSP_RxFifoEmpty(AHB_SSP0));
                apSSP_ReadFIFO(AHB_SSP0, &buf_void);
            }
        }
        return 1;
    }
    for(i=0;i<tx_bufs->count;i++) {
        for (i=0;i<tx_bufs->buffers[i].len;i++) {
            apSSP_WriteFIFO(AHB_SSP0,
                            *((uint32_t*)(tx_bufs->buffers[i].buf) + i));
            while (!apSSP_RxFifoEmpty(AHB_SSP0));
            apSSP_ReadFIFO(AHB_SSP0, rx_bufs->buffers->buf);
        }
    }
	return 1;
}

static int spi_ingchips_transceive(const struct device *dev,
			       const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{

    return spi_ingchips_transceive_impl(dev, config, tx_bufs, rx_bufs, NULL, NULL);
}


static int spi_ingchips_release(const struct device *dev,
			    const struct spi_config *config)
{
    return 0;
}

static struct spi_driver_api spi_ingchips_driver_api = {
};

int spi_ingchips_init(const struct device *dev)
{
	const struct spi_ingchips_config *cfg = dev->config;
	int err;
	/* Configure dt provided device signals when available */
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}
	return 0;
}

#define ingchips_IRQ_CONFIGURE(idx)						   \
	static void spi_ingchips_irq_configure_##idx(void)			   \
	{								   \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), \
			    spi_ingchips_isr,				   \
			    DEVICE_DT_INST_GET(idx), 0);		   \
		irq_enable(DT_INST_IRQN(idx));				   \
	}

#define INGCHIPS_SPI_INIT(idx)						       \
	PINCTRL_DT_INST_DEFINE(id); \
	IF_ENABLED(CONFIG_SPI_ingchips_INTERRUPT, (ingchips_IRQ_CONFIGURE(idx)));      \
	static struct spi_ingchips_data spi_ingchips_data_##idx = {		       \
		SPI_CONTEXT_INIT_LOCK(spi_ingchips_data_##idx, ctx),	       \
		SPI_CONTEXT_INIT_SYNC(spi_ingchips_data_##idx, ctx),	       \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(idx), ctx) };      \
	static struct spi_ingchips_config spi_ingchips_config_##idx = {		       \
		.reg = DT_INST_REG_ADDR(idx),				       \
		.clkid = DT_INST_CLOCKS_CELL(idx, id),			       \
		.reset = RESET_DT_SPEC_INST_GET(idx),			       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),		       \
		IF_ENABLED(CONFIG_SPI_ingchips_DMA, (.dma = DMAS_DECL(idx),))      \
		IF_ENABLED(CONFIG_SPI_ingchips_INTERRUPT,			       \
			   (.irq_configure = spi_ingchips_irq_configure_##idx)) }; \
	DEVICE_DT_INST_DEFINE(idx, &spi_ingchips_init, NULL,		       \
			      &spi_ingchips_data_##idx, &spi_ingchips_config_##idx,    \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,	       \
			      &spi_ingchips_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INGCHIPS_SPI_INIT)
