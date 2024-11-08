/*
 * Copyright (c) 2021 Telink Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ingchips_ing_i2c

#if 1

#include "ingsoc.h"
#include "peripheral_i2c.h"
#include "peripheral_pinctrl.h"
#include "peripheral_sysctrl.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_ingchips);

#include <zephyr/drivers/i2c.h>
#include "i2c-priv.h"
#include <zephyr/drivers/pinctrl.h>

/* I2C configuration structure */
struct i2c_ingchips_cfg {
    uint32_t bitrate;
    const struct pinctrl_dev_config *pcfg;
};

/* I2C data structure */
struct i2c_ingchips_data {
    struct k_sem mutex;
};

/* API implementation: configure */
static int i2c_ingchips_configure(const struct device *dev, uint32_t dev_config) {
    I2C_Role i2c_mode;
    I2C_AddressingMode i2c_addr;
    I2C_ClockFrequenyOptions i2c_speed;

    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_I2C0)
                              | (1 << SYSCTRL_ITEM_APB_PinCtrl));

    ARG_UNUSED(dev);


    /* check address size */
    if (dev_config & I2C_ADDR_10_BITS) {
        i2c_addr = I2C_ADDRESSING_MODE_10BIT;
    } else {
        i2c_addr = I2C_ADDRESSING_MODE_07BIT;
    }
    /* check I2C Master/Slave configuration */
    if (!(dev_config & I2C_MODE_CONTROLLER)) {
        i2c_mode = I2C_ROLE_MASTER;
    } else {
        i2c_mode = I2C_ROLE_SLAVE;
    }

    /* check i2c speed */
    switch (I2C_SPEED_GET(dev_config)) {
        case I2C_SPEED_STANDARD:
            i2c_speed = I2C_CLOCKFREQUENY_STANDARD;//100k;
            break;

        case I2C_SPEED_FAST:
            i2c_speed = I2C_CLOCKFREQUENY_FASTMODE;//400k
            break;

        case I2C_SPEED_FAST_PLUS:
            i2c_speed = I2C_CLOCKFREQUENY_FASTMODE_PLUS;//1M
            break;
        case I2C_SPEED_HIGH:
        case I2C_SPEED_ULTRA:
        default:
            LOG_ERR("Unsupported I2C speed requested");
            return -ENOTSUP;
    }
    I2C_ConfigClkFrequency(APB_I2C0, i2c_speed);
    PINCTRL_SelI2cIn(I2C_PORT_0, 5, 6);
    I2C_Config(APB_I2C0, i2c_mode, i2c_addr, 0x00);

    return 0;
}

static uint8_t i2c_master_read(uint16_t addr, uint8_t *data, uint8_t len){
    uint8_t i;
    I2C_CtrlUpdateDataCnt(APB_I2C0,len);
    APB_I2C0->Addr = addr;
    for(i=0;i<len;i++){
        while((I2C_FifoEmpty(APB_I2C0)));
        data[i] = I2C_DataRead(APB_I2C0);
    }
    return 0;
}

static uint8_t i2c_master_write(uint16_t addr, uint8_t *data, uint8_t len){
    uint8_t i;
    I2C_CtrlUpdateDataCnt(APB_I2C0,len);
    APB_I2C0->Addr = addr;
    for(i=0;i<len;i++){
        while((I2C_FifoFull(APB_I2C0)));
        I2C_DataWrite(APB_I2C0,data[i]);
    }
    return 0;
}

/* API implementation: transfer */
static int i2c_ingchips_transfer(const struct device *dev,
                                 struct i2c_msg *msgs,
                                 uint8_t num_msgs,
                                 uint16_t addr) {
    int status = 0;
    uint8_t send_stop = 0;
    struct i2c_ingchips_data *data = dev->data;

    /* get the mutex */
    k_sem_take(&data->mutex, K_FOREVER);

    /* loop through all messages */
    for (int i = 0; i < num_msgs; i++) {
        /* check addr size */
        if (msgs[i].flags & I2C_MSG_ADDR_10_BITS) {
            LOG_ERR("10-bits address is not supported");
            k_sem_give(&data->mutex);
            return -ENOTSUP;
        }

//        /* config stop bit */
//        send_stop = msgs[i].flags & I2C_MSG_STOP ? 1 : 0;
//        i2c_master_send_stop(send_stop);

        /* transfer data */
        if (msgs[i].flags & I2C_MSG_READ) {
            status = i2c_master_read(addr, msgs[i].buf, msgs[i].len);
        } else {
            status = i2c_master_write(addr, msgs[i].buf, msgs[i].len);
        }
        /* check status */
        if (!status) {
            LOG_ERR("Failed to transfer I2C messages\n");
            k_sem_give(&data->mutex);
            return -EIO;
        }
    }

    /* release the mutex */
    k_sem_give(&data->mutex);

    return 0;
};

/* API implementation: init */
static int i2c_ingchips_init(const struct device *dev) {
    int status = 0;
    const struct i2c_ingchips_cfg *cfg = dev->config;
    struct i2c_ingchips_data *data = dev->data;
    uint32_t dev_config = (I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(cfg->bitrate));

    /* init mutex */
    k_sem_init(&data->mutex, 1, 1);
    /* config i2c on startup */
    status = i2c_ingchips_configure(dev, dev_config);
    if (status != 0) {
        LOG_ERR("Failed to configure I2C on init");
        return status;
    }

    /* configure pins */
//	status = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (status < 0) {
        LOG_ERR("Failed to configure I2C pins");
        return status;
    }

    return 0;
}

/* I2C driver APIs structure */
static const struct i2c_driver_api i2c_ingchips_api = {
        .configure = i2c_ingchips_configure,
        .transfer = i2c_ingchips_transfer,
};

//static struct i2c_ingchips_cfg i2c_ingchips_cfg_##inst = {          \
///*        .bitrate = DT_INST_PROP(inst, clock_frequency),	      \
//        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),         \*/\
//    };                                  \
/*	PINCTRL_DT_INST_DEFINE(inst);				      \*/\

/* I2C driver registration */
#define I2C_ingchips_INIT(inst)                          \
                                      \
                                      \
    static struct i2c_ingchips_data i2c_ingchips_data_##inst;              \
                                      \
    static struct i2c_ingchips_cfg i2c_ingchips_cfg_##inst = {             \
    .bitrate = DT_INST_PROP(inst, clock_frequency),      \
    };                                  \
                                      \
I2C_DEVICE_DT_INST_DEFINE(inst, i2c_ingchips_init,              \
                  NULL,                      \
                  &i2c_ingchips_data_##inst,              \
                  &i2c_ingchips_cfg_##inst,              \
                  POST_KERNEL,                  \
                  CONFIG_I2C_INIT_PRIORITY,          \
                  &i2c_ingchips_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_ingchips_INIT)

#endif 