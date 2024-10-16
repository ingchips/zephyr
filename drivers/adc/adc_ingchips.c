#define DT_DRV_COMPAT ingchips_ing_adc

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/pm/device.h>

#include "peripheral_sysctrl.h"
#include "peripheral_adc.h"

SADC_ftCali_t Adc_FtCali;
typedef void (*config_func_t)(const struct device *dev);
struct adc_ingchips_cfg {
    uint32_t base_address;
    config_func_t config_func;
};


//get mask have channels
static int adc_ingchips_get_read_channel(uint32_t channels,uint8_t *channel_buffer)
{
    uint8_t i,num;
    num = 0;
    for(i=0;i<32;i++)
    {
        if(channels & 0x1)
        {
            channel_buffer[num] = i;
            num++;
        }
        channels = channels>>1;
    }

    return num;
}

//check channel
static int adc_ingchips_check_channel(uint8_t channel, uint8_t *channel_buf, uint8_t buf_num)
{
    uint8_t i;
    for(i=0;i<buf_num;i++)
    {
        if(channel_buf[i] == channel)
            return i;
    }
    return 0;
}

static int adc_ingchips_async_callback(const struct device *dev, const struct adc_sequence *sequence, struct k_poll_signal *async)
{
    uint32_t rc_data;
    uint16_t num;
    SADC_channelId channel;
    uint8_t channel_id_num;
    uint8_t channel_id_buf[32];

//    channel_id_num = adc_ingchips_get_read_channel(sequence->channels,channel_id_buf);
//    if(channel_id_num < sequence->buffer_size)
//        return -EIO;//IO处理操作错误
//
//    while(ADC_GetFifoEmpty())
//    {
//        rc_data = ADC_PopFifoData();
//        channel = ADC_GetDataChannel(rc_data);
//        num = adc_ingchips_check_channel(channel,channel_id_buf,channel_id_num);
//        if(num)
//            *((uint16_t*)sequence->buffer + num)= ADC_GetData(rc_data);
//        else
//            return -EIO;
//    }
    return 0;
}

static int adc_ingchips_channel_setup(const struct device *dev,
                                        const struct adc_channel_cfg *channel_cfg)
{
    SYSCTRL_SetAdcClkDiv(24 / 6);
    ADC_Reset();
    ADC_ftInitCali(&Adc_FtCali);
    ADC_Calibration((SADC_adcIputMode)channel_cfg->differential);

    //now not used int,disable pga
    ADC_ConvCfg(SINGLE_MODE, PGA_PARA_1, 0, (SADC_channelId)channel_cfg->channel_id,
                0, 0, (SADC_adcIputMode)channel_cfg->differential, channel_cfg->acquisition_time);
    return 0;
}

static int adc_ingchips_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
    uint32_t rc_data;
    uint16_t num;
    SADC_channelId channel;
    uint8_t channel_id_num;
    uint8_t channel_id_buf[32];

    channel_id_num = adc_ingchips_get_read_channel(sequence->channels,channel_id_buf);
    if(channel_id_num < sequence->buffer_size)
        return -EIO;//IO处理操作错误

    ADC_Start(1);
    while(!ADC_GetFifoEmpty());
    while(ADC_GetFifoEmpty())
    {
        rc_data = ADC_PopFifoData();
        channel = ADC_GetDataChannel(rc_data);
        num = adc_ingchips_check_channel(channel,channel_id_buf,channel_id_num);
        if(num)
            *((uint16_t*)sequence->buffer + num)= ADC_GetData(rc_data);
        else
            return -EIO;
    }
    return 0;
}

static int adc_ingchips_start_read_async(const struct device *dev, const struct adc_sequence *sequence, struct k_poll_signal *async)
{
    ADC_Start(1);
    //Register interrupt callback function
    return 0;
}

static int adc_ingchips_data_0(const struct device *dev, const struct adc_sequence *sequence)
{
    ADC_Start(1);
    return 0;
}

static int adc_ingchips_init(const struct device *dev)
{
    SYSCTRL_ClearClkGate(SYSCTRL_ITEM_APB_ADC);
    SYSCTRL_ReleaseBlock(SYSCTRL_ITEM_APB_ADC);

    return 0;
}

static const struct adc_driver_api adc_ingchips_api = {
        .channel_setup = adc_ingchips_channel_setup,
        .read = adc_ingchips_start_read,
#ifdef CONFIG_ADC_ASYNC
        .read_async = adc_ingchips_start_read_async;
#endif
        .ref_internal = ADC_REF_VDD_1,
};

#define INGCHIPS_ADC_DEVICE(n) \
                  \
static void ingchips_## n ##_config_func(const struct device *dev)     \
{                              \
      \
    IRQ_CONNECT(DT_INST_IRQN(n), \
    DT_INST_IRQ(n, priority), \
    adc_ingchips_async_callback, \
    DEVICE_DT_INST_GET(n), 0); \
    irq_enable(DT_INST_IRQN(n));    \
}                              \
                               \
static const struct adc_ingchips_cfg adc_inchips_port = { \
        .base_address = DT_INST_REG_ADDR(n),              \
        .config_func = ingchips_## n ##_config_func,                       \
        };                     \
DEVICE_DT_INST_DEFINE(n, adc_ingchips_init, NULL, \
              &adc_ingchips_data_0,       \
              &adc_inchips_port,        \
              POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,    \
              &adc_ingchips_api);
DT_INST_FOREACH_STATUS_OKAY(INGCHIPS_ADC_DEVICE)
