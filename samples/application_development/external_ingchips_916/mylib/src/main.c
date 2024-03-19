#include <stdio.h>
#include <string.h>
#include "profile.h"
#include "ingsoc.h"
#include "platform_api.h"
#include "port_gen_os_driver.h"
#include "trace.h"
#include "../data/setup_soc.cgen"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>

static uint32_t cb_hard_fault(hard_fault_info_t *info, void *_)
{ 
    platform_printf("HARDFAULT:\r\nPC : 0x%08X\r\nLR : 0x%08X\r\nPSR: 0x%08X\r\n"
                    "R0 : 0x%08X\r\nR1 : 0x%08X\r\nR2 : 0x%08X\r\nP3 : 0x%08X\r\n"
                    "R12: 0x%08X\r\n",
                    info->pc, info->lr, info->psr,
                    info->r0, info->r1, info->r2, info->r3, info->r12);
    for (;;);
}

static uint32_t cb_assertion(assertion_info_t *info, void *_)
{
    platform_printf("[ASSERTION] @ %s:%d\r\n",
                    info->file_name,
                    info->line_no);
    for (;;);
}

static uint32_t cb_heap_out_of_mem(uint32_t tag, void *_)
{
    platform_printf("[OOM] @ %d\r\n", tag);
    for (;;);
}

#define TRACE_PORT    APB_UART1

#define PRINT_PORT    APB_UART0

uint32_t cb_putc(char *c, void *dummy)
{
    while (apUART_Check_TXFIFO_FULL(PRINT_PORT) == 1);
    UART_SendData(PRINT_PORT, (uint8_t)*c);
    return 0;
}

int _write(int fd, char *ptr, int len)
{
    int i;
    for (i = 0; i < len; i++)
        cb_putc(ptr + i, NULL);

    return len;
}

void setup_peripherals(void)
{
    cube_setup_peripherals();
}

uint32_t on_lle_init(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    cube_on_lle_init();
    return 0;
}

uint32_t on_deep_sleep_wakeup(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    setup_peripherals();
    return 0;
}

uint32_t query_deep_sleep_allowed(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    // TODO: return 0 if deep sleep is not allowed now; else deep sleep is allowed
    return PLATFORM_ALLOW_DEEP_SLEEP;
}

trace_uart_t trace_ctx = {.port = TRACE_PORT};
#ifdef CONFIG_BT_H4_INGCHIPS
extern uint32_t cb_hci_recv(const platform_hci_recv_t *msg, void *_);
#endif
static const platform_evt_cb_table_t evt_cb_table =
{
    .callbacks = {
        [PLATFORM_CB_EVT_HARD_FAULT] = {
            .f = (f_platform_evt_cb)cb_hard_fault,
        },
        [PLATFORM_CB_EVT_ASSERTION] = {
            .f = (f_platform_evt_cb)cb_assertion,
        },
        [PLATFORM_CB_EVT_HEAP_OOM] = {
            .f = (f_platform_evt_cb)cb_heap_out_of_mem,
        },
        [PLATFORM_CB_EVT_PROFILE_INIT] = {
            .f = setup_profile,
        },
        [PLATFORM_CB_EVT_LLE_INIT] = {
            .f = on_lle_init,
        },
        [PLATFORM_CB_EVT_ON_DEEP_SLEEP_WAKEUP] = {
            .f = (f_platform_evt_cb)on_deep_sleep_wakeup,
        },
        [PLATFORM_CB_EVT_QUERY_DEEP_SLEEP_ALLOWED] = {
            .f = query_deep_sleep_allowed,
        },
        [PLATFORM_CB_EVT_PUTC] = {
            .f = (f_platform_evt_cb)cb_putc,
        },
        [PLATFORM_CB_EVT_TRACE] = {
            .f = (f_platform_evt_cb)cb_trace_uart,
            .user_data = &trace_ctx,
        },
        #ifdef CONFIG_BT_H4_INGCHIPS
        [PLATFORM_CB_EVT_HCI_RECV] = {
            .f = (f_platform_evt_cb)cb_hci_recv,
        },
        #endif
    }
};

// TODO: add RTOS source code to the project.
extern const gen_os_driver_t *os_impl_get_driver(void);
uintptr_t app_main()
{
    #ifdef CONFIG_BT_H4_INGCHIPS
    extern const platform_hci_link_layer_interf_t *hci_interf;
    hci_interf = platform_get_link_layer_interf();
    #endif 
    cube_soc_init();
    // setup event handlers
    platform_set_evt_callback_table(&evt_cb_table);
    setup_peripherals();
    platform_printf("build time %s@%s\r\n",__DATE__, __TIME__);
    platform_config(PLATFORM_CFG_POWER_SAVING, PLATFORM_CFG_ENABLE);
    return (uintptr_t)os_impl_get_driver();
}
#if 1
// struct k_thread test_thread;
// #define TEST_STACK_SIZE 128
// K_KERNEL_STACK_DEFINE(test_thread_stack, TEST_STACK_SIZE);
// static uint8_t run_times = 0;
// void my_thread_func(void *p1, void *p2, void *p3) {
//     platform_printf("my thread\r\n");
//     while (1) {
//         char *p_test = k_malloc(100);
//         memset(p_test, 0, 100);
//         snprintf(p_test,"test string %d", 1,100);
//         platform_printf("my thread func runing %p\r\n", p_test);
//         k_sleep(K_MSEC(10000));  // 线程休眠1秒
//         k_free(p_test);
//         // __disable_irq();
//         // uint32_t ticks = platform_pre_suppress_ticks_and_sleep_processing(0xffffff);
//         // if (ticks < 5) continue;
//         // sysPreSleepProcessing();
//         // sysPostSleepProcessing();
//         // __enable_irq();
//         platform_os_idle_resumed_hook();
//         printf("run myTask %d times\r\n", run_times++);
//         // printk("printk print out ok\r\n");
//     }
// }
void func_callback(void) {
    printk("I am OK@%s, %d\r\n", __FILE__, __LINE__);
}
void main() {

    os_impl_task_create_real();
    // k_tid_t tid = k_thread_create(&test_thread,             // 线程对象
    //                              test_thread_stack,
    //                               1024,                     // 栈大小
    //                               my_thread_func,           // 线程入口函数
    //                               NULL,                     // 线程参数
    //                               NULL,                     // 线程工作区
    //                               NULL,                     // 线程初始化数据
    //                               0,                        // 优先级
    //                               0,                        // 抢占选项
    //                               K_NO_WAIT);               // 退出选项
    // printk("creat thread tid %d\r\n", tid);
    k_sleep(K_MSEC(1000));
    platform_printf("CPU: %dHZ\r\n", SYSCTRL_GetHClk());
    #ifdef CONFIG_BT_H4_INGCHIPS
    char *bt_name = bt_get_name();
    bt_enable(func_callback);
    printk("\r\n ********bt name is %s\r\n", bt_name);
    #endif 
    while(1) {
        static uint8_t i = 8;
        const char* senddata="send hello\r\n";
        // platform_printf("I am ingchip main thread %d times\r\n", i++);
        k_sleep(K_MSEC(1000));
    }
}
#endif