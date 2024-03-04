#include <stdio.h>
#include <string.h>
#include "profile.h"
#include "ingsoc.h"
#include "platform_api.h"
#include "port_gen_os_driver.h"
#include "trace.h"
#include "../data/setup_soc.cgen"
#include <zephyr/kernel.h>
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
    return 0;
}

trace_uart_t trace_ctx = {.port = TRACE_PORT};

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
    }
};

// TODO: add RTOS source code to the project.
extern const gen_os_driver_t *os_impl_get_driver(void);
uintptr_t app_main()
{
    cube_soc_init();

    // setup event handlers
    platform_set_evt_callback_table(&evt_cb_table);
    setup_peripherals();
    printf("build@%s\r\n",__TIME__);

    printf("build@%d\r\n",__LINE__);

    // trace_uart_init(&trace_ctx);
    // platform_set_irq_callback(PLATFORM_CB_IRQ_UART1, (f_platform_irq_cb)trace_uart_isr, &trace_ctx);
    // TODO: config trace mask
    // platform_config(PLATFORM_CFG_TRACE_MASK, 0);
    return (uintptr_t)os_impl_get_driver();
}
#if 1
struct k_thread test_thread;
#define TEST_STACK_SIZE 1024
K_KERNEL_STACK_DEFINE(test_thread_stack, TEST_STACK_SIZE);

void my_thread_func(void *p1, void *p2, void *p3) {
    platform_printf("my thread\r\n");
    while (1) {
        char *p_test = k_malloc(100);
        memset(p_test, 0, 100);
        snprintf(p_test,"test string %d", 1,100);
        platform_printf("my thread func runing %p\r\n", p_test);
        k_sleep(K_MSEC(1000));  // 线程休眠1秒
        k_free(p_test);
        printk("printk print out ok\r\n");
    }
}
void main() {
    printk("I am in main\r\n");
    k_tid_t tid = k_thread_create(&test_thread,          // 线程对象
                                 test_thread_stack,
                                  1024,  // 栈大小
                                  my_thread_func,  // 线程入口函数
                                  NULL,           // 线程参数
                                  NULL,           // 线程工作区
                                  NULL,           // 线程初始化数据
                                  5,               // 优先级
                                  0,              // 抢占选项
                                  K_NO_WAIT);             // 退出选项
    // tester_init();
    if (tid == 0) {
        printk("无法创建线程\n");
    } else { 
        printk("成功创建线程\n");
    }
    // port_task_create( "test",create_task_test, NULL,1024, 5);
    // k_timer_init(&test_timer, test_time_cb, NULL);
    // k_timer_start(&test_timer, K_MSEC(1000), K_MSEC(1000));
    k_sleep(K_MSEC(1000));
    // os_impl_task_create_real();
    platform_printf("CPU: %dHZ\r\n", SYSCTRL_GetHClk());
    while(1) {
        static uint8_t i = 8;
        const char* senddata="send hello\r\n";
        // k_msgq_put(&my_msgq ,senddata, K_NO_WAIT);
        platform_printf("I am ingchip main thread %d times\r\n", i++);
        k_sleep(K_MSEC(1000));
        // btstack_push_user_msg(3, NULL, 0);
        // k_sem_give(&test_binary_semaphore);
        // break;
    }
}
#endif