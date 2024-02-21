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
    platform_printf("HARDFAULT:\nPC : 0x%08X\nLR : 0x%08X\nPSR: 0x%08X\n"
                    "R0 : 0x%08X\nR1 : 0x%08X\nR2 : 0x%08X\nP3 : 0x%08X\n"
                    "R12: 0x%08X\n",
                    info->pc, info->lr, info->psr,
                    info->r0, info->r1, info->r2, info->r3, info->r12);
    for (;;);
}

static uint32_t cb_assertion(assertion_info_t *info, void *_)
{
    platform_printf("[ASSERTION] @ %s:%d\n",
                    info->file_name,
                    info->line_no);
    for (;;);
}

static uint32_t cb_heap_out_of_mem(uint32_t tag, void *_)
{
    platform_printf("[OOM] @ %d\n", tag);
    for (;;);
}

#define PRINT_PORT    APB_UART0

uint32_t cb_putc(char *c, void *dummy)
{
    while (apUART_Check_TXFIFO_FULL(PRINT_PORT) == 1);
    UART_SendData(PRINT_PORT, (uint8_t)*c);
    while (apUART_Check_TXFIFO_FULL(APB_UART1) == 1);
    UART_SendData(APB_UART1, (uint8_t)*c);
    return 0;
}

int fputc(int ch, FILE *f)
{
    cb_putc((char *)&ch, NULL);
    return ch;
}
static uint32_t uart_isr0(void *user_data)
{
    // __disable_irq();
    // while(1);
    uint32_t status;

    while(1)
    {
        status = apUART_Get_all_raw_int_stat(APB_UART0);
        if (status == 0)
            break;

        APB_UART0->IntClear = status;

        while (apUART_Check_RXFIFO_EMPTY(APB_UART0) != 1)
        {
            char c = APB_UART0->DataRead;
            platform_printf("%c", c);
            // console_rx_data(&c, 1);
        }
    }
    return 0;
}

static uint32_t uart_isr1(void *user_data)
{
    // __disable_irq();
    // while(1);
    uint32_t status;

    while(1)
    {
        status = apUART_Get_all_raw_int_stat(APB_UART1);
        if (status == 0)
            break;

        APB_UART1->IntClear = status;

        while (apUART_Check_RXFIFO_EMPTY(APB_UART1) != 1)
        {
            char c = APB_UART1->DataRead;
            platform_printf("%c", c);
            // console_rx_data(&c, 1);
        }
    }
    return 0;
}
void config_user_uart(uint32_t freq, uint32_t baud)
{
    UART_sStateStruct config;

    config.word_length       = UART_WLEN_8_BITS;
    config.parity            = UART_PARITY_NOT_CHECK;
    config.fifo_enable       = 1;
    config.two_stop_bits     = 0;
    config.receive_en        = 1;
    config.transmit_en       = 1;
    config.UART_en           = 1;
    config.cts_en            = 0;
    config.rts_en            = 0;
    config.rxfifo_waterlevel = 4;
    config.txfifo_waterlevel = 1;
    config.ClockFrequency    = freq;
    config.BaudRate          = baud;

    apUART_Initialize(APB_UART0, &config, (1 << bsUART_RECEIVE_INTENAB) | (1 << bsUART_TIMEOUT_INTENAB));
    apUART_Initialize(APB_UART1, &config, (1 << bsUART_RECEIVE_INTENAB) | (1 << bsUART_TIMEOUT_INTENAB));

#ifdef TRACE_TO_UART
    //config.BaudRate          = 921600;
    apUART_Initialize(TRACE_PORT, &config, 1 << bsUART_TRANSMIT_INTENAB);
#endif
}
void setup_peripherals(void)
{
     cube_setup_peripherals();//
     config_user_uart(OSC_CLK_FREQ, 115200);
    platform_set_irq_callback(PLATFORM_CB_IRQ_UART0, uart_isr0, NULL);
    platform_set_irq_callback(PLATFORM_CB_IRQ_UART1, uart_isr1, NULL);
//platform_set_irq_callback(PLATFORM_CB_IRQ_GPIO, (f_platform_irq_cb)&gpio_isr, NULL);
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

trace_rtt_t trace_ctx = {0};


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
            .f = (f_platform_evt_cb)cb_trace_rtt,
            .user_data = &trace_ctx,
        },
    }
};

// TODO: add RTOS source code to the project.
uintptr_t app_main()
{
    cube_soc_init();
    
    // setup event handlers
    platform_set_evt_callback_table(&evt_cb_table);

    setup_peripherals();
    SYSCTRL_Init();

    trace_rtt_init(&trace_ctx);
    // TODO: config trace mask
    platform_config(PLATFORM_CFG_TRACE_MASK, 0);
    return (uintptr_t)os_impl_get_driver();
}



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

    if (tid == 0) {
        printk("无法创建线程\n");
    } else { 
        printk("成功创建线程\n");
    }
    // port_task_create( "test",create_task_test, NULL,1024, 5);
    // k_timer_init(&test_timer, test_time_cb, NULL);
    // k_timer_start(&test_timer, K_MSEC(1000), K_MSEC(1000));
    os_impl_task_create_real();
    
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
// }
// void thread_test (void) {
//     while(1) {
//         // k_sleep(1);
//         // port_event_wait(NULL);
//         // k_sem_take(&test_binary_semaphore, K_FOREVER);
//         platform_printf("I am in thread test\r\n");
//         #ifdef POWER_SAVING
//         platform_printf("Defined POWER SAVING\r\n"); 
//         #ifdef LISTEN_TO_POWER_SAVING
//         platform_printf("Defined LISTEN TO POWER SAVING\r\n");
//         #endif 
//         #endif
//          char buf[20] = {0};
//         //  int ret = k_msgq_get(&my_msgq, buf, K_FOREVER);
//         k_sleep(K_MSEC(1000));
//         // if (ret == 0) {
//         //     platform_printf("received %s\r\n",buf);
//         // }
//         break;
//     }
}
// K_THREAD_DEFINE(blink0_id, 1024, thread_test, NULL, NULL, NULL, 1, 0, 0);//it works

const char welcome_msg[] = "Built with zephyr";
