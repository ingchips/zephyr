#include "port_gen_os_driver.h"
#include "platform_api.h"
#include "profile.h"
#include "ingsoc.h"
#include <zephyr/kernel.h>
// #include <zephyr.h>
#define _SYSTICK_PRI    (*(uint8_t  *)(0xE000ED23UL))
typedef struct {
    const char* task_name;
    void (*entry)(void *);
    void *parameter;
    uint32_t stack_size;
    enum gen_os_task_priority priority; 
} port_task_entry_def;
/* Constants required to manipulate the core.  Registers first... */
#define portNVIC_SYSTICK_CTRL_REG			( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG			( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG	( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSPRI2_REG				( * ( ( volatile uint32_t * ) 0xe000ed20 ) )
/* ...then bits in the registers. */
#define portNVIC_SYSTICK_CLK_BIT	        ( 0UL << 2UL )
#define portNVIC_SYSTICK_INT_BIT			( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT			( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT		( 1UL << 16UL )
#define portNVIC_PENDSVCLEAR_BIT 			( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT		( 1UL << 25UL )
#define portSY_FULL_READ_WRITE              15
typedef uint32_t TickType_t;

#define QUEUE_SIZE 64
#define MSG_SIZE   64//sizeof(int)
struct k_msgq my_msgq1;
K_MSGQ_DEFINE(my_queue, MSG_SIZE, QUEUE_SIZE, 4);

uint32_t _SysTick_Config(uint32_t ticks)
{
    if ((ticks - 1) > 0xFFFFFF)
    {
        return 1;
    }

    _SYSTICK_PRI = 0xFF;
    portNVIC_SYSTICK_LOAD_REG = ticks - 1;    
    portNVIC_SYSTICK_CURRENT_VALUE_REG  = 0;
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT | portNVIC_SYSTICK_CLK_BIT;

    return 0;
}
struct timer_user_data
{
    void *user_data;
    void (* timer_cb)(void *);
};

static void os_timer_cb(struct k_timer *xTimer)
{
    struct timer_user_data *data = (struct timer_user_data *)xTimer->user_data;
    data->timer_cb(data->user_data);
}
#define OPEN_DEBUG 1
static struct k_timer my_timer;
k_timeout_t g_time_set;
void os_open_psp_control(void);

gen_handle_t port_timer_create(
        uint32_t timeout_in_ms,
        void *user_data,
        void (* timer_cb)(void *)
    )
{
    #ifdef OPEN_DEBUG
    platform_printf("port timer create %d,%p,%p\r\n", timeout_in_ms, user_data, timer_cb);
    platform_printf("%.10s%d%s\r\n", __FILE__, __LINE__, __func__);
    #endif
    struct timer_user_data *data = (struct timer_user_data *)k_malloc(sizeof(struct timer_user_data));
    data->user_data = user_data;
    data->timer_cb = timer_cb; 
	k_timer_init(&my_timer, os_timer_cb, data);
    g_time_set.ticks = timeout_in_ms;
    return &my_timer;
}

void port_timer_start(gen_handle_t timer)
{

    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    k_timer_start(timer, K_MSEC(g_time_set.ticks), K_NO_WAIT);
}

void port_timer_stop(gen_handle_t timer)
{
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    k_timer_stop(timer);
}

void port_timer_delete(gen_handle_t timer)
{
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    //todo free timer
    // k_free(timer);
    return;

}
#ifndef configMAX_PRIORITIES
#define configMAX_PRIORITIES 9
#endif 
#define APP_PRIO_LOW               2
#define APP_PRIO_HIGH             (configMAX_PRIORITIES - 1)
#define USER_STACKSIZE	2048
#define TASK_NUMBERS 2
K_THREAD_STACK_DEFINE(user_stack[TASK_NUMBERS], USER_STACKSIZE);
K_SEM_DEFINE(my_binary_semaphore, 0, 1);
struct k_thread user_thread[TASK_NUMBERS];
port_task_entry_def port_task_entry[TASK_NUMBERS];
static uint8_t g_creat_task_index = 0;
int g_ret_task = 123;
// void task_test1 () {
//     while(1) {
//         platform_printf("test1 thread\r\n");
//         k_sleep(K_MSEC(1000));
//         // port_event_set(NULL);
//         const char* senddata="send hello\r\n";
//         k_msgq_put(&my_msgq1 ,senddata, K_NO_WAIT);
//     }
// }
// void task_test2 () {
//     while(1) {
//         platform_printf("test2 thread\r\n");
//         // port_event_wait(NULL);
//         char buf[20] = {0};
//         int ret = k_msgq_get(&my_msgq1, buf, K_FOREVER);
//         if (ret == 0) {
//             platform_printf("received %s\r\n",buf);
//         }
//         // k_sleep(K_MSEC(1000));
//     }
// }
gen_handle_t os_impl_task_create_real()
{
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    // port_task_entry[0].entry = task_test1;
    // port_task_entry[1].entry = task_test2;
    for (uint8_t i = 0; i < g_creat_task_index; i++) {
        k_tid_t tid = k_thread_create(&user_thread[i], 
                        &user_stack[i], 
                        USER_STACKSIZE,
                        port_task_entry[i].entry, 
                        port_task_entry[i].parameter, NULL, NULL,
                        port_task_entry[i].priority, K_USER, K_MSEC(0));
        if (tid != 0) {
            k_thread_name_set(tid, port_task_entry[i].task_name);
        }
    }
    // platform_printf("ret tid = %d\r\n", tid);
    // return (gen_handle_t)123;
    return (gen_handle_t) g_ret_task;

}
gen_handle_t port_task_create(
        const char *name,
        void (*entry)(void *),
        void *parameter,
        uint32_t stack_size,                    // stack size in bytes
        enum gen_os_task_priority priority
    )
{
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    if (g_creat_task_index >= TASK_NUMBERS) {
        platform_printf("port task create too many task\r\n");
        return 0;
    }
    port_task_entry[g_creat_task_index].entry = entry;
    port_task_entry[g_creat_task_index].task_name = name;
    port_task_entry[g_creat_task_index].parameter = parameter;
    port_task_entry[g_creat_task_index].stack_size = stack_size;
    port_task_entry[g_creat_task_index].priority = priority;
    g_creat_task_index++;
    return (gen_handle_t)g_ret_task;

}


gen_handle_t port_queue_create(int len, int item_size)
{
    if(len > QUEUE_SIZE || item_size > MSG_SIZE) {
        platform_printf("port queue not enough len[%d] item_size[%d]\r\n", len, item_size);
        return NULL;
    }
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    platform_printf("%.10s %d %d\r\n", "queue", len, item_size);
    #endif
    k_msgq_init(&my_msgq1, &my_queue, item_size, len);
    return &my_msgq1;
}

int port_queue_send_msg(gen_handle_t queue, void *msg)
{
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    return k_msgq_put(queue, msg, K_NO_WAIT) == 0 ? 0 : 1;
}

// return 0 if msg received; otherwise failed (timeout)
int port_queue_recv_msg(gen_handle_t queue, void *msg)
{
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    return k_msgq_get(queue, msg, K_FOREVER)==0 ? 0 : 1;
}

gen_handle_t port_event_create()
{
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    return (gen_handle_t)&my_binary_semaphore;
}

// return 0 if msg received; otherwise failed (timeout)
int port_event_wait(gen_handle_t event)
{
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    return k_sem_take(&my_binary_semaphore, K_FOREVER) == 0 ? 0 : 1;
}

// event_set(event) will release the task in waiting.
void port_event_set(gen_handle_t event)
{
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    // if (k_is_in_isr()) {
    //     platform_printf("I am in ISR %d\r\n", __LINE__);
    // }
    k_sem_give(&my_binary_semaphore);
    return;
}

extern void sys_clock_isr(void *arg);
extern void z_cstart(void);
extern int sys_clock_driver_init();
//extern z_interrupt_stacks

void port_os_start(void) {
    /*
     * When changing the stack pointer, software must use an ISB instruction
     * immediately after the MSR instruction. This ensures that instructions
     * after the ISB instruction execute using the new stack pointer.
     */
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
 	z_arm_interrupt_init();
    sys_clock_driver_init();//打开systick的中断
    // _SysTick_Config(RTC_CLK_FREQ / 1000);//如果打开，中断不来，且不会
    os_open_psp_control();
    z_cstart();
    
}
void svc_isr_wraper(void) {
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    z_arm_exc_exit();
}
extern void z_arm_exc_exit(void);
void pendsv_isr_wraper(void) {
    #ifdef OPEN_DEBUG
    platform_printf("%.10s %d %s\r\n", __FILE__, __LINE__, __func__);
    #endif
    z_arm_pendsv();
}
const gen_os_driver_t gen_os_driver =
{
    .timer_create = port_timer_create,
    .timer_start = port_timer_start,
    .timer_stop = port_timer_stop,
    .timer_delete = port_timer_delete,

    .task_create = port_task_create,

    .queue_create = port_queue_create,
    .queue_send_msg = port_queue_send_msg,
    .queue_recv_msg = port_queue_recv_msg,

    .event_create = port_event_create,
    .event_set = port_event_set,
    .event_wait = port_event_wait,

    .malloc =  k_malloc,
    .free = k_free,
    .enter_critical = k_sched_lock,
    .leave_critical = k_sched_unlock,
    .os_start = port_os_start,
    .tick_isr = sys_clock_isr,
    .svc_isr = svc_isr_wraper,
    .pendsv_isr = pendsv_isr_wraper,
};

const gen_os_driver_t *os_impl_get_driver(void)
{
    return &gen_os_driver;
}
void os_open_psp_control(void) {
    __asm__ volatile (
    "mrs r0, CONTROL\n\t"
    "movs r1, #2\n\t"
    "orrs r0, r1\n\t"
    "msr CONTROL, r0\n\t"
    "isb\n\t"
    );
}

#if configSUPPORT_STATIC_ALLOCATION
static StaticTask_t idle_task_TCB_buffer;
static StackType_t  idle_task_stack[configMINIMAL_STACK_SIZE];
static StaticTask_t timer_task_TCB_buffer;
static StackType_t  timer_task_stack[configTIMER_TASK_STACK_DEPTH];
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &idle_task_TCB_buffer;
    *ppxIdleTaskStackBuffer = idle_task_stack;
    *pulIdleTaskStackSize = sizeof(idle_task_stack) / sizeof(idle_task_stack[0]);
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &timer_task_TCB_buffer;
    *ppxTimerTaskStackBuffer = timer_task_stack;
    *pulTimerTaskStackSize = sizeof(timer_task_stack) / sizeof(timer_task_stack[0]);
}
#endif

void vApplicationMallocFailedHook(void)
{
    #ifdef OPEN_DEBUG
    platform_printf("%s%d%s\r\n", __FILE__, __LINE__, __func__);
    #endif
    platform_raise_assertion(__FILE__, __LINE__);
}

void platform_get_heap_status(platform_heap_status_t *status)
{
    extern size_t xFreeBytesRemaining;
    extern size_t xMinimumEverFreeBytesRemaining;
    status->bytes_free = xFreeBytesRemaining;
    status->bytes_minimum_ever_free = xMinimumEverFreeBytesRemaining;
}

TickType_t sysPreSuppressTicksAndSleepProcessing(TickType_t expectedTicks)
{
    #ifdef OPEN_DEBUG
    platform_printf("%s%d%s\r\n", __FILE__, __LINE__, __func__);
    #endif
    return platform_pre_suppress_ticks_and_sleep_processing(expectedTicks);
}

void sysPreSleepProcessing(TickType_t idleTime)
{
    #ifdef OPEN_DEBUG
    platform_printf("%s%d%s\r\n", __FILE__, __LINE__, __func__);
    #endif
    platform_pre_sleep_processing();
}

void sysPostSleepProcessing(TickType_t idleTime)
{
    #ifdef OPEN_DEBUG
    platform_printf("%s%d%s\r\n", __FILE__, __LINE__, __func__);
    #endif
    platform_post_sleep_processing();
}

void vApplicationIdleResumedHook(void)
{
    #ifdef OPEN_DEBUG
    platform_printf("%s%d%s\r\n", __FILE__, __LINE__, __func__);
    #endif
    platform_os_idle_resumed_hook();
}