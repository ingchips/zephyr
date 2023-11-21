#include <stdio.h>
#include <string.h>
#include "ingsoc.h"
#include "platform_api.h"
#include "profile.h"
// #include <zephyr/kernel.h>
// #include <zephyr/types.h>
// #include <stddef.h>
// // #include <zephyr/sys/printk.h>
// // #include <zephyr/sys/util.h>

// // #include <zephyr/bluetooth/bluetooth.h>
// // #include <zephyr/bluetooth/hci.h>
// // #include "../../peripheral_console/src/key_detector.h"
// void my_thread_func(void *p1, void *p2, void *p3) {
//     platform_printf("这是新线程\n");
//     while (1) {
//         platform_printf("新线程运行中\n");
//         k_sleep(K_MSEC(1000));  // 线程休眠1秒
//     }
// }
// extern const void *os_impl_get_driver(void);

#include "main_shared.c"

int app_main()
{
    platform_printf("I am in main\r\n");
    _app_main();

#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
    platform_config(PLATFORM_CFG_DEEP_SLEEP_TIME_REDUCTION, 4500);
#endif
    // return NULL;
    return (uintptr_t)os_impl_get_driver();
}
void main() {
    platform_printf("I am in main\r\n");
//     // k_tid_t tid = k_thread_create(&test_thread,          // 线程对象
//     //                              test_thread_stack,
//     //                               1024,  // 栈大小
//     //                               my_thread_func,  // 线程入口函数
//     //                               NULL,           // 线程参数
//     //                               NULL,           // 线程工作区
//     //                               NULL,           // 线程初始化数据
//     //                               5,               // 优先级
//     //                               0,              // 抢占选项
//     //                               K_NO_WAIT);             // 退出选项

//     // if (tid == 0) {
//     //     printk("无法创建线程\n");
//     // } else {
//     //     printk("成功创建线程\n");
//     // }
//     // port_task_create( "test",create_task_test, NULL,1024, 5);
//     os_impl_task_create_real();
//     // k_timer_init(&test_timer, test_time_cb, NULL);
//     // k_timer_start(&test_timer, K_MSEC(1000), K_MSEC(1000));
//     while(1) {
//         static uint8_t i = 8;
//         const char* senddata="send hello\r\n";
//         // k_msgq_put(&my_msgq ,senddata, K_NO_WAIT);
//         platform_printf("I am ingchip main %d times\r\n", i++);
//         k_sleep(K_MSEC(1000));
//         // btstack_push_user_msg(3, NULL, 0);
//         // k_sem_give(&test_binary_semaphore);
//         break;
//     }
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
