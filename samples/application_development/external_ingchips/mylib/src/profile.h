#ifndef _PROFILESTASK_H_
#define _PROFILESTASK_H_

#include <stdint.h>
#include "ingsoc.h"
#include "platform_api.h"

#define HCI_TRANSPORT_UART          0
#define HCI_TRANSPORT_USB           1

#ifndef HCI_TRANSPORT
#define HCI_TRANSPORT               HCI_TRANSPORT_UART
#endif

#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
    #if (HCI_TRANSPORT != HCI_TRANSPORT_UART)
        #error ING918 only support HCI_TRANSPORT_UART
    #endif
#endif

void transport_init(void);
uint32_t cb_hci_recv(const platform_hci_recv_t *data, void *user_data);
uint32_t setup_profile(void *data, void *user_data);
#endif