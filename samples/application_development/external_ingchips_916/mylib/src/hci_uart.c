#define UART_BUFF_SIZE      8192

#include "uart_driver.c"

#define THE_UART    APB_UART1
#define UART_ID     PLATFORM_CB_IRQ_UART1

uint32_t cb_hci_recv(const platform_hci_recv_t *msg, void *_)
{
    while (driver_get_free_size() < 1 + msg->len_of_hci) ;

    driver_append_tx_data(&msg->hci_type, 1);
    driver_append_tx_data(msg->buff, msg->len_of_hci);
    switch (msg->hci_type)
    {
    case HCI_EVENT_PACKET:
        hci_interf->hci_event_processed(msg->handle);
        break;
    case HCI_ACL_DATA_PACKET:
        hci_interf->acl_data_processed(msg->conn_handle, msg->handle);
        break;
    }
    return 0;
}

static void send_event(uint8_t event_code,
    const uint8_t *param,
    int param_len)
{
    static uint8_t packet[20] = {HCI_EVENT_PACKET, 0};
    packet[1] = event_code;
    packet[2] = param_len;
    memcpy(packet + 3, param, param_len);
    driver_append_tx_data(packet, param_len + 3);
}

static void send_command_status(uint8_t Num_HCI_Command_Packets,
    uint16_t Command_Opcode, uint8_t status)
{
    uint8_t param[4];
    param[0] = status;
    param[1] = Num_HCI_Command_Packets;
    little_endian_store_16(param, 2, Command_Opcode);
    send_event(0x0f, param, sizeof(param));
}

#define MAX_SIZE 512

static int rx_len = 0;
static uint8_t buffer[MAX_SIZE] = {0};

void uart_rx_byte(void *user_data, uint8_t c)
{
    if (rx_len >= MAX_SIZE)
        platform_reset();

    buffer[rx_len++] = c;

    switch (buffer[0])
    {
    case HCI_COMMAND_DATA_PACKET:
        if (rx_len >= 1 + sizeof(struct hci_command_packet_header))
        {
            struct hci_command_packet_header *header = (struct hci_command_packet_header *)(buffer + 1);
            uint8_t total_len = 1 + sizeof(struct hci_command_packet_header) + header->param_len;
            if (rx_len == total_len)
            {
                uint16_t opcode = header->opcode;
                hci_interf->send_hci_command(opcode,
                    buffer + 1 + sizeof(struct hci_command_packet_header),
                    header->param_len);
                rx_len = 0;
            }
        }
        break;
    case HCI_ACL_DATA_PACKET:
        if (rx_len >= 1 + sizeof(struct hci_acl_packet_header))
        {
            struct hci_acl_packet_header *header = (struct hci_acl_packet_header *)(buffer + 1);
            uint16_t data_len = header->data_length;
            uint16_t total_len = 1 + sizeof(struct hci_acl_packet_header) + data_len;
            if (rx_len == total_len)
            {
                uint16_t conn_handle = header->conn_handle_flags & 0x0fff;
                uint8_t pbcb = header->conn_handle_flags >> 12;
                hci_interf->send_acl_data(conn_handle, pbcb, data_len,
                    buffer + 1 + sizeof(struct hci_acl_packet_header),
                    data_len);
                rx_len = 0;
            }
        }
        break;
    default:
        platform_reset();
        break;
    }
}

void transport_init(void)
{
    // uart_driver_init(THE_UART, NULL, uart_rx_byte);
    // platform_set_irq_callback(UART_ID, uart_driver_isr, NULL);
}
