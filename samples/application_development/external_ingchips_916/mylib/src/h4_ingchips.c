/* h4.c - H:4 UART based Bluetooth driver */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/bluetooth/hci_driver.h>
#include "platform_api.h"
#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_driver);

#include "common/bt_str.h"
#include "bluetooth.h"

#define H4_NONE 0x00
#define H4_CMD  0x01
#define H4_ACL  0x02
#define H4_SCO  0x03
#define H4_EVT  0x04
#define H4_ISO  0x05
const platform_hci_link_layer_interf_t *hci_interf = NULL;
#define MAX_SIZE 512
static int rx_len = 0;
static uint8_t buffer[MAX_SIZE] = {0};

#pragma pack (push, 1)
struct hci_command_packet_header
{
    uint16_t opcode;
    uint8_t param_len;
};

struct hci_acl_packet_header
{
    uint16_t conn_handle_flags;
    uint16_t data_length;
};

struct hci_event_packet_header
{
    uint8_t code;
    uint8_t param_len;
};
#pragma pack (pop)

static K_KERNEL_STACK_DEFINE(rx_thread_stack, CONFIG_BT_DRV_RX_STACK_SIZE);
static K_KERNEL_STACK_DEFINE(rtx_thread_stack, CONFIG_BT_DRV_RX_STACK_SIZE);
static struct k_thread rx_thread_data;
static struct k_thread rtx_thread_data;

static struct {
	struct net_buf *buf;
	struct k_fifo   fifo;

	uint16_t    remaining;
	uint16_t    discard;

	bool     have_hdr;
	bool     discardable;

	uint8_t     hdr_len;

	uint8_t     type;
	union {
		struct bt_hci_evt_hdr evt;
		struct bt_hci_acl_hdr acl;
		struct bt_hci_iso_hdr iso;
		uint8_t hdr[4];
	};
} rx = {
	.fifo = Z_FIFO_INITIALIZER(rx.fifo),
};

static struct {
	uint8_t type;
	struct net_buf *buf;
	struct k_fifo   fifo;
} tx = {
	.fifo = Z_FIFO_INITIALIZER(tx.fifo),
};
static const struct device *const h4_dev = NULL;
static inline void h4_get_type(void)
{
	/* Get packet type */
	if (hci_rec_buf_read(h4_dev, &rx.type, 1) != 1) {
		LOG_WRN("Unable to read H:4 packet type");
		rx.type = H4_NONE;
		return;
	}

	switch (rx.type) {
	case H4_EVT:
		rx.remaining = sizeof(rx.evt);
		rx.hdr_len = rx.remaining;
		break;
	case H4_ACL:
		rx.remaining = sizeof(rx.acl);
		rx.hdr_len = rx.remaining;
		break;
	case H4_ISO:
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			rx.remaining = sizeof(rx.iso);
			rx.hdr_len = rx.remaining;
			break;
		}
		__fallthrough;
	default:
		LOG_ERR("Unknown H:4 type 0x%02x", rx.type);
		rx.type = H4_NONE;
	}
}

static void h4_read_hdr(void)
{
	int bytes_read = rx.hdr_len - rx.remaining;
	int ret;

	ret = hci_rec_buf_read(h4_dev, rx.hdr + bytes_read, rx.remaining);
	if (unlikely(ret < 0)) {
		LOG_ERR("Unable to read from UART (ret %d)", ret);
	} else {
		rx.remaining -= ret;
	}
}

static inline void get_acl_hdr(void)
{
	h4_read_hdr();

	if (!rx.remaining) {
		struct bt_hci_acl_hdr *hdr = &rx.acl;

		rx.remaining = sys_le16_to_cpu(hdr->len);
		LOG_DBG("Got ACL header. Payload %u bytes", rx.remaining);
		rx.have_hdr = true;
	}
}

static inline void get_iso_hdr(void)
{
	h4_read_hdr();

	if (!rx.remaining) {
		struct bt_hci_iso_hdr *hdr = &rx.iso;

		rx.remaining = bt_iso_hdr_len(sys_le16_to_cpu(hdr->len));
		LOG_DBG("Got ISO header. Payload %u bytes", rx.remaining);
		rx.have_hdr = true;
	}
}



static inline void get_evt_hdr(void)
{
	struct bt_hci_evt_hdr *hdr = &rx.evt;

	h4_read_hdr();

	if (rx.hdr_len == sizeof(*hdr) && rx.remaining < sizeof(*hdr)) {
		switch (rx.evt.evt) {
		case BT_HCI_EVT_LE_META_EVENT:
			rx.remaining++;
			rx.hdr_len++;
			break;
#if defined(CONFIG_BT_BREDR)
		case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
		case BT_HCI_EVT_EXTENDED_INQUIRY_RESULT:
			rx.discardable = true;
			break;
#endif
		}
	}

	if (!rx.remaining) {
		if (rx.evt.evt == BT_HCI_EVT_LE_META_EVENT &&
		    (rx.hdr[sizeof(*hdr)] == BT_HCI_EVT_LE_ADVERTISING_REPORT)) {
			LOG_DBG("Marking adv report as discardable");
			rx.discardable = true;
		}

		rx.remaining = hdr->len - (rx.hdr_len - sizeof(*hdr));
		LOG_DBG("Got event header. Payload %u bytes", hdr->len);
		rx.have_hdr = true;
	}
}

static inline void copy_hdr(struct net_buf *buf)
{
	net_buf_add_mem(buf, rx.hdr, rx.hdr_len);
}

static void reset_rx(void)
{
	rx.type = H4_NONE;
	rx.remaining = 0U;
	rx.have_hdr = false;
	rx.hdr_len = 0U;
	rx.discardable = false;
}

static struct net_buf *get_rx(k_timeout_t timeout)
{
	LOG_DBG("type 0x%02x, evt 0x%02x", rx.type, rx.evt.evt);

	switch (rx.type) {
	case H4_EVT:
		return bt_buf_get_evt(rx.evt.evt, rx.discardable, timeout);
	case H4_ACL:
		return bt_buf_get_rx(BT_BUF_ACL_IN, timeout);
	case H4_ISO:
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			return bt_buf_get_rx(BT_BUF_ISO_IN, timeout);
		}
	}

	return NULL;
}

static void rx_thread(void *p1, void *p2, void *p3)
{
	struct net_buf *buf;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_DBG("started");

	while (1) {
		LOG_DBG("rx.buf %p", rx.buf);

		/* We can only do the allocation if we know the initial
		 * header, since Command Complete/Status events must use the
		 * original command buffer (if available).
		 */
		if (rx.have_hdr && !rx.buf) {
			rx.buf = get_rx(K_FOREVER);
			LOG_DBG("Got rx.buf %p", rx.buf);
			if (rx.remaining > net_buf_tailroom(rx.buf)) {
				LOG_ERR("Not enough space in buffer");
				rx.discard = rx.remaining;
				reset_rx();
			} else {
				copy_hdr(rx.buf);
			}
		}



		buf = net_buf_get(&rx.fifo, K_FOREVER);
		do {

			LOG_DBG("Calling bt_recv(%p)", buf);
			bt_recv(buf);

			/* Give other threads a chance to run if the ISR
			 * is receiving data so fast that rx.fifo never
			 * or very rarely goes empty.
			 */
			k_yield();

			buf = net_buf_get(&rx.fifo, K_NO_WAIT);
		} while (buf);
	}
}

K_MUTEX_DEFINE(ble_hci_rec_buf_mutex);
static uint8_t ble_hci_rec_buf[512];
int ble_hci_rec_buf_write_len = 0;
int ble_hci_rec_buf_read_len = 0;
int hci_rec_buf_read(const struct device *dev, uint8_t *data, int read_len) {

    k_mutex_lock(&ble_hci_rec_buf_mutex, K_FOREVER);
    int buf_len = ble_hci_rec_buf_write_len - ble_hci_rec_buf_read_len;
    int ret = buf_len > read_len ? buf_len: read_len;
    memcpy(data, &ble_hci_rec_buf[ble_hci_rec_buf_read_len], ret);
    ble_hci_rec_buf_read_len = ble_hci_rec_buf_read_len + ret;
    if (ble_hci_rec_buf_read_len == ble_hci_rec_buf_write_len) {
        ble_hci_rec_buf_write_len = 0;
        ble_hci_rec_buf_read_len = 0;
    }
    k_mutex_unlock(&ble_hci_rec_buf_mutex);
    return ret;

}
int hci_rec_buf_write(uint8_t *data,int len) {
    k_mutex_lock(&ble_hci_rec_buf_mutex, K_FOREVER);
    for(int i = 0; i < len; i++) {
        ble_hci_rec_buf[ble_hci_rec_buf_write_len++] = data[i];
    }
    k_mutex_unlock(&ble_hci_rec_buf_mutex);
}
uint32_t cb_hci_recv(const platform_hci_recv_t *msg, void *_)
{


    //todo write fifo
    LOG_DBG("hci rec %d", msg->len_of_hci + 1);
    hci_rec_buf_write(&msg->hci_type, 1);
    hci_rec_buf_write(msg->buff, msg->len_of_hci);

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


static size_t h4_discard(const struct device *uart, size_t len)
{
    ble_hci_rec_buf_write_len = 0;
    ble_hci_rec_buf_read_len = 0;
	return 0;
}

static inline void read_payload(void)
{
	struct net_buf *buf;
	uint8_t evt_flags;
	int read;

	if (!rx.buf) {
		size_t buf_tailroom;

		rx.buf = get_rx(K_NO_WAIT);
		if (!rx.buf) {
			if (rx.discardable) {
				LOG_WRN("Discarding event 0x%02x", rx.evt.evt);
				rx.discard = rx.remaining;
				reset_rx();
				return;
			}

			LOG_WRN("Failed to allocate, deferring to rx_thread");
			return;
		}

		LOG_DBG("Allocated rx.buf %p", rx.buf);

		buf_tailroom = net_buf_tailroom(rx.buf);
		if (buf_tailroom < rx.remaining) {
			LOG_ERR("Not enough space in buffer %u/%zu", rx.remaining, buf_tailroom);
			rx.discard = rx.remaining;
			reset_rx();
			return;
		}

		copy_hdr(rx.buf);
	}
    
	read = hci_rec_buf_read(h4_dev, net_buf_tail(rx.buf), rx.remaining);
	if (unlikely(read < 0)) {
		LOG_ERR("Failed to read UART (err %d)", read);
		return;
	}

	net_buf_add(rx.buf, read);
	rx.remaining -= read;

	LOG_DBG("got %d bytes, remaining %u", read, rx.remaining);
	LOG_DBG("Payload (len %u): %s", rx.buf->len, bt_hex(rx.buf->data, rx.buf->len));

	if (rx.remaining) {
		return;
	}

	buf = rx.buf;
	rx.buf = NULL;

	if (rx.type == H4_EVT) {
		evt_flags = bt_hci_evt_get_flags(rx.evt.evt);
		bt_buf_set_type(buf, BT_BUF_EVT);
	} else {
		evt_flags = BT_HCI_EVT_FLAG_RECV;
		bt_buf_set_type(buf, BT_BUF_ACL_IN);
	}

	reset_rx();

	if (IS_ENABLED(CONFIG_BT_RECV_BLOCKING) &&
	    (evt_flags & BT_HCI_EVT_FLAG_RECV_PRIO)) {
		LOG_DBG("Calling bt_recv_prio(%p)", buf);
		bt_recv_prio(buf);
	}

	if (evt_flags & BT_HCI_EVT_FLAG_RECV) {
		LOG_DBG("Putting buf %p to rx fifo", buf);
		net_buf_put(&rx.fifo, buf);
	}
}

static inline void read_header(void)
{
	switch (rx.type) {
	case H4_NONE:
		h4_get_type();
		return;
	case H4_EVT:
		get_evt_hdr();
		break;
	case H4_ACL:
		get_acl_hdr();
		break;
	case H4_ISO:
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			get_iso_hdr();
			break;
		}
		__fallthrough;
	default:
		CODE_UNREACHABLE;
		return;
	}

	if (rx.have_hdr && rx.buf) {
		if (rx.remaining > net_buf_tailroom(rx.buf)) {
			LOG_ERR("Not enough space in buffer");
			rx.discard = rx.remaining;
			reset_rx();
		} else {
			copy_hdr(rx.buf);
		}
	}
}


static void send_byte(void *user_data, uint8_t c)
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
static int hci_driver_h4_send(const struct device *dev, const uint8_t *tx_data, int size) {
    for (int i = 0; i < size; i++) {
        send_byte(NULL, tx_data[i]);
    }
}
static inline void process_tx(void)
{
	int bytes;

	if (!tx.buf) {
		tx.buf = net_buf_get(&tx.fifo, K_NO_WAIT);
		if (!tx.buf) {
			LOG_ERR("TX interrupt but no pending buffer!");
			return;
		}
	}

	if (!tx.type) {
		switch (bt_buf_get_type(tx.buf)) {
		case BT_BUF_ACL_OUT:
			tx.type = H4_ACL;
			break;
		case BT_BUF_CMD:
			tx.type = H4_CMD;
			break;
		case BT_BUF_ISO_OUT:
			if (IS_ENABLED(CONFIG_BT_ISO)) {
				tx.type = H4_ISO;
				break;
			}
			__fallthrough;
		default:
			LOG_ERR("Unknown buffer type");
			goto done;
		}

		bytes = hci_driver_h4_send(h4_dev, &tx.type, 1);
		if (bytes != 1) {
			LOG_WRN("Unable to send H:4 type");
			tx.type = H4_NONE;
			return;
		}
	}

	bytes = hci_driver_h4_send(h4_dev, tx.buf->data, tx.buf->len);
	if (unlikely(bytes < 0)) {
		LOG_ERR("Unable to write to UART (err %d)", bytes);
	} else {
		net_buf_pull(tx.buf, bytes);
	}

	if (tx.buf->len) {
		return;
	}

done:
	tx.type = H4_NONE;
	net_buf_unref(tx.buf);
	tx.buf = net_buf_get(&tx.fifo, K_NO_WAIT);
	if (!tx.buf) {
		uart_irq_tx_disable(h4_dev);
	}
}

static inline void process_rx(void)
{
	LOG_DBG("remaining %u discard %u have_hdr %u rx.buf %p len %u", rx.remaining, rx.discard,
		rx.have_hdr, rx.buf, rx.buf ? rx.buf->len : 0);

	if (rx.discard) {
		rx.discard -= h4_discard(h4_dev, rx.discard);
		return;
	}

	if (rx.have_hdr) {
		read_payload();
	} else {
		read_header();
	}
}

static void rtx_thread(const struct device *unused, void *user_data)
{
	ARG_UNUSED(unused);
	ARG_UNUSED(user_data);
while(1) {
	process_tx();
	process_rx();
    LOG_DBG("rtx\r\n");
}
		
}

static int h4_send(struct net_buf *buf)
{
	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

	net_buf_put(&tx.fifo, buf);
	return 0;
}

/** Setup the HCI transport, which usually means to reset the Bluetooth IC
  *
  * @param dev The device structure for the bus connecting to the IC
  *
  * @return 0 on success, negative error value on failure
  */
int __weak bt_hci_transport_setup(const struct device *dev)
{
	h4_discard(h4_dev, 32);
	return 0;
}

static int h4_open(void)
{
	int ret;
	k_tid_t tid;
    platform_printf("run@%s\r\n", __FUNCTION__);
	LOG_DBG("");

    const platform_hci_link_layer_interf_t *hci_interf = platform_get_link_layer_interf();
	ret = bt_hci_transport_setup(h4_dev);
	if (ret < 0) {
		return -EIO;
	}

	// uart_irq_callback_set(h4_dev, bt_uart_isr);
	tid = k_thread_create(&rx_thread_data, rx_thread_stack,
			      K_KERNEL_STACK_SIZEOF(rx_thread_stack),
			      rx_thread, NULL, NULL, NULL,
			      K_PRIO_COOP(CONFIG_BT_RX_PRIO),
			      0, K_NO_WAIT);
	k_thread_name_set(tid, "bt_rx_thread");
    tid = k_thread_create(&rtx_thread_data, rtx_thread_stack,
			      K_KERNEL_STACK_SIZEOF(rtx_thread_stack),
			      rtx_thread, NULL, NULL, NULL,
			      K_PRIO_COOP(CONFIG_BT_RX_PRIO),
			      0, K_NO_WAIT);
	k_thread_name_set(tid, "bt_rtx_thread");
	return 0;
}

#if defined(CONFIG_BT_HCI_SETUP)
static int h4_setup(void)
{
	/* Extern bt_h4_vnd_setup function.
	 * This function executes vendor-specific commands sequence to
	 * initialize BT Controller before BT Host executes Reset sequence.
	 * bt_h4_vnd_setup function must be implemented in vendor-specific HCI
	 * extansion module if CONFIG_BT_HCI_SETUP is enabled.
	 */
	extern int bt_h4_vnd_setup(const struct device *dev);

	return bt_h4_vnd_setup(h4_dev);
}
#endif

static const struct bt_hci_driver drv = {
	.name		= "H:4",
	.bus		= BT_HCI_DRIVER_BUS_UART,
	.open		= h4_open,
	.send		= h4_send,
#if defined(CONFIG_BT_HCI_SETUP)
	.setup		= h4_setup
#endif
};

static int bt_ingchips_init(void)
{
    platform_printf("run @ %s %d\r\n", __FILE__, __LINE__);
    LOG_DBG("hellp\r\n");
	if (!device_is_ready(h4_dev)) {
		return -ENODEV;
	}

	bt_hci_driver_register(&drv);

	return 0;
}

SYS_INIT(bt_ingchips_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
