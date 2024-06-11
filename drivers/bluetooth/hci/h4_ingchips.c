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

#define HCI_BT_HCI_SEND_TIMEOUT K_MSEC(2000)
static K_SEM_DEFINE(hci_send_sem, 1, 1);

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

K_SEM_DEFINE(recv_sem, 0, 1);

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
	LOG_DBG("rx.remaining is[%d]", rx.remaining);
}

static void h4_read_hdr(void)
{
	int bytes_read = rx.hdr_len - rx.remaining;
	int ret;

	ret = hci_rec_buf_read(h4_dev, rx.hdr + bytes_read, rx.remaining);
	LOG_DBG("ret = %d\r\n", ret);
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
		LOG_DBG("Got ACL header. Payload %u bytes\r\n", rx.remaining);
		rx.have_hdr = true;
	}
}

static inline void get_iso_hdr(void)
{
	h4_read_hdr();

	if (!rx.remaining) {
		struct bt_hci_iso_hdr *hdr = &rx.iso;

		rx.remaining = bt_iso_hdr_len(sys_le16_to_cpu(hdr->len));
		LOG_DBG("Got ISO header. Payload %u bytes\r\n", rx.remaining);
		rx.have_hdr = true;
	}
}

static bool is_hci_event_discardable(const uint8_t *evt_data)
{
	uint8_t evt_type = evt_data[0];

	switch (evt_type) {
#if defined(CONFIG_BT_BREDR)
	case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
	case BT_HCI_EVT_EXTENDED_INQUIRY_RESULT:
		return true;
#endif
	case BT_HCI_EVT_LE_META_EVENT: {
		uint8_t subevt_type = evt_data[sizeof(struct bt_hci_evt_hdr)];

		switch (subevt_type) {
		case BT_HCI_EVT_LE_ADVERTISING_REPORT:
			return true;
		default:
			return false;
		}
	}
	default:
		return false;
	}
}
static struct net_buf *bt_esp_evt_recv(uint8_t *data, size_t remaining)
{
	bool discardable = false;
	struct bt_hci_evt_hdr hdr;
	struct net_buf *buf;
	size_t buf_tailroom;

	if (remaining < sizeof(hdr)) {
		LOG_ERR("Not enough data for event header");
		return NULL;
	}

	discardable = is_hci_event_discardable(data);

	memcpy((void *)&hdr, data, sizeof(hdr));
	data += sizeof(hdr);
	remaining -= sizeof(hdr);

	if (remaining != hdr.len) {
		LOG_ERR("Event payload length is not correct");
		return NULL;
	}
	LOG_DBG("len %u", hdr.len);

	buf = bt_buf_get_evt(hdr.evt, discardable, K_NO_WAIT);
	if (!buf) {
		if (discardable) {
			LOG_DBG("Discardable buffer pool full, ignoring event");
		} else {
			LOG_ERR("No available event buffers!");
		}
		return buf;
	}

	net_buf_add_mem(buf, &hdr, sizeof(hdr));

	buf_tailroom = net_buf_tailroom(buf);
	if (buf_tailroom < remaining) {
		LOG_ERR("Not enough space in buffer %zu/%zu", remaining, buf_tailroom);
		net_buf_unref(buf);
		return NULL;
	}

	net_buf_add_mem(buf, data, remaining);

	return buf;
}

static struct net_buf *bt_esp_acl_recv(uint8_t *data, size_t remaining)
{
	struct bt_hci_acl_hdr hdr;
	struct net_buf *buf;
	size_t buf_tailroom;

	if (remaining < sizeof(hdr)) {
		LOG_ERR("Not enough data for ACL header");
		return NULL;
	}

	buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
	if (buf) {
		memcpy((void *)&hdr, data, sizeof(hdr));
		data += sizeof(hdr);
		remaining -= sizeof(hdr);

		net_buf_add_mem(buf, &hdr, sizeof(hdr));
	} else {
		LOG_ERR("No available ACL buffers!");
		return NULL;
	}

	if (remaining != sys_le16_to_cpu(hdr.len)) {
		LOG_ERR("ACL payload length is not correct");
		net_buf_unref(buf);
		return NULL;
	}

	buf_tailroom = net_buf_tailroom(buf);
	if (buf_tailroom < remaining) {
		LOG_ERR("Not enough space in buffer %zu/%zu", remaining, buf_tailroom);
		net_buf_unref(buf);
		return NULL;
	}

	LOG_DBG("len %u", remaining);
	net_buf_add_mem(buf, data, remaining);

	return buf;
}

static struct net_buf *bt_esp_iso_recv(uint8_t *data, size_t remaining)
{
	struct bt_hci_iso_hdr hdr;
	struct net_buf *buf;
	size_t buf_tailroom;

	if (remaining < sizeof(hdr)) {
		LOG_ERR("Not enough data for ISO header");
		return NULL;
	}

	buf = bt_buf_get_rx(BT_BUF_ISO_IN, K_NO_WAIT);
	if (buf) {
		memcpy((void *)&hdr, data, sizeof(hdr));
		data += sizeof(hdr);
		remaining -= sizeof(hdr);

		net_buf_add_mem(buf, &hdr, sizeof(hdr));
	} else {
		LOG_ERR("No available ISO buffers!");
		return NULL;
	}

	if (remaining != bt_iso_hdr_len(sys_le16_to_cpu(hdr.len))) {
		LOG_ERR("ISO payload length is not correct");
		net_buf_unref(buf);
		return NULL;
	}

	buf_tailroom = net_buf_tailroom(buf);
	if (buf_tailroom < remaining) {
		LOG_ERR("Not enough space in buffer %zu/%zu", remaining, buf_tailroom);
		net_buf_unref(buf);
		return NULL;
	}

	LOG_DBG("len %zu", remaining);
	net_buf_add_mem(buf, data, remaining);

	return buf;
}

uint32_t cb_hci_recv(const platform_hci_recv_t *msg, void *_)
{


    //todo write fifo
    LOG_DBG("hci drv rec %d", msg->len_of_hci + 1);
    // hci_rec_buf_write(&msg->hci_type, 1);
    // hci_rec_buf_write(msg->buff, msg->len_of_hci);
	uint8_t pkt_indicator;
	struct net_buf *buf = NULL;
	size_t remaining = msg->len_of_hci;
	uint8_t *data = msg->buff;
	pkt_indicator = msg->hci_type;
	LOG_DBG("<<<%02x\r\n", pkt_indicator);
	LOG_HEXDUMP_DBG(data, remaining, "host packet data:");
	static int i = 0;
	i++;
	if( i == 1) {
		LOG_DBG("drop hci rec data");
		goto end;
	}
		switch (pkt_indicator) {
		case H4_EVT:
			buf = bt_esp_evt_recv(data, remaining);
			break;

		case H4_ACL:
			buf = bt_esp_acl_recv(data, remaining);
			break;

		case H4_SCO:
			buf = bt_esp_iso_recv(data, remaining);
			break;

		default:
			LOG_ERR("Unknown HCI type %u", pkt_indicator);
			return -1;
		}
		if (buf) {
			LOG_DBG("Calling bt_recv(%p)", buf);
			bt_recv(buf);
		}
end:

    switch (msg->hci_type)
    {
    case HCI_EVENT_PACKET:
        hci_interf->hci_event_processed(msg->handle);
        break;
    case HCI_ACL_DATA_PACKET:
        hci_interf->acl_data_processed(msg->conn_handle, msg->handle);
        break;
    }
	// k_sem_give(&recv_sem);
	k_sem_give(&hci_send_sem);
    return 0;
}

void ingchips_host_send_packet_to_controller(uint8_t *buffer, uint16_t rx_len) {
	LOG_DBG(">>>");
	LOG_HEXDUMP_DBG(buffer,rx_len,"to controller");
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
				LOG_DBG("CMD SEND");
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
				LOG_DBG("ACL DATA send");
            }
        }
        break;
    default:
		platform_printf("reset********************************************************\r\n");
		LOG_DBG("reset");
		while(1);
        platform_reset();
        break;
    }
}
static int h4_send(struct net_buf *buf)
{
	int err = 0;
	uint8_t pkt_indicator;

	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT:
		pkt_indicator = H4_ACL;
		break;
	case BT_BUF_CMD:
		pkt_indicator = H4_CMD;
		break;
	case BT_BUF_ISO_OUT:
		pkt_indicator = H4_ISO;
		break;
	default:
		LOG_ERR("Unknown type %u", bt_buf_get_type(buf));
		goto done;
	}
	net_buf_push_u8(buf, pkt_indicator);

	if (k_sem_take(&hci_send_sem, HCI_BT_HCI_SEND_TIMEOUT) == 0) {
		ingchips_host_send_packet_to_controller(buf->data, buf->len);
	} else {
		LOG_ERR("Send packet timeout error");
		err = -ETIMEDOUT;
	}
done:
	net_buf_unref(buf);
	k_sem_give(&hci_send_sem);

	return err;
}

/** Setup the HCI transport, which usually means to reset the Bluetooth IC
  *
  * @param dev The device structure for the bus connecting to the IC
  *
  * @return 0 on success, negative error value on failure
  */
int __weak bt_hci_transport_setup(const struct device *dev)
{
	//h4_discard(h4_dev, 32);
	return 0;
}

static int h4_open(void)
{
	int ret;
	k_tid_t tid;
	LOG_DBG("call %s\r\n", __FUNCTION__);
    hci_interf = (const platform_hci_link_layer_interf_t *)platform_get_link_layer_interf();
	ret = bt_hci_transport_setup(h4_dev);
	if (ret < 0) {
		LOG_ERR("ret %d", ret);
		return -EIO;
	}
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
    LOG_DBG("run @ %s %d\r\n", __FILE__, __LINE__);
	bt_hci_driver_register(&drv);
	if (hci_interf == NULL) {
		hci_interf = (const platform_hci_link_layer_interf_t *)platform_get_link_layer_interf();
	}
	return 0;
}

SYS_INIT(bt_ingchips_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
