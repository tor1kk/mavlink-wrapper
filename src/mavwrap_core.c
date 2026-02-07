#define DT_DRV_COMPAT mavlink_wrapper

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <mavwrap.h>
#include "mavwrap_common.h"
#include "mavwrap_dt.h"
#include <common/mavlink.h>

LOG_MODULE_REGISTER(mavwrap, CONFIG_MAVWRAP_LOG_LEVEL);

/**
 * RX thread - parses MAVLink messages from ring buffer
 */
static void mavwrap_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct mavwrap_data *data = dev->data;
	uint8_t rx_byte = 0;

	LOG_INF("[%s] RX thread started", dev->name);

	while (1) {
		uint32_t n = ring_buf_get(&data->rx_rb, &rx_byte, sizeof(rx_byte));

		if (n == 0) {
			k_sem_take(&data->rx_sem, K_FOREVER);
			continue;
		}

		if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, 
		                       &data->rx_msg, &data->rx_status)) {
			atomic_inc(&data->stats.rx_packets);

			/* Call user callback without holding mutex */
			if (data->user_callback) {
				data->user_callback(dev, &data->rx_msg, data->user_data);
			}
		}

		if (data->rx_status.parse_error > 0) {
			atomic_inc(&data->stats.rx_errors);
		}
	}
}


/**
 * Transport RX handler - called by transport layer
 */
static void mavwrap_transport_rx_handler(const struct device *dev,
                                         const uint8_t *buf,
                                         size_t len,
                                         void *user_data)
{
	struct mavwrap_data *data = dev->data;

	if (!buf || len == 0) {
		return;
	}

	uint32_t written = ring_buf_put(&data->rx_rb, buf, len);

	if (written < len) {
		uint32_t dropped = len - written;
		LOG_WRN("[%s] RX ring overflow, dropped %u bytes", dev->name, dropped);
		atomic_add(&data->stats.rx_buff_overflow, dropped);
	}

	if (written > 0) {
		k_sem_give(&data->rx_sem);
	}
}


/**
 * Initialize MAVLink wrapper instance
 */
static int mavwrap_init(const struct device *dev)
{
	struct mavwrap_data *data = dev->data;
	const struct mavwrap_config *config = dev->config;
	int ret;

	data->dev = dev;
	data->rx_stack = config->thread_stack;

	/* Transport data is already zero-initialized in static allocation */

	atomic_set(&data->stats.rx_packets, 0);
	atomic_set(&data->stats.tx_packets, 0);
	atomic_set(&data->stats.rx_errors, 0);
	atomic_set(&data->stats.tx_errors, 0);
	atomic_set(&data->stats.rx_buff_overflow, 0);

	memset(&data->rx_msg, 0, sizeof(data->rx_msg));
	memset(&data->rx_status, 0, sizeof(data->rx_status));

	if (config->transport_type == MAVWRAP_TRANSPORT_UNKNOWN) {
		LOG_ERR("[%s] Transport type not supported", dev->name);
		return -ENODEV;
	}

	if (config->ops->init) {
		ret = config->ops->init(dev);
		if (ret < 0) {
			LOG_ERR("[%s] Failed to init transport: %d", dev->name, ret);
			return ret;
		}
	}

	ring_buf_init(&data->rx_rb,
	              sizeof(data->rx_rb_buf),
	              data->rx_rb_buf);

	k_sem_init(&data->rx_sem, 0, K_SEM_MAX_LIMIT);

	data->rx_tid = k_thread_create(
		&data->rx_thread,
		config->thread_stack,
		config->stack_size,
		mavwrap_rx_thread,
		(void *)dev, NULL, NULL,
		CONFIG_MAVWRAP_RX_THREAD_PRIORITY,
		0, K_NO_WAIT);

#ifdef CONFIG_THREAD_NAME
	char thread_name[CONFIG_THREAD_MAX_NAME_LEN];
	snprintk(thread_name, sizeof(thread_name), "mav_rx_%s", dev->name);
	k_thread_name_set(data->rx_tid, thread_name);
#endif

	ret = config->ops->set_rx_callback(dev,
	                                   mavwrap_transport_rx_handler,
	                                   data);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to set transport RX callback: %d", dev->name, ret);
		k_thread_abort(data->rx_tid);
		return ret;
	}

	LOG_INF("[%s] MAVLink interface initialized (transport: %d)",
	        dev->name, config->transport_type);

	return 0;
}


int mavwrap_set_rx_callback(const struct device *dev,
                            mavwrap_rx_callback_t callback,
                            void *user_data)
{
	struct mavwrap_data *data;

	if (!dev) {
		return -EINVAL;
	}

	data = dev->data;

	data->user_callback = callback;
	data->user_data = user_data;

	return 0;
}


int mavwrap_send_message(const struct device *dev,
                         const mavlink_message_t *msg)
{
	struct mavwrap_data *data;
	const struct mavwrap_config *config;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	int ret;

	if (!dev || !msg) {
		return -EINVAL;
	}

	data = dev->data;
	config = dev->config;

	len = mavlink_msg_to_send_buffer(buf, msg);

	if (!config->ops || !config->ops->send) {
		return -ENOTSUP;
	}

	ret = config->ops->send(dev, buf, len);

	if (ret == 0) {
		atomic_inc(&data->stats.tx_packets);
	} else {
		atomic_inc(&data->stats.tx_errors);
	}

	return ret;
}


int mavwrap_set_property(const struct device *dev,
                         const struct mavwrap_property_value *prop)
{
	const struct mavwrap_config *config;

	if (!dev || !prop) {
		return -EINVAL;
	}

	config = dev->config;

	if (!config->ops || !config->ops->set_property) {
		return -ENOTSUP;
	}

	return config->ops->set_property(dev, prop);
}


int mavwrap_get_property(const struct device *dev,
                         struct mavwrap_property_value *prop)
{
	const struct mavwrap_config *config;

	if (!dev || !prop) {
		return -EINVAL;
	}

	config = dev->config;

	if (!config->ops || !config->ops->get_property) {
		return -ENOTSUP;
	}

	return config->ops->get_property(dev, prop);
}


int mavwrap_get_stats(const struct device *dev,
                      struct mavwrap_stats *stats)
{
	struct mavwrap_data *data;

	if (!dev || !stats) {
		return -EINVAL;
	}

	data = dev->data;

	/* Convert atomic_t to uint32_t */
	stats->rx_packets = (uint32_t)atomic_get(&data->stats.rx_packets);
	stats->tx_packets = (uint32_t)atomic_get(&data->stats.tx_packets);
	stats->rx_errors = (uint32_t)atomic_get(&data->stats.rx_errors);
	stats->tx_errors = (uint32_t)atomic_get(&data->stats.tx_errors);
	stats->rx_buff_overflow = (uint32_t)atomic_get(&data->stats.rx_buff_overflow);

	return 0;
}


int mavwrap_reset_stats(const struct device *dev)
{
	struct mavwrap_data *data;

	if (!dev) {
		return -EINVAL;
	}

	data = dev->data;

	atomic_set(&data->stats.rx_packets, 0);
	atomic_set(&data->stats.tx_packets, 0);
	atomic_set(&data->stats.rx_errors, 0);
	atomic_set(&data->stats.tx_errors, 0);
	atomic_set(&data->stats.rx_buff_overflow, 0);

	return 0;
}


/* Devicetree-based device instantiation */
DT_INST_FOREACH_STATUS_OKAY(MAVWRAP_DEVICE_INIT)
