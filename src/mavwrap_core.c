#define DT_DRV_COMPAT mavlink_wrapper

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <mavwrap.h>
#include "mavwrap_common.h"
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

	memset(&data->transport_data, 0, sizeof(data->transport_data));

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

	if (!data->rx_tid) {
		LOG_ERR("[%s] Failed to create RX thread", dev->name);
		return -ENOMEM;
	}

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


/* Transport node */
#define MAVWRAP_TRANSPORT_NODE(inst) \
	DT_PHANDLE(DT_DRV_INST(inst), transport)

/* Network interface check */
#define MAVWRAP_HAS_NETIF(inst) \
	DT_PROP(DT_DRV_INST(inst), net_interface)

/* UART check */
#define MAVWRAP_HAS_UART(inst) \
	DT_PROP(DT_DRV_INST(inst), serial_interface)

/* Transport device */
#define MAVWRAP_TRANSPORT_DEV(inst) \
	DEVICE_DT_GET(MAVWRAP_TRANSPORT_NODE(inst))

/* Transport type */
#define MAVWRAP_TRANSPORT_TYPE(inst) \
	COND_CODE_1( \
		MAVWRAP_HAS_UART(inst), \
		(MAVWRAP_TRANSPORT_UART), \
		( \
			COND_CODE_1( \
				MAVWRAP_HAS_NETIF(inst), \
				(MAVWRAP_TRANSPORT_NETIF), \
				(MAVWRAP_TRANSPORT_UNKNOWN) \
			) \
		) \
	)

/* Transport ops */
#if CONFIG_MAVWRAP_TRANSPORT_UART
extern const struct mavwrap_transport_ops mavwrap_uart_ops;
#endif

#if CONFIG_MAVWRAP_TRANSPORT_NETIF
extern const struct mavwrap_transport_ops mavwrap_netif_ops;
#endif

#define MAVWRAP_TRANSPORT_OPS(inst) \
	COND_CODE_1( \
		MAVWRAP_HAS_UART(inst), \
		(&mavwrap_uart_ops), \
		( \
			COND_CODE_1( \
				MAVWRAP_HAS_NETIF(inst), \
				(&mavwrap_netif_ops), \
				(NULL) \
			) \
		) \
	)

/* Network configuration macros */
#define MAVWRAP_NETIF_DHCP_ENABLE(inst) \
	DT_PROP(DT_DRV_INST(inst), use_dhcp)

#define MAVWRAP_NETIF_LOCAL_IP(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), local_ip, "0.0.0.0")

#define MAVWRAP_NETIF_REMOTE_IP(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), remote_ip, "0.0.0.0")

#define MAVWRAP_NETIF_LOCAL_PORT(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), local_port, 14550)

#define MAVWRAP_NETIF_REMOTE_PORT(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), remote_port, 14551)

#define MAVWRAP_NETIF_TYPE(inst) \
	((enum mavwrap_net_type)DT_ENUM_IDX(DT_DRV_INST(inst), net_type))

/* UART config - empty for now */
#define MAVWRAP_UART_CONFIG_INIT(inst) \
	{ \
	}

/* Network config */
#define MAVWRAP_NETIF_CONFIG_INIT(inst) \
	{ \
		.local_ip = MAVWRAP_NETIF_LOCAL_IP(inst), \
		.remote_ip = MAVWRAP_NETIF_REMOTE_IP(inst), \
		.local_port = MAVWRAP_NETIF_LOCAL_PORT(inst), \
		.remote_port = MAVWRAP_NETIF_REMOTE_PORT(inst), \
		.net_type = MAVWRAP_NETIF_TYPE(inst), \
		.dhcp_enabled = MAVWRAP_NETIF_DHCP_ENABLE(inst), \
	}

/* Transport config init */
#define MAVWRAP_TRANSPORT_CONFIG_INIT_UART(inst) \
	.uart = MAVWRAP_UART_CONFIG_INIT(inst)

#define MAVWRAP_TRANSPORT_CONFIG_INIT_NETIF(inst) \
	.netif = MAVWRAP_NETIF_CONFIG_INIT(inst)

#define MAVWRAP_TRANSPORT_CONFIG_INIT(inst) \
	COND_CODE_1( \
		MAVWRAP_HAS_UART(inst), \
		(MAVWRAP_TRANSPORT_CONFIG_INIT_UART(inst)), \
		( \
			COND_CODE_1( \
				MAVWRAP_HAS_NETIF(inst), \
				(MAVWRAP_TRANSPORT_CONFIG_INIT_NETIF(inst)), \
				() \
			) \
		) \
	)

/* Main config */
#define MAVWRAP_CONFIG_INIT(inst) \
	{ \
		.transport_dev = MAVWRAP_TRANSPORT_DEV(inst), \
		.ops = MAVWRAP_TRANSPORT_OPS(inst), \
		.transport_type = MAVWRAP_TRANSPORT_TYPE(inst), \
		.thread_stack = mavwrap_rx_stack_##inst, \
		.stack_size = K_THREAD_STACK_SIZEOF(mavwrap_rx_stack_##inst), \
		.transport_config = { \
			MAVWRAP_TRANSPORT_CONFIG_INIT(inst) \
		} \
	}

/* Validation */
#define MAVWRAP_VALIDATE_DT(inst) \
	BUILD_ASSERT( \
		DT_NODE_HAS_PROP(DT_DRV_INST(inst), transport), \
		"mavlink-wrapper: 'transport' phandle is required"); \
	BUILD_ASSERT( \
		MAVWRAP_HAS_UART(inst) || MAVWRAP_HAS_NETIF(inst), \
		"mavlink-wrapper: unknown transport type");

/* Device init */
#define MAVWRAP_DEVICE_INIT(inst) \
	MAVWRAP_VALIDATE_DT(inst); \
	K_THREAD_STACK_DEFINE( \
		mavwrap_rx_stack_##inst, \
		CONFIG_MAVWRAP_RX_STACK_SIZE); \
	static struct mavwrap_data mavwrap_data_##inst; \
	static const struct mavwrap_config mavwrap_config_##inst = \
		MAVWRAP_CONFIG_INIT(inst); \
	DEVICE_DT_INST_DEFINE( \
		inst, \
		mavwrap_init, \
		NULL, \
		&mavwrap_data_##inst, \
		&mavwrap_config_##inst, \
		POST_KERNEL, \
		CONFIG_MAVWRAP_INIT_PRIORITY, \
		NULL);

DT_INST_FOREACH_STATUS_OKAY(MAVWRAP_DEVICE_INIT)
