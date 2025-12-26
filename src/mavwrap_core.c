#define DT_DRV_COMPAT mavlink_mavlink_wrapper

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <mavwrap.h>
#include "mavwrap_transport.h"
#include <common/mavlink.h>


LOG_MODULE_REGISTER(mavwrap, CONFIG_MAVWRAP_LOG_LEVEL);


struct mavwrap_data {
	const struct device *dev;

	mavlink_message_t rx_msg;
	mavlink_status_t rx_status;

	mavwrap_rx_callback_t user_callback;
	void *user_data;

	struct mavwrap_stats stats;
	struct k_mutex stats_mutex;

	struct ring_buf rx_rb;
	uint8_t rx_rb_buf[CONFIG_MAVWRAP_RX_RING_SIZE];
	struct k_sem rx_sem;

	struct k_thread rx_thread;
	k_tid_t rx_tid;
	
	k_thread_stack_t *rx_stack;
};


static void mavwrap_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct mavwrap_data *data = dev->data;
	uint8_t rx_byte;

	LOG_INF("[%s] RX thread started", dev->name);

	while (1) {
		uint32_t n = ring_buf_get(&data->rx_rb, &rx_byte, sizeof(rx_byte));

		if (n == 0) {
			k_sem_take(&data->rx_sem, K_FOREVER);
			continue;
		}

		if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, 
		                       &data->rx_msg, &data->rx_status)) {
			k_mutex_lock(&data->stats_mutex, K_FOREVER);
			data->stats.rx_packets++;
			k_mutex_unlock(&data->stats_mutex);

			if (data->user_callback) {
				data->user_callback(data->dev, &data->rx_msg, data->user_data);
			}
		}

		if (data->rx_status.parse_error > 0) {
			k_mutex_lock(&data->stats_mutex, K_FOREVER);
			data->stats.rx_errors++;
			k_mutex_unlock(&data->stats_mutex);
		}
	}
}


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

		k_mutex_lock(&data->stats_mutex, K_FOREVER);
		data->stats.rx_buff_overflow += dropped;
		k_mutex_unlock(&data->stats_mutex);
	}

	if (written > 0) {
		k_sem_give(&data->rx_sem);
	}
}


static int mavwrap_init(const struct device *dev)
{
	struct mavwrap_data *data = dev->data;
	const struct mavwrap_config *config = dev->config;
	int ret;

	data->dev = dev;
	data->rx_stack = config->thread_stack;

	k_mutex_init(&data->stats_mutex);

	memset(&data->rx_msg, 0, sizeof(data->rx_msg));
	memset(&data->rx_status, 0, sizeof(data->rx_status));

	memset(&data->stats, 0, sizeof(data->stats));

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

	k_mutex_lock(&data->stats_mutex, K_FOREVER);
	if (ret == 0) {
		data->stats.tx_packets++;
	} else {
		data->stats.tx_errors++;
	}
	k_mutex_unlock(&data->stats_mutex);

	return ret;
}


int mavwrap_get_stats(const struct device *dev,
                      struct mavwrap_stats *stats)
{
	struct mavwrap_data *data;

	if (!dev || !stats) {
		return -EINVAL;
	}

	data = dev->data;

	k_mutex_lock(&data->stats_mutex, K_FOREVER);
	memcpy(stats, &data->stats, sizeof(*stats));
	k_mutex_unlock(&data->stats_mutex);

	return 0;
}


int mavwrap_reset_stats(const struct device *dev)
{
	struct mavwrap_data *data;

	if (!dev) {
		return -EINVAL;
	}

	data = dev->data;

	k_mutex_lock(&data->stats_mutex, K_FOREVER);
	memset(&data->stats, 0, sizeof(data->stats));
	k_mutex_unlock(&data->stats_mutex);

	return 0;
}


#if CONFIG_MAVWRAP_TRANSPORT_UART
extern const struct mavwrap_transport_ops mavwrap_uart_ops;
#endif


#define MAVWRAP_TRANSPORT_DEV(inst) 												\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, transport), 							\
		(DEVICE_DT_GET(DT_INST_PHANDLE(inst, transport))), 							\
		(NULL))

#define MAVWRAP_TRANSPORT_TYPE(inst) 												\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, transport), 							\
		(COND_CODE_1(DT_NODE_HAS_COMPAT(DT_INST_PHANDLE(inst, transport), 			\
			zephyr_cdc_acm_uart), 													\
			(MAVWRAP_TRANSPORT_USB_CDC), 											\
			(COND_CODE_1(DT_NODE_HAS_COMPAT(DT_INST_PHANDLE(inst, transport), 		\
				zephyr_uart), 														\
				(MAVWRAP_TRANSPORT_UART), 											\
				(MAVWRAP_TRANSPORT_UART))))), 										\
		(MAVWRAP_TRANSPORT_UART))

#define MAVWRAP_TRANSPORT_OPS(inst) 												\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, transport), 							\
		(COND_CODE_1(DT_NODE_HAS_COMPAT(DT_INST_PHANDLE(inst, transport), 			\
			zephyr_uart), 															\
			(&mavwrap_uart_ops), 													\
			(COND_CODE_1(DT_NODE_HAS_COMPAT(DT_INST_PHANDLE(inst, transport), 		\
				zephyr_cdc_acm_uart), 												\
				(&mavwrap_uart_ops), 												\
				(&mavwrap_uart_ops))))), 											\
		(&mavwrap_uart_ops))

#define MAVWRAP_CONFIG_INIT(inst) 													\
	{ 																				\
		.transport_dev = MAVWRAP_TRANSPORT_DEV(inst),			 					\
		.ops = MAVWRAP_TRANSPORT_OPS(inst), 										\
		.transport_type = MAVWRAP_TRANSPORT_TYPE(inst), 							\
		.thread_stack = mavwrap_rx_stack_##inst, 									\
		.stack_size = K_THREAD_STACK_SIZEOF(mavwrap_rx_stack_##inst), 				\
	}

#define MAVWRAP_DEVICE_INIT(inst) 													\
	K_THREAD_STACK_DEFINE(mavwrap_rx_stack_##inst, CONFIG_MAVWRAP_RX_STACK_SIZE);	\
																					\
	static struct mavwrap_data mavwrap_data_##inst; 								\
																					\
	static const struct mavwrap_config mavwrap_config_##inst = 						\
		MAVWRAP_CONFIG_INIT(inst); 													\
																					\
	DEVICE_DT_INST_DEFINE(inst, 													\
	                      mavwrap_init,		 										\
	                      NULL, 													\
	                      &mavwrap_data_##inst, 									\
	                      &mavwrap_config_##inst, 									\
	                      POST_KERNEL, 												\
	                      CONFIG_MAVWRAP_INIT_PRIORITY, 							\
	                      NULL);

DT_INST_FOREACH_STATUS_OKAY(MAVWRAP_DEVICE_INIT)
