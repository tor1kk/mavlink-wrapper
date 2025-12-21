#define DT_DRV_COMPAT mavlink_mavlink_wrapper

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <mavwrap.h>

#include "mavwrap_transport.h"
#include "common/mavlink.h"


LOG_MODULE_REGISTER(mavwrap, CONFIG_MAVWRAP_LOG_LEVEL);


struct mavwrap_data {
    const struct device *dev;
 
    mavlink_message_t rx_msg;
	mavlink_status_t rx_status;

    mavwrap_rx_callback_t user_callback;
	void *user_data;

    struct mavwrap_stats stats;
    struct k_mutex stats_mutex;
};


enum mavwrap_transport_type {
	MAVWRAP_TRANSPORT_UART,
	// TODO: Ethernet, UDP
};


struct mavwrap_config {
    union {
        const struct device *uart;
		// TODO: Ethernet device, etc
     } transport;
    
    const struct mavwrap_transport_ops *ops;  
    enum mavwrap_transport_type transport_type;
};


/**
 * @brief Transport RX handler - processes buffer of received bytes
 * 
 * Called from workqueue context by transport layer.
 * Parses MAVLink messages byte-by-byte from buffer.
 */
static void mavwrap_transport_rx_handler(const struct device *dev,
                                        const uint8_t *buf,
                                        size_t len,
                                        void *user_data)
{
    struct mavwrap_data *data = user_data;
 
	// Parse each byte in the buffer
	for (size_t i = 0; i < len; i++) {
		if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &data->rx_msg, &data->rx_status)) {
			k_mutex_lock(&data->stats_mutex, K_FOREVER);
			data->stats.rx_packets++;
			k_mutex_unlock(&data->stats_mutex);
			
			// Call user callback for complete message
			if (data->user_callback) {
				data->user_callback(data->dev, &data->rx_msg, data->user_data);
			}
		}
		
		// Check for parse errors
		if (data->rx_status.parse_error > 0) {
			k_mutex_lock(&data->stats_mutex, K_FOREVER);
			data->stats.rx_errors++;
			k_mutex_unlock(&data->stats_mutex);
		}
	}
}


static int mavwrap_init(const struct device *dev)
{
    struct mavwrap_data *data = dev->data;
	const struct mavwrap_config *config = dev->config;

    data->dev = dev;

    k_mutex_init(&data->stats_mutex);

	if (config->ops->init) {
        int ret = config->ops->init(dev);
        if (ret < 0) {
            LOG_ERR("Failed to init transport: %d", ret);
            return ret;
        }
    }

	int ret = config->ops->set_rx_callback(dev,
										mavwrap_transport_rx_handler,
										data);
	if (ret < 0) {
		LOG_ERR("Failed to set transport RX callback: %d", ret);
		return ret;
	}
	
	LOG_INF("MAVLink interface initialized (transport: %d)", config->transport_type);
	return 0;
}


int mavwrap_set_rx_callback(const struct device *dev,
                           mavwrap_rx_callback_t callback,
                           void *user_data)
{
	struct mavwrap_data *data = dev->data;
	
	data->user_callback = callback;
	data->user_data = user_data;
	
	return 0;
}


int mavwrap_send_message(const struct device *dev,
                        const mavlink_message_t *msg)
{
	struct mavwrap_data *data = dev->data;
	const struct mavwrap_config *config = dev->config;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	if (!msg) {
		return -EINVAL;
	}
	
	len = mavlink_msg_to_send_buffer(buf, msg);
	
	if (!config->ops || !config->ops->send) {
		return -ENOTSUP;
	}
	
	int ret = config->ops->send(dev, buf, len);
	
	if (ret == 0) {
		k_mutex_lock(&data->stats_mutex, K_FOREVER);
		data->stats.tx_packets++;
		k_mutex_unlock(&data->stats_mutex);
	} else {
		k_mutex_lock(&data->stats_mutex, K_FOREVER);
		data->stats.tx_errors++;
		k_mutex_unlock(&data->stats_mutex);
	}
	
	return ret;
}


int mavwrap_get_stats(const struct device *dev,
                     struct mavwrap_stats *stats)
{
	struct mavwrap_data *data = dev->data;
	
	if (!stats) {
		return -EINVAL;
	}
	
	k_mutex_lock(&data->stats_mutex, K_FOREVER);
	memcpy(stats, &data->stats, sizeof(*stats));
	k_mutex_unlock(&data->stats_mutex);
	
	return 0;
}


int mavwrap_reset_stats(const struct device *dev)
{
	struct mavwrap_data *data = dev->data;
	
	k_mutex_lock(&data->stats_mutex, K_FOREVER);
	memset(&data->stats, 0, sizeof(data->stats));
	k_mutex_unlock(&data->stats_mutex);
	
	return 0;
}


extern const struct mavwrap_transport_ops mavwrap_uart_ops;

/* UART config */
#define MAVWRAP_CONFIG_UART(inst)                                   	\
    {                                                               	\
        .transport.uart = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))), 	\
        .ops = &mavwrap_uart_ops,                              			\
        .transport_type = MAVWRAP_TRANSPORT_UART,                  		\
    }


#define MAVWRAP_DEVICE_INIT(inst)                                   	\
    static struct mavwrap_data mavwrap_data_##inst;                	 	\
                                                                    	\
    static const struct mavwrap_config mavwrap_config_##inst =      	\
        COND_CODE_1(DT_NODE_HAS_COMPAT(                            		\
                    DT_PARENT(DT_DRV_INST(inst)), zephyr_uart),    		\
            (MAVWRAP_CONFIG_UART(inst)),                          		\
            ({ /* Error or other transports */ })                  		\
        );                                                          	\
                                                                    	\
    DEVICE_DT_INST_DEFINE(inst,                                     	\
                         mavwrap_init,                              	\
                         NULL,                                      	\
                         &mavwrap_data_##inst,                      	\
                         &mavwrap_config_##inst,                    	\
                         POST_KERNEL,                               	\
                         CONFIG_MAVWRAP_INIT_PRIORITY,              	\
                         NULL);

						 
DT_INST_FOREACH_STATUS_OKAY(MAVWRAP_DEVICE_INIT)
