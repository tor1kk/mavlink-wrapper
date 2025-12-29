#ifndef MAVWRAP_TRANSPORT_H
#define MAVWRAP_TRANSPORT_H

#include <zephyr/device.h>
#include <stddef.h>
#include <stdint.h>


typedef void (*mavwrap_transport_rx_cb_t)(const struct device *dev,
                                          const uint8_t *buf,
                                          size_t len,
                                          void *user_data);


struct mavwrap_transport_ops {
	int (*init)(const struct device *dev);
	int (*send)(const struct device *dev, const uint8_t *buf, size_t len);
	int (*set_rx_callback)(const struct device *dev,
	                       mavwrap_transport_rx_cb_t callback,
	                       void *user_data);
};


enum mavwrap_transport_type {
	MAVWRAP_TRANSPORT_UART,
	MAVWRAP_TRANSPORT_UNKNOWN,
};


struct mavwrap_config {
	const struct device *transport_dev;
	const struct mavwrap_transport_ops *ops;
	enum mavwrap_transport_type transport_type;
	k_thread_stack_t *thread_stack;
	size_t stack_size;
};

#endif /* MAVWRAP_TRANSPORT_H */
