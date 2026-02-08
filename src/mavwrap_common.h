/**
 * @file mavwrap_common.h
 * @brief MAVLink wrapper common definitions
 *
 * This file contains transport-agnostic common structures and definitions.
 * Transport-specific headers are conditionally included based on enabled transports.
 */

#ifndef MAVWRAP_COMMON_H
#define MAVWRAP_COMMON_H

#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/kernel.h>

#include <stddef.h>
#include <stdint.h>

#include "mavwrap.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Transport RX callback type (common for all transports)
 */
typedef void (*mavwrap_transport_rx_cb_t)(const struct device *dev,
                                          const uint8_t *buf,
                                          size_t len,
                                          void *user_data);

#ifdef __cplusplus
}
#endif

/* Conditionally include transport-specific headers */
#ifdef CONFIG_MAVWRAP_TRANSPORT_UART
#include "mavwrap_uart.h"
#endif

#ifdef CONFIG_MAVWRAP_TRANSPORT_NETIF
#include "mavwrap_netif.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Transport operations interface
 */
struct mavwrap_transport_ops {
	int (*init)(const struct device *dev);
	int (*send)(const struct device *dev, const uint8_t *buf, size_t len);
	int (*set_rx_callback)(const struct device *dev,
	                       mavwrap_transport_rx_cb_t callback,
	                       void *user_data);
	int (*set_property)(const struct device *dev,
	                    const struct mavwrap_property_value *prop);
	int (*get_property)(const struct device *dev,
	                    struct mavwrap_property_value *prop);
};

/**
 * Transport type
 */
enum mavwrap_transport_type {
	MAVWRAP_TRANSPORT_UART,
	MAVWRAP_TRANSPORT_NETIF,
	MAVWRAP_TRANSPORT_UNKNOWN,
};

#ifdef CONFIG_MAVWRAP_TX_THREAD
/**
 * TX queue item — serialized MAVLink packet ready to send
 */
struct mavwrap_tx_item {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
};
#endif

/**
 * Internal statistics using atomic types for thread-safety
 */
struct mavwrap_stats_atomic {
	atomic_t rx_packets;
	atomic_t tx_packets;
	atomic_t rx_errors;
	atomic_t tx_errors;
	atomic_t rx_buff_overflow;
};

/**
 * MAVLink wrapper runtime data
 */
struct mavwrap_data {
	const struct device *dev;

	mavlink_message_t rx_msg __aligned(8);
	mavlink_status_t rx_status __aligned(8);

	mavwrap_rx_callback_t user_callback;
	void *user_data;
	atomic_t started;

	struct mavwrap_stats_atomic stats;

	struct ring_buf rx_rb;
	uint8_t rx_rb_buf[CONFIG_MAVWRAP_RX_RING_SIZE];
	struct k_sem rx_sem;

	struct k_thread rx_thread;
	k_tid_t rx_tid;

	k_thread_stack_t *rx_stack;

#ifdef CONFIG_MAVWRAP_TX_THREAD
	struct k_msgq tx_msgq;
	char __aligned(4) tx_msgq_buf[CONFIG_MAVWRAP_TX_QUEUE_SIZE *
				      sizeof(struct mavwrap_tx_item)];
	struct k_thread tx_thread;
	k_tid_t tx_tid;
#endif

	/* Pointer to transport-specific runtime data */
	void *transport_data;
};

/**
 * MAVLink wrapper configuration
 */
struct mavwrap_config {
	const struct device *transport_dev;
	const struct mavwrap_transport_ops *ops;
	enum mavwrap_transport_type transport_type;

	k_thread_stack_t *thread_stack;
	size_t stack_size;

#ifdef CONFIG_MAVWRAP_TX_THREAD
	k_thread_stack_t *tx_thread_stack;
	size_t tx_stack_size;
#endif

	/* Pointer to transport-specific configuration */
	const void *transport_config;
};

#ifdef __cplusplus
}
#endif

#endif /* MAVWRAP_COMMON_H */
