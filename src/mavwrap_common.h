#ifndef MAVWRAP_COMMON_H
#define MAVWRAP_COMMON_H

#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>

#include <stddef.h>
#include <stdint.h>

#include "mavwrap.h"
#include "mavwrap_netif_common.h"


/**
 * Transport RX callback
 */
typedef void (*mavwrap_transport_rx_cb_t)(const struct device *dev,
                                          const uint8_t *buf,
                                          size_t len,
                                          void *user_data);


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


/**
 * UART transport data
 */
struct mavwrap_uart_data {
	const struct device *uart_dev;

	mavwrap_transport_rx_cb_t rx_callback;
	void *user_data;

	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	size_t tx_buf_len;
	size_t tx_buf_pos;
	struct k_sem tx_sem;
	bool tx_in_progress;

#ifdef CONFIG_UART_ASYNC_API
	uint8_t rx_buf[2][CONFIG_MAVWRAP_UART_DMA_RX_BUF_SIZE];
	uint8_t rx_buf_idx;
	bool use_dma;
#endif
};


/**
 * Network transport data
 */
struct mavwrap_netif_data {
	struct net_if *iface;
	struct net_context *ctx;
	
	mavwrap_transport_rx_cb_t rx_callback;
	void *user_data;

	/* Runtime configuration (can be changed) */
	struct {
		struct sockaddr_in local_addr;
		struct sockaddr_in remote_addr;
		char remote_ip[16];
		uint16_t local_port;
		uint16_t remote_port;
	} runtime_config;

	/* Original DT configuration (read-only) */
	struct {
		struct sockaddr_in local_addr;
		struct sockaddr_in remote_addr;
	} dt_config;

	/* Network type and operations */
	enum mavwrap_net_type net_type;
	const struct mavwrap_net_ops *ops;

	/* State */
	bool dhcp_enabled;
	bool connected;
	atomic_t state;

	/* Synchronization */
	struct k_mutex config_mutex;
	struct net_mgmt_event_callback mgmt_cb;
	struct k_sem net_ready_sem;
};


/**
 * MAVLink wrapper data
 */
struct mavwrap_data {
	const struct device *dev;

	mavlink_message_t rx_msg __aligned(8);
	mavlink_status_t rx_status __aligned(8);

	mavwrap_rx_callback_t user_callback;
	void *user_data;

	struct mavwrap_stats stats;

	struct ring_buf rx_rb;
	uint8_t rx_rb_buf[CONFIG_MAVWRAP_RX_RING_SIZE];
	struct k_sem rx_sem;

	struct k_thread rx_thread;
	k_tid_t rx_tid;
	
	k_thread_stack_t *rx_stack;

	union mavwrap_transport_data { 
		struct mavwrap_uart_data uart;
		struct mavwrap_netif_data netif;
	} transport_data;
};


/**
 * UART configuration
 */
struct mavwrap_uart_config {
	/* Empty for now - UART settings come from DT uart node */
};


/**
 * Network configuration
 */
struct mavwrap_netif_config {
	const char *local_ip;
	const char *remote_ip;
	uint16_t local_port;
	uint16_t remote_port;
	enum mavwrap_net_type net_type;
	bool dhcp_enabled;
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

	union {
		struct mavwrap_uart_config uart;
		struct mavwrap_netif_config netif;
	} transport_config;
};

#endif /* MAVWRAP_COMMON_H */
