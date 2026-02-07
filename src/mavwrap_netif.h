/**
 * @file mavwrap_netif.h
 * @brief Network transport definitions
 *
 * This file contains network-specific structures and definitions.
 * Only included when CONFIG_MAVWRAP_TRANSPORT_NETIF is enabled.
 */

#ifndef MAVWRAP_NETIF_H
#define MAVWRAP_NETIF_H

#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Forward declaration of transport RX callback
 */
typedef void (*mavwrap_transport_rx_cb_t)(const struct device *dev,
                                          const uint8_t *buf,
                                          size_t len,
                                          void *user_data);

/**
 * Network transport type
 */
enum mavwrap_net_type {
	MAVWRAP_NET_UDP = 0,
};

/**
 * Network state
 */
enum mavwrap_net_state {
	MAVWRAP_NET_STATE_STOPPED = 0,
	MAVWRAP_NET_STATE_RUNNING,
	MAVWRAP_NET_STATE_RECONFIGURING,
};

/**
 * Runtime property types that can be changed
 */
enum mavwrap_net_property {
	MAVWRAP_NET_PROP_REMOTE_IP,
	MAVWRAP_NET_PROP_REMOTE_PORT,
	MAVWRAP_NET_PROP_LOCAL_PORT,
};

/**
 * Property value container
 */
struct mavwrap_net_property_value {
	enum mavwrap_net_property prop;
	union {
		uint16_t port;
		const char *ip_str;
	} value;
};

/**
 * Network transport operations
 * Each network type (UDP, etc.) implements this interface
 */
struct mavwrap_net_ops {
	int (*init)(const struct device *dev);
	int (*connect)(const struct device *dev);
	int (*send)(const struct device *dev, const uint8_t *buf, size_t len);
	int (*reconnect)(const struct device *dev);
	int (*set_property)(const struct device *dev,
	                    const struct mavwrap_net_property_value *prop,
	                    bool apply_immediately);
	int (*get_property)(const struct device *dev,
	                    struct mavwrap_net_property_value *prop);
};

/**
 * Network transport runtime data
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
 * Network transport configuration
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
 * Get network operations for specific type
 */
const struct mavwrap_net_ops* mavwrap_net_get_ops(enum mavwrap_net_type type);

/**
 * Wait for network interface to be ready
 */
int mavwrap_netif_wait_for_network(const struct device *dev, int32_t timeout_ms);

/**
 * Initialize DHCP or static IP configuration
 */
int mavwrap_netif_setup_ip(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* MAVWRAP_NETIF_H */
