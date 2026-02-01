#ifndef MAVWRAP_NETIF_COMMON_H
#define MAVWRAP_NETIF_COMMON_H

#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/kernel.h>


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
	/**
	 * Initialize socket and prepare for connection
	 */
	int (*init)(const struct device *dev);

	/**
	 * Connect/bind socket and start receiving
	 */
	int (*connect)(const struct device *dev);

	/**
	 * Send data
	 */
	int (*send)(const struct device *dev, const uint8_t *buf, size_t len);

	/**
	 * Reconnect with updated configuration
	 */
	int (*reconnect)(const struct device *dev);

	/**
	 * Set runtime property
	 */
	int (*set_property)(const struct device *dev,
	                    const struct mavwrap_net_property_value *prop,
	                    bool apply_immediately);

	/**
	 * Get current property value
	 */
	int (*get_property)(const struct device *dev,
	                    struct mavwrap_net_property_value *prop);
};


/**
 * Get network operations for specific type
 * @param type Network type (UDP, etc.)
 * @return Pointer to ops structure or NULL if type not supported
 */
const struct mavwrap_net_ops* mavwrap_net_get_ops(enum mavwrap_net_type type);


/**
 * Wait for network interface to be ready
 * Handles DHCP if enabled, or waits for link up if static IP
 * @param dev MAVLink wrapper device
 * @param timeout_ms Timeout in milliseconds
 * @return 0 on success, negative errno on error
 */
int mavwrap_netif_wait_for_network(const struct device *dev, int32_t timeout_ms);


/**
 * Initialize DHCP or static IP configuration
 * @param dev MAVLink wrapper device
 * @return 0 on success, negative errno on error
 */
int mavwrap_netif_setup_ip(const struct device *dev);

#endif /* MAVWRAP_NETIF_COMMON_H */
