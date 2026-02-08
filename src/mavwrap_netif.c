#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/logging/log.h>

#include "mavwrap_common.h"

LOG_MODULE_DECLARE(mavwrap);

/* Forward declaration of UDP ops */
extern const struct mavwrap_net_ops mavwrap_udp_ops;

/**
 * Get network operations for specific type
 */
const struct mavwrap_net_ops* mavwrap_net_get_ops(enum mavwrap_net_type type)
{
	switch (type) {
	case MAVWRAP_NET_UDP:
		return &mavwrap_udp_ops;
	default:
		return NULL;
	}
}


/**
 * Network management event handler (DHCP)
 */
static void netif_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                      uint64_t mgmt_event,
                                      struct net_if *iface)
{
	struct mavwrap_netif_data *netif_data =
		CONTAINER_OF(cb, struct mavwrap_netif_data, mgmt_cb);

	if (iface != netif_data->iface) {
		return;
	}

	if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
		LOG_INF("IPv4 address added to interface");
		k_sem_give(&netif_data->net_ready_sem);
	} else if (mgmt_event == NET_EVENT_IPV4_DHCP_BOUND) {
		LOG_INF("IPv4 DHCP lease acquired");
		k_sem_give(&netif_data->net_ready_sem);
	}
}


/**
 * Wait for link up (non-DHCP case)
 */
static int netif_wait_for_link(struct mavwrap_netif_data *netif_data, 
                                k_timeout_t timeout)
{
	int64_t start = k_uptime_get();
	int64_t timeout_ms = k_ticks_to_ms_floor64(timeout.ticks);

	while (!net_if_is_up(netif_data->iface)) {
		if ((k_uptime_get() - start) > timeout_ms) {
			return -ETIMEDOUT;
		}
		k_sleep(K_MSEC(50));
	}
	return 0;
}


/**
 * Wait for network to be ready
 */
int mavwrap_netif_wait_for_network(const struct device *dev, int32_t timeout_ms)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;
	int ret;

	if (!netif_data->dhcp_enabled) {
		ret = netif_wait_for_link(netif_data, K_MSEC(timeout_ms));
		if (ret != 0) {
			LOG_ERR("[%s] Network interface is down", dev->name);
			return -ENETDOWN;
		}
		return 0;
	}

	LOG_INF("[%s] Waiting for network (DHCP)...", dev->name);
	ret = k_sem_take(&netif_data->net_ready_sem, K_MSEC(timeout_ms));
	
	if (ret != 0) {
		LOG_ERR("[%s] Network timeout", dev->name);
		return -ETIMEDOUT;
	}

	LOG_INF("[%s] Network ready", dev->name);
	return 0;
}


/**
 * Setup IP configuration (DHCP or static)
 */
int mavwrap_netif_setup_ip(const struct device *dev)
{
	const struct mavwrap_config *config = dev->config;
	const struct mavwrap_netif_config *netif_config = config->transport_config;
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;

	k_sem_init(&netif_data->net_ready_sem, 0, 1);

	if (netif_data->dhcp_enabled) {
		/* Setup DHCP event callbacks */
		net_mgmt_init_event_callback(&netif_data->mgmt_cb,
		                             netif_mgmt_event_handler,
		                             NET_EVENT_IPV4_ADDR_ADD | 
		                             NET_EVENT_IPV4_DHCP_BOUND);
		net_mgmt_add_event_callback(&netif_data->mgmt_cb);

		/* Start DHCP client */
		net_dhcpv4_start(netif_data->iface);
		LOG_INF("[%s] DHCP client started", dev->name);

	} else {
		/* Static IP configuration */
		LOG_INF("[%s] DHCP disabled, using static IP", dev->name);

		if (!net_if_is_up(netif_data->iface)) {
			net_if_up(netif_data->iface);
		}

		struct in_addr addr;
		if (zsock_inet_pton(AF_INET, netif_config->local_ip, &addr) != 1) {
			LOG_ERR("[%s] Invalid static IP: %s", dev->name, netif_config->local_ip);
			return -EINVAL;
		}

		struct net_if_addr *if_addr = net_if_ipv4_addr_add(netif_data->iface,
		                                                    &addr,
		                                                    NET_ADDR_MANUAL, 0);
		if (!if_addr) {
			LOG_ERR("[%s] Failed to add static IP", dev->name);
			return -EINVAL;
		}
	}

	return 0;
}


static int mavwrap_netif_init(const struct device *dev)
{
	const struct mavwrap_config *config = dev->config;
	const struct mavwrap_netif_config *netif_config = config->transport_config;
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;
	const struct device *netif_dev = config->transport_dev;
	int ret;

	/* Validate network device */
	if (!device_is_ready(netif_dev)) {
		LOG_ERR("[%s] Network interface device %s not ready",
		        dev->name, netif_dev->name);
		return -ENODEV;
	}

	netif_data->iface = net_if_lookup_by_dev(netif_dev);
	if (!netif_data->iface) {
		LOG_ERR("[%s] Failed to get network interface", dev->name);
		return -ENODEV;
	}


	/* Initialize state */
	netif_data->ctx = NULL;
	netif_data->dhcp_enabled = netif_config->dhcp_enabled;
	netif_data->net_type = netif_config->net_type;
	netif_data->connected = false;
	netif_data->rx_callback = NULL;
	netif_data->user_data = NULL;
	atomic_set(&netif_data->state, MAVWRAP_NET_STATE_STOPPED);

	k_mutex_init(&netif_data->config_mutex);

	/* Get operations for this network type */
	netif_data->ops = mavwrap_net_get_ops(netif_data->net_type);
	if (!netif_data->ops) {
		LOG_ERR("[%s] Unsupported network type: %d", dev->name, netif_data->net_type);
		return -ENOTSUP;
	}


	/* Initialize runtime config from DT config */
	netif_data->runtime_config.local_port = netif_config->local_port;
	netif_data->runtime_config.remote_port = netif_config->remote_port;
	strncpy(netif_data->runtime_config.remote_ip,
	        netif_config->remote_ip,
	        sizeof(netif_data->runtime_config.remote_ip) - 1);
	netif_data->runtime_config.remote_ip[sizeof(netif_data->runtime_config.remote_ip) - 1] = '\0';

	/* Parse and store DT addresses */
	netif_data->dt_config.local_addr.sin_family = AF_INET;
	netif_data->dt_config.local_addr.sin_port = htons(netif_config->local_port);
	
	if (zsock_inet_pton(AF_INET, netif_config->local_ip, 
	                    &netif_data->dt_config.local_addr.sin_addr) != 1) {
		LOG_ERR("[%s] Invalid local IP: %s", dev->name, netif_config->local_ip);
		return -EINVAL;
	}

	netif_data->dt_config.remote_addr.sin_family = AF_INET;
	netif_data->dt_config.remote_addr.sin_port = htons(netif_config->remote_port);
	
	if (zsock_inet_pton(AF_INET, netif_config->remote_ip, 
	                    &netif_data->dt_config.remote_addr.sin_addr) != 1) {
		LOG_ERR("[%s] Invalid remote IP: %s", dev->name, netif_config->remote_ip);
		return -EINVAL;
	}


	/* Copy DT config to runtime config */
	memcpy(&netif_data->runtime_config.local_addr,
	       &netif_data->dt_config.local_addr,
	       sizeof(netif_data->runtime_config.local_addr));
	memcpy(&netif_data->runtime_config.remote_addr,
	       &netif_data->dt_config.remote_addr,
	       sizeof(netif_data->runtime_config.remote_addr));

	/* Setup IP (DHCP or static) */
	ret = mavwrap_netif_setup_ip(dev);
	if (ret < 0) {
		return ret;
	}


	/* Call network-type-specific init */
	if (netif_data->ops->init) {
		ret = netif_data->ops->init(dev);
		if (ret < 0) {
			LOG_ERR("[%s] Failed to init network transport: %d", dev->name, ret);
			return ret;
		}
	}

	LOG_INF("[%s] Network transport initialized (type: UDP, %s:%u -> %s:%u, DHCP: %s)",
	        dev->name,
	        netif_config->local_ip, netif_config->local_port,
	        netif_config->remote_ip, netif_config->remote_port,
	        netif_data->dhcp_enabled ? "yes" : "no");

	return 0;
}


static int mavwrap_netif_send(const struct device *dev,
                               const uint8_t *buf,
                               size_t len)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;

	if (!netif_data->ops || !netif_data->ops->send) {
		return -ENOTSUP;
	}

	return netif_data->ops->send(dev, buf, len);
}


static int mavwrap_netif_set_rx_callback(const struct device *dev,
                                         mavwrap_transport_rx_cb_t callback,
                                         void *user_data)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;
	int ret;

	netif_data->rx_callback = callback;
	netif_data->user_data = user_data;

	/* Wait for network to be ready */
	ret = mavwrap_netif_wait_for_network(dev, CONFIG_MAVWRAP_NETIF_CONNECT_TIMEOUT_MS);
	if (ret < 0) {
		return ret;
	}


	/* Connect using network-type-specific ops */
	if (!netif_data->ops || !netif_data->ops->connect) {
		return -ENOTSUP;
	}

	ret = netif_data->ops->connect(dev);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to connect: %d", dev->name, ret);
		return ret;
	}

	atomic_set(&netif_data->state, MAVWRAP_NET_STATE_RUNNING);
	return 0;
}


static int mavwrap_netif_set_property(const struct device *dev,
                                      const struct mavwrap_property_value *prop)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;
	struct mavwrap_net_property_value net_prop;
	int ret;

	if (!netif_data->ops || !netif_data->ops->set_property) {
		return -ENOTSUP;
	}


	/* Convert generic property to network property */
	switch (prop->type) {
	case MAVWRAP_PROPERTY_NET_REMOTE_IP:
		net_prop.prop = MAVWRAP_NET_PROP_REMOTE_IP;
		net_prop.value.ip_str = prop->value.str;
		break;
	case MAVWRAP_PROPERTY_NET_REMOTE_PORT:
		net_prop.prop = MAVWRAP_NET_PROP_REMOTE_PORT;
		net_prop.value.port = prop->value.u16;
		break;
	case MAVWRAP_PROPERTY_NET_LOCAL_PORT:
		net_prop.prop = MAVWRAP_NET_PROP_LOCAL_PORT;
		net_prop.value.port = prop->value.u16;
		break;
	default:
		return -EINVAL;
	}

	ret = netif_data->ops->set_property(dev, &net_prop, prop->apply_immediately);
	
	return ret;
}


static int mavwrap_netif_get_property(const struct device *dev,
                                      struct mavwrap_property_value *prop)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;
	struct mavwrap_net_property_value net_prop;
	int ret;

	if (!netif_data->ops || !netif_data->ops->get_property) {
		return -ENOTSUP;
	}


	/* Convert generic property type to network property type */
	switch (prop->type) {
	case MAVWRAP_PROPERTY_NET_REMOTE_IP:
		net_prop.prop = MAVWRAP_NET_PROP_REMOTE_IP;
		break;
	case MAVWRAP_PROPERTY_NET_REMOTE_PORT:
		net_prop.prop = MAVWRAP_NET_PROP_REMOTE_PORT;
		break;
	case MAVWRAP_PROPERTY_NET_LOCAL_PORT:
		net_prop.prop = MAVWRAP_NET_PROP_LOCAL_PORT;
		break;
	default:
		return -EINVAL;
	}

	ret = netif_data->ops->get_property(dev, &net_prop);
	if (ret < 0) {
		return ret;
	}


	/* Convert back to generic property */
	switch (net_prop.prop) {
	case MAVWRAP_NET_PROP_REMOTE_IP:
		prop->value.str = net_prop.value.ip_str;
		break;
	case MAVWRAP_NET_PROP_REMOTE_PORT:
	case MAVWRAP_NET_PROP_LOCAL_PORT:
		prop->value.u16 = net_prop.value.port;
		break;
	}

	return 0;
}


const struct mavwrap_transport_ops mavwrap_netif_ops = {
	.init = mavwrap_netif_init,
	.send = mavwrap_netif_send,
	.set_rx_callback = mavwrap_netif_set_rx_callback,
	.set_property = mavwrap_netif_set_property,
	.get_property = mavwrap_netif_get_property,
};
