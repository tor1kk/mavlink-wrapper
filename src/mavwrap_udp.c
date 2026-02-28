#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "mavwrap_common.h"


LOG_MODULE_DECLARE(mavwrap);


/**
 * UDP receive callback
 */
static void udp_recv_callback(struct net_context *context,
                              struct net_pkt *pkt,
                              union net_ip_header *ip_hdr,
                              union net_proto_header *proto_hdr,
                              int status,
                              void *user_data)
{
	const struct device *dev = user_data;
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;
	uint8_t rx_buf[MAVLINK_MAX_PACKET_LEN];
	size_t len;
	int ret;

	if (!pkt) {
		if (status == -ECONNRESET || status == -ETIMEDOUT) {
			LOG_WRN("[%s] Connection reset/timeout", dev->name);
			netif_data->connected = false;
		} else if (status < 0) {
			LOG_ERR("[%s] RX error: %d", dev->name, status);
		}
		return;
	}

	len = net_pkt_get_len(pkt);

	if (len == 0) {
		net_pkt_unref(pkt);
		return;
	}

	if (len > MAVLINK_MAX_PACKET_LEN) {
		LOG_WRN("[%s] RX packet too large: %zu > %u", 
		        dev->name, len, MAVLINK_MAX_PACKET_LEN);
		net_pkt_unref(pkt);
		return;
	}

	net_pkt_cursor_init(pkt);

	ret = net_pkt_read(pkt, rx_buf, len);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to read packet data: %d", dev->name, ret);
		net_pkt_unref(pkt);
		return;
	}

	net_pkt_unref(pkt);

	/* Pass to transport layer callback (local copy avoids TOCTOU race) */
	mavwrap_transport_rx_cb_t cb = netif_data->rx_callback;

	if (cb) {
		cb(dev, rx_buf, len, netif_data->user_data);
	}
}


/**
 * Initialize UDP socket structures (called during init)
 */
static int mavwrap_udp_init(const struct device *dev)
{
	/* Nothing specific to do here for UDP */
	/* Socket creation happens in connect() after network is ready */
	return 0;
}


/**
 * Create and bind UDP socket
 */
static int mavwrap_udp_connect(const struct device *dev)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;
	int ret;

	/* Create UDP socket */
	ret = net_context_get(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &netif_data->ctx);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to get UDP context: %d", dev->name, ret);
		return ret;
	}


	/* Bind to local address */
	ret = net_context_bind(netif_data->ctx,
	                       (struct sockaddr *)&netif_data->runtime_config.local_addr,
	                       sizeof(netif_data->runtime_config.local_addr));
	if (ret < 0) {
		LOG_ERR("[%s] Failed to bind UDP: %d", dev->name, ret);
		net_context_put(netif_data->ctx);
		netif_data->ctx = NULL;
		return ret;
	}


	/* Setup receive callback */
	ret = net_context_recv(netif_data->ctx,
	                       udp_recv_callback,
	                       K_NO_WAIT,
	                       (void *)dev);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to setup UDP receive: %d", dev->name, ret);
		net_context_put(netif_data->ctx);
		netif_data->ctx = NULL;
		return ret;
	}

	netif_data->connected = true;
	
	LOG_INF("[%s] UDP socket ready (bound to port %u)",
	        dev->name, ntohs(netif_data->runtime_config.local_addr.sin_port));

	return 0;
}


/**
 * Send UDP packet
 */
static int mavwrap_udp_send(const struct device *dev,
                            const uint8_t *buf,
                            size_t len)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;
	int ret;

	if (!buf || len == 0 || len > MAVLINK_MAX_PACKET_LEN) {
		return -EINVAL;
	}

	if (!netif_data->ctx) {
		LOG_ERR("[%s] UDP context not initialized", dev->name);
		return -ENOTCONN;
	}

	if (!netif_data->connected) {
		LOG_ERR("[%s] UDP not connected", dev->name);
		return -ENOTCONN;
	}


	/* Send to configured remote address */
	ret = net_context_sendto(netif_data->ctx,
	                         buf,
	                         len,
	                         (struct sockaddr *)&netif_data->runtime_config.remote_addr,
	                         sizeof(netif_data->runtime_config.remote_addr),
	                         NULL,
	                         K_MSEC(CONFIG_MAVWRAP_NETIF_SEND_TIMEOUT_MS),
	                         NULL);

	if (ret < 0) {
		LOG_ERR("[%s] Failed to send UDP packet: %d", dev->name, ret);
		return ret;
	}

	return 0;
}


/**
 * Reconnect UDP socket with new configuration
 */
static int mavwrap_udp_reconnect(const struct device *dev)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;

	LOG_INF("[%s] Reconnecting UDP socket...", dev->name);

	/* Close existing socket */
	if (netif_data->ctx) {
		net_context_put(netif_data->ctx);
		netif_data->ctx = NULL;
		netif_data->connected = false;
	}


	/* Re-create socket with updated configuration */
	return mavwrap_udp_connect(dev);
}


/**
 * Set UDP runtime property
 */
static int mavwrap_udp_set_property(const struct device *dev,
                                    const struct mavwrap_net_property_value *prop,
                                    bool apply_immediately)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;
	bool need_reconnect = false;
	int ret = 0;

	k_mutex_lock(&netif_data->config_mutex, K_FOREVER);

	/* Update runtime configuration */
	switch (prop->prop) {
	case MAVWRAP_NET_PROP_REMOTE_IP:
		if (zsock_inet_pton(AF_INET, prop->value.ip_str, 
		                    &netif_data->runtime_config.remote_addr.sin_addr) != 1) {
			LOG_ERR("[%s] Invalid IP address: %s", dev->name, prop->value.ip_str);
			ret = -EINVAL;
			goto unlock;
		}
		strncpy(netif_data->runtime_config.remote_ip,
		        prop->value.ip_str,
		        sizeof(netif_data->runtime_config.remote_ip) - 1);
		netif_data->runtime_config.remote_ip[sizeof(netif_data->runtime_config.remote_ip) - 1] = '\0';
		LOG_INF("[%s] Remote IP updated to %s", dev->name, prop->value.ip_str);
		/* UDP doesn't need reconnect for remote address change */
		break;

	case MAVWRAP_NET_PROP_REMOTE_PORT:
		netif_data->runtime_config.remote_addr.sin_port = htons(prop->value.port);
		netif_data->runtime_config.remote_port = prop->value.port;
		LOG_INF("[%s] Remote port updated to %u", dev->name, prop->value.port);
		/* UDP doesn't need reconnect for remote port change */
		break;

	case MAVWRAP_NET_PROP_LOCAL_PORT:
		netif_data->runtime_config.local_addr.sin_port = htons(prop->value.port);
		netif_data->runtime_config.local_port = prop->value.port;
		LOG_INF("[%s] Local port updated to %u", dev->name, prop->value.port);
		need_reconnect = true;
		break;

	default:
		ret = -EINVAL;
		goto unlock;
	}


	/* Apply changes if requested */
	if (apply_immediately && need_reconnect) {
		enum mavwrap_net_state old_state = atomic_get(&netif_data->state);
		atomic_set(&netif_data->state, MAVWRAP_NET_STATE_RECONFIGURING);

		ret = mavwrap_udp_reconnect(dev);

		if (ret < 0) {
			LOG_ERR("[%s] Failed to reconnect after property change: %d",
			        dev->name, ret);
			atomic_set(&netif_data->state, old_state);
		} else {
			atomic_set(&netif_data->state, MAVWRAP_NET_STATE_RUNNING);
			LOG_INF("[%s] Property applied and reconnected successfully", dev->name);
		}
	}

unlock:
	k_mutex_unlock(&netif_data->config_mutex);
	return ret;
}


/**
 * Get UDP runtime property
 */
static int mavwrap_udp_get_property(const struct device *dev,
                                    struct mavwrap_net_property_value *prop)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_netif_data *netif_data = data->transport_data;
	int ret = 0;

	k_mutex_lock(&netif_data->config_mutex, K_FOREVER);

	switch (prop->prop) {
	case MAVWRAP_NET_PROP_REMOTE_IP:
		prop->value.ip_str = netif_data->runtime_config.remote_ip;
		break;

	case MAVWRAP_NET_PROP_REMOTE_PORT:
		prop->value.port = netif_data->runtime_config.remote_port;
		break;

	case MAVWRAP_NET_PROP_LOCAL_PORT:
		prop->value.port = netif_data->runtime_config.local_port;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	k_mutex_unlock(&netif_data->config_mutex);
	return ret;
}


const struct mavwrap_net_ops mavwrap_udp_ops = {
	.init = mavwrap_udp_init,
	.connect = mavwrap_udp_connect,
	.send = mavwrap_udp_send,
	.reconnect = mavwrap_udp_reconnect,
	.set_property = mavwrap_udp_set_property,
	.get_property = mavwrap_udp_get_property,
};
