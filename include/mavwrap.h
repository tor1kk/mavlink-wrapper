/**
 * @file mavwrap.h
 * @brief MAVLink Wrapper Public API
 * 
 * This file provides the public API for the MAVLink communication wrapper.
 * Supports multiple transport types (UART, UDP network) with runtime
 * configuration capabilities.
 */

#ifndef MAVWRAP_H
#define MAVWRAP_H

#include <zephyr/device.h>
#include <stddef.h>
#include <stdint.h>

#include <common/mavlink.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MAVLink message receive callback
 * 
 * Called when a complete MAVLink message has been received and parsed.
 * 
 * @param dev MAVLink wrapper device
 * @param msg Received MAVLink message
 * @param user_data User data pointer provided during callback registration
 */
typedef void (*mavwrap_rx_callback_t)(const struct device *dev,
                                       const mavlink_message_t *msg,
                                       void *user_data);

/**
 * @brief MAVLink statistics
 *
 * Note: Statistics are updated atomically for thread-safety.
 * Values represent snapshot at the time of mavwrap_get_stats() call.
 */
struct mavwrap_stats {
	uint32_t rx_packets;        /**< Total received packets */
	uint32_t tx_packets;        /**< Total transmitted packets */
	uint32_t rx_errors;         /**< RX parse errors */
	uint32_t tx_errors;         /**< TX errors */
	uint32_t rx_buff_overflow;  /**< RX ring buffer overflows */
};

/**
 * @brief Property types for runtime configuration
 */
enum mavwrap_property_type {
	/* Network properties */
	MAVWRAP_PROPERTY_NET_REMOTE_IP,      /**< Remote IP address (string) */
	MAVWRAP_PROPERTY_NET_REMOTE_PORT,    /**< Remote port (uint16_t) */
	MAVWRAP_PROPERTY_NET_LOCAL_PORT,     /**< Local port (uint16_t) */
	
	/* UART properties (future) */
	MAVWRAP_PROPERTY_UART_BAUDRATE,      /**< UART baudrate (uint32_t) */
};

/**
 * @brief Property value container
 */
struct mavwrap_property_value {
	enum mavwrap_property_type type;  /**< Property type */
	union {
		uint16_t u16;       /**< 16-bit unsigned value */
		uint32_t u32;       /**< 32-bit unsigned value */
		const char *str;    /**< String value */
	} value;                /**< Property value */
	bool apply_immediately; /**< If true, apply change and reconnect immediately */
};

/**
 * @brief Start MAVLink transport
 *
 * Connect the transport and begin receiving messages.
 * Properties can be changed via mavwrap_set_property() before calling this.
 *
 * @param dev MAVLink wrapper device
 * @param callback RX callback function (may be NULL for TX-only)
 * @param user_data User data to pass to callback
 * @return 0 on success, negative errno on error
 */
int mavwrap_start(const struct device *dev,
                  mavwrap_rx_callback_t callback,
                  void *user_data);

/**
 * @brief Send MAVLink message
 * 
 * Serialize and send a MAVLink message through the configured transport.
 * 
 * @param dev MAVLink wrapper device
 * @param msg MAVLink message to send
 * @return 0 on success, negative errno on error
 */
int mavwrap_send_message(const struct device *dev,
                         const mavlink_message_t *msg);

/**
 * @brief Set runtime property
 * 
 * Change transport configuration at runtime. Depending on the property
 * and the apply_immediately flag, this may trigger a reconnection.
 * 
 * @param dev MAVLink wrapper device
 * @param prop Property to set
 * @return 0 on success, negative errno on error
 */
int mavwrap_set_property(const struct device *dev,
                         const struct mavwrap_property_value *prop);

/**
 * @brief Get current property value
 * 
 * Retrieve the current value of a runtime property.
 * 
 * @param dev MAVLink wrapper device
 * @param prop Property to get (type field must be set, value will be filled)
 * @return 0 on success, negative errno on error
 */
int mavwrap_get_property(const struct device *dev,
                         struct mavwrap_property_value *prop);

/**
 * @brief Get statistics
 * 
 * Retrieve current communication statistics.
 * 
 * @param dev MAVLink wrapper device
 * @param stats Output statistics structure
 * @return 0 on success, negative errno on error
 */
int mavwrap_get_stats(const struct device *dev,
                      struct mavwrap_stats *stats);

/**
 * @brief Reset statistics
 * 
 * Reset all statistics counters to zero.
 * 
 * @param dev MAVLink wrapper device
 * @return 0 on success, negative errno on error
 */
int mavwrap_reset_stats(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* MAVWRAP_H */