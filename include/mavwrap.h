#ifndef ZEPHYR_INCLUDE_DRIVERS_MAVWRAP_H_
#define ZEPHYR_INCLUDE_DRIVERS_MAVWRAP_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <common/mavlink.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief MAVLink statistics structure
 */
struct mavwrap_stats {
	uint32_t rx_packets;      /**< Successfully received packets */
	uint32_t tx_packets;      /**< Successfully transmitted packets */
	uint32_t rx_errors;       /**< RX errors (CRC, parse, etc) */
	uint32_t tx_errors;       /**< TX errors */
	uint32_t rx_dropped;      /**< Dropped packets (buffer full) */
    uint32_t tx_timeouted;    /**< Dropped packets (buffer full) */
};


/**
 * @typedef mavwrap_rx_callback_t
 * @brief Callback function for received MAVLink messages
 *
 * Called from RX thread context. Keep processing minimal or defer work.
 *
 * @param dev MAVLink device
 * @param msg Received MAVLink message (valid only during callback)
 * @param user_data User data passed during callback registration
 */
typedef void (*mavwrap_rx_callback_t)(const struct device *dev,
                                     const mavlink_message_t *msg,
                                     void *user_data);


/**
 * @brief Set RX callback for MAVLink messages
 *
 * @param dev MAVLink device
 * @param callback Callback function (NULL to disable)
 * @param user_data User data to pass to callback
 * @return 0 on success, negative errno on failure
 */
int mavwrap_set_rx_callback(const struct device *dev,
                           mavwrap_rx_callback_t callback,
                           void *user_data);


/**
 * @brief Send MAVLink message
 *
 * Message is queued for transmission. Function returns immediately.
 *
 * @param dev MAVLink device
 * @param msg MAVLink message to send
 * @return 0 on success, negative errno on failure
 * @retval -ENOMEM TX buffer full
 * @retval -EINVAL Invalid message
 */
int mavwrap_send_message(const struct device *dev,
                        const mavlink_message_t *msg);


/**
 * @brief Get MAVLink statistics
 *
 * @param dev MAVLink device
 * @param stats Pointer to statistics structure to fill
 * @return 0 on success, negative errno on failure
 */
int mavwrap_get_stats(const struct device *dev,
                     struct mavwrap_stats *stats);


/**
 * @brief Reset MAVLink statistics
 *
 * @param dev MAVLink device
 * @return 0 on success, negative errno on failure
 */
int mavwrap_reset_stats(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MAVWRAP_H_ */