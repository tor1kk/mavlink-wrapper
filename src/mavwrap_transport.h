#ifndef MAVWRAP_TRANSPORT_H_
#define MAVWRAP_TRANSPORT_H_

#include <zephyr/device.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Callback for received data from transport
 * 
 * Called by transport layer (from workqueue context, not IRQ).
 * Core will parse bytes using mavlink_parse_char() in a loop.
 * 
 * @param dev Transport device
 * @param buf Received data buffer
 * @param len Buffer length
 * @param user_data User data from core
 */
typedef void (*mavwrap_transport_rx_cb_t)(const struct device *dev,
                                         const uint8_t *buf,
                                         size_t len,
                                         void *user_data);


/**
 * @brief Transport layer API
 */
struct mavwrap_transport_ops {
	/**
	 * @brief Initialize transport
	 * @param dev MAVLink device (not transport device!)
	 * @return 0 on success, negative errno on failure
	 */
	int (*init)(const struct device *dev);

	/**
	 * @brief Send data over transport
	 * @param dev MAVLink device
	 * @param buf Data buffer
	 * @param len Data length
	 * @return 0 on success, negative errno on failure
	 */
	int (*send)(const struct device *dev, const uint8_t *buf, size_t len);

	/**
	 * @brief Set callback for received data
	 * @param dev MAVLink device
	 * @param callback Callback function (called from workqueue, not IRQ)
	 * @param user_data User data to pass to callback
	 * @return 0 on success, negative errno on failure
	 */
	int (*set_rx_callback)(const struct device *dev,
	                       mavwrap_transport_rx_cb_t callback,
	                       void *user_data);
};


#ifdef __cplusplus
}
#endif

#endif /* MAVWRAP_TRANSPORT_H_ */
