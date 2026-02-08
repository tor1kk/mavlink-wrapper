/**
 * @file mavwrap_uart.h
 * @brief UART transport definitions
 *
 * This file contains UART-specific structures and definitions.
 * Only included when CONFIG_MAVWRAP_TRANSPORT_UART is enabled.
 */

#ifndef MAVWRAP_UART_H
#define MAVWRAP_UART_H

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <common/mavlink.h>

#ifdef __cplusplus
extern "C" {
#endif

/* mavwrap_transport_rx_cb_t is defined in mavwrap_common.h */

/**
 * UART transport runtime data
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
 * UART transport configuration
 */
struct mavwrap_uart_config {
	/* Empty for now - UART settings come from DT uart node */
};

#ifdef __cplusplus
}
#endif

#endif /* MAVWRAP_UART_H */
