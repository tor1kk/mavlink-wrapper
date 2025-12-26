#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include "mavwrap_transport.h"
#include <common/mavlink.h>


LOG_MODULE_DECLARE(mavwrap);


#define IRQ_RX_BUFF_SIZE		256


/**
 * @brief UART transport data (per device)
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
 * @brief Notify core layer of received data
 */
static void uart_notify_rx(const struct device *dev, uint8_t *buff, size_t len)
{
	struct mavwrap_uart_data *data = dev->data;

	if (data->rx_callback && len > 0) {
		data->rx_callback(dev, buff, len, data->user_data);
	}
}


#ifdef CONFIG_UART_ASYNC_API

/**
 * @brief DMA/Async UART callback
 */
static void uart_dma_callback(const struct device *uart_dev,
                               struct uart_event *evt,
                               void *user_data)
{
	const struct device *dev = user_data;
	struct mavwrap_uart_data *data = dev->data;

	switch (evt->type) {
	case UART_RX_RDY:
		uart_notify_rx(dev,
		               evt->data.rx.buf + evt->data.rx.offset,
		               evt->data.rx.len);
		break;

	case UART_RX_BUF_RELEASED:
		break;

	case UART_TX_DONE:
		data->tx_in_progress = false;
		k_sem_give(&data->tx_sem);
		break;

	case UART_TX_ABORTED:
		LOG_ERR("[%s] UART TX aborted", dev->name);
		data->tx_in_progress = false;
		k_sem_give(&data->tx_sem);
		break;

	case UART_RX_DISABLED:
		LOG_WRN("[%s] UART RX disabled", dev->name);
		break;

	case UART_RX_BUF_REQUEST:
		data->rx_buf_idx = (data->rx_buf_idx + 1) % 2;
		uart_rx_buf_rsp(uart_dev,
						data->rx_buf[data->rx_buf_idx],
						CONFIG_MAVWRAP_UART_DMA_RX_BUF_SIZE);
		break;

	default:
		break;
	}
}

#endif /* CONFIG_UART_ASYNC_API */


/**
 * @brief IRQ-based UART callback
 */
static void uart_irq_callback(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = user_data;
	struct mavwrap_uart_data *data = dev->data;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (uart_irq_rx_ready(uart_dev)) {
		uint8_t rx_buff[IRQ_RX_BUFF_SIZE];
		int rx_len = uart_fifo_read(uart_dev, rx_buff, sizeof(rx_buff));

		if (rx_len > 0) {
			uart_notify_rx(dev, rx_buff, rx_len);
		}
	}

	if (uart_irq_tx_ready(uart_dev)) {
		if (data->tx_buf_pos < data->tx_buf_len) {
			size_t written = uart_fifo_fill(uart_dev,
			                                &data->tx_buf[data->tx_buf_pos],
			                                data->tx_buf_len - data->tx_buf_pos);
			data->tx_buf_pos += written;
		} else {
			uart_irq_tx_disable(uart_dev);
			data->tx_buf_len = 0;
			data->tx_buf_pos = 0;
			data->tx_in_progress = false;
			k_sem_give(&data->tx_sem);
		}
	}
}


/**
 * @brief Initialize UART transport
 */
static int mavwrap_uart_init(const struct device *dev)
{
	const struct mavwrap_config *config = dev->config;
	struct mavwrap_uart_data *uart_data = dev->data;
	const struct device *uart_dev = config->transport_dev;

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("[%s] Transport device %s not ready",
		        dev->name, uart_dev->name);
		return -ENODEV;
	}

	uart_data->uart_dev = uart_dev;

	k_sem_init(&uart_data->tx_sem, 1, 1);
	uart_data->tx_buf_len = 0;
	uart_data->tx_buf_pos = 0;
	uart_data->tx_in_progress = false;

#ifdef CONFIG_UART_ASYNC_API
	uart_data->use_dma = true;
	uart_data->rx_buf_idx = 0;
	uart_callback_set(uart_dev, uart_dma_callback, (void *)dev);
	LOG_INF("[%s] UART async API configured", dev->name);
#else
	LOG_INF("[%s] UART IRQ mode configured", dev->name);
	uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);
	uart_irq_callback_user_data_set(uart_dev, uart_irq_callback, (void *)dev);
#endif

	return 0;
}


/**
 * @brief Send data over UART
 */
static int mavwrap_uart_send(const struct device *dev,
                              const uint8_t *buf,
                              size_t len)
{
	const struct mavwrap_config *config = dev->config;
	const struct device *uart_dev = config->transport_dev;
	struct mavwrap_uart_data *uart_data = dev->data;
	int ret;

	if (!buf || len == 0) {
		return -EINVAL;
	}

	if (len > MAVLINK_MAX_PACKET_LEN) {
		LOG_ERR("[%s] TX buffer too large: %u > %u",
		        dev->name, len, MAVLINK_MAX_PACKET_LEN);
		return -ENOMEM;
	}

	ret = k_sem_take(&uart_data->tx_sem, K_MSEC(CONFIG_MAVWRAP_UART_TX_TIMEOUT_MS));
	if (ret != 0) {
		LOG_ERR("[%s] TX timeout waiting for semaphore", dev->name);
		return -ETIMEDOUT;
	}

#ifdef CONFIG_UART_ASYNC_API
	if (uart_data->use_dma) {
		uart_data->tx_in_progress = true;
		ret = uart_tx(uart_dev, buf, len, K_MSEC(CONFIG_MAVWRAP_UART_TX_TIMEOUT_MS));

		if (ret == 0) {
			return 0;
		}

		LOG_WRN("[%s] Async TX failed (%d), switching to IRQ mode", dev->name, ret);
		uart_data->use_dma = false;
		uart_data->tx_in_progress = false;

		uart_irq_rx_disable(uart_dev);
		uart_irq_tx_disable(uart_dev);
		uart_irq_callback_user_data_set(uart_dev, uart_irq_callback, (void *)dev);
	}
#endif

	memcpy(uart_data->tx_buf, buf, len);
	uart_data->tx_buf_len = len;
	uart_data->tx_buf_pos = 0;
	uart_data->tx_in_progress = true;

	uart_irq_tx_enable(uart_dev);

	return 0;
}


/**
 * @brief Set RX callback
 */
static int mavwrap_uart_set_rx_callback(const struct device *dev,
                                        mavwrap_transport_rx_cb_t callback,
                                        void *user_data)
{
	struct mavwrap_uart_data *uart_data = dev->data;
	const struct mavwrap_config *config = dev->config;
	const struct device *uart_dev = config->transport_dev;

	uart_data->rx_callback = callback;
	uart_data->user_data = user_data;

#ifdef CONFIG_UART_ASYNC_API
	if (uart_data->use_dma) {
		int ret = uart_rx_enable(uart_dev,
		                         uart_data->rx_buf[0],
		                         CONFIG_MAVWRAP_UART_DMA_RX_BUF_SIZE,
		                         K_MSEC(CONFIG_MAVWRAP_UART_RX_TIMEOUT_MS));
		if (ret == 0) {
			uart_rx_buf_rsp(uart_dev,
			                uart_data->rx_buf[1],
			                CONFIG_MAVWRAP_UART_DMA_RX_BUF_SIZE);
			LOG_INF("[%s] UART async RX enabled", dev->name);
			return 0;
		}

		LOG_WRN("[%s] Failed to enable async RX (%d), switching to IRQ mode",
		        dev->name, ret);
		uart_data->use_dma = false;

		uart_irq_callback_user_data_set(uart_dev, uart_irq_callback, (void *)dev);
	}
#endif

	uart_irq_rx_enable(uart_dev);
	LOG_INF("[%s] UART IRQ RX enabled", dev->name);

	return 0;
}


const struct mavwrap_transport_ops mavwrap_uart_ops = {
	.init = mavwrap_uart_init,
	.send = mavwrap_uart_send,
	.set_rx_callback = mavwrap_uart_set_rx_callback,
};
