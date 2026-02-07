#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include "mavwrap_common.h"
#include <common/mavlink.h>

LOG_MODULE_DECLARE(mavwrap);

static void uart_notify_rx(const struct device *dev, uint8_t *buff, size_t len)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_uart_data *uart_data = data->transport_data;

	if (uart_data->rx_callback && len > 0) {
		uart_data->rx_callback(dev, buff, len, uart_data->user_data);
	}
}

#ifdef CONFIG_UART_ASYNC_API

static void uart_dma_callback(const struct device *dev,
                               struct uart_event *evt,
                               void *user_data)
{
	const struct device *mavwrap_dev = user_data;
	struct mavwrap_data *data = mavwrap_dev->data;
	struct mavwrap_uart_data *uart_data = data->transport_data;

	switch (evt->type) {
	case UART_RX_RDY:
		uart_notify_rx(mavwrap_dev,
		               evt->data.rx.buf + evt->data.rx.offset,
		               evt->data.rx.len);
		break;

	case UART_RX_BUF_RELEASED:
		break;

	case UART_TX_DONE:
		uart_data->tx_in_progress = false;
		k_sem_give(&uart_data->tx_sem);
		break;

	case UART_TX_ABORTED:
		LOG_WRN("[%s] UART TX aborted", mavwrap_dev->name);
		uart_data->tx_in_progress = false;
		k_sem_give(&uart_data->tx_sem);
		break;

	case UART_RX_DISABLED:
		LOG_WRN("[%s] UART RX disabled", mavwrap_dev->name);
		break;

	case UART_RX_STOPPED:
		LOG_ERR("[%s] UART RX stopped due to error", mavwrap_dev->name);
		/* Try to re-enable RX */
		uart_data->rx_buf_idx = 0;
		uart_rx_enable(dev,
		               uart_data->rx_buf[0],
		               CONFIG_MAVWRAP_UART_DMA_RX_BUF_SIZE,
		               (int32_t)CONFIG_MAVWRAP_UART_RX_TIMEOUT_MS);
		break;

	case UART_RX_BUF_REQUEST:
		uart_data->rx_buf_idx = (uart_data->rx_buf_idx + 1) % 2;
		uart_rx_buf_rsp(dev,
		                uart_data->rx_buf[uart_data->rx_buf_idx],
		                CONFIG_MAVWRAP_UART_DMA_RX_BUF_SIZE);
		break;

	default:
		break;
	}
}

#endif

static void uart_irq_callback(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = user_data;
	struct mavwrap_data *data = dev->data;
	struct mavwrap_uart_data *uart_data = data->transport_data;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (uart_irq_rx_ready(uart_dev)) {
		uint8_t rx_buff[CONFIG_MAVWRAP_UART_IRQ_RX_BUF_SIZE];
		int rx_len = uart_fifo_read(uart_dev, rx_buff, sizeof(rx_buff));

		if (rx_len > 0) {
			uart_notify_rx(dev, rx_buff, rx_len);
		}
	}

	if (uart_irq_tx_ready(uart_dev)) {
		if (uart_data->tx_buf_pos < uart_data->tx_buf_len) {
			size_t remaining = uart_data->tx_buf_len - uart_data->tx_buf_pos;
			size_t written = uart_fifo_fill(uart_dev,
			                                &uart_data->tx_buf[uart_data->tx_buf_pos],
			                                remaining);
			uart_data->tx_buf_pos += written;

			/* This should never overflow - if it does, we have a serious bug */
			__ASSERT(uart_data->tx_buf_pos <= uart_data->tx_buf_len,
			         "TX buffer position overflow: %zu > %zu",
			         uart_data->tx_buf_pos, uart_data->tx_buf_len);
		} else {
			uart_irq_tx_disable(uart_dev);
			uart_data->tx_buf_len = 0;
			uart_data->tx_buf_pos = 0;
			uart_data->tx_in_progress = false;
			k_sem_give(&uart_data->tx_sem);
		}
	}
}


static int mavwrap_uart_init(const struct device *dev)
{
	const struct mavwrap_config *config = dev->config;
	struct mavwrap_data *data = dev->data;
	struct mavwrap_uart_data *uart_data = data->transport_data;
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


static int mavwrap_uart_send(const struct device *dev,
                              const uint8_t *buf,
                              size_t len)
{
	const struct mavwrap_config *config = dev->config;
	struct mavwrap_data *data = dev->data;
	struct mavwrap_uart_data *uart_data = data->transport_data;
	const struct device *uart_dev = config->transport_dev;
	int ret;

	if (!buf || len == 0) {
		return -EINVAL;
	}

	if (len > MAVLINK_MAX_PACKET_LEN) {
		LOG_ERR("[%s] TX buffer too large: %zu > %u",
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
		ret = uart_tx(uart_dev, buf, len, (int32_t)(CONFIG_MAVWRAP_UART_TX_TIMEOUT_MS * 1000));

		if (ret == 0) {
			/* Success - callback will release semaphore */
			return 0;
		}


		/* DMA TX failed - cleanup and fall back to IRQ mode */
		LOG_WRN("[%s] Async TX failed (%d), switching to IRQ mode", dev->name, ret);
		uart_data->use_dma = false;
		uart_data->tx_in_progress = false;
		
		/* Configure IRQ mode */
		uart_irq_rx_disable(uart_dev);
		uart_irq_tx_disable(uart_dev);
		uart_irq_callback_user_data_set(uart_dev, uart_irq_callback, (void *)dev);
		
		/* We still hold the semaphore, so can proceed with IRQ mode */
	}
#endif

	/* IRQ mode transmission */
	memcpy(uart_data->tx_buf, buf, len);
	uart_data->tx_buf_len = len;
	uart_data->tx_buf_pos = 0;
	uart_data->tx_in_progress = true;

	uart_irq_tx_enable(uart_dev);

	return 0;
}


static int mavwrap_uart_set_rx_callback(const struct device *dev,
                                        mavwrap_transport_rx_cb_t callback,
                                        void *user_data)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_uart_data *uart_data = data->transport_data;
	const struct mavwrap_config *config = dev->config;
	const struct device *uart_dev = config->transport_dev;

	uart_data->rx_callback = callback;
	uart_data->user_data = user_data;

#ifdef CONFIG_UART_ASYNC_API
	if (uart_data->use_dma) {
		int ret = uart_rx_enable(uart_dev,
		                         uart_data->rx_buf[0],
		                         CONFIG_MAVWRAP_UART_DMA_RX_BUF_SIZE,
		                         (int32_t)CONFIG_MAVWRAP_UART_RX_TIMEOUT_MS);
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

		/* Configure IRQ mode */
		uart_irq_tx_disable(uart_dev);
		uart_irq_callback_user_data_set(uart_dev, uart_irq_callback, (void *)dev);
	}
#endif

	/* Enable IRQ mode RX */
	uart_irq_rx_enable(uart_dev);
	LOG_INF("[%s] UART IRQ RX enabled", dev->name);

	return 0;
}


const struct mavwrap_transport_ops mavwrap_uart_ops = {
	.init = mavwrap_uart_init,
	.send = mavwrap_uart_send,
	.set_rx_callback = mavwrap_uart_set_rx_callback,
	.set_property = NULL,
	.get_property = NULL,
};