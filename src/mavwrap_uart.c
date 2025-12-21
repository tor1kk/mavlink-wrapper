#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <common/mavlink.h>

#include "mavwrap_transport.h"


LOG_MODULE_DECLARE(mavwrap);


/**
 * @brief RX frame structure for queue
 */
struct mavwrap_rx_frame {
	uint16_t len;
	uint8_t buf[CONFIG_MAVWRAP_UART_RX_BUF_SIZE];
};


/**
 * @brief UART transport data (per device)
 */
struct mavwrap_uart_data {
	const struct device *uart_dev;
	
	mavwrap_transport_rx_cb_t rx_callback;
	void *user_data;
	
	struct k_thread rx_thread;
	k_tid_t rx_tid;
	K_THREAD_STACK_MEMBER(rx_stack, CONFIG_MAVWRAP_UART_RX_STACK_SIZE);
	
	struct k_msgq rx_msgq;
	char __aligned(4) rx_msgq_buffer[CONFIG_MAVWRAP_RX_QUEUE_LEN * 
	                                 sizeof(struct mavwrap_rx_frame)];
	
	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	size_t tx_buf_len;
	size_t tx_buf_pos;
	struct k_sem tx_sem;
	
#ifdef CONFIG_UART_ASYNC_API
	uint8_t rx_buf[2][CONFIG_MAVWRAP_UART_RX_BUF_SIZE];
	uint8_t rx_buf_idx;
	bool use_dma;
#endif
};


/**
 * @brief Enqueue received data for processing
 */
static inline void mavwrap_rx_enqueue(const struct device *dev,
                                      const uint8_t *buf,
                                      size_t len)
{
	struct mavwrap_uart_data *data = dev->data;
	struct mavwrap_rx_frame frame;

	if (len == 0 || len > CONFIG_MAVWRAP_UART_RX_BUF_SIZE) {
		return;
	}

	frame.len = len;
	memcpy(frame.buf, buf, len);

	if (k_msgq_put(&data->rx_msgq, &frame, K_NO_WAIT) != 0) {
		LOG_WRN("[%s] RX queue overflow, dropping %u bytes", 
		        dev->name, len);
	}
}


/**
 * @brief RX processing thread (per device)
 */
static void mavwrap_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	
	const struct device *dev = p1;
	struct mavwrap_uart_data *data = dev->data;
	struct mavwrap_rx_frame frame;

	LOG_INF("[%s] RX thread started", dev->name);

	while (1) {
		k_msgq_get(&data->rx_msgq, &frame, K_FOREVER);

		if (data->rx_callback && frame.len > 0) {
			data->rx_callback(dev, frame.buf, frame.len, data->user_data);
		}
	}
}


#ifdef CONFIG_UART_ASYNC_API

static void uart_dma_callback(const struct device *uart_dev,
                              struct uart_event *evt,
                              void *user_data)
{
	const struct device *dev = user_data;  
	struct mavwrap_uart_data *data = dev->data;
	
	switch (evt->type) {
	case UART_RX_RDY:
		mavwrap_rx_enqueue(dev,
		                  evt->data.rx.buf + evt->data.rx.offset,
		                  evt->data.rx.len);
		break;
		
	case UART_RX_BUF_RELEASED:
		break;
		
	case UART_TX_DONE:
		k_sem_give(&data->tx_sem);
		break;
		
	case UART_TX_ABORTED:
		LOG_ERR("[%s] UART TX aborted", dev->name);
		k_sem_give(&data->tx_sem);
		break;
		
	case UART_RX_DISABLED:
		LOG_WRN("[%s] UART RX disabled", dev->name);
		break;
		
	default:
		break;
	}
}

#endif /* CONFIG_UART_ASYNC_API */


static void uart_irq_callback(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = user_data;  
	struct mavwrap_uart_data *data = dev->data;
	
	if (!uart_irq_update(uart_dev)) {
		return;
	}
	
	if (uart_irq_rx_ready(uart_dev)) {
		uint8_t rx_buf[CONFIG_MAVWRAP_UART_RX_BUF_SIZE];
		size_t rx_len = 0;
		uint8_t byte;

		while (uart_fifo_read(uart_dev, &byte, 1) == 1) {
			rx_buf[rx_len++] = byte;

			if (rx_len >= CONFIG_MAVWRAP_UART_RX_BUF_SIZE) {
				mavwrap_rx_enqueue(dev, rx_buf, rx_len);
				rx_len = 0;
			}
		}
		
		if (rx_len > 0) {
			mavwrap_rx_enqueue(dev, rx_buf, rx_len);
		}
	}
	
	if (uart_irq_tx_ready(uart_dev)) {
		if (data->tx_buf_pos < data->tx_buf_len) {
			uart_fifo_fill(uart_dev, &data->tx_buf[data->tx_buf_pos], 1);
			data->tx_buf_pos++;
		} else {
			uart_irq_tx_disable(uart_dev);
			data->tx_buf_len = 0;
			data->tx_buf_pos = 0;
			k_sem_give(&data->tx_sem);
		}
	}
}


static int mavwrap_uart_init(const struct device *dev)
{
	const struct mavwrap_config *config = dev->config;
	struct mavwrap_uart_data *uart_data = dev->data;
	const struct device *uart_dev = config->transport.uart;
	
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("[%s] UART device not ready", dev->name);
		return -ENODEV;
	}
	
	uart_data->uart_dev = uart_dev;
	
	k_sem_init(&uart_data->tx_sem, 1, 1);
	uart_data->tx_buf_len = 0;
	uart_data->tx_buf_pos = 0;
	
	k_msgq_init(&uart_data->rx_msgq,
	           uart_data->rx_msgq_buffer,
	           sizeof(struct mavwrap_rx_frame),
	           CONFIG_MAVWRAP_RX_QUEUE_LEN);
	
	uart_data->rx_tid = k_thread_create(
		&uart_data->rx_thread,
		uart_data->rx_stack,
		K_THREAD_STACK_SIZEOF(uart_data->rx_stack),
		mavwrap_rx_thread,
		(void *)dev, NULL, NULL,
		CONFIG_MAVWRAP_UART_RX_PRIORITY,
		0, K_NO_WAIT);
	
	char thread_name[CONFIG_THREAD_MAX_NAME_LEN];
	snprintk(thread_name, sizeof(thread_name), "mav_rx_%s", dev->name);
	k_thread_name_set(uart_data->rx_tid, thread_name);
	
#ifdef CONFIG_UART_ASYNC_API
	uart_data->use_dma = true;
	uart_callback_set(uart_dev, uart_dma_callback, (void *)dev);
	LOG_INF("[%s] UART async API configured (DMA if available)", dev->name);
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
	const struct device *uart_dev = config->transport.uart;
	struct mavwrap_uart_data *uart_data = dev->data;
	int ret;
	
	if (len > MAVLINK_MAX_PACKET_LEN) {
		LOG_ERR("[%s] TX buffer too large: %u > %u", 
		        dev->name, len, MAVLINK_MAX_PACKET_LEN);
		return -ENOMEM;
	}
	
	ret = k_sem_take(&uart_data->tx_sem, K_MSEC(CONFIG_MAVWRAP_UART_TX_TIMEOUT_US / 1000));
	if (ret != 0) {
		LOG_ERR("[%s] TX timeout waiting for semaphore", dev->name);
		return -ETIMEDOUT;
	}
	
#ifdef CONFIG_UART_ASYNC_API
	if (uart_data->use_dma) {
		ret = uart_tx(uart_dev, buf, len, K_USEC(CONFIG_MAVWRAP_UART_TX_TIMEOUT_US));
		if (ret == 0) {
			return 0;
		}
		
		LOG_WRN("[%s] Async TX failed (%d), switching to IRQ mode", dev->name, ret);
		uart_data->use_dma = false;
		
		uart_irq_callback_user_data_set(uart_dev, uart_irq_callback, (void *)dev);
	}
#endif
	
	memcpy(uart_data->tx_buf, buf, len);
	uart_data->tx_buf_len = len;
	uart_data->tx_buf_pos = 0;
	
	uart_irq_tx_enable(uart_dev);
	
	return 0;
}


static int mavwrap_uart_set_rx_callback(const struct device *dev,
                                       mavwrap_transport_rx_cb_t callback,
                                       void *user_data)
{
	struct mavwrap_uart_data *uart_data = dev->data;
	const struct mavwrap_config *config = dev->config;
	const struct device *uart_dev = config->transport.uart;
	
	uart_data->rx_callback = callback;
	uart_data->user_data = user_data;
	
#ifdef CONFIG_UART_ASYNC_API
	if (uart_data->use_dma) {
		int ret = uart_rx_enable(uart_dev, 
		                        uart_data->rx_buf[0], 
		                        CONFIG_MAVWRAP_UART_RX_BUF_SIZE,
		                        K_USEC(CONFIG_MAVWRAP_UART_RX_TIMEOUT_US));
		if (ret == 0) {
			uart_rx_buf_rsp(uart_dev, 
			               uart_data->rx_buf[1], 
			               CONFIG_MAVWRAP_UART_RX_BUF_SIZE);
			LOG_INF("[%s] UART async RX enabled (DMA)", dev->name);
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



