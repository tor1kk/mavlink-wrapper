/**
 * @file mavwrap_dt.h
 * @brief MAVLink wrapper devicetree macros
 *
 * This file contains devicetree processing macros for device instantiation.
 */

#ifndef MAVWRAP_DT_H
#define MAVWRAP_DT_H

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif


/* Transport node */
#define MAVWRAP_TRANSPORT_NODE(inst) \
	DT_PHANDLE(DT_DRV_INST(inst), transport)


/* Network interface check */
#define MAVWRAP_HAS_NETIF(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), net_interface, 0)


/* UART check */
#define MAVWRAP_HAS_UART(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), serial_interface, 0)


/* Transport device */
#define MAVWRAP_TRANSPORT_DEV(inst) \
	DEVICE_DT_GET(MAVWRAP_TRANSPORT_NODE(inst))


/* Transport type */
#define MAVWRAP_TRANSPORT_TYPE(inst) \
	COND_CODE_1( \
		MAVWRAP_HAS_UART(inst), \
		(MAVWRAP_TRANSPORT_UART), \
		( \
			COND_CODE_1( \
				MAVWRAP_HAS_NETIF(inst), \
				(MAVWRAP_TRANSPORT_NETIF), \
				(MAVWRAP_TRANSPORT_UNKNOWN) \
			) \
		) \
	)


/* Transport ops declarations */
#if CONFIG_MAVWRAP_TRANSPORT_UART
extern const struct mavwrap_transport_ops mavwrap_uart_ops;
#endif

#if CONFIG_MAVWRAP_TRANSPORT_NETIF
extern const struct mavwrap_transport_ops mavwrap_netif_ops;
#endif


/* Transport ops */
#define MAVWRAP_TRANSPORT_OPS(inst) \
	COND_CODE_1( \
		MAVWRAP_HAS_UART(inst), \
		(&mavwrap_uart_ops), \
		( \
			COND_CODE_1( \
				MAVWRAP_HAS_NETIF(inst), \
				(&mavwrap_netif_ops), \
				(NULL) \
			) \
		) \
	)


/* Network configuration macros */
#define MAVWRAP_NETIF_DHCP_ENABLE(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), use_dhcp, 0)

#define MAVWRAP_NETIF_LOCAL_IP(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), local_ip, "0.0.0.0")

#define MAVWRAP_NETIF_REMOTE_IP(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), remote_ip, "0.0.0.0")

#define MAVWRAP_NETIF_LOCAL_PORT(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), local_port, 14550)

#define MAVWRAP_NETIF_REMOTE_PORT(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), remote_port, 14551)

#define MAVWRAP_NETIF_TYPE(inst) \
	((enum mavwrap_net_type)DT_ENUM_IDX_OR(DT_DRV_INST(inst), net_type, 0))


/* UART config initialization */
#define MAVWRAP_UART_CONFIG_INIT(inst) \
	{ \
	}


/* Network config initialization */
#define MAVWRAP_NETIF_CONFIG_INIT(inst) \
	{ \
		.local_ip = MAVWRAP_NETIF_LOCAL_IP(inst), \
		.remote_ip = MAVWRAP_NETIF_REMOTE_IP(inst), \
		.local_port = MAVWRAP_NETIF_LOCAL_PORT(inst), \
		.remote_port = MAVWRAP_NETIF_REMOTE_PORT(inst), \
		.net_type = MAVWRAP_NETIF_TYPE(inst), \
		.dhcp_enabled = MAVWRAP_NETIF_DHCP_ENABLE(inst), \
	}


/* Define transport-specific data structures (statically allocated) */
#define MAVWRAP_TRANSPORT_DATA_DEFINE(inst) \
	COND_CODE_1(MAVWRAP_HAS_UART(inst), \
		(static struct mavwrap_uart_data mavwrap_uart_data_##inst = {0};), \
		(COND_CODE_1(MAVWRAP_HAS_NETIF(inst), \
			(static struct mavwrap_netif_data mavwrap_netif_data_##inst = {0};), \
			())))


/* Get pointer to transport data */
#define MAVWRAP_TRANSPORT_DATA_PTR(inst) \
	COND_CODE_1(MAVWRAP_HAS_UART(inst), \
		(&mavwrap_uart_data_##inst), \
		(COND_CODE_1(MAVWRAP_HAS_NETIF(inst), \
			(&mavwrap_netif_data_##inst), \
			(NULL))))


/* Define transport-specific config structures (statically allocated) */
#define MAVWRAP_TRANSPORT_CONFIG_DEFINE(inst) \
	COND_CODE_1(MAVWRAP_HAS_UART(inst), \
		(static const struct mavwrap_uart_config mavwrap_uart_cfg_##inst = \
			MAVWRAP_UART_CONFIG_INIT(inst);), \
		(COND_CODE_1(MAVWRAP_HAS_NETIF(inst), \
			(static const struct mavwrap_netif_config mavwrap_netif_cfg_##inst = \
				MAVWRAP_NETIF_CONFIG_INIT(inst);), \
			())))


/* Get pointer to transport config */
#define MAVWRAP_TRANSPORT_CONFIG_PTR(inst) \
	COND_CODE_1(MAVWRAP_HAS_UART(inst), \
		(&mavwrap_uart_cfg_##inst), \
		(COND_CODE_1(MAVWRAP_HAS_NETIF(inst), \
			(&mavwrap_netif_cfg_##inst), \
			(NULL))))


/* TX thread stack (conditional) */
#ifdef CONFIG_MAVWRAP_TX_THREAD
#define MAVWRAP_TX_STACK_DEFINE(inst) \
	K_THREAD_STACK_DEFINE(mavwrap_tx_stack_##inst, CONFIG_MAVWRAP_TX_STACK_SIZE);
#define MAVWRAP_TX_CONFIG_INIT(inst) \
	.tx_thread_stack = mavwrap_tx_stack_##inst, \
	.tx_stack_size = K_THREAD_STACK_SIZEOF(mavwrap_tx_stack_##inst),
#else
#define MAVWRAP_TX_STACK_DEFINE(inst)
#define MAVWRAP_TX_CONFIG_INIT(inst)
#endif


/* Main config initialization */
#define MAVWRAP_CONFIG_INIT(inst) \
	{ \
		.transport_dev = MAVWRAP_TRANSPORT_DEV(inst), \
		.ops = MAVWRAP_TRANSPORT_OPS(inst), \
		.transport_type = MAVWRAP_TRANSPORT_TYPE(inst), \
		.chan = (mavlink_channel_t)(inst), \
		.thread_stack = mavwrap_rx_stack_##inst, \
		.stack_size = K_THREAD_STACK_SIZEOF(mavwrap_rx_stack_##inst), \
		MAVWRAP_TX_CONFIG_INIT(inst) \
		.transport_config = MAVWRAP_TRANSPORT_CONFIG_PTR(inst), \
	}


/* Main data initialization */
#define MAVWRAP_DATA_INIT(inst) \
	{ \
		.transport_data = MAVWRAP_TRANSPORT_DATA_PTR(inst), \
	}


/* Devicetree validation */
#define MAVWRAP_VALIDATE_DT(inst) \
	BUILD_ASSERT( \
		DT_NODE_HAS_PROP(DT_DRV_INST(inst), transport), \
		"mavlink-wrapper: 'transport' phandle is required"); \
	BUILD_ASSERT( \
		MAVWRAP_HAS_UART(inst) || MAVWRAP_HAS_NETIF(inst), \
		"mavlink-wrapper: unknown transport type"); \
	BUILD_ASSERT( \
		!(MAVWRAP_HAS_UART(inst) && MAVWRAP_HAS_NETIF(inst)), \
		"mavlink-wrapper: set only one of serial-interface / net-interface"); \
	BUILD_ASSERT( \
		inst < MAVLINK_COMM_NUM_BUFFERS, \
		"mavlink-wrapper: too many instances, exceeds MAVLINK_COMM_NUM_BUFFERS");


/* Device instantiation */
#define MAVWRAP_DEVICE_INIT(inst) \
	MAVWRAP_VALIDATE_DT(inst); \
	\
	/* Allocate transport-specific data and config */ \
	MAVWRAP_TRANSPORT_DATA_DEFINE(inst) \
	MAVWRAP_TRANSPORT_CONFIG_DEFINE(inst) \
	\
	/* Allocate thread stacks */ \
	K_THREAD_STACK_DEFINE( \
		mavwrap_rx_stack_##inst, \
		CONFIG_MAVWRAP_RX_STACK_SIZE); \
	MAVWRAP_TX_STACK_DEFINE(inst) \
	\
	/* Initialize main data structure with pointer */ \
	static struct mavwrap_data mavwrap_data_##inst = \
		MAVWRAP_DATA_INIT(inst); \
	\
	/* Initialize main config structure with pointer */ \
	static const struct mavwrap_config mavwrap_config_##inst = \
		MAVWRAP_CONFIG_INIT(inst); \
	\
	/* Register device */ \
	DEVICE_DT_INST_DEFINE( \
		inst, \
		mavwrap_init, \
		NULL, \
		&mavwrap_data_##inst, \
		&mavwrap_config_##inst, \
		POST_KERNEL, \
		CONFIG_MAVWRAP_INIT_PRIORITY, \
		NULL);


#ifdef __cplusplus
}
#endif

#endif /* MAVWRAP_DT_H */
