# MAVLink Wrapper

MAVLink communication wrapper module for Zephyr RTOS.
Supports UART and UDP transports with devicetree configuration, runtime property changes, and thread-safe statistics.

## Installation

Add to your `west.yaml`:

```yaml
projects:
    -   name: mavlink-wrapper
        url: https://github.com/tor1kk/mavlink-wrapper.git
        revision: main
        path: modules/mavlink-wrapper
        submodules: true
```

## Configuration

### prj.conf

**UART transport:**
```ini
CONFIG_MAVWRAP=y
CONFIG_MAVWRAP_TRANSPORT_UART=y
CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
```

**UDP transport:**
```ini
CONFIG_MAVWRAP=y
CONFIG_MAVWRAP_TRANSPORT_NETIF=y
CONFIG_NETWORKING=y
CONFIG_NET_IPV4=y
CONFIG_NET_UDP=y
CONFIG_NET_SOCKETS=y
CONFIG_NET_MGMT=y
CONFIG_NET_MGMT_EVENT=y
CONFIG_NET_L2_ETHERNET=y
CONFIG_NET_DHCPV4=y          # if using DHCP
```

**Both transports at the same time:**
```ini
CONFIG_MAVWRAP=y
CONFIG_MAVWRAP_TRANSPORT_UART=y
CONFIG_MAVWRAP_TRANSPORT_NETIF=y
# ... plus serial and networking configs above
```

### Kconfig options

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_MAVWRAP_RX_STACK_SIZE` | 2048 | RX thread stack size (1024-8192) |
| `CONFIG_MAVWRAP_RX_THREAD_PRIORITY` | 5 | RX thread priority (1-99) |
| `CONFIG_MAVWRAP_RX_RING_SIZE` | 1024 | RX ring buffer size (256-4096) |
| `CONFIG_MAVWRAP_INIT_PRIORITY` | 80 | Device init priority (1-99) |
| `CONFIG_MAVWRAP_UART_TX_TIMEOUT_MS` | 1000 | UART TX timeout |
| `CONFIG_MAVWRAP_UART_RX_TIMEOUT_MS` | 100 | UART async RX timeout |
| `CONFIG_MAVWRAP_UART_IRQ_RX_BUF_SIZE` | 32 | UART IRQ mode RX buffer |
| `CONFIG_MAVWRAP_UART_DMA_RX_BUF_SIZE` | 512 | UART DMA RX buffer (needs `UART_ASYNC_API`) |
| `CONFIG_MAVWRAP_NETIF_CONNECT_TIMEOUT_MS` | 5000 | Network connection timeout |
| `CONFIG_MAVWRAP_NETIF_SEND_TIMEOUT_MS` | 1000 | Network send timeout |

### DeviceTree

**UART:**
```dts
mavlink_uart: mavlink-wrapper-uart {
    compatible = "mavlink-wrapper";
    transport = <&usart1>;
    serial-interface;
};
```

**UDP with DHCP:**
```dts
mavlink_netif: mavlink-wrapper-netif {
    compatible = "mavlink-wrapper";
    transport = <&mac>;
    net-interface;
    net-type = "udp";

    use-dhcp;
    local-port = <14550>;
    remote-ip = "192.168.1.100";
    remote-port = <14551>;
};
```

**UDP with static IP:**
```dts
mavlink_netif: mavlink-wrapper-netif {
    compatible = "mavlink-wrapper";
    transport = <&mac>;
    net-interface;
    net-type = "udp";

    local-ip = "192.168.1.10";
    local-port = <14560>;
    remote-ip = "192.168.1.100";
    remote-port = <14561>;
};
```

## API

```c
#include <mavwrap.h>
#include <common/mavlink.h>
```

| Function | Description |
|----------|-------------|
| `mavwrap_set_rx_callback(dev, cb, user_data)` | Register RX message callback |
| `mavwrap_send_message(dev, msg)` | Send a MAVLink message |
| `mavwrap_set_property(dev, prop)` | Change runtime config (IP, port) |
| `mavwrap_get_property(dev, prop)` | Read current config value |
| `mavwrap_get_stats(dev, stats)` | Get TX/RX statistics |
| `mavwrap_reset_stats(dev)` | Reset statistics counters |

### Runtime properties

```c
/* Change remote IP at runtime */
struct mavwrap_property_value prop = {
    .type = MAVWRAP_PROPERTY_NET_REMOTE_IP,
    .value.str = "192.168.1.200",
    .apply_immediately = true,
};
mavwrap_set_property(dev, &prop);
```

## Usage example

Heartbeat + ARM/DISARM on both UART and UDP:

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <mavwrap.h>
#include <common/mavlink.h>

#define SYS_ID  1
#define COMP_ID 1

static const struct device *mav_uart  = DEVICE_DT_GET(DT_NODELABEL(mavlink_uart));
static const struct device *mav_netif = DEVICE_DT_GET(DT_NODELABEL(mavlink_netif));

static bool armed = false;

static void rx_callback(const struct device *dev,
                         const mavlink_message_t *msg,
                         void *user_data)
{
    if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(msg, &cmd);

        if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM &&
            cmd.target_system == SYS_ID) {
            armed = (cmd.param1 > 0.5f);

            mavlink_message_t ack;
            mavlink_msg_command_ack_pack(SYS_ID, COMP_ID, &ack,
                cmd.command, MAV_RESULT_ACCEPTED,
                0, 0, msg->sysid, msg->compid);
            mavwrap_send_message(dev, &ack);
        }
    }
}

int main(void)
{
    mavwrap_set_rx_callback(mav_uart, rx_callback, NULL);
    mavwrap_set_rx_callback(mav_netif, rx_callback, NULL);

    while (1) {
        mavlink_message_t hb;
        mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &hb,
            MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
            armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0, 0,
            armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);

        mavwrap_send_message(mav_uart, &hb);
        mavwrap_send_message(mav_netif, &hb);

        k_sleep(K_MSEC(1000));
    }
}
```
