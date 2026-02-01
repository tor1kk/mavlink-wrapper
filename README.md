# MAVLink Wrapper

MAVLink communication wrapper for Zephyr. Supports UDP with runtime reconfiguration.

### Configure

**prj.conf:**
```ini
CONFIG_MAVWRAP=y
CONFIG_MAVWRAP_TRANSPORT_NETIF=y
CONFIG_NETWORKING=y
CONFIG_NET_IPV4=y
CONFIG_NET_UDP=y
CONFIG_NET_DHCPV4=y  # optional
```

**DeviceTree**

**UDP / network interface**
```dts
/ {
    mavlink: mavlink-wrapper {
        compatible = "mavlink-wrapper";
        transport = <&mac>;
        net-interface;
        net-type = "udp";
        
        use-dhcp;               // or local-ip = "192.168.1.10";
        local-port = <14550>;
        remote-ip = "192.168.1.100";
        remote-port = <14551>;
    };
};
```

**Serial interface**
```dts
mavlink: mavlink-wrapper {
    compatible = "mavlink-wrapper";
    transport = <&usart1>;
    serial-interface;
};
```

### Usage

```c
#include <mavwrap.h>

const struct device *mav = DEVICE_DT_GET(DT_NODELABEL(mavlink));

void rx_callback(const struct device *dev, const mavlink_message_t *msg, void *data) {
    printk("Got message ID: %u\n", msg->msgid);
}

void main(void) {
    mavwrap_set_rx_callback(mav, rx_callback, NULL);
    
    // Send heartbeat
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, 1, &msg, 
        MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE);
    mavwrap_send_message(mav, &msg);
}
```
