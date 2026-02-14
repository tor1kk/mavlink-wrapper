# Changelog

## [0.1.0] - 2026-02-14

Initial release.

### Features
- UART transport (IRQ and DMA modes with automatic fallback)
- UDP transport (DHCP or static IP)
- Dual transport support (can use both simultaneously)
- Optional TX thread for non-blocking sends
- Runtime property changes (IP, port, etc.)
- Statistics tracking
- DeviceTree configuration

### Known limitations
- UDP transport not fully tested in production environments

[0.1.0]: https://github.com/tor1kk/mavlink-wrapper/releases/tag/v0.1.0
