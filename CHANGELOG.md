# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.7.0] - 2026-03-28

### Added
- **Satellite Persistence**: Implemented full persistence for the satellite network. The bridge now "remembers" all known devices and their sensor configurations across reboots by storing them in flash memory (`/devices.json`).
- **Cold-Start Parsing**: The bridge can now parse sensor data packets immediately after a reboot without waiting for a device description packet, thanks to restored sensor metadata.
- **Dynamic Metadata Sync**: Added support for updating device names, manufacturer IDs, and firmware versions as they arrive in LoRa packets.
- **Enhanced Telemetry**: Added RSSI, packet count, and human-readable "Last Seen" timers to the web interface.
- **Mobile-Responsive UI**: Refactored the Svelte dashboard to support vertical layouts on small screens and improved navigation.
- **Online/Offline Monitoring**: Implemented an automated background task to flag devices as offline and invalidate their sensors in Home Assistant if they miss their check-in window.

### Changed
- **og3 v0.6.1 Migration**: Ported to the latest framework core, utilizing the new `require()` dependency system and `ConfigInterface` file IO helpers.
- **Flash Optimization**: Implemented a 90-second coalescing delay for flash writes to minimize wear during high-traffic startup bursts.
- **JSON API**: Standardized all API keys to `camelCase` for cleaner Svelte integration.

## [0.6.0] - 2026-03-10

### Added
- Initial support for RadioLib and the SX1276 LoRa driver.
- Basic Svelte-based web interface for device monitoring.
- Home Assistant MQTT discovery for satellite sensors.
