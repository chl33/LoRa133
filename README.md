# LoRA133

LoRA packet server.

## TODO

- [ ] Document protocol.
- [ ] Maybe move satelite library to a library og3x-satellite -sensor(?)
- [ ] Lifecycle
    - [ ] On device announcement:
      - [ ] Remove device not heard from the longest if necessary.
      - [ ] Write MQTT discovery records
      - [ ] Write device records to flash a few minutes after a new one is created.

HA Discovery

- HADiscovery module needs to be able to register multiple devices.
  - addRoot() should only use Entry information, not other info.
     Otherwise, it cannot register foreign (e.g., Satellite) devices.
- Don't make a discovery module for each device in the satellite system.
