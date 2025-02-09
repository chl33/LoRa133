# LoRA133

LoRA packet server.

## TODO

- [ ] Accounting for number of packets received.
- [ ] Document protocol.
- [ ] Register kinds of devices.
    - [ ] Device info (for HA discovery device description)
    - [ ] Sensor info (multiple sensors for a device)
        - [ ] HA device type and class.
        - [ ] HA MQTT discovery including making device entry.
        - [ ] Protobuf message definition for LoRa data.
	- [x] Sensor classes to define
        - [x] Moisture sensor
        - [x] Temperature/humidity sensor
        - [ ] Voltage sensor
    - [ ] Maybe move all this stuff to a og3x-lora-satellite repository/library
- [ ] Lifecycle
    - [ ] Listen for device announcements
    - [ ] On device announcement:
      - [ ] Create device record (pre-allocate N records)
      - [ ] Remove device not heard from the longest if necessary.
      - [ ] Write MQTT discovery records
      - [ ] Write device records to flash a few minutes after a new one is created.
    - [ ] On sensor packet:
        - [ ] Decode proto
        - [ ] Send MQTT message with sensor info
- [ ] Web interface shows info about all devices and sensors.
