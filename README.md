# LoRA133

LoRA packet server.

## TODO

- [x] Accounting for number of packets received.
- [ ] Document protocol.
- [x] Register kinds of devices.
    - [x] Device info (for HA discovery device description)
    - [x] Sensor info (multiple sensors for a device)
        - [x] HA device type and class.
        - [ ] HA MQTT discovery including making device entry.
        - [x] Protobuf message definition for LoRa data.
	- [x] Sensor classes to define
        - [x] Moisture sensor
        - [x] Temperature/humidity sensor
        - [x] Voltage sensor
    - [ ] Maybe move all this stuff to a og3x-lora-satellite repository/library
- [ ] Lifecycle
    - [x] Listen for device announcements
    - [ ] On device announcement:
      - [x] Create device record (pre-allocate N records)
      - [ ] Remove device not heard from the longest if necessary.
      - [ ] Write MQTT discovery records
      - [ ] Write device records to flash a few minutes after a new one is created.
    - [x] On sensor packet:
        - [x] Decode proto
        - [x] Send MQTT message with sensor info
- [x] Web interface shows info about all devices and sensors.
