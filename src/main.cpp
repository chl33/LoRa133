// Copyright (c) 2025 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include <Arduino.h>
#include <LittleFS.h>
#include <SSD1306Wire.h>
#include <og3/ha_app.h>
#include <og3/html_table.h>
#include <og3/lora.h>
#include <og3/oled_wifi_info.h>
#include <og3/packet_reader.h>
#include <og3/shtc3.h>
#include <og3/text_buffer.h>
#include <pb_decode.h>

#include <map>

#include "device.pb.h"
#include "satellite-sensor.h"

#define VERSION "0.2.0"

// TODO:
// - Packet indicates debug mode

namespace og3 {

static const char kManufacturer[] = "Chris Lee";
static const char kModel[] = "LoRa133";
static const char kSoftware[] = "LoRa133 v" VERSION;

constexpr uint32_t kCleeOrg = 0xc133;
constexpr uint16_t kDevicePktType = 0xde1c;

#if defined(LOG_UDP) && defined(LOG_UDP_ADDRESS)
constexpr App::LogType kLogType = App::LogType::kUdp;
#else
// constexpr App::LogType kLogType = App::LogType::kNone;  // kSerial
constexpr App::LogType kLogType = App::LogType::kSerial;
#endif

HAApp s_app(HAApp::Options(kManufacturer, kModel,
                           WifiApp::Options()
                               .withSoftwareName(kSoftware)
                               .withDefaultDeviceName("rooml33")
#if defined(LOG_UDP) && defined(LOG_UDP_ADDRESS)
                               .withUdpLogHost(IPAddress(LOG_UDP_ADDRESS))
#endif
                               .withOta(OtaManager::Options(OTA_PASSWORD))
                               .withApp(App::Options().withLogType(kLogType))));

VariableGroup s_cvg("lora_cfg");
VariableGroup s_vg("lora");

static const char kTemperature[] = "temperature";
static const char kHumidity[] = "humidity";

// Id needs to be included in device name.
// Hardware config
// Define the pins used by the transceiver module.
constexpr int kLoraSS = 5;
constexpr int kLoraRst = 14;
constexpr int kLoraDio0 = 2;

Shtc3 s_shtc3(kTemperature, kHumidity, &s_app.module_system(), "temperature", s_vg);

// Delay between updates of the OLED.
constexpr unsigned kOledSwitchMsec = 5000;
OledDisplayRing s_oled(&s_app.module_system(), kSoftware, kOledSwitchMsec, Oled::kTenPt,
                       Oled::Orientation::kDefault);

void _on_lora_initialized() {
  LoRa.setSpreadingFactor(12);
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  LoRa.enableCrc();
}

LoRaModule s_lora("lora", LoRaModule::Options(), &s_app, s_vg, _on_lora_initialized);

// Update readings only 1/minute maximum.
constexpr unsigned long kMaxMsecBetweenGardenReadings = 60 * 1000;

uint8_t s_pkt_buffer[1024];

Variable<unsigned> s_pkt_count("LoRa packet count", 0, "", "", 0, s_vg);

const char* str(og3_Sensor_Type val) {
  switch (val) {
    case og3_Sensor_Type_TYPE_UNSPECIFIED:
      return "unspecified";
    case og3_Sensor_Type_TYPE_VOLTAGE:
      return ha::device_class::sensor::kVoltage;
    case og3_Sensor_Type_TYPE_TEMPERATURE:
      return ha::device_class::sensor::kTemperature;
    case og3_Sensor_Type_TYPE_HUMIDITY:
      return ha::device_class::sensor::kHumidity;
    case og3_Sensor_Type_TYPE_MOISTURE:
      return ha::device_class::sensor::kMoisture;
    default:
      return "???";
  }
}

std::map<uint32_t, std::unique_ptr<satellite::Device>> s_id_to_device;

void parse_device_packet(uint16_t seq_id, const uint8_t* msg, std::size_t msg_size) {
  pb_istream_t istream = pb_istream_from_buffer(msg, msg_size);
  og3_Packet packet;
  if (!pb_decode(&istream, &og3_Packet_msg, &packet)) {
    s_app.log().log("Proto decode of device packet failed.");
    return;
  }

  s_app.log().debugf("Parsed device packet (seq_id:%u).", seq_id);

  TextBuffer<64> text;
  text.addf("pkt:%0u dev:%x", seq_id, packet.device_id);

  auto dev_iter = s_id_to_device.find(packet.device_id);
  satellite::Device* pdevice = nullptr;
  if (dev_iter != s_id_to_device.end()) {
    pdevice = dev_iter->second.get();
    pdevice->got_packet(seq_id, LoRa.packetRssi());
    s_app.log().logf("Known device id:%u '%s' (seq_id=%u, dropped=%u).", packet.device_id,
                     pdevice->cname(), seq_id, pdevice->dropped_packets());
  } else {
    if (!packet.has_device) {
      s_app.log().logf("No known device with id=%u.", packet.device_id);
      return;
    }
    const auto& device = packet.device;
    s_app.log().logf("Device: id:0x%X mfg:0x%X '%s'", device.id, device.manufacturer, device.name);
    if (device.has_hardware_version) {
      const auto& ver = device.hardware_version;
      s_app.log().logf(" hw: %u.%u.%u", ver.major, ver.minor, ver.patch);
    }
    if (device.has_software_version) {
      const auto& ver = device.software_version;
      s_app.log().logf(" sw: %u.%u.%u", ver.major, ver.minor, ver.patch);
    }
    auto iter = s_id_to_device.emplace(
        packet.device_id,
        new satellite::Device(packet.device_id, device.name, device.manufacturer,
                              &s_app.module_system(), &s_app.ha_discovery(), seq_id));
    pdevice = iter.first->second.get();
  }

  s_pkt_count = s_pkt_count.value() + 1;

  for (unsigned idx_reading = 0; idx_reading < packet.reading_count; idx_reading++) {
    const auto& reading = packet.reading[idx_reading];
    satellite::FloatSensor* psensor = pdevice->float_sensor(reading.sensor_id);

    if (!psensor) {
      if (!reading.has_sensor) {
        s_app.log().logf("No sensor id:%u known for device:%u.", reading.sensor_id,
                         packet.device_id);
        continue;
      } else {
        constexpr unsigned kDecimals = 1;  // Depend on type???
        psensor = pdevice->add_float_sensor(reading.sensor_id, reading.sensor.name,
                                            str(reading.sensor.type), reading.sensor.units,
                                            kDecimals, pdevice);
        s_app.log().logf(" - set sensor:%u (%s) in device:%x", reading.sensor_id,
                         reading.sensor.name, packet.device_id);
      }
    }
    psensor->value() = reading.value;
    s_app.log().logf(" %u: %s ->  %.2f", reading.sensor_id, psensor->cname(), reading.value);
    const char nc = psensor->cname()[0] ? psensor->cname()[0] : ' ';
    text.addf(" %c:%.0f", nc, reading.value);
  }
  for (unsigned idx_reading = 0; idx_reading < packet.i_reading_count; idx_reading++) {
    const auto& reading = packet.i_reading[idx_reading];
    satellite::IntSensor* psensor = pdevice->int_sensor(reading.sensor_id);

    if (!psensor) {
      if (!reading.has_sensor) {
        s_app.log().logf("No sensor id:%u known for device:%u.", reading.sensor_id,
                         packet.device_id);
        continue;
      } else {
        psensor = pdevice->add_int_sensor(reading.sensor_id, reading.sensor.name, nullptr,
                                          reading.sensor.units, pdevice);
        s_app.log().logf(" - set sensor:%u (%s) in device:%u", reading.sensor_id,
                         reading.sensor.name, packet.device_id);
      }
    }
    psensor->value() = reading.value;
    s_app.log().logf(" %u: %s ->  %d", reading.sensor_id, psensor->cname(), reading.value);
    const char nc = psensor->cname()[0] ? psensor->cname()[0] : ' ';
    text.addf(" %c:%d", nc, reading.value);
  }
  s_app.mqttSend(pdevice->vg());
  text.split(22);
  s_oled.display(text.text());
}

// Global variable for html, so asyncwebserver can send data in the background (single client)
String s_html;

WebButton s_button_wifi_config = s_app.createWifiConfigButton();
WebButton s_button_mqtt_config = s_app.createMqttConfigButton();
WebButton s_button_app_status = s_app.createAppStatusButton();
WebButton s_button_restart = s_app.createRestartButton();

void handleWebRoot(AsyncWebServerRequest* request) {
  s_html.clear();
  html::writeTableInto(&s_html, s_vg);
  for (auto& iter : s_id_to_device) {
    const auto& device = iter.second;
    html::writeTableInto(&s_html, device->vg());
  }
  html::writeTableInto(&s_html, s_app.wifi_manager().variables());
  html::writeTableInto(&s_html, s_app.mqtt_manager().variables());
  s_button_wifi_config.add_button(&s_html);
  s_button_mqtt_config.add_button(&s_html);
  s_button_app_status.add_button(&s_html);
  s_button_restart.add_button(&s_html);
  sendWrappedHTML(request, s_app.board_cname(), kSoftware, s_html.c_str());
}

}  // namespace og3

////////////////////////////////////////////////////////////////////////////////

void setup() {
  og3::s_app.web_server().on("/", og3::handleWebRoot);
  og3::s_app.web_server().on("/config", [](AsyncWebServerRequest* request) {});
  og3::s_app.setup();
}

void loop() {
  og3::s_app.loop();

  if (!og3::s_lora.is_ok()) {
    return;
  }

  // Try to parse a packet.
  const int packetSize = LoRa.parsePacket();
  if (!packetSize) {
    return;
  }

  // Received a packet.
  og3::s_app.log().logf("Got packet: %d bytes.", packetSize);

  // read packet
  while (LoRa.available()) {
    const int buffer_bytes = sizeof(og3::s_pkt_buffer);
    const int nbytes_available = LoRa.available();
    const int nbytes = std::min(nbytes_available, buffer_bytes);
    LoRa.readBytes(og3::s_pkt_buffer, nbytes);

    og3::pkt::PacketReader reader(og3::s_pkt_buffer, sizeof(og3::s_pkt_buffer));
    const og3::pkt::PacketReader::ParseResult result = reader.parse();
    if (result != og3::pkt::PacketReader::ParseResult::kOk) {
      og3::s_app.log().logf("Failed to parse packet: result=%u.", static_cast<unsigned>(result));
      if (result == og3::pkt::PacketReader::ParseResult::kBadPrefix) {
        og3::s_app.log().logf("Prefix: %02x %02x %02x %02x.", og3::s_pkt_buffer[0],
                              og3::s_pkt_buffer[1], og3::s_pkt_buffer[2], og3::s_pkt_buffer[3]);
      }
    } else {
      og3::s_app.log().debug("Parsed packet.");
      // for (unsigned idx = 0; idx < reader.num_available...)
      const uint8_t* msg = nullptr;
      uint16_t msg_type = 0;
      std::size_t msg_size = 0;
      if (!reader.get_msg(0, &msg, &msg_type, &msg_size)) {
        og3::s_app.log().log("Failed to read message from packett.");
      } else {
        switch (msg_type) {
          case og3::kDevicePktType:
            og3::parse_device_packet(reader.seq_id(), msg, msg_size);
            break;
          default:
            og3::s_app.log().logf("Unknown message type in packet: 0x%X.", msg_type);
            break;
        }
      }
    }

    // print RSSI of packet
    // Serial.print("' with RSSI ");
    // Serial.println(LoRa.packetRssi());
  }
}
