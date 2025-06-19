// Copyright (c) 2025 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include <Arduino.h>
#include <LittleFS.h>
#include <SSD1306Wire.h>
#include <og3/base-station.h>
#include <og3/constants.h>
#include <og3/ha_app.h>
#include <og3/html_table.h>
#include <og3/lora.h>
#include <og3/oled_wifi_info.h>
#include <og3/packet_reader.h>
#include <og3/satellite.pb.h>
#include <og3/shtc3.h>
#include <og3/text_buffer.h>
#include <pb_decode.h>

#include <map>

#define VERSION "0.4.2"

// TODO:
// - Move library code to og3x-satellite

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

// Have oled display IP address or AP status.
og3::OledWifiInfo wifi_infof(&s_app.tasks());

// Delay between updates of the OLED.
constexpr unsigned kOledSwitchMsec = 10 * kMsecInSec;
OledDisplayRing s_oled(&s_app.module_system(), kSoftware, kOledSwitchMsec, Oled::kTenPt,
                       Oled::Orientation::kDefault);

auto s_lora_options = []() -> LoRaModule::Options {
  LoRaModule::Options opts;
  opts.sync_word = 0xF0;
  opts.enable_crc = true;

  opts.frequency = lora::Frequency::k915MHz;
  opts.spreading_factor = lora::SpreadingFactor::kSF8;
  opts.signal_bandwidth = lora::SignalBandwidth::k125kHz;

  opts.config_options = static_cast<LoRaModule::OptionSelect>(LoRaModule::kOptionSyncWord |
                                                              LoRaModule::kOptionSpreadingFactor |
                                                              LoRaModule::kOptionSignalBandwidth);
  opts.settable_options = static_cast<LoRaModule::OptionSelect>(LoRaModule::kOptionSyncWord |
                                                                LoRaModule::kOptionSpreadingFactor |
                                                                LoRaModule::kOptionSignalBandwidth);

  return opts;
};

// Variables for lora config
VariableGroup s_lora_vg("lora");
// This variable group is for enabling/disabling MQTT for a device.
VariableGroup s_device_cvg("config");
LoRaModule s_lora(s_lora_options(), &s_app, s_lora_vg);

// Update readings only 1/minute maximum.
constexpr unsigned long kMaxMsecBetweenGardenReadings = 60 * 1000;

uint8_t s_pkt_buffer[1024];

Variable<unsigned> s_pkt_count("LoRa packet count", 0, "", "", 0, s_vg);
Variable<unsigned> s_err_prefix_count("bad prefix count", 0, "", "", 0, s_vg);
Variable<unsigned> s_err_crc_count("bad crc count", 0, "", "", 0, s_vg);
Variable<unsigned> s_err_size_count("bad size count", 0, "", "", 0, s_vg);
Variable<unsigned> s_err_version_count("bad version count", 0, "", "", 0, s_vg);

const char* str(og3_Sensor_Type val) {
  switch (val) {
    case og3_Sensor_Type_TYPE_UNSPECIFIED:
      return nullptr;
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

unsigned decimals(og3_Sensor_Type val) {
  switch (val) {
    case og3_Sensor_Type_TYPE_UNSPECIFIED:
      return 0;
    case og3_Sensor_Type_TYPE_VOLTAGE:
      return 2;
    case og3_Sensor_Type_TYPE_TEMPERATURE:
    case og3_Sensor_Type_TYPE_HUMIDITY:
    case og3_Sensor_Type_TYPE_MOISTURE:
      return 1;
    default:
      return 0;
  }
}

std::map<uint32_t, std::unique_ptr<base_station::Device>> s_id_to_device;

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
  base_station::Device* pdevice = nullptr;
  if (dev_iter != s_id_to_device.end()) {
    pdevice = dev_iter->second.get();
    pdevice->got_packet(seq_id, LoRa.packetRssi());
    s_app.log().logf("Known device id:%u %s'%s' (seq_id=%u, dropped=%u).", packet.device_id,
                     packet.has_device ? "(dev info sent) " : "", pdevice->cname(), seq_id,
                     pdevice->dropped_packets());
  } else {
    if (!packet.has_device) {
      s_app.log().logf("No known device with id=%u (seq_id=%u).", packet.device_id, seq_id);
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
        packet.device_id, new base_station::Device(packet.device_id, device.name,
                                                   device.manufacturer, &s_app.module_system(),
                                                   &s_app.ha_discovery(), seq_id, s_device_cvg));
    pdevice = iter.first->second.get();
  }

  s_pkt_count = s_pkt_count.value() + 1;

  // Read the sensor entries.
  for (unsigned idx_sensor = 0; idx_sensor < packet.sensor_count; idx_sensor++) {
    const auto& sensor = packet.sensor[idx_sensor];
    if (sensor.type == og3_Sensor_Type_TYPE_INT_NUMBER) {
      // IntSensor
      auto* psensor = pdevice->int_sensor(sensor.id);
      if (!psensor) {
        pdevice->add_int_sensor(sensor.id, sensor.name, nullptr, sensor.units, pdevice,
                                sensor.state_class);
        s_app.log().logf(
            " - set int sensor:%u (%s%s) in device:%u", sensor.id, sensor.name,
            (sensor.state_class == og3_Sensor_StateClass_STATE_CLASS_MEASUREMENT ? " measurement"
                                                                                 : ""),
            packet.device_id);
      } else {
        // TODO: update sensor entry if anything changed.
        s_app.log().logf(
            " - sensor info update :%u (%s %s%s) in device:%x", sensor.id, sensor.name,
            sensor.units,
            (sensor.state_class == og3_Sensor_StateClass_STATE_CLASS_MEASUREMENT ? " measurement"
                                                                                 : ""),
            packet.device_id);
      }
    } else {
      // FloatSensor
      base_station::FloatSensor* psensor = pdevice->float_sensor(sensor.id);
      if (!psensor) {
        pdevice->add_float_sensor(sensor.id, sensor.name, str(sensor.type), sensor.units,
                                  decimals(sensor.type), pdevice, sensor.state_class);
        s_app.log().logf(
            " - set sensor:%u (%s %s%s) in device:%x", sensor.id, sensor.name, sensor.units,
            (sensor.state_class == og3_Sensor_StateClass_STATE_CLASS_MEASUREMENT ? " measurement"
                                                                                 : ""),
            packet.device_id);
      } else {
        // TODO: update sensor entry if anything changed.
        s_app.log().logf(
            " - sensor info update :%u (%s %s%s) in device:%x", sensor.id, sensor.name,
            sensor.units,
            (sensor.state_class == og3_Sensor_StateClass_STATE_CLASS_MEASUREMENT ? " measurement"
                                                                                 : ""),
            packet.device_id);
      }
    }
  }

  // Read the float-sensor readings.
  for (unsigned idx_reading = 0; idx_reading < packet.reading_count; idx_reading++) {
    const auto& reading = packet.reading[idx_reading];
    base_station::FloatSensor* psensor = pdevice->float_sensor(reading.sensor_id);

    if (!psensor) {
      s_app.log().logf("No sensor id:%u known for device:%u.", reading.sensor_id, packet.device_id);
      continue;
    }
    psensor->value() = reading.value;
    s_app.log().logf(" %u: %s ->  %.2f", reading.sensor_id, psensor->cname(), reading.value);
    const char nc = psensor->cname()[0] ? psensor->cname()[0] : ' ';
    text.addf(" %c:%.0f", nc, reading.value);
  }
  for (unsigned idx_reading = 0; idx_reading < packet.i_reading_count; idx_reading++) {
    const auto& reading = packet.i_reading[idx_reading];
    base_station::IntSensor* psensor = pdevice->int_sensor(reading.sensor_id);

    if (!psensor) {
      s_app.log().logf("No sensor id:%u known for device:%u.", reading.sensor_id, packet.device_id);
      continue;
    }
    psensor->value() = reading.value;
    s_app.log().logf(" %u: %s ->  %d", reading.sensor_id, psensor->cname(), reading.value);
    const char nc = psensor->cname()[0] ? psensor->cname()[0] : ' ';
    text.addf(" %c:%d", nc, reading.value);
  }
  if (!pdevice->is_disabled()) {
    s_app.mqttSend(pdevice->vg());
  }
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
  html::writeTableInto(&s_html, s_lora_vg);
  html::writeTableInto(&s_html, s_device_cvg);
  s_html += HTML_BUTTON("/lora", "LoRa");
  s_html += HTML_BUTTON("/device", "Device disable");
  s_button_wifi_config.add_button(&s_html);
  s_button_mqtt_config.add_button(&s_html);
  s_button_app_status.add_button(&s_html);
  s_button_restart.add_button(&s_html);
  sendWrappedHTML(request, s_app.board_cname(), kSoftware, s_html.c_str());
}

void handleLoraConfig(AsyncWebServerRequest* request) {
  ::og3::read(*request, s_lora_vg);
  s_html.clear();
  html::writeFormTableInto(&s_html, s_device_cvg);
  s_html += HTML_BUTTON("/", "Back");
  sendWrappedHTML(request, s_app.board_cname(), kSoftware, s_html.c_str());
}

void handleDeviceConfig(AsyncWebServerRequest* request) {
  ::og3::read(*request, s_device_cvg);
  s_html.clear();
  html::writeFormTableInto(&s_html, s_device_cvg);
  s_html += HTML_BUTTON("/", "Back");
  sendWrappedHTML(request, s_app.board_cname(), kSoftware, s_html.c_str());
  s_app.config().write_config(s_lora_vg);
}

void process_lora_packets() {
  const int buffer_bytes = sizeof(s_pkt_buffer);
  const int nbytes_available = LoRa.available();
  const int nbytes = std::min(nbytes_available, buffer_bytes);
  LoRa.readBytes(s_pkt_buffer, nbytes);

  pkt::PacketReader reader(s_pkt_buffer, sizeof(s_pkt_buffer));
  switch (reader.parse()) {
    case pkt::PacketReader::ParseResult::kOk:
      break;
    case pkt::PacketReader::ParseResult::kBadPrefix:
      s_app.log().logf("Pailed to parse packet: bad prefix. %02x %02x %02x %02x.", s_pkt_buffer[0],
                       s_pkt_buffer[1], s_pkt_buffer[2], s_pkt_buffer[3]);
      s_err_prefix_count = s_err_prefix_count.value() + 1;
      return;
    case pkt::PacketReader::ParseResult::kBadCrc:
      s_app.log().logf("Failed to parse packet: Bad CRC. packet:%04x computed:%04x.",
                       reader.packet_crc(), reader.computed_crc());
      s_err_crc_count = s_err_crc_count.value() + 1;
      return;
    case pkt::PacketReader::ParseResult::kBadSize:
      s_app.log().log("Failed to parse packet: Bad size.");
      s_err_size_count = s_err_size_count.value() + 1;
      return;
    case pkt::PacketReader::ParseResult::kBadVersion:
      s_app.log().log("Failed to parse packet: Bad version.");
      s_err_version_count = s_err_version_count.value() + 1;
      return;
  }

  s_app.log().debug("Parsed packet.");
  const uint8_t* msg = nullptr;
  uint16_t msg_type = 0;
  std::size_t msg_size = 0;
  if (!reader.get_msg(0, &msg, &msg_type, &msg_size)) {
    s_app.log().log("Failed to read message from packett.");
    return;
  }

  switch (msg_type) {
    case kDevicePktType:
      parse_device_packet(reader.seq_id(), msg, msg_size);
      break;
    default:
      s_app.log().logf("Unknown message type in packet: 0x%X.", msg_type);
      break;
  }
}

// A periodic task to send basic app information every 5 minutes.
PeriodicTaskScheduler s_mqtt_scheduler(
    10 * og3::kMsecInSec, 5 * og3::kMsecInMin,
    []() {
      s_shtc3.read();
      s_app.mqttSend(s_vg);
    },
    &s_app.tasks());

}  // namespace og3

////////////////////////////////////////////////////////////////////////////////

void setup() {
  og3::s_oled.addDisplayFn([]() {
    char text[80];
    snprintf(text, sizeof(text), "%s\n%.1fC %.1fRH", og3::s_app.board_cname(),
             og3::s_shtc3.temperature(), og3::s_shtc3.humidity());
    og3::s_oled.display(text);
  });

  og3::s_app.web_server().on("/", og3::handleWebRoot);
  og3::s_app.web_server().on("/config", [](AsyncWebServerRequest* request) {});
  og3::s_app.web_server().on("/lora", og3::handleLoraConfig);
  og3::s_app.web_server().on("/device", og3::handleDeviceConfig);
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
    og3::process_lora_packets();
  }
}
