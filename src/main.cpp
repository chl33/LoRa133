// Copyright (c) 2025 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

// Copyright (c) 2024 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include <Arduino.h>
#include <LittleFS.h>
#include <og3/ha_app.h>
#include <og3/html_table.h>
#include <og3/lora.h>
#include <og3/packet_reader.h>
#include <og3/shtc3.h>
#include <pb_decode.h>

#include "device.pb.h"

#define VERSION "0.1.0"

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

// Hardware config
// Define the pins used by the transceiver module.
constexpr int kLoraSS = 5;
constexpr int kLoraRst = 14;
constexpr int kLoraDio0 = 2;

Shtc3 s_shtc3(kTemperature, kHumidity, &s_app.module_system(), "temperature", s_vg);

void _on_lora_initialized() {
  LoRa.setSpreadingFactor(12);
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
}

LoRaModule s_lora("lora", LoRaModule::Options(), &s_app, s_vg, _on_lora_initialized);

// Update readings only 1/minute maximum.
constexpr unsigned long kMaxMsecBetweenGardenReadings = 60 * 1000;

#if 0
constexpr size_t kPktHeaderSize = 12;
constexpr uint8_t kProtocolVersionMajor = 0x0;
constexpr uint8_t kProtocolVersionMinor = 0x1;

bool is_pkt(const uint8_t* buffer, size_t nbytes) {
  return nbytes >= kPktHeaderSize && buffer[0] == 'c' && buffer[1] == '3' &&
         buffer[2] == kProtocolVersionMajor && buffer[3] >= kProtocolVersionMinor;
}

class HeaderReader {
 public:
  HeaderReader() {}

  bool parse_header(const uint8_t* buffer, size_t nbytes) {
    if (!is_pkt(buffer, nbytes)) {
      return false;
    }
    m_version_major = buffer[2];
    m_version_minor = buffer[3];
    memcpy(&m_org_id, &buffer[4], sizeof(m_org_id));
    m_map_id = buffer[6];
    m_pkt_id = buffer[7];
    memcpy(&m_board_id, &buffer[8], sizeof(m_board_id));
    memcpy(&m_seq, &buffer[10], sizeof(m_seq));
    return true;
  }

  bool parse_text_msg(const uint8_t* buffer, size_t nbytes, Logger* logger) {
    constexpr uint16_t kReservedOrgId = 0x1;
    constexpr uint8_t kMapIdDebug = 0xdb;
    constexpr uint8_t kLogMsgPktId = 0x01;

    if (!parse_header(buffer, nbytes)) {
      return false;
    }
    if (m_org_id != kReservedOrgId || m_map_id != kMapIdDebug || m_pkt_id != kLogMsgPktId) {
      return false;
    }
    char msg_buffer[256];
    const int len = std::min(static_cast<int>(nbytes),
                             static_cast<int>(sizeof(msg_buffer) - kPktHeaderSize - 1));
    memcpy(msg_buffer, buffer + kPktHeaderSize, len);
    msg_buffer[len] = 0;
    logger->log(msg_buffer);
    return true;
  }

 protected:
  uint8_t m_version_major = 0xFF;
  uint8_t m_version_minor = 0xFF;
  uint16_t m_board_id = 0xFFFF;
  uint16_t m_org_id = 0xFFFF;
  uint16_t m_seq = 0xFFFF;
  uint8_t m_map_id = 0xFF;
  uint8_t m_pkt_id = 0xFF;
};

constexpr uint16_t kCleeOrg = 0xc133;
constexpr uint8_t kMapIdGarden = 0x6a;
constexpr size_t kMoisturePktSize = kPktHeaderSize + 4 * sizeof(float) + sizeof(uint16_t) +
                                    sizeof(unsigned long) + sizeof(uint32_t);

static const char kMoisture[] = "moisture";
static const char kIMoisture[] = "imoisture";
static const char kMoistureFiltered[] = "moisture_filtered";
static const char kVBattery[] = "vbattery";
static const char kVSolar[] = "vsolar";
static const char kMsec[] = "msec";
static const char kRSSI[] = "rssi";

class MoisturePacketReader : public HeaderReader {
 public:
  static constexpr uint8_t kPktId = 0x01;
  static constexpr uint8_t kProtoPktId = 0x02;

  explicit MoisturePacketReader()
      : HeaderReader()  {
#if 0
        m_moisture_writer(kMoisture, "%", readings),
        m_imoisture_writer(kIMoisture, "", readings),
        m_vbattery_writer(kVBattery, "V", readings),
        m_vsolar_writer(kVSolar, "V", readings),
        m_tempC_writer(kTemp, "°C", readings),
        m_tempF_writer(kTempF, "°F", readings),
        m_humidity_writer(kHumidity, "%", readings),
        m_msec_writer(kMsec, "ms", readings),
        m_rssi_writer(kRSSI, "dBm", readings)
#endif
    m_msg.pmoisture = -1.0f;
    m_msg.imoisture = 0;
    m_msg.vbattery = -1.0f;
    m_msg.vsolar = -1.0f;
    m_msg.tempC = -99.0f;
    m_msg.humidity = -1.0f;
    m_msg.msec = 0;

#if 0
    m_moisture_writer.addMqttConnectedCallback(
        [this](HADiscovery* had, JsonDocument* json) -> bool {
          json->clear();
          had->addRoot(json);
          const char* name = m_moisture_writer.reading().name();
          had->addMeas(name, nullptr, "dBm", 1, json);
          had->addIcon("mdi:water-percent", json);
          return had->mqttSendConfig(name, HADiscovery::kSensor, json);
        });
    m_vbattery_writer.addMqttConnectedCallback(
        [this](HADiscovery* had, JsonDocument* json) -> bool {
          json->clear();
          had->addRoot(json);
          const char* name = m_vbattery_writer.reading().name();
          had->addMeas(name, nullptr, "V", 1, json);
          had->addIcon("mdi:battery-50", json);
          return had->mqttSendConfig(name, HADiscovery::kSensor, json);
        });
    m_vsolar_writer.addMqttConnectedCallback([this](HADiscovery* had, JsonDocument* json) -> bool {
      json->clear();
      had->addRoot(json);
      const char* name = m_vsolar_writer.reading().name();
      had->addMeas(name, nullptr, "V", 1, json);
      had->addIcon("mdi:solar-50", json);
      return had->mqttSendConfig(name, HADiscovery::kSensor, json);
    });
    m_tempC_writer.addMqttConnectedCallback([this](HADiscovery* had, JsonDocument* json) -> bool {
      json->clear();
      had->addRoot(json);
      const char* name = m_tempC_writer.reading().name();
      had->addTempC(name, 1, json);
      had->addIcon("mdi:thermometer", json);
      return had->mqttSendConfig(name, HADiscovery::kSensor, json);
    });
    m_tempF_writer.addMqttConnectedCallback([this](HADiscovery* had, JsonDocument* json) -> bool {
      json->clear();
      had->addRoot(json);
      const char* name = m_tempF_writer.reading().name();
      had->addTempF(name, 1, json);
      had->addIcon("mdi:thermometer", json);
      return had->mqttSendConfig(name, HADiscovery::kSensor, json);
    });
    m_humidity_writer.addMqttConnectedCallback(
        [this](HADiscovery* had, JsonDocument* json) -> bool {
          json->clear();
          had->addRoot(json);
          const char* name = m_tempF_writer.reading().name();
          had->addHumidity(name, 1, json);
          had->addIcon("mdi:water-percent", json);
          return had->mqttSendConfig(name, HADiscovery::kSensor, json);
        });
    m_humidity_writer.addMqttConnectedCallback(
        [this](HADiscovery* had, JsonDocument* json) -> bool {
          json->clear();
          had->addRoot(json);
          const char* name = m_rssi_writer.reading().name();
          had->addHumidity(name, 1, json);
          had->addIcon("mdi:signal-cellular-2", json);
          return had->mqttSendConfig(name, HADiscovery::kSensor, json);
        });
#endif
  }

  bool parse_moisture_pkt(const uint8_t* buffer, size_t nbytes, Logger* logger) {
    if (!parse_header(buffer, nbytes) && m_org_id == kCleeOrg && m_map_id == kMapIdGarden &&
        (m_pkt_id == kPktId || m_pkt_id == kProtoPktId)) {
      logger->logf("org:%04x map:%02x pkt:%02x", static_cast<int>(m_org_id),
                   static_cast<int>(m_map_id), static_cast<int>(m_pkt_id));
      return false;
    }

    // Sometimes the garden sensor sleep doesn't work properly and we get a rapid sequence
    //  of readings, so throttle them.
    const unsigned long now_msec = millis();
    if (now_msec - m_last_reading_millis < kMaxMsecBetweenGardenReadings) {
      return false;
    }
    m_last_reading_millis = now_msec;

    if (m_pkt_id == kPktId) {
      if (nbytes != kMoisturePktSize) {
        logger->logf("Bad packet size %zu != %zu.", nbytes, kMoisturePktSize);
        return false;
      }
      memcpy(&m_msg.pmoisture, &buffer[12], sizeof(m_msg.pmoisture));
      memcpy(&m_msg.imoisture, &buffer[16], sizeof(m_msg.pmoisture));
      memcpy(&m_msg.vbattery, &buffer[18], sizeof(m_msg.vbattery));
      memcpy(&m_msg.vsolar, &buffer[18], sizeof(m_msg.vsolar));
      memcpy(&m_msg.tempC, &buffer[22], sizeof(m_msg.tempC));
      memcpy(&m_msg.humidity, &buffer[26], sizeof(m_msg.humidity));
      memcpy(&m_msg.msec, &buffer[30], sizeof(m_msg.msec));
    } else if (m_pkt_id == kProtoPktId) {
      pb_istream_t istream = pb_istream_from_buffer(buffer + kPktHeaderSize,
                                                    nbytes - kPktHeaderSize);
      m_msg = cl3_MoisturePacket_init_zero;
      if (!pb_decode(&istream, &cl3_MoisturePacket_msg, &m_msg)) {
        logger->logf(
            "Proto decode of MoisturePacket failed. "
            "org:%04x map:%02x pkt:%02x",
            static_cast<int>(m_org_id), static_cast<int>(m_map_id), static_cast<int>(m_pkt_id));
        return false;
      }
    }

#if 0
    if (m_msg.pmoisture >= 0.0f && m_msg.pmoisture <= 100.0f) {
      m_moisture_writer.set(m_msg.pmoisture);
    }
    m_imoisture_writer.set(static_cast<unsigned long>(m_msg.imoisture));
    if (m_msg.vbattery > 0.0f) {
      m_vbattery_writer.set(m_msg.vbattery);
    }
    if (m_msg.vsolar > 0.0f) {
      m_vsolar_writer.set(m_msg.vsolar);
    }
    if (m_msg.tempC > -99.0f) {
      m_tempC_writer.set(m_msg.tempC);
      m_tempF = m_msg.tempC * 9 / 5 + 32;
      m_tempF_writer.set(m_tempF);
    }
    if (m_msg.humidity >= 0.0f) {
      m_humidity_writer.set(m_msg.humidity);
    }
    m_msec_writer.set(static_cast<unsigned long>(m_msg.msec));
    m_rssi_writer.set(LoRa.packetRssi());
#endif
    return true;
  }

  void log(Logger* logger) {
    logger->logf(
        "PKT: bd:%02x seq:%u moist:%.1f%% im:%u vb:%.1f vs:%.1f T:%.1f %.1fF H:%.1f"
        " ms:%lu RSSI:%d",
        static_cast<unsigned>(m_board_id), static_cast<unsigned>(m_seq),
        static_cast<double>(m_msg.pmoisture), static_cast<unsigned>(m_msg.imoisture),
        static_cast<double>(m_msg.vbattery), static_cast<double>(m_msg.vsolar),
        static_cast<double>(m_msg.tempC), static_cast<double>(m_tempF),
        static_cast<double>(m_msg.humidity), static_cast<unsigned long>(m_msg.msec),
        LoRa.packetRssi());
  }

 protected:
  og3_Packet m_msg;
  float m_tempF = -99.0f;
  unsigned long m_last_reading_millis = 0;

#if 0
  ReadingWriter m_moisture_writer;
  ReadingWriter m_imoisture_writer;
  ReadingWriter m_vbattery_writer;
  ReadingWriter m_vsolar_writer;
  ReadingWriter m_tempC_writer;
  ReadingWriter m_tempF_writer;
  ReadingWriter m_humidity_writer;
  ReadingWriter m_msec_writer;
  ReadingWriter m_rssi_writer;
#endif
};
#endif

uint8_t s_pkt_buffer[1024];
// char s_msg_buffer[1024];
// ReadingWriter msg_writer("msg", "", &app.readings());
// ReadingWriter msg_count_writer("msg_count", "", &app.readings());
int s_pkt_count = 0;
// ReadingWriter pkt_count_writer("pkt_count", "", &app.readings());
int s_pkt_err_count = 0;
// ReadingWriter pkt_err_count_writer("pkt_err_count", "", &app.readings());

// MoisturePacketReader s_pkt_reader;  // (&s_garden_readings)

void parse_device_packet(const uint8_t* msg, std::size_t msg_size) {
  pb_istream_t istream = pb_istream_from_buffer(msg, msg_size);
  og3_Packet packet;
  if (!pb_decode(&istream, &og3_Packet_msg, &packet)) {
    s_app.log().log("Proto decode of device packet failed.");
    return;
  }

  auto show_sensor = [](const og3_Sensor& sensor) {
    s_app.log().logf(" sensor:0x%X '%s' ('%s')", sensor.id, sensor.name, sensor.units);
  };
  s_app.log().debug("Parsed device packet.");
  if (packet.has_device) {
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
  }

  for (unsigned idx_component = 0; idx_component < packet.component_count; idx_component++) {
    const auto& component = packet.component[idx_component];
    switch (component.which_kind) {
      case og3_Component_temperature_tag: {
        const auto& temp = component.kind.temperature;
        s_app.log().logf(" - temp[%u]:", idx_component);
        if (temp.has_sensor) {
          show_sensor(temp.sensor);
        }
        s_app.log().logf(" - %.2f", temp.value);
        break;
      }
      case og3_Component_humidity_tag: {
        const auto& humidity = component.kind.humidity;
        if (humidity.has_sensor) {
          show_sensor(humidity.sensor);
        }
        s_app.log().logf(" - %.2f", humidity.value);
        break;
      }
      case og3_Component_moisture_tag: {
        const auto& moisture = component.kind.moisture;
        if (moisture.has_sensor) {
          show_sensor(moisture.sensor);
        }
        s_app.log().logf(" - %.2f (%u)", moisture.percent, moisture.raw_value);
        break;
      }
      default:
        s_app.log().logf("Component %u unknown type: %u.", idx_component, component.which_kind);
        break;
    }
  }
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

#if 0
  snprintf(cl3::s_msg_buffer, sizeof(cl3::s_msg_buffer), "No message");
  cl3::msg_writer.set(cl3::s_msg_buffer);
  cl3::msg_count_writer.set(0);
  cl3::pkt_count_writer.set(cl3::s_pkt_count);
  cl3::pkt_err_count_writer.set(cl3::s_pkt_err_count);
  cl3::app.events().runIn(60 * cl3::kMsecInSec, cl3::sendStatusMqtt);
#endif
}

void loop() {
  og3::s_app.loop();

  if (!og3::s_lora.is_ok()) {
    return;
  }

  // try to parse packet
  const int packetSize = LoRa.parsePacket();
  if (!packetSize) {
    return;
  }
  og3::s_app.log().logf("Got packet: %d bytes.", packetSize);

  // received a packetg
  // Serial.print("Received packet '");

  // read packet
  while (LoRa.available()) {
    const int buffer_bytes = sizeof(og3::s_pkt_buffer);
    const int nbytes_available = LoRa.available();
    const int nbytes = std::min(nbytes_available, buffer_bytes);
    LoRa.readBytes(og3::s_pkt_buffer, nbytes);
    // og3::pkt_count_writer.set(++og3::s_pkt_count);

    og3::pkt::PacketReader reader(og3::s_pkt_buffer, sizeof(og3::s_pkt_buffer));
    const og3::pkt::PacketReader::ParseResult result = reader.parse();
    if (result != og3::pkt::PacketReader::ParseResult::kOk) {
      og3::s_app.log().logf("Failed to parse packet: result=%u.", static_cast<unsigned>(result));
    } else {
      og3::s_app.log().debug("Parsed packet.");
      // for (unsigned idx = 0; idx < reader.num_available...)
      const uint8_t* msg = nullptr;
      uint16_t msg_type = 0;
      std::size_t msg_size = 0;
      if (!reader.get_msg(0, &msg, &msg_type, &msg_size)) {
        og3::s_app.log().log("Failed to read message from packett.");
      } else {
        // constexpr uint32_t kCleeOrg = 0xc133;
        // constexpr uint16_t kDevicePktType = 0xde1c;
        switch (msg_type) {
          case og3::kDevicePktType:
            og3::parse_device_packet(msg, msg_size);
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
