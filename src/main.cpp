// Copyright (c) 2025 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include <Arduino.h>
#include <LittleFS.h>
#include <SPI.h>
#include <SSD1306Wire.h>
#include <WiFi.h>
#include <Wire.h>
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
#include <og3/web_server.h>
#include <pb_decode.h>

#include <map>

#include "og3/variable.h"
#include "svelteesp32async.h"

#define VERSION "0.6.0"

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

VariableGroup s_vg("status");

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
BoolVariable s_device_disable_default("disableDefault", false, "disable by default",
                                      VariableBase::kSettable | VariableBase::kNoPublish,
                                      s_device_cvg);

std::map<uint32_t, std::unique_ptr<base_station::Device>> s_id_to_device;

LoRaModule s_lora(s_lora_options(), &s_app, s_lora_vg);

// Periodically check if satellite devices have timed out.
PeriodicTaskScheduler s_availability_checker(
    30 * kMsecInSec, 30 * kMsecInSec,
    []() {
      for (auto& iter : s_id_to_device) {
        const auto& device = iter.second;
        if (device->isTimedOut()) {
          device->setIsOnline(false);
        }
      }
    },
    &s_app.tasks());

// Update readings only 1/minute maximum.
constexpr unsigned long kMaxMsecBetweenGardenReadings = 60 * 1000;

uint8_t s_pkt_buffer[1024];

Variable<unsigned> s_pkt_count("pktCount", 0, "", "", 0, s_vg);
Variable<unsigned> s_err_prefix_count("errPrefixCount", 0, "", "", 0, s_vg);
Variable<unsigned> s_err_crc_count("errCrcCount", 0, "", "", 0, s_vg);
Variable<unsigned> s_err_size_count("errSizeCount", 0, "", "", 0, s_vg);
Variable<unsigned> s_err_version_count("errVersionCount", 0, "", "", 0, s_vg);

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
    pdevice->got_packet(seq_id, og3::s_lora.last_rssi());
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
        packet.device_id,
        new base_station::Device(packet.device_id, device.name, device.manufacturer,
                                 device.device_type, &s_app.module_system(), &s_app.ha_discovery(),
                                 seq_id, s_device_cvg));
    pdevice = iter.first->second.get();
    pdevice->set_disabled(s_device_disable_default.value());
  }

  if (packet.has_device && packet.device.timeout_secs > 0) {
    pdevice->set_comms_timeout_millis(packet.device.timeout_secs * 1000);
  }

  s_pkt_count = s_pkt_count.value() + 1;

  // Update MQTT availability message if it applies.
  pdevice->setIsOnline(true);

  // Set all device sensor readings to "Failed" so we don't re-use values not included in
  //  this packet.
  pdevice->setAllSensorReadingsFailed();

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

static String s_body;

NetHandlerStatus apiGetWifi(NetRequest* request, NetResponse* response) {
  JsonDocument jsondoc;
  JsonObject json = jsondoc.to<JsonObject>();
  const auto& wifi = s_app.wifi_manager();
  json["board"] = wifi.board();
  json["wifiPassword"] = wifi.wifiPassword();
  json["essId"] = wifi.essId();
  json["ipAddr"] = wifi.ipAddr();
  s_body.clear();
  serializeJson(jsondoc, s_body);
  response->send(200, "application/json", s_body.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus putWifiConfig(NetRequest* request, NetResponse* response, JsonVariant& jsonIn) {
  if (!jsonIn.is<JsonObject>()) {
    response->send(500, "text/plain", "not a json object");
    NET_REPLY(request, ESP_FAIL);
  }
  JsonObject obj = jsonIn.as<JsonObject>();
  s_app.wifi_manager().variables().updateFromJson(obj);
  s_app.config().write_config(s_app.wifi_manager().variables());
  response->send(200, "text/plain", "ok");
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus apiGetMqtt(NetRequest* request, NetResponse* response) {
  JsonDocument jsondoc;
  JsonObject json = jsondoc.to<JsonObject>();
  const auto& mqtt = s_app.mqtt_manager();
  json["enabled"] = mqtt.isEnabled();
  json["hostAddr"] = mqtt.hostAddr();
  json["authPassword"] = mqtt.authPassword();
  json["authUser"] = mqtt.authUser();
  s_body.clear();
  serializeJson(jsondoc, s_body);
  response->send(200, "application/json", s_body.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus putMqttConfig(NetRequest* request, NetResponse* response, JsonVariant& jsonIn) {
  if (!jsonIn.is<JsonObject>()) {
    response->send(500, "text/plain", "not a json object");
    NET_REPLY(request, ESP_FAIL);
  }
  JsonObject obj = jsonIn.as<JsonObject>();
  s_app.mqtt_manager().variables().updateFromJson(obj);
  s_app.config().write_config(s_app.mqtt_manager().variables());
  response->send(200, "text/plain", "ok");
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus apiGetLora(NetRequest* request, NetResponse* response) {
  JsonDocument jsondoc;
  JsonObject json = jsondoc.to<JsonObject>();
  s_lora_vg.toJson(json, 0);
  s_body.clear();
  serializeJson(jsondoc, s_body);
  response->send(200, "application/json", s_body.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus putLoraConfig(NetRequest* request, NetResponse* response, JsonVariant& jsonIn) {
  if (!jsonIn.is<JsonObject>()) {
    response->send(500, "text/plain", "not a json object");
    NET_REPLY(request, ESP_FAIL);
  }
  JsonObject obj = jsonIn.as<JsonObject>();
  s_lora_vg.updateFromJson(obj);
  s_app.config().write_config(s_lora_vg);
  response->send(200, "text/plain", "ok");
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus apiGetStatus(NetRequest* request, NetResponse* response) {
  JsonDocument jsondoc;
  JsonObject json = jsondoc.to<JsonObject>();
  s_app.app_status().variables().toJson(json, 0);
  s_vg.toJson(json, 0);
  json["mqttConnected"] = s_app.mqtt_manager().isConnected();
  s_body.clear();
  serializeJson(jsondoc, s_body);
  response->send(200, "application/json", s_body.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus apiGetDevices(NetRequest* request, NetResponse* response) {
  JsonDocument jsondoc;
  JsonArray arr = jsondoc.to<JsonArray>();
  for (auto& iter : s_id_to_device) {
    const auto& device = iter.second;
    JsonObject obj = arr.add<JsonObject>();
    obj["id"] = iter.first;
    obj["name"] = device->cname();
    obj["type"] = device->cdevice_type();
    obj["disabled"] = device->is_disabled();
    obj["droppedPackets"] = device->dropped_packets();
  }
  s_body.clear();
  serializeJson(jsondoc, s_body);
  response->send(200, "application/json", s_body.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus apiPostDeviceForget(NetRequest* request, NetResponse* response) {
  if (!request->hasParam("id")) {
    response->send(400, "text/plain", "missing id");
    NET_REPLY(request, ESP_FAIL);
  }
  uint32_t id = strtoul(request->getParam("id")->value().c_str(), nullptr, 0);
  auto iter = s_id_to_device.find(id);
  if (iter == s_id_to_device.end()) {
    response->send(404, "text/plain", "not found");
    NET_REPLY(request, ESP_FAIL);
  }
  s_id_to_device.erase(iter);
  response->send(200, "application/json", "{\"isOk\":true}");
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus apiGetDevice(NetRequest* request, NetResponse* response) {
  if (!request->hasParam("id")) {
    response->send(400, "text/plain", "missing id");
    NET_REPLY(request, ESP_FAIL);
  }
  uint32_t id = strtoul(request->getParam("id")->value().c_str(), nullptr, 0);
  auto iter = s_id_to_device.find(id);
  if (iter == s_id_to_device.end()) {
    response->send(404, "text/plain", "not found");
    NET_REPLY(request, ESP_FAIL);
  }
  const auto& device = iter->second;
  JsonDocument jsondoc;
  JsonObject obj = jsondoc.to<JsonObject>();
  obj["id"] = id;
  obj["name"] = device->cname();
  obj["type"] = device->cdevice_type();
  obj["disabled"] = device->is_disabled();
  obj["droppedPackets"] = device->dropped_packets();

  JsonArray sensors = obj["sensors"].to<JsonArray>();
  for (const auto& siter : device->id_to_float_sensor()) {
    const auto& sensor = siter.second;
    JsonObject sobj = sensors.add<JsonObject>();
    sobj["id"] = siter.first;
    sobj["name"] = sensor->cname();
    sobj["units"] = sensor->cunits();
    sobj["type"] = "float";
    if (sensor->value().failed()) {
      sobj["value"] = nullptr;
    } else {
      sobj["value"] = sensor->value().value();
    }
  }
  for (const auto& siter : device->id_to_int_sensor()) {
    const auto& sensor = siter.second;
    JsonObject sobj = sensors.add<JsonObject>();
    sobj["id"] = siter.first;
    sobj["name"] = sensor->cname();
    sobj["units"] = sensor->cunits();
    sobj["type"] = "int";
    if (sensor->value().failed()) {
      sobj["value"] = nullptr;
    } else {
      sobj["value"] = sensor->value().value();
    }
  }

  s_body.clear();
  serializeJson(jsondoc, s_body);
  response->send(200, "application/json", s_body.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus apiPutDeviceConfig(NetRequest* request, NetResponse* response,
                                    JsonVariant& jsonIn) {
  if (!request->hasParam("id")) {
    response->send(400, "text/plain", "missing id");
    NET_REPLY(request, ESP_FAIL);
  }
  uint32_t id = strtoul(request->getParam("id")->value().c_str(), nullptr, 0);
  auto iter = s_id_to_device.find(id);
  if (iter == s_id_to_device.end()) {
    response->send(404, "text/plain", "not found");
    NET_REPLY(request, ESP_FAIL);
  }
  if (!jsonIn.is<JsonObject>()) {
    response->send(500, "text/plain", "not a json object");
    NET_REPLY(request, ESP_FAIL);
  }
  JsonObject obj = jsonIn.as<JsonObject>();
  iter->second->vg().updateFromJson(obj);
  response->send(200, "application/json", "{\"isOk\":true}");
  NET_REPLY(request, ESP_OK);
}

void process_lora_packets(const uint8_t* buffer, size_t nbytes) {
  pkt::PacketReader reader(buffer, nbytes);
  switch (reader.parse()) {
    case pkt::PacketReader::ParseResult::kOk:
      break;
    case pkt::PacketReader::ParseResult::kBadPrefix:
      s_app.log().logf("Failed to parse packet: bad prefix. %02x %02x %02x %02x.", buffer[0],
                       buffer[1], buffer[2], buffer[3]);
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

  initSvelteStaticFiles(&og3::s_app.web_server_module().native_server());
  og3::s_app.web_server_module().on("/api/wifi", HTTP_GET, og3::apiGetWifi);
  og3::s_app.web_server_module().on("/api/mqtt", HTTP_GET, og3::apiGetMqtt);
  og3::s_app.web_server_module().on("/api/lora", HTTP_GET, og3::apiGetLora);
  og3::s_app.web_server_module().on("/api/status", HTTP_GET, og3::apiGetStatus);
  og3::s_app.web_server_module().on("/api/devices", HTTP_GET, og3::apiGetDevices);
  og3::s_app.web_server_module().on("/api/device", HTTP_GET, og3::apiGetDevice);

  og3::s_app.web_server_module().onJson("/api/wifi", HTTP_PUT, og3::putWifiConfig);
  og3::s_app.web_server_module().onJson("/api/mqtt", HTTP_PUT, og3::putMqttConfig);
  og3::s_app.web_server_module().onJson("/api/lora", HTTP_PUT, og3::putLoraConfig);
  og3::s_app.web_server_module().onJson("/api/device/config", HTTP_PUT, og3::apiPutDeviceConfig);
  og3::s_app.web_server_module().on("/api/device/forget", HTTP_POST, og3::apiPostDeviceForget);
  og3::s_app.web_server_module().on("/api/restart", HTTP_POST,
                                    [](og3::NetRequest* request, og3::NetResponse* response) {
                                      response->send(200, "text/plain", "restarting");
                                      og3::s_app.tasks().runIn(1000, []() { ESP.restart(); });
                                      NET_REPLY(request, ESP_OK);
                                    });

  og3::s_app.setup();
}

void loop() {
  og3::s_app.loop();

  if (!og3::s_lora.is_ok()) {
    return;
  }

  // Try to parse a packet.
  int packetSize = og3::s_lora.poll_packet(og3::s_pkt_buffer, sizeof(og3::s_pkt_buffer));
  if (packetSize > 0) {
    // Received a packet.
    og3::s_app.log().logf("Got packet: %d bytes (RSSI: %.1f, SNR: %.1f).", packetSize,
                          og3::s_lora.last_rssi(), og3::s_lora.last_snr());
    og3::process_lora_packets(og3::s_pkt_buffer, packetSize);
  }
}
