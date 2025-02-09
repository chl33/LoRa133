#include "satellite-sensor.h"

namespace og3::satellite {
namespace {
constexpr uint32_t kC133Org = 0xc133;

std::string _manufacturer(uint32_t id) {
  switch (id) {
    case kC133Org:
      return "c133 org";
    default:
      break;
  }

  char buffer[32];
  snprintf(buffer, sizeof(buffer), "manufacturer_%04x", id);
  return buffer;
}

HADiscovery::Options _options(const std::string& device_name, const std::string& manufacturer) {
  return HADiscovery::Options(manufacturer.c_str(), "satellite", "satellite", device_name.c_str());
}
}  // namespace

Sensor::Sensor(const char* name, const char* units, Device* device)
    : m_name(name), m_units(units), m_device(device) {}

FloatSensor::FloatSensor(const char* name, const char* units, unsigned decimals, Device* device)
    : Sensor(name, units, device),
      m_value(m_name.c_str(), 0.0f, m_units.c_str(), "", 0, decimals, device->vg()) {}

Device::Device(uint32_t device_id, const char* name, uint32_t mfg_id, ModuleSystem* module_system,
               uint16_t seq_id)
    : m_device_id(device_id),
      m_name(name),
      m_manufacturer(_manufacturer(mfg_id)),
      m_seq_id(seq_id),
      m_discovery(_options(m_name, m_manufacturer), module_system),
      m_vg(m_name.c_str()),
      m_dropped_packets("dropped packets", 0, "count", "", 0, m_vg),
      m_rssi("RSSI", 0, "dB", "", 0, m_vg) {}

void Device::got_packet(uint16_t seq_id, int rssi) {
  if (seq_id > m_seq_id) {
    m_dropped_packets = m_dropped_packets.value() + static_cast<int>(seq_id) - 1 - m_seq_id;
  } else if (seq_id == m_seq_id) {
    // Not sure why this would happen (maybe not setting seq-id in the sender), so ignore it.
  } else {
    const uint16_t diff = seq_id - 1 - m_seq_id;  // difference with wrapping
    if (diff < 256) {
      m_dropped_packets = m_dropped_packets.value() + diff;
    } else {
      // Assume that the remote device reset, so sent seq_id started with 0.
      m_dropped_packets = m_dropped_packets.value() + seq_id;
    }
  }
  m_seq_id = seq_id;
  m_rssi = rssi;
}

}  // namespace og3::satellite
