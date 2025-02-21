#include "satellite-sensor.h"

namespace og3::satellite {
namespace {
constexpr uint32_t kC133Org = 0xc133;

std::string legalize(const char* name) {
  std::string lname(name);
  for (unsigned i = 0; i < lname.length(); i++) {
    const char c = lname[i];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || (c == '_') ||
        (c == '-')) {
      continue;
    }
    lname[i] = '_';
  }
  return lname;
}

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

std::string _device_name(const char* name, uint32_t device_id) {
  char buffer[80];
  const auto len = snprintf(buffer, sizeof(buffer), "%s %x", name, device_id);
  return std::string(buffer, len);
}

std::string _device_id(const char* name, uint32_t device_id) {
  char buffer[80];
  const auto len = snprintf(buffer, sizeof(buffer), "%s_%x", name, device_id);
  return std::string(buffer, len);
}

}  // namespace

Sensor::Sensor(const char* name, const char* device_class, const char* units, Device* device)
    : m_name(legalize(name)),
      m_device_class(device_class ? device_class : ""),
      m_units(units),
      m_description(name),
      m_device(device) {}

FloatSensor::FloatSensor(const char* name, const char* device_class, const char* units,
                         unsigned decimals, Device* device)
    : Sensor(name, device_class, units, device),
      m_value(m_name.c_str(), 0.0f, m_units.c_str(), "", 0, decimals, device->vg()) {
  HADiscovery::Entry entry(m_value, ha::device_type::kSensor, device_class);
  entry.device_name = device->cname();
  entry.device_id = device->cdevice_id();
  entry.manufacturer = device->manufacturer().c_str();
  // entry.software
  // entry.model
  // entry.icon
  JsonDocument json;
  // TODO(chrishl): should bookkeep and send again if this fails.
  m_device->ha_discovery().addEntry(&json, entry);
}

IntSensor::IntSensor(const char* name, const char* device_class, const char* units, Device* device)
    : Sensor(name, device_class, units, device),
      m_value(m_name.c_str(), 0.0f, m_units.c_str(), "", 0, device->vg()) {
  HADiscovery::Entry entry(m_value, ha::device_type::kSensor, device_class);
  entry.device_name = device->cname();
  entry.device_id = device->cdevice_id();
  entry.manufacturer = device->manufacturer().c_str();
  // entry.software
  // entry.model
  // entry.icon
  JsonDocument json;
  // TODO(chrishl): should bookkeep and send again if this fails.
  m_device->ha_discovery().addEntry(&json, entry);
}

Device::Device(uint32_t device_id_num, const char* name, uint32_t mfg_id,
               ModuleSystem* module_system, HADiscovery* ha_discovery, uint16_t seq_id)
    : m_device_id_num(device_id_num),
      m_name(_device_name(name, device_id_num)),
      m_device_id(_device_id(name, device_id_num)),
      m_manufacturer(_manufacturer(mfg_id)),
      m_seq_id(seq_id),
      m_discovery(ha_discovery),
      m_vg(m_name.c_str(), m_device_id.c_str()),
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
