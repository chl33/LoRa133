#pragma once

#include <og3/ha_discovery.h>
#include <og3/variable.h>

#include <map>
#include <memory>
#include <string>

namespace og3::satellite {

class Device;

class Sensor {
 public:
  Sensor(const char* name, const char* units, Device* device);

  const std::string& name() const { return m_name; }
  const char* cname() const { return name().c_str(); }

 protected:
  const std::string m_name;
  const std::string m_units;
  Device* m_device;
};

class FloatSensor : public Sensor {
 public:
  FloatSensor(const char* name, const char* units, unsigned decimals, Device* device);

  FloatVariable& value() { return m_value; }
  const FloatVariable& value() const { return m_value; }

 private:
  FloatVariable m_value;
};

class Device {
 public:
  Device(uint32_t device_id, const char* name, uint32_t mfg_id, ModuleSystem* module_system,
         uint16_t seq_id);

  const std::string& name() const { return m_name; }
  const char* cname() const { return name().c_str(); }
  const std::string& manufacturer() const { return m_manufacturer; }
  const unsigned dropped_packets() const { return m_dropped_packets; }

  FloatSensor* float_sensor(unsigned id) {
    auto iter = m_id_to_float_sensor.find(id);
    return (iter == m_id_to_float_sensor.end()) ? nullptr : iter->second.get();
  }
  FloatSensor* add_float_sensor(unsigned id, const char* name, const char* units, unsigned decimals,
                                Device* device) {
    auto iter = m_id_to_float_sensor.emplace(id, new FloatSensor(name, units, decimals, this));
    return iter.first->second.get();
  }
  // Updates m_dropped_packets.
  void got_packet(uint16_t seq_id);

  VariableGroup& vg() { return m_vg; }

 private:
  const uint32_t m_device_id;
  const std::string m_name;
  const std::string m_manufacturer;
  uint16_t m_seq_id;
  HADiscovery m_discovery;
  VariableGroup m_vg;
  std::map<unsigned, std::unique_ptr<FloatSensor>> m_id_to_float_sensor;
  unsigned m_dropped_packets = 0;
};

}  // namespace og3::satellite
