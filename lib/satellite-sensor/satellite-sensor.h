#pragma once

#include <og3/ha_discovery.h>
#include <og3/satellite.pb.h>
#include <og3/variable.h>

#include <map>
#include <memory>
#include <string>

namespace og3::satellite {

class Device;

class Sensor {
 public:
  Sensor(const char* name, const char* device_class, const char* units, Device* device,
         og3_Sensor_StateClass state_class);

  const std::string& name() const { return m_name; }
  const char* cname() const { return name().c_str(); }
  const std::string& device_class() const { return m_device_class; }
  const char* cdevice_class() const { return device_class().c_str(); }
  const std::string& units() const { return m_units; }
  const char* cunits() const { return units().c_str(); }
  og3_Sensor_StateClass state_class() const { return m_state_class; }

 protected:
  void addHAEntry(HADiscovery::Entry& entry);

  const std::string m_name;
  const std::string m_device_class;
  const std::string m_units;
  const std::string m_description;
  const og3_Sensor_StateClass m_state_class;
  Device* m_device;
};

class FloatSensor : public Sensor {
 public:
  FloatSensor(const char* name, const char* device_class, const char* units, unsigned decimals,
              Device* device,
              og3_Sensor_StateClass state_class = og3_Sensor_StateClass_STATE_CLASS_UNSPECIFIED);

  FloatVariable& value() { return m_value; }
  const FloatVariable& value() const { return m_value; }

 private:
  FloatVariable m_value;
};

class IntSensor : public Sensor {
 public:
  IntSensor(const char* name, const char* device_class, const char* units, Device* device,
            og3_Sensor_StateClass state_class = og3_Sensor_StateClass_STATE_CLASS_UNSPECIFIED);

  Variable<int>& value() { return m_value; }
  const Variable<int>& value() const { return m_value; }

 private:
  Variable<int> m_value;
};

class Device {
 public:
  Device(uint32_t device_id_num, const char* name, uint32_t mfg_id, ModuleSystem* module_system,
         HADiscovery* ha_discovery, uint16_t seq_id, VariableGroup& cvg);

  const std::string& name() const { return m_name; }
  const char* cname() const { return name().c_str(); }
  const std::string& device_id() const { return m_device_id; }
  const char* cdevice_id() const { return device_id().c_str(); }
  const std::string& manufacturer() const { return m_manufacturer; }
  const unsigned dropped_packets() const { return m_dropped_packets.value(); }
  bool is_disabled() const { return m_disabled.value(); }
  void set_disabled(bool disabled) { m_disabled = disabled; }

  FloatSensor* float_sensor(unsigned id) {
    auto iter = m_id_to_float_sensor.find(id);
    return (iter == m_id_to_float_sensor.end()) ? nullptr : iter->second.get();
  }
  FloatSensor* add_float_sensor(
      unsigned id, const char* name, const char* device_class, const char* units, unsigned decimals,
      Device* device,
      og3_Sensor_StateClass state_class = og3_Sensor_StateClass_STATE_CLASS_UNSPECIFIED) {
    auto iter = m_id_to_float_sensor.emplace(
        id, new FloatSensor(name, device_class, units, decimals, this, state_class));
    return iter.first->second.get();
  }
  IntSensor* int_sensor(unsigned id) {
    auto iter = m_id_to_int_sensor.find(id);
    return (iter == m_id_to_int_sensor.end()) ? nullptr : iter->second.get();
  }
  IntSensor* add_int_sensor(
      unsigned id, const char* name, const char* device_class, const char* units, Device* device,
      og3_Sensor_StateClass state_class = og3_Sensor_StateClass_STATE_CLASS_UNSPECIFIED) {
    auto iter =
        m_id_to_int_sensor.emplace(id, new IntSensor(name, device_class, units, this, state_class));
    return iter.first->second.get();
  }
  // Updates m_dropped_packets.
  void got_packet(uint16_t seq_id, int rssi);

  VariableGroup& vg() { return m_vg; }
  HADiscovery& ha_discovery() { return *m_discovery; }

 private:
  const uint32_t m_device_id_num;
  const std::string m_name;
  const std::string m_device_id;
  const std::string m_manufacturer;
  uint16_t m_seq_id;
  HADiscovery* m_discovery;
  VariableGroup m_vg;
  Variable<unsigned> m_dropped_packets;
  Variable<int> m_rssi;
  std::map<unsigned, std::unique_ptr<FloatSensor>> m_id_to_float_sensor;
  std::map<unsigned, std::unique_ptr<IntSensor>> m_id_to_int_sensor;
  std::string m_str_disabled;
  BoolVariable m_disabled;
};

}  // namespace og3::satellite
