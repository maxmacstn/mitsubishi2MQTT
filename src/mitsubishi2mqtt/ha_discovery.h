/*
  ha_discovery.h - Home Assistant MQTT discovery payload builders.

  Each fill*() function writes one entity's discovery config into a caller
  supplied JsonDocument, so the firmware can reuse a single document for all
  entities (peak heap ~3 KB instead of one large document per entity).
  Header-only so the host tests can check that no payload overflows.
*/
#pragma once

#include <ArduinoJson.h>
#include <math.h>

namespace m2m
{

// Capacity of the shared discovery document. Checked by the host tests
// against worst-case (64-character) names and topics.
static const size_t HA_DISCOVERY_DOC_SIZE = 3072;

struct HaDiscoveryContext
{
  String id;          // unique id prefix (last MAC bytes / chip id)
  String deviceName;  // mqtt_fn
  String swVersion;   // "Mitsubishi2MQTT <version>"
  String hwVersion;
  String configUrl;   // http://<ip>

  bool availabilityReport = true;
  String availabilityTopic;
  const char *payloadAvailable = "online";
  const char *payloadNotAvailable = "offline";

  bool supportHeatMode = true;
  bool useFahrenheit = false;
  float minTempC = 16;
  float maxTempC = 31;
  String tempStep = "1";

  String powerSetTopic;
  String modeSetTopic;
  String tempSetTopic;
  String fanSetTopic;
  String vaneSetTopic;
  String wideVaneSetTopic;
  String stateTopic;
  String energySetTopic;
  String ledSetTopic;
  String beepSetTopic;
  String unitSettingsTopic;
};

inline float haLocalTemp(const HaDiscoveryContext &c, float celsius)
{
  // Same conversion as convertCelsiusToLocalUnit() in the sketch.
  return c.useFahrenheit ? (float)round(1.8 * celsius + 32.0) : celsius;
}

inline String haLocalTempStr(const HaDiscoveryContext &c, float celsius)
{
  return String(haLocalTemp(c, celsius));
}

// Device block plus (optionally) availability, shared by every entity so
// that all of them go "unavailable" when the device drops off MQTT.
inline void haAddDeviceInfo(JsonDocument &doc, const HaDiscoveryContext &c)
{
  JsonObject dev = doc.createNestedObject("device");
  dev["ids"] = c.deviceName;
  dev["name"] = c.deviceName;
  dev["sw"] = c.swVersion;
  dev["mdl"] = "HVAC MITSUBISHI";
  dev["mf"] = "MITSUBISHI ELECTRIC";
  dev["hw"] = c.hwVersion;
  dev["cu"] = c.configUrl;

  if (c.availabilityReport)
  {
    doc["avty_t"] = c.availabilityTopic;
    doc["pl_avail"] = c.payloadAvailable;
    doc["pl_not_avail"] = c.payloadNotAvailable;
  }
}

inline String haCurrentTempTemplate(const HaDiscoveryContext &c)
{
  String s = F("{{ value_json.roomTemperature if (value_json is defined and value_json.roomTemperature is defined and value_json.roomTemperature|int > ");
  s += haLocalTempStr(c, 1) + ") else '" + haLocalTempStr(c, 26) + "' }}";
  return s;
}

inline void haFillClimate(JsonDocument &doc, const HaDiscoveryContext &c, const char *icon)
{
  doc.clear();
  doc["name"] = nullptr;
  doc["unique_id"] = c.id;
  doc["icon"] = icon;

  JsonArray modes = doc.createNestedArray("modes");
  modes.add("heat_cool"); // native AUTO mode
  modes.add("cool");
  modes.add("dry");
  if (c.supportHeatMode)
  {
    modes.add("heat");
  }
  modes.add("fan_only"); // native FAN mode
  modes.add("off");

  doc["mode_cmd_t"] = c.modeSetTopic;
  doc["mode_stat_t"] = c.stateTopic;
  doc["mode_stat_tpl"] = F("{{ value_json.mode if (value_json is defined and value_json.mode is defined and value_json.mode|length) else 'off' }}");
  doc["temp_cmd_t"] = c.tempSetTopic;
  doc["temp_stat_t"] = c.stateTopic;

  String minS = haLocalTempStr(c, c.minTempC);
  String maxS = haLocalTempStr(c, c.maxTempC);
  String tpl = F("{% if (value_json is defined and value_json.temperature is defined) %}{% if (value_json.temperature|int > ");
  tpl += minS + " and value_json.temperature|int < ";
  tpl += maxS + ") %}{{ value_json.temperature }}";
  tpl += "{% elif (value_json.temperature|int < " + minS + ") %}" + minS +
         "{% elif (value_json.temperature|int > " + maxS + ") %}" + maxS +
         "{% endif %}{% else %}" + haLocalTempStr(c, 22) + "{% endif %}";
  doc["temp_stat_tpl"] = tpl;
  doc["curr_temp_t"] = c.stateTopic;
  doc["curr_temp_tpl"] = haCurrentTempTemplate(c);
  doc["min_temp"] = haLocalTemp(c, c.minTempC);
  doc["max_temp"] = haLocalTemp(c, c.maxTempC);
  doc["temp_step"] = c.tempStep;
  doc["pow_cmd_t"] = c.powerSetTopic;
  doc["temperature_unit"] = c.useFahrenheit ? "F" : "C";

  JsonArray fanModes = doc.createNestedArray("fan_modes");
  fanModes.add("AUTO");
  fanModes.add("QUIET");
  fanModes.add("1");
  fanModes.add("2");
  fanModes.add("3");
  fanModes.add("4");

  doc["fan_mode_cmd_t"] = c.fanSetTopic;
  doc["fan_mode_stat_t"] = c.stateTopic;
  doc["fan_mode_stat_tpl"] = F("{{ value_json.fan if (value_json is defined and value_json.fan is defined and value_json.fan|length) else 'AUTO' }}");

  JsonArray swingModes = doc.createNestedArray("swing_modes");
  swingModes.add("AUTO");
  swingModes.add("1");
  swingModes.add("2");
  swingModes.add("3");
  swingModes.add("4");
  swingModes.add("5");
  swingModes.add("SWING");

  doc["swing_mode_cmd_t"] = c.vaneSetTopic;
  doc["swing_mode_stat_t"] = c.stateTopic;
  doc["swing_mode_stat_tpl"] = F("{{ value_json.vane if (value_json is defined and value_json.vane is defined and value_json.vane|length) else 'AUTO' }}");
  doc["action_topic"] = c.stateTopic;
  doc["action_template"] = F("{{ value_json.action if (value_json is defined and value_json.action is defined and value_json.action|length) else 'idle' }}");

  haAddDeviceInfo(doc, c);
}

inline void haFillRoomTemp(JsonDocument &doc, const HaDiscoveryContext &c, const char *icon)
{
  doc.clear();
  doc["name"] = "Room temperature";
  doc["unique_id"] = c.id + "_room_temp";
  doc["icon"] = icon;
  doc["unit_of_measurement"] = c.useFahrenheit ? "°F" : "°C";
  doc["device_class"] = "temperature";
  doc["state_class"] = "measurement";
  doc["state_topic"] = c.stateTopic;
  doc["value_template"] = haCurrentTempTemplate(c);
  haAddDeviceInfo(doc, c);
}

inline void haFillPower(JsonDocument &doc, const HaDiscoveryContext &c, const char *icon)
{
  doc.clear();
  doc["name"] = "Power";
  doc["unique_id"] = c.id + "_power";
  doc["icon"] = icon;
  doc["unit_of_measurement"] = "W";
  doc["device_class"] = "power";
  doc["state_class"] = "measurement";
  doc["state_topic"] = c.stateTopic;
  doc["value_template"] = F("{{ value_json.power if (value_json is defined and value_json.power is defined and value_json.power|int >= 0) else '' }}");
  haAddDeviceInfo(doc, c);
}

inline void haFillEnergy(JsonDocument &doc, const HaDiscoveryContext &c, const char *icon)
{
  doc.clear();
  doc["name"] = "Energy";
  doc["unique_id"] = c.id + "_energy";
  doc["icon"] = icon;
  doc["unit_of_measurement"] = "kWh";
  doc["device_class"] = "energy";
  doc["state_topic"] = c.stateTopic;
  doc["value_template"] = F("{{ value_json.energy if (value_json is defined and value_json.energy is defined and value_json.energy|int >= 0) else '' }}");
  doc["state_class"] = "total_increasing";
  doc["suggested_display_precision"] = 1;
  haAddDeviceInfo(doc, c);
}

inline void haFillEnergyReset(JsonDocument &doc, const HaDiscoveryContext &c, const char *icon)
{
  doc.clear();
  doc["name"] = "Energy Reset";
  doc["unique_id"] = c.id + "_energy_reset";
  doc["icon"] = icon;
  doc["command_topic"] = c.energySetTopic;
  doc["entity_category"] = "config";
  doc["payload_press"] = "0";
  haAddDeviceInfo(doc, c);
}

inline void haFillVaneVertical(JsonDocument &doc, const HaDiscoveryContext &c, const char *icon)
{
  doc.clear();
  doc["name"] = "Vane Vertical";
  doc["unique_id"] = c.id + "_vane_vertical";
  doc["icon"] = icon;
  doc["state_topic"] = c.stateTopic;
  doc["value_template"] = F("{{ value_json.vane if (value_json is defined and value_json.vane is defined and value_json.vane|length) else 'AUTO' }}");
  doc["command_topic"] = c.vaneSetTopic;
  JsonArray options = doc.createNestedArray("options");
  options.add("AUTO");
  options.add("1");
  options.add("2");
  options.add("3");
  options.add("4");
  options.add("5");
  options.add("SWING");
  haAddDeviceInfo(doc, c);
}

inline void haFillVaneHorizontal(JsonDocument &doc, const HaDiscoveryContext &c, const char *icon)
{
  doc.clear();
  doc["name"] = "Vane Horizontal";
  doc["unique_id"] = c.id + "_vane_horizontal";
  doc["icon"] = icon;
  doc["state_topic"] = c.stateTopic;
  doc["value_template"] = F("{{ value_json.wideVane if (value_json is defined and value_json.wideVane is defined and value_json.wideVane|length) else 'SWING' }}");
  doc["command_topic"] = c.wideVaneSetTopic;
  JsonArray options = doc.createNestedArray("options");
  options.add("<<");
  options.add("<");
  options.add("|");
  options.add(">");
  options.add(">>");
  options.add("SWING");
  haAddDeviceInfo(doc, c);
}

inline void haFillLedSwitch(JsonDocument &doc, const HaDiscoveryContext &c, const char *icon)
{
  doc.clear();
  doc["name"] = "LED";
  doc["unique_id"] = c.id + "_unit_led";
  doc["icon"] = icon;
  doc["command_topic"] = c.ledSetTopic;
  doc["entity_category"] = "config";
  doc["state_topic"] = c.unitSettingsTopic;
  doc["value_template"] = F("{{ value_json.led if (value_json is defined and value_json.led is defined and value_json.led|length) else 'ON' }}");
  haAddDeviceInfo(doc, c);
}

inline void haFillBeepSwitch(JsonDocument &doc, const HaDiscoveryContext &c, const char *icon)
{
  doc.clear();
  doc["name"] = "Beep";
  doc["unique_id"] = c.id + "_unit_beep";
  doc["icon"] = icon;
  doc["command_topic"] = c.beepSetTopic;
  doc["entity_category"] = "config";
  doc["state_topic"] = c.unitSettingsTopic;
  doc["value_template"] = F("{{ value_json.beep if (value_json is defined and value_json.beep is defined and value_json.beep|length) else 'ON' }}");
  haAddDeviceInfo(doc, c);
}

} // namespace m2m
