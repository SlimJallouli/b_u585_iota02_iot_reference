
📦 MQTT Payload Structure
Here’s a clean JSON payload your firmware can publish:

```json
{
  "occupancy_level": 0.73,
  "raw_average_mm": 1973.6,
  "timestamp": "2025-07-23T19:22:00Z"
}
```

This gives HA clarity:

occupancy_level: normalized value (0.0–1.0)

raw_average_mm: useful for calibration and dashboards

timestamp: helpful if you retain messages briefly or graph them

You could publish this to a topic like:

```
iot/devices/<device_id>/sensors/occupancy
```
Or the simpler:

```
stm32u585-<device_sn>/tof/occupancy
```
🧠 Home Assistant MQTT Discovery Config
To register it automatically in HA, publish to:

homeassistant/sensor/<device_id>_occupancy/config
With payload:

```json
{
  "name": "Occupancy Level",
  "state_topic": "iot/devices/stm32u585-0049001B3636500A20333342/sensors/occupancy",
  "unit_of_measurement": "%",
  "value_template": "{{ (value_json.occupancy_level * 100) | round(1) }}",
  "json_attributes_topic": "iot/devices/stm32u585-0049001B3636500A20333342/sensors/occupancy",
  "unique_id": "stm32u585-0049001B3636500A20333342_occupancy_level",
  "device_class": "none",
  "entity_category": "diagnostic",
  "device": {
    "identifiers": ["stm32u585-0049001B3636500A20333342"],
    "name": "VL53L5CX ToF Sensor",
    "model": "VL53L5CX",
    "manufacturer": "STMicroelectronics",
    "sw_version": "1.0.0"
  },
  "retain": false
}
```
This will give you a clean diagnostic-style sensor that shows up in HA with percentage formatting and dynamic attributes like raw_average_mm.

Want me to extend this with zone-level publishing (like zone_mask or grid data), or wire it up to a binary motion_detected entity when occupancy crosses a threshold? It’d be slick to pair this with an automation that fades your hallway lights based on proximity intensity.

normalized proximity scale where:

    * 0 mm = 100% occupancy (someone very close)
    * 1970 mm and above = 0% occupancy (no one present)
    * Anything in-between fades linearly

Normalization Logic (C-style pseudocode)

```c
#define MAX_DISTANCE_MM 1970.0f
#define MIN_DISTANCE_MM 0.0f

float normalize_occupancy(float measured_mm) {
    if (measured_mm >= MAX_DISTANCE_MM) return 0.0f;
    if (measured_mm <= MIN_DISTANCE_MM) return 1.0f;
    return 1.0f - (measured_mm / MAX_DISTANCE_MM);
}
```
This returns a float between 0.0 and 1.0, directly suitable for occupancy visualization.

📤 MQTT Payload Example
```json
{
  "occupancy_level": 0.73,
  "raw_distance_mm": 532.8,
  "confidence": "stable",
  "timestamp": "2025-07-23T19:28:00Z"
}
```
You could add confidence from variance checks if averaging multiple frames — totally optional but neat.

🏠 Home Assistant Discovery Config
```json
{
  "name": "ToF Occupancy Intensity",
  "state_topic": "iot/devices/stm32u585-0049001B3636500A20333342/sensors/occupancy",
  "unit_of_measurement": "%",
  "value_template": "{{ (value_json.occupancy_level * 100) | round(1) }}",
  "json_attributes_topic": "iot/devices/stm32u585-0049001B3636500A20333342/sensors/occupancy",
  "unique_id": "stm32u585-0049001B3636500A20333342_tof_occupancy",
  "device_class": "none",
  "entity_category": "diagnostic",
  "retain": false,
  "device": {
    "identifiers": ["stm32u585-0049001B3636500A20333342"],
    "name": "VL53L5CX ToF Sensor",
    "model": "VL53L5CX",
    "manufacturer": "STMicroelectronics",
    "sw_version": "1.0.0"
  }
}
```

This will show up in Home Assistant as a dynamic percentage-based diagnostic sensor.