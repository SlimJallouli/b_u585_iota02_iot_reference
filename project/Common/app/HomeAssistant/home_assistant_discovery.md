# MQTT Auto-Discovery with Home Assistant

This document describes the MQTT topic structure and Home Assistant discovery configuration for your STM32-based IoT device.

---

## 1. Home Assistant MQTT Discovery Topics
Home Assistant uses MQTT discovery to automatically register devices and sensors. Each discovery message must be published to:

```
homeassistant/< component >/< device_id >_< sensor >/config
```

## 2. Device Identity

All STM32 device IDs follow the pattern: `stm32x123-< serial_number >`

Example:

- Without STSAFE : `stm32h573-002C005B3332511738363236`
- With STSAFEA-110: `eval3-0209203A825AD42AC20139`
- With STSAFEA-120: `eval5-0209203A825AD42AC20139`
- With STSAFEA-TPM: `ST1-TPM-TCA01-EBD60101EE7B88`
---


## 3. Home Assistant MQTT Bridge to AWS IoT Core with Auto Discovery

This guide outlines the steps to configure Home Assistant (HA) to connect to AWS IoT Core via Mosquitto and enable MQTT discovery for IoT devices.

### 3.1. Prerequisites

- Home Assistant running with File Editor and Mosquitto Broker add-ons installed
- AWS IoT Core configured with:
  - Device certificates
  - MQTT topics for config and state
- Devices publishing retained MQTT discovery messages

---

### 3.2. Step 1: Install File Editor Add-on

1. Go to **Settings → Add-ons → Add-on Store**
2. Search for **File Editor** and install it
3. Under the **Configuration** tab:
   - Disable `enforce_basepath` to allow editing any file
4. Under the **Info** tab:
   - Enable `Start on boot`
   - Enable `Show in sidebar`
   - Click **Start**

---

### 3.3. Step 2: Install Mosquitto Broker Add-on

1. Go to **Settings → Add-ons → Add-on Store**
2. Search for **Mosquitto Broker** and install it
3. Under the **Configuration** tab:
   - Enable `Customize configuration`
   - This allows Mosquitto to load configs from `/share/mosquitto`
4. Under the **Info** tab:
   - Enable `Start on boot`
   - Click **Start**

---

### 3.4. Step 3: Create Mosquitto Bridge Config

1. Open **File Editor**
2. Navigate to `/share`
3. Create a folder named `mosquitto`
4. Inside `mosquitto`, create a file named `aws_bridge.conf`
5. Paste the following configuration:

```ini
connection aws_bridge
address <your-aws-endpoint>:8883
clientid home-assistant-bridge

bridge_cafile /ssl/AmazonRootCA1.pem
bridge_certfile /ssl/certificate.pem.crt
bridge_keyfile /ssl/private.pem.key

# Allow bidirectional traffic for all device topics
topic +/# both 0

# Allow bidirectional traffic for Home Assistant discovery
topic homeassistant/# both 0

start_type automatic
try_private false
notifications false
```

>***Replace `< your-aws-endpoint >` with your actual AWS IoT Core endpoint.***

### Step 4: Restart Mosquitto Broker
* Go to **Settings → Add-ons → Mosquitto Broker**

1. Click Restart
2. Under the Log tab, confirm that `aws_bridge.conf` was loaded successfully

### Step 5: Enable MQTT Integration in Home Assistant
1. Go to **Settings → Devices & Services**
2. Locate the MQTT integration (or add it if not present)
3. Click Configure

* Ensure the following settings:

1. Enable Discovery
2. Discovery Prefix: `homeassistant`
3. Click Submit

### Step 6: Validate Discovery
Go to **Developer Tools → MQTT**

1. Subscribe to `homeassistant/#`
2. Confirm that retained config messages are received
3. Confirm that state messages are published to the correct topics

Entities should appear automatically under **Settings → Devices & Services → MQTT**

---

## 4. MQTT Topics

### 4.1. Firmware state and revision

#### 4.1.1. HomeAssistant discovery
- **Topic**: `homeassistant/update/< device_id >_fw/config`
- **Example**: `homeassistant/update/stm32u585-003000523636500A20333342_fw/config`
- **Category**: Diagnosic

#### 4.1.2. Config Payload Example:
```json
{
  "name": "Firmware",
  "unique_id": "stm32u585-003000523636500A20333342_fw_update",
  "state_topic": "stm32u585-003000523636500A20333342/fw/state",
  "value_template": "{{ value_json.installed_version }}",
  "latest_version_topic": "stm32u585-003000523636500A20333342/fw/state",
  "latest_version_template": "{{ value_json.latest_version }}",
  "command_topic": "stm32u585-003000523636500A20333342/fw/update",
  "payload_install": "start_update",
  "availability_topic": "stm32u585-003000523636500A20333342/status/availability",
  "payload_available": "online",
  "payload_not_available": "offline",
  "retain": false,
  "device_class": "firmware",
  "entity_category": "diagnostic",
  "device": {
    "identifiers": [
      "stm32u585-003000523636500A20333342"
    ],
    "manufacturer": "STMicroelectronics",
    "model": "B_U585_IOTA02",
    "name": "stm32u585-003000523636500A20333342",
    "sw_version": "1.25.0"
  }
}
```

#### 4.1.3. Device Message
- **Topic**: `< device_id >/fw/state`
- **Retained**: True

The message Sent by the device contains:

  - Current firmware revision
  - New firmware revision available to install
  - Firmware update status

#### 4.1.4. Example Payload:
```json
{
  "installed_version": "1.25.0",
  "latest_version": "1.25.0",
  "status": "completed"
}
```

#### 4.1.5. Firmware version
* If `latest_version` > `installed_version` means a new firmware is available

#### 4.1.6. Firmware status states:
```
* idle
* updating
* completed
* unknown
```

#### 4.1.7. HomeAssistant Message
- **Topic**: `< device_id >/fw/update`
- **Retained**: False

A raw message sent by the HomeAssistant to start the firmware update:

```
start_update
```

### 4.2. LED Status and Control

#### 4.2.1. HomeAssistant discovery
- **Topic**: `homeassistant/switch/< device_id >_led/config`
- **Example**: `homeassistant/switch/stm32u585-003000523636500A20333342_led/config`
- **Category**: Diagnosic

#### 4.2.2. Config Payload Example:
```json
{
  "name": "LED",
  "unique_id": "stm32u585-003000523636500A20333342_led",
  "command_topic": "stm32u585-003000523636500A20333342/led/desired",
  "state_topic": "stm32u585-003000523636500A20333342/led/reported",
  "value_template": "{{ value_json.ledStatus.reported }}",
  "payload_on": "ON",
  "payload_off": "OFF",
  "state_on": "ON",
  "state_off": "OFF",
  "availability_topic": "stm32u585-003000523636500A20333342/status/availability",
  "payload_available": "online",
  "payload_not_available": "offline",
  "retain": false,
  "entity_category": "diagnostic",
  "device": {
    "identifiers": [
      "stm32u585-003000523636500A20333342"
    ],
    "manufacturer": "STMicroelectronics",
    "model": "B_U585_IOTA02",
    "name": "stm32u585-003000523636500A20333342"
  }
}
```

#### 4.2.3. Device Message

- Sent by the device
- Indicates the current LED status
- **Topic**: `< device_id >/led/reported`
- **Retained**: True

#### 4.2.4. Device Payload:
```json
{
  "ledStatus": {
    "reported": "OFF"
  }
}
```
Or
```json
{
  "ledStatus": {
    "reported": "ON"
  }
}
```

#### 4.2.5. HomeAssistant Message
- **Topic**: `< device_id >/led/desired`  
- **Retained**: True

#### 4.2.6. Example Payload:

```
OFF 
```

Or

```
ON
```

### 4.3. Button Status

#### 4.3.1. HomeAssistant discovery
- **Topic**: `homeassistant/binary_sensor/< device_id >_button/config`
- **Example**: `homeassistant/binary_sensor/stm32u585-003000523636500A20333342_button/config`

#### 4.3.2. Config Payload Example:
```json
{
  "name": "Button",
  "unique_id": "stm32u585-001C00444841500520363230_button",
  "state_topic": "stm32u585-001C00444841500520363230/sensor/button/reported",
  "value_template": "{{ value_json.buttonStatus.reported }}",
  "payload_on": "ON",
  "payload_off": "OFF",
  "device_class": "occupancy",
  "availability_topic": "stm32u585-001C00444841500520363230/status/availability",
  "payload_available": "online",
  "payload_not_available": "offline",
  "retain": false,
  "device": {
    "identifiers": [
      "stm32u585-001C00444841500520363230"
    ],
    "manufacturer": "STMicroelectronics",
    "model": "B_U585_IOTA02",
    "name": "stm32u585-001C00444841500520363230"
  }
}
```

#### 4.3.3. Device Message
- Indicates the button status (pressed or released).
- **Topic**: `< device_id >/sensor/button/reported`
- **Retained**: True

#### Payload:
```json
{
  "buttonStatus": {
    "reported": "OFF"
  }
}
```
Or
```json
{
  "buttonStatus": {
    "reported": "ON"
  }
}
```

### 4.4. Reboot

#### 4.4.1. HomeAssistant discovery
- **Topic**: `homeassistant/button/< device_id >_reboot/config`
- **Example**: `homeassistant/button/stm32u585-003000523636500A20333342_reboot/config`
- **Category**: diagnostic

#### 4.4.2. Config Payload Example:
```json
{
  "name": "Reboot",
  "unique_id": "stm32u585-001C00444841500520363230_reboot",
  "command_topic": "stm32u585-001C00444841500520363230/cmd/action",
  "payload_press": "{\"action\":\"reboot\"}",
  "retain": false,
  "availability_topic": "stm32u585-001C00444841500520363230/status/availability",
  "payload_available": "online",
  "payload_not_available": "offline",
  "entity_category": "diagnostic",
  "device": {
    "identifiers": [
      "stm32u585-001C00444841500520363230"
    ],
    "manufacturer": "STMicroelectronics",
    "model": "B_U585_IOTA02",
    "name": "stm32u585-001C00444841500520363230"
  }
}
```

#### 4.4.3. HomeAssistant Message
- Send a command to reboot the device
- **Topic**: `< device_id >/cmd/action`
- **Retained**: False

#### 4.4.4. Example Payload:

```json
{
  "action":"reboot"
}
```

### 4.5. Env sensors

#### 4.5.1. Lux sensor HomeAssistant discovery

  - **Topic**: `homeassistant/sensor/< device_id >_lux_sensor/config`
  - **Example**: `homeassistant/sensor/stm32u585-003000523636500A20333342_lux_sensor/config`

#### 4.5.2. Lux sensor Config Payload Example:
```json
{
  "name": "Ambient Light",
  "unique_id": "stm32u585-001C00444841500520363230_lux_sensor",
  "state_topic": "stm32u585-001C00444841500520363230/sensor/env",
  "value_template": "{{ value_json.als_lux }}",
  "device_class": "illuminance",
  "unit_of_measurement": "lx",
  "availability_topic": "stm32u585-001C00444841500520363230/status/availability",
  "payload_available": "online",
  "payload_not_available": "offline",
  "retain": false,
  "device": {
    "identifiers": [
      "stm32u585-001C00444841500520363230"
    ],
    "manufacturer": "STMicroelectronics",
    "model": "B_U585_IOTA02",
    "name": "stm32u585-001C00444841500520363230"
  }
}
```

#### 4.5.3. White Lux sensor HomeAssistant discovery

  - **Topic**: `homeassistant/sensor/< device_id >_white_lux/config`
  - **Example**: `homeassistant/sensor/stm32u585-003000523636500A20333342_white_lux/config`

#### 4.5.4. Lux sensor Config Payload Example:
```json
{
  "name": "White Light",
  "unique_id": "stm32u585-001C00444841500520363230_white_lux",
  "state_topic": "stm32u585-001C00444841500520363230/sensor/env",
  "value_template": "{{ value_json.white_lux }}",
  "device_class": "illuminance",
  "unit_of_measurement": "lx",
  "availability_topic": "stm32u585-001C00444841500520363230/status/availability",
  "payload_available": "online",
  "payload_not_available": "offline",
  "retain": false,
  "device": {
    "identifiers": [
      "stm32u585-001C00444841500520363230"
    ],
    "manufacturer": "STMicroelectronics",
    "model": "B_U585_IOTA02",
    "name": "stm32u585-001C00444841500520363230"
  }
}
```

#### 4.5.5. Barometer sensor HomeAssistant discovery

  - **Topic**: `homeassistant/sensor/< device_id >_baro_mbar/config`
  - **Example**: `homeassistant/sensor/stm32u585-003000523636500A20333342_baro_mbar/config`

#### 4.5.6. Barometer sensor Config Payload Example:
```json
{
  "name": "Pressure",
  "unique_id": "stm32u585-001C00444841500520363230_env_3",
  "state_topic": "stm32u585-001C00444841500520363230/sensor/env",
  "value_template": "{{ value_json.baro_mbar }}",
  "unit_of_measurement": "mbar",
  "device_class": "pressure",
  "availability_topic": "stm32u585-001C00444841500520363230/status/availability",
  "payload_available": "online",
  "payload_not_available": "offline",
  "retain": false,
  "device": {
    "identifiers": [
      "stm32u585-001C00444841500520363230"
    ],
    "manufacturer": "STMicroelectronics",
    "model": "B_U585_IOTA02",
    "name": "stm32u585-001C00444841500520363230"
  }
}
```

#### 4.5.7. Relative Humidity sensor HomeAssistant discovery

  - **Topic**: `homeassistant/sensor/< device_id >_rh_pct/config`
  - **Example**: `homeassistant/sensor/stm32u585-003000523636500A20333342_rh_pct/config`

#### 4.5.8. Relative Humidity sensor Config Payload Example:
```json
{
  "name": "Humidity",
  "unique_id": "stm32u585-001C00444841500520363230_env_2",
  "state_topic": "stm32u585-001C00444841500520363230/sensor/env",
  "value_template": "{{ value_json.rh_pct }}",
  "unit_of_measurement": "%",
  "device_class": "humidity",
  "availability_topic": "stm32u585-001C00444841500520363230/status/availability",
  "payload_available": "online",
  "payload_not_available": "offline",
  "retain": false,
  "device": {
    "identifiers": [
      "stm32u585-001C00444841500520363230"
    ],
    "manufacturer": "STMicroelectronics",
    "model": "B_U585_IOTA02",
    "name": "stm32u585-001C00444841500520363230"
  }
}
```

#### 4.5.9. Temperature sensor HomeAssistant discovery

  - **Topic**: `homeassistant/sensor/< device_id >_temp_0_c/config`
  - **Example**: `homeassistant/sensor/stm32u585-003000523636500A20333342_temp_0_c/config`

#### 4.5.10. Temperature sensor Config Payload Example:
```json
{
  "name": "Temperature 0",
  "unique_id": "stm32u585-001C00444841500520363230_env_0",
  "state_topic": "stm32u585-001C00444841500520363230/sensor/env",
  "value_template": "{{ value_json.temp_0_c }}",
  "unit_of_measurement": "°C",
  "device_class": "temperature",
  "availability_topic": "stm32u585-001C00444841500520363230/status/availability",
  "payload_available": "online",
  "payload_not_available": "offline",
  "retain": false,
  "device": {
    "identifiers": [
      "stm32u585-001C00444841500520363230"
    ],
    "manufacturer": "STMicroelectronics",
    "model": "B_U585_IOTA02",
    "name": "stm32u585-001C00444841500520363230"
  }
}
```

#### 4.5.11. Device Message

- **Topic**: `< device_id >/sensor/env`
- **Retained**: False

#### 4.5.12. Device Payload:
```json
{
  "temp_0_c": 22.3,
  "rh_pct": 40.2,
  "baro_mbar": 998.1,
  "als_lux": 0,
  "white_lux": 0
}
```

### 4.6. Motion Sensors (Accel, Gyro, Mag)

#### 4.6.1. Motion Sensors HomeAssistant discovery

Each axis is registered as a separate sensor in Home Assistant:

  - **Topics**: 
    - `homeassistant/sensor/< device_id >_acceleration_x/config`
    - `homeassistant/sensor/< device_id >_acceleration_y/config`
    - `homeassistant/sensor/< device_id >_acceleration_z/config`

    - `homeassistant/sensor/< device_id >_gyro_x/config`
    - `homeassistant/sensor/< device_id >_gyro_y/config`
    - `homeassistant/sensor/< device_id >_gyro_z/config`

    - `homeassistant/sensor/< device_id >_magnetometer_x/config`
    - `homeassistant/sensor/< device_id >_magnetometer_y/config`
    - `homeassistant/sensor/< device_id >_magnetometer_z/config`

  - **Example**: `homeassistant/sensor/stm32u585-003000523636500A20333342_acceleration_x/config`

#### 4.6.2. Motion sensor Config Payload Example:
```json
{
  "name": "Acceleration_x",
  "unique_id": "stm32u585-001C00444841500520363230_acceleration_x",
  "state_topic": "stm32u585-001C00444841500520363230/sensor/motion",
  "value_template": "{{ value_json.acceleration_mG.x }}",
  "device_class": "Acceleration",
  "unit_of_measurement": "mG",
  "availability_topic": "stm32u585-001C00444841500520363230/status/availability",
  "payload_available": "online",
  "payload_not_available": "offline",
  "retain": false,
  "device": {
    "identifiers": [
      "stm32u585-001C00444841500520363230"
    ],
    "manufacturer": "STMicroelectronics",
    "model": "B_U585_IOTA02",
    "name": "stm32u585-001C00444841500520363230"
  }
}
```

#### 4.6.3. Device Payload:
```json
{
  "acceleration_mG":{
    "x": -543,
    "y": 855,
    "z": 969
  },
  "gyro_mDPS":{
    "x": 649,
    "y": 720,
    "z": -372
  },
  "magnetometer_mGauss":{
    "x": 460,
    "y": -554,
    "z": 609
  }
}
```

### 4.7. Device Availability

#### 4.7.1. Device Message
- **Topic**: `< device_id >/status/availability`
- **Retained**: True
- **Example**: `stm32u585-003000523636500A20333342/status/availability`

#### 4.7.2. Example Payload:

```
online
```

Or 

```
offline
```
