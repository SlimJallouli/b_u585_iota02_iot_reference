# Garage Door Cover Control (cover_task.c)
This module implements MQTT‑controlled garage door covers for STM32‑based devices.
It supports 1–3 garage doors, relay‑pulse motor control, optional magnetic door sensors, and full Home Assistant MQTT Discovery.

The firmware is designed to work with the Seeed Studio Relay Shield V2  
https://wiki.seeedstudio.com/Relay_Shield_V2/


[Magnetic-RC-33-NC-Recessed-Security](https://www.amazon.com/Magnetic-RC-33-NC-Recessed-Security/dp/B0C394NYZ7/ref=asc_df_B0C394NYZ7?tag=bingshoppinga-20&linkCode=df0&hvadid=80470697129904&hvnetw=o&hvqmt=e&hvbmt=be&hvdev=c&hvlocint=&hvlocphy=111499&hvtargid=pla-4584070167952797&psc=1&msclkid=399944650c9a19c99d312df7050c7f86)

## Hardware Overview
Relay → STM32 Pin Mapping
The Seeed Relay Shield V2 uses four active‑high relays.
This firmware uses up to three of them:

```c
#define RELAY_1_Pin   ARD_D07_Pin
#define RELAY_1_Port  ARD_D07_GPIO_Port

#define RELAY_2_Pin   ARD_D06_Pin
#define RELAY_2_Port  ARD_D06_GPIO_Port

#define RELAY_3_Pin   ARD_D05_Pin
#define RELAY_3_Port  ARD_D05_GPIO_Port

#define RELAY_4_Pin   ARD_D04_Pin
#define RELAY_4_Port  ARD_D04_GPIO_Port
```

Each relay produces a 1‑second pulse, simulating a physical garage door button press.

Optional Magnetic Door Sensors
```c
#define DOOR_SENSPR_1_Pin        ARD_D02_Pin
#define DOOR_SENSPR_1_Port       ARD_D02_GPIO_Port
#define DOOR_SENSPR_1_STATE_OPEN GPIO_PIN_SET

#define DOOR_SENSPR_2_Pin        ARD_D08_Pin
#define DOOR_SENSPR_2_Port       ARD_D08_GPIO_Port
#define DOOR_SENSPR_2_STATE_OPEN GPIO_PIN_SET

#define DOOR_SENSPR_3_Pin        ARD_D09_Pin
#define DOOR_SENSPR_3_Port       ARD_D09_GPIO_Port
#define DOOR_SENSPR_3_STATE_OPEN GPIO_PIN_SET
```

To disable sensors entirely:

```c
#define USE_DOOR_SENSPR 0
```

When disabled, the firmware reports:

- unknown at boot
- The last commanded state (OPEN/CLOSE/STOP)

## How It Works
MQTT Command Topic
Home Assistant sends commands to:

```Code
<ThingName>/cover/<COVER_NAME>/desired
```
Example:

```Code
eval3-02093088825AD42AC20139/cover/GARAGE_DOOR_1/desired
```

Payload:

```Code
OPEN
CLOSE
STOP
```

MQTT State Topic
The firmware reports the current state to:

```Code
<ThingName>/cover/<COVER_NAME>/state
```
Example:

```Code
eval3-02093088825AD42AC20139/cover/GARAGE_DOOR_1/state
```

State values:

```Code
"open"
"closed"
"stopped"
"unknown"
```

Relay Pulse Motor Control
Every command triggers a 1‑second relay pulse, identical to pressing the wall button.

Door Sensors (Optional)
If enabled, sensors are polled at 5 Hz.
If disabled, the firmware never overwrites the commanded state.

## Compile‑Time Configuration
Set the number of garage doors:

```c
#define NUM_COVERS 1   // or 2 or 3
```

The firmware automatically:

- Creates the correct number of relay entries
- Creates the correct number of cover descriptors
- Publishes or clears Home Assistant discovery topics
- Avoids stale retained messages

## MQTT Topics Overview
Command
```Code
<ThingName>/cover/<COVER_NAME>/desired
```

State
```Code
<ThingName>/cover/<COVER_NAME>/state
```
Home Assistant Discovery
```Code
homeassistant/cover/<ThingName>_<COVER_NAME>/config
```
## Example MQTT Messages
Open the garage door
Topic:

```Code
eval3-02093088825AD42AC20139/cover/GARAGE_DOOR_1/desired
```
Payload:

```Code
OPEN
```

Close the garage door
```Code
CLOSE
```

Stop the garage door
```Code
STOP
```

Example state report
Topic:

```Code
eval3-02093088825AD42AC20139/cover/GARAGE_DOOR_1/state
```
Payload:

```Code
open
```

## Home Assistant Integration
The firmware publishes MQTT Discovery messages so Home Assistant automatically creates:

- Cover entities
- Availability status
- Device metadata (serial number, model, firmware version)
- No YAML configuration is required.

## Firmware Architecture
1. vCoverTask()
- Main FreeRTOS task:
- Waits for MQTT agent connection
- Loads ThingName from KVStore
- Subscribes to all cover command topics
- Polls door sensors (if enabled)
- Publishes state changes

2. prvIncomingPublishCallback()
- Handles incoming MQTT commands:
- Extracts cover name
- Extracts command string
= Calls prvHandleCoverCommand()

3. prvHandleCoverCommand()
- Executes the motor action:
- OPEN → relay pulse
- CLOSE → relay pulse
- STOP → relay pulse
- Updates internal state and triggers publish.

4. prvPublishCoverStates()
- Publishes the current state of each cover.

5. prvReadDoorSensor()
- Reads GPIO sensor state (if enabled).

## Relay Hardware Behavior
The Seeed Relay Shield V2 uses active‑high relays.
The firmware pulses each relay for 1000 ms:

```c
HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
HAL_Delay(1000);
HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
```

This mimics a physical garage door button.

## Monitoring MQTT Messages
You can use any MQTT client:

- AWS IoT MQTT test client
- mqtt.cool
- mqttx.app
- mosquitto_sub / mosquitto_pub

Subscribe to:

```Code
<ThingName>/cover/+/state
```
Publish to:

```Code
<ThingName>/cover/+/desired
```

## Summary
This firmware provides a complete, robust, and flexible garage door controller:

- Supports 1–3 garage doors
- Relay pulse control using Seeed Relay Shield V2
- Optional magnetic door sensors
- Full Home Assistant MQTT discovery
- Clean state reporting
- Simple command interface
- FreeRTOS + MQTT Agent architecture
- It is designed to be reliable, scalable, and easy to integrate into any Home Assistant setup.