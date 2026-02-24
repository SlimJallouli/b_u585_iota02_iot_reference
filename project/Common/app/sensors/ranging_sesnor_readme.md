# STM32U585 Ranging Sensor (VL53L5CX) for Garage Door State Detection

This document describes the actual behavior of `ranging_sensor.c` in this project.

The ranging task does **not** publish MQTT payloads directly. Instead, it determines door state (`OPEN` / `CLOSED`) from VL53L5CX distance data and notifies the cover task.

## What This Module Does

- Initializes and configures the VL53L5CX ToF sensor
- Samples center-zone distance periodically
- Applies fixed hysteresis thresholds to infer door state
- Updates global `gDoorState`
- Sets `EVT_DOOR_STATE_CHANGED` so `cover_task.c` can publish cover state

## Source Files

- `project/Common/app/sensors/ranging_sensor.c`
- `project/Common/app/cover/cover_task.c`
- `project/Core/Inc/main.h`
- `project/Core/Src/app_freertos.c`

## Compile-Time Requirements

The following constraints are enforced in firmware:

- `USE_RANGING_SENSOR` and `USE_MAGNETIC_SENSOR` are mutually exclusive
- `USE_RANGING_SENSOR` supports only one cover (`NUM_COVERS == 1`)

Example configuration (`main.h`):

```c
#define NUM_COVERS           1
#define USE_MAGNETIC_SENSOR  0
#define USE_RANGING_SENSOR   1
```

## Task Lifecycle

`vRangingSensorTask()` is created in `app_freertos.c` when `USE_RANGING_SENSOR` is enabled.

Runtime flow:

1. `xInitSensors()` probes/initializes VL53L5CX
2. Sensor profile is configured (`4x4 continuous`, timing budget `30`, frequency `5`)
3. Task reads distance periodically (polling period: `1000 ms`)
4. Center distance is extracted and door state is updated with hysteresis
5. On state change, firmware sets `EVT_DOOR_STATE_CHANGED`

## Door State Logic

Global state:

- `DOOR_STATE_UNKNOWN`
- `DOOR_STATE_OPEN`
- `DOOR_STATE_CLOSED`

Thresholds in `ranging_sensor.c`:

- `DOOR_OPEN_THRESHOLD_MM  = 500` (50 cm)
- `DOOR_CLOSE_THRESHOLD_MM = 800` (80 cm)

Hysteresis behavior:

- From `CLOSED` -> `OPEN` when distance `< 500 mm`
- From `OPEN` -> `CLOSED` when distance `> 800 mm`
- From `UNKNOWN`:
  - `< 500 mm` => `OPEN`
  - otherwise => `CLOSED`

This gap (500-800 mm) avoids rapid state toggling near threshold.

## Distance Extraction Method

`GetCenterDistance()` computes the average distance of the 4 center zones:

- Works for both 4x4 and 8x8 layouts
- Uses only zones with valid targets (`NumberOfTargets > 0`)
- Returns `0` when no center zones report a target

## Integration with Cover Task

`ranging_sensor.c` exports `gDoorState`, which is consumed by `cover_task.c` when `USE_RANGING_SENSOR == 1`.

On state change, the ranging task sets:

```c
xEventGroupSetBits(xSystemEvents, EVT_DOOR_STATE_CHANGED);
```

`cover_task.c` then publishes cover state over MQTT using its normal topics:

- `<thing_name>/cover/<COVER_NAME>/state`

For full cover MQTT details, see:

- [Cover README](../cover/README.md)

## Important Clarification

This module is **not** an occupancy/intensity publisher and does not implement:

- `occupancy_level`
- custom `tof/occupancy` MQTT topics
- Home Assistant sensor discovery payload publication

Its role is specifically **door state detection** for cover control.
