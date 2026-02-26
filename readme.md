# STM32U585 Secure IoT Firmware Reference for B-U585I-IOT02A (FreeRTOS, AWS IoT MQTT, STSAFE, OTA)

[![Board: B-U585I-IOT02A](https://img.shields.io/badge/Board-B--U585I--IOT02A-03234B)](https://www.st.com/en/evaluation-tools/b-u585i-iot02a.html)
[![RTOS: FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-1A73E8)](https://www.freertos.org/)
[![Network: LwIP](https://img.shields.io/badge/Network-LwIP-006064)](https://savannah.nongnu.org/projects/lwip/)
[![TLS: MbedTLS 3.1.1](https://img.shields.io/badge/TLS-MbedTLS%203.1.1-283593)](https://www.keil.arm.com/packs/mbedtls-arm/versions/)
[![Security: STSAFE A110/A120](https://img.shields.io/badge/Security-STSAFE%20A110%2FA120-0B8043)](https://www.st.com/en/secure-mcus/stsafe-a110.html)
[![Home Assistant Compatible](https://img.shields.io/badge/Home%20Assistant-Compatible-18BCF2)](https://www.home-assistant.io/integrations/mqtt)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](./LICENSE.md)

This repository is a production-oriented **secure IoT firmware reference** for the **STM32U585** and **B-U585I-IOT02A Discovery Kit**, built with **FreeRTOS**, **LwIP**, and **MbedTLS**. It demonstrates **MQTT over TLS** connectivity to **AWS IoT Core**, **Mosquitto**, and **EMQX**, including **fleet provisioning**, **device shadow**, **jobs**, and **host OTA update** workflows.

If you are searching for a **secure STM32 AWS IoT example**, **STM32U5 MQTT over TLS reference**, **B-U585I-IOT02A FreeRTOS project**, or **STSAFE secure element integration**, this project is designed for that use case.

## Table of Contents

- [Why This Project](#why-this-project)
- [Upstream and Key Differences](#upstream-and-key-differences)
- [Key Features](#key-features)
- [Supported Hardware and Sensors](#supported-hardware-and-sensors)
- [Security Architecture](#security-architecture)
- [Software Components (FreeRTOS, LwIP, MbedTLS, PKCS#11)](#software-components-freertos-lwip-mbedtls-pkcs11)
- [Flash Memory Layout](#flash-memory-layout)
- [Quick Start](#quick-start)
- [Build Configuration Matrix](#build-configuration-matrix)
- [Provisioning Guides](#provisioning-guides)
- [Run the Examples](#run-the-examples)
- [Required CMSIS Packs](#required-cmsis-packs)
- [Git Submodules](#git-submodules)
- [STM32CubeMX Regeneration Note](#stm32cubemx-regeneration-note)
- [Enable or Disable Examples](#enable-or-disable-examples)

## Why This Project

This reference helps embedded developers build secure, cloud-connected firmware on STM32U5 with a modular architecture and production-style security patterns:

- Portable software stack across Wi-Fi and security variants
- MQTT over TLS with software or hardware-backed credentials
- Standard interfaces for cryptography and runtime configuration
- Proven end-to-end flows for provisioning, telemetry, control, and OTA

## Upstream and Key Differences

This repository is derived from the official FreeRTOS STM32U5 reference project:

- https://github.com/FreeRTOS/iot-reference-stm32u5

Main differences in this repository:

- Adds **AWS IoT Fleet Provisioning** support
- Adds **STSAFE** security flows (A110/A120)
- Supports both **MXCHIP** and **ST67** Wi-Fi module variants
- Adds support for **Mosquitto** and **EMQX** MQTT brokers
- Supports **STM32CubeMX project regeneration** based on **CMSIS packs**
- Stores runtime configuration in **internal flash or STSAFE** (instead of external flash used in the upstream reference)
- Does **not** support **TrustZone**

## Key Features

- **Secure MQTT Connectivity**: AWS IoT Core, test.mosquitto.org, broker.emqx.io
- **Hardware Security**: STSAFE-A110 and STSAFE-A120 support
- **Provisioning at Scale**: AWS Fleet Provisioning, MAR, JITP, JITR
- **AWS IoT Services**: Device Shadow, Jobs, MQTT file delivery, OTA
- **Runtime Applications**: LED, button, sensors, Home Assistant discovery
- **Storage Abstraction**: PKCS#11 and KVS over internal flash or STSAFE

## Supported Hardware and Sensors

- **Main board**: [B-U585I-IOT02A](https://www.st.com/en/evaluation-tools/b-u585i-iot02a.html)
- **Wi-Fi modules**:
  - [X-NUCLEO-67W61M1](https://www.st.com/en/evaluation-tools/x-nucleo-67w61m1.html)
  - MXCHIP EMW3080B
- **Secure element**:
  - [STSAFE-A110](https://www.st.com/en/secure-mcus/stsafe-a110.html)
  - STSAFE-A120 via [X-NUCLEO-ESE01A1](https://www.st.com/en/evaluation-tools/x-nucleo-ese01a1.html)
- **Sensors**:
  - Temperature/Humidity: [HTS221](https://www.st.com/resource/en/datasheet/hts221.pdf)
  - Magnetometer: [IIS2MDC](https://www.st.com/en/mems-and-sensors/iis2mdc.html)
  - IMU: [ISM330DHCX](https://www.st.com/en/mems-and-sensors/ism330dhcx.html)
  - Pressure: [LPS22HH](https://www.st.com/en/mems-and-sensors/lps22hh.html)
  - Ambient light: [VEML3235](https://www.vishay.com/en/product/80131/)
  - Time-of-flight: [VL53L5CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html)

## Security Architecture

### Flexible and Portable Security Architecture

The project supports connections to **AWS IoT Core**, **test.mosquitto.org**, and **broker.emqx.io**. In all cases, **X.509 certificates** are required for device/server authentication, ensuring secure and trusted communication with the MQTT broker.

This project provides multiple build configurations to support a variety of hardware platforms and secure connectivity methods. `MXCHIP`, `ST67_T01`, and `ST67_T02` configurations offer flexibility by supporting different wireless connections. The remaining configurations (`FleetProvisioning`, `STSAFEA110`, and `STSAFEA120` variants) are tailored specifically for **AWS IoT Core**, utilizing features such as Just-In-Time Provisioning and hardware-based secure elements to enable secure device onboarding.

In most configurations, the TLS and MQTT stacks run on the host microcontroller. However, in the `ST67_T01_Single` configuration, TLS and MQTT communication are offloaded to the ST67 Wi-Fi module.

In `STSAFEA110` and `STSAFEA120` configurations, all sensitive assets, including the TLS private key, device certificate, broker root CA, and device configuration parameters (for example: MQTT endpoint, port, and Wi-Fi credentials), are securely stored within the non-volatile memory of the STSAFE-A secure element.

To further strengthen system resilience, cryptographic operations utilize the hardware-based Random Number Generator (RNG) embedded within STSAFE, rather than relying on the STM32 internal RNG. The STSAFE RNG is designed to meet stringent security standards (such as NIST SP 800-90A/B/C) and generates high-entropy randomness suited for secure key generation, session establishment, and cryptographic protocols. This minimizes predictability and entropy bias risks during TLS handshakes and other secure interactions.

Additionally, encrypted I2C communication between the host MCU and STSAFE can be enabled, ensuring confidentiality and integrity of sensitive exchanges, even on potentially untrusted system buses.

By offloading credential storage and secure random number generation to STSAFE, the system reduces attack surface and aligns with hardware-isolation security best practices.

Whether credentials and configuration are stored in internal flash or in a secure element (STSAFE-A110/A120), the application always accesses them through standardized interfaces, providing a consistent abstraction layer across all configurations.

- `PKCS#11` is used for cryptographic assets such as TLS keys and certificates.
- `KVS` (Key-Value Storage) is used for runtime parameters such as MQTT endpoint, port, and Wi-Fi credentials.

This abstraction enables code portability across hardware configurations and promotes secure, modular design without requiring changes in application logic.

### STSAFE Zone Mapping

| Purpose | STSAFEA110 | STSAFEA120 |
|---|---|---|
| Device certificate | Zone 0 | Zone 0 |
| KV store configuration | Zone 1 | Zone 1 |
| Code signing key | Zone 2 | Zone 2 |
| Server CA certificate | Zone 4 | Zone 11 |

Mapping is configured in `stsafe.h` and `stsafe.c`.

## Software Components (FreeRTOS, LwIP, MbedTLS, PKCS#11)

### MbedTLS 3.1.1 TLS and Cryptography Library

See [MbedTLS](https://www.keil.arm.com/packs/mbedtls-arm/versions/) for details.

### Command Line Interface (CLI)

The CLI interface located in `project/Common/` is used to provision the device. It also provides Unix-like utilities. See [`project/Common/cli/ReadMe.md`](project/Common/cli/ReadMe.md) for details.

### Key-Value Store (KVS)

The key-value store located in `project/Common/kvstore/` is used to store runtime configuration values in STM32 internal flash memory or in STSAFE, depending on the selected project configuration. See [`project/Common/kvstore/ReadMe.md`](project/Common/kvstore/ReadMe.md) for details.

### PkiObject API

The PkiObject API handles conversion between different representations of cryptographic objects such as public keys, private keys, and certificates. See [`project/Common/crypto/ReadMe.md`](project/Common/crypto/ReadMe.md) for details.

### PKCS#11

The PKCS#11 API is used to manage keys and certificates, whether they are stored in internal flash or within the STSAFE secure element. See [corePKCS11 Library](https://github.com/FreeRTOS/corePKCS11/blob/main/README.md) for details.

```text
                      ┌────────────────────────────┐
                      │   Application Layer        │
                      │  (TLS, MQTT, Wi-Fi setup)  │
                      └──────┬──────────┬──────────┘
                             │          │
                        ┌────▼────┐┌────▼────┐
Manage keys and certs ->│ PKCS#11 ││  KVS    │ <- stores runtime config like endpoints and Wi-Fi
                        └────┬────┘└────┬────┘
                             │          │
                             │          │
                      ┌──────▼──────────▼───────┐
                      │ Storage Backend Layer   │
                      ├─────────────────────────┤
                      │ Internal Flash (lfs)    │
                      │        OR               │
                      │ STSAFE-A (A110/A120)    │ <- secure storage and cryptographic engine
                      └─────────────────────────┘
```

In STSAFE configurations, all certificates, keys, and runtime settings are stored in STSAFE. In other cases, they reside in the STM32 internal flash.

In `ST67_T01_Single`, certificates and keys are transferred from STM32 internal flash to ST67 internal file system, and TLS/MQTT is managed by ST67. Otherwise, TLS/MQTT runs on the host processor.

## Flash Memory Layout

STM32U5 dual-bank flash is partitioned to support boot, application, OTA staging, and persistent assets.

| Flash Bank | Region | Size | Purpose |
|---|---|---:|---|
| Bank 1 | Bootloader | 64 KB | Boot and handoff |
| Bank 1 | Main application | 768 KB | Primary firmware image |
| Bank 1 | Reserved | 192 KB | Reserved space |
| Bank 2 | HOTA image | 768 KB | Staging area for host OTA updates |
| Bank 2 | LittleFS | 256 KB | Certificates, keys, and runtime configuration (non-STSAFE profiles) |

![STM32U5 Flash Layout](assets/STM32U5_Flash_Layout.png)

Bootloader responsibilities:
- Install validated HOTA image to application region
- Jump to main application

> Note: HOTA is available only with AWS-connected profiles.

> SECURITY WARNING  
> The provided bootloader is for demonstration purposes and does not include full production hardening (for example: secure boot enforcement, anti-rollback, signature policy, debug lock strategy). Perform a security review before deployment.

## Quick Start

> CRITICAL NOTE  
> If you regenerate code with STM32CubeMX, you must run `update.sh` before building. Skipping this step causes build failures.

### 1. Clone with submodules

```bash
git clone https://github.com/SlimJallouli/b_u585_iota02_iot_reference.git --recurse-submodules
```

If already cloned without submodules:

```bash
git submodule update --init --recursive
```

### 2. Update X-NUCLEO-67W61M1 firmware (if used)

Update the ST67 module to firmware revision `1.2.0` using the official binaries/instructions:

- https://github.com/STMicroelectronics/x-cube-st67w61/tree/main/Projects/ST67W6X_Scripts/Binaries

Profile note:
- `profile_t01`: TCP/IP, MQTT, TLS on ST67
- `profile_t02`: TCP/IP, MQTT, TLS on STM32 host (recommanded)

### 3. Quick Start with Prebuilt Binaries and Scripts

If you want the fastest path to run this project, use the prebuilt binaries and automation scripts in `bin/`.

This option is recommended for quick validation because it:
- Avoids immediate IDE/project setup
- Uses prebuilt firmware images
- Automates flash + provisioning through PowerShell scripts

Start here:
- [bin/readme.md](bin/readme.md)

### 4. Import into STM32CubeIDE

- Open STM32CubeIDE
- Select **Import Project**
- Choose the repository root folder `b_u585_iota02_iot_reference`
- Import both projects (bootloader + reference app)

![STM32CubeIDE Info Center](assets/STM32CubeIDE_InfoCenter.png)
![STM32CubeIDE Import Project](assets/STM32CubeIDE_ImportProject.png)

### 5. Select a build configuration and build

Use the dropdown next to the hammer icon in STM32CubeIDE.

![Build Configurations](assets/build_configurations.png)

### 6. Flash and debug

Use the provided debug configurations to:
- Build bootloader + selected app profile
- Flash both images
- Start execution from bootloader

![STM32CubeIDE Debug Configuration](assets/STM32CubeIDE_DebugConfiguration.png)
![STM32CubeIDE Debug Select](assets/STM32CubeIDE_DebugSelect.png)
![STM32CubeIDE Debug Switch](assets/STM32CubeIDE_DebugSwitch.png)
![STM32CubeIDE Debug](assets/STM32CibeIDE_Debug.png)

## Build Configuration Matrix

| Build Config | AWS IoT Core | Mosquitto | EMQX | OTA |
|---|---:|---:|---:|---:|
| `MXCHIP_Single` | Yes | Yes | No | Yes |
| `MXCHIP_FleetProvisioning` | Yes | No | No | Yes |
| `MXCHIP_STSAFEA110` | Yes | No | No | Yes |
| `MXCHIP_STSAFEA120` | Yes | No | No | Yes |
| `ST67_T01_Single` | Yes | Yes | Yes | No |
| `ST67_T02_Single` | Yes | Yes | No | Yes |
| `ST67_T02_FleetProvisioning` | Yes | No | No | Yes |
| `ST67_T02_STSAFEA110` | Yes | No | No | Yes |
| `ST67_T02_STSAFEA120` | Yes | No | No | Yes |

Notes:
- Fleet provisioning, STSAFE, and OTA options are AWS-specific.
- In STSAFE profiles, keys/certs/config are stored in STSAFE.
- In `ST67_T01_Single`, TLS/MQTT is handled by ST67.

## Provisioning Guides

Choose your broker and onboarding method:

1. Mosquitto (`test.mosquitto.org`)
- Supported: `MXCHIP_Single`, `ST67_T01_Single`, `ST67_T02_Single`
- Guide: [Provision and Run with test.mosquitto.org](provision_mosquitto.md)

2. EMQX (`broker.emqx.io`)
- Supported: `ST67_T01_Single`
- Guide: [Provision and Run with EMQX MQTT Broker](provision_emqx.md)

3. AWS IoT Core (Single Thing)
- Supported: `MXCHIP_Single`, `ST67_T01_Single`, `ST67_T02_Single`
- Guides:
  - [Provision and Run with AWS (CLI)](provision_aws_single_cli.md)
  - [Provision and Run with AWS (Script)](provision_aws_single_script.md)

4. AWS IoT Core (Fleet Provisioning)
- Supported: `MXCHIP_FleetProvisioning`, `ST67_T02_FleetProvisioning`
- Guide: [Provision and Run with AWS Fleet Provisioning](provision_aws_FleetProvisioning.md)

5. AWS IoT Core (STSAFE onboarding)
- Supported: `MXCHIP_STSAFEA110`, `MXCHIP_STSAFEA120`, `ST67_T02_STSAFEA110`, `ST67_T02_STSAFEA120`
- Methods: MAR, JITP, JITR
- Guide: [Provision and Run with AWS using STSAFE](provision_aws_STSAFE.md)

## Run the Examples

After provisioning, use these feature guides:

- [LED Control Example](project/Common/app/led/readme.md)
- [Button Status Example](project/Common/app/button/readme.md)
- [Home Assistant Discovery Example](project/Common/app/HomeAssistant/home_assistant_discovery.md)
- [Environmental Sensor Example](project/Common/app/sensors/env_sensor_readme.md)
- [Garage Door Cover Control Example](project/Common/app/cover/README.md)
- [Motion Sensor Example](project/Common/app/sensors/motion_sensor_readme.md)
- [AWS Fleet Provisioning Guide](provision_aws_FleetProvisioning.md)
- [AWS Defender](readme_aws_defender.md)
- [AWS Shadow](readme_aws_shadow.md)
- [AWS OTA](readme_aws_ota.md)

## Required CMSIS Packs

Install these packs in STM32CubeMX **before opening** the `.ioc` file:

- [ARM.CMSIS-FreeRTOS 11.2.0](https://www.keil.com/pack/ARM.CMSIS-FreeRTOS.11.2.0.pack)
- [ARM.mbedTLS 3.1.1](https://www.keil.com/pack/ARM.mbedTLS.3.1.1.pack)
- [AWS IoT OTA 5.0.1](https://d1pm0k3vkcievw.cloudfront.net/AWS.AWS_IoT_Over-the-air_Update.5.0.1.pack)
- [AWS IoT Device Defender 4.2.0](https://freertos-cmsis-packs.s3.us-west-2.amazonaws.com/AWS.AWS_IoT_Device_Defender.4.2.0.pack)
- [AWS IoT Device Shadow 5.1.0](https://freertos-cmsis-packs.s3.us-west-2.amazonaws.com/AWS.AWS_IoT_Device_Shadow.5.1.0.pack)
- [AWS IoT Fleet Provisioning 1.1.0](https://freertos-cmsis-packs.s3.us-west-2.amazonaws.com/AWS.AWS_IoT_Fleet_Provisioning.1.1.0.pack)
- [AWS backoffAlgorithm 4.2.0](https://freertos-cmsis-packs.s3.us-west-2.amazonaws.com/AWS.backoffAlgorithm.4.2.0.pack)
- [AWS coreJSON 4.2.0](https://freertos-cmsis-packs.s3.us-west-2.amazonaws.com/AWS.coreJSON.4.2.0.pack)
- [AWS coreMQTT 5.1.0](https://freertos-cmsis-packs.s3.us-west-2.amazonaws.com/AWS.coreMQTT.5.1.0.pack)
- [AWS coreMQTT_Agent 5.1.0](https://freertos-cmsis-packs.s3.us-west-2.amazonaws.com/AWS.coreMQTT_Agent.5.1.0.pack)
- [lwIP 2.3.0](https://www.keil.com/pack/lwIP.lwIP.2.3.0.pack)

Also used via STM32CubeMX dependency flow:
- [X-CUBE-SAFEA1](https://www.st.com/en/embedded-software/x-cube-safea1.html)
- [X-CUBE-ST67W61](https://www.st.com/en/embedded-software/x-cube-st67w61.html)
- [X-CUBE-MEMS1](https://www.st.com/en/embedded-software/x-cube-mems1.html)

## Git Submodules

This project includes external dependencies as submodules:

- [corePKCS11](https://github.com/FreeRTOS/corePKCS11)
- [littlefs](https://github.com/littlefs-project/littlefs)
- [tinycbor](https://github.com/intel/tinycbor)

## STM32CubeMX Regeneration Note

After regenerating from STM32CubeMX, run the project update script before building to avoid generated-code integration issues:

- `update.sh`

## Enable or Disable Examples

Feature toggles are defined in `project/Core/Inc/main.h`:

```c
#define DEMO_PUB_SUB                            0
#define DEMO_OTA                                1
#define DEMO_ENV_SENSOR                         1
#define DEMO_MOTION_SENSOR                      1
#define DEMO_SHADOW                             1
#define DEMO_DEFENDER                           1
#define DEMO_LED                                1
#define DEMO_BUTTON                             1
#if !defined(ST67W6X_NCP)
#define DEMO_HOME_ASSISTANT                     1
#endif
#define DEMO_ECHO_SERVER                        0
#define DEMO_ECHO_CLIENT                        0
#define DEMO_PING                               0
#if defined(ST67W6X_NCP)
#define DEMO_SNTP                               1
#endif
```

Set a macro to `1` to enable and `0` to disable, then rebuild and flash.
