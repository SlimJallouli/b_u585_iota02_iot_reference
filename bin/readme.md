# Bin Quick Start (Supported Profiles Only)

This `bin/` flow is intended as a **quick start** for these supported profiles:

- `MXCHIP_Single` with `broker_type: "mosquitto"`
- `MXCHIP_Single` with `broker_type: "aws"`
- `MXCHIP_STSAFEA110` with `broker_type: "aws"`
- `MXCHIP_STSAFEA120` with `broker_type: "aws"`

All other binaries in `bin/` are kept for reference.  
If you want to use other profiles, follow the full project guides in the main docs.

## Files in `bin/`

- `flash.ps1`: flashes bootloader + selected app binary
- `provision_mosquitto.ps1`: mosquitto provisioning flow
- `provision_aws_stsafe.ps1`: AWS STSAFE provisioning flow
- `provision_aws_single.ps1`: AWS single-thing provisioning flow
- `run_all.ps1`: runs flash, then provisioning
- `config.json`: profile and connection settings

## Prerequisites

1. Windows + PowerShell
2. Install STM32CubeProgrammer:
   - https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-programmers/stm32cubeprog.html
   - Make sure `STM32_Programmer_CLI.exe` is available at:
   `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe`
3. Board connected through ST-LINK USB
4. Internet access (scripts download broker Root CA automatically)
5. If you plan to use `broker_type: "aws"`:
   - Install AWS CLI v2
   - Configure credentials/region with `aws configure`

## Quick Start

1. Open `bin/config.json`
2. Choose one supported setup:

## What `run_all.ps1` Does

```mermaid
flowchart TD
    A[Start run_all.ps1] --> B[Read config.json]
    B --> C[Select firmware in flash.ps1 by configuration]
    C --> D[Mass erase confirmation]
    D --> E[Flash bootloader + app HEX]
    E --> F{broker_type}

    F -->|mosquitto| G[Run provision_mosquitto.ps1]
    G --> G1[Detect COM + open serial]
    G1 --> G2[Reset basic Wi-Fi config]
    G2 --> G3[Download + import mosquitto root CA]
    G3 --> G4[Generate key + CSR on device]
    G4 --> G5[Request cert from test.mosquitto.org or manual fallback]
    G5 --> G6[Import tls_cert + set endpoint/port + Wi-Fi + commit + reset]

    F -->|aws + single config| H[Run provision_aws_single.ps1]
    H --> H1[Detect COM + open serial]
    H1 --> H2[Reset basic Wi-Fi config]
    H2 --> H3[Download + import AWS root CA]
    H3 --> H4[Generate key + CSR on device]
    H4 --> H5[AWS CLI create cert from CSR]
    H5 --> H6[Ensure Thing exists + attach cert + attach policy]
    H6 --> H7[Import tls_cert + fetch AWS endpoint + set MQTT/Wi-Fi + commit + reset]

    F -->|aws + STSAFE config| I[Run provision_aws_stsafe.ps1]
    I --> I1[Detect COM + open serial]
    I1 --> I2[Download + import AWS root CA]
    I2 --> I3[Reset basic Wi-Fi config]  
    I3 --> I4[Read thing_name + export STSAFE tls_cert]
    I4 --> I5[AWS CLI register cert without CA]
    I5 --> I6[Ensure Thing exists + attach cert + attach policy]
    I6 --> I7[Fetch AWS endpoint + set MQTT/Wi-Fi + commit + reset]

    G6 --> Z[Done]
    H7 --> Z
    I7 --> Z
```

Notes:
- Provisioning output is shown live in console and appended to `bin/log.txt`.
- `run_all.ps1` automatically picks the correct provisioning script from `broker_type` and `configuration`.

### Option A: Mosquitto + `MXCHIP_Single`

Example:

```json
{
  "broker_type": "mosquitto",
  "configuration": "MXCHIP_Single",
  "wifi_ssid": "YOUR_WIFI",
  "wifi_credential": "YOUR_PASSWORD"
}
```

Notes:
- `provision_mosquitto.ps1` generates key + CSR on device.
- It tries to auto-request a client cert from `https://test.mosquitto.org/ssl/`.
- If auto-request fails, it falls back to manual cert download and asks for cert path.

### Option B: AWS + `MXCHIP_Single`

Example:

```json
{
  "broker_type": "aws",
  "configuration": "MXCHIP_Single",
  "wifi_ssid": "YOUR_WIFI",
  "wifi_credential": "YOUR_PASSWORD"
}
```

### Option C: AWS + `MXCHIP_STSAFEA110`

Example:

```json
{
  "broker_type": "aws",
  "configuration": "MXCHIP_STSAFEA110",
  "wifi_ssid": "YOUR_WIFI",
  "wifi_credential": "YOUR_PASSWORD"
}
```

### Option D: AWS + `MXCHIP_STSAFEA120`

Example:

```json
{
  "broker_type": "aws",
  "configuration": "MXCHIP_STSAFEA120",
  "wifi_ssid": "YOUR_WIFI",
  "wifi_credential": "YOUR_PASSWORD"
}
```

Notes:
- AWS endpoint is fetched automatically from AWS CLI (`aws iot describe-endpoint`).
- AWS MQTT port is fixed to `8883` in the script.
- AWS Root CA is downloaded automatically in the script.
- Make sure AWS CLI is installed and configured (`aws configure`).

AWS behavior:
- For single configs, the script creates certificate from CSR and registers/attaches it.
- For STSAFE configs, the script exports device certificate and registers it with AWS (`register-certificate-without-ca`), then attaches Thing/policy.
- Default policy name is `AllowAllDev` (or `aws_policy_name` if provided in `config.json`).
- If the selected AWS IoT policy does not exist, the script creates it automatically with a default allow-all policy document, then attaches it.

Detailed AWS MAR guide:
- https://github.com/stm32-hotspot/stm32mcu_aws_mar?tab=readme-ov-file#73-register-with-aws

3. Run:

```powershell
cd .\bin
.\run_all.ps1
```

4. Confirm the mass-erase prompt (`y`) when asked.

## Important Scope

- This quick-start flow is validated only for:
  - `MXCHIP_Single` + Mosquitto
  - `MXCHIP_Single` + AWS
  - `MXCHIP_STSAFEA110` + AWS
  - `MXCHIP_STSAFEA120` + AWS
- Other profile binaries are provided as reference only in this folder.

## For Other Configurations

Use the main project documentation:

- [Main README](../readme.md)
- [Mosquitto provisioning guide](../provision_mosquitto.md)
- [AWS STSAFE provisioning guide](../provision_aws_STSAFE.md)
- [AWS single-device provisioning guide](../provision_aws_single_script.md)
