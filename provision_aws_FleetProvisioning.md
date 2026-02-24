# AWS IoT Fleet Provisioning for STM32U585 (B-U585I-IOT02A, FreeRTOS)

This guide shows how to provision the STM32U585-based B-U585I-IOT02A project with **AWS IoT Core Fleet Provisioning** for secure, scalable device onboarding.

## Supported Build Configurations

| Build Config | Provisioning Method |
|---|---|
| `MXCHIP_FleetProvisioning` | AWS IoT Fleet Provisioning |
| `ST67_T02_FleetProvisioning` | AWS IoT Fleet Provisioning |

## 1. Hardware Setup

Set up your board based on the selected configuration:

- `MXCHIP_FleetProvisioning` or `ST67_T02_FleetProvisioning`
  - Connect the Wi-Fi module to the `STMod+` or `Arduino` connector.

For all configurations:
- Connect the board to your PC via the ST-Link USB port for power, flashing, and debugging.

## 2. What AWS IoT Fleet Provisioning Does

[AWS IoT Fleet Provisioning (claim-based)](https://docs.aws.amazon.com/iot/latest/developerguide/provision-wo-cert.html#claim-based) automates secure device onboarding by:

- Assigning unique device identity and certificates
- Validating registration attributes (for example via Lambda)
- Creating AWS IoT registry resources and policies

This workflow is recommended for production and large-scale IoT deployments.

## 3. Provisioning Steps

Follow the full step-by-step provisioning workflow here:

- [STM32 AWS Fleet Provisioning Setup Repository](https://github.com/SlimJallouli/stm32mcu_aws_fleetProvisioning)

## 4. Run and Test Examples After Provisioning

After onboarding is complete, run the application examples from the main project README:

- [Run the Examples](readme.md#run-the-examples)

---

[Back to Main README](readme.md)
