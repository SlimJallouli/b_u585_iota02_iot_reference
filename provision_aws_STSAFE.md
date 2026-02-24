# AWS IoT STSAFE Provisioning for STM32U585 (MAR, JITP, JITR)

This guide covers **STSAFE-based AWS IoT provisioning** for STM32U585 platforms, including:

- **Multi-Account Registration (MAR)**
- **Just-in-Time Provisioning (JITP)**
- **Just-in-Time Registration (JITR)**

STSAFE provisioning stores sensitive assets (device certificate, private key, and configuration) in secure hardware for stronger device identity protection.

## Supported Build Configurations

| Build Config | Provisioning Method |
|---|---|
| `MXCHIP_STSAFEA110` | MAR, JITP, JITR |
| `MXCHIP_STSAFEA120` | MAR, JITP, JITR |

## 1. Hardware Setup

- For `MXCHIP_*` profiles: connect the Wi-Fi module to `STMod+`.
- Connect the STSAFE expansion board based on your configuration:
  - [X-NUCLEO-SAFEA1](https://www.st.com/en/ecosystems/x-nucleo-safea1.html)
  - [X-NUCLEO-ESE01A1](https://www.st.com/en/ecosystems/x-nucleo-ese01a1.html)
- For all profiles: connect ST-Link USB to your PC for power, flashing, and debugging.

## 2. Multi-Account Registration (MAR)

[Multi-Account Registration (MAR)](https://aws.amazon.com/about-aws/whats-new/2020/04/simplify-iot-device-registration-and-easily-move-devices-between-aws-accounts-with-aws-iot-core-multi-account-registration/) enables secure onboarding and movement of devices across AWS accounts. With STSAFE, identity material remains protected in hardware.

Setup guide:

- [STM32 AWS MAR Reference](https://github.com/stm32-hotspot/stm32mcu_aws_mar)

## 3. Just-in-Time Provisioning (JITP)

[Just-in-Time Provisioning (JITP)](https://aws.amazon.com/blogs/iot/setting-up-just-in-time-provisioning-with-aws-iot-core/) automatically provisions devices on first connection to AWS IoT Core. STSAFE-backed credentials strengthen trust during bootstrap and onboarding.

Setup guide:

- [AWS IoT Core JITP Guide](https://aws.amazon.com/blogs/iot/setting-up-just-in-time-provisioning-with-aws-iot-core/)

## 4. Just-in-Time Registration (JITR)

[Just-in-Time Registration (JITR)](https://aws.amazon.com/blogs/iot/just-in-time-registration-of-device-certificates-on-aws-iot/) automatically registers device certificates on first AWS IoT connection. STSAFE securely stores keys/certificates used during registration and runtime authentication.

Setup guide:

- [AWS IoT Core JITR Guide](https://aws.amazon.com/blogs/iot/just-in-time-registration-of-device-certificates-on-aws-iot/)

## 5. Run the Examples

After provisioning, continue with:

- [Run the Examples](readme.md#run-the-examples)

---

[Back to Main README](readme.md)
