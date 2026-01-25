# Provision single device with AWS using CLI

[Single Thing Provisioning](https://docs.aws.amazon.com/iot/latest/developerguide/single-thing-provisioning.html) is a method used to provision individual IoT devices in AWS IoT Core. This method is ideal for scenarios where you need to provision devices one at a time.

In this method, you have two options: automated using a Python script or manual.  
**This document describes the manual method using the [AWS CLI](./Common/cli/ReadMe.md) running on STM32.**

This provisioning method is supported by the following project configurations:

| Build Config | Provisioning method       |
|:------------ |:--------------------------|
| Ethernet     | Single Thing Provisioning |
| MXCHIP       | Single Thing Provisioning |
| ST67_T01     | Single Thing Provisioning |
| ST67_T02     | Single Thing Provisioning |

## 1. Hardware Setup

If you’ve selected the MXCHIP, ST67_T01 or ST67_T02 configuration, connect the Wi-Fi module to either the STMod+ or Arduino connector on the board.

If you’re using the Ethernet configuration, connect the Ethernet cable to the board’s Ethernet port.

Then, in all cases, connect the board to your PC via the ST-Link USB port to power it and enable programming/debugging.

## 2. Connect with serial terminal

Open a serial terminal (e.g., Tera Term, PuTTY, [Web based](https://googlechromelabs.github.io/serial-terminal/)) at 115200, 8 bits, 1 stop, no parity.

![alt text](assets/TeraTerm_Config.png)

## 3. Get the ThingName

Each board automatically generates a unique Thing Name in the format `stm32h573-< DeviceUID >`, where `< DeviceUID >` corresponds to the device's hardware Unique ID (UID).  
For example: `stm32h573-002C005B3332511738363236`.

You can retrieve the Thing Name using the CLI. Save this device ID for further use. You can always retrieve it using the `conf get` command.

Type the following command on the serial terminal:

```
conf get
```

![alt text](assets/conf_get.png)

## 4. Generate key pair

Once the command is executed, the system generates an **ECC** key pair using the **MbedTLS** and **PKCS#11** libraries running on the host microcontroller. The key pair is stored in internal Flash via the **LFS** and **PKCS#11** stack.

Upon success, the public key is printed to the terminal, confirming the device is ready to generate a **CSR** (Certificate Signing Request) for further provisioning or certificate issuance.

Type the following command on the serial terminal:

```
pki generate key
```

![alt text](assets/pki_generate_key.png)

## 5. Generate a certificate

Use the following command in the serial terminal to generate a certificate:

```
pki generate cert
```

![alt text](assets/pki_generate_cert.png)

This command uses **MbedTLS** and **PKCS#11** running on the host microcontroller to create a self-signed certificate from the device’s key pair. The certificate is stored in internal Flash via the **LFS** and **PKCS#11** stack.

Upon success, the certificate will be printed in PEM format to the terminal.

Copy and save the certificate as **cert.pem**.

## 6. Register the device with AWS IoT Core

Now that you have your device's unique Thing Name and its certificate, follow these steps to register your device in AWS IoT Core:

### a. Open the AWS IoT Console

1. Go to the [AWS IoT Core Console](https://console.aws.amazon.com/iot).
2. In the left menu, select **Manage** > **Things**.
3. Click **Create things**.

### b. Create a single thing

1. Choose **Create a single thing**.
2. Enter the **Thing Name** you retrieved in step 3 (for example, `stm32h573-002C005B3332511738363236`).
3. Click **Next**.

### c. Attach the device certificate

1. On the **Configure device certificate** page, select **Use my certificate**.
2. For **CA certificate**, select **CA is not registered with AWS IoT**.
3. Upload the certificate you saved in step 5 (**cert.pem**).
4. Click **Next**.

### d. Attach a policy

1. Click **Create a policy**.
2. Name the policy (for example, `AllowAllDev`).
3. Copy and paste the following policy document:

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:*",
      "Resource": "*"
    }
  ]
}
```

4. Create the policy and attach it to your Thing.

### e. Finish

1. Review your settings and click **Create thing**.
2. Your device is now registered with AWS IoT Core and ready to connect using the provisioned certificate and Thing Name.

---

## 7. Download the server root CA certificate

Download the AWS root CA certificate:

```sh
wget https://www.amazontrust.com/repository/SFSRootCAG2.pem
```

Or download it manually from [SFSRootCAG2.pem](https://www.amazontrust.com/repository/SFSRootCAG2.pem).

## 8. Import the AWS server root CA certificate to STM32

Import the AWS server root CA to STM32 so it can be used for TLS authentication.

- On the serial terminal connected to your board, type the following CLI command:

```
pki import cert root_ca_cert
```

- Open the **SFSRootCAG2.pem** file you downloaded in a text editor (such as Notepad, VS Code, or nano).

- Copy the entire contents, including the `-----BEGIN CERTIFICATE-----` and `-----END CERTIFICATE-----` lines.

- Paste the content into the serial terminal where your board is running and press Enter.

The board will verify the certificate and securely store it in internal Flash using **MbedTLS**, **PKCS#11**, and **LFS** libraries. If everything is successful, you’ll see a confirmation message in the terminal.

![alt text](assets/pki_import_root_ca.png)

## 9. Set runtime configuration

- Set the endpoint. You can find your AWS IoT endpoint address in the AWS IoT Console under **Settings**. Type the following command, replacing `<your-aws-endpoint>` with your actual endpoint:

```
conf set mqtt_endpoint <your-aws-endpoint>
```

- Set the MQTT port:

```
conf set mqtt_port 8883
```

- For MXCHIP, ST67_T01 or ST67_T02, set the Wi-Fi SSID and password:

```
conf set wifi_ssid <YourSSID>
conf set wifi_credential <YourPASSWORD>
```

- For ST67_T01, set the MQTT security. This configuration is required to enable TLS mutual authentication:

```
conf set mqtt_security 4
```

- Commit the changes:

```
conf commit
```

![alt text](assets/aws_conf_commit.png)

- Use *conf get* to confirm your configuration.
- Use *conf set <key> <value>* to make updates.
- Use *conf commit* to save configuration updates.

![alt text](assets/aws_conf_get.png)

## 10. Delete old certificates from ST67 internal file system

If you are using the ST67_T01 configuration, ensure that all previously stored certificates, especially **corePKCS11_CA_Cert.dat**, **corePKCS11_Cert.dat**, and **corePKCS11_Key.dat**, are removed from the module’s internal file system before importing new ones.

- List files:

```
w6x_fs ls
```

![alt text](assets/w6x_fs_ls.png)

- Delete a file:

```
w6x_fs rm <filename>
```

![alt text](assets/w6x_fs_rm.png)

## 11. Reset the board

Type the following command:

```
reset
```

The device will reboot and use the newly imported TLS client certificate and configuration to securely connect to the MQTT broker.

For standard configurations, the host microcontroller handles the TLS and MQTT stack directly.

For the ST67_T01 configuration, after each boot the firmware checks for the presence of **corePKCS11_CA_Cert.dat**, **corePKCS11_Cert.dat**, and **corePKCS11_Key.dat** in the ST67 internal file system. If any are missing, the firmware copies them from the microcontroller’s internal file system to ST67.

Once connected, you should see confirmation messages in the terminal indicating a successful TLS handshake and MQTT session establishment.

![alt text](assets/mqtt_connection.png)

## 12. Monitor the MQTT messages

You can monitor MQTT messages from your device using the AWS IoT MQTT test client available in the AWS Console.

### Steps

1. Go to the [AWS IoT Core Console](https://console.aws.amazon.com/iot).
2. In the left menu, select **MQTT test client**.
3. Under **Subscribe to a topic**, enter the topic your device is publishing to (for example, `stm32h573-002C005B3332511738363236/#` or `#`).
4. Click **Subscribe**.

You should now see messages published by your device appear in real time in the test client.

![alt text](assets/aws_mqtt_test_client.png)

## 13. Run and Test the Examples

After provisioning your board, you can run and test the application features. Refer to the **Run and Test the Examples** section in the main README for details.

---

[⬅️ Back to Main README - Run and Test the Examples](readme.md#7-run-and-test-the-examples)
