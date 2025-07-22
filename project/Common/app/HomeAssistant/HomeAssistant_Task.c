/*
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 */

/**
 * @brief A test application which loops through subscribing to a topic and publishing message
 * to a topic. This test application can be used with AWS IoT device advisor test suite to
 * verify that an application implemented using MQTT agent follows best practices in connecting
 * to AWS IoT core.
 */
/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <assert.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

/* MQTT library includes. */
#include "core_mqtt.h"

/* MQTT agent include. */
#include "core_mqtt_agent.h"

/* Subscription manager header include. */
#include "subscription_manager.h"

/* MQTT agent task API. */
#include "mqtt_agent_task.h"

#include "kvstore.h"

#include "ota_appversion32.h"

# define MAXT_TOPIC_LENGTH 128
static char publish_topic[MAXT_TOPIC_LENGTH];

#define MQTT_PUBLISH_TIME_BETWEEN_MS         ( 300 )
/**
 * @brief The maximum amount of time in milliseconds to wait for the commands
 * to be posted to the MQTT agent should the MQTT agent's command queue be full.
 * Tasks wait in the Blocked state, so don't use any CPU time.
 */
#define configMAX_COMMAND_SEND_BLOCK_TIME_MS         ( 500 )

/**
 * @brief Size of statically allocated buffers for holding payloads.
 */
#define configPAYLOAD_BUFFER_LENGTH                  ( 1024 )

/**
 * @brief Format of topic used to publish outgoing messages.
 */
#define configPUBLISH_TOPIC                   publish_topic

/**
 * @brief Format of topic used to subscribe to incoming messages.
 *
 */
#define configSUBSCRIBE_TOPIC_FORMAT   configPUBLISH_TOPIC_FORMAT

/*-----------------------------------------------------------*/

/**
 * @brief Defines the structure to use as the command callback context in this
 * demo.
 */
struct MQTTAgentCommandContext
{
  TaskHandle_t xTaskToNotify;
  void *pArgs;
};

typedef struct EnvSensorDescriptor_t{
    const char *field;
    const char *name;
    const char *unit;
    const char *class;
    const BaseType_t enabled;
} EnvSensorDescriptor_t;

static const EnvSensorDescriptor_t xEnvSensors[] = {
    { "temp_0_c"    , "Temperature 0" , "°C"     , "temperature", pdTRUE  },
#if (USE_AVG_TEMP == 0)
    { "temp_1_c"    , "Temperature 1" , "°C"     , "temperature", pdTRUE  },
#else
    { "temp_1_c"    , "Temperature 1" , "°C"     , "temperature", pdFALSE },
#endif
    { "rh_pct"      , "Humidity"      , "%"      , "humidity"   , pdTRUE  },
    { "baro_mbar"   , "Pressure"      , "mbar"   , "pressure"   , pdTRUE  }
};

typedef struct MotionSensorDescriptor_t{
    const char *root;
    const char *label;
    const char *unit;
    const char *axis;
    const BaseType_t enabled;
} MotionSensorDescriptor_t;

static const MotionSensorDescriptor_t xMotionSensors[] = {
    { "acceleration", "Acceleration"  , "mG"     , "x", pdTRUE },
    { "acceleration", "Acceleration"  , "mG"     , "y", pdTRUE },
    { "acceleration", "Acceleration"  , "mG"     , "z", pdTRUE },
    { "gyro"        , "Gyroscope"     , "mDPS"   , "x", pdTRUE },
    { "gyro"        , "Gyroscope"     , "mDPS"   , "y", pdTRUE },
    { "gyro"        , "Gyroscope"     , "mDPS"   , "z", pdTRUE },
    { "magnetometer", "Magnetometer"  , "mGauss" , "x", pdTRUE },
    { "magnetometer", "Magnetometer"  , "mGauss" , "y", pdTRUE },
    { "magnetometer", "Magnetometer"  , "mGauss" , "z", pdTRUE },
};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/*-----------------------------------------------------------*/

static MQTTAgentHandle_t xMQTTAgentHandle = NULL;

static char *pThingName = NULL;
static char *cPayloadBuf = NULL;

#if (DEMO_OTA == 1)
#define OTA_UPDATE_AVAILABLE     (1 << 0)  // New OTA pending
#define OTA_UPDATE_START         (2 << 0)  // Signal to start OTA

EventGroupHandle_t xOtaEventGroup;
volatile AppVersion32_t newAppFirmwareVersion;
#endif

/*-----------------------------------------------------------*/

/**
 * @brief Passed into MQTTAgent_Publish() as the callback to execute when the
 * broker ACKs the PUBLISH message.  Its implementation sends a notification
 * to the task that called MQTTAgent_Publish() to let the task know the
 * PUBLISH operation completed.  It also sets the xReturnStatus of the
 * structure passed in as the command's context to the value of the
 * xReturnStatus parameter - which enables the task to check the status of the
 * operation.
 *
 * See https://freertos.org/mqtt/mqtt-agent-demo.html#example_mqtt_api_call
 *
 * @param[in] pxCommandContext Context of the initial command.
 * @param[in].xReturnStatus The result of the command.
 */
static void prvPublishCommandCallback(MQTTAgentCommandContext_t *pxCommandContext, MQTTAgentReturnInfo_t *pxReturnInfo);

/**
 * @brief Publishes the given payload using the given qos to the topic provided.
 *
 * Function queues a publish command with the MQTT agent and waits for response. For
 * Qos0 publishes command is successful when the message is sent out of network. For Qos1
 * publishes, the command succeeds once a puback is received. If publish is unsuccessful, the function
 * retries the publish for a configure number of tries.
 *
 * @param[in] xQoS The quality of service (QoS) to use.  Can be zero or one
 * for all MQTT brokers.  Can also be QoS2 if supported by the broker.  AWS IoT
 * does not support QoS2.
 * @param[in] pcTopic NULL terminated topic string to which message is published.
 * @param[in] pucPayload The payload blob to be published.
 * @param[in] xPayloadLength Length of the payload blob to be published.
 */
static MQTTStatus_t prvPublishToTopic(MQTTQoS_t xQoS, bool xRetain, char *pcTopic, uint8_t *pucPayload, size_t xPayloadLength);

/**
 * @brief The function that implements the task demonstrated by this file.
 *
 * @param pvParameters The parameters to the task.
 */
void vHAConfigPublishTask(void *pvParameters);

/**
 * @brief Publishes an empty retained message to the given topic to clear the retained message.
 *
 * This function uses the MQTT agent's publish flow to remove a previously retained message
 * on a topic by publishing a zero-length payload with the retain flag set.
 *
 * @param[in] pcTopic The topic from which to clear the retained message.
 *
 * @return MQTTSuccess if the empty publish succeeded, appropriate MQTTStatus_t error otherwise.
 */
static MQTTStatus_t prvClearRetainedTopic(char *pcTopic);

/*-----------------------------------------------------------*/

/**
 * @brief The MQTT agent manages the MQTT contexts.  This set the handle to the
 * context used by this demo.
 */
extern MQTTAgentContext_t xGlobalMqttAgentContext;

/*-----------------------------------------------------------*/

static void prvPublishCommandCallback(MQTTAgentCommandContext_t *pxCommandContext, MQTTAgentReturnInfo_t *pxReturnInfo)
{
  if (pxCommandContext->xTaskToNotify != NULL)
  {
    xTaskNotify(pxCommandContext->xTaskToNotify, pxReturnInfo->returnCode, eSetValueWithOverwrite);
  }
}

/*-----------------------------------------------------------*/

static MQTTStatus_t prvPublishToTopic(MQTTQoS_t xQoS, bool xRetain, char *pcTopic, uint8_t *pucPayload, size_t xPayloadLength)
{
  MQTTPublishInfo_t xPublishInfo = { 0UL };
  MQTTAgentCommandContext_t xCommandContext = { 0 };
  MQTTStatus_t xMQTTStatus;
  BaseType_t xNotifyStatus;
  MQTTAgentCommandInfo_t xCommandParams = { 0UL };
  uint32_t ulNotifiedValue = 0U;

  /* Create a unique number of the subscribe that is about to be sent.  The number
   * is used as the command context and is sent back to this task as a notification
   * in the callback that executed upon receipt of the subscription acknowledgment.
   * That way this task can match an acknowledgment to a subscription. */
  xTaskNotifyStateClear(NULL);

  /* Configure the publish operation. */
  xPublishInfo.qos = xQoS;
  xPublishInfo.retain = xRetain;
  xPublishInfo.pTopicName = pcTopic;
  xPublishInfo.topicNameLength = (uint16_t) strlen(pcTopic);
  xPublishInfo.pPayload = pucPayload;
  xPublishInfo.payloadLength = xPayloadLength;

  xCommandContext.xTaskToNotify = xTaskGetCurrentTaskHandle();

  xCommandParams.blockTimeMs = configMAX_COMMAND_SEND_BLOCK_TIME_MS;
  xCommandParams.cmdCompleteCallback = prvPublishCommandCallback;
  xCommandParams.pCmdCompleteCallbackContext = &xCommandContext;

  /* Loop in case the queue used to communicate with the MQTT agent is full and
   * attempts to post to it time out.  The queue will not become full if the
   * priority of the MQTT agent task is higher than the priority of the task
   * calling this function. */
  do
  {
    xMQTTStatus = MQTTAgent_Publish(xMQTTAgentHandle, &xPublishInfo, &xCommandParams);

    if (xMQTTStatus == MQTTSuccess)
    {
      /* Wait for this task to get notified, passing out the value it gets  notified with. */
      xNotifyStatus = xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);

      if (xNotifyStatus == pdTRUE)
      {
        if (ulNotifiedValue)
        {
          xMQTTStatus = MQTTSendFailed;
        }
        else
        {
          xMQTTStatus = MQTTSuccess;
        }
      }
      else
      {
        xMQTTStatus = MQTTSendFailed;
      }
    }
  }
  while (xMQTTStatus != MQTTSuccess);

  return xMQTTStatus;
}

void publishAvailabilityStatus(const char *pThingName, char *cPayloadBuf, const char *availability);
/*-----------------------------------------------------------*/

static MQTTStatus_t prvClearRetainedTopic(char *pcTopic)
{
  configASSERT(pcTopic != NULL);
  LogInfo(("Clearing retained message on topic: %s", pcTopic));
  return prvPublishToTopic(MQTTQoS0, pdTRUE, pcTopic, NULL, 0);
}

#if ((DEMO_OTA == 0) || (DEMO_LED == 0) || (DEMO_BUTTON == 0) || (DEMO_LIGHT_SENSOR == 0))
static void clearHA_Config(const char *domain, const char *thing, const char *suffix)
{
    snprintf(configPUBLISH_TOPIC, MAXT_TOPIC_LENGTH, "homeassistant/%s/%s_%s/config", domain, thing, suffix);
    prvClearRetainedTopic(configPUBLISH_TOPIC);
    vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
}
#endif

#if !(DEMO_ENV_SENSOR == 1)
void clearEnvSensorConfigs(const char *pThingName)
{
    for (int i = 0; i < ARRAY_SIZE(xEnvSensors); i++)
    {
        snprintf(configPUBLISH_TOPIC, MAXT_TOPIC_LENGTH, "homeassistant/sensor/%s_%s/config", pThingName, xEnvSensors[i].field);

        prvClearRetainedTopic(configPUBLISH_TOPIC);
        vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
    }
}
#endif

#if !(DEMO_MOTION_SENSOR == 1)
void clearMotionSensorConfigs(const char *pThingName)
{
    for (int i = 0; i < ARRAY_SIZE(xMotionSensors); i++)
    {
        snprintf(configPUBLISH_TOPIC, MAXT_TOPIC_LENGTH, "homeassistant/sensor/%s_%s_%s/config", pThingName, xMotionSensors[i].root, xMotionSensors[i].axis);

        prvClearRetainedTopic(configPUBLISH_TOPIC);
        vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
    }
}
#endif

/*-----------------------------------------------------------*/

#if (DEMO_OTA == 1)
static void publishHA_OtaConfig(const char *pThingName, char *cPayloadBuf)
{
  size_t xPayloadLength = 0;
  MQTTQoS_t xQoS = MQTTQoS0;
  bool xRetain = pdTRUE;
  char * fwVersionStr = (char*) pvPortMalloc(17);
  configASSERT(fwVersionStr != NULL);

  memset(fwVersionStr, 0, 17);

  snprintf(fwVersionStr, 16, "%d.%d.%d",
           appFirmwareVersion.u.x.major,
           appFirmwareVersion.u.x.minor,
           appFirmwareVersion.u.x.build);

  snprintf(configPUBLISH_TOPIC, MAXT_TOPIC_LENGTH, "homeassistant/update/%s_fw/config", pThingName);

  xPayloadLength = snprintf(cPayloadBuf, configPAYLOAD_BUFFER_LENGTH, "{"
      "\"name\": \"Firmware\","
      "\"unique_id\": \"%s_fw_update\","
      "\"state_topic\": \"%s/fw/state\","
      "\"value_template\": \"{{ value_json.installed_version }}\","
      "\"latest_version_topic\": \"%s/fw/state\","
      "\"latest_version_template\": \"{{ value_json.latest_version }}\","
      "\"command_topic\": \"%s/fw/update\","
      "\"payload_install\": \"start_update\","
      "\"availability_topic\": \"%s/status/availability\","
      "\"payload_available\": \"online\","
      "\"payload_not_available\": \"offline\","
      "\"retain\": false,"
      "\"device_class\": \"firmware\","
      "\"device\": {"
      "\"identifiers\": [\"%s\"],"
      "\"manufacturer\": \"STMicroelectronics\","
      "\"model\": \"%s\","
      "\"name\": \"%s\","
      "\"sw_version\": \"%s\""
      "}"
      "}",
      pThingName, // unique_id
      pThingName, // state_topic
      pThingName, // latest_version_topic
      pThingName, // command_topic
      pThingName, // availability_topic
      pThingName, // identifiers
      BOARD,      // model
      pThingName, // name
      fwVersionStr// sw_version
      );

  if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
  {
    prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC, (uint8_t*) cPayloadBuf, xPayloadLength);
  }
  else
  {
    LogError(("Firmware update payload truncated"));
  }

  vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
}

static MQTTStatus_t publishFirmwareVersionStatus(const AppVersion32_t appFirmwareVersion, const AppVersion32_t newAppFirmwareVersion, const char * pcThingName)
{
    char cPayloadBuf[128];
    char cTopicBuf[64];
    int msgLen = 0;
    MQTTQoS_t xQoS = MQTTQoS0;
    bool xRetain = pdTRUE;

    MQTTStatus_t xStatus = MQTTBadParameter;

    // Compose topic: <ThingName>/fw/state
    msgLen = snprintf(cTopicBuf, sizeof(cTopicBuf), "%s/fw/state", pcThingName);

    if (msgLen < 0 || msgLen >= sizeof(cTopicBuf))
    {
        return MQTTBadParameter;
    }

    // Compose JSON payload
    msgLen = snprintf(cPayloadBuf, sizeof(cPayloadBuf),
                      "{\"installed_version\": \"%u.%u.%u\", \"latest_version\": \"%u.%u.%u\"}",
                      appFirmwareVersion.u.x.major,
                      appFirmwareVersion.u.x.minor,
                      appFirmwareVersion.u.x.build,
                      newAppFirmwareVersion.u.x.major,
                      newAppFirmwareVersion.u.x.minor,
                      newAppFirmwareVersion.u.x.build);

    if (msgLen < 0 || msgLen >= sizeof(cPayloadBuf))
    {
        return MQTTBadParameter;
    }

    prvPublishToTopic(xQoS, xRetain, cTopicBuf, (uint8_t*) cPayloadBuf, msgLen);

    vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);

    return xStatus;
}

static void prvHandleFwUpdateCommand(void *pxSubscriptionContext, MQTTPublishInfo_t *pPublishInfo)
{
    if (pPublishInfo == NULL || pPublishInfo->pPayload == NULL || pPublishInfo->payloadLength == 0)
    {
        return;
    }

    const char *payload = (const char *)pPublishInfo->pPayload;

    // Ensure payload is null-terminated for comparison
    char tempPayload[32] = {0};  // Adjust size as needed

    size_t copyLen = (pPublishInfo->payloadLength < sizeof(tempPayload) - 1) ? pPublishInfo->payloadLength : sizeof(tempPayload) - 1;

    memcpy(tempPayload, payload, copyLen);

    tempPayload[copyLen] = '\0';

    if (strcmp(tempPayload, "start_update") == 0)
    {
        LogInfo("Starting Firmware update.");
        xEventGroupSetBits(xOtaEventGroup, OTA_UPDATE_START);
    }
}

static BaseType_t subscribeToFwUpdateTopic(MQTTAgentHandle_t xMQTTAgentHandle, const char *pcThingName)
{
    BaseType_t xResult = pdPASS;
    MQTTStatus_t xMQTTStatus;

    char topicBuf[128];
    snprintf(topicBuf, sizeof(topicBuf), "%s/fw/update", pcThingName);

    if ((xResult == pdPASS) && (xMQTTAgentHandle != NULL))
    {
        xMQTTStatus = MqttAgent_SubscribeSync(xMQTTAgentHandle, topicBuf, MQTTQoS0, prvHandleFwUpdateCommand, NULL );

        if (xMQTTStatus != MQTTSuccess)
        {
            LogError("Failed to subscribe to FW update topic: %s", topicBuf);
            xResult = pdFAIL;
        }
    }

    vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);

    return xResult;
}
#endif

#if (DEMO_LED == 1)
static void publishHA_LedConfig(const char *pThingName, char *cPayloadBuf)
{
  size_t xPayloadLength = 0;
  MQTTQoS_t xQoS = MQTTQoS0;
  bool xRetain = pdTRUE;

  snprintf(configPUBLISH_TOPIC, MAXT_TOPIC_LENGTH, "homeassistant/switch/%s_led/config", pThingName);

  xPayloadLength = snprintf(cPayloadBuf, configPAYLOAD_BUFFER_LENGTH, "{"
      "\"name\": \"LED\","
      "\"unique_id\": \"%s_led\","
      "\"command_topic\": \"%s/led/desired\","
      "\"state_topic\": \"%s/led/reported\","
      "\"value_template\": \"{{ value_json.ledStatus.reported }}\","
      "\"payload_on\": \"ON\","
      "\"payload_off\": \"OFF\","
      "\"state_on\": \"ON\","
      "\"state_off\": \"OFF\","
      "\"availability_topic\": \"%s/status/availability\","
      "\"payload_available\": \"online\","
      "\"payload_not_available\": \"offline\","
      "\"retain\": true,"
      "\"device\": {"
      "\"identifiers\": [\"%s\"],"
      "\"manufacturer\": \"STMicroelectronics\","
      "\"model\": \"%s\","
      "\"name\": \"%s\""
      "}"
      "}",
      pThingName, // unique_id
      pThingName, // command_topic
      pThingName, // state_topic
      pThingName, // availability_topic
      pThingName, // identifiers
      BOARD,      // model
      pThingName  // name
      );

  if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
  {
    prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC, (uint8_t*) cPayloadBuf, xPayloadLength);
  }
  else
  {
    LogError(("LED payload truncated"));
  }

  vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
}
#endif

#if (DEMO_BUTTON == 1)
static void publishHA_ButtonConfig(const char *pThingName, char *cPayloadBuf)
{
  size_t xPayloadLength = 0;
  MQTTQoS_t xQoS = MQTTQoS0;
  bool xRetain = pdTRUE;

  snprintf(configPUBLISH_TOPIC, MAXT_TOPIC_LENGTH, "homeassistant/binary_sensor/%s_button/config", pThingName);

  xPayloadLength = snprintf(cPayloadBuf, configPAYLOAD_BUFFER_LENGTH, "{"
      "\"name\": \"Button\","
      "\"unique_id\": \"%s_button\","
      "\"state_topic\": \"%s/sensor/button/reported\","
      "\"value_template\": \"{{ value_json.buttonStatus.reported }}\","
      "\"payload_on\": \"ON\","
      "\"payload_off\": \"OFF\","
      "\"device_class\": \"occupancy\","
      "\"availability_topic\": \"%s/status/availability\","
      "\"payload_available\": \"online\","
      "\"payload_not_available\": \"offline\","
      "\"retain\": true,"
      "\"device\": {"
      "\"identifiers\": [\"%s\"],"
      "\"manufacturer\": \"STMicroelectronics\","
      "\"model\": \"%s\","
      "\"name\": \"%s\""
      "}"
      "}",
      pThingName, // unique_id
      pThingName, // state_topic
      pThingName, // availability_topic
      pThingName, // identifiers
      BOARD,      // model
      pThingName  // name
      );

  if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
  {
    prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC, (uint8_t*) cPayloadBuf, xPayloadLength);
  }
  else
  {
    LogError(("Button payload truncated"));
  }

  vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
}
#endif

#if (DEMO_LIGHT_SENSOR == 1)
static void publishHA_LuxSensorConfig(const char *pThingName, char *cPayloadBuf)
{
  size_t xPayloadLength = 0;
  MQTTQoS_t xQoS = MQTTQoS0;
  bool xRetain = pdTRUE;

  snprintf(configPUBLISH_TOPIC, MAXT_TOPIC_LENGTH,
           "homeassistant/sensor/%s_lux_sensor/config", pThingName);

  xPayloadLength = snprintf(cPayloadBuf, configPAYLOAD_BUFFER_LENGTH, "{"
      "\"name\": \"Ambient Light\","
      "\"unique_id\": \"%s_lux_sensor\","
      "\"state_topic\": \"%s/sensor/lux_sensor\","
      "\"value_template\": \"{{ value_json.lux }}\","
      "\"device_class\": \"illuminance\","
      "\"unit_of_measurement\": \"lx\","
      "\"availability_topic\": \"%s/status/availability\","
      "\"payload_available\": \"online\","
      "\"payload_not_available\": \"offline\","
      "\"retain\": true,"
      "\"device\": {"
      "\"identifiers\": [\"%s\"],"
      "\"manufacturer\": \"STMicroelectronics\","
      "\"model\": \"%s\","
      "\"name\": \"%s\""
      "}"
      "}",
      pThingName,           // unique_id
      pThingName,           // state_topic
      pThingName,           // availability_topic
      pThingName,           // identifiers
      BOARD,                // model
      pThingName);

  if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
  {
    prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC,
                      (uint8_t*) cPayloadBuf, xPayloadLength);
  }
  else
  {
    LogError(("Lux payload truncated"));
  }

  vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
}
#endif

#if (DEMO_ENV_SENSOR == 1)
void publishEnvSensorConfigs(const char *pThingName, char *cPayloadBuf)
{
  MQTTQoS_t xQoS = MQTTQoS0;
  bool xRetain = pdTRUE;

    for (int i = 0; i < ARRAY_SIZE(xEnvSensors); i++)
    {
        snprintf(configPUBLISH_TOPIC, MAXT_TOPIC_LENGTH, "homeassistant/sensor/%s_%s/config", pThingName, xEnvSensors[i].field);

        if(pdTRUE == xEnvSensors[i].enabled)
        {
          size_t xPayloadLength = snprintf(cPayloadBuf, configPAYLOAD_BUFFER_LENGTH, "{"
              "\"name\": \"%s\","
              "\"unique_id\": \"%s_env_%d\","
              "\"state_topic\": \"%s/sensor/env\","
              "\"value_template\": \"{{ value_json.%s }}\","
              "\"unit_of_measurement\": \"%s\","
              "\"device_class\": \"%s\","
              "\"availability_topic\": \"%s/status/availability\","
              "\"payload_available\": \"online\","
              "\"payload_not_available\": \"offline\","
              "\"retain\": true,"
              "\"device\": {"
              "\"identifiers\": [\"%s\"],"
              "\"manufacturer\": \"STMicroelectronics\","
              "\"model\": \"%s\","
              "\"name\": \"%s\""
              "}"
              "}",
              xEnvSensors[i].name,
              pThingName,
              i,
              pThingName,
              xEnvSensors[i].field,
              xEnvSensors[i].unit,
              xEnvSensors[i].class,
              pThingName,
              pThingName,
              BOARD,
              pThingName);

          if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
          {
            prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC, (uint8_t *)cPayloadBuf, xPayloadLength);
          }
          else
          {
            LogError(("Env sensor %d payload truncated", i));
          }
        }
        else
        {
          prvClearRetainedTopic(configPUBLISH_TOPIC);
        }

        vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
    }
}
#endif

#if (DEMO_MOTION_SENSOR == 1)
void publishMotionSensorConfigs(const char *pThingName, char *cPayloadBuf)
{
    MQTTQoS_t xQoS = MQTTQoS0;
    bool xRetain = pdTRUE;

    for (int i = 0; i < ARRAY_SIZE(xMotionSensors); i++)
    {
        snprintf(configPUBLISH_TOPIC, MAXT_TOPIC_LENGTH, "homeassistant/sensor/%s_%s_%s/config", pThingName, xMotionSensors[i].root, xMotionSensors[i].axis);

        if(pdTRUE == xMotionSensors[i].enabled)
        {
          size_t xPayloadLength = snprintf(cPayloadBuf, configPAYLOAD_BUFFER_LENGTH, "{"
            "\"name\": \"%s %s\","
            "\"unique_id\": \"%s_%s_%s\","
            "\"state_topic\": \"%s/sensor/motion\","
            "\"value_template\": \"{{ value_json.%s.%s }}\","
            "\"unit_of_measurement\": \"%s\","
            "\"availability_topic\": \"%s/status/availability\","
            "\"payload_available\": \"online\","
            "\"payload_not_available\": \"offline\","
            "\"retain\": true,"
            "\"device\": {"
            "\"identifiers\": [\"%s\"],"
            "\"manufacturer\": \"STMicroelectronics\","
            "\"model\": \"%s\","
            "\"name\": \"%s\""
            "}"
            "}",
            xMotionSensors[i].label,
            xMotionSensors[i].axis,
            pThingName,
            xMotionSensors[i].root,
            xMotionSensors[i].axis,
            pThingName,
            xMotionSensors[i].root,
            xMotionSensors[i].axis,
            xMotionSensors[i].unit,
            pThingName,
            pThingName,
            BOARD,
            pThingName);

          if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
          {
            prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC, (uint8_t *)cPayloadBuf, xPayloadLength);
          }
          else
          {
            LogError(("Motion sensor %s %s payload truncated", xMotionSensors[i].label, xMotionSensors[i].axis));
          }
        }
        else
        {
          prvClearRetainedTopic(configPUBLISH_TOPIC);
        }

        vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
    }
}
#endif

void publishAvailabilityStatus(const char *pThingName, char *cPayloadBuf, const char *availability)
{
    MQTTQoS_t xQoS = MQTTQoS0;
    bool xRetain = pdTRUE;

    snprintf(configPUBLISH_TOPIC, MAXT_TOPIC_LENGTH, "%s/status/availability", pThingName);

    // Copy the provided availability string ("online" or "offline") into the payload buffer
    strncpy(cPayloadBuf, availability, configPAYLOAD_BUFFER_LENGTH - 1);
    cPayloadBuf[configPAYLOAD_BUFFER_LENGTH - 1] = '\0'; // Ensure null-termination

    size_t xPayloadLength = strlen(cPayloadBuf);

    if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
    {
        prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC, (uint8_t *)cPayloadBuf, xPayloadLength);
    }
    else
    {
        LogError(("availability update payload truncated"));
    }

    vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
}

/*-----------------------------------------------------------*/

void vHAConfigPublishTask(void *pvParameters)
{
  size_t uxThingNameLen = 0;

  /* Wait until the MQTT agent is ready */
  vSleepUntilMQTTAgentReady();

  /* Get the MQTT Agent handle */
  xMQTTAgentHandle = xGetMqttAgentHandle();
  configASSERT(xMQTTAgentHandle != NULL);

  /* Wait until we are connected to AWS */
  vSleepUntilMQTTAgentConnected();

  pThingName = KVStore_getStringHeap(CS_CORE_THING_NAME, &uxThingNameLen);
  configASSERT(pThingName != NULL);

  cPayloadBuf = (char*) pvPortMalloc(configPAYLOAD_BUFFER_LENGTH);
  configASSERT(cPayloadBuf != NULL);

  LogInfo(("Publishing Home Assistant discovery configuration for device: %s", pThingName));

#if (DEMO_OTA == 1)
  xOtaEventGroup = xEventGroupCreate();
  configASSERT(xOtaEventGroup != NULL);

  newAppFirmwareVersion.u.x.major = appFirmwareVersion.u.x.major;
  newAppFirmwareVersion.u.x.minor = appFirmwareVersion.u.x.minor;
  newAppFirmwareVersion.u.x.build = appFirmwareVersion.u.x.build;

  subscribeToFwUpdateTopic(xMQTTAgentHandle, pThingName);

  publishHA_OtaConfig(pThingName, cPayloadBuf);

  publishFirmwareVersionStatus(appFirmwareVersion, newAppFirmwareVersion, pThingName);
#endif

#if (DEMO_LED == 1)
  publishHA_LedConfig(pThingName, cPayloadBuf);
#else
  clearHA_Config("switch", pThingName, "led");
#endif

#if (DEMO_BUTTON == 1)
  publishHA_ButtonConfig(pThingName, cPayloadBuf);
#else
  clearHA_Config("binary_sensor", pThingName, "button");
#endif

#if (DEMO_LIGHT_SENSOR == 1)
  publishHA_LuxSensorConfig(pThingName, cPayloadBuf);
#else
  clearHA_Config("sensor", pThingName, "lux_sensor");
#endif

#if (DEMO_ENV_SENSOR == 1)
  publishEnvSensorConfigs(pThingName, cPayloadBuf);
#else
  clearEnvSensorConfigs(pThingName);
#endif

#if (DEMO_MOTION_SENSOR == 1)
  publishMotionSensorConfigs(pThingName, cPayloadBuf);
#else
  clearMotionSensorConfigs(pThingName);
#endif

  /* Send availability message  */
  publishAvailabilityStatus(pThingName, cPayloadBuf, "online");

#if (DEMO_OTA == 1)
  LogInfo("Discovery config task completed.");

  while(1)
  {
    xEventGroupWaitBits(
        xOtaEventGroup,
        OTA_UPDATE_AVAILABLE,                       // Bit to wait for
        pdTRUE,                                     // Clear the bit on exit
        pdFALSE,                                    // Wait for any bit (just one in this case)
        portMAX_DELAY                               // Timeout after delay period
    );

    LogInfo("New Firmware available.");
    publishFirmwareVersionStatus(appFirmwareVersion, newAppFirmwareVersion, pThingName);

    xEventGroupWaitBits(
        xOtaEventGroup,
        OTA_UPDATE_START,                           // Bit to wait for
        pdTRUE,                                     // Clear the bit on exit
        pdFALSE,                                    // Wait for any bit (just one in this case)
        portMAX_DELAY                               // Timeout after delay period
    );

    publishAvailabilityStatus(pThingName, cPayloadBuf, "offline");
    vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
  }
#else
  vPortFree(cPayloadBuf);
  vPortFree(pThingName);
  LogInfo("Discovery config task completed. Deleting itself.");
  vTaskDelete(NULL);
#endif
}
