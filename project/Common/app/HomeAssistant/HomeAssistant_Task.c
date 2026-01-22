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
#include "sys_evt.h"


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

/** @brief Maximum length for MQTT topic strings used by Home Assistant. */
# define configMAX_TOPIC_LENGTH                      ( 128 )

/** @brief Topic buffer allocated at runtime for publishing MQTT messages. */
static char *pcPublishTopic = NULL;

/** @brief Delay between MQTT publish operations to avoid flooding the agent. */
#define MQTT_PUBLISH_TIME_BETWEEN_MS                 ( 10 )

/** @brief Max time to block waiting to queue a command to the MQTT agent. */
#define configMAX_COMMAND_SEND_BLOCK_TIME_MS         ( 500 )

/** @brief Size of the payload buffer used for MQTT message bodies. */
#define configPAYLOAD_BUFFER_LENGTH                  ( 1024 )

/** @brief Pointer to the buffer holding the publish topic. */
#define configPUBLISH_TOPIC                   pcPublishTopic

/** @brief Milliseconds in one hour. */
#define MS_PER_HOUR       (60UL * 60UL * 1000UL)
/** @brief Base timeout used for jittered waits in the HA task. */
#define BASE_TIMEOUT_MS   (24UL * MS_PER_HOUR)
/** @brief Jitter range applied to the base timeout. */
#define JITTER_RANGE_MS   (1UL  * MS_PER_HOUR)

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

typedef enum FwUpdateStatus_t
{
    FW_UPDATE_STATUS_IDLE = 0,
    FW_UPDATE_STATUS_UPDATING,
    FW_UPDATE_STATUS_COMPLETED
} FwUpdateStatus_t;

typedef struct EnvSensorDescriptor_t{
    const char *field;
    const char *name;
    const char *unit;
    const char *class;
    const BaseType_t enabled;
} EnvSensorDescriptor_t;

#if (DEMO_LIGHT_SENSOR == 1)
    #define ENV_LUX_ENABLED pdTRUE
#else
    #define ENV_LUX_ENABLED pdFALSE
#endif

static const EnvSensorDescriptor_t xEnvSensors[] = {
    { "temp_0_c"    , "Temperature 0" , "°C"     , "temperature", pdTRUE  },
    { "temp_1_c"    , "Temperature 1" , "°C"     , "temperature", pdTRUE  },
    { "rh_pct"      , "Humidity"      , "%"      , "humidity"   , pdTRUE  },
    { "baro_mbar"   , "Pressure"      , "mbar"   , "pressure"   , pdTRUE  },
    { "white_lux"   , "White Light"   , "lx"     , "illuminance", ENV_LUX_ENABLED },
    { "als_lux"     , "Ambient Light" , "lx"     , "illuminance", ENV_LUX_ENABLED }
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


typedef struct LEDDescriptor_t{
    const char *name;
    const BaseType_t enabled;
} LEDDescriptor_t;

static const LEDDescriptor_t xLEDs[] = {
    { "LED_GREEN", pdTRUE},
    { "LED_RED"  , pdFALSE}
};

typedef struct BUTTONDescriptor_t{
    const char *name;      /* logical name used in JSON and HA; e.g. "USER_Button" */
    const BaseType_t enabled;
} BUTTONDescriptor_t;

static const BUTTONDescriptor_t xBUTTONs[] = {
    { "USER_Button", pdTRUE }
    /* Add more later, e.g.: { "BUTTON2", pdTRUE } */
};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/*-----------------------------------------------------------*/

/** @brief MQTT agent handle shared by the Home Assistant task. */
static MQTTAgentHandle_t xMQTTAgentHandle = NULL;


/** @brief Event group used to track HA commands and OTA state. */
EventGroupHandle_t xHAEventGroup;

#if (DEMO_AWS_OTA == 1)
/** @brief Target firmware version advertised during OTA. */
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

/**
 * @brief Subscribes the MQTT agent to a topic derived from the Thing name.
 *
 * This function builds a topic string by concatenating the provided Thing name
 * with a topic suffix (for example, "cmd/action" or "fw/update"), then issues
 * a synchronous subscribe command to the MQTT agent.
 *
 * The subscription is performed with QoS0. On success, the MQTT agent will
 * invoke the provided callback whenever a message is received on the
 * constructed topic. If the subscribe operation fails, an error is logged
 * including a human readable context string.
 *
 * A temporary topic buffer is allocated from the heap, used for the subscribe
 * call, and freed before returning. A small delay
 * (MQTT_PUBLISH_TIME_BETWEEN_MS) is inserted after the subscribe to respect
 * timing constraints with the MQTT agent and underlying network stack.
 *
 * @param[in] xMQTTAgentHandle Handle to the MQTT agent instance used to
 *            perform the subscribe operation. Must not be NULL.
 * @param[in] pcThingName NULL-terminated string representing the Thing name
 *            (root of the topic hierarchy).
 * @param[in] pcTopicSuffix NULL-terminated suffix appended to the Thing name
 *            to form the full topic (e.g., "cmd/action", "fw/update").
 * @param[in] xCallback Callback invoked by the MQTT agent when a message is
 *            received on the subscribed topic.
 * @param[in] pcLogContext NULL-terminated string used only for logging to
 *            identify the type of subscription (e.g., "reboot command",
 *            "FW update").
 *
 * @return pdPASS if the subscription was successfully issued to the MQTT
 *         broker; pdFAIL if an error occurred (e.g., subscribe failure,
 *         invalid handle).
 */
static BaseType_t xSubscribeToTopic( MQTTAgentHandle_t xMQTTAgentHandle,
                                     const char *pcThingName,
                                     const char *pcTopicSuffix,
                                     IncomingPubCallback_t xCallback,
                                     const char *pcLogContext );
/*-----------------------------------------------------------*/

/**
 * @brief The MQTT agent manages the MQTT contexts.  This set the handle to the
 * context used by this demo.
 */
extern MQTTAgentContext_t xGlobalMqttAgentContext;

/*-----------------------------------------------------------*/
static TickType_t GetJitteredTimeout(void)
{
    // uxRand() returns a 32-bit unsigned random number
    // We want a jitter in the range [-JITTER_RANGE_MS, +JITTER_RANGE_MS]
    int32_t iJitter = (int32_t)(uxRand() % (2 * JITTER_RANGE_MS)) - (int32_t)JITTER_RANGE_MS;

    uint32_t ulFinalTimeoutMs = BASE_TIMEOUT_MS + iJitter;

    // Ensure timeout is at least 1 tick
    if (ulFinalTimeoutMs < 1) {
        ulFinalTimeoutMs = 1;
    }

    return pdMS_TO_TICKS(ulFinalTimeoutMs);
}

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

static void vPublishAvailabilityStatus(const char *pcThingName, char *pcPayloadBuffer, const char *availability);

/*-----------------------------------------------------------*/
static MQTTStatus_t prvClearRetainedTopic(char *pcTopic)
{
  configASSERT(pcTopic != NULL);
  LogInfo(("Clearing retained message on topic: %s", pcTopic));
  return prvPublishToTopic(MQTTQoS0, pdTRUE, pcTopic, NULL, 0);
}

#if ((DEMO_AWS_OTA == 0) || (DEMO_LED == 0) || (DEMO_BUTTON == 0))
static void clearHA_Config(const char *domain, const char *thing, const char *suffix)
{
    snprintf(configPUBLISH_TOPIC, configMAX_TOPIC_LENGTH, "homeassistant/%s/%s_%s/config", domain, thing, suffix);
    prvClearRetainedTopic(configPUBLISH_TOPIC);
    vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
}
#endif

/*-----------------------------------------------------------*/
#if (DEMO_AWS_OTA == 1)
static void publishHAConfig_OTA(const char *pcThingName, char *pcPayloadBuffer)
{
  size_t xPayloadLength = 0;
  MQTTQoS_t xQoS = MQTTQoS0;
  bool xRetain = pdTRUE;
  char * pcFwVersionString = (char*) pvPortMalloc(17);
  configASSERT(pcFwVersionString != NULL);

  memset(pcFwVersionString, 0, 17);

  snprintf(pcFwVersionString, 16, "%d.%d.%d",
           appFirmwareVersion.u.x.major,
           appFirmwareVersion.u.x.minor,
           appFirmwareVersion.u.x.build);

  snprintf(configPUBLISH_TOPIC, configMAX_TOPIC_LENGTH, "homeassistant/update/%s_fw/config", pcThingName);

  xPayloadLength = snprintf(pcPayloadBuffer, configPAYLOAD_BUFFER_LENGTH, "{"
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
      //"\"entity_category\": \"diagnostic\","
      "\"device\": {"
        "\"identifiers\": [\"%s\"],"
        "\"manufacturer\": \"STMicroelectronics\","
        "\"model\": \"%s\","
        "\"name\": \"%s\","
        "\"sw_version\": \"%s\""
      "}"
    "}",
    pcThingName,     // unique_id
    pcThingName,     // state_topic
    pcThingName,     // latest_version_topic
    pcThingName,     // command_topic
    pcThingName,     // availability_topic
    pcThingName,     // identifiers
    BOARD,          // model
    pcThingName,     // name
    pcFwVersionString    // sw_version
  );

  if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
  {
    prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC, (uint8_t*) pcPayloadBuffer, xPayloadLength);
  }
  else
  {
    LogError(("Firmware update payload truncated"));
  }

  vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);

  vPortFree(pcFwVersionString);
}

static const char * fwUpdateStatusToString(FwUpdateStatus_t status)
{
    switch (status)
    {
        case FW_UPDATE_STATUS_IDLE:      return "idle";
        case FW_UPDATE_STATUS_UPDATING:  return "updating";
        case FW_UPDATE_STATUS_COMPLETED: return "completed";
        default:                         return "unknown";
    }
}

static MQTTStatus_t xPublishFirmwareVersionStatus(const AppVersion32_t appFirmwareVersion,
                                                 const AppVersion32_t newAppFirmwareVersion,
                                                 const char *pcThingName,
                                                 FwUpdateStatus_t status)

{
    char pcPayloadBuffer[128];
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

    // Compose JSON payload with status
    const char *statusStr = fwUpdateStatusToString(status);

    msgLen = snprintf(pcPayloadBuffer, sizeof(pcPayloadBuffer),
                      "{\"installed_version\": \"%u.%u.%u\", \"latest_version\": \"%u.%u.%u\", \"status\": \"%s\"}",
                      appFirmwareVersion.u.x.major,
                      appFirmwareVersion.u.x.minor,
                      appFirmwareVersion.u.x.build,
                      newAppFirmwareVersion.u.x.major,
                      newAppFirmwareVersion.u.x.minor,
                      newAppFirmwareVersion.u.x.build,
                      statusStr);


    if (msgLen < 0 || msgLen >= sizeof(pcPayloadBuffer))
    {
        return MQTTBadParameter;
    }

    xStatus = prvPublishToTopic(xQoS, xRetain, cTopicBuf, (uint8_t*) pcPayloadBuffer, msgLen);

    vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);

    return xStatus;
}

static void prvHandleCommand_FwUpdate(void *pxSubscriptionContext, MQTTPublishInfo_t *pPublishInfo)
{
    if (pPublishInfo == NULL || pPublishInfo->pPayload == NULL || pPublishInfo->payloadLength == 0)
    {
        return;
    }

    const char *payload = (const char *)pPublishInfo->pPayload;

    // Ensure payload is null-terminated for comparison
    char tempPayload[32] = {0};  // Adjust size as needed

    /* Clamp to buffer size - 1 to avoid tempPayload overflow and keep space for the null terminator. */
    size_t copyLen = (pPublishInfo->payloadLength < sizeof(tempPayload) - 1) ? pPublishInfo->payloadLength : sizeof(tempPayload) - 1;

    memcpy(tempPayload, payload, copyLen);

    tempPayload[copyLen] = '\0';

    if (strcmp(tempPayload, "start_update") == 0)
    {
        LogInfo("Starting Firmware update.");
        xEventGroupSetBits(xHAEventGroup, EVT_OTA_UPDATE_START);
    }
}
#endif

static BaseType_t xSubscribeToTopic( MQTTAgentHandle_t xMQTTAgentHandle,
                                     const char *pcThingName,
                                     const char *pcTopicSuffix,
                                     IncomingPubCallback_t xCallback,
                                     const char *pcLogContext )
{
    BaseType_t xResult = pdPASS;
    MQTTStatus_t xMQTTStatus;

    char *pcTopicBuffer = pvPortMalloc( configMAX_TOPIC_LENGTH );
    configASSERT( pcTopicBuffer != NULL );

    /* Build full topic: "<thingName>/<suffix>" */
    snprintf( pcTopicBuffer,
              configMAX_TOPIC_LENGTH,
              "%s/%s",
              pcThingName,
              pcTopicSuffix );

    if( ( xResult == pdPASS ) && ( xMQTTAgentHandle != NULL ) )
    {
        xMQTTStatus = MqttAgent_SubscribeSync( xMQTTAgentHandle,
                                               pcTopicBuffer,
                                               MQTTQoS0,
                                               xCallback,
                                               NULL );

        if( xMQTTStatus != MQTTSuccess )
        {
            LogError( "Failed to subscribe to %s topic: %s",
                      pcLogContext,
                      pcTopicBuffer );
            xResult = pdFAIL;
        }
    }

    vTaskDelay( MQTT_PUBLISH_TIME_BETWEEN_MS );

    vPortFree( pcTopicBuffer );

    return xResult;
}
/*-----------------------------------------------------------*/
#if (DEMO_LED == 1)
static void publishHAConfig_LED( const char *pcThingName,
                                 char *pcPayloadBuffer )
{
    size_t    xPayloadLength = 0;
    MQTTQoS_t xQoS           = MQTTQoS0;
    bool      xRetain        = pdTRUE;

    char *commandTopic = ( char * ) pvPortMalloc( configMAX_TOPIC_LENGTH );
    char *stateTopic   = ( char * ) pvPortMalloc( configMAX_TOPIC_LENGTH );
    char  payloadOn[ 32 ];
    char  payloadOff[ 32 ];

    configASSERT( commandTopic != NULL );
    configASSERT( stateTopic   != NULL );

    for( int i = 0; i < ( int ) ARRAY_SIZE( xLEDs ); i++ )
    {
        const LEDDescriptor_t *pxLed = &xLEDs[ i ];

        /* Discovery topic:
         *   homeassistant/switch/<thing>_<LED_NAME>/config
         */
        snprintf( configPUBLISH_TOPIC,
                  configMAX_TOPIC_LENGTH,
                  "homeassistant/switch/%s_%s/config",
                  pcThingName,
                  pxLed->name );

        if( pdTRUE == pxLed->enabled )
        {
            /* Shared command / state topics (no per‑LED suffix): */
            snprintf( commandTopic,
                      configMAX_TOPIC_LENGTH,
                      "%s/led/desired",
                      pcThingName );

            snprintf( stateTopic,
                      configMAX_TOPIC_LENGTH,
                      "%s/led/reported",
                      pcThingName );

            /* Per‑LED command payload values, e.g. "LED_RED_ON", "LED_RED_OFF" */
            snprintf( payloadOn,
                      sizeof( payloadOn ),
                      "%s_ON",
                      pxLed->name );
            snprintf( payloadOff,
                      sizeof( payloadOff ),
                      "%s_OFF",
                      pxLed->name );

            xPayloadLength = snprintf(
                pcPayloadBuffer,
                configPAYLOAD_BUFFER_LENGTH,
                "{"
                  "\"name\": \"%s\","
                  "\"unique_id\": \"%s_%s\","
                  "\"command_topic\": \"%s\","
                  "\"state_topic\": \"%s\","
                  "\"value_template\": \"{{ value_json.ledStatus.%s.reported }}\","
                  "\"payload_on\": \"%s\","
                  "\"payload_off\": \"%s\","
                  "\"state_on\": \"ON\","
                  "\"state_off\": \"OFF\","
                  "\"availability_topic\": \"%s/status/availability\","
                  "\"payload_available\": \"online\","
                  "\"payload_not_available\": \"offline\","
                  "\"retain\": false,"
                  //"\"entity_category\": \"diagnostic\","
                  "\"device\": {"
                    "\"identifiers\": [\"%s\"],"
                    "\"manufacturer\": \"STMicroelectronics\","
                    "\"model\": \"%s\","
                    "\"name\": \"%s\""
                  "}"
                "}",
                pxLed->name,      /* name */
                pcThingName,      /* unique_id prefix  */
                pxLed->name,      /* unique_id suffix  */
                commandTopic,     /* command_topic     */
                stateTopic,       /* state_topic       */
                pxLed->name,      /* value_template    */
                payloadOn,        /* payload_on        */
                payloadOff,       /* payload_off       */
                pcThingName,      /* availability_topic prefix */
                pcThingName,      /* identifiers       */
                BOARD,            /* model             */
                pcThingName       /* device name       */
                );

            if( xPayloadLength < configPAYLOAD_BUFFER_LENGTH )
            {
                prvPublishToTopic( xQoS,
                                   xRetain,
                                   configPUBLISH_TOPIC,
                                   ( uint8_t * ) pcPayloadBuffer,
                                   xPayloadLength );
            }
            else
            {
                LogError( ( "LED %s payload truncated", pxLed->name ) );
            }
        }
        else
        {
            /* Disabled LED: clear its retained config. */
            prvClearRetainedTopic( configPUBLISH_TOPIC );
        }

        vTaskDelay( MQTT_PUBLISH_TIME_BETWEEN_MS );
    }

    vPortFree( commandTopic );
    vPortFree( stateTopic );
}
#endif

/*-----------------------------------------------------------*/
#if (DEMO_BUTTON == 1)
static void publishHAConfig_Button( const char *pcThingName,
                                    char *pcPayloadBuffer )
{
    size_t    xPayloadLength = 0;
    MQTTQoS_t xQoS           = MQTTQoS0;
    bool      xRetain        = pdTRUE;

    for( int i = 0; i < ( int ) ARRAY_SIZE( xBUTTONs ); i++ )
    {
        const BUTTONDescriptor_t *pxButton = &xBUTTONs[ i ];

        /* Discovery topic:
         *   homeassistant/binary_sensor/<thing>_<button_name>/config
         * Example:
         *   homeassistant/binary_sensor/stm32u585-..._USER_Button/config
         */
        snprintf( configPUBLISH_TOPIC,
                  configMAX_TOPIC_LENGTH,
                  "homeassistant/binary_sensor/%s_%s/config",
                  pcThingName,
                  pxButton->name );

        if( pdTRUE == pxButton->enabled )
        {
            /* Single shared reported topic for all buttons:
             *   <thing>/sensor/button/reported
             *
             * The JSON is assumed to look like:
             * {
             *   "buttonStatus": {
             *     "USER_Button": { "reported": "ON" }
             *   }
             * }
             *
             * So the value_template becomes:
             *   {{ value_json.buttonStatus.<name>.reported }}
             */
            xPayloadLength = snprintf(
                pcPayloadBuffer,
                configPAYLOAD_BUFFER_LENGTH,
                "{"
                  "\"name\": \"%s\","
                  "\"unique_id\": \"%s_%s\","
                  "\"state_topic\": \"%s/sensor/button/reported\","
                  "\"value_template\": \"{{ value_json.buttonStatus.%s.reported }}\","
                  "\"payload_on\": \"ON\","
                  "\"payload_off\": \"OFF\","
                  "\"device_class\": \"occupancy\","
                  "\"availability_topic\": \"%s/status/availability\","
                  "\"payload_available\": \"online\","
                  "\"payload_not_available\": \"offline\","
                  "\"retain\": false,"
                  "\"device\": {"
                    "\"identifiers\": [\"%s\"],"
                    "\"manufacturer\": \"STMicroelectronics\","
                    "\"model\": \"%s\","
                    "\"name\": \"%s\""
                  "}"
                "}",
                pxButton->name,      /* name (HA entity name) */
                pcThingName,         /* unique_id prefix  */
                pxButton->name,      /* unique_id suffix  */
                pcThingName,         /* state_topic prefix */
                pxButton->name,      /* value_template button name */
                pcThingName,         /* availability_topic prefix */
                pcThingName,         /* identifiers       */
                BOARD,               /* model             */
                pcThingName          /* device name       */
                );

            if( xPayloadLength < configPAYLOAD_BUFFER_LENGTH )
            {
                prvPublishToTopic( xQoS,
                                   xRetain,
                                   configPUBLISH_TOPIC,
                                   ( uint8_t * ) pcPayloadBuffer,
                                   xPayloadLength );
            }
            else
            {
                LogError( ( "Button %s payload truncated", pxButton->name ) );
            }
        }
        else
        {
            /* Disabled button: clear its retained config. */
            prvClearRetainedTopic( configPUBLISH_TOPIC );
        }

        vTaskDelay( MQTT_PUBLISH_TIME_BETWEEN_MS );
    }
}
#endif

/*-----------------------------------------------------------*/
#if (DEMO_ENV_SENSOR == 1)
static void publishHAConfig_EnvSensors(const char *pcThingName, char *pcPayloadBuffer)
{
  MQTTQoS_t xQoS = MQTTQoS0;
  bool xRetain = pdTRUE;

    for (int i = 0; i < ARRAY_SIZE(xEnvSensors); i++)
    {
        snprintf(configPUBLISH_TOPIC, configMAX_TOPIC_LENGTH, "homeassistant/sensor/%s_%s/config", pcThingName, xEnvSensors[i].field);

        if(pdTRUE == xEnvSensors[i].enabled)
        {
          size_t xPayloadLength = snprintf(pcPayloadBuffer, configPAYLOAD_BUFFER_LENGTH, "{"
              "\"name\": \"%s\","
              "\"unique_id\": \"%s_%s\","
              "\"state_topic\": \"%s/sensor/env\","
              "\"value_template\": \"{{ value_json.%s }}\","
              "\"unit_of_measurement\": \"%s\","
              "\"device_class\": \"%s\","
              "\"availability_topic\": \"%s/status/availability\","
              "\"payload_available\": \"online\","
              "\"payload_not_available\": \"offline\","
              "\"retain\": false,"
              "\"device\": {"
              "\"identifiers\": [\"%s\"],"
              "\"manufacturer\": \"STMicroelectronics\","
              "\"model\": \"%s\","
              "\"name\": \"%s\""
              "}"
              "}",
              xEnvSensors[i].name,   // name
              pcThingName,           // unique_id (prefix)
              xEnvSensors[i].field,  // unique_id (suffix)
              pcThingName,           // state_topic prefix
              xEnvSensors[i].field,  // value_template field
              xEnvSensors[i].unit,   // unit_of_measurement
              xEnvSensors[i].class,  // device_class
              pcThingName,           // availability_topic
              pcThingName,           // identifiers
              BOARD,                 // model
              pcThingName);          // name

          if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
          {
            prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC, (uint8_t *)pcPayloadBuffer, xPayloadLength);
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
#else
static void clearHAConfig_EnvSensors(const char *pcThingName)
{
    for (int i = 0; i < ARRAY_SIZE(xEnvSensors); i++)
    {
        snprintf(configPUBLISH_TOPIC, configMAX_TOPIC_LENGTH, "homeassistant/sensor/%s_%s/config", pcThingName, xEnvSensors[i].field);

        prvClearRetainedTopic(configPUBLISH_TOPIC);
        vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
    }
}
#endif

/*-----------------------------------------------------------*/
#if (DEMO_MOTION_SENSOR == 1)
static void publishHAConfig_MotionSensors(const char *pcThingName, char *pcPayloadBuffer)
{
    MQTTQoS_t xQoS = MQTTQoS0;
    bool xRetain = pdTRUE;

    for (int i = 0; i < ARRAY_SIZE(xMotionSensors); i++)
    {
        snprintf(configPUBLISH_TOPIC, configMAX_TOPIC_LENGTH, "homeassistant/sensor/%s_%s_%s/config", pcThingName, xMotionSensors[i].root, xMotionSensors[i].axis);

        if(pdTRUE == xMotionSensors[i].enabled)
        {
          size_t xPayloadLength = snprintf(pcPayloadBuffer, configPAYLOAD_BUFFER_LENGTH, "{"
            "\"name\": \"%s %s\","
            "\"unique_id\": \"%s_%s_%s\","
            "\"state_topic\": \"%s/sensor/motion\","
            "\"value_template\": \"{{ value_json.%s.%s }}\","
            "\"unit_of_measurement\": \"%s\","
            "\"availability_topic\": \"%s/status/availability\","
            "\"payload_available\": \"online\","
            "\"payload_not_available\": \"offline\","
            "\"retain\": false,"
            "\"device\": {"
            "\"identifiers\": [\"%s\"],"
            "\"manufacturer\": \"STMicroelectronics\","
            "\"model\": \"%s\","
            "\"name\": \"%s\""
            "}"
            "}",
            xMotionSensors[i].label, // name label
            xMotionSensors[i].axis,  // name axis
            pcThingName,             // unique_id prefix
            xMotionSensors[i].root,  // unique_id root
            xMotionSensors[i].axis,  // unique_id axis
            pcThingName,             // state_topic prefix
            xMotionSensors[i].root,  // value_template root
            xMotionSensors[i].axis,  // value_template axis
            xMotionSensors[i].unit,  // unit_of_measurement
            pcThingName,             // availability_topic
            pcThingName,             // identifiers
            BOARD,                   // model
            pcThingName);            // name

          if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
          {
            prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC, (uint8_t *)pcPayloadBuffer, xPayloadLength);
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
#else
static void clearHAConfig_MotionSensors(const char *pcThingName)
{
    for (int i = 0; i < ARRAY_SIZE(xMotionSensors); i++)
    {
        snprintf(configPUBLISH_TOPIC, configMAX_TOPIC_LENGTH, "homeassistant/sensor/%s_%s_%s/config", pcThingName, xMotionSensors[i].root, xMotionSensors[i].axis);

        prvClearRetainedTopic(configPUBLISH_TOPIC);
        vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
    }
}
#endif

/*-----------------------------------------------------------*/
static void vPublishAvailabilityStatus(const char *pcThingName, char *pcPayloadBuffer, const char *availability)
{
    MQTTQoS_t xQoS = MQTTQoS0;
    bool xRetain = pdTRUE;

    snprintf(configPUBLISH_TOPIC, configMAX_TOPIC_LENGTH, "%s/status/availability", pcThingName);

    // Copy the provided availability string ("online" or "offline") into the payload buffer
    strncpy(pcPayloadBuffer, availability, configPAYLOAD_BUFFER_LENGTH - 1);
    pcPayloadBuffer[configPAYLOAD_BUFFER_LENGTH - 1] = '\0'; // Ensure null-termination

    size_t xPayloadLength = strlen(pcPayloadBuffer);

    if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
    {
        prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC, (uint8_t *)pcPayloadBuffer, xPayloadLength);
    }
    else
    {
        LogError(("availability update payload truncated"));
    }

    vTaskDelay(MQTT_PUBLISH_TIME_BETWEEN_MS);
}

static void prvHandleCommand_Reboot(void *pxSubscriptionContext, MQTTPublishInfo_t *pPublishInfo)
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

    // Accept either raw string "reboot" or JSON-style {"action":"reboot"}
    if (strcmp(tempPayload, "reboot") == 0 || strstr(tempPayload, "\"reboot\"") != NULL)
    {
        LogInfo("Reboot command received. Setting EVT_COMMAND_RESET flag.");
        xEventGroupSetBits(xHAEventGroup, EVT_COMMAND_RESET);
    }
    else
    {
        LogWarn("Received unknown reboot command: %s", tempPayload);
    }
}

static void publishHAConfig_RebootButton(const char *pcThingName, char *pcPayloadBuffer)
{
  size_t xPayloadLength = 0;
  MQTTQoS_t xQoS = MQTTQoS0;
  bool xRetain = pdTRUE;

  snprintf(configPUBLISH_TOPIC, configMAX_TOPIC_LENGTH, "homeassistant/button/%s_reboot/config", pcThingName);

  xPayloadLength = snprintf(pcPayloadBuffer, configPAYLOAD_BUFFER_LENGTH, "{"
      "\"name\": \"Reboot\","
      "\"unique_id\": \"%s_reboot\","
      "\"command_topic\": \"%s/cmd/action\","
      "\"payload_press\": \"{\\\"action\\\":\\\"reboot\\\"}\","
      "\"retain\": false,"
      "\"availability_topic\": \"%s/status/availability\","
      "\"payload_available\": \"online\","
      "\"payload_not_available\": \"offline\","
      //"\"entity_category\": \"diagnostic\","
      "\"device\": {"
        "\"identifiers\": [\"%s\"],"
        "\"manufacturer\": \"STMicroelectronics\","
        "\"model\": \"%s\","
        "\"name\": \"%s\""
      "}"
    "}",
    pcThingName, // unique_id
    pcThingName, // command_topic
    pcThingName, // availability_topic
    pcThingName, // identifiers
    BOARD,      // model
    pcThingName  // name
  );

  if (xPayloadLength < configPAYLOAD_BUFFER_LENGTH)
  {
    prvPublishToTopic(xQoS, xRetain, configPUBLISH_TOPIC, (uint8_t*) pcPayloadBuffer, xPayloadLength);
  }
  else
  {
    LogError(("Reboot button payload truncated"));
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

  /* Load device identity and allocate shared publish buffers. */
  char *pcThingName = KVStore_getStringHeap(CS_CORE_THING_NAME, &uxThingNameLen);
  configASSERT(pcThingName != NULL);

  /* Reusable payload buffer for MQTT publishes. */
  char *pcPayloadBuffer = (char*) pvPortMalloc(configPAYLOAD_BUFFER_LENGTH);
  configASSERT(pcPayloadBuffer != NULL);

  /* Reusable payload buffer for MQTT publish topics. */
  pcPublishTopic = (char*) pvPortMalloc(configMAX_TOPIC_LENGTH);
  configASSERT(pcPublishTopic != NULL);

  LogInfo(("Publishing Home Assistant discovery configuration for device: %s", pcThingName));

  xHAEventGroup = xEventGroupCreate();
  configASSERT(xHAEventGroup != NULL);

#if (DEMO_AWS_OTA == 1)
  xSubscribeToTopic( xMQTTAgentHandle,
                     pcThingName,
                     "fw/update",
                     prvHandleCommand_FwUpdate,
                     "FW update" );

  publishHAConfig_OTA(pcThingName, pcPayloadBuffer);
#else
  clearHA_Config("update", pcThingName, "fw");
#endif

#if (DEMO_LED == 1)
  publishHAConfig_LED(pcThingName, pcPayloadBuffer);
#else
  clearHA_Config("switch", pcThingName, "led");
#endif

#if (DEMO_BUTTON == 1)
  publishHAConfig_Button(pcThingName, pcPayloadBuffer);
#else
  clearHA_Config("binary_sensor", pcThingName, "button");
#endif

#if (DEMO_ENV_SENSOR == 1)
  publishHAConfig_EnvSensors(pcThingName, pcPayloadBuffer);
#else
  clearHAConfig_EnvSensors(pcThingName);
#endif

#if (DEMO_MOTION_SENSOR == 1)
  publishHAConfig_MotionSensors(pcThingName, pcPayloadBuffer);
#else
  clearHAConfig_MotionSensors(pcThingName);
#endif

  /* Listen for reboot commands and publish reboot control. */
  xSubscribeToTopic( xMQTTAgentHandle,
                     pcThingName,
                     "cmd/action",
                     prvHandleCommand_Reboot,
                     "reboot command" );

  publishHAConfig_RebootButton(pcThingName, pcPayloadBuffer);

  vTaskDelay(1000);

#if (DEMO_AWS_OTA == 1)
  newAppFirmwareVersion.u.x.major = appFirmwareVersion.u.x.major;
  newAppFirmwareVersion.u.x.minor = appFirmwareVersion.u.x.minor;
  newAppFirmwareVersion.u.x.build = appFirmwareVersion.u.x.build;

  /* Send current firmware version */
  xPublishFirmwareVersionStatus(appFirmwareVersion, newAppFirmwareVersion, pcThingName, FW_UPDATE_STATUS_COMPLETED);
#endif

  /* Send availability message  */
  vPublishAvailabilityStatus(pcThingName, pcPayloadBuffer, "online");

  LogInfo("HomeAssistant config completed.");

  while (1)
  {
    /* Wait for HA/OTA events and react (OTA availability, start, completion, reboot). */
    EventBits_t uxBits = xEventGroupWaitBits(xHAEventGroup,
#if (DEMO_AWS_OTA == 1)
                                             EVT_OTA_UPDATE_AVAILABLE |
                                             EVT_OTA_UPDATE_START     |
                                             EVT_OTA_COMPLETED        |
#endif
                                             EVT_COMMAND_RESET,
                                             pdTRUE,
                                             pdFALSE,
                                             GetJitteredTimeout());
#if (DEMO_AWS_OTA == 1)
    if ((uxBits & EVT_OTA_UPDATE_AVAILABLE) != 0)
    {
      LogInfo("New Firmware available.");
      xPublishFirmwareVersionStatus(appFirmwareVersion, newAppFirmwareVersion, pcThingName, FW_UPDATE_STATUS_IDLE);
    }

    if ((uxBits & EVT_OTA_UPDATE_START) != 0)
    {
      LogInfo("Firmware upload starting...");
      xPublishFirmwareVersionStatus(appFirmwareVersion, newAppFirmwareVersion, pcThingName, FW_UPDATE_STATUS_UPDATING);
      vPublishAvailabilityStatus(pcThingName, pcPayloadBuffer, "offline");
    }

    if (uxBits & EVT_OTA_COMPLETED)
    {
      LogInfo("New Firmware uploaded");
      LogInfo("Generating reset to install New Firmware");
      vDoSystemReset();
    }
#endif

    if ((uxBits & EVT_COMMAND_RESET) || (0 == uxBits))
    {
      LogInfo("Reboot command");
      vPublishAvailabilityStatus(pcThingName, pcPayloadBuffer, "offline");
      vTaskDelay(1000);
      vDoSystemReset();
    }
  }
}

