/**
  ******************************************************************************
  * @file    ranging_sensor.c
  * @brief   VL53L5CX ranging + garage door detection (fixed hysteresis)
  ******************************************************************************
  */
#include "main.h"
#if DEMO_RANGING_SENSOR
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "logging_levels.h"
#define LOG_LEVEL LOG_INFO
#include "logging.h"

#include "custom_bus_os.h"
#include "custom_errno.h"
#include "vl53l5cx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "sys_evt.h"

#include "core_mqtt.h"
#include "core_mqtt_agent.h"
#include "mqtt_agent_task.h"
#include "kvstore.h"

/* -------------------------------------------------------------------------- */
/* Door state                                                                 */
/* -------------------------------------------------------------------------- */

typedef enum {
    DOOR_STATE_UNKNOWN = 0,
    DOOR_STATE_OPEN,
    DOOR_STATE_CLOSED
} door_state_t;

/* Exported so cover_task.c can read it */
door_state_t gDoorState = DOOR_STATE_UNKNOWN;

/* -------------------------------------------------------------------------- */
/* Thresholds                                                                 */
/* -------------------------------------------------------------------------- */

#define POLLING_PERIOD_MS          1000
#define POLLING_PERIOD             pdMS_TO_TICKS(POLLING_PERIOD_MS)
#define DISTANCE_REPORT_STEP_MM    5U /* 5 mm */
#define DISTANCE_REPORT_MAX_MM     1900U
#define MQTT_MAX_TOPIC_LENGTH      96
#define MQTT_PAYLOAD_LENGTH        128
#define MQTT_PUBLISH_BLOCK_MS      500

/* Fixed hysteresis */
#define DOOR_OPEN_THRESHOLD_MM     500   /* 50 cm */
#define DOOR_CLOSE_THRESHOLD_MM    800   /* 80 cm */

/* -------------------------------------------------------------------------- */
/* Sensor objects                                                             */
/* -------------------------------------------------------------------------- */

static VL53L5CX_ProfileConfig_t Profile;
static VL53L5CX_Object_t *pVL53L5CX_Obj = NULL;
static VL53L5CX_Result_t distance;
static MQTTAgentHandle_t xMQTTAgentHandle = NULL;
static char publish_topic[ MQTT_MAX_TOPIC_LENGTH ];

/* -------------------------------------------------------------------------- */
/* Prototypes                                                                 */
/* -------------------------------------------------------------------------- */

static int32_t VL53L5CX_Probe(VL53L5CX_Object_t **ppVL53L5CXObj);
static BaseType_t xInitSensors(void);
static BaseType_t xUpdateSensorData(VL53L5CX_Result_t *pxData);
static uint32_t GetCenterDistance(const VL53L5CX_Result_t *r);

#if USE_RANGING_SENSOR
static void ProcessDoorState(uint32_t distance_mm);
#endif

static MQTTStatus_t prvPublishToTopic(MQTTQoS_t xQoS,
                                      bool xRetain,
                                      char *pcTopic,
                                      uint8_t *pucPayload,
                                      size_t xPayloadLength);
static void prvPublishCommandCallback(MQTTAgentCommandContext_t *pxCommandContext,
                                      MQTTAgentReturnInfo_t *pxReturnInfo);

struct MQTTAgentCommandContext
{
    TaskHandle_t xTaskToNotify;
    void *pArgs;
};

/* -------------------------------------------------------------------------- */
/* Init                                                                       */
/* -------------------------------------------------------------------------- */

static BaseType_t xInitSensors(void)
{
    int32_t result = VL53L5CX_OK;
    uint32_t chipId = 0;

    LogInfo("Initializing VL53L5CX");

    result = VL53L5CX_Probe(&pVL53L5CX_Obj);

    if (result == VL53L5CX_OK)
        result = VL53L5CX_ReadID(pVL53L5CX_Obj, &chipId);

    if (chipId != VL53L5CX_ID)
        result = VL53L5CX_ERROR;

    if (result == VL53L5CX_OK)
    {
        Profile.RangingProfile = VL53L5CX_PROFILE_4x4_CONTINUOUS;
        Profile.TimingBudget   = 30;
        Profile.Frequency      = 5;
        Profile.EnableAmbient  = 0;
        Profile.EnableSignal   = 0;

        result = VL53L5CX_ConfigProfile(pVL53L5CX_Obj, &Profile);
    }

    if (result == VL53L5CX_OK)
        result = VL53L5CX_Start(pVL53L5CX_Obj, VL53L5CX_MODE_BLOCKING_CONTINUOUS);

    return (result == VL53L5CX_OK) ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

static BaseType_t xUpdateSensorData(VL53L5CX_Result_t *pxData)
{
    return (VL53L5CX_GetDistance(pVL53L5CX_Obj, pxData) == VL53L5CX_OK)
           ? BSP_ERROR_NONE
           : BSP_ERROR_COMPONENT_FAILURE;
}

/* -------------------------------------------------------------------------- */
/* Task                                                                       */
/* -------------------------------------------------------------------------- */

void vRangingSensorTask(void *pvParameters)
{
    (void)pvParameters;

    int32_t result = xInitSensors();
    uint32_t distance_mm = 0;
#if USE_RANGING_SENSOR
    uint32_t last_distance = 0;
#endif
    uint32_t last_reported_distance = 0;
    BaseType_t has_reported_distance = pdFALSE;
    char payload[ MQTT_PAYLOAD_LENGTH ];
    MQTTQoS_t xQoS = MQTTQoS0;
    bool xRetain = pdTRUE;
    char *pThingName = NULL;
    size_t uxTempSize = 0;

    LogInfo("Ranging sensor task started");
    vSleepUntilMQTTAgentReady();
    xMQTTAgentHandle = xGetMqttAgentHandle();
    configASSERT(xMQTTAgentHandle != NULL);

    pThingName = KVStore_getStringHeap(CS_CORE_THING_NAME, &uxTempSize);
    configASSERT(pThingName != NULL);

    snprintf(publish_topic, MQTT_MAX_TOPIC_LENGTH, "%s/sensor/ranging/reported", pThingName);
    vPortFree(pThingName);

    while (result == BSP_ERROR_NONE)
    {
        result = xUpdateSensorData(&distance);

        if (result == BSP_ERROR_NONE)
        {
            distance_mm = GetCenterDistance(&distance);
            LogDebug("Center distance: %lu mm", distance_mm);

#if USE_RANGING_SENSOR
            if (distance_mm != last_distance)
            {
                last_distance = distance_mm;
                ProcessDoorState(distance_mm);
            }
#endif
            BaseType_t shouldPublish = pdFALSE;

            if (has_reported_distance == pdFALSE)
            {
                shouldPublish = pdTRUE;
            }
            else
            {
                uint32_t delta = (distance_mm > last_reported_distance)
                                 ? (distance_mm - last_reported_distance)
                                 : (last_reported_distance - distance_mm);

                if (delta >= DISTANCE_REPORT_STEP_MM)
                {
                    shouldPublish = pdTRUE;
                }
            }

            if (shouldPublish == pdTRUE)
            {
#if 0
                if (distance_mm > DISTANCE_REPORT_MAX_MM)
                {
                    LogDebug("Skipping distance report above max threshold: %lu mm", distance_mm);
                }
                else
#endif
                  if (xIsMqttAgentConnected() == pdTRUE)
                {
                    LogInfo("Distance report: %lu mm", distance_mm);
                    size_t xPayloadLength = (size_t) snprintf(payload,
                                                              MQTT_PAYLOAD_LENGTH,
                                                              "{ \"distance_mm\": %lu }",
                                                              distance_mm);

                    if (xPayloadLength < MQTT_PAYLOAD_LENGTH)
                    {
                        MQTTStatus_t xMQTTStatus = prvPublishToTopic(xQoS,
                                                                      xRetain,
                                                                      publish_topic,
                                                                      (uint8_t *) payload,
                                                                      xPayloadLength);

                        if (xMQTTStatus == MQTTSuccess)
                        {
                            last_reported_distance = distance_mm;
                            has_reported_distance = pdTRUE;
                        }
                        else
                        {
                            LogError("Failed to publish ranging data to: %s", publish_topic);
                        }
                    }
                }
            }
        }

        vTaskDelay(POLLING_PERIOD);
    }

    LogError("Ranging task exiting due to error");
    vTaskDelete(NULL);
}

static void prvPublishCommandCallback(MQTTAgentCommandContext_t *pxCommandContext,
                                      MQTTAgentReturnInfo_t *pxReturnInfo)
{
    if (pxCommandContext->xTaskToNotify != NULL)
    {
        xTaskNotify(pxCommandContext->xTaskToNotify, pxReturnInfo->returnCode, eSetValueWithOverwrite);
    }
}

static MQTTStatus_t prvPublishToTopic(MQTTQoS_t xQoS,
                                      bool xRetain,
                                      char *pcTopic,
                                      uint8_t *pucPayload,
                                      size_t xPayloadLength)
{
    MQTTPublishInfo_t xPublishInfo = { 0UL };
    MQTTAgentCommandContext_t xCommandContext = { 0 };
    MQTTStatus_t xMQTTStatus;
    BaseType_t xNotifyStatus;
    MQTTAgentCommandInfo_t xCommandParams = { 0UL };
    uint32_t ulNotifiedValue = 0U;

    xTaskNotifyStateClear(NULL);

    xPublishInfo.qos = xQoS;
    xPublishInfo.retain = xRetain;
    xPublishInfo.pTopicName = pcTopic;
    xPublishInfo.topicNameLength = (uint16_t) strlen(pcTopic);
    xPublishInfo.pPayload = pucPayload;
    xPublishInfo.payloadLength = xPayloadLength;

    xCommandContext.xTaskToNotify = xTaskGetCurrentTaskHandle();

    xCommandParams.blockTimeMs = MQTT_PUBLISH_BLOCK_MS;
    xCommandParams.cmdCompleteCallback = prvPublishCommandCallback;
    xCommandParams.pCmdCompleteCallbackContext = &xCommandContext;

    do
    {
        xMQTTStatus = MQTTAgent_Publish(xMQTTAgentHandle, &xPublishInfo, &xCommandParams);

        if (xMQTTStatus == MQTTSuccess)
        {
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
    } while (xMQTTStatus != MQTTSuccess);

    return xMQTTStatus;
}

/* -------------------------------------------------------------------------- */
/* Door state logic (fixed hysteresis)                                        */
/* -------------------------------------------------------------------------- */
#if USE_RANGING_SENSOR
static void ProcessDoorState(uint32_t distance_mm)
{
    door_state_t newState = gDoorState;

    switch (gDoorState)
    {
        case DOOR_STATE_CLOSED:
            if (distance_mm < DOOR_OPEN_THRESHOLD_MM)
                newState = DOOR_STATE_OPEN;
            break;

        case DOOR_STATE_OPEN:
            if (distance_mm > DOOR_CLOSE_THRESHOLD_MM)
                newState = DOOR_STATE_CLOSED;
            break;

        case DOOR_STATE_UNKNOWN:
        default:
            newState = (distance_mm < DOOR_OPEN_THRESHOLD_MM)
                       ? DOOR_STATE_OPEN
                       : DOOR_STATE_CLOSED;
            break;
    }

    if (newState != gDoorState)
    {
        gDoorState = newState;

        /* Notify cover task */
        xEventGroupSetBits(xSystemEvents, EVT_DOOR_STATE_CHANGED);

        if (newState == DOOR_STATE_OPEN)
            LogInfo("Door state: OPEN (%lu mm)", distance_mm);
        else
            LogInfo("Door state: CLOSED (%lu mm)", distance_mm);
    }
}
#endif

/* -------------------------------------------------------------------------- */
/* Center distance                                                             */
/* -------------------------------------------------------------------------- */

static uint32_t GetCenterDistance(const VL53L5CX_Result_t *r)
{
    uint8_t side = (r->NumberOfZones == 16) ? 4 : 8;

    uint8_t mid1 = side/2 - 1;
    uint8_t mid2 = side/2;

    uint8_t idxs[4] = {
        mid1*side + mid1,
        mid1*side + mid2,
        mid2*side + mid1,
        mid2*side + mid2
    };

    uint32_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t idx = idxs[i];
        if (r->ZoneResult[idx].NumberOfTargets > 0)
        {
            sum += r->ZoneResult[idx].Distance[0];
            count++;
        }
    }

    return (count == 0) ? 0 : (sum / count);
}

/* -------------------------------------------------------------------------- */
/* Probe                                                                      */
/* -------------------------------------------------------------------------- */

static int32_t VL53L5CX_Probe(VL53L5CX_Object_t **ppVL53L5CXObj)
{
    static VL53L5CX_Object_t VL53L5CXObj;
    VL53L5CX_IO_t IOCtx;
    uint32_t id;

    IOCtx.Address  = VL53L5CX_DEVICE_ADDRESS;
    IOCtx.Init     = BSP_I2C2_Init_OS;
    IOCtx.DeInit   = BSP_I2C2_DeInit_OS;
    IOCtx.WriteReg = BSP_I2C2_WriteReg16_OS;
    IOCtx.ReadReg  = BSP_I2C2_ReadReg16_OS;
    IOCtx.GetTick  = BSP_GetTick;

    if (VL53L5CX_RegisterBusIO(&VL53L5CXObj, &IOCtx) != VL53L5CX_OK)
        return VL53L5CX_ERROR;

    if (VL53L5CX_ReadID(&VL53L5CXObj, &id) != VL53L5CX_OK)
        return VL53L5CX_ERROR;

    if (id != VL53L5CX_ID)
        return VL53L5CX_ERROR;

    if (VL53L5CX_Init(&VL53L5CXObj) != VL53L5CX_OK)
        return VL53L5CX_ERROR;

    *ppVL53L5CXObj = &VL53L5CXObj;
    return VL53L5CX_OK;
}
#endif
