/*
 * cover_task.c
 *
 * Home Assistant MQTT cover integration using relay pulse outputs.
 */

#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "sys_evt.h"

#include "core_mqtt.h"
#include "core_mqtt_agent.h"
#include "mqtt_agent_task.h"
#include "subscription_manager.h"
#include "kvstore.h"
#include "core_json.h"

#include "interrupt_handlers.h"

#if (USE_MAGNETIC_SENSOR)&&(USE_RANGING_SENSOR)
#error please select either USE_MAGNETIC_SENSOR or USE_RANGING_SENSOR
#endif

#if (USE_RANGING_SENSOR) && (NUM_COVERS > 1)
#error only one cover is allowed when using the USE_RANGING_SENSOR
#endif

/* -------------------------------------------------------------------------- */
/* Cover model                                                                */
/* -------------------------------------------------------------------------- */

typedef enum
{
    eCOVER_STATE_OPEN,
    eCOVER_STATE_CLOSED,
    eCOVER_STATE_STOPPED,
    eCOVER_STATE_UNKNOWN
} eCoverState_t;

typedef struct
{
    uint16_t      usPin;
    GPIO_TypeDef *pxPort;
} Relay_t;

#if USE_MAGNETIC_SENSOR
typedef struct
{
    GPIO_TypeDef *pxPort;
    uint16_t      usPin;
    GPIO_PinState xOpenState;
} DoorSensor_t;
#endif

typedef struct
{
    const char        *pcName;
    const Relay_t      xRelay;
#if USE_MAGNETIC_SENSOR
    const DoorSensor_t xSensor;
#endif
    eCoverState_t      eStateReported;
} Cover_t;

#if USE_RANGING_SENSOR
typedef enum
{
    eDOOR_STATE_UNKNOWN = 0,
    eDOOR_STATE_OPEN,
    eDOOR_STATE_CLOSED
} eDoorState_t;

extern volatile const eDoorState_t gDoorState;
#endif

/* -------------------------------------------------------------------------- */
/* Static cover table                                                         */
/* -------------------------------------------------------------------------- */

static Cover_t xCovers[] =
{
#if (NUM_COVERS > 0)
    {
        .pcName = "GARAGE_DOOR_1",
        .xRelay = {
            .usPin      = RELAY_1_Pin,
            .pxPort     = RELAY_1_Port
        },
    #if USE_MAGNETIC_SENSOR
        .xSensor = {
            .pxPort     = DOOR_SENSPR_1_Port,
            .usPin      = DOOR_SENSPR_1_Pin,
            .xOpenState = DOOR_SENSPR_1_STATE_OPEN
        },
    #endif
        .eStateReported = eCOVER_STATE_UNKNOWN
    }
#endif

#if (NUM_COVERS > 1)
    ,{
        .pcName = "GARAGE_DOOR_2",
        .xRelay = {
            .usPin      = RELAY_2_Pin,
            .pxPort     = RELAY_2_Port
        },
    #if USE_MAGNETIC_SENSOR
        .xSensor = {
            .pxPort     = DOOR_SENSPR_2_Port,
            .usPin      = DOOR_SENSPR_2_Pin,
            .xOpenState = DOOR_SENSPR_2_STATE_OPEN
        },
    #endif
        .eStateReported = eCOVER_STATE_UNKNOWN
    }
#endif

#if (NUM_COVERS > 2)
    ,{
        .pcName = "GARAGE_DOOR_3",
        .xRelay = {
            .usPin      = RELAY_3_Pin,
            .pxPort     = RELAY_3_Port
        },
    #if USE_MAGNETIC_SENSOR
        .xSensor = {
            .pxPort     = DOOR_SENSPR_3_Port,
            .usPin      = DOOR_SENSPR_3_Pin,
            .xOpenState = DOOR_SENSPR_3_STATE_OPEN
        },
    #endif
        .eStateReported = eCOVER_STATE_UNKNOWN
    }
#endif
};

#define COVER_COUNT ( (uint8_t)( sizeof( xCovers ) / sizeof( xCovers[0] ) ) )

/* -------------------------------------------------------------------------- */
/* MQTT topics                                                                */
/* -------------------------------------------------------------------------- */

#define MAX_TOPIC_LEN 128

static char thingName[64];
static MQTTAgentHandle_t xMQTTAgentHandle = NULL;

/* -------------------------------------------------------------------------- */
/* Command context for synchronous publish                                    */
/* -------------------------------------------------------------------------- */

typedef struct MQTTAgentCommandContext
{
    TaskHandle_t xTaskToNotify;
    void       *pArgs;
} MQTTAgentCommandContext_t;

/* -------------------------------------------------------------------------- */
/* Door sensor                                                                */
/* -------------------------------------------------------------------------- */

static eCoverState_t prvReadDoorSensor(uint8_t ucIndex)
{
#if USE_MAGNETIC_SENSOR
    const DoorSensor_t *pxSensor = &xCovers[ucIndex].xSensor;

    GPIO_PinState xRaw = HAL_GPIO_ReadPin(pxSensor->pxPort, pxSensor->usPin);

    return (xRaw == pxSensor->xOpenState)
           ? eCOVER_STATE_OPEN
           : eCOVER_STATE_CLOSED;

#elif USE_RANGING_SENSOR

    switch (gDoorState)
    {
        case eDOOR_STATE_OPEN:   return eCOVER_STATE_OPEN;
        case eDOOR_STATE_CLOSED: return eCOVER_STATE_CLOSED;
        default:                 return eCOVER_STATE_UNKNOWN;
    }

#else
    (void) ucIndex;
    return eCOVER_STATE_UNKNOWN;
#endif
}

/* -------------------------------------------------------------------------- */
/* Relay pulse and motor actions                                              */
/* -------------------------------------------------------------------------- */

static void prvRelayPulse(const Relay_t *pxRelay)
{
    HAL_GPIO_WritePin(pxRelay->pxPort, pxRelay->usPin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(1000));
    HAL_GPIO_WritePin(pxRelay->pxPort, pxRelay->usPin, GPIO_PIN_RESET);
}

static void prvCoverMotorOpen(Cover_t *pxCover)
{
    prvRelayPulse(&pxCover->xRelay);
}

static void prvCoverMotorClose(Cover_t *pxCover)
{
    prvRelayPulse(&pxCover->xRelay);
}

static void prvCoverMotorStop(Cover_t *pxCover)
{
    prvRelayPulse(&pxCover->xRelay);
}

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

static Cover_t *prvFindCoverByName(const char *pcName)
{
    for(uint8_t i = 0; i < COVER_COUNT; i++)
    {
        if(strcmp(xCovers[i].pcName, pcName) == 0)
            return &xCovers[i];
    }
    return NULL;
}

static const char *prvStateToString(eCoverState_t eState)
{
    switch(eState)
    {
        case eCOVER_STATE_OPEN:    return "open";
        case eCOVER_STATE_CLOSED:  return "closed";
        case eCOVER_STATE_STOPPED: return "stopped";
        default:                   return "unknown";
    }
}

/* -------------------------------------------------------------------------- */
/* Publish command callback                                                   */
/* -------------------------------------------------------------------------- */

static void prvPublishCommandCallback(MQTTAgentCommandContext_t *pxCtx,
                                      MQTTAgentReturnInfo_t     *pxReturn)
{
    if(pxCtx->xTaskToNotify != NULL)
    {
        xTaskNotify(pxCtx->xTaskToNotify,
                    pxReturn->returnCode,
                    eSetValueWithOverwrite);
    }
}

/* -------------------------------------------------------------------------- */
/* Synchronous publish wrapper                                                */
/* -------------------------------------------------------------------------- */

static MQTTStatus_t prvPublishToTopic(MQTTQoS_t xQoS,
                                      bool      bRetain,
                                      char     *pcTopic,
                                      uint8_t  *pucPayload,
                                      size_t    xPayloadLen)
{
    MQTTPublishInfo_t         xPubInfo = {0};
    MQTTAgentCommandInfo_t    xCmdInfo = {0};
    MQTTAgentCommandContext_t xCtx     = {0};
    MQTTStatus_t              xStatus;
    uint32_t                  ulNotifyVal = 0;

    xTaskNotifyStateClear(NULL);

    xPubInfo.qos             = xQoS;
    xPubInfo.retain          = bRetain;
    xPubInfo.pTopicName      = pcTopic;
    xPubInfo.topicNameLength = (uint16_t) strlen(pcTopic);
    xPubInfo.pPayload        = pucPayload;
    xPubInfo.payloadLength   = xPayloadLen;

    xCtx.xTaskToNotify = xTaskGetCurrentTaskHandle();

    xCmdInfo.blockTimeMs                 = 500;
    xCmdInfo.cmdCompleteCallback         = prvPublishCommandCallback;
    xCmdInfo.pCmdCompleteCallbackContext = &xCtx;

    do
    {
        xStatus = MQTTAgent_Publish(xMQTTAgentHandle, &xPubInfo, &xCmdInfo);

        if(xStatus == MQTTSuccess)
        {
            if(xTaskNotifyWait(0, 0, &ulNotifyVal, portMAX_DELAY) == pdTRUE)
                xStatus = (ulNotifyVal == 0) ? MQTTSuccess : MQTTSendFailed;
            else
                xStatus = MQTTSendFailed;
        }
    }
    while(xStatus != MQTTSuccess);

    return xStatus;
}

/* -------------------------------------------------------------------------- */
/* JSON / command parsing                                                     */
/* -------------------------------------------------------------------------- */

static void prvHandleCoverCommand(const char *pcCoverName,
                                  const char *pcCommand)
{
    Cover_t *pxCover = prvFindCoverByName(pcCoverName);

    if(pxCover == NULL)
    {
        LogWarn(("Unknown cover name: %s", pcCoverName));
        return;
    }

    if(strcmp(pcCommand, "OPEN") == 0)
    {
        prvCoverMotorOpen(pxCover);

#if (!USE_MAGNETIC_SENSOR)&&(!USE_RANGING_SENSOR)
        pxCover->eStateReported = eCOVER_STATE_OPEN;
#endif
    }
    else if(strcmp(pcCommand, "CLOSE") == 0)
    {
        prvCoverMotorClose(pxCover);

#if (!USE_MAGNETIC_SENSOR)&&(!USE_RANGING_SENSOR)
        pxCover->eStateReported = eCOVER_STATE_CLOSED;
#endif
    }
    else if(strcmp(pcCommand, "STOP") == 0)
    {
        prvCoverMotorStop(pxCover);

#if (!USE_MAGNETIC_SENSOR)&&(!USE_RANGING_SENSOR)
        pxCover->eStateReported = eCOVER_STATE_STOPPED;
#endif
    }

#if (!USE_MAGNETIC_SENSOR)&&(!USE_RANGING_SENSOR)
    xEventGroupSetBits(xSystemEvents, EVT_DOOR_STATE_CHANGED);
#endif
}

/* -------------------------------------------------------------------------- */
/* Incoming publish callback                                                  */
/* -------------------------------------------------------------------------- */

static void prvIncomingPublishCallback(void *pvCtx,
                                       MQTTPublishInfo_t *pxInfo)
{
    (void) pvCtx;

    char pcTopic[MAX_TOPIC_LEN];
    char pcPayload[64];

    size_t xTopicLen = pxInfo->topicNameLength;
    if(xTopicLen >= sizeof(pcTopic))
        xTopicLen = sizeof(pcTopic) - 1;

    memcpy(pcTopic, pxInfo->pTopicName, xTopicLen);
    pcTopic[xTopicLen] = '\0';

    size_t xPayloadLen = pxInfo->payloadLength;
    if(xPayloadLen >= sizeof(pcPayload))
        xPayloadLen = sizeof(pcPayload) - 1;

    memcpy(pcPayload, pxInfo->pPayload, xPayloadLen);
    pcPayload[xPayloadLen] = '\0';

    const char *pc = strstr(pcTopic, "/cover/");
    if(pc == NULL)
        return;

    pc += strlen("/cover/");
    const char *pcNameStart = pc;
    const char *pcSlash = strchr(pcNameStart, '/');
    if(pcSlash == NULL)
        return;

    char pcCoverName[32];
    size_t xNameLen = (size_t)(pcSlash - pcNameStart);
    if(xNameLen >= sizeof(pcCoverName))
        xNameLen = sizeof(pcCoverName) - 1;

    memcpy(pcCoverName, pcNameStart, xNameLen);
    pcCoverName[xNameLen] = '\0';

    prvHandleCoverCommand(pcCoverName, pcPayload);
}

/* -------------------------------------------------------------------------- */
/* Subscribe                                                                  */
/* -------------------------------------------------------------------------- */

static MQTTStatus_t prvSubscribeToCovers(MQTTQoS_t xQoS)
{
    char pcFilter[MAX_TOPIC_LEN];

    snprintf(pcFilter, sizeof(pcFilter),
             "%s/cover/+/desired", thingName);

    MQTTStatus_t xStatus;

    do
    {
        xStatus = MqttAgent_SubscribeSync(xMQTTAgentHandle,
                                          pcFilter,
                                          xQoS,
                                          prvIncomingPublishCallback,
                                          NULL);

        if(xStatus != MQTTSuccess)
            LogError(("Failed to subscribe: %s", pcFilter));
        else
            LogInfo(("Subscribed: %s", pcFilter));

    } while(xStatus != MQTTSuccess);

    return xStatus;
}

/* -------------------------------------------------------------------------- */
/* State publishing                                                           */
/* -------------------------------------------------------------------------- */

static void prvPublishCoverStates(void)
{
    char pcTopic[MAX_TOPIC_LEN];
    char pcPayload[32];

    for(uint8_t i = 0; i < COVER_COUNT; i++)
    {
        Cover_t *pxCover = &xCovers[i];

        pxCover->eStateReported = prvReadDoorSensor(i);

        snprintf(pcTopic, sizeof(pcTopic),
                 "%s/cover/%s/state",
                 thingName, pxCover->pcName);

        const char *pcState = prvStateToString(pxCover->eStateReported);

        size_t xLen = snprintf(pcPayload, sizeof(pcPayload), "%s", pcState);

        prvPublishToTopic(MQTTQoS1, false,
                          pcTopic,
                          (uint8_t *)pcPayload,
                          xLen);
    }
}

#if USE_MAGNETIC_SENSOR
static void prvDoorSensorInterrupt(void *pvCtx)
{
    (void) pvCtx;
    xEventGroupSetBits(xSystemEvents, EVT_DOOR_STATE_CHANGED);
}

static void prvRegisterDoorInterrupts(void)
{
    for(uint8_t i = 0; i < COVER_COUNT; i++)
    {
        const DoorSensor_t *pxSensor = &xCovers[i].xSensor;

        GPIO_EXTI_Register_Rising_Callback(pxSensor->usPin,
                                           prvDoorSensorInterrupt,
                                           (void *)(uintptr_t)i);

        GPIO_EXTI_Register_Falling_Callback(pxSensor->usPin,
                                            prvDoorSensorInterrupt,
                                            (void *)(uintptr_t)i);
    }
}
#endif

/* -------------------------------------------------------------------------- */
/* Cover task                                                                 */
/* -------------------------------------------------------------------------- */

void vCoverTask(void *pvParams)
{
    (void) pvParams;

    vSleepUntilMQTTAgentReady();
    xMQTTAgentHandle = xGetMqttAgentHandle();
    configASSERT(xMQTTAgentHandle != NULL);

    vSleepUntilMQTTAgentConnected();

    size_t xThingLen = 0;
    char *pcThing = KVStore_getStringHeap(CS_CORE_THING_NAME, &xThingLen);
    configASSERT(pcThing != NULL);

    memcpy(thingName, pcThing, xThingLen);
    thingName[xThingLen] = '\0';
    vPortFree(pcThing);

    LogInfo(("Cover task starting for thing: %s", thingName));

#if USE_MAGNETIC_SENSOR
    prvRegisterDoorInterrupts();
#endif

    prvSubscribeToCovers(MQTTQoS1);

    prvPublishCoverStates();

    for(;;)
    {
        xEventGroupWaitBits(xSystemEvents,
                            EVT_DOOR_STATE_CHANGED,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        for(uint8_t i = 0; i < COVER_COUNT; i++)
        {
            eCoverState_t eSensed = prvReadDoorSensor(i);

            if(eSensed != xCovers[i].eStateReported)
            {
                xCovers[i].eStateReported = eSensed;
                prvPublishCoverStates();
            }
        }
    }
}
