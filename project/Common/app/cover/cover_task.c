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

#include "core_mqtt.h"
#include "core_mqtt_agent.h"
#include "mqtt_agent_task.h"
#include "subscription_manager.h"
#include "kvstore.h"
#include "core_json.h"

/* -------------------------------------------------------------------------- */
/* Cover model                                                                */
/* -------------------------------------------------------------------------- */

typedef enum
{
    COVER_STATE_OPEN,
    COVER_STATE_CLOSED,
    COVER_STATE_STOPPED,
    COVER_STATE_UNKNOWN
} CoverState_t;

typedef struct
{
    const char  *name;          /* "GARAGE_DOOR_1" etc. */
    uint8_t      relayIndex;    /* index into relay[]   */
    CoverState_t stateReported; /* last reported state  */
} CoverDescriptor_t;

/* Relay hardware descriptor (provided elsewhere) */
typedef struct
{
    uint16_t      GPIO_Pin;
    GPIO_TypeDef *GPIO_Port;
} relay_t;

static relay_t relay[] = {
#if (NUM_COVERS>0)
                          {RELAY_1_Pin, RELAY_1_Port}
#endif
#if (NUM_COVERS>1)
                          ,{RELAY_2_Pin, RELAY_2_Port}
#endif
#if (NUM_COVERS>2)
                          ,{RELAY_3_Pin, RELAY_3_Port}
#endif
                          };

#define RELAY_COUNT (sizeof(relay) / sizeof(relay[0]))

/* Map covers to relays: 3 doors → first 3 relays */
static CoverDescriptor_t xCovers[] =
{
#if (NUM_COVERS>0)
     { "GARAGE_DOOR_1", 0, COVER_STATE_UNKNOWN }
#endif
#if (NUM_COVERS>1)
    ,{ "GARAGE_DOOR_2", 1, COVER_STATE_UNKNOWN }
#endif
#if (NUM_COVERS>2)
    ,{ "GARAGE_DOOR_3", 2, COVER_STATE_UNKNOWN }
#endif

};

#define COVER_COUNT ( (uint8_t)( sizeof( xCovers ) / sizeof( xCovers[0] ) ) )

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    GPIO_PinState openState;   /* GPIO_PIN_SET or GPIO_PIN_RESET */
} DoorSensor_t;

static DoorSensor_t doorSensors[] =
{
#if USE_DOOR_SENSPR
#if (NUM_COVERS>0)
    { DOOR_SENSPR_1_Port, DOOR_SENSPR_1_Pin, DOOR_SENSPR_1_STATE_OPEN }
#endif
#if (NUM_COVERS>1)
    ,{ DOOR_SENSPR_2_Port, DOOR_SENSPR_2_Pin, DOOR_SENSPR_2_STATE_OPEN }
#endif
#if (NUM_COVERS>2)
    ,{ DOOR_SENSPR_3_Port, DOOR_SENSPR_3_Pin, DOOR_SENSPR_3_STATE_OPEN }
#endif
#endif
};


#define DOOR_SENSOR_COUNT (sizeof(doorSensors) / sizeof(doorSensors[0]))

/* Polling interval for door sensors */
const TickType_t xPollRate = pdMS_TO_TICKS(200);  /* 5 Hz */

/* -------------------------------------------------------------------------- */
/* MQTT topics                                                                */
/* -------------------------------------------------------------------------- */

#define MAX_TOPIC_LEN 128

static char thingName[64]; /* cached thing name */
static MQTTAgentHandle_t xMQTTAgentHandle = NULL;

static EventGroupHandle_t xCoverEventGroup;
#define COVER_STATUS_CHANGED_EVENT   (1 << 0)

/* We’ll publish state per cover, so we don’t need a single global topic buffer.
 * We build topics on the fly in the publish path. */

/* -------------------------------------------------------------------------- */
/* Command context for synchronous publish                                    */
/* -------------------------------------------------------------------------- */

typedef struct MQTTAgentCommandContext
{
    TaskHandle_t xTaskToNotify;
    void       * pArgs;
} MQTTAgentCommandContext_t;

/* -------------------------------------------------------------------------- */
/* Door sensor                                                                */
/* -------------------------------------------------------------------------- */

static CoverState_t prvReadDoorSensor(uint8_t index)
{
    if (index >= DOOR_SENSOR_COUNT)
        return COVER_STATE_UNKNOWN;

    GPIO_PinState raw = HAL_GPIO_ReadPin(doorSensors[index].port,
                                         doorSensors[index].pin);

    if (raw == doorSensors[index].openState)
        return COVER_STATE_OPEN;
    else
        return COVER_STATE_CLOSED;
}

/* -------------------------------------------------------------------------- */
/* Relay pulse and motor actions                                              */
/* -------------------------------------------------------------------------- */

static void Relay_Pulse( uint8_t index )
{
    if( index >= RELAY_COUNT )
    {
        return;
    }

    HAL_GPIO_WritePin( relay[ index ].GPIO_Port, relay[ index ].GPIO_Pin, GPIO_PIN_SET );
    HAL_Delay( 1000 ); /* 1 second pulse */
    HAL_GPIO_WritePin( relay[ index ].GPIO_Port, relay[ index ].GPIO_Pin, GPIO_PIN_RESET );
}

static void CoverMotor_Open( CoverDescriptor_t *pxCover )
{
    Relay_Pulse( pxCover->relayIndex );
}

static void CoverMotor_Close( CoverDescriptor_t *pxCover )
{
    Relay_Pulse( pxCover->relayIndex );
}

static void CoverMotor_Stop( CoverDescriptor_t *pxCover )
{
    Relay_Pulse( pxCover->relayIndex );
}

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

static CoverDescriptor_t * prvFindCoverByName( const char *pcName )
{
    for( uint8_t i = 0; i < COVER_COUNT; i++ )
    {
        if( strcmp( xCovers[ i ].name, pcName ) == 0 )
        {
            return &xCovers[ i ];
        }
    }
    return NULL;
}

static const char * prvStateToString( CoverState_t state )
{
    switch( state )
    {
        case COVER_STATE_OPEN:    return "open";
        case COVER_STATE_CLOSED:  return "closed";
        case COVER_STATE_STOPPED: return "stopped"; /* not in HA schema, but useful */
        default:                  return "unknown";
    }
}

/* -------------------------------------------------------------------------- */
/* Publish command callback                                                   */
/* -------------------------------------------------------------------------- */

static void prvPublishCommandCallback( MQTTAgentCommandContext_t *pxCommandContext,
                                       MQTTAgentReturnInfo_t     *pxReturnInfo )
{
    if( pxCommandContext->xTaskToNotify != NULL )
    {
        xTaskNotify( pxCommandContext->xTaskToNotify,
                     pxReturnInfo->returnCode,
                     eSetValueWithOverwrite );
    }
}

/* -------------------------------------------------------------------------- */
/* Synchronous publish wrapper                                                */
/* -------------------------------------------------------------------------- */

static MQTTStatus_t prvPublishToTopic( MQTTQoS_t   xQoS,
                                       bool        xRetain,
                                       char       *pcTopic,
                                       uint8_t    *pucPayload,
                                       size_t      xPayloadLength )
{
    MQTTPublishInfo_t         xPublishInfo  = { 0 };
    MQTTAgentCommandInfo_t    xCommandInfo  = { 0 };
    MQTTAgentCommandContext_t xCommandCtx   = { 0 };
    MQTTStatus_t              xMQTTStatus;
    BaseType_t                xNotifyStatus;
    uint32_t                  ulNotifiedVal = 0;

    xTaskNotifyStateClear( NULL );

    xPublishInfo.qos              = xQoS;
    xPublishInfo.retain           = xRetain;
    xPublishInfo.pTopicName       = pcTopic;
    xPublishInfo.topicNameLength  = ( uint16_t ) strlen( pcTopic );
    xPublishInfo.pPayload         = pucPayload;
    xPublishInfo.payloadLength    = xPayloadLength;

    xCommandCtx.xTaskToNotify     = xTaskGetCurrentTaskHandle();
    xCommandCtx.pArgs             = NULL;

    xCommandInfo.blockTimeMs              = 500;
    xCommandInfo.cmdCompleteCallback      = prvPublishCommandCallback;
    xCommandInfo.pCmdCompleteCallbackContext = &xCommandCtx;

    do
    {
        xMQTTStatus = MQTTAgent_Publish( xMQTTAgentHandle,
                                         &xPublishInfo,
                                         &xCommandInfo );

        if( xMQTTStatus == MQTTSuccess )
        {
            xNotifyStatus = xTaskNotifyWait( 0, 0, &ulNotifiedVal, portMAX_DELAY );

            if( xNotifyStatus == pdTRUE )
            {
                xMQTTStatus = ( ulNotifiedVal == 0 ) ? MQTTSuccess : MQTTSendFailed;
            }
            else
            {
                xMQTTStatus = MQTTSendFailed;
            }
        }
    }
    while( xMQTTStatus != MQTTSuccess );

    return xMQTTStatus;
}

/* -------------------------------------------------------------------------- */
/* JSON / command parsing                                                     */
/* -------------------------------------------------------------------------- */

static void prvHandleCoverCommand( const char *pcCoverName,
                                   const char *pcCommand )
{
    CoverDescriptor_t *pxCover = prvFindCoverByName( pcCoverName );
    BaseType_t         anyChanged = pdFALSE;

    if( pxCover == NULL )
    {
        LogWarn( ( "Unknown cover name in topic: %s", pcCoverName ) );
        return;
    }

    if( strcmp( pcCommand, "OPEN" ) == 0 )
    {
        CoverMotor_Open( pxCover );
        pxCover->stateReported = COVER_STATE_OPEN;
        anyChanged = pdTRUE;
    }
    else if( strcmp( pcCommand, "CLOSE" ) == 0 )
    {
        CoverMotor_Close( pxCover );
        pxCover->stateReported = COVER_STATE_CLOSED;
        anyChanged = pdTRUE;
    }
    else if( strcmp( pcCommand, "STOP" ) == 0 )
    {
        CoverMotor_Stop( pxCover );
        pxCover->stateReported = COVER_STATE_STOPPED;
        anyChanged = pdTRUE;
    }

    if( anyChanged == pdTRUE )
    {
        xEventGroupSetBits( xCoverEventGroup, COVER_STATUS_CHANGED_EVENT );
    }
}

/* -------------------------------------------------------------------------- */
/* Incoming publish callback                                                  */
/* -------------------------------------------------------------------------- */

static void prvIncomingPublishCallback( void *pvContext,
                                        MQTTPublishInfo_t *pxPublishInfo )
{
    ( void ) pvContext;

    /* Topic format: <thingName>/cover/<COVER_NAME>/desired */
    char topicBuf[ MAX_TOPIC_LEN ];
    char payloadBuf[ 64 ];

    size_t topicLen = pxPublishInfo->topicNameLength;
    if( topicLen >= sizeof( topicBuf ) )
    {
        topicLen = sizeof( topicBuf ) - 1;
    }

    memcpy( topicBuf, pxPublishInfo->pTopicName, topicLen );
    topicBuf[ topicLen ] = '\0';

    size_t payloadLen = pxPublishInfo->payloadLength;

    if( payloadLen >= sizeof( payloadBuf ) )
    {
        payloadLen = sizeof( payloadBuf ) - 1;
    }

    memcpy( payloadBuf, pxPublishInfo->pPayload, payloadLen );
    payloadBuf[ payloadLen ] = '\0';

    LogInfo( ( "Cover incoming: topic=%s payload=%s", topicBuf, payloadBuf ) );

    /* Extract cover name from topic: "<thing>/cover/<NAME>/desired" */
    const char *p = strstr( topicBuf, "/cover/" );

    if( p == NULL )
    {
        return;
    }

    p += strlen( "/cover/" );
    const char *nameStart = p;
    const char *slash = strchr( nameStart, '/' );

    if( slash == NULL )
    {
        return;
    }

    char coverName[ 32 ];
    size_t nameLen = ( size_t )( slash - nameStart );

    if( nameLen >= sizeof( coverName ) )
    {
        nameLen = sizeof( coverName ) - 1;
    }

    memcpy( coverName, nameStart, nameLen );
    coverName[ nameLen ] = '\0';

    /* payload is a simple string: "OPEN", "CLOSE", "STOP" */
    prvHandleCoverCommand( coverName, payloadBuf );
}

/* -------------------------------------------------------------------------- */
/* Subscribe                                                                  */
/* -------------------------------------------------------------------------- */

static MQTTStatus_t prvSubscribeToCovers( MQTTQoS_t xQoS )
{
    MQTTStatus_t xStatus;
    char         topicFilter[ MAX_TOPIC_LEN ];

    /* Wildcard subscription: <thingName>/cover/+/desired */
    snprintf( topicFilter,
              sizeof( topicFilter ),
              "%s/cover/+/desired",
              thingName );

    do
    {
        xStatus = MqttAgent_SubscribeSync( xMQTTAgentHandle,
                                           topicFilter,
                                           xQoS,
                                           prvIncomingPublishCallback,
                                           NULL );

        if( xStatus != MQTTSuccess )
        {
            LogError( ( "Failed to SUBSCRIBE to topic %s, error=%u",
                        topicFilter,
                        xStatus ) );
        }
        else
        {
            LogInfo( ( "Subscribed to topic filter: %s", topicFilter ) );
        }
    }
    while( xStatus != MQTTSuccess );

    return xStatus;
}

/* -------------------------------------------------------------------------- */
/* State publishing                                                           */
/* -------------------------------------------------------------------------- */

static void prvPublishCoverStates( void )
{
    char   topic[ MAX_TOPIC_LEN ];
    char   payload[ 32 ];
    size_t len;
    MQTTStatus_t xStatus;

    for( uint8_t i = 0; i < COVER_COUNT; i++ )
    {
        CoverDescriptor_t *pxCover = &xCovers[ i ];

        snprintf( topic,
                  sizeof( topic ),
                  "%s/cover/%s/state",
                  thingName,
                  pxCover->name );

        const char *stateStr = prvStateToString( pxCover->stateReported );

        len = snprintf( payload,
                        sizeof( payload ),
                        "%s",
                        stateStr );

        xStatus = prvPublishToTopic( MQTTQoS1,
                                     false,
                                     topic,
                                     (uint8_t *) payload,
                                     len );

        if( xStatus == MQTTSuccess )
        {
            LogInfo( ( "Published cover state: %s -> %s",
                       pxCover->name,
                       stateStr ) );
        }
        else
        {
            LogError( ( "Failed to publish cover state: %s", pxCover->name ) );
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Cover task                                                                 */
/* -------------------------------------------------------------------------- */

void vCoverTask( void *pvParameters )
{
    ( void ) pvParameters;

    vSleepUntilMQTTAgentReady();
    xMQTTAgentHandle = xGetMqttAgentHandle();
    configASSERT( xMQTTAgentHandle != NULL );

    vSleepUntilMQTTAgentConnected();

    size_t thingLen = 0;
    char *pThing = KVStore_getStringHeap( CS_CORE_THING_NAME, &thingLen );
    configASSERT( pThing != NULL );

    configASSERT( thingLen < sizeof( thingName ) );
    memcpy( thingName, pThing, thingLen );
    thingName[ thingLen ] = '\0';
    vPortFree( pThing );

    LogInfo( ( "MQTT Agent connected. Starting cover task for thing: %s", thingName ) );

    xCoverEventGroup = xEventGroupCreate();
    configASSERT( xCoverEventGroup != NULL );

    /* Subscribe to all cover commands */
    prvSubscribeToCovers( MQTTQoS1 );

    /* Force initial state publish (all unknown) */
    xEventGroupSetBits( xCoverEventGroup, COVER_STATUS_CHANGED_EVENT );

    for (;;)
    {
        /* 1. Poll sensors */
        for (uint8_t i = 0; i < COVER_COUNT; i++)
        {
            CoverState_t sensed = prvReadDoorSensor(i);

            if (sensed != xCovers[i].stateReported)
            {
                xCovers[i].stateReported = sensed;
                xEventGroupSetBits(xCoverEventGroup, COVER_STATUS_CHANGED_EVENT);
            }
        }

        /* 2. Handle MQTT publishes */
        EventBits_t bits = xEventGroupWaitBits(
            xCoverEventGroup,
            COVER_STATUS_CHANGED_EVENT,
            pdTRUE,
            pdTRUE,
            xPollRate  /* timeout so we can poll again */
        );

        if (bits & COVER_STATUS_CHANGED_EVENT)
        {
            prvPublishCoverStates();
        }
    }

}
