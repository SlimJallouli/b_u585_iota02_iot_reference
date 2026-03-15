/*
 * Event-driven IMU MQTT trigger task for ISM330DHCX
 * Replaces periodic sensor publishing with interrupt-based events.
 */

#include "logging_levels.h"
#define LOG_LEVEL LOG_INFO
#include "logging.h"

/* Standard includes */
#include <string.h>
#include <stdio.h>
#include <assert.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* System includes */
#include "kvstore.h"
#include "sys_evt.h"
#include "interrupt_handlers.h"

/* MQTT includes */
#include "core_mqtt.h"
#include "core_mqtt_agent.h"
#include "mqtt_agent_task.h"
#include "subscription_manager.h"

/* Sensor includes */
#include "ism330dhcx.h"
#include "iis2mdc.h"
#include "custom_bus_os.h"
#include "custom_errno.h"

#if DEMO_MOTION_IMU
/* -------------------------------------------------------------------------- */
/* Globals                                                                    */
/* -------------------------------------------------------------------------- */

static ISM330DHCX_Object_t ISM330DHCX_Obj;

/* MQTT publish parameters */
#define MQTT_PUBLISH_MAX_LEN              (128)
#define MQTT_PUBLISH_TOPIC_EVENT          "imu/event"
#define MQTT_PUBLICH_TOPIC_STR_LEN        (256)
#define MQTT_PUBLISH_BLOCK_TIME_MS        (200)
#define MQTT_PUBLISH_NOTIFICATION_WAIT_MS (1000)
#define MQTT_NOTIFY_IDX                   (1)
#define MQTT_PUBLISH_QOS                  (MQTTQoS0)

static EventGroupHandle_t xIMUEventGroup = NULL;
#define IMU_EVENT      ( 1 << 0 )
/* -------------------------------------------------------------------------- */
/* MQTT Agent Callback                                                        */
/* -------------------------------------------------------------------------- */

struct MQTTAgentCommandContext
{
    MQTTStatus_t xReturnStatus;
    TaskHandle_t xTaskToNotify;
    uint32_t ulNotificationValue;
};

static void prvPublishCommandCallback( MQTTAgentCommandContext_t * pxCommandContext,
                                       MQTTAgentReturnInfo_t * pxReturnInfo )
{
    TaskHandle_t xTaskHandle = ( TaskHandle_t ) pxCommandContext;
    configASSERT( pxReturnInfo != NULL );

    uint32_t ulNotifyValue = pxReturnInfo->returnCode;

    if( xTaskHandle != NULL )
    {
        ( void ) xTaskNotifyIndexed( xTaskHandle,
                                     MQTT_NOTIFY_IDX,
                                     ulNotifyValue,
                                     eSetValueWithOverwrite );
    }
}

static BaseType_t prvPublishAndWaitForAck( MQTTAgentHandle_t xMQTTAgentHandle,
                                           const char * pcTopic,
                                           const void * pvPublishData,
                                           size_t xPublishDataLen )
{
    MQTTStatus_t xStatus;
    size_t uxTopicLen = strnlen( pcTopic, UINT16_MAX );

    MQTTPublishInfo_t xPublishInfo =
    {
        .qos             = MQTT_PUBLISH_QOS,
        .retain          = 0,
        .dup             = 0,
        .pTopicName      = pcTopic,
        .topicNameLength = ( uint16_t ) uxTopicLen,
        .pPayload        = pvPublishData,
        .payloadLength   = xPublishDataLen
    };

    MQTTAgentCommandInfo_t xCommandParams =
    {
        .blockTimeMs                 = MQTT_PUBLISH_BLOCK_TIME_MS,
        .cmdCompleteCallback         = prvPublishCommandCallback,
        .pCmdCompleteCallbackContext = ( void * ) xTaskGetCurrentTaskHandle(),
    };

    xTaskNotifyStateClearIndexed( NULL, MQTT_NOTIFY_IDX );

    xStatus = MQTTAgent_Publish( xMQTTAgentHandle,
                                 &xPublishInfo,
                                 &xCommandParams );

    if( xStatus == MQTTSuccess )
    {
        uint32_t ulNotifyValue = 0;
        BaseType_t xResult = xTaskNotifyWaitIndexed(
                                MQTT_NOTIFY_IDX,
                                0xFFFFFFFF,
                                0xFFFFFFFF,
                                &ulNotifyValue,
                                pdMS_TO_TICKS( MQTT_PUBLISH_NOTIFICATION_WAIT_MS ) );

        if( xResult )
        {
            xStatus = ( MQTTStatus_t ) ulNotifyValue;
            if( xStatus != MQTTSuccess )
            {
                LogError( "MQTT Agent returned error code: %d", xStatus );
                return pdFALSE;
            }
        }
        else
        {
            LogError( "Timed out waiting for publish ACK" );
            return pdFALSE;
        }
    }
    else
    {
        LogError( "MQTTAgent_Publish returned error %d", xStatus );
        return pdFALSE;
    }

    return pdTRUE;
}

/* -------------------------------------------------------------------------- */
/* Sensor Initialization                                                       */
/* -------------------------------------------------------------------------- */

static BaseType_t xInitSensors(void)
{
    uint8_t ISM330DHCX_Id;
    uint8_t Status;
    ISM330DHCX_IO_t io_ctx = {0};

#if defined(BUS_I2C1_INSTANCE)
    io_ctx.BusType = ISM330DHCX_I2C_BUS;
    io_ctx.Address = ISM330DHCX_I2C_ADD_H;
    io_ctx.Init    = BSP_I2C1_Init_OS;
    io_ctx.DeInit  = BSP_I2C1_DeInit_OS;
    io_ctx.ReadReg = BSP_I2C1_ReadReg_OS;
    io_ctx.WriteReg= BSP_I2C1_WriteReg_OS;
#elif defined(BUS_I2C2_INSTANCE)
    io_ctx.BusType = ISM330DHCX_I2C_BUS;
    io_ctx.Address = ISM330DHCX_I2C_ADD_H;
    io_ctx.Init    = BSP_I2C2_Init_OS;
    io_ctx.DeInit  = BSP_I2C2_DeInit_OS;
    io_ctx.ReadReg = BSP_I2C2_ReadReg_OS;
    io_ctx.WriteReg= BSP_I2C2_WriteReg_OS;
#endif

    ISM330DHCX_RegisterBusIO(&ISM330DHCX_Obj, &io_ctx);
    ISM330DHCX_Init(&ISM330DHCX_Obj);
    ISM330DHCX_ReadID(&ISM330DHCX_Obj, &ISM330DHCX_Id);

    if( ISM330DHCX_Id != ISM330DHCX_ID )
    {
        return pdFALSE;
    }

    /* Enable accelerometer only (gyro/mag not needed for event triggers) */
    ISM330DHCX_ACC_Enable(&ISM330DHCX_Obj);

    /* Wait for data-ready */
    do {
        vTaskDelay(5);
        ISM330DHCX_ACC_Get_DRDY_Status(&ISM330DHCX_Obj, &Status);
    } while (Status != 1);

    /* Enable event detection */
    ISM330DHCX_ACC_Enable_Free_Fall_Detection(&ISM330DHCX_Obj, ISM330DHCX_INT1_PIN);

//    vTaskDelay(500);
//    ISM330DHCX_ACC_Enable_Single_Tap_Detection(&ISM330DHCX_Obj, ISM330DHCX_INT1_PIN);

    vTaskDelay(500);
    ISM330DHCX_ACC_Enable_Double_Tap_Detection(&ISM330DHCX_Obj, ISM330DHCX_INT1_PIN);

//    vTaskDelay(500);
//    ISM330DHCX_ACC_Enable_Wake_Up_Detection(&ISM330DHCX_Obj, ISM330DHCX_INT1_PIN);

    return pdTRUE;
}

/* -------------------------------------------------------------------------- */
/* EXTI Callback                                                              */
/* -------------------------------------------------------------------------- */

static void vISM330ExtiCallback( uint16_t usGpioPinMask, void * pvContext )
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xEventGroupSetBitsFromISR( xIMUEventGroup, IMU_EVENT, NULL );
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* -------------------------------------------------------------------------- */
/* IMU Event Task                                                              */
/* -------------------------------------------------------------------------- */

void vIMUTask( void * pvParameters )
{
    MQTTAgentHandle_t xMQTTAgentHandle = NULL;
    char * pcTopicString = NULL;
    char payload[32];

    (void) pvParameters;

    LogInfo("IMU task started");

    /* Build topic string */
    pcTopicString = pvPortMalloc( MQTT_PUBLICH_TOPIC_STR_LEN );
    char * pcDeviceId = KVStore_getStringHeap( CS_CORE_THING_NAME, NULL );

    if( pcDeviceId == NULL )
    {
        LogError("Device ID missing");
        vTaskDelete(NULL);
    }

    snprintf( pcTopicString,
              MQTT_PUBLICH_TOPIC_STR_LEN,
              "%s/%s",
              pcDeviceId,
              MQTT_PUBLISH_TOPIC_EVENT );

    vPortFree( pcDeviceId );

    /* Wait for MQTT agent */
    vSleepUntilMQTTAgentReady();
    xMQTTAgentHandle = xGetMqttAgentHandle();
    configASSERT( xMQTTAgentHandle != NULL );
    vSleepUntilMQTTAgentConnected();

    xIMUEventGroup = xEventGroupCreate();
    configASSERT( xIMUEventGroup != NULL );

    /* Register EXTI callback */
    GPIO_EXTI_Register_Rising_Callback(
        ISM330DHCX_INT1_Pin,
        (GPIOInterruptCallback_t)vISM330ExtiCallback,
        (void *) xTaskGetCurrentTaskHandle()
    );

    if( xInitSensors() != pdTRUE )
    {
        LogError("ISM330DHCX init failed");
        vTaskDelete(NULL);
    }

    /* Main loop: wait for IMU interrupts */
    for( ;; )
    {
        /* Block until ISR notifies us */
//        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        xEventGroupWaitBits( xIMUEventGroup,
                             IMU_EVENT,
                             pdTRUE,
                             pdFALSE,
                             portMAX_DELAY );

        ISM330DHCX_Event_Status_t evt;
        ISM330DHCX_ACC_Get_Event_Status(&ISM330DHCX_Obj, &evt);

        const char * msg = NULL;

        if( evt.FreeFallStatus )
        {
            msg = "free_fall";
        }
        else if( evt.DoubleTapStatus )
        {
            msg = "double_tap";
        }
        else if( evt.TapStatus )
        {
            msg = "single_tap";
        }
        else if( evt.WakeUpStatus )
        {
            msg = "wake_up";
        }

        if( msg != NULL )
        {
            snprintf(payload, sizeof(payload), "%s", msg);

            prvPublishAndWaitForAck(
                xMQTTAgentHandle,
                pcTopicString,
                payload,
                strlen(payload)
            );

            LogInfo("IMU event published: %s to topic : %s", msg, pcTopicString);
        }
    }
}
#endif
