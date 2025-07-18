/*
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Derived from simple_sub_pub_demo.c
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
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#include "logging_levels.h"
/* define LOG_LEVEL here if you want to modify the logging level from the default */

#define LOG_LEVEL    LOG_INFO

#include "logging.h"

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "kvstore.h"

/* MQTT library includes. */
#include "core_mqtt.h"
#include "core_mqtt_agent.h"
#include "sys_evt.h"

/* Subscription manager header include. */
#include "subscription_manager.h"

/* Sensor includes */
#if USE_SENSORS
#include "veml3235.h"
#endif

#if USE_SENSORS
#include "custom_bus_os.h"
#include "custom_errno.h"
/* USER CODE BEGIN PV */
VEML3235_Object_t  VEML3235_Obj;
#else
#define BSP_ERROR_NONE 0
#endif

typedef struct
{
  uint32_t ALS_Lux;
  uint32_t WHITE_lux;
} LightSensorData_t;

#define MQTT_PUBLISH_MAX_LEN                 ( 512 )
#define MQTT_PUBLISH_TIME_BETWEEN_MS         ( 1000 * 60)
#define MQTT_PUBLISH_TOPIC                   "sensor/lux_sensor"
#define MQTT_PUBLICH_TOPIC_STR_LEN           ( 256 )
#define MQTT_PUBLISH_BLOCK_TIME_MS           ( 1000 )
#define MQTT_PUBLISH_NOTIFICATION_WAIT_MS    ( 1000 )

#define MQTT_NOTIFY_IDX                      ( 1 )
#define MQTT_PUBLISH_QOS                     ( MQTTQoS0 )

extern UBaseType_t uxRand(void);

static int32_t  LIGHT_SENSOR_ConvertToLUX(VEML3235_Object_t *pObj, uint32_t Value, uint32_t *LuxLevel);
static uint32_t LIGHT_SENSOR_LuxCompensation(uint32_t Value);

/*-----------------------------------------------------------*/

/**
 * @brief Defines the structure to use as the command callback context in this
 * demo.
 */
struct MQTTAgentCommandContext
{
  MQTTStatus_t xReturnStatus;
  TaskHandle_t xTaskToNotify;
};

/*-----------------------------------------------------------*/

static void prvPublishCommandCallback(MQTTAgentCommandContext_t *pxCommandContext, MQTTAgentReturnInfo_t *pxReturnInfo)
{
  configASSERT(pxCommandContext != NULL);
  configASSERT(pxReturnInfo != NULL);

  pxCommandContext->xReturnStatus = pxReturnInfo->returnCode;

  if (pxCommandContext->xTaskToNotify != NULL)
  {
    /* Send the context's ulNotificationValue as the notification value so
     * the receiving task can check the value it set in the context matches
     * the value it receives in the notification. */
    (void) xTaskNotifyGiveIndexed(pxCommandContext->xTaskToNotify, MQTT_NOTIFY_IDX);
  }
}

/*-----------------------------------------------------------*/

static BaseType_t prvPublishAndWaitForAck(MQTTAgentHandle_t xMQTTAgentHandle, const char *pcTopic, const void *pvPublishData, size_t xPublishDataLen)
{
  BaseType_t xResult = pdFALSE;
  MQTTStatus_t xStatus;

  configASSERT(pcTopic != NULL);
  configASSERT(pvPublishData != NULL);
  configASSERT(xPublishDataLen > 0);

  MQTTPublishInfo_t xPublishInfo =
  { .qos = MQTT_PUBLISH_QOS, .retain = 0, .dup = 0, .pTopicName = pcTopic, .topicNameLength = strlen(pcTopic), .pPayload = pvPublishData, .payloadLength = xPublishDataLen };

  MQTTAgentCommandContext_t xCommandContext =
  { .xTaskToNotify = xTaskGetCurrentTaskHandle(), .xReturnStatus = MQTTIllegalState, };

  MQTTAgentCommandInfo_t xCommandParams =
  { .blockTimeMs = MQTT_PUBLISH_BLOCK_TIME_MS, .cmdCompleteCallback = prvPublishCommandCallback, .pCmdCompleteCallbackContext = &xCommandContext, };

  /* Clear the notification index */
  xTaskNotifyStateClearIndexed(NULL, MQTT_NOTIFY_IDX);

  xStatus = MQTTAgent_Publish(xMQTTAgentHandle, &xPublishInfo, &xCommandParams);

  if (xStatus == MQTTSuccess)
  {
    xResult = ulTaskNotifyTakeIndexed(MQTT_NOTIFY_IDX, pdTRUE, pdMS_TO_TICKS( MQTT_PUBLISH_NOTIFICATION_WAIT_MS ));

    if (xResult == 0)
    {
      LogError("Timed out while waiting for publish ACK or Sent event. xTimeout = %d", pdMS_TO_TICKS( MQTT_PUBLISH_NOTIFICATION_WAIT_MS ));
      xResult = pdFALSE;
    }
    else if (xCommandContext.xReturnStatus != MQTTSuccess)
    {
      LogError("MQTT Agent returned error code: %d during publish operation.", xCommandContext.xReturnStatus);
      xResult = pdFALSE;
    }
  }
  else
  {
    LogError("MQTTAgent_Publish returned error code: %d.", xStatus);
  }

  return xResult;
}

/*-----------------------------------------------------------*/

static BaseType_t xIsMqttConnected(void)
{
  /* Wait for MQTT to be connected */
  EventBits_t uxEvents = xEventGroupWaitBits(xSystemEvents,
  EVT_MASK_MQTT_CONNECTED,
  pdFALSE,
  pdTRUE, 0);

  return ((uxEvents & EVT_MASK_MQTT_CONNECTED) == EVT_MASK_MQTT_CONNECTED);
}

/*-----------------------------------------------------------*/

static BaseType_t xInitSensors(void)
{
  uint32_t VEML3235_Id;
  int32_t Status;
  VEML3235_IO_t VEML3235_io_ctx = { 0 };

  VEML3235_io_ctx.ReadAddress   = VEML3235_I2C_READ_ADD;
  VEML3235_io_ctx.WriteAddress  = VEML3235_I2C_WRITE_ADD;
  VEML3235_io_ctx.Init          = BSP_I2C2_Init_OS;
  VEML3235_io_ctx.DeInit        = BSP_I2C2_DeInit_OS;
  VEML3235_io_ctx.IsReady       = BSP_I2C2_IsReady_OS;
  VEML3235_io_ctx.ReadReg       = BSP_I2C2_ReadReg_OS;
  VEML3235_io_ctx.WriteReg      = BSP_I2C2_WriteReg_OS;
  VEML3235_io_ctx.GetTick       = BSP_GetTick;

  Status = VEML3235_RegisterBusIO(&VEML3235_Obj, &VEML3235_io_ctx);
  Status = VEML3235_Init         (&VEML3235_Obj);
  Status = VEML3235_ReadID       (&VEML3235_Obj, &VEML3235_Id);

  if (VEML3235_ID != VEML3235_Id)
  {
    Status = VEML3235_ERROR;
  }

  if(VEML3235_OK == Status)
  {
    Status = VEML3235_Stop(&VEML3235_Obj);
  }

  if(VEML3235_OK == Status)
  {
    Status = VEML3235_SetGain(&VEML3235_Obj, VEML3235_ALS_CHANNEL, VEML3235_CONF_GAIN_1);
  }

  if(VEML3235_OK == Status)
  {
    Status = VEML3235_SetExposureTime(&VEML3235_Obj, VEML3235_CONF_IT100);
  }

  return Status == VEML3235_OK;
}

/*-----------------------------------------------------------*/

static BaseType_t xUpdateSensorData(LightSensorData_t *pxData)
{
    int32_t Status = VEML3235_OK;
    uint32_t pResult[2] = {0}; // [0] ALS, [1] WHITE

    // Start sensor in continuous mode
    Status = VEML3235_Start(&VEML3235_Obj, VEML3235_MODE_CONTINUOUS);

    if (Status != VEML3235_OK)
    {
        return pdFAIL;
    }

    // Read raw values
    Status = VEML3235_GetValues(&VEML3235_Obj, pResult);

    if (Status != VEML3235_OK)
    {
        VEML3235_Stop(&VEML3235_Obj); // Cleanup attempt
        return pdFAIL;
    }

    // Convert raw values to lux
    if (LIGHT_SENSOR_ConvertToLUX(&VEML3235_Obj, pResult[0], &pxData->ALS_Lux) != VEML3235_OK ||
        LIGHT_SENSOR_ConvertToLUX(&VEML3235_Obj, pResult[1], &pxData->WHITE_lux) != VEML3235_OK)
    {
        VEML3235_Stop(&VEML3235_Obj);
        return pdFAIL;
    }

    // Apply compensation if needed
    if (pxData->ALS_Lux > 10000)
    {
        pxData->ALS_Lux = LIGHT_SENSOR_LuxCompensation(pxData->ALS_Lux);
    }

    if (pxData->WHITE_lux > 10000)
    {
        pxData->WHITE_lux = LIGHT_SENSOR_LuxCompensation(pxData->WHITE_lux);
    }

    // Stop sensor after read
    VEML3235_Stop(&VEML3235_Obj);

    return pdPASS;
}


static float IntegrationTime_800[3]     = {0.00426, 0.00852, 0.01704};
static float IntegrationTime_400[3]     = {0.00852, 0.01704, 0.03408};
static float IntegrationTime_200[3]     = {0.01704, 0.03408, 0.06816};
static float IntegrationTime_100[3]     = {0.03408, 0.06816, 0.13632};
static float IntegrationTime_50[3]      = {0.06816, 0.13632, 0.27264};

static int32_t LIGHT_SENSOR_ConvertToLUX(VEML3235_Object_t *pObj, uint32_t Value, uint32_t *LuxLevel)
{
  int32_t ret = VEML3235_OK;
  float luxConv = 0;
  uint8_t convPos = 0;
  uint32_t pGain;
  uint32_t pExposureTime;
  int32_t Status;

  Status = VEML3235_GetGain(pObj, VEML3235_ALS_CHANNEL, &pGain);

  if(VEML3235_OK == Status)
  {
    Status = VEML3235_GetExposureTime(pObj, &pExposureTime);
  }

  if(VEML3235_OK == Status)
  {
    if (pGain == VEML3235_CONF_GAIN_1)
    {
      convPos = 2;
    }
    else if (pGain == VEML3235_CONF_GAIN_2)
    {
      convPos = 1;
    }
    else if (pGain == VEML3235_CONF_GAIN_4)
    {
      convPos = 0;
    }
    else
    {
      Status = VEML3235_INVALID_PARAM;
    }
  }

  if(VEML3235_OK == Status)
  {
    if(pExposureTime == VEML3235_CONF_IT800)
    {
      luxConv = IntegrationTime_800[convPos];
    }
    else if(pExposureTime == VEML3235_CONF_IT400)
    {
      luxConv = IntegrationTime_400[convPos];
    }
    else if(pExposureTime == VEML3235_CONF_IT200)
    {
      luxConv = IntegrationTime_200[convPos];
    }
    else if(pExposureTime == VEML3235_CONF_IT100)
    {
      luxConv = IntegrationTime_100[convPos];
    }
    else if(pExposureTime == VEML3235_CONF_IT50)
    {
      luxConv = IntegrationTime_50[convPos];
    }
    else
    {
      Status = VEML3235_INVALID_PARAM;
    }
  }

  *LuxLevel =(uint32_t)(luxConv * Value);

  return ret;
}

static uint32_t LIGHT_SENSOR_LuxCompensation(uint32_t Value)
{

  /* Polynomial is pulled from the datasheet */
  float compLux = (.00000000000060135 * (pow(Value, 4))) -
                      (.0000000093924 * (pow(Value, 3))) +
                      (.000081488 * (pow(Value,2))) +
                      (1.0023 * Value);
  return (uint32_t)compLux;
}

/*-----------------------------------------------------------*/

void vLightSensorPublishTask(void *pvParameters)
{
  BaseType_t xResult   = pdFALSE;
  BaseType_t xExitFlag = pdFALSE;

  MQTTAgentHandle_t xMQTTAgentHandle = NULL;
  char * pcPayloadBuf            = NULL;
  char * pcTopicString           = NULL;
  char * pcDeviceId              = NULL;

  size_t uxTopicLen    = 0;
  LightSensorData_t xSensorData = {0};

  pcPayloadBuf  = pvPortMalloc(MQTT_PUBLISH_MAX_LEN      );
  pcTopicString = pvPortMalloc(MQTT_PUBLICH_TOPIC_STR_LEN);

  (void) pvParameters;

  xResult = xInitSensors();

  if (xResult != pdTRUE)
  {
    LogError("Error while initializing light sensors.");
    vTaskDelete( NULL);
  }

  pcDeviceId = KVStore_getStringHeap( CS_CORE_THING_NAME, NULL );

  if( pcDeviceId == NULL )
  {
      xExitFlag = pdTRUE;
  }
  else
  {
    uxTopicLen = snprintf( pcTopicString, ( size_t ) MQTT_PUBLICH_TOPIC_STR_LEN, "%s/%s", pcDeviceId, MQTT_PUBLISH_TOPIC );
    vPortFree( pcDeviceId );
  }

  if ((uxTopicLen == 0) || (uxTopicLen >= MQTT_PUBLICH_TOPIC_STR_LEN))
  {
    LogError("Failed to construct topic string.");
    xExitFlag = pdTRUE;
  }

  /* Wait until the MQTT agent is ready */
  vSleepUntilMQTTAgentReady();

  /* Get the MQTT Agent handle */
  xMQTTAgentHandle = xGetMqttAgentHandle();
  configASSERT(xMQTTAgentHandle != NULL);

  /* Wait until we are connected to AWS */
  vSleepUntilMQTTAgentConnected();

  vTaskDelay(pdMS_TO_TICKS(10000));

  while (xExitFlag == pdFALSE)
  {
    TickType_t xTicksToWait = pdMS_TO_TICKS(MQTT_PUBLISH_TIME_BETWEEN_MS);
    TimeOut_t xTimeOut;

    vTaskSetTimeOutState(&xTimeOut);

    xResult = xUpdateSensorData(&xSensorData);

    if (xResult != pdTRUE)
    {
      LogError("Error while reading sensor data.");
    }
    else if (xIsMqttConnected() == pdTRUE)
    {
      int lbytesWritten = 0;

      /* Write to */
      lbytesWritten = snprintf(pcPayloadBuf,
                               MQTT_PUBLISH_MAX_LEN,
                               "{ \"lux\": %f }",
                               (float)xSensorData.ALS_Lux);

      if( ( lbytesWritten < MQTT_PUBLISH_MAX_LEN ) && ( xIsMqttAgentConnected() == pdTRUE ) )
      {
    	  LogInfo(( "Sending publish message to topic: %s , message : %*s", pcTopicString, lbytesWritten, ( char * ) pcPayloadBuf ));

        xResult = prvPublishAndWaitForAck(xMQTTAgentHandle,
                                          pcTopicString,
                                          pcPayloadBuf,
                                          ( size_t ) lbytesWritten );

        if( xResult != pdPASS )
        {
            LogError( "Failed to publish motion sensor data" );
        }
      }
    }

    /* Adjust remaining tick count */
    if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE)
    {
      /* Wait until its time to poll the sensors again */
      vTaskDelay(xTicksToWait);
    }
  }
}
