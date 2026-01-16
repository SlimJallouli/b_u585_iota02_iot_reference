/*
 * FreeRTOS STM32 Reference Integration (MXCHIP -> ST67W6X migration)
 *
 * This file is intentionally patterned after mx_netconn.c.
 *
 * Notes:
 *  - st67_lwip_netif.c owns the RX task and driver integration.
 *  - This module owns the "network manager" task (net_main) and glues:
 *      lwIP init + netif add + DHCP + system event bits.
 */

#include "logging_levels.h"
#define LOG_LEVEL    LOG_DEBUG
#include "logging.h"

#include "main.h"

#include <stdint.h>
#include <string.h>

#include "../W6X_ARCH_T02/st67_netconn.h"
#include "../W6X_ARCH_T02/st67_lwip_mxcompat.h"
#include "../W6X_ARCH_T02/st67_lwip_netif.h"
#include "w6x_types.h"

#include "spi_iface.h" /* spi falling/rising_callback */

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "kvstore.h"

#include "lwip/tcpip.h"
#include "lwip/netifapi.h"

#include "sys_evt.h"

/* If your project uses w6x_api.h, include it so you can fill in the weak hooks. */
#include "w6x_api.h"

/** BLE data buffer to receive message from the ST67W6X Driver */
static uint8_t a_APP_AvailableData[247] = {0};

/** Event when Wi-Fi is connected to an Access Point */
#define EVT_APP_WIFI_CONNECTED                      (1<<1)
/** Event when Wi-Fi got an IP address from the Access Point */
#define EVT_APP_WIFI_GOT_IP                         (1<<2)
/** Event when Wi-Fi is disconnected from the Access Point */
#define EVT_APP_WIFI_DISCONNECTED                   (1<<2)

/** Application event group */
static EventGroupHandle_t app_evt_current;

/* --------------------------------------------------------------------------
 * Internal state
 * -------------------------------------------------------------------------- */

static TaskHandle_t xNetTaskHandle = NULL;
static St67NetConnectCtx_t *pxActiveCtx = NULL;

/* --------------------------------------------------------------------------
 * Weak hooks: bind these to your ST67W6X Wi-Fi connection implementation.
 *
 * The reference MXCHIP integration pulls SSID/PSK from KVStore and initiates
 * a connection itself. ST67W6X projects vary (CLI, auto-connect, user-driven,
 * etc.), so these are left as hooks.
 * -------------------------------------------------------------------------- */

__attribute__( ( weak ))
W6X_Status_t st67_wifi_connect_from_kvstore(void)
{
  W6X_Status_t ret = W6X_STATUS_OK;
  W6X_WiFi_Connect_Opts_t ConnectOpts = { 0 };

#if defined(LFS_CONFIG)
  KVStore_getString(CS_WIFI_SSID, (char*) ConnectOpts.SSID, W6X_WIFI_MAX_SSID_SIZE);
  KVStore_getString(CS_WIFI_CREDENTIAL, (char*) ConnectOpts.Password, W6X_WIFI_MAX_PASSWORD_SIZE);
#else
  snprintf((char *)ConnectOpts.SSID    , W6X_WIFI_MAX_SSID_SIZE    , "%s", DEFAULT_SSID);
  snprintf((char *)ConnectOpts.Password, W6X_WIFI_MAX_PASSWORD_SIZE, "%s", DEFAULT_PSWD);
#endif

  ret = W6X_WiFi_Connect(&ConnectOpts);

  return ret;
}

__attribute__( ( weak ))
 BaseType_t st67_wifi_disconnect(void)
{
  /* Implement in your application if you want net_request_reconnect() to work. */
  return pdFALSE;
}

/* --------------------------------------------------------------------------
 * Utilities
 * -------------------------------------------------------------------------- */

static uint32_t ulWaitForNotifyBits(BaseType_t uxIndexToWaitOn, uint32_t ulTargetBits, TickType_t xTicksToWait)
{
  TickType_t xRemainingTicks = xTicksToWait;
  TimeOut_t xTimeOut;

  vTaskSetTimeOutState(&xTimeOut);

  uint32_t ulNotifyValueAccumulate = 0x0;

  while ((ulNotifyValueAccumulate & ulTargetBits) != ulTargetBits)
  {
    uint32_t ulNotifyValue = 0x0;
    (void) xTaskNotifyWaitIndexed(uxIndexToWaitOn, 0x0, ulTargetBits, &ulNotifyValue, xRemainingTicks);

    if (ulNotifyValue != 0)
    {
      ulNotifyValueAccumulate |= ulNotifyValue;
    }

    if (xTaskCheckForTimeOut(&xTimeOut, &xRemainingTicks) == pdTRUE)
    {
      break;
    }
  }

  /* Preserve non-target bits */
  if ((ulNotifyValueAccumulate & (~ulTargetBits)) != 0)
  {
    (void) xTaskNotifyIndexed(xTaskGetCurrentTaskHandle(), uxIndexToWaitOn, 0, eNoAction);
  }

  return ((ulTargetBits & ulNotifyValueAccumulate) != 0);
}

static void vLwipReadyCallback(void *pvCtx)
{
  St67NetConnectCtx_t *pxCtx = (St67NetConnectCtx_t*) pvCtx;

  if ((pxCtx != NULL) && (pxCtx->xNetTaskHandle != NULL))
  {
    (void) xTaskNotifyIndexed(pxCtx->xNetTaskHandle, NET_EVT_IDX, NET_LWIP_READY_BIT, eSetBits);
  }
}

/* --------------------------------------------------------------------------
 * ST67W6X netif link callbacks
 *
 * These are invoked by the W6X netif component (st67_lwip_netif.c) when the
 * module indicates link up/down.
 * -------------------------------------------------------------------------- */

static int32_t prvStaLinkUpCb(void)
{
  if (pxActiveCtx != NULL)
  {
    vSetLinkUp(&pxActiveCtx->xNetif);
  }
  return 0;
}

static int32_t prvStaLinkDownCb(void)
{
  if (pxActiveCtx != NULL)
  {
    vSetLinkDown(&pxActiveCtx->xNetif);
  }
  return 0;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

BaseType_t net_request_reconnect(void)
{
  BaseType_t xReturn = pdFALSE;

  LogDebug( "net_request_reconnect" );

  if (xNetTaskHandle != NULL)
  {
    xReturn = xTaskNotifyIndexed(xNetTaskHandle, NET_EVT_IDX, ASYNC_REQUEST_RECONNECT_BIT, eSetBits);
  }

  return xReturn;
}

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Wi-Fi event callback
  * @param  event_id: Event ID
  * @param  event_args: Event arguments
  */
static void APP_wifi_cb(W6X_event_id_t event_id, void *event_args);

/**
  * @brief  Network event callback
  * @param  event_id: Event ID
  * @param  event_args: Event arguments
  */
static void APP_net_cb(W6X_event_id_t event_id, void *event_args);

/**
  * @brief  MQTT event callback
  * @param  event_id: Event ID
  * @param  event_args: Event arguments
  */
static void APP_mqtt_cb(W6X_event_id_t event_id, void *event_args);

/**
  * @brief  BLE event callback
  * @param  event_id: Event ID
  * @param  event_args: Event arguments
  */
static void APP_ble_cb(W6X_event_id_t event_id, void *event_args);

/**
  * @brief  W6X error callback
  * @param  ret_w6x: W6X status
  * @param  func_name: function name
  */
static void APP_error_cb(W6X_Status_t ret_w6x, char const *func_name);

/**
  * @brief  Set event group to release the waiting task
  * @param  app_event: Event group
  * @param  evt: Event to set
  */
static void APP_setevent(EventGroupHandle_t *app_event, uint32_t evt);


void net_main(void *pvParameters)
{
  int32_t ret = 0;
  W6X_Net_if_cb_t xNetIfCb;
  (void) pvParameters;

  BaseType_t xResult;
  err_t xLwipError;

  St67NetConnectCtx_t xCtx;
  (void) memset(&xCtx, 0, sizeof(xCtx));

  xNetTaskHandle = xTaskGetCurrentTaskHandle();
  xCtx.xNetTaskHandle = xNetTaskHandle;
  pxActiveCtx = &xCtx;

  /* Register the application callback to received events from ST67W6X Driver */
  W6X_App_Cb_t App_cb = {0};

  App_cb.APP_wifi_cb  = APP_wifi_cb;
  App_cb.APP_net_cb   = APP_net_cb;
  App_cb.APP_ble_cb   = APP_ble_cb;
  App_cb.APP_mqtt_cb  = APP_mqtt_cb;
  App_cb.APP_error_cb = APP_error_cb;

  W6X_RegisterAppCb(&App_cb);

  app_evt_current = xEventGroupCreate();

  GPIO_EXTI_Register_Rising_Callback (SPI_RDY_Pin, spi_on_txn_data_ready, NULL);
  GPIO_EXTI_Register_Falling_Callback(SPI_RDY_Pin, spi_on_header_ack    , NULL);

  /* Initialize the ST67W6X Driver */
  ret = W6X_Init();

  if (ret != W6X_STATUS_OK)
  {
    LogError("failed to initialize ST67W6X Driver, %" PRIi32 "\n", ret);
    vTaskDelete(NULL);
  }

  LogInfo("W6X init is done");

  /* Initialize the ST67W6X Wi-Fi module */
  ret = W6X_WiFi_Init();

  if (ret)
  {
    LogError("failed to initialize ST67W6X Wi-Fi component, %" PRIi32 "\n", ret);
    vTaskDelete(NULL);
  }

  LogInfo("Wi-Fi init is done");

  /* Init lwIP */
  tcpip_init(vLwipReadyCallback, &xCtx);

  xResult = ulWaitForNotifyBits( NET_EVT_IDX, NET_LWIP_READY_BIT, portMAX_DELAY);

  configASSERT(xResult != 0);

  /* Init the ST67W6X netif adapter (creates RX task + registers RX callbacks) */

  (void) memset(&xNetIfCb, 0, sizeof(xNetIfCb));

  xNetIfCb.link_sta_up_fn   = prvStaLinkUpCb;
  xNetIfCb.link_sta_down_fn = prvStaLinkDownCb;

  if (net_if_init(&xNetIfCb) != 0)
  {
    LogError("net_if_init failed");
  }

  /* Try to populate MAC for lwIP netif init (optional) */
  {
    uint8_t ucMac[6] = { 0 };
    if (W6X_WiFi_Station_GetMACAddress(ucMac) == 0)
    {
      (void) memcpy(xCtx.xMacAddress.addr, ucMac, 6);
    }
  }

  /* Register lwIP netif */
  xLwipError = netifapi_netif_add(&xCtx.xNetif,
                                  NULL,
                                  NULL,
                                  NULL,
                                  &xCtx,
                                  &prvInitNetInterface,
                                  tcpip_input);

  configASSERT(xLwipError == ERR_OK);

  (void) netifapi_netif_set_default(&xCtx.xNetif);
  (void) netifapi_netif_set_up     (&xCtx.xNetif);

  (void) xEventGroupSetBits(xSystemEvents, EVT_MASK_NET_INIT);

  /* Connect the device to the pre-defined Access Point */
  LogInfo("Connecting to Local Access Point");

  if (st67_wifi_connect_from_kvstore() != W6X_STATUS_OK)
  {
    LogError("Failed to access point");
  }
  else
  {
    LogInfo("Connected to access point");
  }

  for (;;)
  {
    uint32_t ulNotificationValue = 0;

    xResult = xTaskNotifyWaitIndexed(NET_EVT_IDX, 0x0, 0xFFFFFFFF, &ulNotificationValue, pdMS_TO_TICKS( 30 * 1000 ));
    (void) xResult;

    if (ulNotificationValue == 0)
    {
      continue;
    }

    /* Latch in current flags */
    uint8_t ucNetifFlags = xCtx.xNetif.flags;

    if (ulNotificationValue & NET_LWIP_IP_CHANGE_BIT)
    {
      LogSys("IP Address Change.");
      vLogAddress("IP Address:", xCtx.xNetif.ip_addr);
      vLogAddress("Gateway:", xCtx.xNetif.gw);
      vLogAddress("Netmask:", xCtx.xNetif.netmask);
      (void) xEventGroupSetBits(xSystemEvents, EVT_MASK_NET_CONNECTED);
    }

    if (ulNotificationValue & NET_LWIP_IFUP_BIT)
    {
      LogInfo("Administrative UP event.");
      vStartDhcp(&xCtx.xNetif);
    }
    else if ((ulNotificationValue & NET_LWIP_LINK_UP_BIT) && ((ucNetifFlags & NETIF_FLAG_LINK_UP) != 0))
    {
      LogInfo("Link UP event.");
      vSetAdminUp(&xCtx.xNetif);
      vStartDhcp(&xCtx.xNetif);
      LogSys("Network Link Up.");
    }
    else if (ulNotificationValue & NET_LWIP_IFDOWN_BIT)
    {
      LogInfo("Administrative DOWN event.");
      vStopDhcp(&xCtx.xNetif);
      vClearAddress(&xCtx.xNetif);
      (void) xEventGroupClearBits(xSystemEvents, EVT_MASK_NET_CONNECTED);
    }
    else if ((ulNotificationValue & NET_LWIP_LINK_DOWN_BIT) && ((ucNetifFlags & NETIF_FLAG_LINK_UP) == 0))
    {
      vSetAdminDown(&xCtx.xNetif);
      vStopDhcp(&xCtx.xNetif);
      vClearAddress(&xCtx.xNetif);
      LogSys("Network Link Down.");
      (void) xEventGroupClearBits(xSystemEvents, EVT_MASK_NET_CONNECTED);
    }

    if (ulNotificationValue & ASYNC_REQUEST_RECONNECT_BIT)
    {
      (void) xEventGroupClearBits(xSystemEvents, EVT_MASK_NET_CONNECTED);
      (void) st67_wifi_disconnect();
      (void) st67_wifi_connect_from_kvstore();
    }
  }
}

/* Private Functions Definition ----------------------------------------------*/
static void APP_wifi_cb(W6X_event_id_t event_id, void *event_args)
{
  /* USER CODE BEGIN APP_wifi_cb_1 */

  /* USER CODE END APP_wifi_cb_1 */

  W6X_WiFi_CbParamData_t *cb_data = {0};

  switch (event_id)
  {
    case W6X_WIFI_EVT_CONNECTED_ID:
      APP_setevent(&app_evt_current, EVT_APP_WIFI_CONNECTED);
      break;

    case W6X_WIFI_EVT_DISCONNECTED_ID:
      LogInfo("Station disconnected from Access Point\n");
      break;

    case W6X_WIFI_EVT_REASON_ID:
      LogInfo("Reason: %s\n", W6X_WiFi_ReasonToStr(event_args));
      break;

    case W6X_WIFI_EVT_DIST_STA_IP_ID:
      break;

    case W6X_WIFI_EVT_STA_CONNECTED_ID:
      cb_data = (W6X_WiFi_CbParamData_t *)event_args;
      //LogInfo("Station connected to soft-AP : [ MACSTR "]\n", MAC2STR(cb_data->MAC));
      break;

    case W6X_WIFI_EVT_STA_DISCONNECTED_ID:
      cb_data = (W6X_WiFi_CbParamData_t *)event_args;
      //LogInfo("Station disconnected from soft-AP : [" MACSTR "]\n", MAC2STR(cb_data->MAC));
      break;

    default:
      break;
  }
  /* USER CODE BEGIN APP_wifi_cb_End */

  /* USER CODE END APP_wifi_cb_End */
}

static void APP_net_cb(W6X_event_id_t event_id, void *event_args)
{
  /* USER CODE BEGIN APP_net_cb_1 */

  /* USER CODE END APP_net_cb_1 */
}

static void APP_mqtt_cb(W6X_event_id_t event_id, void *event_args)
{
  /* USER CODE BEGIN APP_mqtt_cb_1 */

  /* USER CODE END APP_mqtt_cb_1 */
}

static void APP_ble_cb(W6X_event_id_t event_id, void *event_args)
{
  /* USER CODE BEGIN APP_ble_cb_1 */

  /* USER CODE END APP_ble_cb_1 */
  uint8_t service_index = 0;
  uint8_t charac_index = 0;
  uint32_t charac_handle = 0;
  uint32_t charac_value_handle = 0;

  W6X_Ble_Service_t *service = NULL;
  char tmp_UUID[33];
  uint8_t uuid_size = 0;

  W6X_Ble_CbParamData_t *p_param_ble_data = (W6X_Ble_CbParamData_t *) event_args;

  switch (event_id)
  {
    case W6X_BLE_EVT_CONNECTED_ID:
      LogInfo(" -> BLE CONNECTED: Conn_Handle: %" PRIu16 "\n", p_param_ble_data->remote_ble_device.conn_handle);
      W6X_Ble_SetRecvDataPtr(a_APP_AvailableData, sizeof(a_APP_AvailableData));
      break;

    case W6X_BLE_EVT_CONNECTION_PARAM_ID:
      LogInfo(" -> BLE CONNECTION PARAM UPDATE\n");
      break;

    case W6X_BLE_EVT_DISCONNECTED_ID:
      LogInfo(" -> BLE DISCONNECTED.\n");
      break;

    case W6X_BLE_EVT_INDICATION_STATUS_ENABLED_ID:
      LogInfo(" -> BLE INDICATION ENABLED [Connection: %" PRIu16 ", Service: %" PRIu16 ", Charac: %" PRIu16 "]\n",
              p_param_ble_data->remote_ble_device.conn_handle, p_param_ble_data->service_idx,
              p_param_ble_data->charac_idx);
      break;

    case W6X_BLE_EVT_INDICATION_STATUS_DISABLED_ID:
      LogInfo(" -> BLE INDICATION DISABLED [Connection: %" PRIu16 ", Service: %" PRIu16 ", Charac: %" PRIu16 "]\n",
              p_param_ble_data->remote_ble_device.conn_handle, p_param_ble_data->service_idx,
              p_param_ble_data->charac_idx);
      break;

    case W6X_BLE_EVT_NOTIFICATION_STATUS_ENABLED_ID:
      LogInfo(" -> BLE NOTIFICATION ENABLED [Connection: %" PRIu16 ", Service: %" PRIu16 ", Charac: %" PRIu16 "]\n",
              p_param_ble_data->remote_ble_device.conn_handle, p_param_ble_data->service_idx,
              p_param_ble_data->charac_idx);
      break;

    case W6X_BLE_EVT_NOTIFICATION_STATUS_DISABLED_ID:
      LogInfo(" -> BLE NOTIFICATION DISABLED [Connection: %" PRIu16 ", Service: %" PRIu16 ", Charac: %" PRIu16 "]\n",
              p_param_ble_data->remote_ble_device.conn_handle, p_param_ble_data->service_idx,
              p_param_ble_data->charac_idx);
      break;

    case W6X_BLE_EVT_NOTIFICATION_DATA_ID:
      LogInfo(" -> BLE NOTIFICATION [Connection: %" PRIu16 ", Charac value handle: %" PRIu16 "]\n",
              p_param_ble_data->remote_ble_device.conn_handle, p_param_ble_data->charac_value_handle);
      for (uint32_t i = 0; i < p_param_ble_data->available_data_length; i++)
      {
        LogInfo("0x%02" PRIX16 "\n", a_APP_AvailableData[i]);
      }
      memset(a_APP_AvailableData, 0, sizeof(a_APP_AvailableData));
      break;

    case W6X_BLE_EVT_WRITE_ID:
      LogInfo(" -> BLE WRITE [Connection: %" PRIu16 ", Service: %" PRIu16 ", Charac: %" PRIu16 "]\n",
              p_param_ble_data->remote_ble_device.conn_handle, p_param_ble_data->service_idx,
              p_param_ble_data->charac_idx);
      for (uint32_t i = 0; i < p_param_ble_data->available_data_length; i++)
      {
        LogInfo("0x%02" PRIX16 "\n", a_APP_AvailableData[i]);
      }
      memset(a_APP_AvailableData, 0, sizeof(a_APP_AvailableData));
      break;

    case W6X_BLE_EVT_READ_ID:
      LogInfo(" -> BLE READ [Connection: %" PRIu16 ", Service: %" PRIu16 ", Charac: %" PRIu16 "]\n",
              p_param_ble_data->remote_ble_device.conn_handle, p_param_ble_data->service_idx,
              p_param_ble_data->charac_idx);
      for (uint32_t i = 0; i < p_param_ble_data->available_data_length; i++)
      {
        LogInfo("0x%02" PRIX16 "\n", a_APP_AvailableData[i]);
      }
      memset(a_APP_AvailableData, 0, sizeof(a_APP_AvailableData));
      break;

    case W6X_BLE_EVT_SERVICE_FOUND_ID:
      service_index = p_param_ble_data->Service.service_idx;

      service = &p_param_ble_data->Service;
      memset(tmp_UUID, 0x20, 33);

      uuid_size = service->uuid_type == W6X_BLE_UUID_TYPE_16 ? 4 : 16;
      for (int32_t i = 0; i < uuid_size; i++)
      {
        sprintf(&tmp_UUID[i * 2], "%02" PRIx16, service->service_uuid[i]);
      }

      LogInfo(" -> BLE SERVICE DISCOVERED:\nidx = %" PRIu16 ", UUID = %s\n",
              service_index, tmp_UUID);
      break;

    case W6X_BLE_EVT_CHAR_FOUND_ID:
      service_index = p_param_ble_data->Service.service_idx;
      charac_index = p_param_ble_data->Service.charac[0].char_idx;
      charac_handle = p_param_ble_data->Service.charac[0].char_handle;
      charac_value_handle = p_param_ble_data->Service.charac[0].char_value_handle;

      memset(tmp_UUID, 0x20, 33);

      uuid_size = p_param_ble_data->Service.charac[0].uuid_type == W6X_BLE_UUID_TYPE_16 ? 4 : 16;
      for (int32_t i = 0; i < uuid_size; i++)
      {
        sprintf(&tmp_UUID[i * 2], "%02" PRIx16, p_param_ble_data->Service.charac[0].char_uuid[i]);
      }

      LogInfo(" -> BLE CHARACTERISTIC DISCOVERED:\nService idx = %" PRIu16 ", Charac idx = %" PRIu16
              ", UUID = %s, \r\nChar Handle = %" PRIu32 ",Char Value Handle = %" PRIu32 "\n",
              service_index, charac_index, tmp_UUID, charac_handle, charac_value_handle);
      break;

    case W6X_BLE_EVT_PASSKEY_ENTRY_ID:
      LogInfo(" -> BLE PassKey Entry: Conn_Handle: %" PRIu16 "\n", p_param_ble_data->remote_ble_device.conn_handle);
      LogInfo("    BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
              p_param_ble_data->remote_ble_device.BDAddr[0], p_param_ble_data->remote_ble_device.BDAddr[1],
              p_param_ble_data->remote_ble_device.BDAddr[2], p_param_ble_data->remote_ble_device.BDAddr[3],
              p_param_ble_data->remote_ble_device.BDAddr[4], p_param_ble_data->remote_ble_device.BDAddr[5]);
      LogInfo("    BD Addr type: %" PRIu32 "\n", p_param_ble_data->remote_ble_device.bd_addr_type);
      break;

    case W6X_BLE_EVT_PASSKEY_CONFIRM_ID:
      LogInfo(" -> BLE PassKey received = %06" PRIu32 ", Conn_Handle: %" PRIu16 "\n", p_param_ble_data->PassKey,
              p_param_ble_data->remote_ble_device.conn_handle);
      LogInfo("    BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
              p_param_ble_data->remote_ble_device.BDAddr[0], p_param_ble_data->remote_ble_device.BDAddr[1],
              p_param_ble_data->remote_ble_device.BDAddr[2], p_param_ble_data->remote_ble_device.BDAddr[3],
              p_param_ble_data->remote_ble_device.BDAddr[4], p_param_ble_data->remote_ble_device.BDAddr[5]);
      LogInfo("    BD Addr type: %" PRIu32 "\n", p_param_ble_data->remote_ble_device.bd_addr_type);
      break;

    case W6X_BLE_EVT_PAIRING_CONFIRM_ID:
      LogInfo(" -> BLE Pairing Confirm: Conn_Handle: %" PRIu16 "\n", p_param_ble_data->remote_ble_device.conn_handle);
      LogInfo("    BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
              p_param_ble_data->remote_ble_device.BDAddr[0], p_param_ble_data->remote_ble_device.BDAddr[1],
              p_param_ble_data->remote_ble_device.BDAddr[2], p_param_ble_data->remote_ble_device.BDAddr[3],
              p_param_ble_data->remote_ble_device.BDAddr[4], p_param_ble_data->remote_ble_device.BDAddr[5]);
      LogInfo("    BD Addr type: %" PRIu32 "\n", p_param_ble_data->remote_ble_device.bd_addr_type);
      break;

    case W6X_BLE_EVT_PAIRING_COMPLETED_ID:
      LogInfo(" -> BLE Pairing Completed\n\n");
      LogInfo("    BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
              p_param_ble_data->remote_ble_device.BDAddr[0], p_param_ble_data->remote_ble_device.BDAddr[1],
              p_param_ble_data->remote_ble_device.BDAddr[2], p_param_ble_data->remote_ble_device.BDAddr[3],
              p_param_ble_data->remote_ble_device.BDAddr[4], p_param_ble_data->remote_ble_device.BDAddr[5]);
      LogInfo("    BD Addr type: %" PRIu32 "\n", p_param_ble_data->remote_ble_device.bd_addr_type);
      LogInfo("    LTK: %s\n", p_param_ble_data->LongTermKey);
      break;

    case W6X_BLE_EVT_PASSKEY_DISPLAY_ID:
      LogInfo(" -> BLE PASSKEY  = %06" PRIu32 "\n", p_param_ble_data->PassKey);
      break;

    case W6X_BLE_EVT_PAIRING_FAILED_ID:
      LogInfo(" -> BLE Pairing Failed: Conn_Handle: %" PRIu16 "\n", p_param_ble_data->remote_ble_device.conn_handle);
      LogInfo("    BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
              p_param_ble_data->remote_ble_device.BDAddr[0], p_param_ble_data->remote_ble_device.BDAddr[1],
              p_param_ble_data->remote_ble_device.BDAddr[2], p_param_ble_data->remote_ble_device.BDAddr[3],
              p_param_ble_data->remote_ble_device.BDAddr[4], p_param_ble_data->remote_ble_device.BDAddr[5]);
      LogInfo("    BD Addr type: %" PRIu32 "\n", p_param_ble_data->remote_ble_device.bd_addr_type);
      break;

    case W6X_BLE_EVT_PAIRING_CANCELED_ID:
      LogInfo(" -> BLE Pairing Canceled: Conn_Handle: %" PRIu16 "\n", p_param_ble_data->remote_ble_device.conn_handle);
      LogInfo("    BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
              p_param_ble_data->remote_ble_device.BDAddr[0], p_param_ble_data->remote_ble_device.BDAddr[1],
              p_param_ble_data->remote_ble_device.BDAddr[2], p_param_ble_data->remote_ble_device.BDAddr[3],
              p_param_ble_data->remote_ble_device.BDAddr[4], p_param_ble_data->remote_ble_device.BDAddr[5]);
      LogInfo("    BD Addr type: %" PRIu32 "\n", p_param_ble_data->remote_ble_device.bd_addr_type);
      break;

    default:
      break;
  }
  /* USER CODE BEGIN APP_ble_cb_End */

  /* USER CODE END APP_ble_cb_End */
}

static void APP_error_cb(W6X_Status_t ret_w6x, char const *func_name)
{
  /* USER CODE BEGIN APP_error_cb_1 */

  /* USER CODE END APP_error_cb_1 */
  LogError("[%s] in %s API\n", W6X_StatusToStr(ret_w6x), func_name);
  /* USER CODE BEGIN APP_error_cb_2 */

  /* USER CODE END APP_error_cb_2 */
}

static void APP_setevent(EventGroupHandle_t *app_event, uint32_t evt)
{
  /* USER CODE BEGIN APP_setevent_1 */

  /* USER CODE END APP_setevent_1 */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (xPortIsInsideInterrupt())
  {
    xEventGroupSetBitsFromISR(*app_event, evt, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  else
  {
    xEventGroupSetBits(*app_event, evt);
  }
  /* USER CODE BEGIN APP_setevent_End */

  /* USER CODE END APP_setevent_End */
}
