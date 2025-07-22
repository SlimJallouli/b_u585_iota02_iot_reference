/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

#include "safea1_conf.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BOARD "B_U585_IOTA02"
#define RunTimeStats_Timer htim5
#define xConsoleHandle huart1
#define w6x_spi hspi1
#define MXCHIP_SPI hspi2
#define democonfigDEVICE_PREFIX "stm32u585"
#define USE_SENSORS 1
#define RESERVED_OTA_SECTORS 96
#define OTA_FILE_NAME "b_u585_iota02_iot_reference"
#define USE_PRE_LOADED_HOST_KEYS 0
#define MXCHIP_FLOW_Pin GPIO_PIN_15
#define MXCHIP_FLOW_GPIO_Port GPIOG
#define MXCHIP_FLOW_EXTI_IRQn EXTI15_IRQn
#define ARD_D06_Pin GPIO_PIN_6
#define ARD_D06_GPIO_Port GPIOB
#define MXCHIP_MOSI_Pin GPIO_PIN_4
#define MXCHIP_MOSI_GPIO_Port GPIOD
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define ARD_D05_Pin GPIO_PIN_0
#define ARD_D05_GPIO_Port GPIOE
#define MXCHIP_MISO_Pin GPIO_PIN_3
#define MXCHIP_MISO_GPIO_Port GPIOD
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define USER_Button_Pin GPIO_PIN_13
#define USER_Button_GPIO_Port GPIOC
#define USER_Button_EXTI_IRQn EXTI13_IRQn
#define LED_RED_Pin GPIO_PIN_6
#define LED_RED_GPIO_Port GPIOH
#define MXCHIP_SCK_Pin GPIO_PIN_1
#define MXCHIP_SCK_GPIO_Port GPIOD
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOH
#define T_VCP_RX_Pin GPIO_PIN_10
#define T_VCP_RX_GPIO_Port GPIOA
#define T_VCP_TX_Pin GPIO_PIN_9
#define T_VCP_TX_GPIO_Port GPIOA
#define MXCHIP_NOTIFY_Pin GPIO_PIN_14
#define MXCHIP_NOTIFY_GPIO_Port GPIOD
#define MXCHIP_NOTIFY_EXTI_IRQn EXTI14_IRQn
#define ARD_SPI_MISO_Pin GPIO_PIN_14
#define ARD_SPI_MISO_GPIO_Port GPIOE
#define ARD_D03_Pin GPIO_PIN_2
#define ARD_D03_GPIO_Port GPIOB
#define ARD_D03_EXTI_IRQn EXTI2_IRQn
#define ARD_SPI_SCK_Pin GPIO_PIN_13
#define ARD_SPI_SCK_GPIO_Port GPIOE
#define MXCHIP_NSS_Pin GPIO_PIN_12
#define MXCHIP_NSS_GPIO_Port GPIOB
#define STSAFE_EN_Pin GPIO_PIN_11
#define STSAFE_EN_GPIO_Port GPIOF
#define ARD_SPI_MOSI_Pin GPIO_PIN_15
#define ARD_SPI_MOSI_GPIO_Port GPIOE
#define MXCHIP_RESET_Pin GPIO_PIN_15
#define MXCHIP_RESET_GPIO_Port GPIOF
#define ARD_D10_Pin GPIO_PIN_12
#define ARD_D10_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
/********** X-NUCLEO-WW611M1 Pin definition ***********/
#if !defined(BOOT_Pin)
#define BOOT_Pin                                ARD_D06_Pin
#define BOOT_GPIO_Port                          ARD_D06_GPIO_Port
#endif

#if !defined(CHIP_EN_Pin)
#define CHIP_EN_Pin                             ARD_D05_Pin
#define CHIP_EN_GPIO_Port                       ARD_D05_GPIO_Port
#endif

#if !defined(SPI_CS_Pin)
#define SPI_CS_Pin                              ARD_D10_Pin
#define SPI_CS_GPIO_Port                        ARD_D10_GPIO_Port
#endif

#if !defined(SPI_RDY_Pin)
#define SPI_RDY_Pin                             ARD_D03_Pin
#define SPI_RDY_GPIO_Port                       ARD_D03_GPIO_Port
#define SPI_RDY_EXTI_IRQn                       ARD_D03_EXTI_IRQn
#endif

/********** X-NUCLEO-STSAFE Pin definition ***********/
#if !defined(STSAFE_EN_Pin)
#define STSAFE_EN_Pin                           ARD_D02_Pin
#define STSAFE_EN_GPIO_Port                     ARD_D02_GPIO_Port
#endif

/********** MXCHIP EMW3080 Pin definition ***********/
#if !defined(MXCHIP_NSS_Pin)
#define MXCHIP_NSS_Pin                          STMOD_01_Pin
#define MXCHIP_NSS_GPIO_Port                    STMOD_01_GPIO_Port
#endif

#if !defined(MXCHIP_RESET_Pin)
#define MXCHIP_RESET_Pin                        STMOD_08_Pin
#define MXCHIP_RESET_GPIO_Port                  STMOD_08_GPIO_Port
#endif

#if !defined(MXCHIP_FLOW_Pin)
#define MXCHIP_FLOW_Pin                         STMOD_17_Pin
#define MXCHIP_FLOW_GPIO_Port                   STMOD_17_GPIO_Port
#define MXCHIP_FLOW_EXTI_IRQn                   STMOD_17_EXTI_IRQn
#endif

#if !defined(MXCHIP_NOTIFY_Pin)
#define MXCHIP_NOTIFY_Pin                       STMOD_19_Pin
#define MXCHIP_NOTIFY_GPIO_Port                 STMOD_19_GPIO_Port
#define MXCHIP_NOTIFY_EXTI_IRQn                 STMOD_19_EXTI_IRQn
#endif

/************ Board LED Pin configuration *************/
#define LED_RED_ON                              GPIO_PIN_RESET
#define LED_RED_OFF                             GPIO_PIN_SET

/**************** MbedTLS debug config ****************/
#define MBEDTLS_DEBUG_NO_DEBUG                  0 /* No debug messages are displayed                                        */
#define MBEDTLS_DEBUG_ERROR                     1 /* Only error messages are shown                                          */
#define MBEDTLS_DEBUG_CHANGE                    2 /* Messages related to state changes in the SSL/TLS process are displayed */
#define MBEDTLS_DEBUG_INFO                      3 /* Provides general information about the SSL/TLS process                 */
#define MBEDTLS_DEBUG_VERBOSE                   4 /* Displays detailed debug information, including low-level operations    */

#define MBEDTLS_DEBUG_THRESHOLD                 MBEDTLS_DEBUG_INFO

/******************** Tasks config ********************/
#define DEMO_PUB_SUB                            0   // Publish/Subscribe Example
#define DEMO_OTA                                1   // OTA Update Example
#define DEMO_ENV_SENSOR                         1   // Environmental Sensor Example
#define DEMO_LIGHT_SENSOR                       1   // Light Sensor Example
#define DEMO_MOTION_SENSOR                      0   // Motion Sensor Example
#define DEMO_SHADOW                             0   // AWS IoT Shadow Example
#define DEMO_DEFENDER                           1   // AWS IoT Defender Example
#define DEMO_LED                                1   // LED Control Example
#define DEMO_BUTTON                             1   // Button Status Example
#if !defined(ST67W6X_NCP)
#define DEMO_HOME_ASSISTANT                     1   // Home Assistant Discovery Example
#endif
#define DEMO_ECHO_SERVER                        0   // Echo server example
#define DEMO_ECHO_CLIENT                        0   // Echo Client example
#define DEMO_PING                               0   // Ping example
#if defined(ST67W6X_NCP)
#define DEMO_SNTP                               1   // SNTP example
#endif

#define MQTT_ENABLED                            (DEMO_PUB_SUB       || \
                                                 DEMO_OTA           || \
                                                 DEMO_ENV_SENSOR    || \
												                         DEMO_LIGHT_SENSOR  || \
                                                 DEMO_MOTION_SENSOR || \
                                                 DEMO_SHADOW        || \
												                         DEMO_LED           || \
																		             DEMO_BUTTON        || \
                                                 defined(DEMO_FLEET_PROVISION))

#define USE_AVG_TEMP                            1

#define TASK_PRIO_OTA                           (tskIDLE_PRIORITY      + 1 )
#define TASK_PRIO_fleetProvisioning             (tskIDLE_PRIORITY      + 1 )
#define TASK_PRIO_SNTP                          (tskIDLE_PRIORITY      + 2 )
#define TASK_PRIO_PING                          (tskIDLE_PRIORITY      + 3 )
#define TASK_PRIO_ECHO_SERVER                   (tskIDLE_PRIORITY      + 4 )
#define TASK_PRIO_ECHO_CLIENT                   (tskIDLE_PRIORITY      + 5 )
#define TASK_PRIO_DEFENDER                      (tskIDLE_PRIORITY      + 6 )
#define TASK_PRIO_BUTTON                        (tskIDLE_PRIORITY      + 6 )
#define TASK_PRIO_SHADOW                        (tskIDLE_PRIORITY      + 7 )
#define TASK_PRIO_LED                           (tskIDLE_PRIORITY      + 7 )
#define TASK_PRIO_PUBLISH                       (tskIDLE_PRIORITY      + 8 )
#define TASK_PRIO_LIGHT                         (tskIDLE_PRIORITY      + 9 )
#define TASK_PRIO_ENV                           (tskIDLE_PRIORITY      + 10)
#define TASK_PRIO_MOTION                        (tskIDLE_PRIORITY      + 11)
#define TASK_PRIO_HS                            (tskIDLE_PRIORITY      + 12)
#define TASK_PRIO_CLI                           (tskIDLE_PRIORITY      + 13)
#define TASK_PRIO_MQTTA_AGENT                   (tskIDLE_PRIORITY      + 14)
#define TASK_PRIO_W6X                           (TASK_PRIO_MQTTA_AGENT + 1 )
#define TASK_PRIO_MXCHIP                        (tskIDLE_PRIORITY      + 23)
#define TASK_PRIO_NET_ETH                       (TASK_PRIO_MQTTA_AGENT + 1 )
#define TASK_PRIO_SUBSCRIPTION                  (tskIDLE_PRIORITY      + 25)  /** Priority of the subscription process task        */

#define TASK_STACK_SIZE_OTA                     4096/** Stack size of the OAT process task               */
#define TASK_STACK_SIZE_SNTP                    2024/** Stack size of the vSNTPTask process task         */
#define TASK_STACK_SIZE_DEFENDER                2024/** Stack size of the AWSDefender process task       */
#define TASK_STACK_SIZE_SHADOW                  2024/** Stack size of the ShadowDevice process task      */
#define TASK_STACK_SIZE_LED                     2024/** Stack size of the LED process task               */
#define TASK_STACK_SIZE_BUTTON                  2024/** Stack size of the Button process task            */
#define TASK_STACK_SIZE_PUBLISH                 2024/** Stack size of the publish process task           */
#define TASK_STACK_SIZE_ENV                     2024/** Stack size of the EnvSense process task          */
#define TASK_STACK_SIZE_LIGHT                   2024/** Stack size of the LightSense process task        */
#define TASK_STACK_SIZE_MOTION                  2024/** Stack size of the MotionS process task           */
#define TASK_STACK_SIZE_HS                      2024/** Stack size of the Home Assistant process task    */
#define TASK_STACK_SIZE_CLI                     2048/** Stack size of the CLI process task               */
#define TASK_STACK_SIZE_MQTT_AGENT              2048/** Stack size of the MQTTAgent process task         */
#define TASK_STACK_SIZE_W6X                     2048/** Stack size of the W6X process task               */
#define TASK_STACK_SIZE_MXCHIP                  1024/** Stack size of the MXCHIP process task            */
#define TASK_STACK_SIZE_SUBSCRIPTION            1024/** Stack size of the MQTT subscription process task */
#define TASK_STACK_SIZE_NET_ETH                 1024/** Stack size of the Ethernet task                  */
#define TASK_STACK_SIZE_fleetProvisioning       1024/** Stack size of the fleetProvisioning task         */

/******************** W6X debug config ********************/
#define W61_AT_LOG_ENABLE                       0         /* w61_driver_config.h */
#define SYS_DBG_ENABLE_TA4                      0         /* w61_driver_config.h */
#define W6X_TRACE_RECORDER_DBG_LEVEL            LOG_ERROR /* trcRecorder.h       */
/** Global verbosity level (LOG_NONE, LOG_ERROR, LOG_WARNING, LOG_INFO, LOG_DEBUG) */
#define W6X_LOG_LEVEL                           LOG_WARN

/******************** W6X Tasks config ********************/
#define W61_ATD_RX_TASK_PRIO                    20         /* w61_at_rx_parser.h, W61 AT Rx parser task priority, recommended to be higher than application tasks */
#define SPI_THREAD_PRIO                         17         /* spi_iface.c        */

/********************* Board config *********************/
#define democonfigMAX_THING_NAME_LENGTH         128

#if defined(MXCHIP)
#define CONNECTIVITY                            "MXCHIP"
#endif

#if defined(ST67W6X_NCP)
#define CONNECTIVITY                            "ST67_NCP"
#endif

#if defined(ETHERNET)
#define CONNECTIVITY                            "Ethernet"
#define ETH_RX_BUFFER_SIZE	1524
#endif

/* Select where the certs and keys are located */
#if defined(__USE_STSAFE__)
#define PKCS11_PAL_LITTLEFS                     0
#define PKCS11_PAL_STSAFE                       1
#else
#define PKCS11_PAL_LITTLEFS                     1
#define PKCS11_PAL_STSAFE                       0
#endif

/* Select where the KV_STORE is located */
#if !defined(__USE_STSAFE__)
  #define KV_STORE_NVIMPL_LITTLEFS              1
  #define KV_STORE_NVIMPL_ARM_PSA               0
  #define KV_STORE_NVIMPL_STSAFE                0
#else
  #define KV_STORE_NVIMPL_LITTLEFS              0
  #define KV_STORE_NVIMPL_ARM_PSA               0
  #define KV_STORE_NVIMPL_STSAFE                1
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
