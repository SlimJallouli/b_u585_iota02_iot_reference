/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : custom_bus.h
  * @brief          : header file for the BSP BUS IO driver
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
#ifndef CUSTOM_BUS_OS_H
#define CUSTOM_BUS_OS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "custom_bus.h"

/** @defgroup CUSTOM_LOW_LEVEL_Exported_Variables LOW LEVEL Exported Constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup CUSTOM_BUS_Exported_Functions
  * @{
  */
#if defined(BUS_I2C1_INSTANCE)
/* BUS IO driver over I2C Peripheral */
int32_t BSP_I2C1_Init_OS(void);
int32_t BSP_I2C1_DeInit_OS(void);
int32_t BSP_I2C1_IsReady_OS(uint16_t DevAddr, uint32_t Trials);
int32_t BSP_I2C1_WriteReg_OS(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_ReadReg_OS(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_WriteReg16_OS(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_ReadReg16_OS(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_Send_OS(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_Recv_OS(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_SendRecv_OS(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata, uint16_t Length);
#endif

#if defined(BUS_I2C2_INSTANCE)
/* BUS IO driver over I2C Peripheral */
int32_t BSP_I2C2_Init_OS(void);
int32_t BSP_I2C2_DeInit_OS(void);
int32_t BSP_I2C2_IsReady_OS(uint16_t DevAddr, uint32_t Trials);
int32_t BSP_I2C2_WriteReg_OS(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_ReadReg_OS(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_WriteReg16_OS(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_ReadReg16_OS(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_Send_OS(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_Recv_OS(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_SendRecv_OS(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata, uint16_t Length);
#endif
int32_t BSP_GetTick_OS(void);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_BUS_H */

