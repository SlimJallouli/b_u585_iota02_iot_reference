/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos_hooks.c
 * Description        : FreeRTOS Hook functions
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"

#include "logging_levels.h"
#define LOG_LEVEL    LOG_DEBUG
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <string.h>

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  static StaticTask_t timerTaskTCB;
  static StackType_t timerTaskStack[configTIMER_TASK_STACK_DEPTH];

  *ppxTimerTaskTCBBuffer = &timerTaskTCB;
  *ppxTimerTaskStackBuffer = timerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  static StaticTask_t idleTaskTCB;
  static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];

  *ppxIdleTaskTCBBuffer = &idleTaskTCB;
  *ppxIdleTaskStackBuffer = idleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

#if configUSE_IDLE_HOOK == 1
__weak void vApplicationIdleHook( void )
{
  vPetWatchdog();
  __WFI();
}
#endif

#if configUSE_MALLOC_FAILED_HOOK
__weak void vApplicationMallocFailedHook(void)
{
	LogError("Malloc Fail\n");
}
#endif

#if (configCHECK_FOR_STACK_OVERFLOW > 0)
__weak void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
    taskENTER_CRITICAL();

    LogSys( "Stack overflow in %s", pcTaskName );
    ( void ) xTask;

    vDoSystemReset();

    taskEXIT_CRITICAL();
}
#endif

void vDoSystemReset(void)
{
  vPetWatchdog();

  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
  {
    vTaskSuspendAll();
  }

  LogSys("System Reset in progress.");

  /* Drain log buffers */
  vDyingGasp();

  NVIC_SystemReset();
}

#if  configGENERATE_RUN_TIME_STATS
extern TIM_HandleTypeDef RunTimeStats_Timer;
#define pRunTimeStats_Timer (&RunTimeStats_Timer)

void configureTimerForRunTimeStats(void)
{
  HAL_TIM_Base_Stop(pRunTimeStats_Timer);
  pRunTimeStats_Timer->Instance->CNT = 0;
  HAL_TIM_Base_Start(pRunTimeStats_Timer);
}

unsigned long getRunTimeCounterValue(void)
{
  static configRUN_TIME_COUNTER_TYPE counter = 0;
  static uint32_t last_cnt_val = 0;
  uint32_t current_cnt_val = 0;
  uint32_t difference;

  current_cnt_val = pRunTimeStats_Timer->Instance->CNT;

  difference = current_cnt_val - last_cnt_val;

  last_cnt_val = current_cnt_val;

  counter += difference;

  return counter;
}

static configRUN_TIME_COUNTER_TYPE IRQ_Timer = 0;

void incrementIRQRunTime(configRUN_TIME_COUNTER_TYPE Initial_time)
{
  IRQ_Timer += getRunTimeCounterValue() - Initial_time;
}

configRUN_TIME_COUNTER_TYPE getIRQTimeCounterValue(void)
{
  return IRQ_Timer;
}
#endif

void vPetWatchdog(void)
{
#if defined(HAL_IWDG_MODULE_ENABLED)
  extern IWDG_HandleTypeDef hiwdg;
  HAL_IWDG_Refresh( &hiwdg );
#endif
}

/**
  * @brief This function provides minimum delay (in milliseconds) based
  *        on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay  specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(uint32_t Delay)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
  {
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = Delay;

    /* Add a freq to guarantee minimum wait */
    if (wait < HAL_MAX_DELAY)
    {
      wait += (uint32_t) (uwTickFreq);
    }

    while ((HAL_GetTick() - tickstart) < wait)
    {
    }
  }
  else
  {
    vTaskDelay(pdMS_TO_TICKS(Delay));
  }
}

#if defined(HAL_IWDG_MODULE_ENABLED)
void HAL_IWDG_EarlyWakeupCallback(IWDG_HandleTypeDef *hiwdg)
{
  LogWarn("IWDG EarlyWakeup");

  configASSERT(0);
}
#endif
