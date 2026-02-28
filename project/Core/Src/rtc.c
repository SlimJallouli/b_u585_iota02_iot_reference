#include "main.h"

extern RTC_HandleTypeDef hrtc;

/**
  * @brief  Writes a 32-bit value to a specific RTC Backup Register.
  * @param  data: The 32-bit value to store.
  * @retval HAL Status
  */
HAL_StatusTypeDef RTC_SaveBackupData(uint32_t data)
{
    // 1. Enable access to the Backup domain
    // (Required for U5 to modify RTC/TAMP registers)
    HAL_PWR_EnableBkUpAccess();

    // 2. Write the actual data to Data Register 1
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, data);

    // 3. Optional: Write a "Magic Number" to Data Register 0
    // This allows you to check on boot if the data in DR1 is valid
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0xCAFEFEED);

    // 4. Disable access to protect against accidental corruption
    HAL_PWR_DisableBkUpAccess();

    return HAL_OK;
}
