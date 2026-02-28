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

/**
 * @brief Validates the RTC backup registers.
 * @return 1 if both the Magic Number and Data match, 0 otherwise.
 */
int Check_RTC_Backup_Integrity(uint32_t expected_data)
{
  // Read the Magic Number from DR0
  uint32_t magic = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);

  // Read the Data from DR1
  uint32_t actual_data = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);

  if (magic == 0xCAFEFEED && actual_data == expected_data)
  {
      return 1; // Success: Data is valid and matches 0xABCDEF01
  }

  return 0; // Failure: Data corrupted, battery lost, or not yet initialized
}
