/*
 * rtc.h
 *
 *  Created on: Feb 27, 2026
 *      Author: stred
 */

#ifndef SRC_RTC_RTC_H_
#define SRC_RTC_RTC_H_

#include "main.h"

HAL_StatusTypeDef RTC_SaveBackupData(uint32_t data);
int Check_RTC_Backup_Integrity(uint32_t expected_data);

#endif /* SRC_RTC_RTC_H_ */
