/*
 * FreeRTOS STM32U5 Reference Integration
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

#ifndef _SYS_EVT_H
#define _SYS_EVT_H

#include "FreeRTOS.h"
#include "event_groups.h"

#define EVT_MASK_FS_READY          (0x01<<0)
#define EVT_MASK_NET_INIT          (0x01<<1)
#define EVT_MASK_NET_CONNECTED     (0x01<<2)
#define EVT_MASK_NET_DISCONNECTED  (0x01<<3)
#define EVT_MASK_MQTT_INIT         (0x01<<4)
#define EVT_MASK_MQTT_CONNECTED    (0x01<<5)
#define EVT_MASK_NET_READABLE      (0x01<<6)

#if DEMO_HOME_ASSISTANT
#define EVT_OTA_UPDATE_AVAILABLE   (0x01 << 0)
#define EVT_OTA_UPDATE_START       (0x01 << 1)
#define EVT_OTA_COMPLETED          (0x01 << 2)
#define EVT_COMMAND_RESET          (0x01 << 3)
#endif

extern EventGroupHandle_t xSystemEvents;

#endif /* _SYS_EVT_H */
