/* MiCO Team
 * Copyright (c) 2017 MXCHIP Information Tech. Co.,Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "platform_peripheral.h"
#include "stm32f412zx.h"

RTC_HandleTypeDef RTCHandle;

OSStatus platform_mcu_powersave_disable( void )
{
    return kUnsupportedErr;
}

OSStatus platform_mcu_powersave_enable( void )
{
    return kUnsupportedErr;
}

void platform_mcu_enter_standby( uint32_t secondsToWakeup )
{
    time_t seconds;
    struct time * timeinfo;

    rtc_init();
    //clear WUF flag
    SET_BIT(PWR->CR , (PWR_FLAG_WU) << 2U);

    RTCHandle.Instance = RTC;

    HAL_RTCEx_SetWakeUpTimer_IT(&RTCHandle, secondsToWakeup,
                                    RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

    HAL_PWR_EnterSTANDBYMode( );
}

