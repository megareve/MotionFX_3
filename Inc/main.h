/**
 ******************************************************************************
 * @file    main.h
 * @author  MEMS Software Solutions Team
 * @brief   This file contains definitions for the main.c file.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#ifdef USE_IKS01A2
//#include "nucleo_f401re_bus.h"
//#include "nucleo_f401re_errno.h"
//#include "iks01a2_env_sensors.h"
//#include "iks01a2_env_sensors_ex.h"
//#include "iks01a2_motion_sensors.h"
//#include "iks01a2_motion_sensors_ex.h"
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define STORE_CALIB_FLASH

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))

/* Definition for TIMx clock resources : Timer used for algorithm */
#define TIM_ALGO                          TIM3
#define TIM_ALGO_CLK_ENABLE               __TIM3_CLK_ENABLE
#define TIM_ALGO_CLK_DISABLE              __TIM3_CLK_DISABLE

/* Definition for TIMx's NVIC */
#define TIM_ALGO_IRQn                     TIM3_IRQn
#define TIM_ALGO_IRQHandler               TIM3_IRQHandler

#elif (defined (USE_STM32L0XX_NUCLEO))

/* Definition for TIMx clock resources : Timer used for algorithm */
#define TIM_ALGO                          TIM2
#define TIM_ALGO_CLK_ENABLE               __TIM2_CLK_ENABLE
#define TIM_ALGO_CLK_DISABLE              __TIM2_CLK_DISABLE

/* Definition for TIMx's NVIC */
#define TIM_ALGO_IRQn                     TIM2_IRQn
#define TIM_ALGO_IRQHandler               TIM2_IRQHandler

#else
#error Not supported platform
#endif

/* Enable sensor masks */
#define PRESSURE_SENSOR                         0x00000001U
#define TEMPERATURE_SENSOR                      0x00000002U
#define HUMIDITY_SENSOR                         0x00000004U
#define UV_SENSOR                               0x00000008U /* for future use */
#define ACCELEROMETER_SENSOR                    0x00000010U
#define GYROSCOPE_SENSOR                        0x00000020U
#define MAGNETIC_SENSOR                         0x00000040U

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} IKS01A2_MOTION_SENSOR_Axes_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} IKS01A2_MOTION_SENSOR_AxesRaw_t;

/* Exported functions --------------------------------------------------------*/
void Error_Handler(void);
//void RTC_DateRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t dw);
//void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
