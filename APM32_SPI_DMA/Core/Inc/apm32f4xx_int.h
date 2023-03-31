/*!
 * @file       apm32f4xx_int.h
 *
 * @brief      This file contains the headers of the interrupt handlers
 *
 * @version     V1.0.2
 *
 * @date        2022-06-23
 *
 * @attention
 *
 *  Copyright (C) 2021-2022 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be usefull and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/* Define to prevent recursive inclusion */
#ifndef __APM32F4XX_INT_H
#define __APM32F4XX_INT_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes */
#include "apm32f4xx.h"

/** @addtogroup Examples
  @{
  */

/** @addtogroup ADC_DMA
  @{
  */

/** @defgroup ADC_DMA_INT_Functions INT_Functions
  @{
*/

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel4_IRQHandler(void);
void TIM1_UP_TIM16_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_DAC_IRQHandler(void);
void I2C4_EV_IRQHandler(void);
void I2C4_ER_IRQHandler(void);
/* USER CODE BEGIN EFP */


#ifdef __cplusplus
}
#endif

#endif /*__APM32F4XX_INT_H */

/**@} end of group ADC_DMA */
/**@} end of group Examples */
