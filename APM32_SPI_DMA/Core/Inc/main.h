/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Includes */
#include "apm32f4xx.h"

/* Private includes ----------------------------------------------------------*/
#include "apm32f4xx_rcm.h"
#include "apm32f4xx_misc.h"

#include "apm32f4xx_gpio.h"
#include "apm32f4xx_spi.h"
#include "apm32f4xx_dma.h"

#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define SPI2_CLK_Pin GPIO_PIN_13
#define SPI2_CLK_GPIO_Port GPIOB
#define TP10_Pin GPIO_PIN_14
#define TP10_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB

#define TP10_LOW                GPIO_ResetBit(TP10_GPIO_Port, TP10_Pin)
#define TP10_HIGH               GPIO_SetBit(TP10_GPIO_Port, TP10_Pin)
#define TP10_TOGGLE             GPIO_ToggleBit(TP10_GPIO_Port, TP10_Pin)

#define SPI2_NSS_ENABLE         GPIO_ResetBit(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin)
#define SPI2_NSS_DISABLE        GPIO_SetBit(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin)

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
