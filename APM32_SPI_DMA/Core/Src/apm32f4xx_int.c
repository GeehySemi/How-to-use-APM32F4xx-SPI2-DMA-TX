
/* Includes */
#include "main.h"
#include "apm32f4xx_int.h"

extern uint8_t gb_spi_dma_started;

/*!
 * @brief     This function handles NMI exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void NMI_Handler(void)
{
}

/*!
 * @brief     This function handles Hard Fault exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/*!
 * @brief     This function handles Memory Manage exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/*!
 * @brief     This function handles Bus Fault exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/*!
 * @brief     This function handles Usage Fault exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/*!
 * @brief     This function handles SVCall exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void SVC_Handler(void)
{
}

/*!
 * @brief     This function handles Debug Monitor exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void DebugMon_Handler(void)
{
}

/*!
 * @brief     This function handles PendSV_Handler exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void PendSV_Handler(void)
{
}

/*!
 * @brief     This function handles SysTick Handler
 *
 * @param     None
 *
 * @retval    None
 *
 */
void SysTick_Handler(void)
{

}

/**@} end of group ADC_DMA_INT_Functions */
/**@} end of group ADC_DMA */

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_STR4_IRQHandler(void)
{
    if(DMA_ReadStatusFlag(DMA1_Stream4, DMA_FLAG_TCIFLG4) == SET)
    {
        DMA_ClearStatusFlag(DMA1_Stream4, DMA_FLAG_TCIFLG4);
        DMA_Disable(DMA1_Stream4);

        while(SPI_I2S_ReadStatusFlag(SPI2, SPI_FLAG_BSY) == SET) {};

        gb_spi_dma_started = 0;
        SPI2_NSS_DISABLE;
    }
    else if(DMA_ReadStatusFlag(DMA1_Stream4, DMA_FLAG_TEIFLG4) == SET)
    {
        DMA_ClearStatusFlag(DMA1_Stream4, DMA_FLAG_TEIFLG4);
    }
    
    if(DMA_ReadStatusFlag(DMA1_Stream4, DMA_FLAG_FEIFLG4) == SET)
    {
        DMA_ClearStatusFlag(DMA1_Stream4, DMA_FLAG_FEIFLG4);
    }
}

