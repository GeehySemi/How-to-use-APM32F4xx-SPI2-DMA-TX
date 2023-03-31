#include "main.h"

#define SPI_WRITE_CNT        10
#define SPI_BUFFER_LEN       30
static uint16_t gn_spi_wrtie_buffer[SPI_WRITE_CNT][SPI_BUFFER_LEN];
static uint16_t gn_spi_dma_buffer_idx;
uint8_t gb_spi_dma_started;

#define LL_MAX_DELAY    0xFFFFFFFF

void SystemClock_Config(void);
static void APM32_GPIO_Init(void);
static void APM32_DMA_Init(void);
static void APM32_SPI2_Init(void);

__STATIC_INLINE void LL_mDelay(uint32_t delay)
{
    __IO uint32_t tmp = SysTick->CTRL;  /* Clear the COUNTFLAG first */
    ((void)tmp);

    /* Add a period to guaranty minimum wait */
    if(delay < LL_MAX_DELAY)
    {
        ++delay;
    }

    while(delay)
    {
        if((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0)
        {
            --delay;
        }
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    __disable_irq();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    APM32_GPIO_Init();
    APM32_SPI2_Init();
    APM32_DMA_Init();

    LL_mDelay(50);

    /* Init. buffer for SPI writing */
    for(uint16_t i=0 ; i<SPI_WRITE_CNT ; ++i)
    {
        for(uint16_t j=0 ; j<SPI_BUFFER_LEN ; ++j)
        {
            gn_spi_wrtie_buffer[i][j] = 400 * (i+1);
        }
    }
    __enable_irq();

    while (1)
    {
        if(gb_spi_dma_started == 0)
        {
            gb_spi_dma_started = 1;
            
            SPI2_NSS_ENABLE;
            
            SPI_I2S_DisableDMA(SPI2, SPI_I2S_DMA_REQ_TX);
            
            while (DMA_ReadCmdStatus(DMA1_Stream4) != DISABLE)
            {
            }

            DMA_ConfigMemoryTarget(DMA1_Stream4, (uint32_t)gn_spi_wrtie_buffer[gn_spi_dma_buffer_idx], DMA_MEMORY_0);
            DMA_ConfigDataNumber(DMA1_Stream4, SPI_BUFFER_LEN);
            DMA_Enable(DMA1_Stream4);   /* Start DMA for SPI wirting */
            
            while (DMA_ReadCmdStatus(DMA1_Stream4) != ENABLE)
          {
          }
            
            SPI_I2S_EnableDMA(SPI2, SPI_I2S_DMA_REQ_TX);
            
            ++gn_spi_dma_buffer_idx;
            if(gn_spi_dma_buffer_idx == SPI_WRITE_CNT)
            {
                gn_spi_dma_buffer_idx = 0;
            }
        }
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    /** SystemFrequency / 1000 = 1ms */
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /** Capture error */
        while (1);
    }

    NVIC_ConfigPriorityGroup(NVIC_PRIORITY_GROUP_2);

    //RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void APM32_SPI2_Init(void)
{
    GPIO_Config_T GPIO_InitStructure;
    SPI_Config_T  SPI2_InitStructure;

    /** Enable related Clock */
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI2);

    /** Config SPI2 PinAF */
    GPIO_ConfigPinAF(SPI2_CLK_GPIO_Port, GPIO_PIN_SOURCE_13, GPIO_AF_SPI2);
    GPIO_ConfigPinAF(SPI2_MOSI_GPIO_Port, GPIO_PIN_SOURCE_15, GPIO_AF_SPI2);

    /** Config SPI2, SCK=PB13, MOSI=PB15 */
    GPIO_ConfigStructInit(&GPIO_InitStructure);
    GPIO_InitStructure.pin = SPI2_CLK_Pin | SPI2_MOSI_Pin;
    GPIO_InitStructure.speed = GPIO_SPEED_100MHz;
    GPIO_InitStructure.mode = GPIO_MODE_AF;
    GPIO_InitStructure.otype = GPIO_OTYPE_PP;
    GPIO_InitStructure.pupd = GPIO_PUPD_NOPULL;
    GPIO_Config(GPIOB, &GPIO_InitStructure);

    /** Config SPI2 */
    SPI_ConfigStructInit(&SPI2_InitStructure);
    SPI2_InitStructure.direction = SPI_DIRECTION_2LINES_FULLDUPLEX;
    SPI2_InitStructure.mode = SPI_MODE_MASTER;
    SPI2_InitStructure.length = SPI_DATA_LENGTH_16B;
    SPI2_InitStructure.polarity = SPI_CLKPOL_LOW;
    SPI2_InitStructure.phase = SPI_CLKPHA_2EDGE;
    SPI2_InitStructure.nss = SPI_NSS_SOFT;
    SPI2_InitStructure.baudrateDiv = SPI_BAUDRATE_DIV_2;
    SPI2_InitStructure.firstBit = SPI_FIRSTBIT_MSB;
    SPI2_InitStructure.crcPolynomial = 7;
    SPI_Config(SPI2, &SPI2_InitStructure);
    SPI_DisableCRC(SPI2);

    //SPI_I2S_EnableDMA(SPI2, SPI_I2S_DMA_REQ_TX);

    /** Enable SPI  */
    SPI_Enable(SPI2);
}

/**
  * Enable DMA controller clock
  */
static void APM32_DMA_Init(void)
{
  /* SPI2 DMA Init */
  /* DMA Configure */
    DMA_Config_T dmaConfig;

    /* Enable DMA clock */
    RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_DMA1);

    DMA_Reset(DMA1_Stream4);

    while (DMA_ReadCmdStatus(DMA1_Stream4) != DISABLE)
    {
    }

    /* Configure DMA Stream */
    /* size of buffer*/
    dmaConfig.bufferSize = 1;
    /* set memory Data Size*/
    dmaConfig.memoryDataSize = DMA_MEMORY_DATA_SIZE_HALFWORD;
    /* Set peripheral Data Size*/
    dmaConfig.peripheralDataSize = DMA_PERIPHERAL_DATA_SIZE_HALFWORD;
    /* Enable Memory Address increase*/
    dmaConfig.memoryInc = DMA_MEMORY_INC_ENABLE;
    /* Disable Peripheral Address increase*/
    dmaConfig.peripheralInc = DMA_PERIPHERAL_INC_DISABLE;
    /* Reset Circular Mode*/
    dmaConfig.loopMode = DMA_MODE_NORMAL;
    /* set priority*/
    dmaConfig.priority = DMA_PRIORITY_HIGH;
    /* read from peripheral*/
    dmaConfig.dir = DMA_DIR_MEMORYTOPERIPHERAL;
    /* Set memory Address*/
    dmaConfig.memoryBaseAddr = (uint32_t)0;
    /* Set Peripheral Address*/
    dmaConfig.peripheralBaseAddr = (uint32_t)&SPI2->DATA;

    dmaConfig.channel = DMA_CHANNEL_0;
    dmaConfig.fifoMode = DMA_FIFOMODE_DISABLE;
    dmaConfig.fifoThreshold = DMA_FIFOTHRESHOLD_FULL;
    dmaConfig.peripheralBurst = DMA_PERIPHERALBURST_SINGLE;
    dmaConfig.memoryBurst = DMA_MEMORYBURST_SINGLE;
    DMA_Config(DMA1_Stream4, &dmaConfig);

    /* Clear DMA TF flag*/
    DMA_ClearIntFlag(DMA1_Stream4, DMA_INT_TCIFLG4);
    /* Enable DMA Interrupt*/
    DMA_EnableInterrupt(DMA1_Stream4, DMA_INT_TCIFLG);
    DMA_EnableInterrupt(DMA1_Stream4, DMA_INT_TEIFLG);
    //DMA_EnableInterrupt(DMA1_Stream4, DMA_INT_FEIFLG);

    NVIC_EnableIRQRequest(DMA1_STR4_IRQn, 0, 1);

    DMA_Disable(DMA1_Stream4);
    //DMA_Enable(DMA1_Stream4);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void APM32_GPIO_Init(void)
{
    GPIO_Config_T  GPIO_InitStruct;

    /** Enable the GPIO_Clock */
    RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOB);

    /**/
    GPIO_SetBit(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin);

    /**/
    GPIO_ResetBit(TP10_GPIO_Port, TP10_Pin);

    /**/
    GPIO_InitStruct.pin = SPI2_NSS_Pin;
    GPIO_InitStruct.mode = GPIO_MODE_OUT;
    GPIO_InitStruct.speed = GPIO_SPEED_100MHz;
    GPIO_InitStruct.otype = GPIO_OTYPE_PP;
    GPIO_InitStruct.pupd = GPIO_PUPD_NOPULL;
    GPIO_Config(SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.pin = TP10_Pin;
    GPIO_InitStruct.mode = GPIO_MODE_OUT;
    GPIO_InitStruct.speed = GPIO_SPEED_100MHz;
    GPIO_InitStruct.otype = GPIO_OTYPE_PP;
    GPIO_InitStruct.pupd = GPIO_PUPD_NOPULL;
    GPIO_Config(TP10_GPIO_Port, &GPIO_InitStruct);
}

