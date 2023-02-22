
#ifdef CFG_CORE
/**********************************************************************/

#include "Core.h"

/**********************************************************************
// CoreTick
**********************************************************************/

inline uint32_t udivFast(uint64_t val64, const int Div, const int Shf) {
  const uint32_t S = (1ULL << 32) / Shf;
  const uint32_t K = Div / Shf;
  const uint32_t A = S / K;
  const uint32_t B = S % K;

  uint32_t hi = val64 >> 32;
  uint32_t lo = val64 >>  0;
  return (A*hi) + ((B*hi + (lo/Shf)) / K);
}


/**********************************************************************/
uint64_t micros64() {
  return CoreTick64() / CFG_Core_ClkFreq_MHz / 1;
}

uint64_t millis64() {
  return CoreTick64() / CFG_Core_ClkFreq_MHz / 1000;
}

uint32_t micros() {
  switch (CFG_Core_ClkFreq_MHz) {
    case 48: return udivFast(CoreTick64(), 48, _BV(4));
    case 72: return udivFast(CoreTick64(), 72, _BV(3));
    default: return (uint32_t)micros64();
  }
}

uint32_t millis()
{
  switch (CFG_Core_ClkFreq_MHz)
  {
    case 48: return udivFast(CoreTick64(), 48*1000, _BV(7));
    case 72: return udivFast(CoreTick64(), 72*1000, _BV(6));
    default: return (uint32_t)millis64();
  }
}


/**********************************************************************/
#ifdef CFG_CORE_CORETICK_SOURCE_DWT
  void CoreTick_begin() {
    BITMASK_SET(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk);  /// enable Debug block
    DWT->CYCCNT = 0;                                            /// reset counter
    BITMASK_SET(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk);             /// enable DWT counter
  }

  uint64_t last_cycle_count_64 = 0;
  uint64_t CoreTick64() {
    last_cycle_count_64 += DWT->CYCCNT - (uint32_t)(last_cycle_count_64);
    return last_cycle_count_64;
  }

  uint32_t CoreTick32() {
    return DWT->CYCCNT;
  }
#endif // CFG_CORE_CORETICK_SOURCE_DWT


/**********************************************************************/
#ifdef CFG_CORE_CORETICK_SOURCE_SYSTICK
  void CoreTick_begin() {
    SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;       /// set reload register
    SysTick->VAL  = 0;                             /// set SysTick Counter Value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_ENABLE_Msk;       /// enable SysTick Timer
  }

  uint64_t last_cycle_count_64 = 0;
  uint64_t CoreTick64() {
    uint32_t delta = (SysTick24() - ((uint32_t)last_cycle_count_64)) & 0xFFFFFF;
    last_cycle_count_64 += delta;
    return last_cycle_count_64;
  }

#endif // CFG_CORE_CORETICK_SOURCE_SYSTICK

/**********************************************************************/
#ifdef CFG_CORE_CFG_CORE_CORETICK_SOURCE_SYSTICK_IRQ

  void CoreTick_begin() {
    SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;       /// set reload register
    SysTick->VAL  = 0;                             /// set SysTick Counter Value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk       /// enable SysTick IRQ
                  | SysTick_CTRL_ENABLE_Msk;       /// enable SysTick Timer
  }

  volatile uint64_t last_cycle_count_64 = 0;

  void SysTick_Handler() {
    last_cycle_count_64 = (last_cycle_count_64 & (~0xFFFFFFULL)) + _BV(24);
  }

  uint64_t CoreTick64()
  {
    __disable_irq();

    uint32_t delta = (SysTick24() - ((uint32_t)last_cycle_count_64)) & 0xFFFFFF;
    last_cycle_count_64 += delta;

    __enable_irq();
    return last_cycle_count_64;
  }

#endif // CFG_CORE_CFG_CORE_CORETICK_SOURCE_SYSTICK_IRQ


/**********************************************************************/
#if defined(CFG_CORE_CORETICK_SOURCE_SYSTICK) || \
    defined(CFG_CORE_CFG_CORE_CORETICK_SOURCE_SYSTICK_IRQ)

  uint32_t CoreTick32() {
    return (uint32_t)CoreTick64();
  }

  uint32_t SysTick24() {
    return -(SysTick->VAL);
  }
#endif


/**********************************************************************
// Delay
**********************************************************************/

void delay_until_us(uint32_t target_us) {
  int delta = target_us - micros();
  if (delta > 0) delay_us(delta);
}

void delay_until_ms(uint32_t target_ms) {
  delay_until_us(target_ms * 1000);
}


/**********************************************************************/
#ifdef CFG_CORE_WAIT_MODE_EVENTS

  #ifdef STM32F10X_MD
    #define  DELAY_TIMER      TIM4
    #define  DELAY_TIMER_RCC  RCC_APB1Periph_TIM4
    #define  DELAY_TIMER_IRQ  TIM4_IRQn
  #endif

  #ifdef STM32F0XX
    #define  DELAY_TIMER      TIM14
    #define  DELAY_TIMER_RCC  RCC_APB1Periph_TIM14
    #define  DELAY_TIMER_IRQ  TIM14_IRQn
  #endif

  void delay_begin() {
    /// TIMER config (for delay)
    BITMASK_SET(RCC->APB1ENR, DELAY_TIMER_RCC);     /// enable clock

    DELAY_TIMER->CR1 = 0;                           /// disable Timer
    DELAY_TIMER->PSC = CFG_Core_ClkFreq_MHz - 1;        /// CLK = 1MHz
    DELAY_TIMER->ARR = 0xFFFF;                      /// reload value
    DELAY_TIMER->EGR = TIM_PSCReloadMode_Immediate; /// Generate an update event to reload the Prescaler values immediately

    BITMASK_SET(SCB->SCR, SCB_SCR_SEVONPEND_Msk);   /// enable events
    DELAY_TIMER->DIER = TIM_IT_Update;              /// enable update interrupt
  }

  void delay_us(uint32_t us) {
    if (us == 0)
      return;

    DELAY_TIMER->CNT = -us;                         /// 16 bit low
    DELAY_TIMER->CR1 = TIM_CR1_CEN;                 /// enable Timer
    uint32_t loop = (us - 1) >> 16;                 /// 16 bit high

    do {
      DELAY_TIMER->SR = 0;                          /// clear update flag
      DELAY_TIMER->SR;                              /// wait_process clear update flag
      NVIC_ClearPendingIRQ(DELAY_TIMER_IRQ);

      do __WFE(); while (DELAY_TIMER->SR == 0);     /// check TIM_SR_UIF

      CoreTick64();   // last_cycle_count_64 += us * CFG_Core_ClkFreq_MHz;
    }
    while (loop--);

    DELAY_TIMER->CR1 = 0;                           /// disable Timer
  }

  void delay_ms(uint32_t ms) {
    delay_us(ms * 1000);
  }
#endif // CFG_CORE_WAIT_MODE_EVENTS



/**********************************************************************
// Timeout
**********************************************************************/
#ifdef CFG_CORE_WAIT_MODE_EVENTS

  int timeout_loop;

  void timeout_start_ms(uint32_t ms)
  {
    timeout_start_us(ms * 1000);
  }

  void timeout_start_us(uint32_t us) {
    if (us == 0)
      return;

    DELAY_TIMER->CNT = -us;                 /// 16 bit low
    DELAY_TIMER->CR1 = TIM_CR1_CEN;         /// enable Timer

    DELAY_TIMER->SR = 0;                    /// clear update flag
    DELAY_TIMER->SR;                        /// wait_process clear update flag
    NVIC_ClearPendingIRQ(DELAY_TIMER_IRQ);

    timeout_loop = (us - 1) >> 16;          /// 16 bit high
  }


  bool timeout_is_busy() {
    __WFE();

    if (DELAY_TIMER->SR == 0)                   /// check TIM_SR_UIF
      return true;

    DELAY_TIMER->SR = 0;                        /// clear update flag
    DELAY_TIMER->SR;                            /// wait_process clear update flag
    NVIC_ClearPendingIRQ(DELAY_TIMER_IRQ);

    CoreTick64();                              /// update last_cycle_count_64

    return ((--timeout_loop) < 0) ? false : true;
  }


  void timeout_end() {
    DELAY_TIMER->CR1 = 0;      /// disable Timer
    timeout_loop = -1;         /// clear
  }
#endif // CFG_CORE_WAIT_MODE_EVENTS



/**********************************************************************/
#ifdef CFG_CORE_WAIT_MODE_CLASSIC
  void delay_begin() {}

  void delay_us(uint32_t us)
  {
    uint32_t finish = CoreTick32() + (us * CFG_Core_ClkFreq_MHz);
    while (((int32_t)(finish - CoreTick32())) > 0);
  }

  void delay_ms(uint32_t ms)
  {
    for (; ms; ms--)
      delay_us(1000);
  }
#endif // CFG_CORE_WAIT_MODE_CLASSIC



/**********************************************************************
// Core_begin
**********************************************************************/
void Core_begin() {
  CoreTick_begin();
  delay_begin();
  Flash_SetLatency(CFG_Core_Flash_Latency);
}



/**********************************************************************
// Core_EraseOptionBytes
**********************************************************************/
#ifdef STM32F10X_MD
  void Core_EraseOptionBytes() {
    if (OB->USER == 0xFFFF)  // reset value
      return;

    FLASH_Unlock();

    if (FLASH_EraseOptionBytes() != FLASH_COMPLETE)
      FLASH_EraseOptionBytes();   // try again

    FLASH_Lock();
  }
#endif


#ifdef STM32F0XX
  void Core_EraseOptionBytes() {
    if (OB->USER == 0xFFFF)  // reset value
      return;

    FLASH_Unlock();
    FLASH_OB_Unlock();

    if (FLASH_OB_Erase() != FLASH_COMPLETE)
      FLASH_OB_Erase();   // try again

    FLASH_OB_Lock();
    FLASH_Lock();
  }
#endif



/**********************************************************************
// Core_HSIConfig
**********************************************************************/
#ifdef STM32F0XX
  void Core_HSIConfig() {

    // Select HSI as system clock source
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI;

    // Disable PLL
    RCC->CR &= ~RCC_CR_PLLON;


    // Enable Prefetch Buffer and set Flash Latency
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    // HCLK = SYSCLK
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

    // PCLK = HCLK
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;

    // PLL configuration = (HSI/2) * 12 = ~48 MHz
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLMULL12);

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;

    // Wait till PLL is ready
    while((RCC->CR & RCC_CR_PLLRDY) == 0);

    // Select PLL as system clock source
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

    // Wait till PLL is used as system clock source
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL);

    // Disable HSE
    RCC->CR &= ~((uint32_t)RCC_CR_HSEON);
  }
#endif



/**********************************************************************
// Reset Flags
**********************************************************************/
uint32_t reset_flag_status;

void ResetFlags_Begin() {
  reset_flag_status = RCC->CSR;
  ResetFlags_Clear();
}

void ResetFlags_Clear() {
  BITMASK_SET(RCC->CSR, RCC_CSR_RMVF);
}

bool ResetFlags_Check(uint32_t flag) {
  return BITMASK_IS_SET(reset_flag_status, flag) ? true : false;
}



/**********************************************************************
// Flash
**********************************************************************/
void Flash_SetLatency(uint32_t WaitState) {
  FLASH->ACR = (FLASH->ACR & ~0b111) | WaitState;
}



/**********************************************************************
// GPIO
**********************************************************************/
#ifdef STM32F10X_MD
  inline uint32_t F103_GPIO_get_rcc_clock_mask(void *periph) {
    switch ((uint32_t)periph) {
      case ((uint32_t)GPIOA): return RCC_APB2Periph_GPIOA;
      case ((uint32_t)GPIOB): return RCC_APB2Periph_GPIOB;
      case ((uint32_t)GPIOC): return RCC_APB2Periph_GPIOC;
      default: return 0;
    }
  }

  inline void F103_GPIO_pinMode_output(GPIO_TypeDef* GPIOx, int pin, int mode_speed) {
    if (pin < 8) BITPOS_SET_X4(GPIOx->CRL, pin - 0, mode_speed);
    else         BITPOS_SET_X4(GPIOx->CRH, pin - 8, mode_speed);
  }

  inline void F103_GPIO_pinMode_input(GPIO_TypeDef* GPIOx, int pin, int mode) {
    if (pin < 8) BITPOS_SET_X4(GPIOx->CRL, pin - 0, mode);
    else         BITPOS_SET_X4(GPIOx->CRH, pin - 8, mode);

    if      (mode == GPIO_Mode_IPU)  GPIOx->BSRR = _BV(pin);    /// pull-UP
    else if (mode == GPIO_Mode_IPD)  GPIOx->BRR  = _BV(pin);    /// pull-DOWN
  }

  void F103_GPIO_external_interrupt_enable(GPIO_TypeDef* GPIOx, int pin, int trigger) {
    // selects the GPIO pin used as EXTI Line
         if (GPIOx == GPIOA) BITPOS_SET_X4(AFIO->EXTICR[pin >> 2], pin & 0x03, GPIO_PortSourceGPIOA);
    else if (GPIOx == GPIOB) BITPOS_SET_X4(AFIO->EXTICR[pin >> 2], pin & 0x03, GPIO_PortSourceGPIOB);
    else if (GPIOx == GPIOC) BITPOS_SET_X4(AFIO->EXTICR[pin >> 2], pin & 0x03, GPIO_PortSourceGPIOC);

    // trigger selection
    switch (trigger) {
      case EXTI_Trigger_Rising:
        BITMASK_SET(  EXTI->RTSR, _BV(pin));
        BITMASK_CLEAR(EXTI->FTSR, _BV(pin));
        break;

      case EXTI_Trigger_Falling:
        BITMASK_CLEAR(EXTI->RTSR, _BV(pin));
        BITMASK_SET(  EXTI->FTSR, _BV(pin));
        break;

      case EXTI_Trigger_Rising_Falling:
        BITMASK_SET(EXTI->RTSR, _BV(pin));
        BITMASK_SET(EXTI->FTSR, _BV(pin));
        break;
    }

    // enable interrupt request
    BITMASK_SET(EXTI->IMR, _BV(pin));
  }

#endif // STM32F10X_MD


/**********************************************************************/
#ifdef STM32F0XX

  inline uint32_t F030_GPIO_get_rcc_clock_mask(void *periph) {
    switch ((uint32_t)periph) {
      case ((uint32_t)GPIOA): return RCC_AHBPeriph_GPIOA;
      case ((uint32_t)GPIOB): return RCC_AHBPeriph_GPIOB;
      case ((uint32_t)GPIOC): return RCC_AHBPeriph_GPIOC;
      case ((uint32_t)GPIOF): return RCC_AHBPeriph_GPIOF;
      default: return 0;
    }
  }

  inline void F030_GPIO_pinMode_output_pupd(GPIO_TypeDef* GPIOx, int pin, int mode, int type, int speed, int pupd) {
    BITPOS_SET_X2(GPIOx->MODER,   pin, mode );
    BITPOS_SET_X1(GPIOx->OTYPER,  pin, type );
    BITPOS_SET_X2(GPIOx->OSPEEDR, pin, speed);
    BITPOS_SET_X2(GPIOx->PUPDR,   pin, pupd );
  }

  inline void F030_GPIO_pinMode_output(GPIO_TypeDef* GPIOx, int pin, int mode, int type, int speed) {
    F030_GPIO_pinMode_output_pupd(GPIOx, pin, mode, type, speed, GPIO_PuPd_NOPULL);
  }

  inline void F030_GPIO_pinMode_input(GPIO_TypeDef* GPIOx, int pin, int mode, int pupd) {
    BITPOS_SET_X2(GPIOx->MODER, pin, mode);
    BITPOS_SET_X2(GPIOx->PUPDR, pin, pupd);
  }

  inline void F030_GPIO_pinAF_select(GPIO_TypeDef* GPIOx, int pin, int selection) {
    BITPOS_SET_X4(GPIOx->AFR[pin / 8], pin & 0x07, selection);
  }

  void F030_GPIO_external_interrupt_enable(GPIO_TypeDef* GPIOx, int pin, int trigger) {
    // select the source input
         if (GPIOx == GPIOA) BITPOS_SET_X4(SYSCFG->EXTICR[pin >> 2], pin & 0x03, EXTI_PortSourceGPIOA);
    else if (GPIOx == GPIOB) BITPOS_SET_X4(SYSCFG->EXTICR[pin >> 2], pin & 0x03, EXTI_PortSourceGPIOB);

    // select trigger
    switch (trigger) {
      case EXTI_Trigger_Rising:
        BITMASK_SET(  EXTI->RTSR, _BV(pin));
        BITMASK_CLEAR(EXTI->FTSR, _BV(pin));
        break;

      case EXTI_Trigger_Falling:
        BITMASK_CLEAR(EXTI->RTSR, _BV(pin));
        BITMASK_SET(  EXTI->FTSR, _BV(pin));
        break;

      case EXTI_Trigger_Rising_Falling:
        BITMASK_SET(EXTI->RTSR, _BV(pin));
        BITMASK_SET(EXTI->FTSR, _BV(pin));
        break;
    }

    // enable interrupt request
    BITMASK_SET(EXTI->IMR, _BV(pin));
  }

#endif // STM32F0XX



/**********************************************************************
// PrintInfo
**********************************************************************/
#include <stdio.h>

#ifdef STM32F10X_MD

  void Core_PrintInfo() {
    printf("\n");
    printf("U_ID = %08X %08X %08X\n", FLASH_UID_2, FLASH_UID_1, FLASH_UID_0);
    printf("FLASH_SIZE = %d KB\n", FLASH_SIZE);
    printf("\n");
    printf("RCC_CR    = 0x%08X\n", RCC->CR);
    printf("RCC_CFGR  = 0x%08X\n", RCC->CFGR);
    printf("RCC_CSR   = 0x%08X\n", RCC->CSR);
    printf("FLASH_ACR = 0x%08X\n", FLASH->ACR);
    printf("\n");

    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    printf("RCC_SYSCLK_Frequency = %ld\n", RCC_ClocksStatus.SYSCLK_Frequency);
    printf("RCC_HCLK_Frequency   = %ld\n", RCC_ClocksStatus.HCLK_Frequency);
    printf("RCC_PCLK1_Frequency  = %ld\n", RCC_ClocksStatus.PCLK1_Frequency);
    printf("RCC_PCLK2_Frequency  = %ld\n", RCC_ClocksStatus.PCLK2_Frequency);
    printf("RCC_ADCCLK_Frequency = %ld\n", RCC_ClocksStatus.ADCCLK_Frequency);
    printf("\n");
  }
#endif // STM32F10X_MD


/**********************************************************************/
#ifdef STM32F0XX
  void Core_PrintInfo() {
    printf("\n");
    printf("U_ID = %08X %08X %08X\n", FLASH_UID_2, FLASH_UID_1, FLASH_UID_0);
    printf("FLASH_SIZE = %d KB\n", FLASH_SIZE);
    printf("\n");
    printf("RCC_CR    = 0x%08X\n", RCC->CR);
    printf("RCC_CFGR  = 0x%08X\n", RCC->CFGR);
    printf("RCC_CSR   = 0x%08X\n", RCC->CSR);
    printf("FLASH_ACR = 0x%08X\n", FLASH->ACR);

    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    printf("\n");
    printf("RCC_SYSCLK_Frequency = %ld\n", RCC_ClocksStatus.SYSCLK_Frequency);
    printf("RCC_HCLK_Frequency   = %ld\n", RCC_ClocksStatus.HCLK_Frequency);
    printf("RCC_PCLK_Frequency   = %ld\n", RCC_ClocksStatus.PCLK_Frequency);
    printf("RCC_ADCCLK_Frequency = %ld\n", RCC_ClocksStatus.ADCCLK_Frequency);
    printf("\n");
  }
#endif // STM32F0XX


uint16_t crc16(void *data, uint8_t len) {
  uint8_t *data8 = (uint8_t *)data;

  uint16_t crc = 0xFFFF;  // init value
  while (len--) {
    crc  = (crc >> 8) | (crc << 8);
    crc ^= (*data8++);
    crc ^= ((uint8_t)crc) >> 4;
    crc ^= (crc << 12);
    crc ^= (crc & 0xFF) << 5;
  }
  return crc;
}

uint16_t Core_GetUid16() {
  return crc16(&FLASH_UID_0, 12);
}



/**********************************************************************
// USART1
**********************************************************************/
#define  GPIO_Speed_TxPin   GPIO_Speed_2MHz

#ifdef STM32F10X_MD
  #define  USART1_TxData  USART1->DR
  #define  USART1_RxData  USART1->DR
  #define  USART1_Status  USART1->SR
#endif

#ifdef STM32F0XX
  #define  USART1_TxData  USART1->TDR
  #define  USART1_RxData  USART1->RDR
  #define  USART1_Status  USART1->ISR
#endif

/**********************************************************************/
#ifdef CFG_CORE_USART1_TX_MODE_DMA_CHANNEL2
  #define  USART1_TX_DMA_CHANNEL  DMA1_Channel2
  #define  USART1_TX_DMA_TC_FLAG  DMA_ISR_TCIF2

  #ifdef STM32F10X_MD
    #define USART1_TX_DMA_IRQ     DMA1_Channel2_IRQn
  #endif
  #ifdef STM32F0XX
    #define USART1_TX_DMA_IRQ     DMA1_Channel2_3_IRQn
  #endif
#endif // CFG_CORE_USART1_TX_MODE_DMA_CHANNEL2

#ifdef CFG_CORE_USART1_TX_MODE_DMA_CHANNEL4
  #define  USART1_TX_DMA_CHANNEL  DMA1_Channel4
  #define  USART1_TX_DMA_TC_FLAG  DMA_ISR_TCIF4

  #ifdef STM32F10X_MD
    #define USART1_TX_DMA_IRQ     DMA1_Channel4_IRQn
  #endif
  #ifdef STM32F0XX
    #define USART1_TX_DMA_IRQ     DMA1_Channel4_5_IRQn
  #endif
#endif // CFG_CORE_USART1_TX_MODE_DMA_CHANNEL4



/**********************************************************************
// USART1 TX
**********************************************************************/

void USART1_write(int data) {
  printf("%c", data);
}

void USART1_flushTx() {
  fflush(stdout);
}

/**********************************************************************/

#ifdef STM32F10X_MD
  inline void USART1_Remap_enable() {
    BITMASK_CLEAR(AFIO->MAPR, AFIO_MAPR_USART1_REMAP);  /// No remap (Tx/A9, Rx/A10)
  }

  inline void USART1_Remap_disable() {}
#endif

#ifdef STM32F0XX
  inline void USART1_Remap_enable() {
    F030_GPIO_pinAF_select(CFG_USART1_GPIO_SOURCE, CFG_CORE_USART1_TxPin, CFG_USART1_AF_SOURCE);
  }

  inline void USART1_Remap_disable() {}
#endif

/**********************************************************************/
#ifdef STM32F10X_MD
  void USART1_GPIO_enable() {
    F103_GPIO_pinMode_output(GPIOA, CFG_CORE_USART1_TxPin, GPIO_Mode_AF_PP | GPIO_Speed_TxPin);
  }

  void USART1_GPIO_disable() {
    GPIOA->BSRR = _BV(CFG_CORE_USART1_TxPin);  /// HIGH level
    F103_GPIO_pinMode_output(GPIOA, CFG_CORE_USART1_TxPin, GPIO_Mode_Out_PP | GPIO_Speed_TxPin);
  }
#endif

#ifdef STM32F0XX
  void USART1_GPIO_enable() {
    F030_GPIO_pinMode_output(CFG_USART1_GPIO_SOURCE, CFG_CORE_USART1_TxPin, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Speed_TxPin);
  }

  void USART1_GPIO_disable() {
    CFG_USART1_GPIO_SOURCE->BSRR = _BV(CFG_CORE_USART1_TxPin);  /// HIGH level
    F030_GPIO_pinMode_output(CFG_USART1_GPIO_SOURCE, CFG_CORE_USART1_TxPin, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_Speed_TxPin);
  }
#endif


/**********************************************************************/
#ifdef CFG_CORE_WAIT_MODE_EVENTS

  #ifdef CFG_CORE_DMA_BACKUP
    uint32_t core_dma_usart1tx_user_ccr;
    uint32_t core_dma_usart1tx_user_cpar;
    uint32_t core_dma_usart1tx_user_cmar;

    void Core_DMA_backup() {
      core_dma_usart1tx_user_cpar = USART1_TX_DMA_CHANNEL->CPAR;
      core_dma_usart1tx_user_cmar = USART1_TX_DMA_CHANNEL->CMAR;
      core_dma_usart1tx_user_ccr  = USART1_TX_DMA_CHANNEL->CCR;
    }

    void Core_DMA_restore() {
      USART1_TX_DMA_CHANNEL->CPAR = core_dma_usart1tx_user_cpar;
      USART1_TX_DMA_CHANNEL->CMAR = core_dma_usart1tx_user_cmar;
      USART1_TX_DMA_CHANNEL->CCR  = core_dma_usart1tx_user_ccr;
    }
  #else
    void Core_DMA_backup()  {}
    void Core_DMA_restore() {}
  #endif // CFG_CORE_DMA_BACKUP

  #ifdef STM32F10X_MD
    #define DMA_CCR_EN     DMA_CCR1_EN
    #define DMA_CCR_TCIE   DMA_CCR1_TCIE
  #endif

  #define USART1_TX_DMA_CHANNEL_CCR   ( DMA_DIR_PeripheralDST        \
                                      | DMA_MemoryInc_Enable         \
                                      | DMA_MemoryDataSize_Byte      \
                                      | DMA_PeripheralInc_Disable    \
                                      | DMA_PeripheralDataSize_Byte  \
                                      | DMA_M2M_Disable              \
                                      | DMA_Mode_Normal              \
                                      | DMA_Priority_Medium          \
                                      | DMA_CCR_EN )

  #ifdef STM32F10X_MD
    #define USART_Clear_TC_flag()    USART1->SR = 0
  #endif

  #ifdef STM32F0XX
    #define USART_Clear_TC_flag()    USART1->ICR = USART_ICR_TCCF
  #endif


  inline void USART1_DMA_enable(char *ptr, int len)
  {
    Core_DMA_backup();

    #ifdef STM32F0XX
      #ifdef CFG_CORE_USART1_TX_MODE_DMA_CHANNEL2
        BITMASK_CLEAR(SYSCFG->CFGR1, SYSCFG_DMARemap_USART1Tx);   /// DMA_USART1_Tx no remap
      #endif

      #ifdef CFG_CORE_USART1_TX_MODE_DMA_CHANNEL4
        BITMASK_SET(SYSCFG->CFGR1, SYSCFG_DMARemap_USART1Tx);     /// DMA_USART1_Tx remap
      #endif
    #endif

    BITMASK_SET(USART1->CR3, USART_CR3_DMAT);                   /// enable DMA transmitter

    USART1_TX_DMA_CHANNEL->CPAR  = (uint32_t)(&USART1_TxData);  /// peripheral address
    USART1_TX_DMA_CHANNEL->CMAR  = (uint32_t)(ptr);             /// memory address
    USART1_TX_DMA_CHANNEL->CNDTR = len;
    USART1_TX_DMA_CHANNEL->CCR   = USART1_TX_DMA_CHANNEL_CCR;   /// enable DMA
  }

  inline void USART1_DMA_disable() {
    USART1_TX_DMA_CHANNEL->CCR = 0;                             /// disable DMA
    BITMASK_CLEAR(USART1->CR3, USART_CR3_DMAT);                 /// disable DMA transmitter

    Core_DMA_restore();
  }


  void USART1_Tx_begin() {
    #ifdef STM32F0XX
      BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_SYSCFG);   /// SYSCFG clock enabled
    #endif

    BITMASK_SET(RCC->AHBENR, RCC_AHBPeriph_DMA1);         /// enable DMA1 clock
  }

  ATTRIBUTE_USED
  int _write(int file, char *ptr, int len) {
    USART1_Remap_enable();
    USART1_GPIO_enable();
    USART1_DMA_enable(ptr, len);

    BITMASK_SET(USART1->CR1, USART_CR1_TCIE);                  /// enable TC interrupt
    USART_Clear_TC_flag();
    //USART1_Status;
    NVIC_ClearPendingIRQ(USART1_IRQn);                         /// clear NVIC flag
    do __WFE(); while ((USART1_Status & USART_FLAG_TC) == 0);  /// wait_process until transmission complete
    BITMASK_CLEAR(USART1->CR1, USART_CR1_TCIE);                /// disable TC interrupt

    USART1_DMA_disable();
    USART1_GPIO_disable();
    USART1_Remap_disable();
    return len;
  }

#endif // CFG_CORE_WAIT_MODE_EVENTS


/**********************************************************************/
#ifdef CFG_CORE_WAIT_MODE_CLASSIC
  void USART1_Tx_begin() {}

  ATTRIBUTE_USED
  int _write(int file, char *ptr, int len) {
    USART1_Remap_enable();
    USART1_GPIO_enable();

    for (int i = len; i > 0; i--) {
      while ((USART1_Status & USART_FLAG_TXE) == 0);
      USART1_TxData = *ptr++;
    }
    while ((USART1_Status & USART_FLAG_TC) == 0);

    USART1_GPIO_disable();
    USART1_Remap_disable();
    return len;
  }
#endif // CFG_CORE_WAIT_MODE_CLASSIC



/**********************************************************************
// USART1 RX
**********************************************************************/
#if defined(CFG_CORE_USART1_RX_MODE_DMA_CHANNEL3) || \
    defined(CFG_CORE_USART1_RX_MODE_DMA_CHANNEL5) || \
    defined(CFG_CORE_USART1_RX_MODE_IRQ)

  #define USART1_RX_MODE_ENABLE

  uint8_t usart1_rx_buff[CFG_CORE_USART1_RX_BUFF_SIZE];
  int usart1_rx_buff_tail = 0;

  #ifdef CFG_CORE_USART1_RX_MODE_DMA_CHANNEL3
    #define  USART1_RX_DMA_CHANNEL  DMA1_Channel3
  #endif

  #ifdef CFG_CORE_USART1_RX_MODE_DMA_CHANNEL5
    #define  USART1_RX_DMA_CHANNEL  DMA1_Channel5
  #endif


/**********************************************************************/
  #ifdef CFG_CORE_USART1_RX_MODE_IRQ
    volatile int usart1_rx_buff_head = 0;

    void USART1_Rx_begin() {
      USART1->CR1 |= USART_Mode_Rx      /// enable RX mode
                  |  USART_CR1_RXNEIE;  /// enable RXNE interrupt
      NVIC_EnableIRQ(USART1_IRQn);      /// enable USART1_IRQHandler
    }

    void USART1_IRQHandler() {
      usart1_rx_buff[usart1_rx_buff_head] = USART1_RxData;
      usart1_rx_buff_head = (usart1_rx_buff_head + 1) % CFG_CORE_USART1_RX_BUFF_SIZE;
    }
  #endif // CFG_CORE_USART1_RX_MODE_IRQ


/**********************************************************************/
  #ifdef USART1_RX_DMA_CHANNEL

    #define  usart1_rx_buff_head  (CFG_CORE_USART1_RX_BUFF_SIZE - USART1_RX_DMA_CHANNEL->CNDTR)

    void USART1_Rx_begin() {
      #if defined (STM32F0XX) && defined (CFG_CORE_USART1_RX_MODE_DMA_CHANNEL5)
        BITMASK_SET(RCC->APB2ENR,  RCC_APB2Periph_SYSCFG);          /// SYSCFG clock enabled
        BITMASK_SET(SYSCFG->CFGR1, SYSCFG_DMARemap_USART1Rx);       /// DMA_USART1_Rx remap
      #endif

      /// USART1 Rx DMA config
      BITMASK_SET(USART1->CR1, USART_Mode_Rx);                      /// enable RX
      BITMASK_SET(USART1->CR3, USART_CR3_DMAR);                     /// enable DMA receiver

      /// DMA1 config
      //BITMASK_SET(RCC->AHBENR, RCC_AHBPeriph_DMA1);                 /// enable DMA1 clock
      USART1_RX_DMA_CHANNEL->CNDTR = CFG_CORE_USART1_RX_BUFF_SIZE;
      USART1_RX_DMA_CHANNEL->CPAR  = (uint32_t)(&USART1_RxData);    /// peripheral address
      USART1_RX_DMA_CHANNEL->CMAR  = (uint32_t)(&usart1_rx_buff);   /// memory address
      USART1_RX_DMA_CHANNEL->CCR   = DMA_DIR_PeripheralSRC
                                   | DMA_PeripheralInc_Disable
                                   | DMA_PeripheralDataSize_Byte
                                   | DMA_MemoryInc_Enable
                                   | DMA_MemoryDataSize_Byte
                                   | DMA_M2M_Disable
                                   | DMA_Mode_Circular
                                   | DMA_Priority_Medium
                                   | DMA_CCR_EN;                    /// enable channel
    }
  #endif // USART1_RX_DMA_CHANNEL


/**********************************************************************/
  int USART1_available() {
    return ((unsigned int)(CFG_CORE_USART1_RX_BUFF_SIZE + usart1_rx_buff_head - usart1_rx_buff_tail)) % CFG_CORE_USART1_RX_BUFF_SIZE;
  }

  int USART1_read() {
    if (usart1_rx_buff_tail == usart1_rx_buff_head)
      return -1;

    unsigned char c = usart1_rx_buff[usart1_rx_buff_tail];
    usart1_rx_buff_tail = (usart1_rx_buff_tail + 1) % CFG_CORE_USART1_RX_BUFF_SIZE;
    return c;
  }

  int USART1_peek() {
    if (usart1_rx_buff_tail == usart1_rx_buff_head)
      return -1;

    return usart1_rx_buff[usart1_rx_buff_tail];
  }

  void USART1_flushRx() {
    usart1_rx_buff_tail = usart1_rx_buff_head;
  }
#endif // USART1_RX_MODE


/**********************************************************************/
#ifdef CFG_CORE_USART1_RX_MODE_DISABLE
  void USART1_Rx_begin() {}

  int USART1_available() {
    return -1;
  }

  int USART1_read() {
    return -1;
  }

  int USART1_peek() {
    return -1;
  }

  void USART1_flushRx() {}
#endif // CFG_CORE_USART1_RX_MODE_DISABLE



/**********************************************************************
// USART1 begin
**********************************************************************/

#define USART1_DIV_SAMPLING(__BAUD__)  (((CFG_Core_ClkFreq_MHz * 1000000) + ((__BAUD__)/2U)) / (__BAUD__))

void USART1_begin(uint32_t baud) {
  /// Configure pins: Tx_A9, Rx_A10
  #ifdef STM32F10X_MD
    BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_AFIO);   /// enable AFIO clock
    BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_GPIOA);  /// enable GPIO clock
    USART1_GPIO_disable();                            /// disable TxPin

    #ifdef USART1_RX_MODE_ENABLE
      F103_GPIO_pinMode_input(GPIOA, CFG_CORE_USART1_RxPin, GPIO_Mode_IPU);
    #endif
  #endif // STM32F10X_MD

  #ifdef STM32F0XX
    //BITMASK_SET(RCC->AHBENR, RCC_AHBPeriph_GPIOA);    /// enable GPIO clock
    //BITMASK_SET(RCC->AHBENR, RCC_AHBPeriph_GPIOB);    /// enable GPIO clock
    BITMASK_SET(RCC->AHBENR, CFG_RCC_APBH);    /// enable GPIO clock  
    USART1_GPIO_disable();                            /// disable TxPin

    #ifdef USART1_RX_MODE_ENABLE
       F030_GPIO_pinAF_select (CFG_USART1_GPIO_SOURCE, CFG_CORE_USART1_RxPin, CFG_USART1_AF_SOURCE);
       F030_GPIO_pinMode_input(CFG_USART1_GPIO_SOURCE, CFG_CORE_USART1_RxPin, GPIO_Mode_AF, GPIO_PuPd_UP);
    #endif
  #endif // STM32F0XX

  /// Configure USART1
  BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_USART1);   /// enable USART1 clock

  USART1->BRR = USART1_DIV_SAMPLING(baud);
  USART1->CR3 = USART_HardwareFlowControl_None;
  USART1->CR2 = USART_StopBits_1;
  USART1->CR1 = USART_WordLength_8b
              | USART_Parity_No
              | USART_CR1_TE;                         /// enable TX

  USART1_Tx_begin();
   USART1_Rx_begin();
  BITMASK_SET(USART1->CR1, USART_CR1_UE);             /// enable USART1
}

void USART1_end() {
  USART_DeInit(USART1);
}

/**********************************************************************/

/*==============================================================================

Change log

181013 - TranDucNam
  Bug: lỗi hàm
    void USART1_write(int data) {
      while ((USART1_Status & USART_FLAG_TXE) == 0);
      USART1_TxData = data;
    }
    Không truyền được hoặc truyền sai dữ liệu.

  Cause:
    Chưa gọi USART1_GPIO_enable() trước khi ghi dữ liệu vào USART1_TxData.

  Fix:
    Không ghi trực tiếp dữ liệu vào USART1_TxData mà ghi vào bộ đệm stdout:
    printf("%c", data);


181103 - TranDucNam
  Update: thêm hàm (dành cho STM32F0XX)
    void Core_HSIConfig();
    Sử dụng HSI làm SysClock


181103 - TranDucNam
  Bug: lỗi hàm Core_HSIConfig()
    Không cấu hình được HSI làm SysClock trên KIT có hàn thạch anh

  Cause:
    Chưa disable PLL trước khi ghi đè tham số mới

  Fix:
    Disable PLL trước khi cấu hình


190219 - TranDucNam
  Update: thêm hàm
    void Core_EraseOptionBytes()
    Reset toàn bộ OptionBytes của chip STM32
    Tắt tính năng Watchdog Hardware

==============================================================================*/

#endif  // CFG_CORE
