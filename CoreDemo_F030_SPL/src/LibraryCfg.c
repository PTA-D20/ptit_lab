
/**********************************************************************/
///                           CoreCfg                                ///
/**********************************************************************/


/**********************************************************************/
/// Select Options

//#define  CFG_CORE_CORETICK_SOURCE_DWT              /// not available on F0XX
//#define  CFG_CORE_CORETICK_SOURCE_SYSTICK_IRQ
  #define  CFG_CORE_CORETICK_SOURCE_SYSTICK

//#define  CFG_CORE_WAIT_MODE_CLASSIC                /// using classic method
  #define  CFG_CORE_WAIT_MODE_EVENTS                 /// using __WFE() and DMA


/**********************************************************************/
/// Clock and Flash latency settings

#ifdef STM32F10X_MD
  #define  CFG_Core_ClkFreq_MHz    (72UL)
  #define  CFG_Core_Flash_Latency  FLASH_Latency_1
#endif

#ifdef STM32F0XX
  #define  CFG_Core_ClkFreq_MHz    (48UL)
  #define  CFG_Core_Flash_Latency  FLASH_Latency_1
#endif


/**********************************************************************/
/// USART1

#ifdef STM32F10X_MD
 #define  CFG_CORE_USART1_TxPin   9
 #define  CFG_CORE_USART1_RxPin   10

//#define  CFG_CORE_USART1_TxPin   2
//#define  CFG_CORE_USART1_RxPin   3
#endif

#ifdef STM32F0XX
/// PortA_USART1
// #define  CFG_CORE_USART1_TxPin   9
// #define  CFG_CORE_USART1_RxPin   10
//#define  CFG_CORE_USART1_TxPin   2
//#define  CFG_CORE_USART1_RxPin   3
//#define CFG_USART1_GPIO_SOURCE   GPIOA
//#define CFG_USART1_AF_SOURCE     GPIO_AF_1
//#define CFG_RCC_APBH             RCC_AHBPeriph_GPIOA

/// PortB_USART1
#define  CFG_CORE_USART1_TxPin   6
#define  CFG_CORE_USART1_RxPin   7
#define  CFG_USART1_GPIO_SOURCE  GPIOB
#define  CFG_USART1_AF_SOURCE    GPIO_AF_0
#define  CFG_RCC_APBH            RCC_AHBPeriph_GPIOB

#endif



#define  CFG_CORE_USART1_RX_BUFF_SIZE  32

#ifdef CFG_CORE_WAIT_MODE_EVENTS
  //#define  CFG_CORE_USART1_TX_MODE_DMA_CHANNEL2    /// not available on F10X_MD
    #define  CFG_CORE_USART1_TX_MODE_DMA_CHANNEL4

  //#define  CFG_CORE_USART1_RX_MODE_DISABLE
  //#define  CFG_CORE_USART1_RX_MODE_DMA_CHANNEL3    /// not available on F10X_MD
    #define  CFG_CORE_USART1_RX_MODE_DMA_CHANNEL5

  //#define  CFG_CORE_DMA_BACKUP
#endif

#ifdef CFG_CORE_WAIT_MODE_CLASSIC
//#define  CFG_CORE_USART1_RX_MODE_DISABLE
  #define  CFG_CORE_USART1_RX_MODE_IRQ
#endif


/**********************************************************************/
#define  CFG_CORE
#include "Core.c.h"

/**********************************************************************/
