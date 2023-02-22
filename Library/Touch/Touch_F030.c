#include "Touch_F030.h"

/**********************************************************************/
#define TOUCH_BASELINE_N         (128L)

//#define TOUCH_AVERAGE            (4 / 4)
#define TOUCH_AVERAGE            (128 / 4)

#define TOUCH_THRESHOLD_DEFAULT  (700)
#define TOUCH_SKIP               (64)
#define TOUCH_NPIN               (16)

#define TOUCH_ADAPTIVE_THRESHOLD  (100)
#define TOUCH_ADAPTIVE_RATE       (32)
#define TOUCH_OUTPP_SPEED         GPIO_Speed_Level_1

/**********************************************************************/
int touch_base_line[TOUCH_NPIN];
int touch_threshold[TOUCH_NPIN];
uint32_t touch_mask = 0;


/**********************************************************************/
int Touch_is_used(int touch_pin) {
  return (touch_mask & _BV(touch_pin)) ? 1 : 0;
}

/**********************************************************************/
__attribute__( ( always_inline ) ) static __INLINE uint32_t Touch_A0_read_core()
{
  /// B1: discharge the sampling cap and pullup to charge the External Cap
  ADC1->CHSELR = DISCHARGE_CHANNEL;
  ADC1->CR = ADC_CR_ADSTART; // Start conversion
  // (treo ADC_FLAG_EOSMP)
//  ADC1->ISR = ADC_FLAG_EOSMP;                 // clear flag
//  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  __NOP();__NOP();__NOP();__NOP();
  ADC1->CR = ADC_CR_ADSTP;  // stop conversion
  ADC1->ISR = ADC_FLAG_EOSMP;  // clear flag
  ADC1->CHSELR = ADC_Channel_0;
  uint32_t gpio_mode_outpp = GPIOA->MODER;

  /// B2: Select channel touch and read adc value
  while ((ADC1->CR & ADC_CR_ADSTP) != 0);
  GPIOA->MODER = gpio_mode_outpp ^ ((GPIO_Mode_AN ^ GPIO_Mode_OUT) << (2*0));  // Analog input
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  GPIOA->MODER = gpio_mode_outpp;  // Output Push_Pull
  while ((ADC1->ISR & ADC_FLAG_EOC) == 0);  // Wait end of conversion
  return ADC1->DR;  // Read data conversion
}

int __attribute__((aligned(16))) Touch_A0_read_raw()
{
  int value = 0;
  for (int i = (volatile int)TOUCH_AVERAGE; i > 0; i--) {
    value += Touch_A0_read_core();
    value += Touch_A0_read_core();
    value += Touch_A0_read_core();
    value += Touch_A0_read_core();
  }
  return value;
}


/**********************************************************************/
__attribute__( ( always_inline ) ) static __INLINE uint32_t Touch_A1_read_core()
{
  /// B1: discharge the sampling cap and pullup to charge the External Cap
  ADC1->CHSELR = DISCHARGE_CHANNEL;
  ADC1->CR = ADC_CR_ADSTART; // Start conversion
  // (treo ADC_FLAG_EOSMP)
//  ADC1->ISR = ADC_FLAG_EOSMP;                 // clear flag
//  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  ADC1->CR = ADC_CR_ADSTP;  // stop conversion
  ADC1->ISR = ADC_FLAG_EOSMP;  // clear flag
  ADC1->CHSELR = ADC_Channel_1;
  uint32_t gpio_mode_outpp = GPIOA->MODER;

  /// B2: Select channel touch and read adc value
  while ((ADC1->CR & ADC_CR_ADSTP) != 0);
  GPIOA->MODER = gpio_mode_outpp ^ ((GPIO_Mode_AN ^ GPIO_Mode_OUT) << (2*1));  // Analog input
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  GPIOA->MODER = gpio_mode_outpp;  // Output Push_Pull
  while ((ADC1->ISR & ADC_FLAG_EOC) == 0);  // Wait end of conversion
  return ADC1->DR;  // Read data conversion
}

int __attribute__((aligned(16))) Touch_A1_read_raw()
{
  int value = 0;
  for (int i = (volatile int)TOUCH_AVERAGE; i > 0; i--)
  {
    value += Touch_A1_read_core();
    value += Touch_A1_read_core();
    value += Touch_A1_read_core();
    value += Touch_A1_read_core();
  }
  return value;
}

/**********************************************************************/
__attribute__( ( always_inline ) ) static __INLINE uint32_t Touch_A2_read_core()
{
  /// B1: discharge the sampling cap and pullup to charge the External Cap
  ADC1->CHSELR = DISCHARGE_CHANNEL;
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  // (treo ADC_FLAG_EOSMP)
//  ADC1->ISR = ADC_FLAG_EOSMP;                 // clear flag
//  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  ADC1->CR = ADC_CR_ADSTP;                    // stop conversion
  ADC1->ISR = ADC_FLAG_EOSMP;  // clear flag
  ADC1->CHSELR = ADC_Channel_2;
  uint32_t gpio_mode_outpp = GPIOA->MODER;

  /// B2: Select channel touch and read adc value
  while ((ADC1->CR & ADC_CR_ADSTP) != 0);  // stop ongoing
  GPIOA->MODER = gpio_mode_outpp ^ ((GPIO_Mode_AN ^ GPIO_Mode_OUT) << (2*2));  // Analog input
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  GPIOA->MODER = gpio_mode_outpp;  // Output Push_Pull
  while ((ADC1->ISR & ADC_FLAG_EOC) == 0);  // Wait end of conversion
  return ADC1->DR;  // Read data conversion
}

int __attribute__((aligned(16))) Touch_A2_read_raw()
{
  int value = 0;
  for (int i = (volatile int)TOUCH_AVERAGE; i > 0; i--)
  {
    value += Touch_A2_read_core();
    value += Touch_A2_read_core();
    value += Touch_A2_read_core();
    value += Touch_A2_read_core();
  }
  return value;
}

/**********************************************************************/
__attribute__( ( always_inline ) ) static __INLINE uint32_t Touch_A3_read_core()
{
  /// B1: discharge the sampling cap and pullup to charge the External Cap
  ADC1->CHSELR = DISCHARGE_CHANNEL;
  ADC1->CR = ADC_CR_ADSTART; // Start conversion
  // (treo ADC_FLAG_EOSMP)
//  ADC1->ISR = ADC_FLAG_EOSMP;                 // clear flag
//  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  ADC1->CR = ADC_CR_ADSTP;  // stop conversion
  ADC1->ISR = ADC_FLAG_EOSMP;  // clear flag
  ADC1->CHSELR = ADC_Channel_3;
  uint32_t gpio_mode_outpp = GPIOA->MODER;

  /// B2: Select channel touch and read adc value
  while ((ADC1->CR & ADC_CR_ADSTP) != 0);
  GPIOA->MODER = gpio_mode_outpp ^ ((GPIO_Mode_AN ^ GPIO_Mode_OUT) << (2*3));  // Analog input
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  GPIOA->MODER = gpio_mode_outpp;  // Output Push_Pull
  while ((ADC1->ISR & ADC_FLAG_EOC) == 0);  // Wait end of conversion
  return ADC1->DR;  // Read data conversion
}

int __attribute__((aligned(16))) Touch_A3_read_raw()
{
  int value = 0;
  for (int i = (volatile int)TOUCH_AVERAGE; i > 0; i--)
  {
    value += Touch_A3_read_core();
    value += Touch_A3_read_core();
    value += Touch_A3_read_core();
    value += Touch_A3_read_core();
  }
  return value;
}

/**********************************************************************/
__attribute__( ( always_inline ) ) static __INLINE uint32_t Touch_A4_read_core()
{
  /// B1: discharge the sampling cap and pullup to charge the External Cap
  ADC1->CHSELR = DISCHARGE_CHANNEL;
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  // (treo ADC_FLAG_EOSMP)
//  ADC1->ISR = ADC_FLAG_EOSMP;                 // clear flag
//  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  __NOP();__NOP();__NOP();__NOP();
  ADC1->CR = ADC_CR_ADSTP;                    // stop conversion
  ADC1->ISR = ADC_FLAG_EOSMP;  // clear flag
  ADC1->CHSELR = ADC_Channel_4;
  uint32_t gpio_mode_outpp = GPIOA->MODER;

  /// B2: Select channel touch and read adc value
  while ((ADC1->CR & ADC_CR_ADSTP) != 0);  // stop ongoing
  GPIOA->MODER = gpio_mode_outpp ^ ((GPIO_Mode_AN ^ GPIO_Mode_OUT) << (2*4));
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  GPIOA->MODER = gpio_mode_outpp;  // Output Push_Pull
  while ((ADC1->ISR & ADC_FLAG_EOC) == 0);  // Wait end of conversion
  return ADC1->DR;  // Read data conversion
}

int __attribute__((aligned(16))) Touch_A4_read_raw()
{
  int value = 0;
  for (int i = (volatile int)TOUCH_AVERAGE; i > 0; i--)
  {
    value += Touch_A4_read_core();
    value += Touch_A4_read_core();
    value += Touch_A4_read_core();
    value += Touch_A4_read_core();
  }
  return value;
}


/**********************************************************************/
__attribute__( ( always_inline ) ) static __INLINE uint32_t Touch_A5_read_core()
{
  /// B1: discharge the sampling cap and pullup to charge the External Cap
  ADC1->CHSELR = DISCHARGE_CHANNEL;
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  // (treo ADC_FLAG_EOSMP)
//  ADC1->ISR = ADC_FLAG_EOSMP;                 // clear flag
//  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  __NOP();__NOP();__NOP();__NOP();
  ADC1->CR = ADC_CR_ADSTP;                    // stop conversion
  ADC1->ISR = ADC_FLAG_EOSMP;  // clear flag
  ADC1->CHSELR = ADC_Channel_5;
  uint32_t gpio_mode_outpp = GPIOA->MODER;

  /// B2: Select channel touch and read adc value
  while ((ADC1->CR & ADC_CR_ADSTP) != 0);  // stop ongoing
  GPIOA->MODER = gpio_mode_outpp ^ ((GPIO_Mode_AN ^ GPIO_Mode_OUT) << (2*5));
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  GPIOA->MODER = gpio_mode_outpp;  // Output Push_Pull
  while ((ADC1->ISR & ADC_FLAG_EOC) == 0);  // Wait end of conversion
  return ADC1->DR;  // Read data conversion
}

int __attribute__((aligned(16))) Touch_A5_read_raw()
{
  int value = 0;
  for (int i = (volatile int)TOUCH_AVERAGE; i > 0; i--)
  {
    value += Touch_A5_read_core();
    value += Touch_A5_read_core();
    value += Touch_A5_read_core();
    value += Touch_A5_read_core();
  }
  return value;
}


/**********************************************************************/
__attribute__( ( always_inline ) ) static __INLINE uint32_t Touch_A6_read_core()
{
  /// B1: discharge the sampling cap and pullup to charge the External Cap
  ADC1->CHSELR = DISCHARGE_CHANNEL;
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  // (treo ADC_FLAG_EOSMP)
//  ADC1->ISR = ADC_FLAG_EOSMP;                 // clear flag
//  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  __NOP();__NOP();__NOP();__NOP();
  ADC1->CR = ADC_CR_ADSTP;                    // stop conversion
  ADC1->ISR = ADC_FLAG_EOSMP;  // clear flag
  ADC1->CHSELR = ADC_Channel_6;
  uint32_t gpio_mode_outpp = GPIOA->MODER;

  /// B2: Select channel touch and read adc value
  while ((ADC1->CR & ADC_CR_ADSTP) != 0);  // stop ongoing
  GPIOA->MODER = gpio_mode_outpp ^ ((GPIO_Mode_AN ^ GPIO_Mode_OUT) << (2*6));
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  GPIOA->MODER = gpio_mode_outpp;  // Output Push_Pull
  while ((ADC1->ISR & ADC_FLAG_EOC) == 0);  // Wait end of conversion
  return ADC1->DR;  // Read data conversion
}

int __attribute__((aligned(16))) Touch_A6_read_raw()
{
  int value = 0;
  for (int i = (volatile int)TOUCH_AVERAGE; i > 0; i--)
  {
    value += Touch_A6_read_core();
    value += Touch_A6_read_core();
    value += Touch_A6_read_core();
    value += Touch_A6_read_core();
  }
  return value;
}

/**********************************************************************/
__attribute__( ( always_inline ) ) static __INLINE uint32_t Touch_A7_read_core()
{
  /// B1: discharge the sampling cap and pullup to charge the External Cap
  ADC1->CHSELR = DISCHARGE_CHANNEL;
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  // (treo ADC_FLAG_EOSMP)
//  ADC1->ISR = ADC_FLAG_EOSMP;                 // clear flag
//  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  __NOP();__NOP();__NOP();__NOP();
  ADC1->CR = ADC_CR_ADSTP;                    // stop conversion
  ADC1->ISR = ADC_FLAG_EOSMP;  // clear flag
  ADC1->CHSELR = ADC_Channel_7;
  uint32_t gpio_mode_outpp = GPIOA->MODER;

  /// B2: Select channel touch and read adc value
  while ((ADC1->CR & ADC_CR_ADSTP) != 0);  // stop ongoing
  GPIOA->MODER = gpio_mode_outpp ^ ((GPIO_Mode_AN ^ GPIO_Mode_OUT) << (2*7));
  ADC1->CR = ADC_CR_ADSTART;  // Start conversion
  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);  // End of sampling phase reached
  GPIOA->MODER = gpio_mode_outpp;  // Output Push_Pull
  while ((ADC1->ISR & ADC_FLAG_EOC) == 0);  // Wait end of conversion
  return ADC1->DR;  // Read data conversion
}

int __attribute__((aligned(16))) Touch_A7_read_raw()
{
  int value = 0;
  for (int i = (volatile int)TOUCH_AVERAGE; i > 0; i--)
  {
    value += Touch_A7_read_core();
    value += Touch_A7_read_core();
    value += Touch_A7_read_core();
    value += Touch_A7_read_core();
  }
  return value;
}

/**********************************************************************/
__attribute__( ( always_inline ) ) static __INLINE uint32_t Touch_B1_read_core()
{
  /// B1: discharge the sampling cap and charge the external cap
  ADC1->CHSELR = DISCHARGE_CHANNEL;
  ADC1->CR = ADC_CR_ADSTART;                   // start conversion
  // (treo ADC_FLAG_EOSMP)
  //ADC1->ISR = ADC_FLAG_EOSMP;                // clear flag
  //while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0); // End of sampling phase reached
  __NOP();__NOP();__NOP();__NOP();
  ADC1->CR  = ADC_CR_ADSTP;                    // stop conversion
  ADC1->ISR = ADC_FLAG_EOSMP;                  // clear flag
  ADC1->CHSELR = ADC_Channel_9;                // channel touch
  uint32_t gpio_mode_outpp = GPIOB->MODER;
  while ((ADC1->CR & ADC_CR_ADSTP) != 0);      // wait stop ongoing

  /// B2: select channel touch and read adc value
  GPIOB->MODER = gpio_mode_outpp ^ ((GPIO_Mode_AN ^ GPIO_Mode_OUT) << (2*1));  // analog input
  ADC1->CR = ADC_CR_ADSTART;                   // start conversion
  while ((ADC1->ISR & ADC_FLAG_EOSMP) == 0);   // End of sampling phase reached
  GPIOB->MODER = gpio_mode_outpp;              // Output Push_Pull
  while ((ADC1->ISR & ADC_FLAG_EOC) == 0);     // Wait end of conversion
  return ADC1->DR;                             // Read data conversion
}

int __attribute__((aligned(16))) Touch_B1_read_raw()
{
  int value = 0;
  for (int i = (volatile int)TOUCH_AVERAGE; i > 0; i--)
  {
    value += Touch_B1_read_core();
    value += Touch_B1_read_core();
    value += Touch_B1_read_core();
    value += Touch_B1_read_core();
  }
  return value;
}


/**********************************************************************/
int Touch_dummy() {
  return 0;
}

int (*Touch_read_raw[TOUCH_NPIN])() =
{
  Touch_A0_read_raw,
  Touch_A1_read_raw,
  Touch_A2_read_raw,
  Touch_A3_read_raw,
  Touch_A4_read_raw,
  Touch_A5_read_raw,
  Touch_A6_read_raw,
  Touch_A7_read_raw,
  Touch_B1_read_raw,
  Touch_dummy,
  Touch_dummy,
  Touch_dummy,
  Touch_dummy,
  Touch_dummy,
  Touch_dummy,
  Touch_dummy,
};


/**********************************************************************/
int Touch_read(int touch_pin)
{
  touch_pin &= 0x0F;
  int read = ((*Touch_read_raw[touch_pin])() - touch_base_line[touch_pin]) / TOUCH_AVERAGE;
//  printf("%d\n", read);
  if (read < TOUCH_ADAPTIVE_THRESHOLD)
    touch_base_line[touch_pin] += read / ((read < 0) ? (TOUCH_ADAPTIVE_RATE) : (TOUCH_ADAPTIVE_RATE * 2));

  return read;
}

/**********************************************************************/
int Touch_read_state(int touch_pin)
{
  touch_pin &= 0x0F;
  return (Touch_read(touch_pin) > touch_threshold[touch_pin]) ? 1 : 0;
}

/**********************************************************************/
void Touch_calibrate()
{
  // 1. Skip
  for (int touch_pin = 0; touch_pin < TOUCH_NPIN; touch_pin++)
  {
    for (int i = 0; i < TOUCH_SKIP; i++)
      (*Touch_read_raw[touch_pin])();
  }

  // 2. Read baseline
  for (int touch_pin = 0; touch_pin < TOUCH_NPIN; touch_pin++)
  {
    uint64_t sum_baseline = TOUCH_BASELINE_N / 2;
    for (int i = 0; i < TOUCH_BASELINE_N; i++)
      sum_baseline += (*Touch_read_raw[touch_pin])();

    touch_base_line[touch_pin] = sum_baseline / TOUCH_BASELINE_N;
  }
}

/**********************************************************************/
int Touch_get_base_line(int touch_pin)
{
  return touch_base_line[touch_pin & 0x0F];
}

/**********************************************************************/
void Touch_set_threshhold(int touch_pin, int threshold)
{
  touch_threshold[touch_pin & 0x0F] = threshold;
}

/**********************************************************************/
void Touch_begin(uint32_t mask)
{
  touch_mask = mask & (_BV(TOUCH_NPIN) - 1);

  BITMASK_SET(RCC->AHBENR, RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB);  // enable clock GPIO

  /// discharge channel: Analog input
  BITPOS_SET_X2(DISCHARGE_PORT->MODER, DISCHARGE_PIN, GPIO_Mode_AN);

  /// touch channel: PA.0 - PA.7
  for (int touch_pin = TOUCH_PIN_A0; touch_pin <= TOUCH_PIN_A7; touch_pin++)
  {
    if (Touch_is_used(touch_pin))
    {
//      printf("%d\n", touch_pin);
      BITPOS_SET_X2(GPIOA->OSPEEDR, touch_pin, TOUCH_OUTPP_SPEED);  // speed
      BITPOS_SET_X2(GPIOA->MODER,   touch_pin,  GPIO_Mode_OUT   );  // Output Push_Pull
      GPIOA->ODR |= _BV(touch_pin);                                // set HIGH level (when output mode)
      Touch_set_threshhold(touch_pin, TOUCH_THRESHOLD_DEFAULT);
    }
    else
      Touch_read_raw[touch_pin] = &Touch_dummy;
  }

  /// touch channel: PB.1
  if (Touch_is_used(TOUCH_PIN_B1))
  {
    BITPOS_SET_X2(GPIOB->OSPEEDR, 1, TOUCH_OUTPP_SPEED);  // speed
    BITPOS_SET_X2(GPIOB->MODER,   1, GPIO_Mode_OUT    );  // Output Push_Pull
    GPIOB->ODR |= _BV(1);                                // set HIGH level (when output mode)
    Touch_set_threshhold(TOUCH_PIN_B1, TOUCH_THRESHOLD_DEFAULT);
  }
  else
    Touch_read_raw[TOUCH_PIN_B1] = &Touch_dummy;


  /// Initialize ADC1
  BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_ADC1);  // ADC1 Periph clock enable
  ADC_DeInit(ADC1);
  ADC1->CFGR1 = ADC_Resolution_12b
              | ADC_DataAlign_Right
              | ADC_ExternalTrigConvEdge_None
              | ADC_ScanDirection_Upward;
  ADC1->CFGR2  = ADC_ClockMode_SynClkDiv4;  // ADC clock = PCLK / 4

  ADC1->SMPR = ADC_SampleTime_1_5Cycles;  // sampling time for all channels4

  ADC1->CR = ADC_CR_ADCAL;          // ADC Calibration
  while (ADC1->CR & ADC_CR_ADCAL);  // wait calibration complete

  ADC1->CR = ADC_CR_ADEN;                     // enable ADC1
  while ((ADC1->ISR & ADC_FLAG_ADRDY) == 0);  // wait ADC1 ready

  Touch_calibrate();
}

/**********************************************************************/
