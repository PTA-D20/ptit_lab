
#include "Core.h"
#include <stdio.h>

/**********************************************************************/
#define LED_PIN         4
#define LED_PORT        GPIOA
#define LED_PIN_SPEED   GPIO_Speed_2MHz

#define LED_LOW()       LED_PORT->BRR  = _BV(LED_PIN)
#define LED_HIGH()      LED_PORT->BSRR = _BV(LED_PIN)
#define LED_TOGGLE()    BITMASK_TGL(LED_PORT->ODR, _BV(LED_PIN))

/**********************************************************************/
void GPIO_begin() {
  // enable GPIO clock
  BITMASK_SET(RCC->AHBENR, RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB);

  F030_GPIO_pinMode_output(LED_PORT, LED_PIN, GPIO_Mode_OUT, GPIO_OType_PP, LED_PIN_SPEED);
}


/**********************************************************************/
// main
/**********************************************************************/
int main(void) {
  /// Setup
  Core_HSIConfig();
//Core_EraseOptionBytes();

  Core_begin();
  GPIO_begin();

  USART1_begin(BAUD_1M);
  /// Cấu hình và khởi động USART1, chi tiết cấu hình xem trong file LibraryCfg.c
  Core_PrintInfo();
  /// in thông tin về cấu hình flash ra serial monitor

  /// Loop
  uint32_t delay_target = 0;
  while (1) {
    printf("%12u\t", micros());
    /// in thời gian ở micro giây ra màn hình
    printf("%9u\t", millis());
    /// in thời gian ở milli giây ra màn hình
    printf("\n");

    LED_TOGGLE();
    delay_until_ms(delay_target += 100);
    LED_TOGGLE();
    /// đảo trạng thái đèn led sau mỗi 100ms.

    if (USART1_available() > 0) {
      while (USART1_available() > 0) {
        printf("%c", USART1_read());/// in kí tự được gửi đến từ serial monitor
     }
     /// USART1_available() : Chức năng giống như hàm Serial.available của arduino
      /// Hàm này trả về số byte dữ liệu có trong bộ đệm. Nó thường được dùng để xét xem đã có dữ liệu chưa, hoặc lượng dữ liệu cần nhận đã đủ chưa
     /// USART1_read() : hàm đọc kí tự (kiểu char) mà USART1 nhận được (receive)
      printf("\n");
      LED_TOGGLE();
    }
    /// cả đoạn chương trình trên nhằm giúp in ra màn hình monitor những gì được nhập vào ở serial monitor

    delay_ms(1000);
    delay_until_ms(delay_target += 900);
    /// chờ 1900ms

    LED_TOGGLE();
    delay(50);
  }
}

/**********************************************************************/
