
#include <stdio.h>
#include "Core.h"
#include "ADC.h"

/**********************************************************************/

#define LED_1_PIN         1
#define LED_1_PORT        GPIOA
#define LED_PIN_1_SPEED   GPIO_Speed_2MHz

#define LED_2_PIN         2
#define LED_2_PORT        GPIOA
#define LED_PIN_2_SPEED   GPIO_Speed_2MHz

#define LED_LOW(LED_PORT,LED_PIN)       LED_PORT->BRR  = _BV(LED_PIN)
#define LED_HIGH(LED_PORT,LED_PIN)      LED_PORT->BSRR = _BV(LED_PIN)

#define length_           400

/**********************************************************************/
void GPIO_begin()
{
  // enable GPIO clock
  BITMASK_SET(RCC->AHBENR, RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB);
  /// Cấp clock cho port A và port B của kit

  F030_GPIO_pinMode_output(LED_1_PORT, LED_1_PIN, GPIO_Mode_OUT, GPIO_OType_PP, LED_PIN_1_SPEED);
  F030_GPIO_pinMode_output(LED_1_PORT, LED_2_PIN, GPIO_Mode_OUT, GPIO_OType_PP, LED_PIN_2_SPEED);
  /// Cấu hình cho chân led pin ở PA1 là chế dộ output_push-pull
}


/**********************************************************************/
// main
/**********************************************************************/
int main(void)
{
  /// Setup:
  Core_begin();

  USART1_begin(BAUD_1M);/// (3M,2M,1.5M,1M,115200,9600)
  /// cấu hình giao tiếp usart tại A9 và A10(xem chi tiết cấu hình ở LibraryCfg.c), với baud rate = 1000000.

  Core_PrintInfo();
  /// In thông cấu hình flash lên serial monitor

  GPIO_begin();

  F030_GPIO_pinMode_input(GPIOA, 6, GPIO_Mode_AN, GPIO_PuPd_NOPULL);
  /// Cấu hình cho chân PA6 là chế độ vào Analog, không có điện trở kéo
  /// Tham khảo thư viện stm32f0xx_gpio.h và Core.h

  ADC1_begin();
  /// Khởi động, cấu hình cho ADC, chi tiết xem tại ADC.c


  uint32_t current_time;
  uint16_t adc_arr[length_];
  uint32_t time_gap[length_];
  uint32_t start_point;
  uint32_t end_point;
  uint32_t avg_e= 0;
  uint32_t avg_value = 0;

        LED_HIGH(LED_2_PORT,LED_2_PIN); /// bật led 2 => báo hiệu mạch đang ở trạng thái đọc triết áp

        for(int i=0;i<length_;i++){
            /// bat dau vong

            start_point = micros(); /// ghi lại thời gian giữa các lần lấy mẫu

            /// bắt đầu chuyển đổi (conversion)
          adc_arr[i] = ADC1_read16(ADC_Channel_6, ADC_SampleTime_239_5Cycles);
            /// kết thúc chuyển đổi


          delay_until_ms(100*i); /// tạo độ trễ giữa các lần lấy mẫu ( thử với 100,50,30,10 ms)
          //delay_until_us(75*i);

          /// ket thuc vong
            end_point = micros();
            /// ghi lại thời gian kết thúc

            time_gap[i] = end_point - start_point;
            /// lưu lại khoảng thời gian từ khi tổng thời gian trong một chu kỳ đọc triết áp
        }
        LED_LOW(LED_2_PORT,LED_2_PIN); /// tắt led 2 => báo hiệu mạch kết thúc trạng thái đọc triết áp


        /************************************/
        /// phần in kết quả ra.
        delay_ms(1000); /// trễ 1000ms

        printf(" bat dau in\n");
        LED_HIGH(LED_1_PORT,LED_1_PIN); /// bật led 1 => báo hiệu mạch đang ở trạng thái in kết quả

        for(int i=0;i<length_;i++){

          printf("time_gap: %12u       ",time_gap[i]);
          /// in khoảng thời gian trong một chu kỳ đọc triết áp

          printf("gia tri do: %u         ",adc_arr[i]);
          /// in giá trị triết áp đọc được


          uint16_t value_gap = abs(adc_arr[i] - adc_arr[i-1]);
          /// tính chênh lệch giữa giá trị của triết áp hiện tại với giá trị trước đó của nó

          if(i>=1)printf("value_gap: %u\n", value_gap);
          /// in ra độ chênh lệch giá trị của triết áp với lần đo trước đó

          avg_value += adc_arr[i];
          /// cộng dồn giá trị của biến trở

          avg_e += value_gap;
          /// cộng dồn độ chênh lệch

          delay_until_ms(300*i); /// tạo độ trễ cho quá trình in kết quả là 300ms
        }

        printf(" /////////////////////////////////////////KET THUC///////////////////////////////////////////////////\n");

        LED_LOW(LED_1_PORT,LED_1_PIN); /// led 1 tắt => báo hiệu mạch kết thúc trạng thái in kết quả.

        printf("gia tri trung binh cua %u lan lay mau la: %u\n",length_,avg_value/length_);
        /// in ra giá trị trung bình của độ chênh lệch giữa các lần đo

        printf("gia tri trung binh cua do chenh lech giua cac lan do: %u\n",avg_e/length_);
        /// in ra giá trị trung bình của độ chênh lệch giữa các lần đo
}

/**********************************************************************/


