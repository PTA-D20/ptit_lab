
#include <stdio.h>
#include <math.h>
#include "Core.h"
#include "ADC.h"

/**********************************************************************/

#define LED_1_PIN         1
#define LED_1_PORT        GPIOA
#define LED_PIN_1_SPEED   GPIO_Speed_2MHz

#define LED_2_PIN         2
#define LED_2_PORT        GPIOA
#define LED_PIN_2_SPEED   GPIO_Speed_2MHz

#define LED_3_PIN         3
#define LED_3_PORT        GPIOA
#define LED_PIN_3_SPEED   GPIO_Speed_2MHz

#define LED_LOW(LED_PORT,LED_PIN)       LED_PORT->BRR  = _BV(LED_PIN)
#define LED_HIGH(LED_PORT,LED_PIN)      LED_PORT->BSRR = _BV(LED_PIN)

#define length_           200

/**********************************************************************/
  uint16_t adc_arr[length_]; /// chứa giá trị đo của triết áp
  uint32_t time_start_point[length_]; /// thoi gian bat dau vong lap
  uint32_t adc_start_point[length_]; /// thoi gian truoc luc goi ADC1_read16()
  uint32_t time_end_point[length_]; /// thoi gian ket thuc vog lap

void GPIO_begin()
{
  // enable GPIO clock
  BITMASK_SET(RCC->AHBENR, RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB);
  /// Cấp clock cho port A và port B của kit

  F030_GPIO_pinMode_output(LED_1_PORT, LED_1_PIN, GPIO_Mode_OUT, GPIO_OType_PP, LED_PIN_1_SPEED);
  F030_GPIO_pinMode_output(LED_2_PORT, LED_2_PIN, GPIO_Mode_OUT, GPIO_OType_PP, LED_PIN_2_SPEED);
  F030_GPIO_pinMode_output(LED_3_PORT, LED_3_PIN, GPIO_Mode_OUT, GPIO_OType_PP, LED_PIN_3_SPEED);
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
  uint16_t adc_read_time_avg = 0;
  ADC1_begin();
  /// Khởi động, cấu hình cho ADC, chi tiết xem tại ADC.c

        LED_HIGH(LED_2_PORT,LED_2_PIN); /// bật led 2 => báo hiệu mạch đang ở trạng thái đọc triết áp

        for(int i=0;i<length_;i++){
            /// bat dau vong

            time_start_point[i] = micros(); /// ghi lại thời gian bat dau vong lap

            delay_until_us(time_start_point[i] + 200); /// tạo độ trễ giữa các lần lấy mẫu ( thử với 1000,500,200 us)

            adc_start_point[i] = micros();
            /// bắt đầu chuyển đổi (conversion)
            adc_arr[i] = ADC1_read16(ADC_Channel_6, ADC_SampleTime_1_5Cycles);
            /// kết thúc chuyển đổi

          /// ket thuc vong
            time_end_point[i] = micros();
            /// ghi lại thời gian kết thúc

        }
        LED_LOW(LED_2_PORT,LED_2_PIN); /// tắt led 2 => báo hiệu mạch kết thúc trạng thái đọc triết áp


        /************************************/
        /// phần in kết quả ra.
        LED_HIGH(LED_3_PORT,LED_3_PIN);
        uint32_t delay_target = millis();
        delay_until_ms(delay_target+=2000); /// trễ 1000ms
        LED_LOW(LED_3_PORT,LED_3_PIN);

        printf(" //////////////////////////BAT DAU IN//////////////////////////////////////////\n");
        LED_HIGH(LED_1_PORT,LED_1_PIN); /// bật led 1 => báo hiệu mạch đang ở trạng thái in kết quả

        for(int i=0;i<length_;i++){
//          printf("bat dau: %12u  ",time_start_point[i]);
//          /// thoi diem bat dau chu ky
//
//          printf("adc read: %12u  ",adc_start_point[i]);
//          /// thoi diem bat dau goi ham read16
//
//          printf("ket thuc: %12u  ",time_end_point[i]);
//          /// thoi diem ket thuc chu ky

          printf("time_gap: %12u  ",time_end_point[i] - time_start_point[i]);
          /// in khoảng thời gian trong một chu kỳ đọc triết áp

          printf("time_delay: %12u   ", adc_start_point[i] - time_start_point[i]);
          /// in khoang thoi gian cua ham delay tao ra

          uint8_t adc_read_time = time_end_point[i] - adc_start_point[i];
          adc_read_time_avg+= adc_read_time;
          //printf("time_ADC_read: %12u   ",time_end_point[i] - adc_start_point[i] );
          printf("time_ADC_read: %12u   ",adc_read_time);

          printf("gia tri do: %u\n",adc_arr[i]);
          /// in giá trị triết áp đọc được

          delay_until_ms(delay_target+=300); /// tạo độ trễ cho quá trình in kết quả là 300ms
        }

        printf(" /////////////////////////////////////////KET THUC///////////////////////////////////////////////////\n");
        printf("tong adc_time: %6u\n",adc_read_time_avg);
        LED_LOW(LED_1_PORT,LED_1_PIN); /// led 1 tắt => báo hiệu mạch kết thúc trạng thái in kết quả.
}

/**********************************************************************/


