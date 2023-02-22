#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "basic.h"

C_DECLS_BEGIN

/**********************************************************************/
#define DISCHARGE_CHANNEL  ADC_Channel_0
#define DISCHARGE_PORT     GPIOA
#define DISCHARGE_PIN      0

#define TOUCH_PIN_A0    0
#define TOUCH_PIN_A1    1
#define TOUCH_PIN_A2    2
#define TOUCH_PIN_A3    3
#define TOUCH_PIN_A4    4
#define TOUCH_PIN_A5    5
#define TOUCH_PIN_A6    6
#define TOUCH_PIN_A7    7
#define TOUCH_PIN_B1    8

/**********************************************************************/
void Touch_begin(uint32_t mask);
void Touch_calibrate();

void Touch_set_threshold(int touch_pin, int threshold);
int  Touch_get_base_line(int touch_pin);

int  Touch_read(int touch_pin);
int  Touch_read_state(int touch_pin);

/**********************************************************************/
C_DECLS_END
#endif // __TOUCH_H__
