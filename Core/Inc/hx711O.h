/*
 * HX711.h
 *
 *  Created on: 16 nov. 2021
 *      Author: PCov3r
 */

#ifndef APPLICATION_CORE_HX711O_H_
#define APPLICATION_CORE_HX711O_H_

#include "stdint.h"
#include "stdbool.h"
#include "main.h"

#define CHANNEL_A 0
#define CHANNEL_B 1
#define interrupts() __enable_irq()
#define noInterrupts() __disable_irq()

typedef struct
{
  GPIO_TypeDef  *clk_gpio;
  GPIO_TypeDef  *dat_gpio;
  uint16_t      clk_pin;
  uint16_t      dat_pin;
  long       	Aoffset;
  float         Ascale;
  uint8_t		Again;
  long       	Boffset;
  float         Bscale;
  uint8_t		Bgain;

}hx711_tt;

/* Setup functions */
void hx7111_init(hx711_tt *hx7111, GPIO_TypeDef *clk_gpio, uint16_t clk_pin, GPIO_TypeDef *dat_gpio, uint16_t dat_pin);
void set_scale(hx711_tt *hx7111, float Ascale, float Bscale);
void set_gain(hx711_tt *hx7111, uint8_t Again, uint8_t Bgain);

/* Load cell reading functions */
void tare(hx711_tt *hx711, uint8_t times, uint8_t channel);
void tare_all(hx711_tt *hx7111, uint8_t times);
float get_weight(hx711_tt *hx7111, int8_t times, uint8_t channel);

/* Others */
void set_offset(hx711_tt *hx7111, long offset, uint8_t channel);
uint8_t shiftIn(hx711_tt *hx7111, uint8_t bit_count);
bool is_ready(hx711_tt *hx7111);
void wait_ready(hx711_tt *hx7111);
long read(hx711_tt *hx7111, uint8_t channel);
long read_average(hx711_tt *hx7111, int8_t times, uint8_t channel);
double get_value(hx711_tt *hx7111, int8_t times, uint8_t channel);


#endif /* APPLICATION_CORE_HX711_H_ */
