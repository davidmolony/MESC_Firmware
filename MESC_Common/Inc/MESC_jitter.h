/*
 * MESC_jitter.h
 *
 *  Created on: Aug 31, 2025
 *      Author: owhite
 */

#ifndef INC_MESC_JITTER_H_
#define INC_MESC_JITTER_H_

#include "stm32f4xx.h"

/* owen block of code to measure jitter */
#ifdef JITTER_TEST

// PB5 setup
static inline void jitter_pin_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; (void)RCC->AHB1ENR;

  GPIOB->MODER   = (GPIOB->MODER & ~(3U << (5U*2))) | (1U << (5U*2));
  GPIOB->OTYPER &= ~(1U << 5);
  GPIOB->OSPEEDR |= (3U << (5U*2));
  GPIOB->PUPDR &= ~(3U << (5U*2));
  GPIOB->BSRR = (1U << 5) << 16;
}

static inline void jit_set(void)    { GPIOB->BSRR =  (1U << 5); }
static inline void jit_clr(void)    { GPIOB->BSRR = ((1U << 5) << 16); }
static inline void jit_toggle(void) {  GPIOB->ODR ^= (1U << 5); }

#endif /* JITTER_TEST */

#endif /* INC_MESC_JITTER_H_ */
