#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LEDS_NUMBER    1

#define LED_START      26
#define LED_1          26
#define LED_STOP       26

#define LEDS_ACTIVE_STATE 1

#define LEDS_LIST { LED_1 }

#define LEDS_INV_MASK  LEDS_MASK

#define BSP_LED_0      LED_1

#define BUTTONS_NUMBER 1

#define BUTTON_1       25
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1


// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
