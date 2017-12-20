/*
 * Copyright (c) 2017, George Oikonomou - http://www.spd.gr
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup nrf52832-gpio-hal
 * @{
 *
 * \file
 *     Implementation file for the nrf52832 GPIO HAL functions
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/gpio-hal.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"

#include <stdint.h>
#if 0
nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
config.pull = NRF_GPIO_PIN_PULLUP;
nrf_drv_gpiote_in_init(pin, &config, gpiote_event_handler);
nrf_drv_gpiote_in_event_enable(pin, true);
#endif
/*---------------------------------------------------------------------------*/
static void
pin_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  gpio_hal_event_handler(gpio_hal_pin_to_mask(pin));
}
/*---------------------------------------------------------------------------*/
void
gpio_hal_arch_pin_cfg_set(gpio_hal_pin_t pin, gpio_hal_pin_cfg_t cfg)
{
  gpio_hal_pin_cfg_t tmp;
  nrf_drv_gpiote_in_config_t gpiote_config = {
    .is_watcher = false,
    .hi_accuracy = false,
  };

  tmp = cfg & GPIO_HAL_PIN_CFG_EDGE_BOTH;
  if(tmp == GPIO_HAL_PIN_CFG_EDGE_NONE) {
    gpiote_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
  } else if(tmp == GPIO_HAL_PIN_CFG_EDGE_RISING) {
    gpiote_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
  } else if(tmp == GPIO_HAL_PIN_CFG_EDGE_FALLING) {
    gpiote_config.sense = NRF_GPIOTE_POLARITY_HITOLO;
  } else if(tmp == GPIO_HAL_PIN_CFG_EDGE_BOTH) {
    gpiote_config.sense = GPIOTE_CONFIG_POLARITY_Toggle;
  }

  tmp = cfg & GPIO_HAL_PIN_CFG_PULL_MASK;
  if(tmp == GPIO_HAL_PIN_CFG_PULL_NONE) {
    gpiote_config.pull = NRF_GPIO_PIN_NOPULL;
  } else if(tmp == GPIO_HAL_PIN_CFG_PULL_DOWN) {
    gpiote_config.pull = NRF_GPIO_PIN_PULLDOWN;
  } else if(tmp == GPIO_HAL_PIN_CFG_PULL_UP) {
    gpiote_config.pull = NRF_GPIO_PIN_PULLUP;
  }

  nrf_drv_gpiote_in_init(pin, &gpiote_config, pin_event_handler);

  tmp = cfg & GPIO_HAL_PIN_CFG_INT_MASK;
  if(tmp == GPIO_HAL_PIN_CFG_INT_DISABLE) {
    nrf_drv_gpiote_in_event_disable(pin);
  } else if(tmp == GPIO_HAL_PIN_CFG_INT_ENABLE) {
    nrf_drv_gpiote_in_event_enable(pin, true);
  }
}
/*---------------------------------------------------------------------------*/
gpio_hal_pin_cfg_t
gpio_hal_arch_pin_cfg_get(gpio_hal_pin_t pin)
{
  gpio_hal_pin_cfg_t cfg = GPIO_HAL_PIN_CFG_PULL_NONE |
                           GPIO_HAL_PIN_CFG_EDGE_NONE |
                           GPIO_HAL_PIN_CFG_INT_DISABLE;
  nrf_gpio_pin_sense_t sense;

//  uint32_t config = NRF_GPIO->PIN_CNF[pin];

//  sense = (config & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos;

  sense = nrf_gpio_pin_sense_get(pin);

  if(sense == NRF_GPIO_PIN_SENSE_HIGH) {
    cfg |= GPIO_HAL_PIN_CFG_EDGE_RISING;
  } else if(sense == NRF_GPIO_PIN_SENSE_LOW) {
    cfg |= GPIO_HAL_PIN_CFG_EDGE_FALLING;
  }

  return cfg;
}
/*---------------------------------------------------------------------------*/
void
gpio_hal_arch_write_pins(gpio_hal_pin_mask_t pins, gpio_hal_pin_mask_t value)
{
  gpio_hal_pin_mask_t keep = nrf_gpio_pins_read() & ~pins;

  nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT0,
                      (keep & 0xFF) | (value & 0xFF));

  nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1,
                      ((keep >> 8) & 0xFF) | ((value >> 8) & 0xFF));

  nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT2,
                      ((keep >> 16) & 0xFF) | ((value >> 16) & 0xFF));

  nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT3,
                      ((keep >> 24) & 0xFF) | ((value >> 24) & 0xFF));
}
/*---------------------------------------------------------------------------*/
void
gpio_hal_arch_toggle_pins(gpio_hal_pin_mask_t pins)
{
  gpio_hal_arch_write_pins(pins, ~gpio_hal_arch_read_pins(pins));
}
/*---------------------------------------------------------------------------*/
/** @} */
