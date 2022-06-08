#pragma once

#include "stm32f4xx_hal.h"

class Button {
public:
  Button() {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef init;
    init.Pin = GPIO_PIN_0;
    init.Mode = GPIO_MODE_INPUT;
    init.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(GPIOD, &init);
  }

  bool read() {
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
  }
};
