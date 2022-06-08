#pragma once

#include "stm32f4xx_hal.h"

class InternalDac {
  DAC_HandleTypeDef hdac_;

public:
  InternalDac() {
    // DAC_OUT1 (PA4) pin config
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Speed = GPIO_SPEED_LOW;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    // DAC config
    __HAL_RCC_DAC_CLK_ENABLE();
    hdac_.Instance = DAC1;

    DAC_ChannelConfTypeDef conf;
    conf.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
    conf.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;

    HAL_DAC_Init(&hdac_);
    HAL_DAC_ConfigChannel(&hdac_, &conf, DAC_CHANNEL_1);

    if (HAL_DAC_Start(&hdac_, DAC_CHANNEL_1) != HAL_OK)
      while(1);
  }

  void set(uint16_t val) {
    HAL_DAC_SetValue(&hdac_, DAC_CHANNEL_1, DAC_ALIGN_12B_L, val);
    HAL_DAC_Start(&hdac_, DAC_CHANNEL_1);
  }
};

// InternalDac internal_dac_;
