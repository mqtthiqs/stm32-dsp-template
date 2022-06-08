#pragma once

#include "parameters.h"
#include "dsp.h"

class Microphone {

  static constexpr int kDecimationFactor = 8; // 4,8,16 ok
  static constexpr int kPdmBlockSize = kBlockSize * kDecimationFactor;
  PdmFilter<1, kDecimationFactor> filter_;

  uint8_t pdm_block_[kPdmBlockSize * 2];
  int16_t pcm_block_[kBlockSize];
  I2S_HandleTypeDef hi2s_;

public:
  DMA_HandleTypeDef hdma_;

  struct ProcessCallback {
    virtual void Process(short* in, size_t size) = 0;
  };
private:
  ProcessCallback *callback_;

public:
  static Microphone *instance_;

  Microphone(I2S_Freq freq, ProcessCallback *callback) : callback_(callback) {
    instance_ = this;

    /*PLLI2S configuration */
    const uint32_t I2SFreq[8] = {8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000};
    const uint32_t I2SPLLN[8] = {258, 429, 213, 429, 426, 271, 260, 344};
    const uint32_t I2SPLLR[8] = {3, 4, 4, 4, 4, 6, 3, 2};

    // Enable PLLI2S clock
    RCC_PeriphCLKInitTypeDef rcc;
    HAL_RCCEx_GetPeriphCLKConfig(&rcc);
    rcc.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rcc.PLLI2S.PLLI2SN = I2SPLLN[freq];
    rcc.PLLI2S.PLLI2SR = I2SPLLR[freq];
    HAL_RCCEx_PeriphCLKConfig(&rcc);

    /* GPIO configuration: SCK (PB10), MOSI (PC3) */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef  GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Pin       = GPIO_PIN_10;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* I2S configuration */

    hi2s_.Instance = SPI2;
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_I2S_DISABLE(&hi2s_);

    hi2s_.Init.AudioFreq    = I2SFreq[freq] * kDecimationFactor / 4; // TODO comprendre
    hi2s_.Init.ClockSource  = I2S_CLOCK_PLL;
    hi2s_.Init.CPOL         = I2S_CPOL_HIGH;
    hi2s_.Init.DataFormat   = I2S_DATAFORMAT_16B;
    hi2s_.Init.MCLKOutput   = I2S_MCLKOUTPUT_DISABLE;
    hi2s_.Init.Mode         = I2S_MODE_MASTER_RX;
    hi2s_.Init.Standard     = I2S_STANDARD_LSB;

    if(HAL_I2S_Init(&hi2s_) != HAL_OK) {
      while(1);
    }

    /* DMA configuration */

    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Configure the hdma handle parameters */
    hdma_.Instance = DMA1_Stream3;
    hdma_.Init.Channel             = DMA_CHANNEL_0;
    hdma_.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_.Init.Mode                = DMA_CIRCULAR;
    hdma_.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_.Init.PeriphBurst         = DMA_MBURST_SINGLE;

    __HAL_LINKDMA(&hi2s_, hdmarx, hdma_);

    HAL_DMA_DeInit(&hdma_);
    HAL_DMA_Init(&hdma_);

    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  }

  void Start() {
    // size = kPdmBlockSize * 2 (double buffering) / 2 (uint16_t -> uint8_t)
    if (HAL_I2S_Receive_DMA(&hi2s_, (uint16_t*)pdm_block_, kPdmBlockSize) != HAL_OK)
      while(1);
  }

  void Process(int offset) {
    uint8_t *pdm_buf = &pdm_block_[offset * kPdmBlockSize];
    filter_.Process(pdm_buf, pcm_block_, kBlockSize);
    callback_->Process(pcm_block_, kBlockSize);
  }
};

Microphone *Microphone::instance_;

extern "C" {
  void DMA1_Stream3_IRQHandler() {
    HAL_DMA_IRQHandler(&Microphone::instance_->hdma_);
  }

  void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    Microphone::instance_->Process(0);
  }

  void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
    Microphone::instance_->Process(1);
  }
}
