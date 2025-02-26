// Copyright 2015-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Modifications copyright (C) 2021 CarlosDerSeher

#include <esp_types.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "i2s.h"
#include "soc/lldesc.h"

#include "soc/rtc.h"

#include "esp_attr.h"
#include "soc/chip_revision.h"
#include "hal/efuse_hal.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_rom_gpio.h"

#include "sdkconfig.h"

static const char *I2S_TAG = "c_I2S";

#define I2S_CHECK(a, str, ret)                                    \
  if (!(a)) {                                                     \
    ESP_LOGE(I2S_TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
    return (ret);                                                 \
  }

#define I2S_ENTER_CRITICAL_ISR() portENTER_CRITICAL_ISR(&i2s_spinlock[i2s_num])
#define I2S_EXIT_CRITICAL_ISR() portEXIT_CRITICAL_ISR(&i2s_spinlock[i2s_num])
#define I2S_ENTER_CRITICAL() portENTER_CRITICAL(&i2s_spinlock[i2s_num])
#define I2S_EXIT_CRITICAL() portEXIT_CRITICAL(&i2s_spinlock[i2s_num])
#define I2S_FULL_DUPLEX_SLAVE_MODE_MASK \
  (I2S_MODE_TX | I2S_MODE_RX | I2S_MODE_SLAVE)
#define I2S_FULL_DUPLEX_MASTER_MODE_MASK \
  (I2S_MODE_TX | I2S_MODE_RX | I2S_MODE_MASTER)

// TODO: Refactor to put this logic into LL
#define I2S_AD_BCK_FACTOR (2)
#define I2S_PDM_BCK_FACTOR (64)
#define I2S_BASE_CLK (2 * APB_CLK_FREQ)

/**
 * @brief DMA buffer object
 *
 */
typedef struct {
  char **buf;
  int buf_size;
  int rw_pos;
  void *curr_ptr;
  SemaphoreHandle_t mux;
  xQueueHandle queue;
  lldesc_t **desc;
} i2s_dma_t;

/**
 * @brief I2S object instance
 *
 */
typedef struct {
  i2s_port_t i2s_num;      /*!< I2S port number*/
  int queue_size;          /*!< I2S event queue size*/
  QueueHandle_t i2s_queue; /*!< I2S queue handler*/
  int dma_buf_count;       /*!< DMA buffer count, number of buffer*/
  int dma_buf_len;         /*!< DMA buffer length, length of each buffer*/
  i2s_dma_t *rx;           /*!< DMA Tx buffer*/
  i2s_dma_t *tx;           /*!< DMA Rx buffer*/
  i2s_isr_handle_t i2s_isr_handle; /*!< I2S Interrupt handle*/
  int channel_num;                 /*!< Number of channels*/
  int bytes_per_sample;            /*!< Bytes per sample*/
  int bits_per_sample;             /*!< Bits per sample*/
  i2s_mode_t mode;                 /*!< I2S Working mode*/
  uint32_t sample_rate;            /*!< I2S sample rate */
  bool use_apll;                   /*!< I2S use APLL clock */
  bool tx_desc_auto_clear; /*!< I2S auto clear tx descriptor on underflow */
  int fixed_mclk;          /*!< I2S fixed MLCK clock */
  double real_rate;
#ifdef CONFIG_PM_ENABLE
  esp_pm_lock_handle_t pm_lock;
#endif
  i2s_hal_context_t hal; /*!< I2S hal context*/
} i2s_obj_t;

static i2s_obj_t *p_i2s_obj[I2S_NUM_MAX] = {0};

static portMUX_TYPE i2s_spinlock[I2S_NUM_MAX];

static i2s_dma_t *i2s_create_dma_queue(i2s_port_t i2s_num, int dma_buf_count,
                                       int dma_buf_len);
static esp_err_t i2s_destroy_dma_queue(i2s_port_t i2s_num, i2s_dma_t *dma);

static inline void gpio_matrix_out_check(int gpio, uint32_t signal_idx,
                                         bool out_inv, bool oen_inv) {
  // if pin = -1, do not need to configure
  if (gpio != -1) {
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(gpio, signal_idx, out_inv, oen_inv);
  }
}

static inline void gpio_matrix_in_check(int gpio, uint32_t signal_idx,
                                        bool inv) {
  if (gpio != -1) {
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    // Set direction, for some GPIOs, the input function are not enabled as
    // default.
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
    esp_rom_gpio_connect_in_signal(gpio, signal_idx, inv);
  }
}

esp_err_t i2s_custom_clear_intr_status(i2s_port_t i2s_num, uint32_t clr_mask) {
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  i2s_hal_clear_intr_status(&(p_i2s_obj[i2s_num]->hal), clr_mask);
  return ESP_OK;
}

esp_err_t i2s_custom_enable_rx_intr(i2s_port_t i2s_num) {
  I2S_ENTER_CRITICAL();
  i2s_hal_enable_rx_intr(&(p_i2s_obj[i2s_num]->hal));
  I2S_EXIT_CRITICAL();
  return ESP_OK;
}

esp_err_t i2s_custom_disable_rx_intr(i2s_port_t i2s_num) {
  I2S_ENTER_CRITICAL();
  i2s_hal_disable_rx_intr(&(p_i2s_obj[i2s_num]->hal));
  I2S_EXIT_CRITICAL();
  return ESP_OK;
}

esp_err_t i2s_custom_disable_tx_intr(i2s_port_t i2s_num) {
  I2S_ENTER_CRITICAL();
  i2s_hal_disable_tx_intr(&(p_i2s_obj[i2s_num]->hal));
  I2S_EXIT_CRITICAL();
  return ESP_OK;
}

esp_err_t i2s_custom_enable_tx_intr(i2s_port_t i2s_num) {
  I2S_ENTER_CRITICAL();
  i2s_hal_enable_tx_intr(&(p_i2s_obj[i2s_num]->hal));
  I2S_EXIT_CRITICAL();
  return ESP_OK;
}

float i2s_custom_get_clk(i2s_port_t i2s_num) {
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  return p_i2s_obj[i2s_num]->real_rate;
}

static esp_err_t i2s_isr_register(i2s_port_t i2s_num, int intr_alloc_flags,
                                  void (*fn)(void *), void *arg,
                                  i2s_isr_handle_t *handle) {
  return esp_intr_alloc(i2s_periph_signal[i2s_num].irq, intr_alloc_flags, fn,
                        arg, handle);
}

static float i2s_apll_get_fi2s(int bits_per_sample, int sdm0, int sdm1,
                               int sdm2, int odir) {
  int f_xtal = (int)rtc_clk_xtal_freq_get() * 1000000;

#if CONFIG_IDF_TARGET_ESP32
    /* ESP32 rev0 silicon issue for APLL range/accuracy, please see ESP32 ECO document for more information on this */
    if (!ESP_CHIP_REV_ABOVE(efuse_hal_chip_revision(), 100)) {
        sdm0 = 0;
        sdm1 = 0;
    }
#endif
  float fout = f_xtal * (sdm2 + sdm1 / 256.0f + sdm0 / 65536.0f + 4);
  if (fout < SOC_I2S_APLL_MIN_FREQ || fout > SOC_I2S_APLL_MAX_FREQ) {
    return SOC_I2S_APLL_MAX_FREQ;
  }
  float fpll = fout / (2 * (odir + 2));  //== fi2s (N=1, b=0, a=1)
  return fpll / 2;
}

/**
 * @brief     APLL calculate function, was described by following:
 *            APLL Output frequency is given by the formula:
 *
 *            apll_freq = xtal_freq * (4 + sdm2 + sdm1/256 +
 * sdm0/65536)/((o_div + 2) * 2) apll_freq = fout / ((o_div + 2) * 2)
 *
 *            The dividend in this expression should be in the range of 240 -
 * 600 MHz. In rev. 0 of ESP32, sdm0 and sdm1 are unused and always set to 0.
 *            * sdm0  frequency adjustment parameter, 0..255
 *            * sdm1  frequency adjustment parameter, 0..255
 *            * sdm2  frequency adjustment parameter, 0..63
 *            * o_div  frequency divider, 0..31
 *
 *            The most accurate way to find the sdm0..2 and odir parameters is
 * to loop through them all, then apply the above formula, finding the closest
 * frequency to the desired one. But 256*256*64*32 = 134.217.728 loops are too
 * slow with ESP32
 *            1. We will choose the parameters with the highest level of
 * change, With 350MHz<fout<500MHz, we limit the sdm2 from 4 to 9, Take average
 * frequency close to the desired frequency, and select sdm2
 *            2. Next, we look for sequences of less influential and more
 * detailed parameters, also by taking the average of the largest and smallest
 * frequencies closer to the desired frequency.
 *            3. And finally, loop through all the most detailed of the
 * parameters, finding the best desired frequency
 *
 * @param[in]  rate                  The I2S Frequency (MCLK)
 * @param[in]  bits_per_sample       The bits per sample
 * @param[out]      sdm0             The sdm 0
 * @param[out]      sdm1             The sdm 1
 * @param[out]      sdm2             The sdm 2
 * @param[out]      odir             The odir
 *
 * @return     ESP_ERR_INVALID_ARG or ESP_OK
 */

esp_err_t i2s_apll_calculate_fi2s(int rate, int bits_per_sample, int *sdm0,
                                  int *sdm1, int *sdm2, int *odir) {
  int _odir, _sdm0, _sdm1, _sdm2;
  float avg;
  float min_rate, max_rate, min_diff;
  if (rate / bits_per_sample / 2 / 8 < SOC_I2S_APLL_MIN_RATE) {
    return ESP_ERR_INVALID_ARG;
  }

  *sdm0 = 0;
  *sdm1 = 0;
  *sdm2 = 0;
  *odir = 0;
  min_diff = SOC_I2S_APLL_MAX_FREQ;

  for (_sdm2 = 4; _sdm2 < 9; _sdm2++) {
    max_rate = i2s_apll_get_fi2s(bits_per_sample, 255, 255, _sdm2, 0);
    min_rate = i2s_apll_get_fi2s(bits_per_sample, 0, 0, _sdm2, 31);
    avg = (max_rate + min_rate) / 2;
    if (abs(avg - rate) < min_diff) {
      min_diff = abs(avg - rate);
      *sdm2 = _sdm2;
    }
  }
  min_diff = SOC_I2S_APLL_MAX_FREQ;
  for (_odir = 0; _odir < 32; _odir++) {
    max_rate = i2s_apll_get_fi2s(bits_per_sample, 255, 255, *sdm2, _odir);
    min_rate = i2s_apll_get_fi2s(bits_per_sample, 0, 0, *sdm2, _odir);
    avg = (max_rate + min_rate) / 2;
    if (abs(avg - rate) < min_diff) {
      min_diff = abs(avg - rate);
      *odir = _odir;
    }
  }
  min_diff = SOC_I2S_APLL_MAX_FREQ;
  for (_sdm2 = 4; _sdm2 < 9; _sdm2++) {
    max_rate = i2s_apll_get_fi2s(bits_per_sample, 255, 255, _sdm2, *odir);
    min_rate = i2s_apll_get_fi2s(bits_per_sample, 0, 0, _sdm2, *odir);
    avg = (max_rate + min_rate) / 2;
    if (abs(avg - rate) < min_diff) {
      min_diff = abs(avg - rate);
      *sdm2 = _sdm2;
    }
  }

  min_diff = SOC_I2S_APLL_MAX_FREQ;
  for (_sdm1 = 0; _sdm1 < 256; _sdm1++) {
    max_rate = i2s_apll_get_fi2s(bits_per_sample, 255, _sdm1, *sdm2, *odir);
    min_rate = i2s_apll_get_fi2s(bits_per_sample, 0, _sdm1, *sdm2, *odir);
    avg = (max_rate + min_rate) / 2;
    if (abs(avg - rate) < min_diff) {
      min_diff = abs(avg - rate);
      *sdm1 = _sdm1;
    }
  }

  min_diff = SOC_I2S_APLL_MAX_FREQ;
  for (_sdm0 = 0; _sdm0 < 256; _sdm0++) {
    avg = i2s_apll_get_fi2s(bits_per_sample, _sdm0, *sdm1, *sdm2, *odir);
    if (abs(avg - rate) < min_diff) {
      min_diff = abs(avg - rate);
      *sdm0 = _sdm0;
    }
  }

  return ESP_OK;
}

esp_err_t i2s_custom_init_dma_tx_queues(i2s_port_t i2s_num, uint8_t *data,
                                        size_t size, size_t *written,
                                        uint32_t *currentDescriptor,
                                        uint32_t *currentDescriptorOffset) {
  size_t tmpSize = size;
  uint32_t i;

  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  I2S_CHECK((p_i2s_obj[i2s_num]->tx), "tx NULL", ESP_ERR_INVALID_ARG);

  i2s_custom_stop(i2s_num);

  xSemaphoreTake(p_i2s_obj[i2s_num]->tx->mux, (portTickType)portMAX_DELAY);

  i2s_hal_set_out_link_addr(&(p_i2s_obj[i2s_num]->hal),
                            (uint32_t)p_i2s_obj[i2s_num]->tx->desc[0]);

  p_i2s_obj[i2s_num]->tx->curr_ptr = NULL;
  p_i2s_obj[i2s_num]->tx->rw_pos = 0;

  // fill DMA buffers
  if ((data != NULL) && (written != NULL) && (size != 0)) {
    size_t offset = *currentDescriptorOffset;
    size_t maxDmaBufBytes = p_i2s_obj[i2s_num]->dma_buf_len *
                            p_i2s_obj[i2s_num]->bytes_per_sample *
                            p_i2s_obj[i2s_num]->channel_num;

    for (i = *currentDescriptor; i < p_i2s_obj[i2s_num]->dma_buf_count; i++) {
      char *buf = (char *)p_i2s_obj[i2s_num]->tx->desc[i]->buf;

      if (tmpSize > maxDmaBufBytes) {
        memcpy(buf, &data[offset], maxDmaBufBytes);
        offset += maxDmaBufBytes;

        // ESP_LOGW(I2S_TAG, "wrote %d to desc[%d]", maxDmaBufBytes, i);

        tmpSize -= maxDmaBufBytes;
      } else {
        memcpy(buf, &data[offset], tmpSize);
        offset += tmpSize;

        // ESP_LOGW(I2S_TAG, "wrote %d to desc[%d]", tmpSize, i);

        tmpSize = 0;
      }

      if (tmpSize == 0) {
        break;
      }
    }

    if (currentDescriptor) {
      *currentDescriptor = i + 1;
    }

    if (currentDescriptorOffset) {
      if (offset == size) {
        *currentDescriptorOffset = 0;
      } else {
        *currentDescriptorOffset = offset;
      }
    }

    *written = offset;
  }

  // empty queue
  xQueueReset(p_i2s_obj[i2s_num]->tx->queue);

  xSemaphoreGive(p_i2s_obj[i2s_num]->tx->mux);

  return ESP_OK;
}

esp_err_t i2s_custom_set_clk(i2s_port_t i2s_num, uint32_t rate,
                             i2s_bits_per_sample_t bits, i2s_channel_t ch) {
  int factor =
      (256 % bits) ? 384 : 256;  // According to hardware codec
                                 // requirement(supported 256fs or 384fs)
  int clkmInteger, clkmDecimals, bck = 0;
  double denom = (double)1 / 64;
  int channel = 2;
  i2s_dma_t *save_tx = NULL, *save_rx = NULL;

  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);

  if (bits % 8 != 0 || bits > I2S_BITS_PER_SAMPLE_32BIT ||
      bits < I2S_BITS_PER_SAMPLE_16BIT) {
    ESP_LOGE(I2S_TAG, "Invalid bits per sample");
    return ESP_ERR_INVALID_ARG;
  }

  if (p_i2s_obj[i2s_num] == NULL) {
    ESP_LOGE(I2S_TAG, "Not initialized yet");
    return ESP_ERR_INVALID_ARG;
  }
  p_i2s_obj[i2s_num]->sample_rate = rate;
  double clkmdiv = (double)I2S_BASE_CLK / (rate * factor);

  if (clkmdiv > 256) {
    ESP_LOGE(I2S_TAG, "clkmdiv is too large\r\n");
    return ESP_ERR_INVALID_ARG;
  }

  // wait all on-going writing finish
  if ((p_i2s_obj[i2s_num]->mode & I2S_MODE_TX) && p_i2s_obj[i2s_num]->tx) {
    xSemaphoreTake(p_i2s_obj[i2s_num]->tx->mux, (portTickType)portMAX_DELAY);
  }
  if ((p_i2s_obj[i2s_num]->mode & I2S_MODE_RX) && p_i2s_obj[i2s_num]->rx) {
    xSemaphoreTake(p_i2s_obj[i2s_num]->rx->mux, (portTickType)portMAX_DELAY);
  }

  i2s_custom_stop(i2s_num);
  i2s_hal_set_rx_mode(&(p_i2s_obj[i2s_num]->hal), ch, bits);
  i2s_hal_set_tx_mode(&(p_i2s_obj[i2s_num]->hal), ch, bits);

  if (p_i2s_obj[i2s_num]->channel_num != (int)ch) {
    p_i2s_obj[i2s_num]->channel_num = (ch == 2) ? 2 : 1;
  }

  if ((int)bits != p_i2s_obj[i2s_num]->bits_per_sample) {
    p_i2s_obj[i2s_num]->bits_per_sample = bits;

    // Round bytes_per_sample up to next multiple of 16 bits
    int halfwords_per_sample = (bits + 15) / 16;
    p_i2s_obj[i2s_num]->bytes_per_sample = halfwords_per_sample * 2;

    // Because limited of DMA buffer is 4092 bytes
    if (p_i2s_obj[i2s_num]->dma_buf_len * p_i2s_obj[i2s_num]->bytes_per_sample *
            p_i2s_obj[i2s_num]->channel_num >
        4092) {
      p_i2s_obj[i2s_num]->dma_buf_len = 4092 /
                                        p_i2s_obj[i2s_num]->bytes_per_sample /
                                        p_i2s_obj[i2s_num]->channel_num;
    }
    // Re-create TX DMA buffer
    if (p_i2s_obj[i2s_num]->mode & I2S_MODE_TX) {
      save_tx = p_i2s_obj[i2s_num]->tx;

      p_i2s_obj[i2s_num]->tx =
          i2s_create_dma_queue(i2s_num, p_i2s_obj[i2s_num]->dma_buf_count,
                               p_i2s_obj[i2s_num]->dma_buf_len);
      if (p_i2s_obj[i2s_num]->tx == NULL) {
        ESP_LOGE(I2S_TAG, "Failed to create tx dma buffer");
        i2s_custom_driver_uninstall(i2s_num);
        return ESP_ERR_NO_MEM;
      }
      i2s_hal_set_out_link_addr(&(p_i2s_obj[i2s_num]->hal),
                                (uint32_t)p_i2s_obj[i2s_num]->tx->desc[0]);

      // destroy old tx dma if exist
      if (save_tx) {
        i2s_destroy_dma_queue(i2s_num, save_tx);
      }
    }
    // Re-create RX DMA buffer
    if (p_i2s_obj[i2s_num]->mode & I2S_MODE_RX) {
      save_rx = p_i2s_obj[i2s_num]->rx;

      p_i2s_obj[i2s_num]->rx =
          i2s_create_dma_queue(i2s_num, p_i2s_obj[i2s_num]->dma_buf_count,
                               p_i2s_obj[i2s_num]->dma_buf_len);
      if (p_i2s_obj[i2s_num]->rx == NULL) {
        ESP_LOGE(I2S_TAG, "Failed to create rx dma buffer");
        i2s_custom_driver_uninstall(i2s_num);
        return ESP_ERR_NO_MEM;
      }
      i2s_hal_set_in_link(&(p_i2s_obj[i2s_num]->hal),
                          p_i2s_obj[i2s_num]->dma_buf_len *
                              p_i2s_obj[i2s_num]->channel_num *
                              p_i2s_obj[i2s_num]->bytes_per_sample,
                          (uint32_t)p_i2s_obj[i2s_num]->rx->desc[0]);
      // destroy old rx dma if exist
      if (save_rx) {
        i2s_destroy_dma_queue(i2s_num, save_rx);
      }
    }
  }

  double mclk;
  int sdm0, sdm1, sdm2, odir, m_scale = 8;
  int fi2s_clk = rate * channel * bits * m_scale;
#if SOC_I2S_SUPPORTS_PDM
  if (p_i2s_obj[i2s_num]->mode & I2S_MODE_PDM) {
    uint32_t b_clk = 0;
    if (p_i2s_obj[i2s_num]->mode & I2S_MODE_TX) {
      uint32_t fp, fs;
      i2s_hal_get_tx_pdm(&(p_i2s_obj[i2s_num]->hal), &fp, &fs);
      // Recommended set `fp = 960, fs = sample_rate / 100`
      fs = rate / 100;
      i2s_hal_tx_pdm_cfg(&(p_i2s_obj[i2s_num]->hal), fp, fs);
      b_clk = rate * I2S_PDM_BCK_FACTOR * fp / fs;
    } else if (p_i2s_obj[i2s_num]->mode & I2S_MODE_RX) {
      uint32_t dsr;
      i2s_hal_get_rx_pdm(&(p_i2s_obj[i2s_num]->hal), &dsr);
      b_clk = rate * I2S_PDM_BCK_FACTOR * (dsr ? 2 : 1);
    }
    fi2s_clk = b_clk * m_scale;
    int factor2 = 5;
    mclk = b_clk * factor2;
    clkmdiv = ((double)I2S_BASE_CLK) / mclk;
    clkmInteger = clkmdiv;
    clkmDecimals = (clkmdiv - clkmInteger) / denom;
    bck = mclk / b_clk;
  } else
#endif
  {
    clkmInteger = clkmdiv;
    clkmDecimals = (clkmdiv - clkmInteger) / denom;
    mclk = clkmInteger + denom * clkmDecimals;
    bck = factor / (bits * channel);
  }

  if (p_i2s_obj[i2s_num]->use_apll && p_i2s_obj[i2s_num]->fixed_mclk) {
    fi2s_clk = p_i2s_obj[i2s_num]->fixed_mclk;
    m_scale = fi2s_clk / bits / rate / channel;
  }
  if (p_i2s_obj[i2s_num]->use_apll &&
      i2s_apll_calculate_fi2s(fi2s_clk, bits, &sdm0, &sdm1, &sdm2, &odir) ==
          ESP_OK) {
    ESP_LOGD(I2S_TAG, "sdm0=%d, sdm1=%d, sdm2=%d, odir=%d", sdm0, sdm1, sdm2,
             odir);
    rtc_clk_apll_enable(1, sdm0, sdm1, sdm2, odir);
    i2s_hal_set_clk_div(&(p_i2s_obj[i2s_num]->hal), 1, 1, 0, m_scale, m_scale);
    i2s_hal_set_clock_sel(&(p_i2s_obj[i2s_num]->hal), I2S_CLK_APLL);
    double fi2s_rate = i2s_apll_get_fi2s(bits, sdm0, sdm1, sdm2, odir);
    p_i2s_obj[i2s_num]->real_rate = fi2s_rate / bits / channel / m_scale;
    ESP_LOGI(I2S_TAG,
             "APLL: Req RATE: %d, real rate: %0.3f, BITS: %u, CLKM: %u, "
             "BCK_M: %u, MCLK: %0.3f, SCLK: %f, diva: %d, divb: %d",
             rate, fi2s_rate / bits / channel / m_scale, bits, 1, m_scale,
             fi2s_rate, fi2s_rate / 8, 1, 0);
  } else {
    i2s_hal_set_clock_sel(&(p_i2s_obj[i2s_num]->hal), I2S_CLK_D2CLK);
    i2s_hal_set_clk_div(&(p_i2s_obj[i2s_num]->hal), clkmInteger, 63,
                        clkmDecimals, bck, bck);
    double real_rate = (double)(I2S_BASE_CLK / (bck * bits * clkmInteger) / 2);
    p_i2s_obj[i2s_num]->real_rate = real_rate;
    ESP_LOGI(I2S_TAG,
             "PLL_D2: Req RATE: %d, real rate: %0.3f, BITS: %u, CLKM: %u, "
             "BCK: %u, MCLK: %0.3f, SCLK: %f, diva: %d, divb: %d",
             rate, real_rate, bits, clkmInteger, bck,
             (double)I2S_BASE_CLK / mclk, real_rate * bits * channel, 64,
             clkmDecimals);
  }
  if (p_i2s_obj[i2s_num]->mode & I2S_MODE_TX) {
    p_i2s_obj[i2s_num]->tx->curr_ptr = NULL;
    p_i2s_obj[i2s_num]->tx->rw_pos = 0;
  }
  if (p_i2s_obj[i2s_num]->mode & I2S_MODE_RX) {
    p_i2s_obj[i2s_num]->rx->curr_ptr = NULL;
    p_i2s_obj[i2s_num]->rx->rw_pos = 0;
  }

  i2s_hal_set_tx_bits_mod(&(p_i2s_obj[i2s_num]->hal), bits);
  i2s_hal_set_rx_bits_mod(&(p_i2s_obj[i2s_num]->hal), bits);

  // wait all writing on-going finish
  if ((p_i2s_obj[i2s_num]->mode & I2S_MODE_TX) && p_i2s_obj[i2s_num]->tx) {
    xSemaphoreGive(p_i2s_obj[i2s_num]->tx->mux);
  }
  if ((p_i2s_obj[i2s_num]->mode & I2S_MODE_RX) && p_i2s_obj[i2s_num]->rx) {
    xSemaphoreGive(p_i2s_obj[i2s_num]->rx->mux);
  }

  //    i2s_custom_start(i2s_num);	// don't start just yet, we want to
  //    fill dma buffer first return ESP_OK;

  // ensure all DMA buffers are available right after initialization
  // ENSURE i2s_custom_start() isn't called before i2s_write has filled at
  // least one buffer
  return i2s_custom_init_dma_tx_queues(i2s_num, NULL, 0, NULL, NULL, NULL);
}

static void IRAM_ATTR i2s_intr_handler_default(void *arg) {
  i2s_obj_t *p_i2s = (i2s_obj_t *)arg;
  uint32_t status;
  i2s_hal_get_intr_status(&(p_i2s->hal), &status);
  if (status == 0) {
    // Avoid spurious interrupt
    return;
  }

  i2s_event_t i2s_event;
  int dummy;

  portBASE_TYPE high_priority_task_awoken = 0;

  lldesc_t *finish_desc = NULL;

  if ((status & I2S_INTR_OUT_DSCR_ERR) || (status & I2S_INTR_IN_DSCR_ERR)) {
    ESP_EARLY_LOGE(I2S_TAG, "dma error, interrupt status: 0x%08x", status);
    if (p_i2s->i2s_queue) {
      i2s_event.type = I2S_EVENT_DMA_ERROR;
      if (xQueueIsQueueFullFromISR(p_i2s->i2s_queue)) {
        xQueueReceiveFromISR(p_i2s->i2s_queue, &dummy,
                             &high_priority_task_awoken);
      }
      xQueueSendFromISR(p_i2s->i2s_queue, (void *)&i2s_event,
                        &high_priority_task_awoken);
    }
  }

  if ((status & I2S_INTR_OUT_EOF) && p_i2s->tx) {
    i2s_hal_get_out_eof_des_addr(&(p_i2s->hal), (uint32_t *)&finish_desc);
    // All buffers are empty. This means we have an underflow on our hands.
    if (xQueueIsQueueFullFromISR(p_i2s->tx->queue)) {
      xQueueReceiveFromISR(p_i2s->tx->queue, &dummy,
                           &high_priority_task_awoken);
      // See if tx descriptor needs to be auto cleared:
      // This will avoid any kind of noise that may get introduced due to
      // transmission of previous data from tx descriptor on I2S line.
      if (p_i2s->tx_desc_auto_clear == true) {
        memset((void *)dummy, 0, p_i2s->tx->buf_size);
      }
    }
    xQueueSendFromISR(p_i2s->tx->queue, (void *)(&finish_desc->buf),
                      &high_priority_task_awoken);
    if (p_i2s->i2s_queue) {
      i2s_event.type = I2S_EVENT_TX_DONE;
      if (xQueueIsQueueFullFromISR(p_i2s->i2s_queue)) {
        xQueueReceiveFromISR(p_i2s->i2s_queue, &dummy,
                             &high_priority_task_awoken);
      }
      xQueueSendFromISR(p_i2s->i2s_queue, (void *)&i2s_event,
                        &high_priority_task_awoken);
    }
  }

  if ((status & I2S_INTR_IN_SUC_EOF) && p_i2s->rx) {
    // All buffers are full. This means we have an overflow.
    i2s_hal_get_in_eof_des_addr(&(p_i2s->hal), (uint32_t *)&finish_desc);
    if (xQueueIsQueueFullFromISR(p_i2s->rx->queue)) {
      xQueueReceiveFromISR(p_i2s->rx->queue, &dummy,
                           &high_priority_task_awoken);
    }
    xQueueSendFromISR(p_i2s->rx->queue, (void *)(&finish_desc->buf),
                      &high_priority_task_awoken);
    if (p_i2s->i2s_queue) {
      i2s_event.type = I2S_EVENT_RX_DONE;
      if (p_i2s->i2s_queue && xQueueIsQueueFullFromISR(p_i2s->i2s_queue)) {
        xQueueReceiveFromISR(p_i2s->i2s_queue, &dummy,
                             &high_priority_task_awoken);
      }
      xQueueSendFromISR(p_i2s->i2s_queue, (void *)&i2s_event,
                        &high_priority_task_awoken);
    }
  }
  i2s_hal_clear_intr_status(&(p_i2s->hal), status);

  if (high_priority_task_awoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static esp_err_t i2s_destroy_dma_queue(i2s_port_t i2s_num, i2s_dma_t *dma) {
  int bux_idx;
  if (p_i2s_obj[i2s_num] == NULL) {
    ESP_LOGE(I2S_TAG, "Not initialized yet");
    return ESP_ERR_INVALID_ARG;
  }
  if (dma == NULL) {
    ESP_LOGE(I2S_TAG, "dma is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  for (bux_idx = 0; bux_idx < p_i2s_obj[i2s_num]->dma_buf_count; bux_idx++) {
    if (dma->desc && dma->desc[bux_idx]) {
      free(dma->desc[bux_idx]);
    }
    if (dma->buf && dma->buf[bux_idx]) {
      free(dma->buf[bux_idx]);
    }
  }
  if (dma->buf) {
    free(dma->buf);
  }
  if (dma->desc) {
    free(dma->desc);
  }
  vQueueDelete(dma->queue);
  vSemaphoreDelete(dma->mux);
  free(dma);
  return ESP_OK;
}

static i2s_dma_t *i2s_create_dma_queue(i2s_port_t i2s_num, int dma_buf_count,
                                       int dma_buf_len) {
  int bux_idx;
  int sample_size =
      p_i2s_obj[i2s_num]->bytes_per_sample * p_i2s_obj[i2s_num]->channel_num;
  i2s_dma_t *dma = (i2s_dma_t *)malloc(sizeof(i2s_dma_t));
  if (dma == NULL) {
    ESP_LOGE(I2S_TAG, "Error malloc i2s_dma_t");
    return NULL;
  }
  memset(dma, 0, sizeof(i2s_dma_t));

  dma->buf = (char **)malloc(sizeof(char *) * dma_buf_count);
  if (dma->buf == NULL) {
    ESP_LOGE(I2S_TAG, "Error malloc dma buffer pointer");
    free(dma);
    return NULL;
  }
  memset(dma->buf, 0, sizeof(char *) * dma_buf_count);

  for (bux_idx = 0; bux_idx < dma_buf_count; bux_idx++) {
    dma->buf[bux_idx] =
        (char *)heap_caps_calloc(1, dma_buf_len * sample_size, MALLOC_CAP_DMA);
    if (dma->buf[bux_idx] == NULL) {
      ESP_LOGE(I2S_TAG, "Error malloc dma buffer");
      i2s_destroy_dma_queue(i2s_num, dma);
      return NULL;
    }
    ESP_LOGD(I2S_TAG, "Addr[%d] = %d", bux_idx, (int)dma->buf[bux_idx]);
  }

  dma->desc = (lldesc_t **)malloc(sizeof(lldesc_t *) * dma_buf_count);
  if (dma->desc == NULL) {
    ESP_LOGE(I2S_TAG, "Error malloc dma description");
    i2s_destroy_dma_queue(i2s_num, dma);
    return NULL;
  }
  for (bux_idx = 0; bux_idx < dma_buf_count; bux_idx++) {
    dma->desc[bux_idx] =
        (lldesc_t *)heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (dma->desc[bux_idx] == NULL) {
      ESP_LOGE(I2S_TAG, "Error malloc dma description entry");
      i2s_destroy_dma_queue(i2s_num, dma);
      return NULL;
    }
  }

  for (bux_idx = 0; bux_idx < dma_buf_count; bux_idx++) {
    dma->desc[bux_idx]->owner = 1;
    dma->desc[bux_idx]->eof = 1;
    dma->desc[bux_idx]->sosf = 0;
    dma->desc[bux_idx]->length = dma_buf_len * sample_size;
    dma->desc[bux_idx]->size = dma_buf_len * sample_size;
    dma->desc[bux_idx]->buf = (uint8_t *)dma->buf[bux_idx];
    dma->desc[bux_idx]->offset = 0;
    dma->desc[bux_idx]->empty =
        (uint32_t)((bux_idx < (dma_buf_count - 1)) ? (dma->desc[bux_idx + 1])
                                                   : dma->desc[0]);
  }

  dma->queue = xQueueCreate(dma_buf_count - 1, sizeof(char *));
  dma->mux = xSemaphoreCreateMutex();
  dma->buf_size = dma_buf_len * sample_size;
  ESP_LOGI(I2S_TAG, "DMA Malloc info, datalen=blocksize=%d, dma_buf_count=%d",
           dma_buf_len * sample_size, dma_buf_count);
  return dma;
}

esp_err_t i2s_custom_start(i2s_port_t i2s_num) {
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  // start DMA link
  I2S_ENTER_CRITICAL();
  i2s_hal_reset(&(p_i2s_obj[i2s_num]->hal));

  esp_intr_disable(p_i2s_obj[i2s_num]->i2s_isr_handle);
  i2s_hal_clear_intr_status(&(p_i2s_obj[i2s_num]->hal), I2S_INTR_MAX);
  if (p_i2s_obj[i2s_num]->mode & I2S_MODE_TX) {
    i2s_custom_enable_tx_intr(i2s_num);
    i2s_hal_start_tx(&(p_i2s_obj[i2s_num]->hal));
  }
  if (p_i2s_obj[i2s_num]->mode & I2S_MODE_RX) {
    i2s_custom_enable_rx_intr(i2s_num);
    i2s_hal_start_rx(&(p_i2s_obj[i2s_num]->hal));
  }
  esp_intr_enable(p_i2s_obj[i2s_num]->i2s_isr_handle);
  I2S_EXIT_CRITICAL();
  return ESP_OK;
}

esp_err_t i2s_custom_stop(i2s_port_t i2s_num) {
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  I2S_ENTER_CRITICAL();
  esp_intr_disable(p_i2s_obj[i2s_num]->i2s_isr_handle);
  if (p_i2s_obj[i2s_num]->mode & I2S_MODE_TX) {
    i2s_hal_stop_tx(&(p_i2s_obj[i2s_num]->hal));
    i2s_custom_disable_tx_intr(i2s_num);
  }
  if (p_i2s_obj[i2s_num]->mode & I2S_MODE_RX) {
    i2s_hal_stop_rx(&(p_i2s_obj[i2s_num]->hal));
    i2s_custom_disable_rx_intr(i2s_num);
  }
  uint32_t mask;
  i2s_hal_get_intr_status(&(p_i2s_obj[i2s_num]->hal), &mask);
  i2s_hal_clear_intr_status(&(p_i2s_obj[i2s_num]->hal), mask);
  I2S_EXIT_CRITICAL();
  return ESP_OK;
}

esp_err_t i2s_custom_set_pin(i2s_port_t i2s_num, const i2s_pin_config_t *pin) {
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  if (pin == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (pin->bck_io_num != -1 && !GPIO_IS_VALID_GPIO(pin->bck_io_num)) {
    ESP_LOGE(I2S_TAG, "bck_io_num error");
    return ESP_FAIL;
  }
  if (pin->ws_io_num != -1 && !GPIO_IS_VALID_GPIO(pin->ws_io_num)) {
    ESP_LOGE(I2S_TAG, "ws_io_num error");
    return ESP_FAIL;
  }
  if (pin->data_out_num != -1 &&
      !GPIO_IS_VALID_OUTPUT_GPIO(pin->data_out_num)) {
    ESP_LOGE(I2S_TAG, "data_out_num error");
    return ESP_FAIL;
  }
  if (pin->data_in_num != -1 && !GPIO_IS_VALID_GPIO(pin->data_in_num)) {
    ESP_LOGE(I2S_TAG, "data_in_num error");
    return ESP_FAIL;
  }

  int bck_sig = -1, ws_sig = -1, data_out_sig = -1, data_in_sig = -1;
  // Each IIS hw module has a RX and TX unit.
  // For TX unit, the output signal index should be I2SnO_xxx_OUT_IDX
  // For TX unit, the input signal index should be I2SnO_xxx_IN_IDX
  if (p_i2s_obj[i2s_num]->mode & I2S_MODE_TX) {
    if (p_i2s_obj[i2s_num]->mode & I2S_MODE_MASTER) {
      bck_sig = i2s_periph_signal[i2s_num].o_bck_out_sig;
      ws_sig = i2s_periph_signal[i2s_num].o_ws_out_sig;
      data_out_sig = i2s_periph_signal[i2s_num].o_data_out_sig;
    } else if (p_i2s_obj[i2s_num]->mode & I2S_MODE_SLAVE) {
      bck_sig = i2s_periph_signal[i2s_num].o_bck_in_sig;
      ws_sig = i2s_periph_signal[i2s_num].o_ws_in_sig;
      data_out_sig = i2s_periph_signal[i2s_num].o_data_out_sig;
    }
  }
  // For RX unit, the output signal index should be I2SnI_xxx_OUT_IDX
  // For RX unit, the input signal index should be I2SnI_xxx_IN_IDX
  if (p_i2s_obj[i2s_num]->mode & I2S_MODE_RX) {
    if (p_i2s_obj[i2s_num]->mode & I2S_MODE_MASTER) {
      bck_sig = i2s_periph_signal[i2s_num].i_bck_out_sig;
      ws_sig = i2s_periph_signal[i2s_num].i_ws_out_sig;
      data_in_sig = i2s_periph_signal[i2s_num].i_data_in_sig;
    } else if (p_i2s_obj[i2s_num]->mode & I2S_MODE_SLAVE) {
      bck_sig = i2s_periph_signal[i2s_num].i_bck_in_sig;
      ws_sig = i2s_periph_signal[i2s_num].i_ws_in_sig;
      data_in_sig = i2s_periph_signal[i2s_num].i_data_in_sig;
    }
  }
  // For "full-duplex + slave" mode, we should select RX signal index for ws
  // and bck. For "full-duplex + master" mode, we should select TX signal index
  // for ws and bck.
  if ((p_i2s_obj[i2s_num]->mode & I2S_FULL_DUPLEX_SLAVE_MODE_MASK) ==
      I2S_FULL_DUPLEX_SLAVE_MODE_MASK) {
    bck_sig = i2s_periph_signal[i2s_num].i_bck_in_sig;
    ws_sig = i2s_periph_signal[i2s_num].i_ws_in_sig;
  } else if ((p_i2s_obj[i2s_num]->mode & I2S_FULL_DUPLEX_MASTER_MODE_MASK) ==
             I2S_FULL_DUPLEX_MASTER_MODE_MASK) {
    bck_sig = i2s_periph_signal[i2s_num].o_bck_out_sig;
    ws_sig = i2s_periph_signal[i2s_num].o_ws_out_sig;
  }
  gpio_matrix_out_check(pin->data_out_num, data_out_sig, 0, 0);
  gpio_matrix_in_check(pin->data_in_num, data_in_sig, 0);
  if (p_i2s_obj[i2s_num]->mode & I2S_MODE_MASTER) {
    gpio_matrix_out_check(pin->ws_io_num, ws_sig, 0, 0);
    gpio_matrix_out_check(pin->bck_io_num, bck_sig, 0, 0);
  } else if (p_i2s_obj[i2s_num]->mode & I2S_MODE_SLAVE) {
    gpio_matrix_in_check(pin->ws_io_num, ws_sig, 0);
    gpio_matrix_in_check(pin->bck_io_num, bck_sig, 0);
  }
  ESP_LOGD(I2S_TAG, "data: out %d, in: %d, ws: %d, bck: %d", data_out_sig,
           data_in_sig, ws_sig, bck_sig);

  return ESP_OK;
}

esp_err_t i2s_custom_set_sample_rates(i2s_port_t i2s_num, uint32_t rate) {
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  I2S_CHECK((p_i2s_obj[i2s_num]->bytes_per_sample > 0),
            "bits_per_sample not set", ESP_ERR_INVALID_ARG);
  return i2s_custom_set_clk(i2s_num, rate, p_i2s_obj[i2s_num]->bits_per_sample,
                            p_i2s_obj[i2s_num]->channel_num);
}

#if SOC_I2S_SUPPORTS_PDM
esp_err_t i2s_custom_set_pdm_rx_down_sample(i2s_port_t i2s_num,
                                            i2s_pdm_dsr_t dsr) {
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  i2s_hal_rx_pdm_cfg(&(p_i2s_obj[i2s_num]->hal), dsr);
  return i2s_custom_set_clk(i2s_num, p_i2s_obj[i2s_num]->sample_rate,
                            p_i2s_obj[i2s_num]->bits_per_sample,
                            p_i2s_obj[i2s_num]->channel_num);
}
#endif

static esp_err_t i2s_custom_check_cfg_static(i2s_port_t i2s_num,
                                             const i2s_config_t *cfg) {
#if SOC_I2S_SUPPORTS_PDM
  // We only check if the I2S number is invalid when set to PDM mode.
  I2S_CHECK(!((cfg->mode & I2S_MODE_PDM) && (i2s_num != I2S_NUM_0)),
            "I2S DAC PDM only support on I2S0", ESP_ERR_INVALID_ARG);
  return ESP_OK;
#endif

  I2S_CHECK(cfg->communication_format &&
                (cfg->communication_format < I2S_COMM_FORMAT_STAND_MAX),
            "invalid communication formats", ESP_ERR_INVALID_ARG);
  I2S_CHECK(!((cfg->communication_format & I2S_COMM_FORMAT_STAND_MSB) &&
              (cfg->communication_format & I2S_COMM_FORMAT_STAND_PCM_LONG)),
            "multiple communication formats specified", ESP_ERR_INVALID_ARG);
  return ESP_OK;
}

static esp_err_t i2s_param_config(i2s_port_t i2s_num,
                                  const i2s_config_t *i2s_config) {
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  I2S_CHECK((i2s_config), "param null", ESP_ERR_INVALID_ARG);
  I2S_CHECK((i2s_custom_check_cfg_static(i2s_num, i2s_config) == ESP_OK),
            "param check error", ESP_ERR_INVALID_ARG);

  periph_module_enable(i2s_periph_signal[i2s_num].module);

  // configure I2S data port interface.
  i2s_hal_config_param(&(p_i2s_obj[i2s_num]->hal), i2s_config);
  if ((p_i2s_obj[i2s_num]->mode & I2S_MODE_RX) &&
      (p_i2s_obj[i2s_num]->mode & I2S_MODE_TX)) {
    i2s_hal_enable_sig_loopback(&(p_i2s_obj[i2s_num]->hal));
    if (p_i2s_obj[i2s_num]->mode & I2S_MODE_MASTER) {
      i2s_hal_enable_master_mode(&(p_i2s_obj[i2s_num]->hal));
    } else {
      i2s_hal_enable_slave_mode(&(p_i2s_obj[i2s_num]->hal));
    }
  }

  p_i2s_obj[i2s_num]->use_apll = i2s_config->use_apll;
  p_i2s_obj[i2s_num]->tx_desc_auto_clear = i2s_config->tx_desc_auto_clear;
  p_i2s_obj[i2s_num]->fixed_mclk = i2s_config->fixed_mclk;
  return ESP_OK;
}

esp_err_t i2s_custom_zero_dma_buffer(i2s_port_t i2s_num) {
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  if (p_i2s_obj[i2s_num]->rx && p_i2s_obj[i2s_num]->rx->buf != NULL &&
      p_i2s_obj[i2s_num]->rx->buf_size != 0) {
    for (int i = 0; i < p_i2s_obj[i2s_num]->dma_buf_count; i++) {
      memset(p_i2s_obj[i2s_num]->rx->buf[i], 0,
             p_i2s_obj[i2s_num]->rx->buf_size);
    }
  }
  if (p_i2s_obj[i2s_num]->tx && p_i2s_obj[i2s_num]->tx->buf != NULL &&
      p_i2s_obj[i2s_num]->tx->buf_size != 0) {
    int bytes_left = 0;
    bytes_left =
        (p_i2s_obj[i2s_num]->tx->buf_size - p_i2s_obj[i2s_num]->tx->rw_pos) % 4;
    if (bytes_left) {
      size_t zero_bytes = 0, bytes_written;
      i2s_custom_write(i2s_num, (void *)&zero_bytes, bytes_left, &bytes_written,
                       portMAX_DELAY);
    }
    for (int i = 0; i < p_i2s_obj[i2s_num]->dma_buf_count; i++) {
      memset(p_i2s_obj[i2s_num]->tx->buf[i], 0,
             p_i2s_obj[i2s_num]->tx->buf_size);
    }
  }
  return ESP_OK;
}

esp_err_t i2s_custom_driver_install(i2s_port_t i2s_num,
                                    const i2s_config_t *i2s_config,
                                    int queue_size, void *i2s_queue) {
  esp_err_t err;
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  I2S_CHECK((i2s_config != NULL), "I2S configuration must not NULL",
            ESP_ERR_INVALID_ARG);
  I2S_CHECK(
      (i2s_config->dma_buf_count >= 2 && i2s_config->dma_buf_count <= 128),
      "I2S buffer count less than 128 and more than 2", ESP_ERR_INVALID_ARG);
  I2S_CHECK((i2s_config->dma_buf_len >= 8 && i2s_config->dma_buf_len <= 1024),
            "I2S buffer length at most 1024 and more than 8",
            ESP_ERR_INVALID_ARG);
  if (p_i2s_obj[i2s_num] == NULL) {
    p_i2s_obj[i2s_num] = (i2s_obj_t *)malloc(sizeof(i2s_obj_t));
    if (p_i2s_obj[i2s_num] == NULL) {
      ESP_LOGE(I2S_TAG, "Malloc I2S driver error");
      return ESP_ERR_NO_MEM;
    }
    memset(p_i2s_obj[i2s_num], 0, sizeof(i2s_obj_t));

    portMUX_TYPE i2s_spinlock_unlocked[1] = {portMUX_INITIALIZER_UNLOCKED};
    for (int x = 0; x < I2S_NUM_MAX; x++) {
      i2s_spinlock[x] = i2s_spinlock_unlocked[0];
    }
    // To make sure hardware is enabled before any hardware register
    // operations.
    periph_module_enable(i2s_periph_signal[i2s_num].module);
    i2s_hal_init(&(p_i2s_obj[i2s_num]->hal), i2s_num);

    p_i2s_obj[i2s_num]->i2s_num = i2s_num;
    p_i2s_obj[i2s_num]->dma_buf_count = i2s_config->dma_buf_count;
    p_i2s_obj[i2s_num]->dma_buf_len = i2s_config->dma_buf_len;
    p_i2s_obj[i2s_num]->i2s_queue = i2s_queue;
    p_i2s_obj[i2s_num]->mode = i2s_config->mode;

    p_i2s_obj[i2s_num]->bits_per_sample = 0;
    p_i2s_obj[i2s_num]->bytes_per_sample = 0;  // Not initialized yet
    p_i2s_obj[i2s_num]->channel_num =
        i2s_config->channel_format < I2S_CHANNEL_FMT_ONLY_RIGHT ? 2 : 1;

#ifdef CONFIG_PM_ENABLE
    if (i2s_config->use_apll) {
      err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "i2s_driver",
                               &p_i2s_obj[i2s_num]->pm_lock);
    } else {
      err = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "i2s_driver",
                               &p_i2s_obj[i2s_num]->pm_lock);
    }
    if (err != ESP_OK) {
      free(p_i2s_obj[i2s_num]);
      p_i2s_obj[i2s_num] = NULL;
      ESP_LOGE(I2S_TAG, "I2S pm lock error");
      return err;
    }
#endif  // CONFIG_PM_ENABLE

    // initial interrupt
    err = i2s_isr_register(i2s_num, i2s_config->intr_alloc_flags,
                           i2s_intr_handler_default, p_i2s_obj[i2s_num],
                           &p_i2s_obj[i2s_num]->i2s_isr_handle);
    if (err != ESP_OK) {
#ifdef CONFIG_PM_ENABLE
      if (p_i2s_obj[i2s_num]->pm_lock) {
        esp_pm_lock_delete(p_i2s_obj[i2s_num]->pm_lock);
      }
#endif
      free(p_i2s_obj[i2s_num]);
      p_i2s_obj[i2s_num] = NULL;
      ESP_LOGE(I2S_TAG, "Register I2S Interrupt error");
      return err;
    }
    i2s_custom_stop(i2s_num);
    err = i2s_param_config(i2s_num, i2s_config);
    if (err != ESP_OK) {
      i2s_custom_driver_uninstall(i2s_num);
      ESP_LOGE(I2S_TAG, "I2S param configure error");
      return err;
    }

    if (i2s_queue) {
      p_i2s_obj[i2s_num]->i2s_queue =
          xQueueCreate(queue_size, sizeof(i2s_event_t));
      *((QueueHandle_t *)i2s_queue) = p_i2s_obj[i2s_num]->i2s_queue;
      ESP_LOGI(I2S_TAG, "queue free spaces: %d",
               uxQueueSpacesAvailable(p_i2s_obj[i2s_num]->i2s_queue));
    } else {
      p_i2s_obj[i2s_num]->i2s_queue = NULL;
    }
    // set clock and start
    return i2s_custom_set_clk(i2s_num, i2s_config->sample_rate,
                              i2s_config->bits_per_sample,
                              p_i2s_obj[i2s_num]->channel_num);
  }

  ESP_LOGW(I2S_TAG, "I2S driver already installed");
  return ESP_OK;
}

esp_err_t i2s_custom_driver_uninstall(i2s_port_t i2s_num) {
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  if (p_i2s_obj[i2s_num] == NULL) {
    ESP_LOGI(I2S_TAG, "already uninstalled");
    return ESP_OK;
  }
  i2s_custom_stop(i2s_num);
  esp_intr_free(p_i2s_obj[i2s_num]->i2s_isr_handle);

  if (p_i2s_obj[i2s_num]->tx != NULL &&
      (p_i2s_obj[i2s_num]->mode & I2S_MODE_TX)) {
    i2s_destroy_dma_queue(i2s_num, p_i2s_obj[i2s_num]->tx);
    p_i2s_obj[i2s_num]->tx = NULL;
  }
  if (p_i2s_obj[i2s_num]->rx != NULL &&
      (p_i2s_obj[i2s_num]->mode & I2S_MODE_RX)) {
    i2s_destroy_dma_queue(i2s_num, p_i2s_obj[i2s_num]->rx);
    p_i2s_obj[i2s_num]->rx = NULL;
  }

  if (p_i2s_obj[i2s_num]->i2s_queue) {
    vQueueDelete(p_i2s_obj[i2s_num]->i2s_queue);
    p_i2s_obj[i2s_num]->i2s_queue = NULL;
  }

  if (p_i2s_obj[i2s_num]->use_apll) {
    rtc_clk_apll_enable(0, 0, 0, 0, 0);
  }
#ifdef CONFIG_PM_ENABLE
  if (p_i2s_obj[i2s_num]->pm_lock) {
    esp_pm_lock_delete(p_i2s_obj[i2s_num]->pm_lock);
  }
#endif

  free(p_i2s_obj[i2s_num]);
  p_i2s_obj[i2s_num] = NULL;
  periph_module_disable(i2s_periph_signal[i2s_num].module);

  return ESP_OK;
}

esp_err_t i2s_custom_write(i2s_port_t i2s_num, const void *src, size_t size,
                           size_t *bytes_written, TickType_t ticks_to_wait) {
  char *data_ptr, *src_byte;
  size_t bytes_can_write;
  *bytes_written = 0;
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  I2S_CHECK((size < SOC_I2S_MAX_BUFFER_SIZE), "size is too large",
            ESP_ERR_INVALID_ARG);
  I2S_CHECK((p_i2s_obj[i2s_num]->tx), "tx NULL", ESP_ERR_INVALID_ARG);
  xSemaphoreTake(p_i2s_obj[i2s_num]->tx->mux, (portTickType)portMAX_DELAY);
#ifdef CONFIG_PM_ENABLE
  esp_pm_lock_acquire(p_i2s_obj[i2s_num]->pm_lock);
#endif
  src_byte = (char *)src;
  while (size > 0) {
    //        for (int i = 0; i < p_i2s_obj[i2s_num]->dma_buf_count; i++) {
    //        	ESP_LOGI(I2S_TAG,"%d: EOF %d",  i,
    //        p_i2s_obj[i2s_num]->tx->desc[i]->eof);
    //        }

    if (p_i2s_obj[i2s_num]->tx->rw_pos == p_i2s_obj[i2s_num]->tx->buf_size ||
        p_i2s_obj[i2s_num]->tx->curr_ptr == NULL) {
      if (xQueueReceive(p_i2s_obj[i2s_num]->tx->queue,
                        &p_i2s_obj[i2s_num]->tx->curr_ptr,
                        ticks_to_wait) == pdFALSE) {
        break;
      }
      p_i2s_obj[i2s_num]->tx->rw_pos = 0;
    }
    ESP_LOGD(I2S_TAG, "size: %d, rw_pos: %d, buf_size: %d, curr_ptr: %d", size,
             p_i2s_obj[i2s_num]->tx->rw_pos, p_i2s_obj[i2s_num]->tx->buf_size,
             (int)p_i2s_obj[i2s_num]->tx->curr_ptr);
    data_ptr = (char *)p_i2s_obj[i2s_num]->tx->curr_ptr;
    data_ptr += p_i2s_obj[i2s_num]->tx->rw_pos;
    bytes_can_write =
        p_i2s_obj[i2s_num]->tx->buf_size - p_i2s_obj[i2s_num]->tx->rw_pos;
    if (bytes_can_write > size) {
      bytes_can_write = size;
    }
    memcpy(data_ptr, src_byte, bytes_can_write);
    size -= bytes_can_write;
    src_byte += bytes_can_write;
    p_i2s_obj[i2s_num]->tx->rw_pos += bytes_can_write;
    (*bytes_written) += bytes_can_write;
  }
#ifdef CONFIG_PM_ENABLE
  esp_pm_lock_release(p_i2s_obj[i2s_num]->pm_lock);
#endif

  xSemaphoreGive(p_i2s_obj[i2s_num]->tx->mux);
  return ESP_OK;
}

esp_err_t i2s_custom_write_expand(i2s_port_t i2s_num, const void *src,
                                  size_t size, size_t src_bits, size_t aim_bits,
                                  size_t *bytes_written,
                                  TickType_t ticks_to_wait) {
  char *data_ptr;
  int bytes_can_write, tail;
  int src_bytes, aim_bytes, zero_bytes;
  *bytes_written = 0;
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  I2S_CHECK((size > 0), "size must greater than zero", ESP_ERR_INVALID_ARG);
  I2S_CHECK((aim_bits * size < SOC_I2S_MAX_BUFFER_SIZE), "size is too large",
            ESP_ERR_INVALID_ARG);
  I2S_CHECK((aim_bits >= src_bits), "aim_bits mustn't be less than src_bits",
            ESP_ERR_INVALID_ARG);
  I2S_CHECK((p_i2s_obj[i2s_num]->tx), "tx NULL", ESP_ERR_INVALID_ARG);
  if (src_bits < I2S_BITS_PER_SAMPLE_8BIT ||
      aim_bits < I2S_BITS_PER_SAMPLE_8BIT) {
    ESP_LOGE(I2S_TAG, "bits mustn't be less than 8, src_bits %d aim_bits %d",
             src_bits, aim_bits);
    return ESP_ERR_INVALID_ARG;
  }
  if (src_bits > I2S_BITS_PER_SAMPLE_32BIT ||
      aim_bits > I2S_BITS_PER_SAMPLE_32BIT) {
    ESP_LOGE(I2S_TAG,
             "bits mustn't be greater than 32, src_bits %d aim_bits %d",
             src_bits, aim_bits);
    return ESP_ERR_INVALID_ARG;
  }
  if ((src_bits == I2S_BITS_PER_SAMPLE_16BIT ||
       src_bits == I2S_BITS_PER_SAMPLE_32BIT) &&
      (size % 2 != 0)) {
    ESP_LOGE(I2S_TAG,
             "size must be a even number while src_bits is even, src_bits "
             "%d size %d",
             src_bits, size);
    return ESP_ERR_INVALID_ARG;
  }
  if (src_bits == I2S_BITS_PER_SAMPLE_24BIT && (size % 3 != 0)) {
    ESP_LOGE(I2S_TAG,
             "size must be a multiple of 3 while src_bits is 24, size %d",
             size);
    return ESP_ERR_INVALID_ARG;
  }

  src_bytes = src_bits / 8;
  aim_bytes = aim_bits / 8;
  zero_bytes = aim_bytes - src_bytes;
  xSemaphoreTake(p_i2s_obj[i2s_num]->tx->mux, (portTickType)portMAX_DELAY);
  size = size * aim_bytes / src_bytes;
  ESP_LOGD(I2S_TAG, "aim_bytes %d src_bytes %d size %d", aim_bytes, src_bytes,
           size);
  while (size > 0) {
    if (p_i2s_obj[i2s_num]->tx->rw_pos == p_i2s_obj[i2s_num]->tx->buf_size ||
        p_i2s_obj[i2s_num]->tx->curr_ptr == NULL) {
      if (xQueueReceive(p_i2s_obj[i2s_num]->tx->queue,
                        &p_i2s_obj[i2s_num]->tx->curr_ptr,
                        ticks_to_wait) == pdFALSE) {
        break;
      }
      p_i2s_obj[i2s_num]->tx->rw_pos = 0;
    }
    data_ptr = (char *)p_i2s_obj[i2s_num]->tx->curr_ptr;
    data_ptr += p_i2s_obj[i2s_num]->tx->rw_pos;
    bytes_can_write =
        p_i2s_obj[i2s_num]->tx->buf_size - p_i2s_obj[i2s_num]->tx->rw_pos;
    if (bytes_can_write > (int)size) {
      bytes_can_write = size;
    }
    tail = bytes_can_write % aim_bytes;
    bytes_can_write = bytes_can_write - tail;

    memset(data_ptr, 0, bytes_can_write);
    for (int j = 0; j < bytes_can_write; j += (aim_bytes - zero_bytes)) {
      j += zero_bytes;
      memcpy(&data_ptr[j], (const char *)(src + *bytes_written),
             aim_bytes - zero_bytes);
      (*bytes_written) += (aim_bytes - zero_bytes);
    }
    size -= bytes_can_write;
    p_i2s_obj[i2s_num]->tx->rw_pos += bytes_can_write;
  }
  xSemaphoreGive(p_i2s_obj[i2s_num]->tx->mux);
  return ESP_OK;
}

esp_err_t i2s_custom_read(i2s_port_t i2s_num, void *dest, size_t size,
                          size_t *bytes_read, TickType_t ticks_to_wait) {
  char *data_ptr, *dest_byte;
  int bytes_can_read;
  *bytes_read = 0;
  dest_byte = (char *)dest;
  I2S_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
  I2S_CHECK((size < SOC_I2S_MAX_BUFFER_SIZE), "size is too large",
            ESP_ERR_INVALID_ARG);
  I2S_CHECK((p_i2s_obj[i2s_num]->rx), "rx NULL", ESP_ERR_INVALID_ARG);
  xSemaphoreTake(p_i2s_obj[i2s_num]->rx->mux, (portTickType)portMAX_DELAY);
#ifdef CONFIG_PM_ENABLE
  esp_pm_lock_acquire(p_i2s_obj[i2s_num]->pm_lock);
#endif
  while (size > 0) {
    if (p_i2s_obj[i2s_num]->rx->rw_pos == p_i2s_obj[i2s_num]->rx->buf_size ||
        p_i2s_obj[i2s_num]->rx->curr_ptr == NULL) {
      if (xQueueReceive(p_i2s_obj[i2s_num]->rx->queue,
                        &p_i2s_obj[i2s_num]->rx->curr_ptr,
                        ticks_to_wait) == pdFALSE) {
        break;
      }
      p_i2s_obj[i2s_num]->rx->rw_pos = 0;
    }
    data_ptr = (char *)p_i2s_obj[i2s_num]->rx->curr_ptr;
    data_ptr += p_i2s_obj[i2s_num]->rx->rw_pos;
    bytes_can_read =
        p_i2s_obj[i2s_num]->rx->buf_size - p_i2s_obj[i2s_num]->rx->rw_pos;
    if (bytes_can_read > (int)size) {
      bytes_can_read = size;
    }
    memcpy(dest_byte, data_ptr, bytes_can_read);
    size -= bytes_can_read;
    dest_byte += bytes_can_read;
    p_i2s_obj[i2s_num]->rx->rw_pos += bytes_can_read;
    (*bytes_read) += bytes_can_read;
  }
#ifdef CONFIG_PM_ENABLE
  esp_pm_lock_release(p_i2s_obj[i2s_num]->pm_lock);
#endif
  xSemaphoreGive(p_i2s_obj[i2s_num]->rx->mux);
  return ESP_OK;
}
