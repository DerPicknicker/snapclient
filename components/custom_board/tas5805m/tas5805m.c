/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2020 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in
 * which case, it is free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the
 * Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include "tas5805m.h"
#include "board.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "tas5805m_reg_cfg.h"

static const char *TAG = "TAS5805M";

#define TAS5805M_ADDR 0x5c
// #define TAS5805M_RST_GPIO get_pa_enable_gpio ()
#define TAS5805M_VOLUME_MAX 254
#define TAS5805M_MUTE 255
#define TAS5805M_VOLUME_MIN 0

static uint8_t VolumeLevel = 0;

// Default I2C config


/*
 * Operate fuction of PA
 */
audio_hal_func_t AUDIO_CODEC_TAS5805M_DEFAULT_HANDLE = {
    .audio_codec_initialize = tas5805m_init,
    .audio_codec_deinitialize = tas5805m_deinit,
    .audio_codec_ctrl = tas5805m_ctrl,
    .audio_codec_config_iface = tas5805m_config_iface,
    .audio_codec_set_mute = tas5805m_set_mute,
    .audio_codec_set_volume = tas5805m_set_volume,
    .audio_codec_get_volume = tas5805m_get_volume,
    .audio_hal_lock = NULL,
    .handle = NULL,
};

static i2c_config_t i2c_cfg = { 
    .mode = I2C_MODE_MASTER,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
};

void i2c_master_init() {
  int i2c_master_port = I2C_MASTER_NUM;

  get_i2c_pins(I2C_NUM_0, &i2c_cfg);

  esp_err_t res = i2c_param_config(i2c_master_port, &i2c_cfg);
  printf("Driver param setup : %d\n", res);
  res = i2c_driver_install(i2c_master_port, i2c_cfg.mode,
                           I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
                           0);
  printf("Driver installed   : %d\n", res);
}

esp_err_t tas5805m_read_byte(uint8_t register_name, uint8_t *data)
{

  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, TAS5805M_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, register_name, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_TAS5805M_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK)
  {
    ESP_LOGW(TAG, "I2C ERROR");
  }

  vTaskDelay(1 / portTICK_RATE_MS);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, TAS5805M_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_TAS5805M_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t tas5805m_write_byte(uint8_t register_name, uint8_t value)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, TAS5805M_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, register_name, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_TAS5805M_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t tas5805m_init()
{
  int ret;
  // Init the I2C-Driver 
  i2c_master_init();
  /* Register the PDN pin as output and write 1 to enable the TAS chip */
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = TAS5805M_GPIO_PDN_MASK;

  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  // log_d("Power down pin: %d", TAS5805M_GPIO_PDN);
  gpio_config(&io_conf);
  gpio_set_level(TAS5805M_GPIO_PDN, 0);
  vTaskDelay(200 / portTICK_RATE_MS);
  gpio_set_level(TAS5805M_GPIO_PDN, 1);
  vTaskDelay(200 / portTICK_RATE_MS);

  /* set PDN to 1 */
  gpio_set_level(TAS5805M_GPIO_PDN, 1);
  vTaskDelay(100 / portTICK_RATE_MS);

  // LOGI(TAG,"Setting to HI Z");
  ret = tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, 0x02);
  vTaskDelay(100 / portTICK_RATE_MS);
  if (ret != ESP_OK)
    return ret;

  // LOG_I(TAG,"Setting to PLAY");
  ret = tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, 0x03);
  if (ret != ESP_OK)
    return ret;

// Check if Bridge-Mode is enabled
#ifdef CONFIG_DAC_OPERATION_MODE
  uint8_t value = 0;
  ret = tas5805m_read_byte(TAS5805M_DEVICE_CTRL_1_REGISTER, &value);
  if (ret != ESP_OK)
    return ret;
  value = 0b100;
  // log_i("Setting to MONO mode, CTRL_1 = %d", value);
  ret = tas5805m_write_byte(TAS5805M_DEVICE_CTRL_1_REGISTER, value);
  if (ret != ESP_OK)
    return ret;
#endif

  vTaskDelay(100 / portTICK_RATE_MS);
  return ret;
}

esp_err_t
tas5805m_set_volume(int vol)
{

  uint8_t volume = vol;
  if (volume > TAS5805M_VOLUME_MAX)
  { // Check if Volume is not bigger than 254 if yes, set the Volume to 254
    return tas5805m_write_byte(TAS5805M_DIG_VOL_CTRL_REGISTER, volume);
  }
  else
  {
    return tas5805m_write_byte(TAS5805M_DIG_VOL_CTRL_REGISTER, volume);
  }
}

esp_err_t tas5805m_get_volume()
{
  int ret = tas5805m_read_byte(TAS5805M_DIG_VOL_CTRL_REGISTER, VolumeLevel);
  if (ret != ESP_OK)
  {
    ESP_LOGW(TAG, "Cant get Volume.");
    return ret;
  }
  return ret;
}

esp_err_t
tas5805m_set_mute(bool enable)
{

  if (VolumeLevel != TAS5805M_MUTE)
  {
    int ret = tas5805m_set_volume(TAS5805M_MUTE); // Set Volume to 255 for Mute
    return ret;
  }
  else
  {
    return ESP_OK;
  }
}

esp_err_t tas5805m_deinit(void)
{
  // TODO
  return ESP_OK;
}

esp_err_t
tas5805m_get_mute(bool *enabled)
{
  if (tas5805m_get_volume() == TAS5805M_MUTE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

esp_err_t tas5805m_ctrl(audio_hal_codec_mode_t mode,
                        audio_hal_ctrl_t ctrl_state)
{
  // TODO
  return ESP_OK;
}

esp_err_t tas5805m_config_iface(audio_hal_codec_mode_t mode,
                                audio_hal_codec_i2s_iface_t *iface)
{
  // TODO
  return ESP_OK;
}
