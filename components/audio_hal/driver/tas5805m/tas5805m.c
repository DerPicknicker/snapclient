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
#define TAS5805M_VOLUME_MIN 0

esp_err_t tas5805m_ctrl(audio_hal_codec_mode_t mode,
                        audio_hal_ctrl_t ctrl_state);
esp_err_t tas5805m_conig_iface(audio_hal_codec_mode_t mode,
                               audio_hal_codec_i2s_iface_t *iface);
static i2c_bus_handle_t i2c_handler;

/*
 * i2c default configuration
 */
static i2c_config_t i2c_cfg = {
    .mode = I2C_MODE_MASTER,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,
};

/*
 * Operate fuction of PA
 */
audio_hal_func_t AUDIO_CODEC_TAS5805M_DEFAULT_HANDLE = {
    .audio_codec_initialize = tas5805m_init,
    .audio_codec_deinitialize = tas5805m_deinit,
    .audio_codec_ctrl = tas5805m_ctrl,
    .audio_codec_config_iface = tas5805m_conig_iface,
    .audio_codec_set_mute = tas5805m_set_mute,
    .audio_codec_set_volume = tas5805m_set_volume,
    .audio_codec_get_volume = tas5805m_get_volume,
    .audio_hal_lock = NULL,
    .handle = NULL,
};


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
        log_e("I2C ERROR");
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
  esp_err_t ret;
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

esp_err_t
tas5805m_init(audio_hal_codec_config_t *codec_cfg)
{
  int test =1;
  return test;
}

esp_err_t
tas5805m_set_volume(int vol)
{
  uint8_t volume = vol;
  return tas5805m_write_byte(TAS5805M_DIG_VOL_CTRL_REGISTER, volume);
}

esp_err_t
tas5805m_get_volume(int *value)
{
  int test =1;
  return test;
}

esp_err_t
tas5805m_set_mute(bool enable)
{
  int test =1;
  return test;
}

esp_err_t
tas5805m_get_mute(int *value)
{
  int test =1;
  return test;
}
