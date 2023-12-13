/*
 * ANALOG tas5805m audio hal
 */

#ifndef _TAS5805M_H_
#define _TAS5805M_H_

#include "audio_hal.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize tas5805m codec chip
 */
esp_err_t tas5805m_init(audio_hal_codec_config_t *codec_cfg);

/**
 * Deinitialize tas5805m codec chip
 */
esp_err_t tas5805m_deinit(void);

/**
 * Set volume - NOT AVAILABLE
 */
esp_err_t tas5805m_set_volume(int vol);

/**
 * Get volume - NOT AVAILABLE
 */
esp_err_t tas5805m_get_volume(int *vol);

/**
 * Set tas5805m mute or not
 */
esp_err_t tas5805m_set_mute(bool enable);

/**
 * Get tas5805m mute status - NOT IMPLEMENTED
 */
esp_err_t tas5805m_get_mute(bool *enabled);

#ifdef __cplusplus
}
#endif

#endif  // _tas5805m_H_
