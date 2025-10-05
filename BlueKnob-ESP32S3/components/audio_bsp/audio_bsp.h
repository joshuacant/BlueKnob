#ifndef AUDIO_BSP_H
#define AUDIO_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

void audio_bsp_init(void);
int volume_adjustment(uint8_t vol);

#ifdef __cplusplus
}
#endif

#endif
