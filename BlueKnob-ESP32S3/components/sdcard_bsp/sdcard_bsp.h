#ifndef SDCARD_BSP_H
#define SDCARD_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

void sdcard_init(void);
void *fs_open(struct _lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode);
lv_fs_res_t fs_close(struct _lv_fs_drv_t *drv, void *file_p);
lv_fs_res_t fs_read(struct _lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br);
lv_fs_res_t fs_seek(struct _lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence);
lv_fs_res_t fs_tell(struct _lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p);

#ifdef __cplusplus
}
#endif

#endif