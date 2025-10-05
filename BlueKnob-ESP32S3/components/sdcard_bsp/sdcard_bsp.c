#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include "driver/sdmmc_host.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sdcard_bsp.h"
#include "settings.h"
#include "lvgl.h"
#include "ff.h"

sdmmc_card_t *card_host = NULL;
typedef FIL file_t;
typedef FF_DIR dir_t;

static float sdcard_get_value(void) {
  if (card_host != NULL) {
    return (float)(card_host->csd.capacity) / 2048 / 1024;
  } else
    return 0;
}

void sdcard_init(void) {
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024 * 3,
  };
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.width = 4;
  slot_config.clk = SDMMC_CLK_PIN;
  slot_config.cmd = SDMMC_CMD_PIN;
  slot_config.d0 = SDMMC_D0_PIN;
  slot_config.d1 = SDMMC_D1_PIN;
  slot_config.d2 = SDMMC_D2_PIN;
  slot_config.d3 = SDMMC_D3_PIN;
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card_host));
  if (card_host != NULL) {
    sdmmc_card_print_info(stdout, card_host);
    ESP_LOGI("SD Card", "Usable size: %.2fG\n", sdcard_get_value());
  }
}

// when using, src path must be in 'S:filename.ext' format
void *fs_open(struct _lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode) {
  LV_UNUSED(drv);
  // uint8_t flags = 0;
  // if (mode == LV_FS_MODE_WR)
  //   flags = FA_WRITE | FA_OPEN_ALWAYS;
  // else if (mode == LV_FS_MODE_RD)
  //   flags = FA_READ;
  // else if (mode == (LV_FS_MODE_WR | LV_FS_MODE_RD))
  //   flags = FA_READ | FA_WRITE | FA_OPEN_ALWAYS;
  FIL *f = (FIL *)lv_mem_alloc(sizeof(FIL));
  if (f == NULL) return NULL;
  FRESULT res = f_open(f, path, FA_READ);
  if (res == FR_OK) {
    return f;
  } else {
    ESP_LOGI("SD", "SD Card open failed");
    lv_mem_free(f);
    return NULL;
  }
}

lv_fs_res_t fs_close(struct _lv_fs_drv_t *drv, void *file_p) {
  LV_UNUSED(drv);
  f_close((FIL *)file_p);
  lv_mem_free(file_p);
  return LV_FS_RES_OK;
}

lv_fs_res_t fs_read(struct _lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br) {
  LV_UNUSED(drv);
  FRESULT res = f_read((FIL *)file_p, buf, btr, (UINT *)br);
  if (res == FR_OK)
    return LV_FS_RES_OK;
  else
    return LV_FS_RES_UNKNOWN;
}

lv_fs_res_t fs_seek(struct _lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence) {
  LV_UNUSED(drv);
  switch (whence) {
    case LV_FS_SEEK_SET:
      f_lseek((FIL *)file_p, pos);
      break;
    case LV_FS_SEEK_CUR:
      f_lseek((FIL *)file_p, f_tell((file_t *)file_p) + pos);
      break;
    case LV_FS_SEEK_END:
      f_lseek((FIL *)file_p, f_size((file_t *)file_p) + pos);
      break;
    default:
      break;
  }
  return LV_FS_RES_OK;
}

lv_fs_res_t fs_tell(struct _lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p) {
  LV_UNUSED(drv);
  *pos_p = f_tell((file_t *)file_p);
  return LV_FS_RES_OK;
}
