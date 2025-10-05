idf_component_register(
  SRCS "sdcard_bsp.c"
  PRIV_REQUIRES fatfs sdmmc main lvgl
  INCLUDE_DIRS "./")
#REQUIRES fatfs
#PRIV_REQUIRES