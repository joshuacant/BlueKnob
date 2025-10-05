idf_component_register(
  SRCS "lcd_touch_bsp.c"

  PRIV_REQUIRES
  i2c_bsp

  INCLUDE_DIRS "./")

##REQUIRES