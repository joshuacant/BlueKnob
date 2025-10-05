idf_component_register(
  SRCS "src/bidi_switch_knob.c" "user_encoder_bsp.c"
  PRIV_REQUIRES main esp_driver_gpio esp_timer
  INCLUDE_DIRS "./" "./src")
