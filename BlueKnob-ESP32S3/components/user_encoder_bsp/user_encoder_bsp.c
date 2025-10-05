#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "user_encoder_bsp.h"
#include "settings.h"
#include "bidi_switch_knob.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "encoder";
EventGroupHandle_t knob_even_ = NULL;
static knob_handle_t s_knob = 0;

static void _knob_left_cb(void *arg, void *data) {
  uint8_t eventBits_ = 0;
  SET_BIT(eventBits_, 0);
  xEventGroupSetBits(knob_even_, eventBits_);
}

static void _knob_right_cb(void *arg, void *data) {
  uint8_t eventBits_ = 0;
  SET_BIT(eventBits_, 1);
  xEventGroupSetBits(knob_even_, eventBits_);
}

void user_encoder_init(void) {
  knob_even_ = xEventGroupCreate();
  // create knob
  knob_config_t cfg =
      {
          .gpio_encoder_a = ENCODER_ECA_PIN,
          .gpio_encoder_b = ENCODER_ECB_PIN,
      };
  s_knob = iot_knob_create(&cfg);
  if (NULL == s_knob) {
    ESP_LOGE(TAG, "knob create failed");
  }
  ESP_ERROR_CHECK(iot_knob_register_cb(s_knob, KNOB_LEFT, _knob_left_cb, NULL));
  ESP_ERROR_CHECK(iot_knob_register_cb(s_knob, KNOB_RIGHT, _knob_right_cb, NULL));
}