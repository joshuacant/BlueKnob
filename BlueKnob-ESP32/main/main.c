#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

RTC_SLOW_ATTR static struct timeval sleep_enter_time;

static void deep_sleep_task(void *args) {
#if !SOC_RTC_FAST_MEM_SUPPORTED
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  nvs_handle_t nvs_handle;
  err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
    printf("Open NVS done\n");
  }

  // Get deep sleep enter time
  nvs_get_i32(nvs_handle, "slp_enter_sec", (int32_t *)&sleep_enter_time.tv_sec);
  nvs_get_i32(nvs_handle, "slp_enter_usec", (int32_t *)&sleep_enter_time.tv_usec);
#endif

  struct timeval now;
  gettimeofday(&now, NULL);
  int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

  switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_TIMER: {
      printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
      break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
      printf("Not a deep sleep reset\n");
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);

#if CONFIG_IDF_TARGET_ESP32
  // Isolate GPIO12 pin from external circuits. This is needed for modules
  // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
  // to minimize current consumption.
  rtc_gpio_isolate(GPIO_NUM_12);
#endif

  printf("Entering deep sleep\n");

  // get deep sleep enter time
  gettimeofday(&sleep_enter_time, NULL);

#if !SOC_RTC_FAST_MEM_SUPPORTED
  // record deep sleep enter time via nvs
  ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "slp_enter_sec", sleep_enter_time.tv_sec));
  ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "slp_enter_usec", sleep_enter_time.tv_usec));
  ESP_ERROR_CHECK(nvs_commit(nvs_handle));
  nvs_close(nvs_handle);
#endif

  // enter deep sleep
  esp_deep_sleep_start();
}

void app_main(void) {
  xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 6, NULL);
}