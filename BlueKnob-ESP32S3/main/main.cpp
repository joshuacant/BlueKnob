extern "C" {
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
// #include "driver/spi_master.h"
// #include "driver/sdmmc_host.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
// #include "esp_chip_info.h"
// #include "esp_check.h"
#include "esp_sleep.h"
#include "esp_event.h"
// #include "esp_flash.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#include "esp_hidd_prf_api.h"
#include "esp_lcd_panel_ops.h"
// #include "esp_lcd_panel_vendor.h"
// #include "esp_lcd_sh8601.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_err.h"
// #include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_timer.h"
// #include "esp_vfs_fat.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "ui.h"
#include "hid_dev.h"
#include "i2c_bsp.h"
#include "lcd_bl_pwm_bsp.h"
#include "lcd_touch_bsp.h"
#include "lvgl.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "user_encoder_bsp.h"
// #include "esp_wifi_bsp.h"
// #include "sdcard_bsp.h"
// #include "sdmmc_cmd.h"
// #include "audio_bsp.h"
}
#include "nvs_handle.hpp"
#include "SensorDRV2605.hpp"
#include "esp_lcd_panel_io.h"
#include "constants.cpp"
#include "settings.h"


// Globals
static lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
static lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
static SemaphoreHandle_t lvgl_mux = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static uint8_t brightness = DEFAULT_BRIGHTNESS;
static int saved_brightness = brightness;
static bool theme = false;
static uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
static lv_obj_t *saved_screen;
static int64_t screenchange_timestamp = 0;
static int time_since_screenchange = 0;
static int64_t widget_timestamp = 0;
static int64_t inactive_widget_time = 0;
static bool bt_connected = false;
static uint16_t hid_conn_id = 0;
static int display_timeout = DISPLAY_TIMEOUT;
static int device_timeout = DEVICE_TIMEOUT;
static int device_state = 0;
static adc_oneshot_unit_handle_t handle = NULL;
static adc_cali_handle_t cali_handle = NULL;
static int mv_output = 1;
static float voltage = 0.00;
static int batt_percent = 50;
static int adc_raw_value = 0;
static bool plugged_in = false;
static uint8_t rollerDisplay_idx = 0;
static uint8_t rollerDevice_idx = 0;
static int64_t inactive_knob_time = 0;
static int64_t inactive_screen_time = 0;
static uint16_t tp_x;
static uint16_t tp_y;
static uint16_t prev_pointX = 0;
static uint16_t prev_pointY = 0;
static uint8_t win;
static lv_indev_state_t prev_state = LV_INDEV_STATE_RELEASED;
static int8_t deltaX = 0;
static int8_t deltaY = 0;
static bool press_active = false;
static int64_t press_start_time = 0;
static int press_duration = 0;
static int press_distance = 0;
static EventBits_t knob_bits;
static int knob_state = 0;
static int64_t knob_timestamp = 0;
static SensorDRV2605 haptic_motor;


// Declarations
static bool lvgl_lock(int timeout_ms);
static void lvgl_unlock(void);
static void taptic_feedback(void);
static void set_screen(lv_obj_t *screen);
static void print_board_info(void) {
  // esp_chip_info_t chip_info;
  // uint32_t flash_size;
  // esp_chip_info(&chip_info);
  // ESP_LOGI(DEVICE_NAME, "This is %s chip with %d CPU core(s), %s%s%s%s, ",
  //        CONFIG_IDF_TARGET,
  //        chip_info.cores,
  //        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
  //        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
  //        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
  //        (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");
  // unsigned major_rev = chip_info.revision / 100;
  // unsigned minor_rev = chip_info.revision % 100;
  // ESP_LOGI(DEVICE_NAME, "silicon revision v%d.%d, ", major_rev, minor_rev);
  // if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
  //   ESP_LOGE(DEVICE_NAME, "Get flash size failed");
  //   return;
  // }
  // ESP_LOGI(DEVICE_NAME, "%" PRIu32 "MB %s flash", flash_size / (uint32_t)(1024 * 1024),
  //        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
  // ESP_LOGI(DEVICE_NAME, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
}


// BLE Actions
static void send_normal_keypress(int mask, uint8_t key_name) {
  if (bt_connected) {
    uint8_t key = {key_name};
    esp_hidd_send_keyboard_value(hid_conn_id, mask, &key, 1);
    esp_hidd_send_keyboard_value(hid_conn_id, mask, &key, 0);
  } else {
    // ESP_LOGI(DEVICE_NAME, "Bluetooth not connected");
  }
}

static void send_consumer_keypress(int key_name) {
  if (bt_connected) {
    esp_hidd_send_consumer_value(hid_conn_id, key_name, true);
    esp_hidd_send_consumer_value(hid_conn_id, key_name, false);
  } else {
    // ESP_LOGI(DEVICE_NAME, "Bluetooth not connected");
  }
}

static void send_consumer_keyhold(int key_name) {
  if (bt_connected) {
    esp_hidd_send_consumer_value(hid_conn_id, key_name, true);
    int hold_time = TOUCH_LONG_PRESS_REPEAT - 2;
    vTaskDelay(hold_time / portTICK_PERIOD_MS);
    esp_hidd_send_consumer_value(hid_conn_id, key_name, false);
  } else {
    // ESP_LOGI(DEVICE_NAME, "Bluetooth not connected");
  }
}

static void send_mouse_status(lv_indev_data_t *data) {
  if (data->state == LV_INDEV_STATE_PRESSED) {
    if (prev_state == LV_INDEV_STATE_RELEASED) {
      press_start_time = esp_timer_get_time();
      press_distance = 0;
    }
    press_duration = (esp_timer_get_time() - press_start_time) / 1000;
    if ((prev_pointX == 0 && prev_pointY == 0) || inactive_screen_time > 48) {
      prev_pointX = data->point.x;
      prev_pointY = data->point.y;
    }
    if (press_duration > 1500 && press_distance < 10) {
      data->state = LV_INDEV_STATE_RELEASED;
      taptic_feedback();
      set_screen(ui_screenMediaControls);
    }
    deltaX = (prev_pointX - data->point.x);
    deltaY = (prev_pointY - data->point.y);
    press_distance = press_distance + abs(deltaX) + abs(deltaY);
    if (abs(deltaX) > 4) deltaX *= 1.5;
    if (abs(deltaY) > 4) deltaY *= 1.5;
    esp_hidd_send_mouse_value(hid_conn_id, 0, deltaX, deltaY);
    prev_pointX = data->point.x;
    prev_pointY = data->point.y;
    prev_state = data->state;
    // ESP_LOGI("Touch", "Move X %d and Y %d | Total duration %d | Total distance %d",
    //          deltaX, deltaY, press_duration, press_distance);
  }
  if (data->state == LV_INDEV_STATE_RELEASED && prev_state == LV_INDEV_STATE_PRESSED) {
    press_duration = (esp_timer_get_time() - press_start_time) / 1000;
    if (press_duration <= 64) {
      esp_hidd_send_mouse_value(hid_conn_id, (1 << 0), 0, 0);
      esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0);
    }
    prev_state = data->state;
    // ESP_LOGI("Touch", "Touch released | Total duration %d | Total distance %d",
    //           press_duration, press_distance);
  }
}


// NV Storage
static void nvs_init(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

static void save_nvs_blob(void) {
  nvs_handle_t my_handle;
  esp_err_t err;
  preference_data_t preference_data = {
    .id = 118,
    .brightness = brightness,
    .rollerDisplay_idx = rollerDisplay_idx,
    .rollerDevice_idx = rollerDevice_idx,
    .theme = theme,
    };
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(DEVICE_NAME, "Error (%s) opening NVS handle!", esp_err_to_name(err));
  }
  // ESP_LOGI(DEVICE_NAME, "Saving preference data blob...");
  err = nvs_set_blob(my_handle, "preference_data", &preference_data, sizeof(preference_data_t));
  if (err != ESP_OK) {
    ESP_LOGE(DEVICE_NAME, "Failed to write preference data blob!");
    nvs_close(my_handle);
  }
  err = nvs_commit(my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(DEVICE_NAME, "Failed to commit data");
  }
  nvs_close(my_handle);
}

static void read_nvs_blobs(void) {
  nvs_handle_t my_handle;
  esp_err_t err;
  err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(DEVICE_NAME, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    return;
  }
  // ESP_LOGI(DEVICE_NAME, "Reading preference data blob:");
  preference_data_t preference_data;
  size_t preference_data_size = sizeof(preference_data_t);
  err = nvs_get_blob(my_handle, "preference_data", &preference_data, &preference_data_size);
  if (err == ESP_OK) {
    brightness = preference_data.brightness;
    rollerDisplay_idx = preference_data.rollerDisplay_idx;
    rollerDevice_idx = preference_data.rollerDevice_idx;
    display_timeout = rollerDisplay_options[rollerDisplay_idx];
    device_timeout = rollerDevice_options[rollerDevice_idx];
    theme = preference_data.theme;
    // ESP_LOGI(DEVICE_NAME, "id: %d", preference_data.id);
    // ESP_LOGI(DEVICE_NAME, "brightness: %d", preference_data.brightness);
    // ESP_LOGI(DEVICE_NAME, "rollerDisplay_idx: %d", preference_data.rollerDisplay_idx);
    // ESP_LOGI(DEVICE_NAME, "rollerDevice_idx: %d", preference_data.rollerDevice_idx);
    // ESP_LOGI(DEVICE_NAME, "theme: %d", preference_data.theme);
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(DEVICE_NAME, "Preference data not found. Initalizing.");
    save_nvs_blob();
  }
  nvs_close(my_handle);
}


// SPIFFS Storage
static void spiffs_storage_init(void) {
  // // access images via "A:folder/filename.bin"
  // ESP_LOGI(DEVICE_NAME, "Initializing SPIFFS");
  // esp_vfs_spiffs_conf_t conf = {
  //   .base_path = SPIFFS_MOUNT_POINT,
  //   .partition_label = NULL,
  //   .max_files = 5,
  //   .format_if_mount_failed = false,
  // };
  // esp_err_t ret = esp_vfs_spiffs_register(&conf);

  // if (ret != ESP_OK) {
  //   if (ret == ESP_FAIL) {
  //     ESP_LOGE(DEVICE_NAME, "Failed to mount or format filesystem");
  //   } else if (ret == ESP_ERR_NOT_FOUND) {
  //     ESP_LOGE(DEVICE_NAME, "Failed to find SPIFFS partition");
  //   } else {
  //     ESP_LOGE(DEVICE_NAME, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
  //   }
  //   return;
  // }
  // size_t total = 0, used = 0;
  // ret = esp_spiffs_info(NULL, &total, &used);
  // if (ret != ESP_OK) {
  //   ESP_LOGE(DEVICE_NAME, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
  // } else {
  //   ESP_LOGI(DEVICE_NAME, "Partition size: total: %d, used: %d", total, used);
  // }
}


// SD Card Storage
static void sdcard_storage_init(void) {
  // access images via "B:folder/filename.bin"
  // sdcard_init();
}


// Haptic Motor
bool i2c_dev_Callback(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len, bool writeReg, bool isWrite) {
  uint8_t ret;
  int areg = reg;
  i2c_master_dev_handle_t i2c_dev_handle = NULL;
  if (addr == DRV2605_ADDR)
    i2c_dev_handle = drv2605_dev_handle;
  if (isWrite) {
    if (writeReg) {
      ret = i2c_write_buff(i2c_dev_handle, areg, buf, len);
    } else {
      ret = i2c_write_buff(i2c_dev_handle, -1, buf, len);
    }
  } else {
    if (writeReg) {
      ret = i2c_read_buff(i2c_dev_handle, areg, buf, len);
    } else {
      ret = i2c_read_buff(i2c_dev_handle, -1, buf, len);
    }
  }
  return (ret == ESP_OK) ? true : false;
}

static void haptic_init(void) {
    if (!haptic_motor.begin(i2c_dev_Callback)) {
    ESP_LOGE(DEVICE_NAME, "drv2605 init failed");
  } else {
    haptic_motor.selectLibrary(5);
    haptic_motor.setMode(SensorDRV2605::MODE_INTTRIG);
  }
}

static void taptic_feedback(void) {
  haptic_motor.setWaveform(0, 1);
  haptic_motor.setWaveform(1, 0);
  haptic_motor.run();
}


// Rotary Encoder
static void knob_task(void *arg) {
  for (;;) {
    knob_bits = xEventGroupWaitBits(knob_even_, BIT_EVEN_ALL, pdTRUE, pdFALSE, pdMS_TO_TICKS(5000));
    if (READ_BIT(knob_bits, 1)) {
        knob_state = press_active ? 3 : 1;
    } else if (READ_BIT(knob_bits, 0)) {
        knob_state = press_active ? 4 : 2;
    } else {
        knob_state = 0;
    }
    switch (knob_state) {
      case 1:
        // ESP_LOGI(DEVICE_NAME, "knob turned right %d", press_active);
        send_consumer_keypress(HID_CONSUMER_VOLUME_UP);
        knob_timestamp = esp_timer_get_time();
        break;
      case 2:
        // ESP_LOGI(DEVICE_NAME, "knob turned left %d", press_active);
        send_consumer_keypress(HID_CONSUMER_VOLUME_DOWN);
        knob_timestamp = esp_timer_get_time();
        break;
      case 3:
        // ESP_LOGI(DEVICE_NAME, "knob turned right %d", press_active);
        send_consumer_keypress(HID_CONSUMER_FAST_FORWARD);
        knob_timestamp = esp_timer_get_time();
        break;
      case 4:
        // ESP_LOGI(DEVICE_NAME, "knob turned left %d", press_active);
        send_consumer_keypress(HID_CONSUMER_REWIND);
        knob_timestamp = esp_timer_get_time();
        break;
    }
  }
}

static void knob_trig_activity(void) {
  inactive_knob_time = 0;
}


// Bluetooth
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
    // first uuid, 16bit, [12],[13] is the value
};
static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x03C1,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};
static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr =
    //.peer_addr_type =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param) {
  switch (event) {
    case ESP_HIDD_EVENT_REG_FINISH: {
      if (param->init_finish.state == ESP_HIDD_INIT_OK) {
        // esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&hidd_adv_data);
      }
      break;
    }
    case ESP_BAT_EVENT_REG: {
      break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
      break;
    case ESP_HIDD_EVENT_BLE_CONNECT: {
      // ESP_LOGI(DEVICE_NAME, "ESP_HIDD_EVENT_BLE_CONNECT");
      hid_conn_id = param->connect.conn_id;
      break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT: {
      bt_connected = false;
      // ESP_LOGI(DEVICE_NAME, "ESP_HIDD_EVENT_BLE_DISCONNECT");
      esp_ble_gap_start_advertising(&hidd_adv_params);
      break;
    }
    case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
      // ESP_LOGI(DEVICE_NAME, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
      ESP_LOG_BUFFER_HEX(DEVICE_NAME, param->vendor_write.data, param->vendor_write.length);
      break;
    }
    case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
      // ESP_LOGI(DEVICE_NAME, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
      ESP_LOG_BUFFER_HEX(DEVICE_NAME, param->led_write.data, param->led_write.length);
      break;
    }
    default:
      break;
  }
  return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
      esp_ble_gap_start_advertising(&hidd_adv_params);
      break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
      for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
        ESP_LOGD(DEVICE_NAME, "%x:", param->ble_security.ble_req.bd_addr[i]);
      }
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      esp_bd_addr_t bd_addr;
      memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
      // ESP_LOGI(DEVICE_NAME, "remote BD_ADDR: %08x%04x",
      //          (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
      //          (bd_addr[4] << 8) + bd_addr[5]);
      // ESP_LOGI(DEVICE_NAME, "address type = %d", param->ble_security.auth_cmpl.addr_type);
      // ESP_LOGI(DEVICE_NAME, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
      if (param->ble_security.auth_cmpl.success) {
        bt_connected = true;
        // ESP_LOGI(DEVICE_NAME, "secure connection established.");
      } else {
        ESP_LOGE(DEVICE_NAME, "pairing failed, reason = 0x%x",
                 param->ble_security.auth_cmpl.fail_reason);
      }
      break;
    default:
      break;
  }
}

static void ble_init(void) {
  esp_err_t ret;
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(DEVICE_NAME, "%s initialize controller failed", __func__);
    return;
  }
  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(DEVICE_NAME, "%s enable controller failed", __func__);
    return;
  }
  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(DEVICE_NAME, "%s init bluedroid failed", __func__);
    return;
  }
  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(DEVICE_NAME, "%s init bluedroid failed", __func__);
    return;
  }
  if ((ret = esp_hidd_profile_init()) != ESP_OK) {
    ESP_LOGE(DEVICE_NAME, "%s init bluedroid failed", __func__);
  }
  esp_ble_gap_register_callback(gap_event_handler);
  esp_hidd_register_callbacks(hidd_event_callback);
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
  // esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9 );
}

static void ble_deinit(void) {
  esp_err_t ret;
  ret = esp_ble_gap_stop_advertising();
  if (ret) {
    ESP_LOGE(DEVICE_NAME, "%s failed", __func__);
    return;
  }
  ret = esp_bluedroid_disable();
  if (ret) {
    ESP_LOGE(DEVICE_NAME, "%s failed", __func__);
    return;
  }
  ret = esp_bluedroid_deinit();
  if (ret) {
    ESP_LOGE(DEVICE_NAME, "%s failed", __func__);
    return;
  }
  ret = esp_bt_controller_disable();
  if (ret) {
    ESP_LOGE(DEVICE_NAME, "%s failed", __func__);
    return;
  }
  ret = esp_bt_controller_deinit();
  if (ret) {
    ESP_LOGE(DEVICE_NAME, "%s failed", __func__);
    return;
  }
}


// Battery
static void adc_init(void) {
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      // .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .chan = ADC_CHANNEL_0,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &handle));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, ADC_CHANNEL_0, &config));
  ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));
}

static void read_adc(void* parameter) {
  // schematic says this is checking 5v but is labeled "BATT_ADC"
  // 5V - 10k - [BATT_ADC] - 10k - GND
  // likely coming out of the TLV62569DBVT buck dcdc chip?
  while (1) {
    ESP_ERROR_CHECK(adc_oneshot_read(handle, ADC_CHANNEL_0, &adc_raw_value));
    adc_cali_raw_to_voltage(cali_handle, adc_raw_value, &mv_output);
    voltage = 0.001 * mv_output * 2;
    plugged_in = (adc_raw_value > BATT_MAX);
    double adc_adjusted_value = adc_raw_value - BATT_MIN;
    double batt_ratio = adc_adjusted_value / (BATT_MAX - BATT_MIN);
    batt_percent = (int) (batt_ratio * 100);
    if (batt_percent > 100) batt_percent = 100;
    // ESP_LOGI(DEVICE_NAME,"ADC Raw: %d, Voltage: %f",adc_raw_value,voltage);
    // ESP_LOGI(DEVICE_NAME, "batt_percent: %d", batt_lut[batt_percent]);
    // ESP_LOGI(DEVICE_NAME, "plugged_in: %d", plugged_in);
    vTaskDelay(pdMS_TO_TICKS(BATTERY_CHECK_TIMEOUT));
  }
}


// Display
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
  lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
  lv_disp_flush_ready(disp_driver);
  return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
  const int offsetx1 = area->x1;
  const int offsetx2 = area->x2;
  const int offsety1 = area->y1;
  const int offsety2 = area->y2;
  // copy a buffer's content to a specific area of the display
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

void lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area) {
  uint16_t x1 = area->x1;
  uint16_t x2 = area->x2;
  uint16_t y1 = area->y1;
  uint16_t y2 = area->y2;
  // round the start of coordinate down to the nearest 2M number
  area->x1 = (x1 >> 1) << 1;
  area->y1 = (y1 >> 1) << 1;
  // round the end of coordinate up to the nearest 2N+1 number
  area->x2 = ((x2 >> 1) << 1) + 1;
  area->y2 = ((y2 >> 1) << 1) + 1;
}

static void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  time_since_screenchange = (esp_timer_get_time() - screenchange_timestamp) / 1000;
  win = tpGetCoordinates(&tp_x, &tp_y);
  if (win && time_since_screenchange > 500) {
    data->point.x = tp_x;
    data->point.y = tp_y;
    if (data->point.x > LCD_H_RES)
      data->point.x = LCD_H_RES;
    if (data->point.y > LCD_V_RES)
      data->point.y = LCD_V_RES;
    data->state = LV_INDEV_STATE_PRESSED;
    press_active = true;
    // ESP_LOGI("Touch", "(%d,%d)", data->point.x, data->point.y);
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
    press_active = false;
  }
  if (lv_scr_act() == ui_screenTrackpad) send_mouse_status(data);
}

static void lvgl_increase_tick(void *arg) {
  lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static bool lvgl_lock(int timeout_ms) {
  assert(lvgl_mux && "bsp_display_start must be called first");
  const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void lvgl_unlock(void) {
  assert(lvgl_mux && "bsp_display_start must be called first");
  xSemaphoreGive(lvgl_mux);
}

static void lvgl_port_task(void *arg) {
  // ESP_LOGI(DEVICE_NAME, "Starting LVGL task");
  while (1) {
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_lock(-1)) {
      task_delay_ms = lv_timer_handler();
      // Release the mutex
      lvgl_unlock();
    }
    if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
      task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
      task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
    }
    vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
  }
}

static void disp_init(void) {
  lcd_bl_pwm_bsp_init(0);
  ESP_LOGI(DEVICE_NAME, "Initialize SPI bus");
  const spi_bus_config_t buscfg = {
    .data0_io_num = PIN_NUM_LCD_DATA0,
    .data1_io_num = PIN_NUM_LCD_DATA1,
    .sclk_io_num = PIN_NUM_LCD_PCLK,
    .data2_io_num = PIN_NUM_LCD_DATA2,
    .data3_io_num = PIN_NUM_LCD_DATA3,
    .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
  };
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
  ESP_LOGI(DEVICE_NAME, "Install panel IO");
  esp_lcd_panel_io_handle_t io_handle = NULL;
  static lv_disp_drv_t disp_drv;
  const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(
    PIN_NUM_LCD_CS,
    notify_lvgl_flush_ready,
    &disp_drv);
  sh8601_vendor_config_t vendor_config = {
    .init_cmds = lcd_init_cmds,
    .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
    .flags = {
        .use_qspi_interface = 1,
    },
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
  const esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = PIN_NUM_LCD_RST,
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    .bits_per_pixel = LCD_BIT_PER_PIXEL,
    .vendor_config = &vendor_config,
  };
  ESP_LOGI(DEVICE_NAME, "Install SH8601 panel driver");
  ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  lcd_touch_init();
  ESP_LOGI(DEVICE_NAME, "Initialize LVGL library");
  lv_init();
  assert(buf1);
  assert(buf2);
  static lv_disp_draw_buf_t disp_buf;
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * LVGL_BUF_HEIGHT);
  ESP_LOGI(DEVICE_NAME, "Register display driver to LVGL");
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LCD_H_RES;
  disp_drv.ver_res = LCD_V_RES;
  disp_drv.flush_cb = lvgl_flush_cb;
  disp_drv.rounder_cb = lvgl_rounder_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
  ESP_LOGI(DEVICE_NAME, "Install LVGL tick timer");
  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &lvgl_increase_tick,
    .name = "lvgl_tick"};
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.disp = disp;
  indev_drv.long_press_time = TOUCH_LONG_PRESS;
  indev_drv.long_press_repeat_time = TOUCH_LONG_PRESS_REPEAT;
  indev_drv.read_cb = lvgl_touch_cb;
  lv_indev_drv_register(&indev_drv);
  lvgl_mux = xSemaphoreCreateMutex();
  assert(lvgl_mux);
  xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
  if (lvgl_lock(-1)) {
    ESP_LOGI(DEVICE_NAME, "Start UI");
    ui_init();
    lvgl_unlock();
  }
}

static void set_lcd_brightness(int new_brightness) {
  if (new_brightness < MIN_BRIGHTNESS) new_brightness = MIN_BRIGHTNESS;
  if (new_brightness > 255) new_brightness = 255;
  if (abs(brightness - new_brightness) > MIN_BRIGHTNESS_CHANGE) {
    setUpduty(new_brightness);
    // ESP_LOGI(DEVICE_NAME, "Set brightness to %d", new_brightness);
    brightness = new_brightness;
    save_nvs_blob();
  }
}

static void set_random_MediaControls_background(void) {
  #ifdef ENABLE_BACKGROUNDS
  int num_images = sizeof(images) / sizeof(images[0]);
  lv_obj_set_style_bg_img_src(ui_screenMediaControls, randomElement(images, num_images), LV_PART_MAIN | LV_STATE_DEFAULT);
  #endif
  // lv_obj_set_style_bg_img_src(ui_screenMediaControls, "B:assets/blurtest.bin", LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void set_Trackpad_background(void) {
  #ifdef ENABLE_BACKGROUNDS
  lv_obj_set_style_bg_img_src(ui_screenTrackpad, &ui_img_trackpad_png, LV_PART_MAIN | LV_STATE_DEFAULT);
  #endif
}

static void set_theme (void) {
  if (theme) {
    lv_obj_set_style_border_color(ui_buttonPlay, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_buttonNext, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_buttonPrev, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_labelPlay, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_labelNext, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_labelPrev, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else {
    lv_obj_set_style_border_color(ui_buttonPlay, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_buttonNext, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_buttonPrev, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_labelPlay, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_labelNext, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_labelPrev, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
}

static void set_screen(lv_obj_t *screen) {
  screenchange_timestamp = esp_timer_get_time();
  lv_scr_load(screen);
}


// Sleep
static void sleep_display(void) {
  // ESP_LOGI(DEVICE_NAME, "sleepdisplay %d", brightness);
  if (brightness != 0) {
    saved_brightness = brightness;
    brightness = 0;
    setUpduty(brightness);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, false));
    if (lvgl_lock(-1)) {
      saved_screen = lv_scr_act();
      set_screen(ui_screenBlank);
      lvgl_unlock();
    }
    ESP_LOGI(DEVICE_NAME, "Sleeping Display");
  }
}

static void wake_display(void) {
  // ESP_LOGI(DEVICE_NAME, "wakedisplay %d", brightness);
  if (brightness == 0) {
    if (lvgl_lock(-1)) {
      set_random_MediaControls_background();
      set_screen(saved_screen);
      lvgl_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(BACKLIGHT_DELAY));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    brightness = saved_brightness;
    setUpduty(brightness);
    ESP_LOGI(DEVICE_NAME, "Waking Display");
  }
}

static void wait_gpio_inactive(void) {
  // while (gpio_get_level((gpio_num_t)GPIO_WAKEUP_NUM) == GPIO_WAKEUP_LEVEL) {
  //   vTaskDelay(pdMS_TO_TICKS(10));
  // }
}

static void register_timer_wakeup(int uS) {
  esp_sleep_enable_timer_wakeup(uS);
  // ESP_LOGI(DEVICE_NAME, "timer wakeup source is ready");
}

static void register_gpio_wakeup(int gpio_pin) {
  gpio_config_t config = {
    .pin_bit_mask = BIT64(gpio_pin),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&config);
  gpio_num_t gpio_num = (gpio_num_t)gpio_pin;
  gpio_wakeup_enable(gpio_num, GPIO_WAKEUP_LEVEL == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  wait_gpio_inactive();
  // ESP_LOGI(DEVICE_NAME, "gpio wakeup source %d is ready", gpio_pin);
}

static void wakeups_init(void) {
  register_gpio_wakeup(GPIO_WAKEUP_KNOB1);
  register_gpio_wakeup(GPIO_WAKEUP_KNOB2);
  register_gpio_wakeup(GPIO_WAKEUP_TOUCH);
  register_timer_wakeup(DEVICE_DEEP_TIMEOUT_US);
}

static void sleep_device(void) {
  ble_deinit();
  ESP_LOGI(DEVICE_NAME, "Sleeping Device");
  vTaskDelay(pdMS_TO_TICKS(50));
  int64_t t_before_us = esp_timer_get_time();
  esp_light_sleep_start();

  // ===== IMPORTANT NOTE =====
  // USB UART impossible to recover after sleep. No logging
  // at all over USB is possible beyond this point.
  // To reset usb uart, deepsleep with a 1ms wake...
  // register_timer_wakeup(1000);
  // esp_deep_sleep_start();
  // ...or enable code below to wake normally but you will
  // not see any further ESP_LOG or print via USB UART.
  // ===== IMPORTANT NOTE =====

  int64_t t_after_us = esp_timer_get_time();
  ESP_LOGI(DEVICE_NAME, "Exiting light sleep");
  const char *wakeup_reason;
  switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_TIMER:
      wakeup_reason = "timer";
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      vTaskDelay(pdMS_TO_TICKS(100));
      esp_deep_sleep_start();
      break;
    case ESP_SLEEP_WAKEUP_GPIO:
      wakeup_reason = "pin";
      wait_gpio_inactive();
      break;
    default:
      wakeup_reason = "other";
      break;
  }
  // ESP_LOGI(DEVICE_NAME, "Returned from light sleep, reason: %s, t=%lld ms, slept for %lld ms\n",
  //   wakeup_reason, t_after_us / 1000, (t_after_us - t_before_us) / 1000);
  vTaskDelay(pdMS_TO_TICKS(50));
  ble_init();
  disp_init();
  if (lvgl_lock(-1)) {
    lv_disp_trig_activity(NULL);
    lvgl_unlock();
  }
  knob_trig_activity();
  if (brightness == 0) brightness = saved_brightness;
  setUpduty(brightness);
}


// UI Widgets
static void widgets_init(void) {
  if (lvgl_lock(-1)) {
    if (theme) {
      lv_obj_add_state(ui_switchTheme, LV_STATE_CHECKED);
    } else {
      lv_obj_clear_state(ui_switchTheme, LV_STATE_CHECKED);
    }
    set_theme();
    lv_arc_set_value(ui_brightnessArc, brightness);
    lv_roller_set_selected(ui_rollerDisplay, rollerDisplay_idx, LV_ANIM_OFF);
    lv_roller_set_selected(ui_rollerDevice, rollerDevice_idx, LV_ANIM_OFF);
    lv_bar_set_value(ui_barBattery, 0, LV_ANIM_OFF);
    set_random_MediaControls_background();
    set_Trackpad_background();
    lv_disp_trig_activity(NULL);
    lvgl_unlock();
  }
}

static void update_battery_value(void) {
  if (adc_raw_value > BATT_MAX) {
    lv_bar_set_value(ui_barBattery, 100, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(ui_barBattery, lv_color_hex(0xB9F110), LV_PART_INDICATOR | LV_STATE_DEFAULT);
  } else {
    lv_obj_set_style_bg_color(ui_barBattery, lv_color_hex(0x18A917), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_bar_set_value(ui_barBattery, batt_lut[batt_percent], LV_ANIM_OFF);
  }
  // on screen display of raw value, for debugging and calibration
  // lv_label_set_text_fmt(ui_labelVoltage, "%d", adc_raw_value);
}

static void update_bluetooth_value(void) {
  if (bt_connected) {
    lv_obj_set_style_text_color(ui_labelBluetooth, lv_color_hex(0x6CA2FA), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else {
    lv_obj_set_style_text_color(ui_labelBluetooth, lv_color_hex(0x4E4E4E), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
}

static void update_ui_widgets(void) {
  if (lvgl_lock(-1)) {
    update_battery_value();
    update_bluetooth_value();
    lvgl_unlock();
    widget_timestamp = esp_timer_get_time();
  }
}


// UI Events
void anyButton_onPressed(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "anyButton_onPressed");
  taptic_feedback();
}

void buttonPlay_onShortClicked(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonPlay_onShortClicked");
  send_consumer_keypress(HID_CONSUMER_PLAY);
}

void buttonPlay_onLongPressed(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonPlay_onLongPressed");
  taptic_feedback();
  send_consumer_keypress(HID_CONSUMER_POWER);
}

void buttonPrev_onShortClicked(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonPrev_onShortClicked");
  send_consumer_keypress(HID_CONSUMER_SCAN_PREV_TRK);
}

void buttonPrev_onLongPressed(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonPrev_onLongPressed");
  taptic_feedback();
  send_consumer_keyhold(HID_CONSUMER_REWIND);
}

void buttonNext_onShortClicked(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonNext_onShortClicked");
  send_consumer_keypress(HID_CONSUMER_SCAN_NEXT_TRK);
}

void buttonNext_onLongPressed(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonNext_onLongPressed");
  taptic_feedback();
  send_consumer_keyhold(HID_CONSUMER_FAST_FORWARD);
}

void brightnessArc_onValueChanged(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "brightnessArc_onValueChanged");
  lv_obj_t * arc = lv_event_get_target(e);
  int arc_value = lv_arc_get_value(arc);
  set_lcd_brightness(arc_value);
}

void switchTheme_onShortClicked(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "switchTheme_onShortClicked");
  theme = lv_obj_has_state(ui_switchTheme, LV_STATE_CHECKED);
  set_theme();
  save_nvs_blob();
}

void screenMediaControls_onGestureLeft(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "screenMediaControls_onGestureLeft");
  set_screen(ui_screenDpad);
}

void screenMediaControls_onGestureRight(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "screenMediaControls_onGestureRight");
  set_screen(ui_screenSettings);
}

void screenSettings_onGestureLeft(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "screenSettings_onGestureLeft");
  set_screen(ui_screenMediaControls);
}

void screenSettings_onGestureRight(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "screenSettings_onGestureLeft");
}

void screenDpad_onGestureLeft(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "screenDpad_onGestureLeft");
  set_screen(ui_screenTrackpad);
}

void screenDpad_onGestureRight(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "screenDpad_onGestureRight");
  set_screen(ui_screenMediaControls);
}

void rollerDisplay_onValueChanged(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "rollerDisplay_onValueChanged");
  rollerDisplay_idx = lv_roller_get_selected(ui_rollerDisplay);
  // ESP_LOGI(DEVICE_NAME, "rollerDisplay_options %d", rollerDisplay_options[rollerDisplay_idx]);
  display_timeout = rollerDisplay_options[rollerDisplay_idx];
  save_nvs_blob();
}

void rollerDevice_onValueChanged(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "rollerDevice_onValueChanged");
  rollerDevice_idx = lv_roller_get_selected(ui_rollerDevice);
  // ESP_LOGI(DEVICE_NAME, "rollerDevice_options %d", rollerDevice_options[rollerDevice_idx]);
  device_timeout = rollerDevice_options[rollerDevice_idx];
  save_nvs_blob();
}

void buttonDpadUp_onShortClicked(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonDpadUp_onShortClicked");
  send_normal_keypress(0, HID_KEY_UP_ARROW);
}

void buttonDpadDown_onShortClicked(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonDpadDown_onShortClicked");
  send_normal_keypress(0, HID_KEY_DOWN_ARROW);
}

void buttonDpadLeft_onShortClicked(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonDpadLeft_onShortClicked");
  send_normal_keypress(0, HID_KEY_LEFT_ARROW);
}

void buttonDpadRight_onShortClicked(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonDpadRight_onShortClicked");
  send_normal_keypress(0, HID_KEY_RIGHT_ARROW);
}

void buttonDpadEnter_onShortClicked(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonDpadEnter_onShortClicked");
  send_normal_keypress(0, HID_KEY_RETURN);
}

void buttonDpadBack_onShortClicked(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonDpadBack_onShortClicked");
  send_normal_keypress(LEFT_GUI_KEY_MASK, HID_KEY_DELETE);
}

void buttonDpadEnter_onLongPressed(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonDpadEnter_onLongPressed");
  taptic_feedback();
  send_normal_keypress(LEFT_GUI_KEY_MASK, HID_KEY_N);
}

void buttonDpadBack_onLongPressed(lv_event_t * e) {
  // ESP_LOGI(DEVICE_NAME, "buttonDpadBack_onLongPressed");
  send_normal_keypress(LEFT_GUI_KEY_MASK, HID_KEY_RETURN);
  taptic_feedback();
}


// Setup
static void app_setup(void) {
  // print board details
  // print_board_info();

  // audio [unused, untested]
  // audio_bsp_init();

  // NV storage
  nvs_init();

  // restore saved preferences
  read_nvs_blobs();

  // i2c for haptic motor and LCD touch
  i2c_master_Init();

  // haptic motor
  haptic_init();

  // rotary encoder
  user_encoder_init();
  xTaskCreate(knob_task, "knob_task", 3000, NULL, 2, NULL);

  // battery level
  adc_init();
  xTaskCreate(read_adc, "read_adc", 3000, NULL, 2, NULL);

  // wifi [unused, tested working]
  // espwifi_Init();

  // bluetooth
  ble_init();

  // light sleep wakeups
  wakeups_init();

  // display (includes: lcd panel, backlight, touch, and lvgl ui)
  disp_init();

  // sd storage (for images) [working, too slow]
  // sdcard_storage_init();

  // SPIFFS storage (for images) [working, even slower]
  // spiffs_storage_init();

  // ui widgets initial state setup
  widgets_init();

  // wait for everything to settle and then turn on display
  vTaskDelay(pdMS_TO_TICKS(BACKLIGHT_DELAY));
  setUpduty(brightness);
}


// Loop
extern "C" void app_main(void) {
  app_setup();

  // main thread loop
  while (1) {
    inactive_screen_time = lv_disp_get_inactive_time(NULL);
    inactive_knob_time = (esp_timer_get_time() - knob_timestamp) / 1000;
    inactive_widget_time = (esp_timer_get_time() - widget_timestamp) / 1000;
    device_state = 0;
    if (inactive_screen_time > display_timeout) {
      device_state = 1;
    }
    if ((inactive_knob_time > device_timeout) &&
        (inactive_screen_time > device_timeout) &&
        (plugged_in == false)) {
      device_state = 2;
    }
    // ESP_LOGI(DEVICE_NAME, "inactive_screen_time %d", inactive_screen_time);
    // ESP_LOGI(DEVICE_NAME, "inactive_knob_time %d", inactive_knob_time);
    // ESP_LOGI(DEVICE_NAME, "display_timeout %d", display_timeout);
    // ESP_LOGI(DEVICE_NAME, "device_timeout %d", device_timeout);
    // ESP_LOGI(DEVICE_NAME, "device_state %d", device_state);
    switch (device_state) {
      case 0:
        wake_display();
        if (inactive_widget_time > WIDGET_UPDATE_INTERVAL) {
          update_ui_widgets();
        }
        break;
      case 1:
        sleep_display();
        break;
      case 2:
        sleep_device();
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(MAIN_UPDATE_RATE));
  }
}
