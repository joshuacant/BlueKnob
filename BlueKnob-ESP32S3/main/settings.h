#define USER_CONFIG_H
#define DEVICE_NAME "Blue Knob"


// I2C
#define ESP32_SCL_NUM (GPIO_NUM_12)
#define ESP32_SDA_NUM (GPIO_NUM_11)


// I2S
#define I2S_STD_BCLK_PIN (gpio_num_t)39
#define I2S_STD_WS_PIN (gpio_num_t)40
#define I2S_STD_DOUT_PIN (gpio_num_t)41
#define I2S_PDM_DATA_PIN (gpio_num_t)46
#define I2S_PDM_CLK_PIN (gpio_num_t)45


// Rotary Encoder
#define ENCODER_ECA_PIN 8
#define ENCODER_ECB_PIN 7


// DRV2605 Haptic
#define DRV2605_ADDR 0x5a


// UI
#define ENABLE_BACKGROUNDS
#define MAIN_UPDATE_RATE 20
#define WIDGET_UPDATE_INTERVAL 250
#define MIN_BRIGHTNESS 5
#define MIN_BRIGHTNESS_CHANGE 10
#define DEFAULT_BRIGHTNESS 25
#define LVGL_BUF_HEIGHT (LCD_V_RES / 10)
#define LVGL_TICK_PERIOD_MS 2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 5
#define LVGL_TASK_STACK_SIZE (4 * 1024)
#define LVGL_TASK_PRIORITY 2


// LCD
#define LCD_H_RES 360
#define LCD_V_RES 360
#define LCD_HOST SPI2_HOST
#define LCD_BIT_PER_PIXEL (16)
#define PIN_NUM_LCD_CS (gpio_num_t)14
#define PIN_NUM_LCD_PCLK (gpio_num_t)13
#define PIN_NUM_LCD_DATA0 (gpio_num_t)15
#define PIN_NUM_LCD_DATA1 (gpio_num_t)16
#define PIN_NUM_LCD_DATA2 (gpio_num_t)17
#define PIN_NUM_LCD_DATA3 (gpio_num_t)18
#define PIN_NUM_LCD_RST (gpio_num_t)21
#define PIN_NUM_BK_LIGHT (gpio_num_t)47


// Touch
#define TOUCH_LONG_PRESS 500
#define TOUCH_LONG_PRESS_REPEAT 260
#define TOUCH_HOST I2C_NUM_0
#define TOUCH_ADDR 0x15
#define PIN_NUM_TOUCH_RST (gpio_num_t)10
#define PIN_NUM_TOUCH_INT (gpio_num_t)9


// Bit
#define SET_BIT(reg, bit) (reg |= ((uint32_t)0x01 << bit))
#define CLEAR_BIT(reg, bit) (reg &= (~((uint32_t)0x01 << bit)))
#define READ_BIT(reg, bit) (((uint32_t)reg >> bit) & 0x01)
#define BIT_EVEN_ALL (0x00ffffff)


// Sleep
#define BACKLIGHT_DELAY 200
#define DISPLAY_TIMEOUT 15000
#define DEVICE_TIMEOUT 120000
#define DEVICE_DEEP_TIMEOUT 1800000 // wake up after time in light sleep and go to deep sleep
#define DEVICE_DEEP_TIMEOUT_US (DEVICE_DEEP_TIMEOUT * 1000)
#define GPIO_WAKEUP_KNOB1 ENCODER_ECA_PIN
#define GPIO_WAKEUP_KNOB2 ENCODER_ECB_PIN
#define GPIO_WAKEUP_TOUCH PIN_NUM_TOUCH_INT
#define GPIO_WAKEUP_LEVEL 0


// Battery
#define BATTERY_CHECK_TIMEOUT 10000
#define BATT_MAX 2550 // observed at full charge
#define BATT_MIN 1600 // observed at device shutdown


// SD Card
#define SD_MOUNT_POINT "/sdcard" // Use B: in lvgl
#define SDMMC_CMD_PIN (gpio_num_t)3
#define SDMMC_D0_PIN (gpio_num_t)5
#define SDMMC_D1_PIN (gpio_num_t)6
#define SDMMC_D2_PIN (gpio_num_t)42
#define SDMMC_D3_PIN (gpio_num_t)2
#define SDMMC_CLK_PIN (gpio_num_t)4


// SPIFFS
#define SPIFFS_MOUNT_POINT "/spiffs" // Use A: in lvgl


// Non-volatile Memory
#define STORAGE_NAMESPACE "settings"


// Audio [unused, untested]
// #define Loobackmode 0
// #define PlaybackMusicmode 1
// #define AudioMode PlaybackMusicmode
