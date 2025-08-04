/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_task_wdt.h"
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_lcd_io_spi.h"
// #include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "esp_freertos_hooks.h"
#include "driver/gpio.h"
#include "huc.h"

#define NOT_USED_PIN -1
#define PIN_NUM_SCLK 7
#define PIN_NUM_MOSI 16
#define PIN_NUM_DC 19
#define PIN_NUM_RST 20
#define PIN_NUM_BCKL 18
#define PCKL_HZ 80000000 // 80MHz
#define CMD_BITS 8
#define PARAM_BITS 8
#define LCD_H_RES 240 // Horizontal resolution of the LCD panel
#define LCD_V_RES 240
#define LCD_HOST SPI2_HOST
#define LCD_BK_LIGHT_ON_LEVEL 1
#define LVGL_TICK_PERIOD_MS 2
#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))

// esp_timer_handle_t lvgl_tick_timer;

// // Timer callback function
// void lvgl_tick_timer_cb(void *arg)
// {
//     lv_tick_inc(LVGL_TICK_PERIOD_MS);
// }

// void flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
// {
//     /* The most simple case (also the slowest) to send all rendered pixels to the
//      * screen one-by-one.  `put_px` is just an example.  It needs to be implemented by you. */
//     uint16_t *buf16 = (uint16_t *)px_map; /* Let's say it's a 16 bit (RGB565) display */
//     esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(display);
//     esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, buf16);

//     /* IMPORTANT!!!
//      * Inform LVGL that flushing is complete so buffer can be modified again. */
//     printf("Flush is called and ready\n");
//     lv_display_flush_ready(display);
// }

void app_main(void)
{
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = NOT_USED_PIN,
        .quadwp_io_num = NOT_USED_PIN,
        .quadhd_io_num = NOT_USED_PIN,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t), // transfer 80 lines of pixels (assume pixel is RGB565) at most in one SPI transaction
    };

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO)); // Enable the DMA feature

    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = NOT_USED_PIN,
        .pclk_hz = PCKL_HZ,
        .lcd_cmd_bits = CMD_BITS,
        .lcd_param_bits = PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    // Create LCD panel handle for ST7789, with the SPI IO device handle
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    // the gap is LCD panel specific, even panels with the same driver IC, can have different gap value
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 240, 240, huc));

    // lv_init();
    // lv_tick_set_cb(xTaskGetTickCount);

    // // order matter here, timer must start after the LVGL is initialized
    // //  Create a periodic timer to call lv_tick_inc every 1ms
    // const esp_timer_create_args_t lvgl_tick_timer_args = {
    //     .callback = &lvgl_tick_timer_cb,
    //     .name = "lvgl_tick_timer"};
    // ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000)); // Call every 1ms (1000 microseconds)

    // lv_display_t *st_7789_display = lv_display_create(LCD_H_RES, LCD_V_RES);
    // lv_display_set_user_data(st_7789_display, panel_handle);

    // /* Declare buffer for 1/10 screen size; BYTES_PER_PIXEL will be 2 for RGB565. */
    // static uint8_t lcd_draw_buffer[LCD_H_RES * LCD_V_RES / 10 * BYTES_PER_PIXEL];
    // static uint8_t lcd_draw_buffer_second[LCD_H_RES * LCD_V_RES / 10 * BYTES_PER_PIXEL];
    // /* Set display buffer for display `st_7789_display`. */
    // lv_display_set_buffers(st_7789_display, lcd_draw_buffer, lcd_draw_buffer_second, sizeof(lcd_draw_buffer), LV_DISPLAY_RENDER_MODE_PARTIAL);

    // lv_display_set_flush_cb(st_7789_display, flush_cb);
    // lv_display_set_default(st_7789_display);

    // lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003a57), LV_PART_MAIN);
    // if (lv_screen_active() == NULL)
    // {
    //     printf("lv_screen_active() returned NULL, check LVGL initialization\n");
    // }

    // /*Create a white label, set its text and align it to the center*/
    // lv_obj_t *label = lv_label_create(lv_screen_active());
    // lv_label_set_text(label, "Hello world");
    // lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff), LV_PART_MAIN);
    // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    // while (true)
    // {
    //     lv_timer_handler_run_in_period(50);
    // }
}