/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "esp_private/esp_cache_private.h"
#include "esp_codec_dev.h"
#include "esp_lcd_panel_io.h"
#include "driver/gpio.h"
#include "bsp/esp-bsp.h"
#include "app_stream_adapter.h"
#include "sdkconfig.h"
#include "driver/jpeg_decode.h"

static const char *TAG = "main";

#define DISPLAY_BUFFER_SIZE (BSP_LCD_H_RES * BSP_LCD_V_RES * 2)
#define DISPLAY_CHUNK_LINES            (10)
#define MP4_FILENAME   BSP_SD_MOUNT_POINT "/" CONFIG_MP4_FILENAME

static esp_lcd_panel_handle_t lcd_panel;
static esp_lcd_panel_io_handle_t lcd_io;
static void *lcd_buffer[CONFIG_BSP_LCD_DPI_BUFFER_NUMS];
static app_stream_adapter_handle_t stream_adapter;
static esp_codec_dev_handle_t g_audio_dev = NULL;  // Global audio device handle
static SemaphoreHandle_t g_lcd_flush_sem;
static bool g_lcd_flush_sync_enabled;
static int64_t g_last_display_ms;

static void play_media_file(const char *filename);

static bool IRAM_ATTR on_lcd_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                              esp_lcd_panel_io_event_data_t *edata,
                                              void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    (void)user_ctx;
    if (g_lcd_flush_sem == NULL) {
        return false;
    }
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(g_lcd_flush_sem, &task_woken);
    return task_woken == pdTRUE;
}

static esp_err_t display_decoded_frame(uint8_t *buffer, uint32_t buffer_size,
                                       uint32_t width, uint32_t height,
                                       uint32_t buffer_index, void *user_data)
{
    (void)buffer_index;
    (void)user_data;

    if (buffer == NULL || width == 0 || height == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t pixel_count = width * height;
    if (pixel_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t bytes_per_pixel = buffer_size / pixel_count;
    if (bytes_per_pixel == 0) {
        bytes_per_pixel = 2;
    }

    uint32_t y = 0;
    while (y < height) {
        uint32_t chunk_h = DISPLAY_CHUNK_LINES;
        uint32_t remain = height - y;
        if (chunk_h > remain) {
            chunk_h = remain;
        }
        uint32_t offset = y * width * bytes_per_pixel;
        uint32_t chunk_bytes = chunk_h * width * bytes_per_pixel;
        if (offset + chunk_bytes > buffer_size) {
            return ESP_ERR_INVALID_SIZE;
        }

        esp_err_t ret = esp_lcd_panel_draw_bitmap(lcd_panel, 0, y, width, y + chunk_h, buffer + offset);
        if (ret != ESP_OK) {
            return ret;
        }
        y += chunk_h;
    }

    g_last_display_ms = esp_timer_get_time() / 1000;
    return ESP_OK;
}

void app_main()
{
    ESP_LOGI(TAG, "Starting HDMI MP4 Player application");

    g_lcd_flush_sem = xSemaphoreCreateBinary();
    g_lcd_flush_sync_enabled = false;
    g_last_display_ms = 0;

    esp_err_t ret = bsp_display_new(NULL, &lcd_panel, &lcd_io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display: %s", esp_err_to_name(ret));
        return;
    }

    if (g_lcd_flush_sem != NULL && lcd_io != NULL) {
        esp_lcd_panel_io_callbacks_t io_cbs = {
            .on_color_trans_done = on_lcd_color_trans_done,
        };
        ret = esp_lcd_panel_io_register_event_callbacks(lcd_io, &io_cbs, NULL);
        if (ret == ESP_OK) {
            g_lcd_flush_sync_enabled = true;
        } else {
            ESP_LOGW(TAG, "Panel IO callback register failed: %s", esp_err_to_name(ret));
        }
    }

    ret = bsp_display_brightness_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize backlight: %s", esp_err_to_name(ret));
        return;
    }

    bsp_display_backlight_on();

    jpeg_decode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    size_t out_size = 0;
    uint32_t decode_buffer_size = 0;
    int buf_index = 0;
    while (buf_index < CONFIG_BSP_LCD_DPI_BUFFER_NUMS) {
        lcd_buffer[buf_index] = jpeg_alloc_decoder_mem(DISPLAY_BUFFER_SIZE, &mem_cfg, &out_size);
        if (lcd_buffer[buf_index] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate display buffer %d", buf_index);
            return;
        }
        decode_buffer_size = (uint32_t)out_size;
        buf_index++;
    }

    ret = bsp_sdcard_mount();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card");
        return;
    }

    // Initialize audio codec
    ESP_LOGI(TAG, "Initializing audio codec...");
    esp_codec_dev_handle_t audio_dev = bsp_audio_codec_speaker_init();
    if (audio_dev == NULL) {
        ESP_LOGW(TAG, "Failed to initialize audio codec, continuing without audio");
        g_audio_dev = NULL;
    } else {
        ESP_LOGI(TAG, "Audio codec initialized successfully");
        esp_codec_dev_set_out_vol(audio_dev, 80);
        g_audio_dev = audio_dev;
    }

    // Initialize stream adapter with unified configuration
    ESP_LOGI(TAG, "Initializing stream adapter...");

    // User data can be any context information needed by the callback
    void *custom_user_data = NULL;  // Can be any struct pointer or data

    app_stream_adapter_config_t adapter_config = {
        .frame_cb = display_decoded_frame,
        .user_data = custom_user_data,
        .decode_buffers = lcd_buffer,
        .buffer_count = CONFIG_BSP_LCD_DPI_BUFFER_NUMS,
        .buffer_size = decode_buffer_size,
        .audio_dev = audio_dev,
        .jpeg_config = {
            .output_format = APP_STREAM_JPEG_OUTPUT_RGB565,
            .bgr_order = false
        }
    };

    ret = app_stream_adapter_init(&adapter_config, &stream_adapter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize stream adapter: %d", ret);
        if (audio_dev) {
            esp_codec_dev_close(audio_dev);
        }
        return;
    }

    const char *format_str = (adapter_config.jpeg_config.output_format == APP_STREAM_JPEG_OUTPUT_RGB888) ? "RGB888" : "RGB565";
    ESP_LOGI(TAG, "Stream adapter initialized with %s format%s", format_str,
             audio_dev ? " and audio support" : "");

    FILE *fp = fopen(MP4_FILENAME, "rb");
    if (fp) {
        fclose(fp);

        app_stream_adapter_set_file(stream_adapter, MP4_FILENAME, g_audio_dev != NULL);
        play_media_file(MP4_FILENAME);
    } else {
        ESP_LOGW(TAG, "MP4 file not found: %s", MP4_FILENAME);
    }
}

static void play_media_file(const char *filename)
{
    ESP_LOGI(TAG, "Starting loop playback of %s", filename);

    // Get stream info once
    uint32_t width, height, fps, duration;
    esp_err_t ret = app_stream_adapter_get_info(stream_adapter, &width, &height, &fps, &duration);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Media info: %"PRIu32"x%"PRIu32", %"PRIu32" fps, duration: %"PRIu32" ms",
                 width, height, fps, duration);
    }

    uint32_t loop_count = 0;

    // Continuous playback loop
    while (1) {
        ESP_LOGI(TAG, "Starting playback loop #%" PRIu32, ++loop_count);

        // Start playback
        ret = app_stream_adapter_start(stream_adapter);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start playback: %d", ret);
            break;
        }

        // Monitor playback with improved detection
        uint32_t stable_count = 0;
        uint32_t last_frames = 0;

        while (1) {
            app_stream_stats_t stats;
            ret = app_stream_adapter_get_stats(stream_adapter, &stats);

            if (ret == ESP_OK) {
                if (stats.frames_processed == last_frames) {
                    stable_count++;
                    // If frame count hasn't changed for 3 consecutive checks, assume end
                    if (stable_count >= 3) {
                        ESP_LOGI(TAG, "Playback loop #%" PRIu32 " finished (%" PRIu32 " frames), restarting...",
                                 loop_count, stats.frames_processed);
                        break;
                    }
                } else {
                    stable_count = 0;
                    last_frames = stats.frames_processed;
                }
            } else {
                ESP_LOGW(TAG, "Error getting stats, assuming playback ended");
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(500)); // Check every 500ms
        }

        // Stop and reset for next loop
        app_stream_adapter_stop(stream_adapter);
        vTaskDelay(pdMS_TO_TICKS(200)); // Brief pause
        app_stream_adapter_set_file(stream_adapter, filename, g_audio_dev != NULL);
    }
}
