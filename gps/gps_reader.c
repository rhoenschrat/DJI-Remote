/*
 * DJI Camera Remote Control - GPS Reader Module
 * 
 * This file implements GPS communication and NMEA parsing for the M5Stack Module GPS v2.0.
 * It reads GPS data from UART2, parses NMEA sentences (GPGGA, GPRMC, GPGSA), and provides
 * thread-safe access to GPS data including position, velocity, accuracy, and fix status.
 * 
 * Key features:
 * - UART2 communication with configurable baud rate
 * - NMEA sentence parsing (GPGGA, GPRMC, GPGSA)
 * - Thread-safe GPS data access via mutex
 * - GPS data age tracking
 * - Accuracy filtering (HDOP/VDOP based)
 */

#include "gps_reader.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "GPS";

/* GPS data storage with mutex protection */
static gps_data_t g_gps_data = {0};
static SemaphoreHandle_t g_gps_mutex = NULL;
static TaskHandle_t g_gps_task_handle = NULL;
static bool g_gps_initialized = false;

/* Forward declarations */
static void gps_task(void *pvParameters);
static void gps_debug_task(void *pvParameters);
static bool parse_nmea_sentence(const char *sentence);
static bool parse_gpgga_sentence(const char *sentence);
static bool parse_gprmc_sentence(const char *sentence);
static bool parse_gpgsa_sentence(const char *sentence);
static float parse_coordinate_ddmm(float coord_ddmm, char hemisphere);

/* Debug configuration */
#define GPS_DEBUG_RAW_NMEA 0  /* Set to 1 to enable raw NMEA sentence logging */
#define GPS_DEBUG_UART_DATA 1  /* Set to 1 to enable UART data reception logging */
#define GPS_DEBUG_INTERVAL_MS 5000  /* Debug output interval in milliseconds (increased from 2500 to reduce log frequency) */
#define GPS_DEBUG_TASK_STACK_SIZE 6144  /* Stack size for gps_debug task (increased from 2048 to handle SD logging overhead) */

/**
 * @brief Initialize GPS module
 */
esp_err_t gps_init(void) {
    if (g_gps_initialized) {
        ESP_LOGW(TAG, "GPS already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing GPS module on UART%d", GPS_UART_NUM);

    /* Create mutex for thread-safe GPS data access */
    g_gps_mutex = xSemaphoreCreateMutex();
    if (g_gps_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create GPS mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Configure UART parameters */
    uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    /* Install UART driver */
    esp_err_t ret = uart_driver_install(GPS_UART_NUM, GPS_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_gps_mutex);
        return ret;
    }

    /* Configure UART parameters */
    ret = uart_param_config(GPS_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        uart_driver_delete(GPS_UART_NUM);
        vSemaphoreDelete(g_gps_mutex);
        return ret;
    }

    /* Set UART pins */
    ret = uart_set_pin(GPS_UART_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(GPS_UART_NUM);
        vSemaphoreDelete(g_gps_mutex);
        return ret;
    }

    /* Initialize GPS data structure */
    memset(&g_gps_data, 0, sizeof(gps_data_t));
    g_gps_data.has_fix = false;

    /* Create GPS parsing task */
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, &g_gps_task_handle);
    if (g_gps_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create GPS task");
        uart_driver_delete(GPS_UART_NUM);
        vSemaphoreDelete(g_gps_mutex);
        return ESP_ERR_NO_MEM;
    }

    g_gps_initialized = true;
    ESP_LOGI(TAG, "GPS UART initialized successfully on UART%d (TX=%d, RX=%d, Baud=%d)", 
             GPS_UART_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, GPS_UART_BAUD_RATE);
    ESP_LOGI(TAG, "GPS module initialized successfully");
    ESP_LOGI(TAG, "Note: GPS module may need several minutes for cold start fix");
    ESP_LOGI(TAG, "Note: Verify GPS module DIP switch settings match baud rate (%d)", GPS_UART_BAUD_RATE);

    /* Create GPS debug task for periodic status output
     * Note: Stack size is increased to 6144 bytes to provide sufficient
     * headroom for logging operations and GPS data processing.
     */
    xTaskCreate(gps_debug_task, "gps_debug", GPS_DEBUG_TASK_STACK_SIZE, NULL, 1, NULL);

    return ESP_OK;
}

/**
 * @brief Deinitialize GPS module
 */
esp_err_t gps_deinit(void) {
    if (!g_gps_initialized) {
        return ESP_OK;
    }

    /* Delete GPS task */
    if (g_gps_task_handle != NULL) {
        vTaskDelete(g_gps_task_handle);
        g_gps_task_handle = NULL;
    }

    /* Delete UART driver */
    uart_driver_delete(GPS_UART_NUM);

    /* Delete mutex */
    if (g_gps_mutex != NULL) {
        vSemaphoreDelete(g_gps_mutex);
        g_gps_mutex = NULL;
    }

    g_gps_initialized = false;
    ESP_LOGI(TAG, "GPS module deinitialized");

    return ESP_OK;
}

/**
 * @brief GPS parsing task
 * 
 * Continuously reads NMEA sentences from UART and parses them.
 */
static void gps_task(void *pvParameters) {
    char nmea_buffer[GPS_NMEA_MAX_LEN];
    int buffer_pos = 0;

    ESP_LOGI(TAG, "GPS task started");

    uint32_t total_bytes_received = 0;
    uint32_t last_uart_activity_log = 0;
    
    while (1) {
        /* Read available data from UART */
        uint8_t data[128];
        int len = uart_read_bytes(GPS_UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            total_bytes_received += len;
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            /* Log UART activity periodically (every 5 seconds) */
            if (current_time - last_uart_activity_log > 5000) {
                ESP_LOGI(TAG, "UART2 receiving data: %d bytes in last period (total: %lu bytes)", 
                         len, (unsigned long)total_bytes_received);
                last_uart_activity_log = current_time;
                
                /* Check if data looks like valid NMEA (contains $ or valid NMEA characters) */
                bool has_nmea_start = false;
                bool has_valid_chars = false;
                for (int j = 0; j < len && j < 20; j++) {
                    if (data[j] == '$') {
                        has_nmea_start = true;
                        break;
                    }
                    /* Check for valid NMEA characters (letters, numbers, commas, dots, asterisks) */
                    if ((data[j] >= 'A' && data[j] <= 'Z') || 
                        (data[j] >= '0' && data[j] <= '9') ||
                        data[j] == ',' || data[j] == '.' || data[j] == '*') {
                        has_valid_chars = true;
                    }
                }
                
                /* Only warn if we have no $ AND no valid NMEA characters (suggests corruption) */
                if (!has_nmea_start && !has_valid_chars && len > 10) {
                    ESP_LOGW(TAG, "WARNING: Received data does not look like NMEA (no $ or valid chars)");
                    ESP_LOGW(TAG, "This may indicate baud rate mismatch! Current: %d, try: 9600 or 115200", GPS_UART_BAUD_RATE);
                }
                
#if GPS_DEBUG_UART_DATA
                /* Log first few bytes for debugging */
                int log_len = len < 32 ? len : 32;
                char hex_buffer[128];  /* Buffer for hex representation */
                int hex_pos = 0;
                for (int j = 0; j < log_len && hex_pos < (sizeof(hex_buffer) - 5); j++) {
                    if (data[j] >= 32 && data[j] < 127) {
                        hex_buffer[hex_pos++] = data[j];
                    } else {
                        hex_pos += snprintf(hex_buffer + hex_pos, sizeof(hex_buffer) - hex_pos, "\\x%02X", data[j]);
                    }
                }
                hex_buffer[hex_pos] = '\0';
                ESP_LOGI(TAG, "Raw UART data (first %d bytes): %s", log_len, hex_buffer);
#endif
            }
            
            data[len] = '\0';  /* Null terminate */

            /* Process each character */
            for (int i = 0; i < len; i++) {
                char c = data[i];

                /* Check for start of NMEA sentence */
                if (c == '$') {
                    /* If we were building a sentence, discard it (incomplete) */
                    if (buffer_pos > 0) {
                        ESP_LOGW(TAG, "Discarding incomplete sentence: %.*s", buffer_pos, nmea_buffer);
                    }
                    buffer_pos = 0;
                    nmea_buffer[buffer_pos++] = c;
                }
                /* Check for end of sentence (CR or LF) */
                else if (c == '\r' || c == '\n') {
                    /* Skip multiple CR/LF characters */
                    if (buffer_pos > 0) {
                        nmea_buffer[buffer_pos] = '\0';
                        
                        /* Only parse if sentence is long enough and starts with $ */
                        if (buffer_pos > 6 && nmea_buffer[0] == '$') {
#if GPS_DEBUG_RAW_NMEA
                            /* Log raw NMEA sentence for debugging (verbose level to reduce log volume) */
                            ESP_LOGV(TAG, "Raw NMEA: %s", nmea_buffer);
#else
                            /* Log first part of sentence for debugging (verbose level to reduce log volume) */
                            int log_len = buffer_pos < 50 ? buffer_pos : 50;
                            ESP_LOGV(TAG, "Parsing NMEA: %.*s", log_len, nmea_buffer);
#endif
                            
                            /* Parse NMEA sentence */
                            if (parse_nmea_sentence(nmea_buffer)) {
                                ESP_LOGV(TAG, "✓ Successfully parsed NMEA sentence");
                                /* Update timestamp */
                                if (xSemaphoreTake(g_gps_mutex, portMAX_DELAY) == pdTRUE) {
                                    g_gps_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
                                    xSemaphoreGive(g_gps_mutex);
                                }
                            } else {
                                ESP_LOGV(TAG, "NMEA sentence not parsed (unsupported type): %.*s", 30, nmea_buffer);
                            }
                        } else if (buffer_pos > 0) {
                            ESP_LOGW(TAG, "Discarding invalid sentence fragment: %.*s", buffer_pos, nmea_buffer);
                        }
                        
                        buffer_pos = 0;
                    }
                }
                /* Add character to buffer if we're in a sentence */
                else if (buffer_pos > 0 && buffer_pos < GPS_NMEA_MAX_LEN - 1) {
                    nmea_buffer[buffer_pos++] = c;
                }
                /* If we see a non-$ character and we're not in a sentence, ignore it */
                /* (This handles garbage data between sentences) */
            }
        } else {
            /* No data received - check UART buffer status and log periodically */
            static uint32_t last_no_data_log = 0;
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            if (current_time - last_no_data_log > 10000) {  /* Every 10 seconds */
                if (total_bytes_received == 0) {
                    /* Check if there's data waiting in UART buffer */
                    size_t buffered_size = 0;
                    uart_get_buffered_data_len(GPS_UART_NUM, &buffered_size);
                    
                    if (buffered_size > 0) {
                        ESP_LOGW(TAG, "UART2: %zu bytes buffered but not read. Possible buffer overflow or reading issue.", buffered_size);
                    } else {
                        ESP_LOGW(TAG, "UART2: No data received. Check GPS module:");
                        ESP_LOGW(TAG, "  1. Module is connected to M-Bus connector");
                        ESP_LOGW(TAG, "  2. Module is powered (LED should be on)");
                        ESP_LOGW(TAG, "  3. DIP switch matches baud rate (current: %d)", GPS_UART_BAUD_RATE);
                        ESP_LOGW(TAG, "  4. Pins: TX=GPIO17, RX=GPIO16");
                        ESP_LOGW(TAG, "  5. Try 115200 baud if 9600 doesn't work");
                    }
                }
                last_no_data_log = current_time;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  /* Small delay to prevent CPU hogging */
    }
}

/**
 * @brief Parse NMEA sentence
 * 
 * Routes NMEA sentences to appropriate parsers based on sentence type.
 * 
 * @param sentence NMEA sentence string
 * @return true if sentence was successfully parsed
 */
static bool parse_nmea_sentence(const char *sentence) {
    if (sentence == NULL || strlen(sentence) < 6) {
        return false;
    }

    /* Route to appropriate parser based on sentence type */
    /* Support both GPS ($GP) and GNSS ($GN) variants */
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        return parse_gpgga_sentence(sentence);
    } else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        return parse_gprmc_sentence(sentence);
    } else if (strncmp(sentence, "$GPGSA", 6) == 0 || strncmp(sentence, "$GNGSA", 6) == 0) {
        return parse_gpgsa_sentence(sentence);
    }

    return false;
}

/**
 * @brief Parse GPGGA sentence
 * 
 * Parses GPGGA (Global Positioning System Fix Data) sentence.
 * Example: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 * 
 * @param sentence NMEA sentence string
 * @return true if sentence was successfully parsed
 */
static bool parse_gpgga_sentence(const char *sentence) {
    /* Parse GPGGA sentence */
    float lat_ddmm, lon_ddmm, alt;
    uint8_t fix_quality, satellites;
    char lat_hem, lon_hem;

    /* Example: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47 */
    /* Extract fields manually for better control */
    const char *p = sentence;
    int field = 0;
    char fields[15][32] = {0};
    int char_idx = 0;

    /* Split sentence into comma-separated fields */
    while (*p && field < 15) {
        if (*p == ',') {
            fields[field][char_idx] = '\0';
            field++;
            char_idx = 0;
        } else if (*p != '$' && *p != '*') {
            if (char_idx < 31) {
                fields[field][char_idx++] = *p;
            }
        }
        p++;
    }
    fields[field][char_idx] = '\0';

    /* Parse relevant fields */
    if (field >= 9) {
        /* Field 2: Latitude (DDMM.MMMM) */
        if (fields[2][0] != '\0') {
            lat_ddmm = strtof(fields[2], NULL);
            lat_hem = fields[3][0];
        } else {
            return false;
        }

        /* Field 4: Longitude (DDDMM.MMMM) */
        if (fields[4][0] != '\0') {
            lon_ddmm = strtof(fields[4], NULL);
            lon_hem = fields[5][0];
        } else {
            return false;
        }

        /* Field 6: Fix quality */
        fix_quality = (uint8_t)atoi(fields[6]);

        /* Field 7: Satellite count */
        satellites = (uint8_t)atoi(fields[7]);

        /* Field 8: HDOP (Horizontal Dilution of Precision) */
        float hdop = 0.0f;
        if (fields[8][0] != '\0') {
            hdop = strtof(fields[8], NULL);
        }

        /* Field 9: Altitude */
        alt = strtof(fields[9], NULL);

        /* Convert coordinates from DDMM.MMMM to decimal degrees */
        float lat_deg = parse_coordinate_ddmm(lat_ddmm, lat_hem);
        float lon_deg = parse_coordinate_ddmm(lon_ddmm, lon_hem);

        /* Update GPS data with mutex protection */
        if (xSemaphoreTake(g_gps_mutex, portMAX_DELAY) == pdTRUE) {
            g_gps_data.latitude = lat_deg;
            g_gps_data.longitude = lon_deg;
            g_gps_data.altitude = alt;
            g_gps_data.fix_quality = fix_quality;
            g_gps_data.satellite_count = satellites;
            g_gps_data.hdop = hdop;
            g_gps_data.has_fix = (fix_quality > 0);
            xSemaphoreGive(g_gps_mutex);
        }

        return true;
    }
    
    return false;
}

/**
 * @brief Parse GPRMC sentence
 * 
 * Parses GPRMC (Recommended Minimum) sentence for speed and course.
 * Example: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
 * 
 * @param sentence NMEA sentence string
 * @return true if sentence was successfully parsed
 */
static bool parse_gprmc_sentence(const char *sentence) {
    float speed_knots, course;
    char status;

    /* Example: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A */
    int parsed = sscanf(sentence, "$GPRMC,%*f,%c,%*f,%*c,%*f,%*c,%f,%f,%*f,%*f,%*c",
                       &status, &speed_knots, &course);

    if (parsed >= 3 && status == 'A') {  /* 'A' = valid fix */
        /* Convert speed from knots to m/s */
        float speed_ms = speed_knots * 0.514444f;

        /* Update GPS data with mutex protection */
        if (xSemaphoreTake(g_gps_mutex, portMAX_DELAY) == pdTRUE) {
            g_gps_data.speed = speed_ms;
            g_gps_data.course = course;
            xSemaphoreGive(g_gps_mutex);
        }

        return true;
    }
    
    return false;
}

/**
 * @brief Parse GPGSA sentence
 * 
 * Parses GPGSA (GPS DOP and Active Satellites) sentence for fix type, DOP values, and satellites used.
 * Example: $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
 * 
 * @param sentence NMEA sentence string
 * @return true if sentence was successfully parsed
 */
static bool parse_gpgsa_sentence(const char *sentence) {
    /* Example: $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39 */
    /* Fields: 0=$GPGSA, 1=Mode, 2=Fix type, 3-14=Satellite IDs, 15=PDOP, 16=HDOP, 17=VDOP */
    
    const char *p = sentence;
    int field = 0;
    char fields[18][32] = {0};
    int char_idx = 0;

    /* Split sentence into comma-separated fields */
    while (*p && field < 18) {
        if (*p == ',') {
            fields[field][char_idx] = '\0';
            field++;
            char_idx = 0;
        } else if (*p != '$' && *p != '*') {
            if (char_idx < 31) {
                fields[field][char_idx++] = *p;
            }
        }
        p++;
    }
    fields[field][char_idx] = '\0';

    /* Need at least fix type field (field 2) */
    if (field < 2) {
        return false;
    }

    /* Field 2: Fix type (1=no fix, 2=2D, 3=3D) */
    uint8_t fix_type = 0;
    if (fields[2][0] != '\0') {
        int fix_type_val = atoi(fields[2]);
        if (fix_type_val == 1) {
            fix_type = 0;  /* No fix */
        } else if (fix_type_val == 2) {
            fix_type = 1;  /* 2D fix */
        } else if (fix_type_val == 3) {
            fix_type = 2;  /* 3D fix */
        }
    }

    /* Count satellites used (non-empty fields 3-14) */
    uint8_t satellites_used = 0;
    for (int i = 3; i <= 14; i++) {
        if (fields[i][0] != '\0') {
            satellites_used++;
        }
    }

    /* Field 15: PDOP */
    float pdop = 0.0f;
    if (field >= 15 && fields[15][0] != '\0') {
        pdop = strtof(fields[15], NULL);
    }

    /* Field 16: HDOP */
    float hdop = 0.0f;
    if (field >= 16 && fields[16][0] != '\0') {
        hdop = strtof(fields[16], NULL);
    }

    /* Field 17: VDOP */
    float vdop = 0.0f;
    if (field >= 17 && fields[17][0] != '\0') {
        vdop = strtof(fields[17], NULL);
    }

    /* Update GPS data with mutex protection */
    if (xSemaphoreTake(g_gps_mutex, portMAX_DELAY) == pdTRUE) {
        g_gps_data.fix_type = fix_type;
        g_gps_data.satellites_used = satellites_used;
        if (pdop > 0.0f) {
            g_gps_data.pdop = pdop;
        }
        if (hdop > 0.0f) {
            g_gps_data.hdop = hdop;  /* GPGSA HDOP may override GPGGA HDOP */
        }
        if (vdop > 0.0f) {
            g_gps_data.vdop = vdop;
        }
        xSemaphoreGive(g_gps_mutex);
    }

    return true;
}

/**
 * @brief Parse coordinate from NMEA DDMM.MMMM format to decimal degrees
 * 
 * NMEA format: DDMM.MMMM (degrees and minutes)
 * Converts to decimal degrees: DD + MM.MMMM / 60
 * 
 * @param coord_ddmm Coordinate in DDMM.MMMM format
 * @param hemisphere Hemisphere character ('N', 'S', 'E', 'W')
 * @return Coordinate in decimal degrees
 */
static float parse_coordinate_ddmm(float coord_ddmm, char hemisphere) {
    /* Convert from DDMM.MMMM to decimal degrees */
    int degrees = (int)(coord_ddmm / 100);
    float minutes = coord_ddmm - (degrees * 100);
    float decimal_degrees = degrees + (minutes / 60.0f);
    
    /* Apply hemisphere sign */
    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal_degrees = -decimal_degrees;
    }
    
    return decimal_degrees;
}

/**
 * @brief Check if GPS has a valid fix
 */
bool gps_has_fix(void) {
    bool has_fix = false;
    
    if (g_gps_mutex != NULL && xSemaphoreTake(g_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        has_fix = g_gps_data.has_fix;
        xSemaphoreGive(g_gps_mutex);
    }
    
    return has_fix;
}

/**
 * @brief Get current GPS data
 */
esp_err_t gps_get_data(gps_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_gps_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(g_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(data, &g_gps_data, sizeof(gps_data_t));
        xSemaphoreGive(g_gps_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Get latitude
 */
float gps_get_latitude(void) {
    float lat = 0.0f;
    
    if (g_gps_mutex != NULL && xSemaphoreTake(g_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        lat = g_gps_data.latitude;
        xSemaphoreGive(g_gps_mutex);
    }
    
    return lat;
}

/**
 * @brief Get longitude
 */
float gps_get_longitude(void) {
    float lon = 0.0f;
    
    if (g_gps_mutex != NULL && xSemaphoreTake(g_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        lon = g_gps_data.longitude;
        xSemaphoreGive(g_gps_mutex);
    }
    
    return lon;
}

/**
 * @brief Get altitude
 */
float gps_get_altitude(void) {
    float alt = 0.0f;
    
    if (g_gps_mutex != NULL && xSemaphoreTake(g_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        alt = g_gps_data.altitude;
        xSemaphoreGive(g_gps_mutex);
    }
    
    return alt;
}

/**
 * @brief Get speed
 */
float gps_get_speed(void) {
    float speed = 0.0f;
    
    if (g_gps_mutex != NULL && xSemaphoreTake(g_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        speed = g_gps_data.speed;
        xSemaphoreGive(g_gps_mutex);
    }
    
    return speed;
}

/**
 * @brief Get satellite count
 */
uint8_t gps_get_satellite_count(void) {
    uint8_t count = 0;
    
    if (g_gps_mutex != NULL && xSemaphoreTake(g_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        count = g_gps_data.satellite_count;
        xSemaphoreGive(g_gps_mutex);
    }
    
    return count;
}

/**
 * @brief Get course
 */
float gps_get_course(void) {
    float course = 0.0f;
    
    if (g_gps_mutex != NULL && xSemaphoreTake(g_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        course = g_gps_data.course;
        xSemaphoreGive(g_gps_mutex);
    }
    
    return course;
}

/**
 * @brief Read raw NMEA sentence
 */
int gps_read_nmea(char *buffer, size_t buffer_size) {
    if (buffer == NULL || buffer_size == 0) {
        return -1;
    }
    
    int len = uart_read_bytes(GPS_UART_NUM, (uint8_t *)buffer, buffer_size - 1, pdMS_TO_TICKS(100));
    
    if (len > 0) {
        buffer[len] = '\0';
    }
    
    return len;
}

/**
 * @brief GPS debug task - periodic status output
 * 
 * Logs GPS status every 5 seconds to help verify UART communication
 * and GPS data reception. Stack size increased to 6144 bytes to provide
 * sufficient headroom for logging operations and GPS data processing.
 */
static void gps_debug_task(void *pvParameters) {
    ESP_LOGI(TAG, "GPS debug task started");
    
    uint32_t stack_check_counter = 0;
    const uint32_t STACK_CHECK_INTERVAL = 10000 / GPS_DEBUG_INTERVAL_MS;  /* Check every ~10 seconds */
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(GPS_DEBUG_INTERVAL_MS));
        
        if (!g_gps_initialized) {
            continue;
        }
        
        /* Check stack high-watermark periodically to monitor stack usage */
        stack_check_counter++;
        if (stack_check_counter >= STACK_CHECK_INTERVAL) {
            UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG, "GPS debug task stack high-watermark: %u bytes remaining", 
                     stack_remaining * sizeof(StackType_t));
            stack_check_counter = 0;
        }
        
        gps_data_t gps_data;
        if (gps_get_data(&gps_data) == ESP_OK) {
            uint32_t age_ms = gps_get_last_update_age_ms();
            
            if (age_ms < 10000) {  /* Data received within last 10 seconds */
                /* Simplified output: single line with essential info */
                ESP_LOGI(TAG, "GPS: Fix=%s, Lat=%.6f, Lon=%.6f, Alt=%.1fm, Sats=%d/%d, HDOP=%.1f",
                         gps_data.has_fix ? "YES" : "NO",
                         gps_data.latitude,
                         gps_data.longitude,
                         gps_data.altitude,
                         gps_data.satellites_used,
                         gps_data.satellite_count,
                         gps_data.hdop);
            } else {
                ESP_LOGW(TAG, "GPS Status: No data received recently (last update %lu ms ago)", age_ms);
            }
        } else {
            ESP_LOGW(TAG, "GPS Status: Failed to retrieve GPS data");
        }
    }
}

/**
 * @brief Get human-readable fix type string
 */
const char* gps_get_fix_type_string(uint8_t fix_type) {
    switch (fix_type) {
        case 0:
            return "No Fix";
        case 1:
            return "2D";
        case 2:
            return "3D";
        default:
            return "Unknown";
    }
}

/**
 * @brief Get milliseconds since last GPS update
 */
uint32_t gps_get_last_update_age_ms(void) {
    if (g_gps_mutex == NULL || !g_gps_initialized) {
        return UINT32_MAX;
    }
    
    uint32_t age_ms = UINT32_MAX;
    if (xSemaphoreTake(g_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (g_gps_data.timestamp > 0) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (current_time >= g_gps_data.timestamp) {
                age_ms = current_time - g_gps_data.timestamp;
            }
        }
        xSemaphoreGive(g_gps_mutex);
    }
    
    return age_ms;
}

/**
 * @brief Test GPS communication at current baud rate
 * 
 * Reads data from the GPS module and checks if it looks like valid NMEA.
 */
bool gps_test_communication(uint32_t duration_ms) {
    if (!g_gps_initialized) {
        ESP_LOGE(TAG, "GPS not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Testing GPS communication at %d baud for %lu ms...", 
             GPS_UART_BAUD_RATE, (unsigned long)duration_ms);
    
    uint8_t buffer[256];
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t total_bytes = 0;
    uint32_t nmea_start_count = 0;
    uint32_t valid_ascii_count = 0;
    uint32_t invalid_char_count = 0;
    bool found_nmea = false;
    
    /* Flush UART buffer first */
    uart_flush(GPS_UART_NUM);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < duration_ms) {
        int len = uart_read_bytes(GPS_UART_NUM, buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            buffer[len] = '\0';
            total_bytes += len;
            
            /* Analyze the data */
            for (int i = 0; i < len; i++) {
                uint8_t c = buffer[i];
                
                /* Count NMEA sentence starts */
                if (c == '$') {
                    nmea_start_count++;
                    found_nmea = true;
                }
                
                /* Count valid ASCII characters (printable + CR/LF) */
                if ((c >= 32 && c <= 126) || c == '\r' || c == '\n') {
                    valid_ascii_count++;
                } else if (c != '$') {
                    invalid_char_count++;
                }
            }
            
            /* Show sample of data */
            if (total_bytes < 100) {
                ESP_LOGI(TAG, "Sample data: %.*s", len < 64 ? len : 64, buffer);
            }
        }
    }
    
    /* Calculate statistics */
    float valid_percent = total_bytes > 0 ? (100.0f * valid_ascii_count / total_bytes) : 0.0f;
    
    ESP_LOGI(TAG, "Test results:");
    ESP_LOGI(TAG, "  Total bytes: %lu", (unsigned long)total_bytes);
    ESP_LOGI(TAG, "  NMEA starts ($): %lu", (unsigned long)nmea_start_count);
    ESP_LOGI(TAG, "  Valid ASCII: %lu (%.1f%%)", (unsigned long)valid_ascii_count, valid_percent);
    ESP_LOGI(TAG, "  Invalid chars: %lu", (unsigned long)invalid_char_count);
    
    /* Determine if data looks valid */
    bool is_valid = found_nmea && (valid_percent > 80.0f) && (nmea_start_count > 0);
    
    if (is_valid) {
        ESP_LOGI(TAG, "✓ Data looks like valid NMEA at %d baud", GPS_UART_BAUD_RATE);
    } else if (total_bytes == 0) {
        ESP_LOGW(TAG, "✗ No data received - check connections");
    } else if (!found_nmea) {
        ESP_LOGW(TAG, "✗ Data received but no NMEA markers ($) - BAUD RATE MISMATCH!");
    } else {
        ESP_LOGW(TAG, "✗ Data corrupted (%.1f%% valid) - BAUD RATE MISMATCH!", valid_percent);
    }
    
    return is_valid;
}

/**
 * @brief Auto-detect GPS module baud rate
 * 
 * Tests common baud rates to find which one works.
 */
uint32_t gps_auto_detect_baud_rate(void) {
    const uint32_t baud_rates[] = {9600, 19200, 38400, 57600, 115200};
    const int num_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "GPS Baud Rate Auto-Detection");
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "Testing %d common baud rates...", num_rates);
    ESP_LOGI(TAG, "");
    
    for (int i = 0; i < num_rates; i++) {
        uint32_t baud = baud_rates[i];
        
        ESP_LOGI(TAG, "--- Testing %lu baud ---", (unsigned long)baud);
        
        /* Reconfigure UART to test baud rate */
        uart_config_t uart_config = {
            .baud_rate = baud,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };
        
        esp_err_t ret = uart_param_config(GPS_UART_NUM, &uart_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure UART at %lu baud: %s", 
                     (unsigned long)baud, esp_err_to_name(ret));
            continue;
        }
        
        /* Flush and wait for stable data */
        uart_flush(GPS_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(200));
        
        /* Test this baud rate */
        uint8_t buffer[256];
        uint32_t total_bytes = 0;
        uint32_t nmea_count = 0;
        uint32_t valid_ascii = 0;
        
        uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < 2000) {
            int len = uart_read_bytes(GPS_UART_NUM, buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(100));
            
            if (len > 0) {
                buffer[len] = '\0';
                total_bytes += len;
                
                for (int j = 0; j < len; j++) {
                    uint8_t c = buffer[j];
                    if (c == '$') nmea_count++;
                    if ((c >= 32 && c <= 126) || c == '\r' || c == '\n') valid_ascii++;
                }
                
                /* Show first sample */
                if (total_bytes <= (uint32_t)len && len > 10) {
                    ESP_LOGI(TAG, "  First bytes: %.*s", len < 50 ? len : 50, buffer);
                }
            }
        }
        
        float valid_percent = total_bytes > 0 ? (100.0f * valid_ascii / total_bytes) : 0.0f;
        
        ESP_LOGI(TAG, "  Results: %lu bytes, %lu NMEA ($), %.1f%% valid ASCII",
                 (unsigned long)total_bytes, (unsigned long)nmea_count, valid_percent);
        
        /* Check if this looks good */
        if (total_bytes > 50 && nmea_count > 0 && valid_percent > 80.0f) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "======================================");
            ESP_LOGI(TAG, "✓ FOUND VALID BAUD RATE: %lu", (unsigned long)baud);
            ESP_LOGI(TAG, "======================================");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "ACTION REQUIRED:");
            ESP_LOGI(TAG, "  1. Update GPS_UART_BAUD_RATE in gps_reader.h to %lu", (unsigned long)baud);
            ESP_LOGI(TAG, "  2. OR adjust GPS module DIP switches to match %d baud", GPS_UART_BAUD_RATE);
            ESP_LOGI(TAG, "");
            return baud;
        }
        
        ESP_LOGI(TAG, "");
    }
    
    ESP_LOGW(TAG, "======================================");
    ESP_LOGW(TAG, "✗ No valid baud rate found");
    ESP_LOGW(TAG, "======================================");
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "Possible issues:");
    ESP_LOGW(TAG, "  1. GPS module not connected");
    ESP_LOGW(TAG, "  2. GPS module not powered");
    ESP_LOGW(TAG, "  3. Wrong TX/RX pins (current: TX=%d, RX=%d)", GPS_UART_TX_PIN, GPS_UART_RX_PIN);
    ESP_LOGW(TAG, "  4. Loose connection on M-Bus");
    ESP_LOGW(TAG, "");
    
    return 0;
}

