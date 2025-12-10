/*
 * Central Logging Configuration Implementation
 * 
 * Implements the logging configuration functions that control ESP-IDF's
 * global log level via esp_log_level_set("*", level).
 */

#include "log_config.h"
#include "esp_log.h"

/* Current application log level */
static app_log_level_t s_app_log_level = APP_DEFAULT_LOG_LEVEL;

void app_log_set_level(app_log_level_t level) {
    /* Store the level */
    s_app_log_level = level;
    
    /* Map app_log_level_t to esp_log_level_t */
    esp_log_level_t esp_level;
    switch (level) {
        case APP_LOG_LEVEL_NONE:
            esp_level = ESP_LOG_NONE;
            break;
        case APP_LOG_LEVEL_ERROR:
            esp_level = ESP_LOG_ERROR;
            break;
        case APP_LOG_LEVEL_WARN:
            esp_level = ESP_LOG_WARN;
            break;
        case APP_LOG_LEVEL_INFO:
            esp_level = ESP_LOG_INFO;
            break;
        case APP_LOG_LEVEL_DEBUG:
            esp_level = ESP_LOG_DEBUG;
            break;
        case APP_LOG_LEVEL_VERBOSE:
            esp_level = ESP_LOG_VERBOSE;
            break;
        default:
            /* Fallback to INFO if invalid level */
            esp_level = ESP_LOG_INFO;
            s_app_log_level = APP_LOG_LEVEL_INFO;
            break;
    }
    
    /* Apply globally to all tags using wildcard "*" */
    esp_log_level_set("*", esp_level);
}

app_log_level_t app_log_get_level(void) {
    return s_app_log_level;
}

