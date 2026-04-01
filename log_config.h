/*
 * Central Logging Configuration for DJI-Remote
 * 
 * This module provides a centralized way to control logging verbosity
 * across the entire application using ESP-IDF's logging system.
 * 
 * All ESP_LOGx calls throughout the codebase will respect the log level
 * set via app_log_set_level(). The level can be configured at compile time
 * via APP_DEFAULT_LOG_LEVEL or at runtime via app_log_set_level().
 */

#ifndef LOG_CONFIG_H
#define LOG_CONFIG_H

#include <stdint.h>

/**
 * @brief Application log level enumeration
 * 
 * Maps directly to ESP-IDF's esp_log_level_t levels:
 * - APP_LOG_LEVEL_NONE: No logging (ESP_LOG_NONE)
 * - APP_LOG_LEVEL_ERROR: Error messages only (ESP_LOG_ERROR)
 * - APP_LOG_LEVEL_WARN: Warnings and errors (ESP_LOG_WARN)
 * - APP_LOG_LEVEL_INFO: Info, warnings, and errors (ESP_LOG_INFO)
 * - APP_LOG_LEVEL_DEBUG: Debug, info, warnings, and errors (ESP_LOG_DEBUG)
 * - APP_LOG_LEVEL_VERBOSE: All log messages (ESP_LOG_VERBOSE)
 */
typedef enum {
    APP_LOG_LEVEL_NONE = 0,
    APP_LOG_LEVEL_ERROR,
    APP_LOG_LEVEL_WARN,
    APP_LOG_LEVEL_INFO,
    APP_LOG_LEVEL_DEBUG,
    APP_LOG_LEVEL_VERBOSE
} app_log_level_t;

/**
 * @brief Default log level for the application
 * 
 * Can be overridden at build time using -DAPP_DEFAULT_LOG_LEVEL=APP_LOG_LEVEL_XXX
 * Examples:
 *   - Development: APP_LOG_LEVEL_INFO (default)
 *   - Testing: APP_LOG_LEVEL_WARN
 *   - Production: APP_LOG_LEVEL_ERROR or APP_LOG_LEVEL_NONE
 */
#ifndef APP_DEFAULT_LOG_LEVEL
#define APP_DEFAULT_LOG_LEVEL APP_LOG_LEVEL_INFO
#endif

/**
 * @brief Set the global application log level
 * 
 * This function applies the specified log level globally to all ESP_LOGx
 * calls throughout the application by calling esp_log_level_set("*", level).
 * 
 * @param level The desired log level (APP_LOG_LEVEL_NONE through APP_LOG_LEVEL_VERBOSE)
 */
void app_log_set_level(app_log_level_t level);

/**
 * @brief Get the current application log level
 * 
 * @return The currently configured log level
 */
app_log_level_t app_log_get_level(void);

#endif /* LOG_CONFIG_H */

