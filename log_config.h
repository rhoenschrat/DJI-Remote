/*
 * Central Logging Configuration for DJI-Remote
 *
 * This module provides a centralized way to control logging verbosity
 * across the entire application using ESP-IDF's logging system.
 *
 * All ESP_LOGx calls throughout the codebase respect the level set via
 * app_log_set_level(). The level is chosen at build time via
 * APP_DEFAULT_LOG_LEVEL (default: ERROR) and applied once at boot.
 *
 * CONFIG_LOG_MAXIMUM_LEVEL_DEBUG is set in the board defaults, so ESP_LOGD
 * is compiled in and DEBUG can be enabled per tag at runtime.
 *
 * Build profiles:
 *   - Release:     APP_DEFAULT_LOG_LEVEL=APP_LOG_LEVEL_ERROR (default)
 *   - Field log:   APP_DEFAULT_LOG_LEVEL=APP_LOG_LEVEL_INFO
 *                  (one line per state change or user action)
 *   - Diagnostics: field log plus APP_BLE_DIAG_LOG=1
 *                  (one BLE_DIAG line per paired slot every 10 s)
 *
 * At INFO and above, NimBLE's own per-procedure INFO lines (tag "NimBLE",
 * e.g. "GATT procedure initiated") are filtered to WARN at runtime.
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
 *   - Release (default): APP_LOG_LEVEL_ERROR
 *   - Field logging:     APP_LOG_LEVEL_INFO
 *   - Testing:           APP_LOG_LEVEL_WARN
 */
#ifndef APP_DEFAULT_LOG_LEVEL
#define APP_DEFAULT_LOG_LEVEL APP_LOG_LEVEL_ERROR
#endif

/**
 * @brief Per-slot BLE diagnostics line (tag APP_LOG_TAG_BLE_DIAG)
 *
 * 1 enables DEBUG for the diagnostics tag; link_evt_task then logs one line
 * per paired slot every 10 s. Can be overridden with -DAPP_BLE_DIAG_LOG=1.
 */
#ifndef APP_BLE_DIAG_LOG
#define APP_BLE_DIAG_LOG 0
#endif

/** Log tag of the per-slot BLE diagnostics line */
#define APP_LOG_TAG_BLE_DIAG "BLE_DIAG"

/** NimBLE host log tag (filtered to WARN at INFO and above) */
#define APP_LOG_TAG_NIMBLE "NimBLE"

/**
 * @brief Set the global application log level
 * 
 * This function applies the specified log level globally to all ESP_LOGx
 * calls throughout the application by calling esp_log_level_set("*", level),
 * then applies the per-tag overrides (NimBLE filter, BLE_DIAG switch).
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

