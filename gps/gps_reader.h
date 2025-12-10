/*
 * GPS Reader Module for M5Stack Module GPS v2.0
 * 
 * This module provides GPS functionality using the M5Stack Module GPS v2.0
 * (u-blox M8130K) connected via UART on the M-Bus connector.
 * 
 * Features:
 * - UART2 communication with GPS module
 * - NMEA sentence parsing (GPGGA, GPRMC, GPGSV)
 * - Thread-safe GPS data access
 * - Automatic fix detection
 * - Satellite count tracking
 * 
 * Hardware: M5Stack Module GPS v2.0 (u-blox M8130K)
 * Connection: M-Bus connector (UART2)
 * Protocol: NMEA 0183
 * Baud Rate: 9600 or 115200 (configurable)
 */

#ifndef GPS_READER_H
#define GPS_READER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* GPS Data Structure
 * Contains all parsed GPS information from NMEA sentences
 */
typedef struct {
    float latitude;          /* Latitude in degrees (positive = North) */
    float longitude;         /* Longitude in degrees (positive = East) */
    float altitude;         /* Altitude in meters above sea level */
    float speed;            /* Ground speed in m/s */
    float course;           /* Course over ground in degrees (0-360) */
    uint8_t satellite_count; /* Number of satellites in view */
    bool has_fix;           /* True if GPS has valid fix */
    uint32_t timestamp;     /* Last update timestamp (milliseconds) */
    uint8_t fix_quality;    /* GPS fix quality (0=invalid, 1=GPS, 2=DGPS) */
    float hdop;             /* Horizontal Dilution of Precision (from GPGGA) */
    float vdop;             /* Vertical DOP (from GPGSA, 0.0 if unavailable) */
    float pdop;             /* Position DOP (from GPGSA, 0.0 if unavailable) */
    uint8_t satellites_used; /* Number of satellites used for fix (from GPGSA, 0 if unavailable) */
    uint8_t fix_type;       /* Fix type: 0=No Fix, 1=2D, 2=3D (from GPGSA or inferred) */
} gps_data_t;

/* GPS Module Configuration */
#define GPS_UART_NUM        UART_NUM_2
#define GPS_UART_BAUD_RATE  115200      /* Baud rate for M5Stack GPS Module v2.0 (verified via auto-detection) */
#define GPS_UART_TX_PIN     17          /* TX pin (configure via GPS module DIP switch) */
#define GPS_UART_RX_PIN     16          /* RX pin (configure via GPS module DIP switch) */
#define GPS_UART_BUF_SIZE   1024        /* UART buffer size */
#define GPS_NMEA_MAX_LEN    128         /* Maximum NMEA sentence length */

/**
 * @brief Initialize GPS module
 * 
 * Initializes UART2 for communication with the GPS module and starts
 * the NMEA parsing task.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t gps_init(void);

/**
 * @brief Deinitialize GPS module
 * 
 * Stops the GPS parsing task and releases UART resources.
 * 
 * @return ESP_OK on success
 */
esp_err_t gps_deinit(void);

/**
 * @brief Check if GPS has a valid fix
 * 
 * @return true if GPS has valid fix, false otherwise
 */
bool gps_has_fix(void);

/**
 * @brief Get current GPS data
 * 
 * Retrieves the latest GPS data in a thread-safe manner.
 * 
 * @param data Pointer to GPS data structure to fill
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t gps_get_data(gps_data_t *data);

/**
 * @brief Get latitude
 * 
 * @return Latitude in degrees (positive = North)
 */
float gps_get_latitude(void);

/**
 * @brief Get longitude
 * 
 * @return Longitude in degrees (positive = East)
 */
float gps_get_longitude(void);

/**
 * @brief Get altitude
 * 
 * @return Altitude in meters above sea level
 */
float gps_get_altitude(void);

/**
 * @brief Get speed
 * 
 * @return Ground speed in m/s
 */
float gps_get_speed(void);

/**
 * @brief Get satellite count
 * 
 * @return Number of satellites in view
 */
uint8_t gps_get_satellite_count(void);

/**
 * @brief Get course over ground
 * 
 * @return Course in degrees (0-360)
 */
float gps_get_course(void);

/**
 * @brief Read raw NMEA sentence from GPS module
 * 
 * Reads a complete NMEA sentence from the UART buffer.
 * This is primarily for debugging purposes.
 * 
 * @param buffer Buffer to store NMEA sentence
 * @param buffer_size Size of buffer
 * @return Number of bytes read, or -1 on error
 */
int gps_read_nmea(char *buffer, size_t buffer_size);

/**
 * @brief Get human-readable fix type string
 * 
 * @param fix_type Fix type value (0=No Fix, 1=2D, 2=3D)
 * @return Human-readable string
 */
const char* gps_get_fix_type_string(uint8_t fix_type);

/**
 * @brief Get milliseconds since last GPS update
 * 
 * @return Milliseconds since last GPS data update, or UINT32_MAX if no data received
 */
uint32_t gps_get_last_update_age_ms(void);

/**
 * @brief Test GPS communication at current baud rate
 * 
 * Reads data from the GPS module for a specified duration and analyzes
 * whether the data looks like valid NMEA sentences. This helps diagnose
 * baud rate mismatches.
 * 
 * @param duration_ms Duration to test in milliseconds (default: 3000ms)
 * @return true if valid NMEA data detected, false if corrupted/invalid data
 */
bool gps_test_communication(uint32_t duration_ms);

/**
 * @brief Auto-detect GPS module baud rate
 * 
 * Tests common baud rates (9600, 19200, 38400, 57600, 115200) to find
 * which one the GPS module is using. This helps when DIP switch settings
 * are unknown.
 * 
 * @return Detected baud rate, or 0 if detection failed
 */
uint32_t gps_auto_detect_baud_rate(void);

#endif /* GPS_READER_H */

