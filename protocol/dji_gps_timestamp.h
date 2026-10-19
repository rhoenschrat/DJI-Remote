#ifndef DJI_GPS_TIMESTAMP_H
#define DJI_GPS_TIMESTAMP_H

#include <stdint.h>

/**
 * @brief Encode a GPS UTC epoch into the two DJI GPS push time fields
 *
 * Wire format of the GPS data push (CmdSet 0x00, CmdID 0x17):
 *   - year_month_day     = Y*10000 + M*100 + D      (UTC date)
 *   - hour_minute_second = h*10000 + min*100 + sec  (UTC time, hour 0..23)
 *
 * Plain UTC, not DJI's demo encoding (UTC hour + 8): hardware tests showed
 * the cameras do not remove the +8. The offset constant in
 * dji_gps_timestamp.c is 0. See ADR-023.
 *
 * Pure arithmetic: no state, no logging, no locks; safe from any task.
 *
 * @param utc_epoch          Seconds since 1970-01-01 00:00:00 UTC
 * @param year_month_day     Output: encoded date field
 * @param hour_minute_second Output: encoded time field
 */
void dji_gps_encode_timestamp(uint32_t utc_epoch, int32_t *year_month_day,
                              int32_t *hour_minute_second);

#endif /* DJI_GPS_TIMESTAMP_H */
