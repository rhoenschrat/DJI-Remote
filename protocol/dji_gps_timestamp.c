#include "dji_gps_timestamp.h"

#include <time.h>

/* Hours added to GPS UTC before encoding (ADR-023).
 *
 * 0 = plain UTC. DJI's demo adds 8 (Beijing time), but hardware tests on
 * Osmo Action 5 Pro, Osmo Action 6 and Osmo 360 (2026-10-19) showed that the
 * cameras do not remove that offset: they store "received time - 1 h" as
 * GPSDateTime in the clip, whatever value is sent. Sending plain UTC keeps
 * the wire value honest and the stored time closest to UTC. The offset is
 * applied to the epoch, so any value gets correct date/month/year rollover. */
#define DJI_GPS_TS_OFFSET_HOURS 0

void dji_gps_encode_timestamp(uint32_t utc_epoch, int32_t *year_month_day,
                              int32_t *hour_minute_second)
{
    struct tm tm;
    time_t t = (time_t)utc_epoch + (time_t)DJI_GPS_TS_OFFSET_HOURS * 3600;

    if (gmtime_r(&t, &tm) == NULL) {
        *year_month_day = 0;
        *hour_minute_second = 0;
        return;
    }
    *year_month_day = (int32_t)((tm.tm_year + 1900) * 10000 +
                                (tm.tm_mon + 1) * 100 + tm.tm_mday);
    *hour_minute_second = (int32_t)(tm.tm_hour * 10000 + tm.tm_min * 100 + tm.tm_sec);
}
