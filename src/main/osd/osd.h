/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/time.h"

#include "pg/pg.h"

#include "sensors/esc_sensor.h"

#define OSD_NUM_TIMER_TYPES 4
extern const char * const osdTimerSourceNames[OSD_NUM_TIMER_TYPES];

#define OSD_ELEMENT_BUFFER_LENGTH 32

#ifdef USE_OSD_PROFILES
#define OSD_PROFILE_COUNT 3
#else
#define OSD_PROFILE_COUNT 1
#endif

#define OSD_PROFILE_BITS_POS 11
#define OSD_PROFILE_MASK    (((1 << OSD_PROFILE_COUNT) - 1) << OSD_PROFILE_BITS_POS)
#define OSD_POS_MAX   0x3FF
#define OSD_POSCFG_MAX   (OSD_PROFILE_MASK | 0x3FF) // For CLI values
#define OSD_PROFILE_FLAG(x)  (1 << ((x) - 1 + OSD_PROFILE_BITS_POS))
#define OSD_PROFILE_1_FLAG  OSD_PROFILE_FLAG(1)


#ifdef USE_OSD_PROFILES
#define VISIBLE(x) osdElementVisible(x)
#define VISIBLE_IN_OSD_PROFILE(item, profile)    ((item) & ((OSD_PROFILE_1_FLAG) << ((profile)-1)))
#else
#define VISIBLE(x) ((x) & OSD_PROFILE_MASK)
#define VISIBLE_IN_OSD_PROFILE(item, profile) VISIBLE(item)
#endif


// Character coordinate
#define OSD_POSITION_BITS 5 // 5 bits gives a range 0-31
#define OSD_POSITION_XY_MASK ((1 << OSD_POSITION_BITS) - 1)
#define OSD_POS(x,y)  ((x & OSD_POSITION_XY_MASK) | ((y & OSD_POSITION_XY_MASK) << OSD_POSITION_BITS))
#define OSD_X(x)      (x & OSD_POSITION_XY_MASK)
#define OSD_Y(x)      ((x >> OSD_POSITION_BITS) & OSD_POSITION_XY_MASK)

// Timer configuration
// Stored as 15[alarm:8][precision:4][source:4]0
#define OSD_TIMER(src, prec, alarm) ((src & 0x0F) | ((prec & 0x0F) << 4) | ((alarm & 0xFF ) << 8))
#define OSD_TIMER_SRC(timer)        (timer & 0x0F)
#define OSD_TIMER_PRECISION(timer)  ((timer >> 4) & 0x0F)
#define OSD_TIMER_ALARM(timer)      ((timer >> 8) & 0xFF)

// NB: to ensure backwards compatibility, new enum values must be appended at the end but before the OSD_XXXX_COUNT entry.

// *** IMPORTANT ***
// See the information at the top of osd/osd_elements.c for instructions
// on how to add OSD elements.
typedef enum {
    OSD_RSSI_VALUE,
    OSD_MAIN_BATT_VOLTAGE,
    OSD_CROSSHAIRS,
    OSD_ARTIFICIAL_HORIZON,
    OSD_HORIZON_SIDEBARS,
    OSD_ITEM_TIMER_1,
    OSD_ITEM_TIMER_2,
    OSD_FLYMODE,
    OSD_CRAFT_NAME,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
    OSD_GPS_SPEED,
    OSD_GPS_SATS,
    OSD_ALTITUDE,
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_PIDRATE_PROFILE,
    OSD_WARNINGS,
    OSD_AVG_CELL_VOLTAGE,
    OSD_GPS_LON,
    OSD_GPS_LAT,
    OSD_DEBUG,
    OSD_PITCH_ANGLE,
    OSD_ROLL_ANGLE,
    OSD_MAIN_BATT_USAGE,
    OSD_DISARMED,
    OSD_HOME_DIR,
    OSD_HOME_DIST,
    OSD_NUMERICAL_HEADING,
    OSD_NUMERICAL_VARIO,
    OSD_COMPASS_BAR,
    OSD_ESC_TMP,
    OSD_ESC_RPM,
    OSD_REMAINING_TIME_ESTIMATE,
    OSD_RTC_DATETIME,
    OSD_ADJUSTMENT_RANGE,
    OSD_CORE_TEMPERATURE,
    OSD_ANTI_GRAVITY,
    OSD_G_FORCE,
    OSD_MOTOR_DIAG,
    OSD_LOG_STATUS,
    OSD_FLIP_ARROW,
    OSD_LINK_QUALITY,
    OSD_FLIGHT_DIST,
    OSD_STICK_OVERLAY_LEFT,
    OSD_STICK_OVERLAY_RIGHT,
    OSD_DISPLAY_NAME,
    OSD_ESC_RPM_FREQ,
    OSD_ITEM_COUNT // MUST BE LAST
} osd_items_e;

// *** IMPORTANT ***
// If the stats enumeration is reordered then the PR version must be incremented. Otherwise there
// is no indication that the stored config must be reset and the bitmapped values will be incorrect.
//
// The stats display order was previously required to match the enumeration definition so it matched
// the order shown in the configurator. However, to allow reordering this screen without breaking the
// compatibility, this requirement has been relaxed to a best effort approach. Reordering the elements
// on the stats screen will have to be more beneficial than the hassle of not matching exactly to the
// configurator list.
typedef enum {
    OSD_STAT_RTC_DATE_TIME,
    OSD_STAT_TIMER_1,
    OSD_STAT_TIMER_2,
    OSD_STAT_MAX_SPEED,
    OSD_STAT_MAX_DISTANCE,
    OSD_STAT_MIN_BATTERY,
    OSD_STAT_END_BATTERY,
    OSD_STAT_BATTERY,
    OSD_STAT_MIN_RSSI,
    OSD_STAT_MAX_CURRENT,
    OSD_STAT_USED_MAH,
    OSD_STAT_MAX_ALTITUDE,
    OSD_STAT_BLACKBOX,
    OSD_STAT_BLACKBOX_NUMBER,
    OSD_STAT_MAX_G_FORCE,
    OSD_STAT_MAX_ESC_TEMP,
    OSD_STAT_MAX_ESC_RPM,
    OSD_STAT_MIN_LINK_QUALITY,
    OSD_STAT_FLIGHT_DISTANCE,
    OSD_STAT_MAX_FFT,
    OSD_STAT_TOTAL_FLIGHTS,
    OSD_STAT_TOTAL_TIME,
    OSD_STAT_TOTAL_DIST,
    OSD_STAT_COUNT // MUST BE LAST
} osd_stats_e;

// Make sure the number of stats do not exceed the available 32bit storage
STATIC_ASSERT(OSD_STAT_COUNT <= 32, osdstats_overflow);

typedef enum {
    OSD_UNIT_IMPERIAL,
    OSD_UNIT_METRIC
} osd_unit_e;

typedef enum {
    OSD_TIMER_1,
    OSD_TIMER_2,
    OSD_TIMER_COUNT
} osd_timer_e;

typedef enum {
    OSD_TIMER_SRC_ON,
    OSD_TIMER_SRC_TOTAL_ARMED,
    OSD_TIMER_SRC_LAST_ARMED,
    OSD_TIMER_SRC_ON_OR_ARMED,
    OSD_TIMER_SRC_COUNT
} osd_timer_source_e;

typedef enum {
    OSD_TIMER_PREC_SECOND,
    OSD_TIMER_PREC_HUNDREDTHS,
    OSD_TIMER_PREC_COUNT
} osd_timer_precision_e;

typedef enum {
    OSD_WARNING_ARMING_DISABLE,
    OSD_WARNING_BATTERY_NOT_FULL,
    OSD_WARNING_BATTERY_WARNING,
    OSD_WARNING_BATTERY_CRITICAL,
    OSD_WARNING_VISUAL_BEEPER,
    OSD_WARNING_CRASH_FLIP,
    OSD_WARNING_ESC_FAIL,
    OSD_WARNING_CORE_TEMPERATURE,
    OSD_WARNING_RC_SMOOTHING,
    OSD_WARNING_FAIL_SAFE,
    OSD_WARNING_LAUNCH_CONTROL,
    OSD_WARNING_GPS_RESCUE_UNAVAILABLE,
    OSD_WARNING_GPS_RESCUE_DISABLED,
    OSD_WARNING_RSSI,
    OSD_WARNING_LINK_QUALITY,
    OSD_WARNING_COUNT // MUST BE LAST
} osdWarningsFlags_e;

// Make sure the number of warnings do not exceed the available 32bit storage
STATIC_ASSERT(OSD_WARNING_COUNT <= 32, osdwarnings_overflow);

#define ESC_RPM_ALARM_OFF -1
#define ESC_TEMP_ALARM_OFF INT8_MIN
#define ESC_CURRENT_ALARM_OFF -1

#define OSD_GPS_RESCUE_DISABLED_WARNING_DURATION_US 3000000 // 3 seconds

extern const uint16_t osdTimerDefault[OSD_TIMER_COUNT];

typedef struct osdConfig_s {
    uint16_t item_pos[OSD_ITEM_COUNT];

    // Alarms
    uint16_t cap_alarm;
    uint16_t alt_alarm;
    uint8_t rssi_alarm;

    osd_unit_e units;

    uint16_t timers[OSD_TIMER_COUNT];
    uint32_t enabledWarnings;

    uint8_t ahMaxPitch;
    uint8_t ahMaxRoll;
    uint32_t enabled_stats;
    int8_t esc_temp_alarm;
    int16_t esc_rpm_alarm;
    int16_t esc_current_alarm;
    uint8_t core_temp_alarm;
    uint8_t ahInvert;         // invert the artificial horizon
    uint8_t osdProfileIndex;
    uint8_t overlay_radio_mode;
    uint8_t link_quality_alarm;
} osdConfig_t;

PG_DECLARE(osdConfig_t, osdConfig);

typedef struct statistic_s {
    timeUs_t armed_time;
    int16_t max_speed;
    int16_t min_voltage; // /100
    int16_t max_current; // /10
    uint8_t min_rssi;
    int32_t max_altitude;
    int16_t max_distance;
    float max_g_force;
    int16_t max_esc_temp;
    int32_t max_esc_rpm;
    uint8_t min_link_quality;
} statistic_t;

extern timeUs_t resumeRefreshAt;
extern timeUs_t osdFlyTime;
#if defined(USE_ACC)
extern float osdGForce;
#endif
#ifdef USE_ESC_SENSOR
extern escSensorData_t *osdEscDataCombined;
#endif


struct displayPort_s;
void osdInit(struct displayPort_s *osdDisplayPort);
bool osdInitialized(void);
void osdUpdate(timeUs_t currentTimeUs);
void osdStatSetState(uint8_t statIndex, bool enabled);
bool osdStatGetState(uint8_t statIndex);
void osdWarnSetState(uint8_t warningIndex, bool enabled);
bool osdWarnGetState(uint8_t warningIndex);
void osdSuppressStats(bool flag);

uint8_t getCurrentOsdProfileIndex(void);
void changeOsdProfileIndex(uint8_t profileIndex);
bool osdElementVisible(uint16_t value);
bool osdGetVisualBeeperState(void);
statistic_t *osdGetStats(void);
