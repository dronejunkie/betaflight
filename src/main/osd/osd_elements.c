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

/*
    *****************************************
    Instructions for adding new OSD Elements:
    *****************************************

    First add the new element to the osd_items_e enumeration in osd/osd.h. The
    element must be added to the end just before OSD_ITEM_COUNT.

    Next add the element to the osdElementDisplayOrder array defined in this file.
    If the element needs special runtime conditional processing then it should be added
    to the osdAnalyzeActiveElements() function instead.

    Create the function to "draw" the element. It should be named like "osdElementSomething()"
    where the "Something" describes the element.

    Finally add the mapping from the element ID added in the first step to the function
    created in the third step to the osdElementDrawFunction array.
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#ifdef USE_OSD

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/display.h"
#include "drivers/max7456_symbols.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "fc/config.h"
#include "fc/core.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/failsafe.h"
#include "flight/position.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/vtx.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"
#include "sensors/sensors.h"
#include "sensors/rpm_filter.h"


#define AH_SYMBOL_COUNT 9
#define AH_SIDEBAR_WIDTH_POS 7
#define AH_SIDEBAR_HEIGHT_POS 3

// Stick overlay size
#define OSD_STICK_OVERLAY_WIDTH 7
#define OSD_STICK_OVERLAY_HEIGHT 5
#define OSD_STICK_OVERLAY_SPRITE_HEIGHT 3
#define OSD_STICK_OVERLAY_VERTICAL_POSITIONS (OSD_STICK_OVERLAY_HEIGHT * OSD_STICK_OVERLAY_SPRITE_HEIGHT)

#define FULL_CIRCLE 360

#ifdef USE_OSD_STICK_OVERLAY
typedef struct radioControls_s {
    uint8_t left_vertical;
    uint8_t left_horizontal;
    uint8_t right_vertical;
    uint8_t right_horizontal;
} radioControls_t;

static const radioControls_t radioModes[4] = {
    { PITCH,    YAW,    THROTTLE,   ROLL }, // Mode 1
    { THROTTLE, YAW,    PITCH,      ROLL }, // Mode 2
    { PITCH,    ROLL,   THROTTLE,   YAW  }, // Mode 3
    { THROTTLE, ROLL,   PITCH,      YAW  }, // Mode 4
};
#endif

static const char compassBar[] = {
  SYM_HEADING_W,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_N,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_E,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_S,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_W,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_N,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE
};

static unsigned activeOsdElementCount = 0;
static uint8_t activeOsdElementArray[OSD_ITEM_COUNT];

// Blink control
static bool blinkState = true;
static uint32_t blinkBits[(OSD_ITEM_COUNT + 31) / 32];
#define SET_BLINK(item) (blinkBits[(item) / 32] |= (1 << ((item) % 32)))
#define CLR_BLINK(item) (blinkBits[(item) / 32] &= ~(1 << ((item) % 32)))
#define IS_BLINK(item) (blinkBits[(item) / 32] & (1 << ((item) % 32)))
#define BLINK(item) (IS_BLINK(item) && blinkState)

#if defined(USE_ESC_SENSOR) || defined(USE_RPM_FILTER)
typedef int (*getEscRpmOrFreqFnPtr)(int i);

static int getEscRpm(int i)
{
#ifdef USE_RPM_FILTER
    if (motorConfig()->dev.useDshotTelemetry) {
        return 100.0f / (motorConfig()->motorPoleCount / 2.0f) * getDshotTelemetry(i);
    }
#endif 
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        return calcEscRpm(getEscSensorData(i)->rpm);
    } else { 
        return 0;
    }
}

static int getEscRpmFreq(int i) 
{
    return getEscRpm(i) / 60;
}

static void renderOsdEscRpmOrFreq(getEscRpmOrFreqFnPtr escFnPtr, osdElementParms_t *element)
{
    int x = element->elemPosX;
    int y = element->elemPosY;
    for (int i=0; i < getMotorCount(); i++) {
        char rpmStr[6];
        const int rpm = MIN((*escFnPtr)(i),99999);
        const int len = tfp_sprintf(rpmStr, "%d", rpm);
        rpmStr[len] = '\0'; 
        displayWrite(element->osdDisplayPort, x, y + i, rpmStr);
    }
    element->drawElement = false;
}
#endif

#if defined(USE_ADC_INTERNAL) || defined(USE_ESC_SENSOR)
int osdConvertTemperatureToSelectedUnit(int tempInDegreesCelcius)
{
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        return lrintf(((tempInDegreesCelcius * 9.0f) / 5) + 32);
    default:
        return tempInDegreesCelcius;
    }
}
#endif

static void osdFormatAltitudeString(char * buff, int32_t altitudeCm)
{
    const int alt = osdGetMetersToSelectedUnit(altitudeCm) / 10;

    tfp_sprintf(buff, "%5d %c", alt, osdGetMetersToSelectedUnitSymbol());
    buff[5] = buff[4];
    buff[4] = '.';
}

#ifdef USE_GPS
static void osdFormatCoordinate(char *buff, char sym, int32_t val)
{
    // latitude maximum integer width is 3 (-90).
    // longitude maximum integer width is 4 (-180).
    // We show 7 decimals, so we need to use 12 characters:
    // eg: s-180.1234567z   s=symbol, z=zero terminator, decimal separator  between 0 and 1

    static const int coordinateMaxLength = 13;//12 for the number (4 + dot + 7) + 1 for the symbol

    buff[0] = sym;
    const int32_t integerPart = val / GPS_DEGREES_DIVIDER;
    const int32_t decimalPart = labs(val % GPS_DEGREES_DIVIDER);
    const int written = tfp_sprintf(buff + 1, "%d.%07d", integerPart, decimalPart);
    // pad with blanks to coordinateMaxLength
    for (int pos = 1 + written; pos < coordinateMaxLength; ++pos) {
        buff[pos] = SYM_BLANK;
    }
    buff[coordinateMaxLength] = '\0';
}
#endif // USE_GPS

static void osdFormatMessage(char *buff, size_t size, const char *message)
{
    memset(buff, SYM_BLANK, size);
    if (message) {
        memcpy(buff, message, strlen(message));
    }
    // Ensure buff is zero terminated
    buff[size - 1] = '\0';
}

static void osdFormatPID(char * buff, const char * label, const pidf_t * pid)
{
    tfp_sprintf(buff, "%s %3d %3d %3d", label, pid->P, pid->I, pid->D);
}

#ifdef USE_RTC_TIME
bool osdFormatRtcDateTime(char *buffer)
{
    dateTime_t dateTime;
    if (!rtcGetDateTime(&dateTime)) {
        buffer[0] = '\0';

        return false;
    }

    dateTimeFormatLocalShort(buffer, &dateTime);

    return true;
}
#endif

void osdFormatTime(char * buff, osd_timer_precision_e precision, timeUs_t time)
{
    int seconds = time / 1000000;
    const int minutes = seconds / 60;
    seconds = seconds % 60;

    switch (precision) {
    case OSD_TIMER_PREC_SECOND:
    default:
        tfp_sprintf(buff, "%02d:%02d", minutes, seconds);
        break;
    case OSD_TIMER_PREC_HUNDREDTHS:
        {
            const int hundredths = (time / 10000) % 100;
            tfp_sprintf(buff, "%02d:%02d.%02d", minutes, seconds, hundredths);
            break;
        }
    }
}

static char osdGetTimerSymbol(osd_timer_source_e src)
{
    switch (src) {
    case OSD_TIMER_SRC_ON:
        return SYM_ON_M;
    case OSD_TIMER_SRC_TOTAL_ARMED:
    case OSD_TIMER_SRC_LAST_ARMED:
        return SYM_FLY_M;
    default:
        return ' ';
    }
}

static timeUs_t osdGetTimerValue(osd_timer_source_e src)
{
    switch (src) {
    case OSD_TIMER_SRC_ON:
        return micros();
    case OSD_TIMER_SRC_TOTAL_ARMED:
        return osdFlyTime;
    case OSD_TIMER_SRC_LAST_ARMED: {
        statistic_t *stats = osdGetStats();
        return stats->armed_time;
    }
    default:
        return 0;
    }
}

void osdFormatTimer(char *buff, bool showSymbol, bool usePrecision, int timerIndex)
{
    const uint16_t timer = osdConfig()->timers[timerIndex];
    const uint8_t src = OSD_TIMER_SRC(timer);

    if (showSymbol) {
        *(buff++) = osdGetTimerSymbol(src);
    }

    osdFormatTime(buff, (usePrecision ? OSD_TIMER_PRECISION(timer) : OSD_TIMER_PREC_SECOND), osdGetTimerValue(src));
}

static char osdGetBatterySymbol(int cellVoltage)
{
    if (getBatteryState() == BATTERY_CRITICAL) {
        return SYM_MAIN_BATT; // FIXME: currently the BAT- symbol, ideally replace with a battery with exclamation mark
    } else {
        // Calculate a symbol offset using cell voltage over full cell voltage range
        const int symOffset = scaleRange(cellVoltage, batteryConfig()->vbatmincellvoltage, batteryConfig()->vbatmaxcellvoltage, 0, 8);
        return SYM_BATT_EMPTY - constrain(symOffset, 0, 6);
    }
}

static uint8_t osdGetHeadingIntoDiscreteDirections(int heading, unsigned directions)
{
    heading += FULL_CIRCLE;  // Ensure positive value

    // Split input heading 0..359 into sectors 0..(directions-1), but offset
    // by half a sector so that sector 0 gets centered around heading 0.
    // We multiply heading by directions to not loose precision in divisions
    // In this way each segment will be a FULL_CIRCLE length
    int direction = (heading * directions + FULL_CIRCLE / 2) /  FULL_CIRCLE; // scale with rounding
    direction %= directions; // normalize

    return direction; // return segment number
}

static uint8_t osdGetDirectionSymbolFromHeading(int heading)
{
    heading = osdGetHeadingIntoDiscreteDirections(heading, 16);

    // Now heading has a heading with Up=0, Right=4, Down=8 and Left=12
    // Our symbols are Down=0, Right=4, Up=8 and Left=12
    // There're 16 arrow symbols. Transform it.
    heading = 16 - heading;
    heading = (heading + 8) % 16;

    return SYM_ARROW_SOUTH + heading;
}


/**
 * Converts altitude based on the current unit system.
 * @param meters Value in meters to convert
 */
int32_t osdGetMetersToSelectedUnit(int32_t meters)
{
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        return (meters * 328) / 100; // Convert to feet / 100
    default:
        return meters;               // Already in metre / 100
    }
}

/**
 * Gets the correct altitude symbol for the current unit system
 */
char osdGetMetersToSelectedUnitSymbol(void)
{
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        return SYM_FT;
    default:
        return SYM_M;
    }
}

/**
 * Converts speed based on the current unit system.
 * @param value in cm/s to convert
 */
int32_t osdGetSpeedToSelectedUnit(int32_t value)
{
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        return CM_S_TO_MPH(value);
    default:
        return CM_S_TO_KM_H(value);
    }
}

/**
 * Gets the correct speed symbol for the current unit system
 */
char osdGetSpeedToSelectedUnitSymbol(void)
{
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        return SYM_MPH;
    default:
        return SYM_KPH;
    }
}

#if defined(USE_ADC_INTERNAL) || defined(USE_ESC_SENSOR)
char osdGetTemperatureSymbolForSelectedUnit(void)
{
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        return SYM_F;
    default:
        return SYM_C;
    }
}
#endif

// *************************
// Element drawing functions
// *************************

#ifdef USE_OSD_ADJUSTMENTS
static void osdElementAdjustmentRange(osdElementParms_t *element)
{
    if (getAdjustmentsRangeName()) {
        tfp_sprintf(element->buff, "%s: %3d", getAdjustmentsRangeName(), getAdjustmentsRangeValue());
    }
}
#endif // USE_OSD_ADJUSTMENTS

static void osdElementAltitude(osdElementParms_t *element)
{
    bool haveBaro = false;
    bool haveGps = false;
#ifdef USE_BARO
    haveBaro = sensors(SENSOR_BARO);
#endif // USE_BARO
#ifdef USE_GPS
    haveGps = sensors(SENSOR_GPS) && STATE(GPS_FIX);
#endif // USE_GPS
    if (haveBaro || haveGps) {
        osdFormatAltitudeString(element->buff, getEstimatedAltitudeCm());
    } else {
        // We use this symbol when we don't have a valid measure
        element->buff[0] = SYM_COLON;
        // overwrite any previous altitude with blanks
        memset(element->buff + 1, SYM_BLANK, 6);
        element->buff[7] = '\0';
    }
}

#ifdef USE_ACC
static void osdElementAngleRollPitch(osdElementParms_t *element)
{
    const int angle = (element->item == OSD_PITCH_ANGLE) ? attitude.values.pitch : attitude.values.roll;
    tfp_sprintf(element->buff, "%c%02d.%01d", angle < 0 ? '-' : ' ', abs(angle / 10), abs(angle % 10));
}
#endif

static void osdElementAntiGravity(osdElementParms_t *element)
{
    if (pidOsdAntiGravityActive()) {
        strcpy(element->buff, "AG");
    }
}

#ifdef USE_ACC
static void osdElementArtificialHorizon(osdElementParms_t *element)
{
    // Get pitch and roll limits in tenths of degrees
    const int maxPitch = osdConfig()->ahMaxPitch * 10;
    const int maxRoll = osdConfig()->ahMaxRoll * 10;
    const int ahSign = osdConfig()->ahInvert ? -1 : 1;
    const int rollAngle = constrain(attitude.values.roll * ahSign, -maxRoll, maxRoll);
    int pitchAngle = constrain(attitude.values.pitch * ahSign, -maxPitch, maxPitch);
    // Convert pitchAngle to y compensation value
    // (maxPitch / 25) divisor matches previous settings of fixed divisor of 8 and fixed max AHI pitch angle of 20.0 degrees
    if (maxPitch > 0) {
        pitchAngle = ((pitchAngle * 25) / maxPitch);
    }
    pitchAngle -= 41; // 41 = 4 * AH_SYMBOL_COUNT + 5

    for (int x = -4; x <= 4; x++) {
        const int y = ((-rollAngle * x) / 64) - pitchAngle;
        if (y >= 0 && y <= 81) {
            displayWriteChar(element->osdDisplayPort, element->elemPosX + x, element->elemPosY + (y / AH_SYMBOL_COUNT), (SYM_AH_BAR9_0 + (y % AH_SYMBOL_COUNT)));
        }
    }

    element->drawElement = false;  // element already drawn
}
#endif // USE_ACC

static void osdElementAverageCellVoltage(osdElementParms_t *element)
{
    const int cellV = getBatteryAverageCellVoltage();
    element->buff[0] = osdGetBatterySymbol(cellV);
    tfp_sprintf(element->buff + 1, "%d.%02d%c", cellV / 100, cellV % 100, SYM_VOLT);
}

static void osdElementCompassBar(osdElementParms_t *element)
{
    memcpy(element->buff, compassBar + osdGetHeadingIntoDiscreteDirections(DECIDEGREES_TO_DEGREES(attitude.values.yaw), 16), 9);
    element->buff[9] = 0;
}

#ifdef USE_ADC_INTERNAL
static void osdElementCoreTemperature(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%3d%c", osdConvertTemperatureToSelectedUnit(getCoreTemperatureCelsius()), osdGetTemperatureSymbolForSelectedUnit());
}
#endif // USE_ADC_INTERNAL

static void osdElementCraftName(osdElementParms_t *element)
{
    // This does not strictly support iterative updating if the craft name changes at run time. But since the craft name is not supposed to be changing this should not matter, and blanking the entire length of the craft name string on update will make it impossible to configure elements to be displayed on the right hand side of the craft name.
    //TODO: When iterative updating is implemented, change this so the craft name is only printed once whenever the OSD 'flight' screen is entered.

    if (strlen(pilotConfig()->name) == 0) {
        strcpy(element->buff, "CRAFT_NAME");
    } else {
        unsigned i;
        for (i = 0; i < MAX_NAME_LENGTH; i++) {
            if (pilotConfig()->name[i]) {
                element->buff[i] = toupper((unsigned char)pilotConfig()->name[i]);
            } else {
                break;
            }
        }
        element->buff[i] = '\0';
    }
}

#ifdef USE_ACC
static void osdElementCrashFlipArrow(osdElementParms_t *element)
{
    int rollAngle = attitude.values.roll / 10;
    const int pitchAngle = attitude.values.pitch / 10;
    if (abs(rollAngle) > 90) {
        rollAngle = (rollAngle < 0 ? -180 : 180) - rollAngle;
    }

    if ((isFlipOverAfterCrashActive() || (!ARMING_FLAG(ARMED) && !STATE(SMALL_ANGLE))) && !((imuConfig()->small_angle < 180) && STATE(SMALL_ANGLE)) && (rollAngle || pitchAngle)) {
        if (abs(pitchAngle) < 2 * abs(rollAngle) && abs(rollAngle) < 2 * abs(pitchAngle)) {
            if (pitchAngle > 0) {
                if (rollAngle > 0) {
                    element->buff[0] = SYM_ARROW_WEST + 2;
                } else {
                    element->buff[0] = SYM_ARROW_EAST - 2;
                }
            } else {
                if (rollAngle > 0) {
                    element->buff[0] = SYM_ARROW_WEST - 2;
                } else {
                    element->buff[0] = SYM_ARROW_EAST + 2;
                }
            }
        } else {
            if (abs(pitchAngle) > abs(rollAngle)) {
                if (pitchAngle > 0) {
                    element->buff[0] = SYM_ARROW_SOUTH;
                } else {
                    element->buff[0] = SYM_ARROW_NORTH;
                }
            } else {
                if (rollAngle > 0) {
                    element->buff[0] = SYM_ARROW_WEST;
                } else {
                    element->buff[0] = SYM_ARROW_EAST;
                }
            }
        }
    } else {
        element->buff[0] = ' ';
    }

    element->buff[1] = '\0';
}
#endif // USE_ACC

static void osdElementCrosshairs(osdElementParms_t *element)
{
    element->buff[0] = SYM_AH_CENTER_LINE;
    element->buff[1] = SYM_AH_CENTER;
    element->buff[2] = SYM_AH_CENTER_LINE_RIGHT;
    element->buff[3] = 0;
}

static void osdElementCurrentDraw(osdElementParms_t *element)
{
    const int32_t amperage = getAmperage();
    tfp_sprintf(element->buff, "%3d.%02d%c", abs(amperage) / 100, abs(amperage) % 100, SYM_AMP);
}

static void osdElementDebug(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "DBG %5d %5d %5d %5d", debug[0], debug[1], debug[2], debug[3]);
}

static void osdElementDisarmed(osdElementParms_t *element)
{
    if (!ARMING_FLAG(ARMED)) {
        tfp_sprintf(element->buff, "DISARMED");
    }
}

static void osdElementDisplayName(osdElementParms_t *element)
{
    // This does not strictly support iterative updating if the display name changes at run time. But since the display name is not supposed to be changing this should not matter, and blanking the entire length of the display name string on update will make it impossible to configure elements to be displayed on the right hand side of the display name.
    //TODO: When iterative updating is implemented, change this so the display name is only printed once whenever the OSD 'flight' screen is entered.

    if (strlen(pilotConfig()->displayName) == 0) {
        strcpy(element->buff, "DISPLAY_NAME");
    } else {
        unsigned i;
        for (i = 0; i < MAX_NAME_LENGTH; i++) {
            if (pilotConfig()->displayName[i]) {
                element->buff[i] = toupper((unsigned char)pilotConfig()->displayName[i]);
            } else {
                break;
            }
        }
        element->buff[i] = '\0';
    }
}

#ifdef USE_ESC_SENSOR
static void osdElementEscTemperature(osdElementParms_t *element)
{
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        tfp_sprintf(element->buff, "%3d%c", osdConvertTemperatureToSelectedUnit(osdEscDataCombined->temperature), osdGetTemperatureSymbolForSelectedUnit());
    }
}
#endif // USE_ESC_SENSOR

#if defined(USE_ESC_SENSOR) || defined(USE_RPM_FILTER)
static void osdElementEscRpm(osdElementParms_t *element)
{
    renderOsdEscRpmOrFreq(&getEscRpm,element);
}

static void osdElementEscRpmFreq(osdElementParms_t *element)
{
    renderOsdEscRpmOrFreq(&getEscRpmFreq,element);
}
#endif

static void osdElementFlymode(osdElementParms_t *element)
{
    // Note that flight mode display has precedence in what to display.
    //  1. FS
    //  2. GPS RESCUE
    //  3. ANGLE, HORIZON, ACRO TRAINER
    //  4. AIR
    //  5. ACRO

    if (FLIGHT_MODE(FAILSAFE_MODE)) {
        strcpy(element->buff, "!FS!");
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        strcpy(element->buff, "RESC");
    } else if (FLIGHT_MODE(HEADFREE_MODE)) {
        strcpy(element->buff, "HEAD");
    } else if (FLIGHT_MODE(ANGLE_MODE)) {
        strcpy(element->buff, "STAB");
    } else if (FLIGHT_MODE(HORIZON_MODE)) {
        strcpy(element->buff, "HOR ");
    } else if (IS_RC_MODE_ACTIVE(BOXACROTRAINER)) {
        strcpy(element->buff, "ATRN");
    } else if (airmodeIsEnabled()) {
        strcpy(element->buff, "AIR ");
    } else {
        strcpy(element->buff, "ACRO");
    }
}

#ifdef USE_ACC
static void osdElementGForce(osdElementParms_t *element)
{
    const int gForce = lrintf(osdGForce * 10);
    tfp_sprintf(element->buff, "%01d.%01dG", gForce / 10, gForce % 10);
}
#endif // USE_ACC

#ifdef USE_GPS
static void osdElementGpsFlightDistance(osdElementParms_t *element)
{
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        const int32_t distance = osdGetMetersToSelectedUnit(GPS_distanceFlownInCm / 100);
        tfp_sprintf(element->buff, "%d%c", distance, osdGetMetersToSelectedUnitSymbol());
    } else {
        // We use this symbol when we don't have a FIX
        element->buff[0] = SYM_COLON;
        // overwrite any previous distance with blanks
        memset(element->buff + 1, SYM_BLANK, 6);
        element->buff[7] = '\0';
    }
}

static void osdElementGpsHomeDirection(osdElementParms_t *element)
{
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        if (GPS_distanceToHome > 0) {
            const int h = GPS_directionToHome - DECIDEGREES_TO_DEGREES(attitude.values.yaw);
            element->buff[0] = osdGetDirectionSymbolFromHeading(h);
        } else {
            // We don't have a HOME symbol in the font, by now we use this
            element->buff[0] = SYM_THR1;
        }

    } else {
        // We use this symbol when we don't have a FIX
        element->buff[0] = SYM_COLON;
    }

    element->buff[1] = 0;
}

static void osdElementGpsHomeDistance(osdElementParms_t *element)
{
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        const int32_t distance = osdGetMetersToSelectedUnit(GPS_distanceToHome);
        tfp_sprintf(element->buff, "%d%c", distance, osdGetMetersToSelectedUnitSymbol());
    } else {
        // We use this symbol when we don't have a FIX
        element->buff[0] = SYM_COLON;
        // overwrite any previous distance with blanks
        memset(element->buff + 1, SYM_BLANK, 6);
        element->buff[7] = '\0';
    }
}

static void osdElementGpsLatitude(osdElementParms_t *element)
{
    // The SYM_LAT symbol in the actual font contains only blank, so we use the SYM_ARROW_NORTH
    osdFormatCoordinate(element->buff, SYM_ARROW_NORTH, gpsSol.llh.lat);
}

static void osdElementGpsLongitude(osdElementParms_t *element)
{
    // The SYM_LON symbol in the actual font contains only blank, so we use the SYM_ARROW_EAST
    osdFormatCoordinate(element->buff, SYM_ARROW_EAST, gpsSol.llh.lon);
}

static void osdElementGpsSats(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%c%c%2d", SYM_SAT_L, SYM_SAT_R, gpsSol.numSat);
}

static void osdElementGpsSpeed(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%3d%c", osdGetSpeedToSelectedUnit(gpsSol.groundSpeed), osdGetSpeedToSelectedUnitSymbol());
}
#endif // USE_GPS

static void osdElementHorizonSidebars(osdElementParms_t *element)
{
    // Draw AH sides
    const int8_t hudwidth = AH_SIDEBAR_WIDTH_POS;
    const int8_t hudheight = AH_SIDEBAR_HEIGHT_POS;
    for (int y = -hudheight; y <= hudheight; y++) {
        displayWriteChar(element->osdDisplayPort, element->elemPosX - hudwidth, element->elemPosY + y, SYM_AH_DECORATION);
        displayWriteChar(element->osdDisplayPort, element->elemPosX + hudwidth, element->elemPosY + y, SYM_AH_DECORATION);
    }

    // AH level indicators
    displayWriteChar(element->osdDisplayPort, element->elemPosX - hudwidth + 1, element->elemPosY, SYM_AH_LEFT);
    displayWriteChar(element->osdDisplayPort, element->elemPosX + hudwidth - 1, element->elemPosY, SYM_AH_RIGHT);

    element->drawElement = false;  // element already drawn
}

#ifdef USE_RX_LINK_QUALITY_INFO
static void osdElementLinkQuality(osdElementParms_t *element)
{
    // change range to 0-9 (two sig. fig. adds little extra value, also reduces screen estate)
    uint8_t osdLinkQuality = rxGetLinkQuality() * 10 / LINK_QUALITY_MAX_VALUE;
    if (osdLinkQuality >= 10) {
        osdLinkQuality = 9;
    }

    tfp_sprintf(element->buff, "%1d", osdLinkQuality);
}
#endif // USE_RX_LINK_QUALITY_INFO

#ifdef USE_BLACKBOX
static void osdElementLogStatus(osdElementParms_t *element)
{
    if (IS_RC_MODE_ACTIVE(BOXBLACKBOX)) {
        if (!isBlackboxDeviceWorking()) {
            tfp_sprintf(element->buff, "L-");
        } else if (isBlackboxDeviceFull()) {
            tfp_sprintf(element->buff, "L>");
        } else {
            tfp_sprintf(element->buff, "L%d", blackboxGetLogNumber());
        }
    }
}
#endif // USE_BLACKBOX

static void osdElementMahDrawn(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%4d%c", getMAhDrawn(), SYM_MAH);
}

static void osdElementMainBatteryUsage(osdElementParms_t *element)
{
    // Set length of indicator bar
    #define MAIN_BATT_USAGE_STEPS 11 // Use an odd number so the bar can be centered.

    // Calculate constrained value
    const float value = constrain(batteryConfig()->batteryCapacity - getMAhDrawn(), 0, batteryConfig()->batteryCapacity);

    // Calculate mAh used progress
    const uint8_t mAhUsedProgress = ceilf((value / (batteryConfig()->batteryCapacity / MAIN_BATT_USAGE_STEPS)));

    // Create empty battery indicator bar
    element->buff[0] = SYM_PB_START;
    for (int i = 1; i <= MAIN_BATT_USAGE_STEPS; i++) {
        element->buff[i] = i <= mAhUsedProgress ? SYM_PB_FULL : SYM_PB_EMPTY;
    }
    element->buff[MAIN_BATT_USAGE_STEPS + 1] = SYM_PB_CLOSE;
    if (mAhUsedProgress > 0 && mAhUsedProgress < MAIN_BATT_USAGE_STEPS) {
        element->buff[1 + mAhUsedProgress] = SYM_PB_END;
    }
    element->buff[MAIN_BATT_USAGE_STEPS+2] = '\0';
}

static void osdElementMainBatteryVoltage(osdElementParms_t *element)
{
    element->buff[0] = osdGetBatterySymbol(getBatteryAverageCellVoltage());
    tfp_sprintf(element->buff + 1, "%2d.%02d%c", getBatteryVoltage() / 100, getBatteryVoltage() % 100, SYM_VOLT);
}

static void osdElementMotorDiagnostics(osdElementParms_t *element)
{
    int i = 0;
    const bool motorsRunning = areMotorsRunning();
    for (; i < getMotorCount(); i++) {
        if (motorsRunning) {
            element->buff[i] =  0x88 - scaleRange(motor[i], motorOutputLow, motorOutputHigh, 0, 8);
        } else {
            element->buff[i] =  0x88;
        }
    }
    element->buff[i] = '\0';
}

static void osdElementNumericalHeading(osdElementParms_t *element)
{
    const int heading = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
    tfp_sprintf(element->buff, "%c%03d", osdGetDirectionSymbolFromHeading(heading), heading);
}

#ifdef USE_VARIO
static void osdElementNumericalVario(osdElementParms_t *element)
{
    bool haveBaro = false;
    bool haveGps = false;
#ifdef USE_BARO
    haveBaro = sensors(SENSOR_BARO);
#endif // USE_BARO
#ifdef USE_GPS
    haveGps = sensors(SENSOR_GPS) && STATE(GPS_FIX);
#endif // USE_GPS
    if (haveBaro || haveGps) {
        const int verticalSpeed = osdGetMetersToSelectedUnit(getEstimatedVario());
        const char directionSymbol = verticalSpeed < 0 ? SYM_ARROW_SOUTH : SYM_ARROW_NORTH;
        tfp_sprintf(element->buff, "%c%01d.%01d", directionSymbol, abs(verticalSpeed / 100), abs((verticalSpeed % 100) / 10));
    } else {
        // We use this symbol when we don't have a valid measure
        element->buff[0] = SYM_COLON;
        // overwrite any previous vertical speed with blanks
        memset(element->buff + 1, SYM_BLANK, 6);
        element->buff[7] = '\0';
    }
}
#endif // USE_VARIO

static void osdElementPidRateProfile(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%d-%d", getCurrentPidProfileIndex() + 1, getCurrentControlRateProfileIndex() + 1);
}

static void osdElementPidsPitch(osdElementParms_t *element)
{
    osdFormatPID(element->buff, "PIT", &currentPidProfile->pid[PID_PITCH]);
}

static void osdElementPidsRoll(osdElementParms_t *element)
{
    osdFormatPID(element->buff, "ROL", &currentPidProfile->pid[PID_ROLL]);
}

static void osdElementPidsYaw(osdElementParms_t *element)
{
    osdFormatPID(element->buff, "YAW", &currentPidProfile->pid[PID_YAW]);
}

static void osdElementPower(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%4dW", getAmperage() * getBatteryVoltage() / 10000);
}

static void osdElementRemainingTimeEstimate(osdElementParms_t *element)
{
    const int mAhDrawn = getMAhDrawn();

    if (mAhDrawn <= 0.1 * osdConfig()->cap_alarm) {  // also handles the mAhDrawn == 0 condition
        tfp_sprintf(element->buff, "--:--");
    } else if (mAhDrawn > osdConfig()->cap_alarm) {
        tfp_sprintf(element->buff, "00:00");
    } else {
        const int remaining_time = (int)((osdConfig()->cap_alarm - mAhDrawn) * ((float)osdFlyTime) / mAhDrawn);
        osdFormatTime(element->buff, OSD_TIMER_PREC_SECOND, remaining_time);
    }
}

static void osdElementRssi(osdElementParms_t *element)
{
    uint16_t osdRssi = getRssi() * 100 / 1024; // change range
    if (osdRssi >= 100) {
        osdRssi = 99;
    }

    tfp_sprintf(element->buff, "%c%2d", SYM_RSSI, osdRssi);
}

#ifdef USE_RTC_TIME
static void osdElementRtcTime(osdElementParms_t *element)
{
    osdFormatRtcDateTime(&element->buff[0]);
}
#endif // USE_RTC_TIME

#ifdef USE_OSD_STICK_OVERLAY
static void osdElementStickOverlay(osdElementParms_t *element)
{
    const uint8_t xpos = element->elemPosX;
    const uint8_t ypos = element->elemPosY;

    // Draw the axis first
    for (unsigned x = 0; x < OSD_STICK_OVERLAY_WIDTH; x++) {
        for (unsigned  y = 0; y < OSD_STICK_OVERLAY_HEIGHT; y++) {
            // draw the axes, vertical and horizonal
            if ((x == ((OSD_STICK_OVERLAY_WIDTH - 1) / 2)) && (y == (OSD_STICK_OVERLAY_HEIGHT - 1) / 2)) {
                displayWriteChar(element->osdDisplayPort, xpos + x, ypos + y, SYM_STICK_OVERLAY_CENTER);
            } else if (x == ((OSD_STICK_OVERLAY_WIDTH - 1) / 2)) {
                displayWriteChar(element->osdDisplayPort, xpos + x, ypos + y, SYM_STICK_OVERLAY_VERTICAL);
            } else if (y == ((OSD_STICK_OVERLAY_HEIGHT - 1) / 2)) {
                displayWriteChar(element->osdDisplayPort, xpos + x, ypos + y, SYM_STICK_OVERLAY_HORIZONTAL);
            }
        }
    }

    // Now draw the cursor
    rc_alias_e vertical_channel, horizontal_channel;

    if (element->item == OSD_STICK_OVERLAY_LEFT) {
        vertical_channel = radioModes[osdConfig()->overlay_radio_mode-1].left_vertical;
        horizontal_channel = radioModes[osdConfig()->overlay_radio_mode-1].left_horizontal;
    } else {
        vertical_channel = radioModes[osdConfig()->overlay_radio_mode-1].right_vertical;
        horizontal_channel = radioModes[osdConfig()->overlay_radio_mode-1].right_horizontal;
    }

    const uint8_t cursorX = scaleRange(constrain(rcData[horizontal_channel], PWM_RANGE_MIN, PWM_RANGE_MAX - 1), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, OSD_STICK_OVERLAY_WIDTH);
    const uint8_t cursorY = OSD_STICK_OVERLAY_VERTICAL_POSITIONS - 1 - scaleRange(constrain(rcData[vertical_channel], PWM_RANGE_MIN, PWM_RANGE_MAX - 1), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, OSD_STICK_OVERLAY_VERTICAL_POSITIONS);
    const char cursor = SYM_STICK_OVERLAY_SPRITE_HIGH + (cursorY % OSD_STICK_OVERLAY_SPRITE_HEIGHT);

    displayWriteChar(element->osdDisplayPort, xpos + cursorX, ypos + cursorY / OSD_STICK_OVERLAY_SPRITE_HEIGHT, cursor);

    element->drawElement = false;  // element already drawn
}
#endif // USE_OSD_STICK_OVERLAY

static void osdElementThrottlePosition(osdElementParms_t *element)
{
    element->buff[0] = SYM_THR;
    element->buff[1] = SYM_THR1;
    tfp_sprintf(element->buff + 2, "%3d", calculateThrottlePercent());
}

static void osdElementTimer(osdElementParms_t *element)
{
    osdFormatTimer(element->buff, true, true, element->item - OSD_ITEM_TIMER_1);
}

#ifdef USE_VTX_COMMON
static void osdElementVtxChannel(osdElementParms_t *element)
{
    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    const char vtxBandLetter = vtxCommonLookupBandLetter(vtxDevice, vtxSettingsConfig()->band);
    const char *vtxChannelName = vtxCommonLookupChannelName(vtxDevice, vtxSettingsConfig()->channel);
    uint8_t vtxPower = vtxSettingsConfig()->power;
    if (vtxDevice && vtxSettingsConfig()->lowPowerDisarm) {
        vtxCommonGetPowerIndex(vtxDevice, &vtxPower);
    }
    tfp_sprintf(element->buff, "%c:%s:%1d", vtxBandLetter, vtxChannelName, vtxPower);
}
#endif // USE_VTX_COMMON

static void osdElementWarnings(osdElementParms_t *element)
{
#define OSD_WARNINGS_MAX_SIZE 11
#define OSD_FORMAT_MESSAGE_BUFFER_SIZE (OSD_WARNINGS_MAX_SIZE + 1)

    STATIC_ASSERT(OSD_FORMAT_MESSAGE_BUFFER_SIZE <= OSD_ELEMENT_BUFFER_LENGTH, osd_warnings_size_exceeds_buffer_size);

    const batteryState_e batteryState = getBatteryState();
    const timeUs_t currentTimeUs = micros();

    static timeUs_t armingDisabledUpdateTimeUs;
    static unsigned armingDisabledDisplayIndex;

    CLR_BLINK(OSD_WARNINGS);

    // Cycle through the arming disabled reasons
    if (osdWarnGetState(OSD_WARNING_ARMING_DISABLE)) {
        if (IS_RC_MODE_ACTIVE(BOXARM) && isArmingDisabled()) {
            const armingDisableFlags_e armSwitchOnlyFlag = 1 << (ARMING_DISABLE_FLAGS_COUNT - 1);
            armingDisableFlags_e flags = getArmingDisableFlags();

            // Remove the ARMSWITCH flag unless it's the only one
            if ((flags & armSwitchOnlyFlag) && (flags != armSwitchOnlyFlag)) {
                flags -= armSwitchOnlyFlag;
            }

            // Rotate to the next arming disabled reason after a 0.5 second time delay
            // or if the current flag is no longer set
            if ((currentTimeUs - armingDisabledUpdateTimeUs > 5e5) || !(flags & (1 << armingDisabledDisplayIndex))) {
                if (armingDisabledUpdateTimeUs == 0) {
                    armingDisabledDisplayIndex = ARMING_DISABLE_FLAGS_COUNT - 1;
                }
                armingDisabledUpdateTimeUs = currentTimeUs;

                do {
                    if (++armingDisabledDisplayIndex >= ARMING_DISABLE_FLAGS_COUNT) {
                        armingDisabledDisplayIndex = 0;
                    }
                } while (!(flags & (1 << armingDisabledDisplayIndex)));
            }

            osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, armingDisableFlagNames[armingDisabledDisplayIndex]);
            return;
        } else {
            armingDisabledUpdateTimeUs = 0;
        }
    }

#ifdef USE_DSHOT
    if (isTryingToArm() && !ARMING_FLAG(ARMED)) {
        int armingDelayTime = (getLastDshotBeaconCommandTimeUs() + DSHOT_BEACON_GUARD_DELAY_US - currentTimeUs) / 1e5;
        if (armingDelayTime < 0) {
            armingDelayTime = 0;
        }
        if (armingDelayTime >= (DSHOT_BEACON_GUARD_DELAY_US / 1e5 - 5)) {
            osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, " BEACON ON"); // Display this message for the first 0.5 seconds
        } else {
            char armingDelayMessage[OSD_FORMAT_MESSAGE_BUFFER_SIZE];
            tfp_sprintf(armingDelayMessage, "ARM IN %d.%d", armingDelayTime / 10, armingDelayTime % 10);
            osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, armingDelayMessage);
        }
        return;
    }
#endif // USE_DSHOT
    if (osdWarnGetState(OSD_WARNING_FAIL_SAFE) && failsafeIsActive()) {
        osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "FAIL SAFE");
        SET_BLINK(OSD_WARNINGS);
        return;
    }

    // Warn when in flip over after crash mode
    if (osdWarnGetState(OSD_WARNING_CRASH_FLIP) && isFlipOverAfterCrashActive()) {
        osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "CRASH FLIP");
        return;
    }

#ifdef USE_LAUNCH_CONTROL
    // Warn when in launch control mode
    if (osdWarnGetState(OSD_WARNING_LAUNCH_CONTROL) && isLaunchControlActive()) {
#ifdef USE_ACC
        if (sensors(SENSOR_ACC)) {
            char launchControlMsg[OSD_FORMAT_MESSAGE_BUFFER_SIZE];
            const int pitchAngle = constrain((attitude.raw[FD_PITCH] - accelerometerConfig()->accelerometerTrims.raw[FD_PITCH]) / 10, -90, 90);
            tfp_sprintf(launchControlMsg, "LAUNCH %d", pitchAngle);
            osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, launchControlMsg);
        } else
#endif // USE_ACC
        {
            osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "LAUNCH");
        }
        return;
    }
#endif // USE_LAUNCH_CONTROL

    if (osdWarnGetState(OSD_WARNING_BATTERY_CRITICAL) && batteryState == BATTERY_CRITICAL) {
        osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, " LAND NOW");
        SET_BLINK(OSD_WARNINGS);
        return;
    }

#ifdef USE_GPS_RESCUE
    if (osdWarnGetState(OSD_WARNING_GPS_RESCUE_UNAVAILABLE) &&
       ARMING_FLAG(ARMED) &&
       gpsRescueIsConfigured() &&
       !gpsRescueIsDisabled() &&
       !gpsRescueIsAvailable()) {
        osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "RESCUE N/A");
        SET_BLINK(OSD_WARNINGS);
        return;
    }

    if (osdWarnGetState(OSD_WARNING_GPS_RESCUE_DISABLED) &&
       ARMING_FLAG(ARMED) &&
       gpsRescueIsConfigured() &&
       gpsRescueIsDisabled()) {

        statistic_t *stats = osdGetStats();
        if (cmpTimeUs(stats->armed_time, OSD_GPS_RESCUE_DISABLED_WARNING_DURATION_US) < 0) {
            osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "RESCUE OFF");
            SET_BLINK(OSD_WARNINGS);
            return;
        }
    }

#endif // USE_GPS_RESCUE

    // Show warning if in HEADFREE flight mode
    if (FLIGHT_MODE(HEADFREE_MODE)) {
        osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "HEADFREE");
        SET_BLINK(OSD_WARNINGS);
        return;
    }

#ifdef USE_ADC_INTERNAL
    const int16_t coreTemperature = getCoreTemperatureCelsius();
    if (osdWarnGetState(OSD_WARNING_CORE_TEMPERATURE) && coreTemperature >= osdConfig()->core_temp_alarm) {
        char coreTemperatureWarningMsg[OSD_FORMAT_MESSAGE_BUFFER_SIZE];
        tfp_sprintf(coreTemperatureWarningMsg, "CORE: %3d%c", osdConvertTemperatureToSelectedUnit(coreTemperature), osdGetTemperatureSymbolForSelectedUnit());

        osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, coreTemperatureWarningMsg);
        SET_BLINK(OSD_WARNINGS);
        return;
    }
#endif // USE_ADC_INTERNAL

#ifdef USE_ESC_SENSOR
    // Show warning if we lose motor output, the ESC is overheating or excessive current draw
    if (featureIsEnabled(FEATURE_ESC_SENSOR) && osdWarnGetState(OSD_WARNING_ESC_FAIL)) {
        char escWarningMsg[OSD_FORMAT_MESSAGE_BUFFER_SIZE];
        unsigned pos = 0;

        const char *title = "ESC";

        // center justify message
        while (pos < (OSD_WARNINGS_MAX_SIZE - (strlen(title) + getMotorCount())) / 2) {
            escWarningMsg[pos++] = ' ';
        }

        strcpy(escWarningMsg + pos, title);
        pos += strlen(title);

        unsigned i = 0;
        unsigned escWarningCount = 0;
        while (i < getMotorCount() && pos < OSD_FORMAT_MESSAGE_BUFFER_SIZE - 1) {
            escSensorData_t *escData = getEscSensorData(i);
            const char motorNumber = '1' + i;
            // if everything is OK just display motor number else R, T or C
            char warnFlag = motorNumber;
            if (ARMING_FLAG(ARMED) && osdConfig()->esc_rpm_alarm != ESC_RPM_ALARM_OFF && calcEscRpm(escData->rpm) <= osdConfig()->esc_rpm_alarm) {
                warnFlag = 'R';
            }
            if (osdConfig()->esc_temp_alarm != ESC_TEMP_ALARM_OFF && escData->temperature >= osdConfig()->esc_temp_alarm) {
                warnFlag = 'T';
            }
            if (ARMING_FLAG(ARMED) && osdConfig()->esc_current_alarm != ESC_CURRENT_ALARM_OFF && escData->current >= osdConfig()->esc_current_alarm) {
                warnFlag = 'C';
            }

            escWarningMsg[pos++] = warnFlag;

            if (warnFlag != motorNumber) {
                escWarningCount++;
            }

            i++;
        }

        escWarningMsg[pos] = '\0';

        if (escWarningCount > 0) {
            osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, escWarningMsg);
            SET_BLINK(OSD_WARNINGS);
            return;
        }
    }
#endif // USE_ESC_SENSOR

    if (osdWarnGetState(OSD_WARNING_BATTERY_WARNING) && batteryState == BATTERY_WARNING) {
        osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "LOW BATTERY");
        SET_BLINK(OSD_WARNINGS);
        return;
    }

#ifdef USE_RC_SMOOTHING_FILTER
    // Show warning if rc smoothing hasn't initialized the filters
    if (osdWarnGetState(OSD_WARNING_RC_SMOOTHING) && ARMING_FLAG(ARMED) && !rcSmoothingInitializationComplete()) {
        osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "RCSMOOTHING");
        SET_BLINK(OSD_WARNINGS);
        return;
    }
#endif // USE_RC_SMOOTHING_FILTER

    // Show warning if battery is not fresh
    if (osdWarnGetState(OSD_WARNING_BATTERY_NOT_FULL) && !ARMING_FLAG(WAS_EVER_ARMED) && (getBatteryState() == BATTERY_OK)
          && getBatteryAverageCellVoltage() < batteryConfig()->vbatfullcellvoltage) {
        osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "BATT < FULL");
        return;
    }

    // Visual beeper
    if (osdWarnGetState(OSD_WARNING_VISUAL_BEEPER) && osdGetVisualBeeperState()) {
        osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "  * * * *");
        return;
    }

    osdFormatMessage(element->buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, NULL);
}

// Define the order in which the elements are drawn.
// Elements positioned later in the list will overlay the earlier
// ones if their character positions overlap
// Elements that need special runtime conditional processing should be added
// to osdAnalyzeActiveElements()

static const uint8_t osdElementDisplayOrder[] = {
    OSD_MAIN_BATT_VOLTAGE,
    OSD_RSSI_VALUE,
    OSD_CROSSHAIRS,
    OSD_HORIZON_SIDEBARS,
    OSD_ITEM_TIMER_1,
    OSD_ITEM_TIMER_2,
    OSD_REMAINING_TIME_ESTIMATE,
    OSD_FLYMODE,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
    OSD_CRAFT_NAME,
    OSD_ALTITUDE,
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_PIDRATE_PROFILE,
    OSD_WARNINGS,
    OSD_AVG_CELL_VOLTAGE,
    OSD_DEBUG,
    OSD_PITCH_ANGLE,
    OSD_ROLL_ANGLE,
    OSD_MAIN_BATT_USAGE,
    OSD_DISARMED,
    OSD_NUMERICAL_HEADING,
    OSD_NUMERICAL_VARIO,
    OSD_COMPASS_BAR,
    OSD_ANTI_GRAVITY,
#ifdef USE_BLACKBOX
    OSD_LOG_STATUS,
#endif
    OSD_MOTOR_DIAG,
#ifdef USE_ACC
    OSD_FLIP_ARROW,
#endif
    OSD_DISPLAY_NAME,
#ifdef USE_RTC_TIME
    OSD_RTC_DATETIME,
#endif
#ifdef USE_OSD_ADJUSTMENTS
    OSD_ADJUSTMENT_RANGE,
#endif
#ifdef USE_ADC_INTERNAL
    OSD_CORE_TEMPERATURE,
#endif
#ifdef USE_RX_LINK_QUALITY_INFO
    OSD_LINK_QUALITY,
#endif
#ifdef USE_OSD_STICK_OVERLAY
    OSD_STICK_OVERLAY_LEFT,
    OSD_STICK_OVERLAY_RIGHT,
#endif
};

// Define the mapping between the OSD element id and the function to draw it

const osdElementDrawFn osdElementDrawFunction[OSD_ITEM_COUNT] = {
    [OSD_RSSI_VALUE]              = osdElementRssi,
    [OSD_MAIN_BATT_VOLTAGE]       = osdElementMainBatteryVoltage,
    [OSD_CROSSHAIRS]              = osdElementCrosshairs,
#ifdef USE_ACC
    [OSD_ARTIFICIAL_HORIZON]      = osdElementArtificialHorizon,
#endif
    [OSD_HORIZON_SIDEBARS]        = osdElementHorizonSidebars,
    [OSD_ITEM_TIMER_1]            = osdElementTimer,
    [OSD_ITEM_TIMER_2]            = osdElementTimer,
    [OSD_FLYMODE]                 = osdElementFlymode,
    [OSD_CRAFT_NAME]              = osdElementCraftName,
    [OSD_THROTTLE_POS]            = osdElementThrottlePosition,
#ifdef USE_VTX_COMMON
    [OSD_VTX_CHANNEL]             = osdElementVtxChannel,
#endif
    [OSD_CURRENT_DRAW]            = osdElementCurrentDraw,
    [OSD_MAH_DRAWN]               = osdElementMahDrawn,
#ifdef USE_GPS
    [OSD_GPS_SPEED]               = osdElementGpsSpeed,
    [OSD_GPS_SATS]                = osdElementGpsSats,
#endif
    [OSD_ALTITUDE]                = osdElementAltitude,
    [OSD_ROLL_PIDS]               = osdElementPidsRoll,
    [OSD_PITCH_PIDS]              = osdElementPidsPitch,
    [OSD_YAW_PIDS]                = osdElementPidsYaw,
    [OSD_POWER]                   = osdElementPower,
    [OSD_PIDRATE_PROFILE]         = osdElementPidRateProfile,
    [OSD_WARNINGS]                = osdElementWarnings,
    [OSD_AVG_CELL_VOLTAGE]        = osdElementAverageCellVoltage,
#ifdef USE_GPS
    [OSD_GPS_LON]                 = osdElementGpsLongitude,
    [OSD_GPS_LAT]                 = osdElementGpsLatitude,
#endif
    [OSD_DEBUG]                   = osdElementDebug,
#ifdef USE_ACC
    [OSD_PITCH_ANGLE]             = osdElementAngleRollPitch,
    [OSD_ROLL_ANGLE]              = osdElementAngleRollPitch,
#endif
    [OSD_MAIN_BATT_USAGE]         = osdElementMainBatteryUsage,
    [OSD_DISARMED]                = osdElementDisarmed,
#ifdef USE_GPS
    [OSD_HOME_DIR]                = osdElementGpsHomeDirection,
    [OSD_HOME_DIST]               = osdElementGpsHomeDistance,
#endif
    [OSD_NUMERICAL_HEADING]       = osdElementNumericalHeading,
#ifdef USE_VARIO
    [OSD_NUMERICAL_VARIO]         = osdElementNumericalVario,
#endif
    [OSD_COMPASS_BAR]             = osdElementCompassBar,
#ifdef USE_ESC_SENSOR
    [OSD_ESC_TMP]                 = osdElementEscTemperature,
#endif
#if defined(USE_RPM_FILTER) || defined(USE_ESC_SENSOR)
    [OSD_ESC_RPM]                 = osdElementEscRpm,
#endif
    [OSD_REMAINING_TIME_ESTIMATE] = osdElementRemainingTimeEstimate,
#ifdef USE_RTC_TIME
    [OSD_RTC_DATETIME]            = osdElementRtcTime,
#endif
#ifdef USE_OSD_ADJUSTMENTS
    [OSD_ADJUSTMENT_RANGE]        = osdElementAdjustmentRange,
#endif
#ifdef USE_ADC_INTERNAL
    [OSD_CORE_TEMPERATURE]        = osdElementCoreTemperature,
#endif
    [OSD_ANTI_GRAVITY]            = osdElementAntiGravity,
#ifdef USE_ACC
    [OSD_G_FORCE]                 = osdElementGForce,
#endif
    [OSD_MOTOR_DIAG]              = osdElementMotorDiagnostics,
#ifdef USE_BLACKBOX
    [OSD_LOG_STATUS]              = osdElementLogStatus,
#endif
#ifdef USE_ACC
    [OSD_FLIP_ARROW]              = osdElementCrashFlipArrow,
#endif
#ifdef USE_RX_LINK_QUALITY_INFO
    [OSD_LINK_QUALITY]            = osdElementLinkQuality,
#endif
#ifdef USE_GPS
    [OSD_FLIGHT_DIST]             = osdElementGpsFlightDistance,
#endif
#ifdef USE_OSD_STICK_OVERLAY
    [OSD_STICK_OVERLAY_LEFT]      = osdElementStickOverlay,
    [OSD_STICK_OVERLAY_RIGHT]     = osdElementStickOverlay,
#endif
    [OSD_DISPLAY_NAME]            = osdElementDisplayName,
#if defined(USE_RPM_FILTER) || defined(USE_ESC_SENSOR)
    [OSD_ESC_RPM_FREQ]            = osdElementEscRpmFreq,
#endif
};

static void osdAddActiveElement(osd_items_e element)
{
    if (VISIBLE(osdConfig()->item_pos[element])) {
        activeOsdElementArray[activeOsdElementCount++] = element;
    }
}

// Examine the elements and build a list of only the active (enabled)
// ones to speed up rendering.

void osdAnalyzeActiveElements(void)
{
    activeOsdElementCount = 0;

#ifdef USE_ACC
    if (sensors(SENSOR_ACC)) {
        osdAddActiveElement(OSD_ARTIFICIAL_HORIZON);
        osdAddActiveElement(OSD_G_FORCE);
    }
#endif

    for (unsigned i = 0; i < sizeof(osdElementDisplayOrder); i++) {
        osdAddActiveElement(osdElementDisplayOrder[i]);
    }

#ifdef USE_GPS
    if (sensors(SENSOR_GPS)) {
        osdAddActiveElement(OSD_GPS_SATS);
        osdAddActiveElement(OSD_GPS_SPEED);
        osdAddActiveElement(OSD_GPS_LAT);
        osdAddActiveElement(OSD_GPS_LON);
        osdAddActiveElement(OSD_HOME_DIST);
        osdAddActiveElement(OSD_HOME_DIR);
        osdAddActiveElement(OSD_FLIGHT_DIST);
    }
#endif // GPS
#ifdef USE_ESC_SENSOR  	
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        osdAddActiveElement(OSD_ESC_TMP);
    }
#endif

#if defined(USE_RPM_FILTER) || defined(USE_ESC_SENSOR)
    if ((featureIsEnabled(FEATURE_ESC_SENSOR)) || (motorConfig()->dev.useDshotTelemetry)) {
        osdAddActiveElement(OSD_ESC_RPM);
        osdAddActiveElement(OSD_ESC_RPM_FREQ);
    }
#endif
}

static bool osdDrawSingleElement(displayPort_t *osdDisplayPort, uint8_t item)
{
    if (BLINK(item)) {
        return false;
    }

    uint8_t elemPosX = OSD_X(osdConfig()->item_pos[item]);
    uint8_t elemPosY = OSD_Y(osdConfig()->item_pos[item]);
    char buff[OSD_ELEMENT_BUFFER_LENGTH] = "";

    osdElementParms_t element;
    element.item = item;
    element.elemPosX = elemPosX;
    element.elemPosY = elemPosY;
    element.buff = (char *)&buff;
    element.osdDisplayPort = osdDisplayPort;
    element.drawElement = true;

    // Call the element drawing function
    osdElementDrawFunction[item](&element);
    if (element.drawElement) {
        displayWrite(osdDisplayPort, elemPosX, elemPosY, buff);
    }

    return true;
}

void osdDrawActiveElements(displayPort_t *osdDisplayPort, timeUs_t currentTimeUs)
{
#ifdef USE_GPS
    static bool lastGpsSensorState;
    // Handle the case that the GPS_SENSOR may be delayed in activation
    // or deactivate if communication is lost with the module.
    const bool currentGpsSensorState = sensors(SENSOR_GPS);
    if (lastGpsSensorState != currentGpsSensorState) {
        lastGpsSensorState = currentGpsSensorState;
        osdAnalyzeActiveElements();
    }
#endif // USE_GPS

    blinkState = (currentTimeUs / 200000) % 2;

    for (unsigned i = 0; i < activeOsdElementCount; i++) {
        osdDrawSingleElement(osdDisplayPort, activeOsdElementArray[i]);
    }
}

void osdResetAlarms(void)
{
    memset(blinkBits, 0, sizeof(blinkBits));
}

void osdUpdateAlarms(void)
{
    // This is overdone?

    int32_t alt = osdGetMetersToSelectedUnit(getEstimatedAltitudeCm()) / 100;

    if (getRssiPercent() < osdConfig()->rssi_alarm) {
        SET_BLINK(OSD_RSSI_VALUE);
    } else {
        CLR_BLINK(OSD_RSSI_VALUE);
    }

    if (getBatteryState() == BATTERY_OK) {
        CLR_BLINK(OSD_MAIN_BATT_VOLTAGE);
        CLR_BLINK(OSD_AVG_CELL_VOLTAGE);
    } else {
        SET_BLINK(OSD_MAIN_BATT_VOLTAGE);
        SET_BLINK(OSD_AVG_CELL_VOLTAGE);
    }

#ifdef USE_GPS
    if ((STATE(GPS_FIX) == 0) || (gpsSol.numSat < 5)
#ifdef USE_GPS_RESCUE
            || ((gpsSol.numSat < gpsRescueConfig()->minSats) && gpsRescueIsConfigured())
#endif
            ) {
        SET_BLINK(OSD_GPS_SATS);
    } else {
        CLR_BLINK(OSD_GPS_SATS);
    }
#endif //USE_GPS

    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
        const uint16_t timer = osdConfig()->timers[i];
        const timeUs_t time = osdGetTimerValue(OSD_TIMER_SRC(timer));
        const timeUs_t alarmTime = OSD_TIMER_ALARM(timer) * 60000000; // convert from minutes to us
        if (alarmTime != 0 && time >= alarmTime) {
            SET_BLINK(OSD_ITEM_TIMER_1 + i);
        } else {
            CLR_BLINK(OSD_ITEM_TIMER_1 + i);
        }
    }

    if (getMAhDrawn() >= osdConfig()->cap_alarm) {
        SET_BLINK(OSD_MAH_DRAWN);
        SET_BLINK(OSD_MAIN_BATT_USAGE);
        SET_BLINK(OSD_REMAINING_TIME_ESTIMATE);
    } else {
        CLR_BLINK(OSD_MAH_DRAWN);
        CLR_BLINK(OSD_MAIN_BATT_USAGE);
        CLR_BLINK(OSD_REMAINING_TIME_ESTIMATE);
    }

    if (alt >= osdConfig()->alt_alarm) {
        SET_BLINK(OSD_ALTITUDE);
    } else {
        CLR_BLINK(OSD_ALTITUDE);
    }

#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        // This works because the combined ESC data contains the maximum temperature seen amongst all ESCs
        if (osdConfig()->esc_temp_alarm != ESC_TEMP_ALARM_OFF && osdEscDataCombined->temperature >= osdConfig()->esc_temp_alarm) {
            SET_BLINK(OSD_ESC_TMP);
        } else {
            CLR_BLINK(OSD_ESC_TMP);
        }
    }
#endif
}

#endif // USE_OSD
