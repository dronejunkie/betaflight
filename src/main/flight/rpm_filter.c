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


#include <math.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RPM_FILTER)

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "drivers/pwm_output_counts.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/pg_ids.h"

#include "scheduler/scheduler.h"

#include "sensors/gyro.h"

#include "rpm_filter.h"

#define RPM_FILTER_MAXHARMONICS 3
#define SECONDS_PER_MINUTE      60.0f
#define ERPM_PER_LSB            100.0f
#define MIN_UPDATE_T            0.001f

typedef union gyroLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} gyroLowpass_t;

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} dtermLowpass_t;

static FAST_RAM_ZERO_INIT pt1Filter_t rpmFilters[MAX_SUPPORTED_MOTORS];
static FAST_RAM_ZERO_INIT gyroLowpass_t gyroLPFFilter[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT dtermLowpass_t dTermLPFFilter[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT filterApplyFnPtr dTermLPFApplyFn;
static FAST_RAM_ZERO_INIT filterApplyFnPtr gyroLPFApplyFn;

typedef struct rpmNotchFilter_s
{
    uint8_t harmonics;
    float   minHz;
    float   maxHz;
    float   q;
    float   loopTime;
    biquadFilter_t notch[XYZ_AXIS_COUNT][MAX_SUPPORTED_MOTORS][RPM_FILTER_MAXHARMONICS];
} rpmNotchFilter_t;

FAST_RAM_ZERO_INIT static float   erpmToHz;
FAST_RAM_ZERO_INIT static float   filteredMotorErpm[MAX_SUPPORTED_MOTORS];
FAST_RAM_ZERO_INIT static uint8_t numberFilters;
FAST_RAM_ZERO_INIT static uint8_t numberRpmNotchFilters;
FAST_RAM_ZERO_INIT static uint8_t filterUpdatesPerIteration;
FAST_RAM_ZERO_INIT static float   pidLooptime;
FAST_RAM_ZERO_INIT static rpmNotchFilter_t filters[2];
FAST_RAM_ZERO_INIT static rpmNotchFilter_t* gyroFilter;
FAST_RAM_ZERO_INIT static rpmNotchFilter_t* dtermFilter;
FAST_RAM_ZERO_INIT static uint8_t gyroLPFType;
FAST_RAM_ZERO_INIT static uint8_t dTermLPFType;
FAST_RAM_ZERO_INIT static float dT;
FAST_RAM_ZERO_INIT static float dTermLPFMin;
FAST_RAM_ZERO_INIT static float dTermLPFMax;
FAST_RAM_ZERO_INIT static float gyroLPFMin;
FAST_RAM_ZERO_INIT static float gyroLPFMax;

PG_REGISTER_WITH_RESET_FN(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 3);

void pgResetFn_rpmFilterConfig(rpmFilterConfig_t *config)
{
    config->gyro_rpm_notch_harmonics = 3;
    config->gyro_rpm_notch_min = 100;
    config->gyro_rpm_notch_q = 500;
    config->dterm_rpm_notch_harmonics = 0;
    config->dterm_rpm_notch_min = 100;
    config->dterm_rpm_notch_q = 500;
    config->rpm_lpf = 150;
    config->rpm_gyro_lpf_min = 0;
    config->rpm_gyro_lpf_max = 0;
    config->rpm_dterm_lpf_min = 0;
    config->rpm_dterm_lpf_max = 0;
    config->rpm_gyro_lpf_type = FILTER_PT1;
    config->rpm_dterm_lpf_type = FILTER_BIQUAD;
}

static void rpmNotchFilterInit(rpmNotchFilter_t* filter, int harmonics, int minHz, int q, float looptime)
{
    filter->harmonics = harmonics;
    filter->minHz = minHz;
    filter->q = q / 100.0f;
    filter->loopTime = looptime;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int motor = 0; motor < getMotorCount(); motor++) {
            for (int i = 0; i < harmonics; i++) {
                biquadFilterInit(
                    &filter->notch[axis][motor][i], minHz * i, looptime, filter->q, FILTER_NOTCH);
            }
        }
    }
}

void rpmFilterInit(const rpmFilterConfig_t *config)
{
    numberRpmNotchFilters = 0;
    if (!motorConfig()->dev.useDshotTelemetry) {
        gyroFilter = dtermFilter = NULL;
        return;
    }

    pidLooptime = gyro.targetLooptime * pidConfig()->pid_process_denom;
    if (config->gyro_rpm_notch_harmonics) {
        gyroFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(gyroFilter, config->gyro_rpm_notch_harmonics,
                           config->gyro_rpm_notch_min, config->gyro_rpm_notch_q, gyro.targetLooptime);
        // don't go quite to nyquist to avoid oscillations
        gyroFilter->maxHz = 0.48f / (gyro.targetLooptime * 1e-6f);
    }

    if (config->dterm_rpm_notch_harmonics) {
        dtermFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(dtermFilter, config->dterm_rpm_notch_harmonics,
                           config->dterm_rpm_notch_min, config->dterm_rpm_notch_q, pidLooptime);
        // don't go quite to nyquist to avoid oscillations
        dtermFilter->maxHz = 0.48f / (pidLooptime * 1e-6f);
    }

    for (int i = 0; i < getMotorCount(); i++) {
        pt1FilterInit(&rpmFilters[i], pt1FilterGain(config->rpm_lpf, pidLooptime * 1e-6f));
    }

    erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE  / (motorConfig()->motorPoleCount / 2.0f);
    const float loopIterationsPerUpdate = MIN_UPDATE_T / (pidLooptime * 1e-6f);
    numberFilters = getMotorCount() * (filters[0].harmonics + filters[1].harmonics);
    const float filtersPerLoopIteration = numberFilters / loopIterationsPerUpdate;
    filterUpdatesPerIteration = rintf(filtersPerLoopIteration + 0.49f);
    gyroLPFType = config->rpm_gyro_lpf_type;
    dTermLPFType = config->rpm_dterm_lpf_type;
    dT = gyro.targetLooptime * 1e-6f;
    gyroLPFMin = config->rpm_gyro_lpf_min;
    gyroLPFMax = MIN(config->rpm_gyro_lpf_max, gyroFilter->maxHz);
    if (gyroLPFMin > 0 ) {
        switch (gyroLPFType){
        case FILTER_PT1:
            gyroLPFApplyFn = (filterApplyFnPtr) pt1FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterInit(&gyroLPFFilter[axis].pt1Filter, pt1FilterGain(gyroLPFMin, dT));
            }
            break;
        case FILTER_BIQUAD:
            gyroLPFApplyFn = (filterApplyFnPtr) biquadFilterApplyDF1;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterInitLPF(&gyroLPFFilter[axis].biquadFilter, gyroLPFMin, gyro.targetLooptime);
            }
            break;

        default:
            gyroLPFApplyFn = (filterApplyFnPtr) pt1FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterInit(&gyroLPFFilter[axis].pt1Filter, pt1FilterGain(gyroLPFMin, dT));
            }
            break;
        }

    }

    dTermLPFMin = config->rpm_dterm_lpf_min;
    dTermLPFMax = MIN (config->rpm_dterm_lpf_max, dtermFilter->maxHz);
    if (dTermLPFMin >= dTermLPFMax ) {
        dTermLPFMin = 0;
    }

    if (dTermLPFMin > 0 ) {
        switch (dTermLPFType){
        case FILTER_PT1:
            dTermLPFApplyFn = (filterApplyFnPtr) pt1FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterInit(&dTermLPFFilter[axis].pt1Filter, pt1FilterGain(dTermLPFMin, pidLooptime * 1e-6f));
            }
            break;
        case FILTER_BIQUAD:
            dTermLPFApplyFn = (filterApplyFnPtr) biquadFilterApplyDF1;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterInitLPF(&dTermLPFFilter[axis].biquadFilter, dTermLPFMin, pidLooptime);
            }
            break;
        default:
            dTermLPFApplyFn = (filterApplyFnPtr) biquadFilterApplyDF1;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterInitLPF(&dTermLPFFilter[axis].biquadFilter, dTermLPFMin, pidLooptime);
            }
            break;
        }
    }
}

static float applyFilter(rpmNotchFilter_t* filter, int axis, float value)
{
    if (filter == NULL) {
        return value;
    }
    for (int motor = 0; motor < getMotorCount(); motor++) {
        for (int i = 0; i < filter->harmonics; i++) {
            value = biquadFilterApplyDF1(&filter->notch[axis][motor][i], value);
        }
    }
    return value;
}

float rpmFilterGyro(int axis, float value)
{
    if (gyroLPFMin >  0) {
        value = gyroLPFApplyFn((filter_t *) &gyroLPFFilter[axis],value);
    }

    return applyFilter(gyroFilter, axis, value);
}

float rpmFilterDterm(int axis, float value)
{
    if (dTermLPFMin > 0) {
        value = dTermLPFApplyFn((filter_t *) &dTermLPFFilter[axis],value);
    }

    return applyFilter(dtermFilter, axis, value);
}

FAST_RAM_ZERO_INIT static float motorFrequency[MAX_SUPPORTED_MOTORS];

FAST_CODE_NOINLINE void rpmFilterUpdate()
{
    if (gyroFilter == NULL && dtermFilter == NULL) {
        return;
    }

    FAST_RAM_ZERO_INIT static uint8_t motor;
    FAST_RAM_ZERO_INIT static uint8_t harmonic;
    FAST_RAM_ZERO_INIT static uint8_t filter;
    FAST_RAM static rpmNotchFilter_t* currentFilter = &filters[0];
    FAST_RAM_ZERO_INIT static float lowestFundamentalFreq;

    for (int motor = 0; motor < getMotorCount(); motor++) {
        filteredMotorErpm[motor] = pt1FilterApply(&rpmFilters[motor], getDshotTelemetry(motor));
        if (motor < 4) {
            DEBUG_SET(DEBUG_RPM_FILTER, motor, motorFrequency[motor]);
        }

        if ( motor == 0 ) {
            lowestFundamentalFreq = filteredMotorErpm[motor] * erpmToHz;
        } else {
            lowestFundamentalFreq = MIN(lowestFundamentalFreq, filteredMotorErpm[motor] * erpmToHz);
        }
    }

    float gyroLPFCutoff;
    if  (gyroLPFMin > 0) {
        gyroLPFCutoff = lowestFundamentalFreq;
        // Cutoff at the edge of the notch. Do not go beyond 5.
        gyroLPFCutoff -=  MAX(gyroLPFCutoff / 10.0f, gyroLPFCutoff / currentFilter->q / 2.0f);
        gyroLPFCutoff = constrainf(gyroLPFCutoff, gyroLPFMin, gyroLPFMax);
        switch (gyroLPFType){
        case FILTER_PT1:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&gyroLPFFilter[axis].pt1Filter, pt1FilterGain(gyroLPFCutoff, dT));
            }
            break;
        case FILTER_BIQUAD:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&gyroLPFFilter[axis].biquadFilter, gyroLPFCutoff, gyro.targetLooptime);
            }
            break;
        default:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&gyroLPFFilter[axis].pt1Filter, pt1FilterGain(gyroLPFCutoff, dT));
            }
            break;
        }
    }

    float dTermLPFCutoff;
    if (dTermLPFMin > 0) {
        dTermLPFCutoff = lowestFundamentalFreq;
        dTermLPFCutoff -= MAX(dTermLPFCutoff / 10.0f, dTermLPFCutoff / currentFilter->q / 2.0f);
        dTermLPFCutoff = constrainf(dTermLPFCutoff, dTermLPFMin, dTermLPFMax);
        switch (dTermLPFType){
        case FILTER_PT1:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&dTermLPFFilter[axis].pt1Filter, pt1FilterGain(dTermLPFCutoff, pidLooptime * 1e-6f));
            }
            break;
        case FILTER_BIQUAD:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&dTermLPFFilter[axis].biquadFilter, dTermLPFCutoff, pidLooptime);
            }
            break;
        default:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&dTermLPFFilter[axis].biquadFilter, dTermLPFCutoff, pidLooptime);
            }
            break;
        }
    }

    for (int i = 0; i < filterUpdatesPerIteration; i++) {
        float frequency = constrainf(
            (harmonic + 1) * motorFrequency[motor], currentFilter->minHz, currentFilter->maxHz);
        biquadFilter_t* template = &currentFilter->notch[0][motor][harmonic];
        // uncomment below to debug filter stepping. Need to also comment out motor rpm DEBUG_SET above
        /* DEBUG_SET(DEBUG_RPM_FILTER, 0, harmonic); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 1, motor); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 2, currentFilter == &gyroFilter); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 3, frequency) */
        biquadFilterUpdate(
            template, frequency, currentFilter->loopTime, currentFilter->q, FILTER_NOTCH);
        for (int axis = 1; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilter_t* clone = &currentFilter->notch[axis][motor][harmonic];
            clone->b0 = template->b0;
            clone->b1 = template->b1;
            clone->b2 = template->b2;
            clone->a1 = template->a1;
            clone->a2 = template->a2;
        }

        if (++harmonic == currentFilter->harmonics) {
            harmonic = 0;
            if (++filter == numberRpmNotchFilters) {
                filter = 0;
                if (++motor == getMotorCount()) {
                    motor = 0;
                }
                motorFrequency[motor] = erpmToHz * filteredMotorErpm[motor];
            }
            currentFilter = &filters[filter];
        }

    }
}

bool isRpmFilterEnabled(void)
{
    return (motorConfig()->dev.useDshotTelemetry && (rpmFilterConfig()->gyro_rpm_notch_harmonics || rpmFilterConfig()->dterm_rpm_notch_harmonics));
}

#endif
