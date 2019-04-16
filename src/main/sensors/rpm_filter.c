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
#include "build/debug.h"
#include "common/filter.h"
#include "common/maths.h"
#include "drivers/pwm_output_counts.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "pg/pg_ids.h"
#include "scheduler/scheduler.h"
#include "sensors/rpm_filter.h"
#include "sensors/gyro.h"

#define RPM_FILTER_MAXHARMONICS 3
#define SECONDS_PER_MINUTE      60.0f
#define ERPM_PER_LSB            100.0f
#define MIN_UPDATE_T            0.001f
#define M_PI_FLOAT  3.14159265358979323846f

#if defined(USE_RPM_FILTER)

static pt1Filter_t rpmFilters[MAX_SUPPORTED_MOTORS];
static pt1Filter_t gyroLPFFilter[XYZ_AXIS_COUNT];
static biquadFilter_t dTermLPFFilter[XYZ_AXIS_COUNT];
typedef struct rpmNotchFilter_s
{
    uint8_t harmonics;
    float   minHz;
    float   maxHz;
    float   q[3];
    float   loopTime;
    float motorLowFreq;
    float motorHighFreq;
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
FAST_RAM_ZERO_INIT static float gyroLPFMin;
static float  notch_min_cutoff_pc;
static float q_scale;
static float q_scale_cutoff;
static float dT;
static float dTermLPFMin;
static float dTermLPFMax;




PG_REGISTER_WITH_RESET_FN(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 3);

void pgResetFn_rpmFilterConfig(rpmFilterConfig_t *config)
{
    config->gyro_rpm_notch_harmonics = 3;
    config->gyro_rpm_notch_min = 100;
    config->gyro_rpm_notch_q = 500;
    config->gyro_rpm_notch_q1 = 600;
    config->gyro_rpm_notch_q2 = 600;

    config->dterm_rpm_notch_harmonics = 0;
    config->dterm_rpm_notch_min = 100;
    config->dterm_rpm_notch_q = 500;
    config->dterm_rpm_notch_q1 = 600;
    config->dterm_rpm_notch_q2 = 600;

    config->rpm_lpf = 150;
    config->rpm_notch_min_cutoff_pc = 100;
    config->rpm_q_scale = 10;
    config->rpm_q_scale_cutoff = 200;
    config->rpm_gyro_lpf = 0;
    config->rpm_dterm_lpf_min = 0;
    config->rpm_dterm_lpf_max = 0;
}

static void rpmNotchFilterInit(rpmNotchFilter_t* filter, int harmonics, int minHz, const float q[], float looptime)
{
    filter->harmonics = harmonics;
    filter->minHz = minHz;
    filter->q[0] = q[0] / 100.0f;
    filter->q[1] = q[1] / 100.0f;
    filter->q[2] = q[2] / 100.0f;
    filter->loopTime = looptime;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int motor = 0; motor < getMotorCount(); motor++) {
            for (int i = 0; i < harmonics; i++) {
                biquadFilterInit(
                    &filter->notch[axis][motor][i], minHz * i, looptime, filter->q[i], FILTER_NOTCH);
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
        float gyro_notch_q[3];
        gyro_notch_q[0] = config->gyro_rpm_notch_q;
        gyro_notch_q[1] = config->gyro_rpm_notch_q1;
        gyro_notch_q[2] = config->gyro_rpm_notch_q2;
        rpmNotchFilterInit(gyroFilter, config->gyro_rpm_notch_harmonics,
                           config->gyro_rpm_notch_min, gyro_notch_q, gyro.targetLooptime);
        // don't go quite to nyquist to avoid oscillations
        gyroFilter->maxHz = 0.48f / (gyro.targetLooptime * 1e-6f);
    }
    if (config->dterm_rpm_notch_harmonics) {
        dtermFilter = &filters[numberRpmNotchFilters++];
        float dterm_notch_q[3];
        dterm_notch_q[0] = config->dterm_rpm_notch_q;
        dterm_notch_q[1] = config->dterm_rpm_notch_q1;
        dterm_notch_q[2] = config->dterm_rpm_notch_q2;
        rpmNotchFilterInit(dtermFilter, config->dterm_rpm_notch_harmonics,
                           config->dterm_rpm_notch_min, dterm_notch_q, pidLooptime);
        // don't go quite to nyquist to avoid oscillations
        dtermFilter->maxHz = 0.48f / (pidLooptime * 1e-6f);
    }
    dT = gyro.targetLooptime * 1e-6f;
    for (int i = 0; i < getMotorCount(); i++) {
        pt1FilterInit(&rpmFilters[i], pt1FilterGain(config->rpm_lpf, pidLooptime * 1e-6f));
    }

    gyroLPFMin = config->rpm_gyro_lpf;
    if (gyroLPFMin > 0 ) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            pt1FilterInit(&gyroLPFFilter[axis], pt1FilterGain(gyroLPFMin, dT));

        }
    }
    dTermLPFMin = config->rpm_dterm_lpf_min;
    dTermLPFMax = config->rpm_dterm_lpf_max;
    if (dTermLPFMin >= dTermLPFMax ) {
        dTermLPFMin = 0;
    }

    if (dTermLPFMin > 0 ) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&dTermLPFFilter[axis], dTermLPFMin, pidLooptime);
        }
    }
    erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE  / (motorConfig()->motorPoleCount / 2.0f);

    const float loopIterationsPerUpdate = MIN_UPDATE_T / (pidLooptime * 1e-6f);
    numberFilters = getMotorCount() * (filters[0].harmonics + filters[1].harmonics);
    const float filtersPerLoopIteration = numberFilters / loopIterationsPerUpdate;
    filterUpdatesPerIteration = rintf(filtersPerLoopIteration + 0.49f);
    q_scale = config->rpm_q_scale / 10.0f;
    q_scale_cutoff = config->rpm_q_scale_cutoff;
    notch_min_cutoff_pc=config->rpm_notch_min_cutoff_pc/100.0f;

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
    float raw = value;
    float f_cut = 0;
    if (gyroLPFMin >  0) {
        // Still filter but do not use it until required.
	    value = pt1FilterApply(&gyroLPFFilter[axis],value);
		f_cut = (gyroLPFFilter[0].k) / ((dT - gyroLPFFilter[0].k * dT) * 2 * M_PI_FLOAT);
		if (gyroFilter->motorHighFreq <= gyroLPFMin) {
		    value = raw;
		}
	}
    if ( axis == 0) {
        DEBUG_SET(DEBUG_RPM_FILTER, 0, raw);
        DEBUG_SET(DEBUG_RPM_FILTER, 1, gyroFilter->motorHighFreq);
        DEBUG_SET(DEBUG_RPM_FILTER, 2, f_cut);
        DEBUG_SET(DEBUG_RPM_FILTER, 3, value);
    }
    value = applyFilter(gyroFilter, axis, value);
//    if (axis == 0 ) {
//        DEBUG_SET(DEBUG_RPM_FILTER, 3, value);
//    }
    return value;
}

float rpmFilterDterm(int axis, float value)
{
    float raw = value;

    if (dTermLPFMin > 0) {
        value = biquadFilterApply(&dTermLPFFilter[axis],value);
    }
    if ( axis == 0) {
        DEBUG_SET(DEBUG_RPM_DTERM, 0, raw);
        DEBUG_SET(DEBUG_RPM_DTERM, 1, value);
        DEBUG_SET(DEBUG_RPM_DTERM, 2, gyroFilter->motorLowFreq);
        DEBUG_SET(DEBUG_RPM_DTERM, 3, gyroFilter->motorHighFreq);
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
    FAST_RAM static uint8_t hop;

    float gyroLPFCutoff;
    for (int motor = 0; motor < getMotorCount(); motor++) {
        filteredMotorErpm[motor] = pt1FilterApply(&rpmFilters[motor], getDshotTelemetry(motor));
        if ( motor == 0 ) {
            gyroLPFCutoff = filteredMotorErpm[motor] * erpmToHz;
            currentFilter-> motorHighFreq = gyroLPFCutoff;
        } else {
            gyroLPFCutoff = MIN(gyroLPFCutoff,filteredMotorErpm[motor] * erpmToHz);
            currentFilter-> motorHighFreq = MAX (currentFilter-> motorHighFreq,filteredMotorErpm[motor] * erpmToHz);
        }
    }
    currentFilter-> motorLowFreq = gyroLPFCutoff;
    float dTermLPFCutoff;
    if (dTermLPFMin > 0) {
        dTermLPFCutoff = gyroLPFCutoff;
        dTermLPFCutoff -= dTermLPFCutoff/5.0f/2.0f;  // hard code to q 5
        dTermLPFCutoff = constrainf(dTermLPFCutoff,dTermLPFMin,dTermLPFMax);

        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterUpdateLPF(&dTermLPFFilter[axis],dTermLPFCutoff,pidLooptime);
        }
     }
    if  (gyroLPFMin > 0) {
        gyroLPFCutoff -= gyroLPFCutoff / currentFilter->q[0] / 2;  // set cutoff at the edge of the notch.
        gyroLPFCutoff = constrainf(gyroLPFCutoff,gyroLPFMin, 0.48f / (gyro.targetLooptime * 1e-6f));
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    	    pt1FilterUpdateCutoff(&gyroLPFFilter[axis], pt1FilterGain(gyroLPFCutoff, dT));
        }
        //f_cut = (gyroLPFFilter[0].k) / ((dT - gyroLPFFilter[0].k * dT) * 2 * M_PI_FLOAT);
	}


    for (int i = 0; i < filterUpdatesPerIteration; i++) {

        float frequency = (harmonic + hop + 1) * motorFrequency[motor];
        float q = currentFilter->q[harmonic];
        // Look for the next harmonic instead of parking. notch_min_cutoff_pc allow notch to go lower if required.
        // currently hard coded with q 10.
		if (frequency < notch_min_cutoff_pc * currentFilter->minHz) {
		    hop = ceilf((notch_min_cutoff_pc  * currentFilter->minHz) / (float) motorFrequency[motor]);
		    frequency = (hop--) * motorFrequency[motor];
		}

        frequency = constrainf(frequency, currentFilter->minHz * notch_min_cutoff_pc , currentFilter->maxHz);

        biquadFilter_t* template = &currentFilter->notch[0][motor][harmonic];


        if ( frequency < currentFilter -> minHz) {
            q = 10.0f;
        } else if ( frequency < q_scale_cutoff ) {  // from minHz -> q_scale_cutoff use different q is desired.
            q = q / q_scale;
        }



        // uncomment below to debug filter stepping. Need to also comment out motor rpm DEBUG_SET above
        /* DEBUG_SET(DEBUG_RPM_FILTER, 0, harmonic); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 1, motor); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 2, currentFilter == &gyroFilter); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 3, frequency) */
        biquadFilterUpdate(
            template, frequency, currentFilter->loopTime, q, FILTER_NOTCH);
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
            hop = 0;
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
