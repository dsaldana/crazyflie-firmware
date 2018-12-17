/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#include "power_distribution.h"

#include "log.h"
#include "param.h"
#include "num.h"

#include "motors.h"


static bool motorSetEnable = false;
static uint16_t motor_set_timer = 0;

static float r1 = -1;
static float r2 = -1;
static float r3 = 1;
static float r4 = 1;
static float p1 = 1;
static float p2 = -1;
static float p3 = -1;
static float p4 = 1;
static float cz = 1;


static struct {
    uint32_t m1;
    uint32_t m2;
    uint32_t m3;
    uint32_t m4;
} motorPower;

static struct {
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} motorPowerSet;

void powerDistributionInit(void) {
    motorsInit(motorMapDefaultBrushed);
}

bool powerDistributionTest(void) {
    bool pass = true;

    pass &= motorsTest();

    return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerDistribution(const control_t *control) {
#ifdef QUAD_FORMATION_X
    int16_t r = control->roll / 2.0f;
    int16_t p = control->pitch / 2.0f;

    motorPower.m1 =  limitThrust(control->thrust + r1*r + p1*p + cz*control->yaw); //M13
    motorPower.m2 =  limitThrust(control->thrust + r2*r + p2*p - cz*control->yaw); //M14
    motorPower.m3 =  limitThrust(control->thrust + r3*r + p3*p + cz*control->yaw); //M15
    motorPower.m4 =  limitThrust(control->thrust + r4*r + p4*p - cz*control->yaw); //M16

#else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll - control->yaw);
    motorPower.m3 = limitThrust(control->thrust - control->pitch + control->yaw);
    motorPower.m4 = limitThrust(control->thrust + control->roll - control->yaw);

    motorPower.m1 = limitThrust(control->thrust + control->pitch / 2 + control->yaw / 2);
    motorPower.m2 = limitThrust(control->thrust - control->roll / 2 - control->yaw / 2);
    motorPower.m3 = limitThrust(control->thrust - control->pitch / 2 + control->yaw / 2);
    motorPower.m4 = limitThrust(control->thrust + control->roll / 2 - control->yaw / 2);
#endif

    // The timer is reduced on every tick.
    if (motor_set_timer){
        motor_set_timer--;
    }

    if (motorSetEnable || motor_set_timer) {
        motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
        motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
        motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
        motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
    } else {
        motorsSetRatio(MOTOR_M1, motorPower.m1);
        motorsSetRatio(MOTOR_M2, motorPower.m2);
        motorsSetRatio(MOTOR_M3, motorPower.m3);
        motorsSetRatio(MOTOR_M4, motorPower.m4);
    }
}


PARAM_GROUP_START(var)
PARAM_ADD(PARAM_FLOAT, roll1, &r1)
PARAM_ADD(PARAM_FLOAT, roll2, &r2)
PARAM_ADD(PARAM_FLOAT, roll3, &r3)
PARAM_ADD(PARAM_FLOAT, roll4, &r4)
PARAM_ADD(PARAM_FLOAT, pitch1, &p1)
PARAM_ADD(PARAM_FLOAT, pitch2, &p2)
PARAM_ADD(PARAM_FLOAT, pitch3, &p3)
PARAM_ADD(PARAM_FLOAT, pitch4, &p4)
PARAM_ADD(PARAM_FLOAT, czz, &cz)
PARAM_GROUP_STOP(var)

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, motor_timer, &motor_set_timer)  // Enable based on timer
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(ring)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)
