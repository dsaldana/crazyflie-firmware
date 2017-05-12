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

static float r1 = 0;
static float r2 = 0;
static float r3 = 0;
static float r4 = 0;
static float p1 = 0;
static float p2 = 0;
static float p3 = 0;
static float  p4 = 0;
static float tr = 0;



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
	int16_t r = 10000; //control->roll / 2.0f;
	int16_t p = 10000; //control->pitch / 2.0f;

	motorPower.m1 = limitThrust(tr + r1*r + p1*p + control->yaw);
	motorPower.m2 = limitThrust(tr + r2*r + p2*p + control->yaw);
	motorPower.m3 = limitThrust(tr + r3*r + p3*p + control->yaw);
	motorPower.m4 = limitThrust(tr + r4*r + p4*p + control->yaw);
	//Robot1 in the most left

//	motorPower.m1 = limitThrust(control->thrust - 6.5f*0.24f*(r/2) + (p/2.0f) + (control->yaw/2.0f));
//	motorPower.m2 = limitThrust(control->thrust - 6.5f*0.24f*(r/2) - (p/2.0f) - (control->yaw/2.0f));
//	motorPower.m3 = limitThrust(control->thrust - 6.5f*0.76f*(r/2) - (p/2.0f) + (control->yaw/2.0f));
//    motorPower.m4 = limitThrust(control->thrust - 6.5f*0.76f*(r/2) + (p/2.0f) - (control->yaw/2.0f));

//	motorPower.m1 = limitThrust(control->thrust - 6.5f*0.36f*(r/2) + (p/2.0f) + (control->yaw/2.0f));
//	motorPower.m2 = limitThrust(control->thrust - 6.5f*0.36f*(r/2) - (p/2.0f) - (control->yaw/2.0f));
//	motorPower.m3 = limitThrust(control->thrust - 6.5f*1.18f*(r/2) - (p/2.0f) + (control->yaw/2.0f));
//    motorPower.m4 = limitThrust(control->thrust - 6.5f*1.18f*(r/2) + (p/2.0f) - (control->yaw/2.0f));

	//Robot 2 in the most right

	// motorPower.m1 = limitThrust(control->thrust  + 6.5f*0.76f*(r/2) + (p/2.0f) + (control->yaw/2.0f));  //motor6
	 //motorPower.m2 = limitThrust(control->thrust  + 6.5f*0.76f*(r/2) - (p/2.0f) - (control->yaw/2.0f));  //motor7
	 //motorPower.m3 =  limitThrust(control->thrust + 6.5f*0.24f*(r/2) - (p/2.0f) + (control->yaw/2.0f)); //motor8
	 //motorPower.m4 =  limitThrust(control->thrust + 6.5f*0.24f*(r/2) + (p/2.0f) - (control->yaw/2.0f)); //motor9


//	 motorPower.m1 = limitThrust(control->thrust  + 6.5f*1.18f*(r/2) + (p/2.0f) + (control->yaw/2.0f));  //motor6
//	 motorPower.m2 = limitThrust(control->thrust  + 6.5f*1.18f*(r/2) - (p/2.0f) - (control->yaw/2.0f));  //motor7
//	 motorPower.m3 =  limitThrust(control->thrust + 6.5f*0.36f*(r/2) - (p/2.0f) + (control->yaw/2.0f)); //motor8
//	 motorPower.m4 =  limitThrust(control->thrust + 6.5f*0.36f*(r/2) + (p/2.0f) - (control->yaw/2.0f)); //motor9


//float dc = 33.; //mm
//float d1 = 210; //mm
//float d4 = 143; //mm
//float d5 = 95; //mm
//float d8 = 28; //mm




//////Robot1
//
//       motorPower.m1 =  limitThrust(control->thrust - 24*(r/4.0f)*(dc/d1) + p/4.0f + control->yaw/4.0f); //M1
//       motorPower.m2 =  limitThrust(control->thrust - 24*(r/4.0f)*(dc/d1) - p/4.0f - control->yaw/4.0f); //M2
//       motorPower.m3 =  limitThrust(control->thrust - 24*(r/4.0f)*(dc/d4) - p/4.0f - control->yaw/4.0f); //M3
//       motorPower.m4 =  limitThrust(control->thrust - 24*(r/4.0f)*(dc/d4) + p/4.0f + control->yaw/4.0f); //M4

//
////Robot2

//       motorPower.m1 =  limitThrust(control->thrust - 24*(r/4.0f)*(dc/d5) + p/4.0f + control->yaw/4.0f); //M5
//       motorPower.m2 =  limitThrust(control->thrust - 24*(r/4.0f)*(dc/d5) - p/4.0f - control->yaw/4.0f); //M6
//       motorPower.m3 =  limitThrust(control->thrust - 24*(r/4.0f)*(dc/d8) - p/4.0f - control->yaw/4.0f); //M7
//       motorPower.m4 =  limitThrust(control->thrust - 24*(r/4.0f)*(dc/d8) + p/4.0f + control->yaw/4.0f); //M8
////
//
//////Robot3
//
//        motorPower.m1 =  limitThrust(control->thrust + 24*(r/4.0f)*(dc/d8) + p/4.0f - control->yaw/4.0f); //M9
//        motorPower.m2 =  limitThrust(control->thrust + 24*(r/4.0f)*(dc/d8) - p/4.0f + control->yaw/4.0f); //M10
//        motorPower.m3 =  limitThrust(control->thrust + 24*(r/4.0f)*(dc/d5) - p/4.0f + control->yaw/4.0f); //M11
//        motorPower.m4 =  limitThrust(control->thrust + 24*(r/4.0f)*(dc/d5) + p/4.0f - control->yaw/4.0f); //M12
////
////
////Robot4
//
//        motorPower.m1 =  limitThrust(control->thrust + 50*(r/4.0f)*(dc/d4) + p/4.0f - control->yaw/4.0f); //M13
//        motorPower.m2 =  limitThrust(control->thrust + 50*(r/4.0f)*(dc/d4) - p/4.0f + control->yaw/4.0f); //M14
//        motorPower.m3 =  limitThrust(control->thrust + 50*(r/4.0f)*(dc/d1) - p/4.0f + control->yaw/4.0f); //M15
//        motorPower.m4 =  limitThrust(control->thrust + 50*(r/4.0f)*(dc/d1) + p/4.0f - control->yaw/4.0f); //M16

        ////Robot1 Official

//               motorPower.m1 =  limitThrust(control->thrust + 3.0f*(r) + 3.0f*p + control->yaw); //M1
//               motorPower.m2 =  limitThrust(control->thrust + 3.0f*(r) - 3.0f*p - control->yaw); //M2
//               motorPower.m3 =  limitThrust(control->thrust + 3.0f*(r) - 3.0f*p + control->yaw); //M3
//               motorPower.m4 =  limitThrust(control->thrust + 3.0f*(r) + 3.0f*p - control->yaw); //M4

        //
        ////Robot2

//               motorPower.m1 =  limitThrust(control->thrust - 5.0f*(r) + p/4.0f + control->yaw/4.0f); //M5
//               motorPower.m2 =  limitThrust(control->thrust - 5.0f*(r) - p/4.0f - control->yaw/4.0f); //M6
//               motorPower.m3 =  limitThrust(control->thrust - 5.0f*(r) - p/4.0f - control->yaw/4.0f); //M7
//               motorPower.m4 =  limitThrust(control->thrust - 5.0f*(r) + p/4.0f + control->yaw/4.0f); //M8
        //
        //
        //////Robot3
        //
//                motorPower.m1 =  limitThrust(control->thrust + 5.0f*(r) + p/4.0f - control->yaw/4.0f); //M9
//                motorPower.m2 =  limitThrust(control->thrust + 5.0f*(r) - p/4.0f + control->yaw/4.0f); //M10
//                motorPower.m3 =  limitThrust(control->thrust + 5.0f*(r) - p/4.0f + control->yaw/4.0f); //M11
//                motorPower.m4 =  limitThrust(control->thrust + 5.0f*(r) + p/4.0f - control->yaw/4.0f); //M12
        ////
        ////
        ////Robot4
        //
//                motorPower.m1 =  limitThrust(control->thrust + 3.0f*(r) - 3.0f*p + control->yaw); //M13
//                motorPower.m2 =  limitThrust(control->thrust + 3.0f*(r) - 3.0f*p - control->yaw); //M14
//                motorPower.m3 =  limitThrust(control->thrust + 3.0f*(r) - 3.0f*p + control->yaw); //M15
//                motorPower.m4 =  limitThrust(control->thrust + 3.0f*(r) - 3.0f*p - control->yaw); //M16


#else // QUAD_FORMATION_NORMAL
	motorPower.m1 = limitThrust(
			control->thrust + control->pitch + control->yaw);
	motorPower.m2 = limitThrust(control->thrust - control->roll - control->yaw);
	motorPower.m3 = limitThrust(
			control->thrust - control->pitch + control->yaw);
	motorPower.m4 = limitThrust(control->thrust + control->roll - control->yaw);

	motorPower.m1 = limitThrust(
			control->thrust + control->pitch / 2 + control->yaw / 2);
	motorPower.m2 = limitThrust(
			control->thrust - control->roll / 2 - control->yaw / 2);
	motorPower.m3 = limitThrust(
			control->thrust - control->pitch / 2 + control->yaw / 2);
	motorPower.m4 = limitThrust(
			control->thrust + control->roll / 2 - control->yaw / 2);
#endif

	if (motorSetEnable) {
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
PARAM_ADD(PARAM_FLOAT, thrust, &tr)
PARAM_GROUP_STOP(var)

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(ring)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_GROUP_STOP(motor)
