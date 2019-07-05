/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * sitAw.h - Implementation of situation awareness.
 */

#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "math3d.h"

#include "log.h"
#include "param.h"
#include "trigger.h"
#include "sitaw.h"
#include "commander.h"
#include "position_external.h"

/* Trigger object used to detect Free Fall situation. */
static trigger_t sitAwFFAccWZ;

/* Trigger object used to detect At Rest situation. */
static trigger_t sitAwARAccZ;

/* Trigger object used to detect Tumbled situation. */
static trigger_t sitAwTuAngle;

/* Trigger object used to detect collision avoidance. */
static uint8_t sitAwCAActive = 0;

/* Trigger object used to break. */
static uint8_t sitAwBreak = 0;

// Break point of the crazyflie
struct vec breakpoint;
bool stopped = false;

static uint64_t lastTime = 0;

// Log variables
static float targetX;
static float targetY;
static float stateX;
static float stateY;
static float offX;
static float offY;

#if defined(SITAW_ENABLED)

#if defined(SITAW_LOG_ENABLED) /* Enable the log group. */
LOG_GROUP_START(sitAw)
#if defined(SITAW_FF_LOG_ENABLED) /* Log trigger variables for Free Fall detection. */
LOG_ADD(LOG_UINT32, FFAccWZTestCounter, &sitAwFFAccWZ.testCounter)
LOG_ADD(LOG_UINT8, FFAccWZDetected, &sitAwFFAccWZ.released)
#endif
#if defined(SITAW_AR_LOG_ENABLED) /* Log trigger variables for At Rest detection. */
LOG_ADD(LOG_UINT32, ARTestCounter, &sitAwARAccZ.testCounter)
LOG_ADD(LOG_UINT8, ARDetected, &sitAwARAccZ.released)
#endif
#if defined(SITAW_TU_LOG_ENABLED) /* Log trigger variables for Tumbled detection. */
LOG_ADD(LOG_UINT32, TuTestCounter, &sitAwTuAngle.testCounter)
LOG_ADD(LOG_UINT8, TuDetected, &sitAwTuAngle.released)
#endif
#if defined(SITAW_LOG_ALL_DETECT_ENABLED) /* Log all the 'Detected' flags. */
LOG_ADD(LOG_UINT8, FFAccWZDetected, &sitAwFFAccWZ.released)
LOG_ADD(LOG_UINT8, ARDetected, &sitAwARAccZ.released)
LOG_ADD(LOG_UINT8, TuDetected, &sitAwTuAngle.released)
#endif
LOG_GROUP_STOP(sitAw)
LOG_GROUP_START(colAw)
LOG_ADD(LOG_FLOAT, tX, &targetX)
LOG_ADD(LOG_FLOAT, tY, &targetY)
LOG_ADD(LOG_FLOAT, stX, &stateX)
LOG_ADD(LOG_FLOAT, stY, &stateY)
LOG_ADD(LOG_FLOAT, offX, &offX)
LOG_ADD(LOG_FLOAT, offY, &offY)
LOG_GROUP_STOP(colAw)
#endif /* SITAW_LOG_ENABLED */

#if defined(SITAW_PARAM_ENABLED) /* Enable the param group. */
PARAM_GROUP_START(sitAw)
#if defined(SITAW_FF_PARAM_ENABLED) /* Param variables for Free Fall detection. */
PARAM_ADD(PARAM_UINT8, FFActive, &sitAwFFAccWZ.active)
PARAM_ADD(PARAM_UINT32, FFTriggerCount, &sitAwFFAccWZ.triggerCount)
PARAM_ADD(PARAM_FLOAT, FFaccWZ, &sitAwFFAccWZ.threshold)
#endif
#if defined(SITAW_AR_PARAM_ENABLED) /* Param variables for At Rest detection. */
PARAM_ADD(PARAM_UINT8, ARActive, &sitAwARAccZ.active)
PARAM_ADD(PARAM_UINT32, ARTriggerCount, &sitAwARAccZ.triggerCount)
PARAM_ADD(PARAM_FLOAT, ARaccZ, &sitAwARAccZ.threshold)
#endif
#if defined(SITAW_TU_PARAM_ENABLED) /* Param variables for Tumbled detection. */
PARAM_ADD(PARAM_UINT8, TuActive, &sitAwTuAngle.active)
PARAM_ADD(PARAM_UINT32, TuTriggerCount, &sitAwTuAngle.triggerCount)
PARAM_ADD(PARAM_FLOAT, TuAngle, &sitAwTuAngle.threshold)
#endif
#if defined(SITAW_CA_PARAM_ENABLED) /* Param variables for collision avoidance. */
PARAM_ADD(PARAM_UINT8, CAActive, &sitAwCAActive)
PARAM_ADD(PARAM_UINT8, Break, &sitAwBreak)
PARAM_ADD(PARAM_FLOAT, SearchRadius, &SEARCH_RADIUS)
PARAM_ADD(PARAM_FLOAT, SeparationRadius, &SEPARATION_RADIUS)
PARAM_ADD(PARAM_FLOAT, TargetRadius, &TARGET_RADIUS)
PARAM_ADD(PARAM_FLOAT, Anisotropy, &ANISOTROPY)
PARAM_ADD(PARAM_FLOAT, RepGain, &REP_GAIN)
PARAM_ADD(PARAM_FLOAT, MaxSpeed, &MAX_SPEED)
#endif
PARAM_GROUP_STOP(sitAw)
#endif /* SITAW_PARAM_ENABLED */

#endif /* SITAW_ENABLED */

/**
 * Initialize the Free Fall detection.
 *
 * See the sitAwFFTest() function for details.
 */
void sitAwFFInit(void)
{
  triggerInit(&sitAwFFAccWZ, triggerFuncIsLE, SITAW_FF_THRESHOLD, SITAW_FF_TRIGGER_COUNT);
  triggerActivate(&sitAwFFAccWZ, true);
}


static void sitAwPostStateUpdateCallOut(const sensorData_t *sensorData,
                                        const state_t *state)
{
  /* Code that shall run AFTER each attitude update, should be placed here. */

#if defined(SITAW_ENABLED)
#ifdef SITAW_FF_ENABLED
  float accMAG = (sensorData->acc.x*sensorData->acc.x) +
                 (sensorData->acc.y*sensorData->acc.y) +
                 (sensorData->acc.z*sensorData->acc.z);

  /* Test values for Free Fall detection. */
  sitAwFFTest(state->acc.z, accMAG);
#endif
#ifdef SITAW_TU_ENABLED
  /* Test values for Tumbled detection. */
  sitAwTuTest(state->attitude.roll, state->attitude.pitch);
#endif
#ifdef SITAW_AR_ENABLED
/* Test values for At Rest detection. */
  sitAwARTest(sensorData->acc.x, sensorData->acc.y, sensorData->acc.z);
#endif
#endif
}

static void sitAwPreThrustUpdateCallOut(setpoint_t *setpoint)
{
  /* Code that shall run BEFORE each thrust distribution update, should be placed here. */

#if defined(SITAW_ENABLED)
#ifdef SITAW_TU_ENABLED
      if(sitAwTuDetected()) {
        /* Kill the thrust to the motors if a Tumbled situation is detected. */
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.z = modeDisable;
        setpoint->thrust = 0;
      }
#endif

#ifdef SITAW_FF_ENABLED
      /* Force altHold mode if free fall is detected.
         FIXME: Needs a flying/landing state (as soon as althold is enabled,
                                              we are not freefalling anymore)
       */
      if(sitAwFFDetected() && !sitAwTuDetected()) {
        setpoint->mode.z = modeVelocity;
        setpoint->velocity.z = 0;
      }
#endif
#endif
}

static void sitAwCollisionAvoidance(setpoint_t *setpoint, const state_t *state)
{
  #if defined(SITAW_ENABLED)
  #ifdef SITAW_CA_ENABLED
  if (sitAwCAActive > 0) {
    // Routine for avoiding the obstacle or a drone

    float dt = (xTaskGetTickCount() - lastTime) / 1000.0f;
    dt = fmax(dt, 0.005);

    // 2D case (x,y)
    struct vec rt = {setpoint->position.x, setpoint->position.y, 0.0f};
    struct vec ri = {state->position.x, state->position.y, 0.0f};

    struct vec rit = vsub(rt,ri);

    float alpha; // angle between rit, vj
    float theta; // angle between rij, rit
    float rho; // angle between uij, -rit
    struct vec vj;
    struct vec vi_rep = {0.0f,0.0f,0.0f};
    struct vec rij;
    uint8_t j;
    for (j = 0; j < 5; j++) {

      if (neighborDrones[j].id > 0) {
        vj = neighborDrones[j].velocity;
        rij = vsub(neighborDrones[j].position, ri);

        float umag = 0.0f; // magnitude of repulsion
        float rmag = vmag(rij); // magnitude of rij

        if (rmag < SEPARATION_RADIUS) {
          umag = REP_GAIN * (SEPARATION_RADIUS - rmag);

          alpha = vangle(rit, vj);
          theta = vangle(rij, rit);

          if (alpha <= M_PI_F/3.0f) {
            if (theta <= M_PI_F/2) {
              rho = ANISOTROPY * theta;
            } else {
              rho = M_PI_F + ANISOTROPY * (theta - M_PI_F);
            }
          } else {
            rho = (1.0f - ANISOTROPY/2.0f) * (theta - M_PI_F) + M_PI_F;
          }

          float ux = cosf(rho) * -rit.x + sinf(rho) * -rit.y;
          float uy = sinf(rho) * -rit.x + cosf(rho) * -rit.y;
          struct vec uij = vnormalize(mkvec(ux, uy, 0.0f)); // new direction

          vi_rep = vadd(vi_rep, vscl(umag, uij));
        }
      }
    }

    struct vec offset = vi_rep;
    float rit_mag = vmag(rit);
    if (TARGET_RADIUS < rit_mag) {
      offset = vadd(offset, vscl(MAX_SPEED,vnormalize(rit)));
    }
    setpoint->position.x = state->position.x + offset.x*dt;
    setpoint->position.y = state->position.y + offset.y*dt;
    setpoint->velocity.x = offset.x;
    setpoint->velocity.y = offset.y;

    targetX = rt.x;
    targetY = rt.y;
    stateX = setpoint->position.x;
    stateY = setpoint->position.y;
    offX = offset.x;
    offY = offset.y;
/*
    // 1. Alignment
    uint8_t i, count;
    struct vec v = mkvec(0.0f,0.0f,0.0f);
    for (i = 0, count = 0; i < 5; i++) {
      if (neighborDrones[i].id >= 0) {
        v.x = v.x + cosf(neighborDrones[i].yaw);
        v.y = v.y + sinf(neighborDrones[i].yaw);
        count++;
      }
    }
    v.x = v.x/count;
    v.y = v.y/count;
    v = vnormalize(v);
    struct vec v_align = v;

    // 2. Cohesion
    v = mkvec(0.0f,0.0f,0.0f);
    for (i = 0, count = 0; i < 5; i++) {
      if (neighborDrones[i].id >= 0) {
        v.x = v.x + neighborDrones[i].x;
        v.y = v.y + neighborDrones[i].y;
        count++;
      }
    }
    v.x = v.x/count;
    v.y = v.y/count;
    v = vnormalize(v);
    struct vec v_cohese = v;

    // 3. Separation
    v = mkvec(0.0f,0.0f,0.0f);
    float dist;
    struct vec v_dist;02131 1538050
    for (i = 0, count = 0; i < 5; i++) {
      v_dist = vsub(mkvec(state->position.x,state->position.y,0.0f), mkvec(neighborDrones[i].x,neighborDrones[i].y,0.0f));
      dist = vmag(v_dist);
      if (neighborDrones[i].id >= 0 && d < SEPARATION_RADIUS) {
        v_dist = vdiv(vnormalize(vdist),dist);
        v.x = v.x + v_dist.x;
        v.y = v.y + v_dist.y;
        count++;
      }
    }
    v.x = v.x/count;
    v.y = v.y/count;
    struct vec v_separate = v;

    struct vec offset = vadd3(vscl(0.25f,v_align),vscl(0.25f,v_cohese),vscl(0.5f,v_separate));

    setpoint->position.x = setpoint->position.x + offset.x;
    setpoint->position.y = setpoint->position.y + offset.y;
  */

  }
  if (sitAwBreak == 0) {
    stopped = false;
  } else {
    // Break and stop the crazyflie mid-air
    if (stopped == false) {
      breakpoint.x = state->position.x;
      breakpoint.y = state->position.y;
      breakpoint.z = state->position.z;
      stopped = true;
    }

      setpoint->position.x = breakpoint.x;
      setpoint->position.y = breakpoint.y;
      setpoint->position.z = breakpoint.z;
      setpoint->velocity.x = 0.0;
      setpoint->velocity.y = 0.0;
      setpoint->velocity.z = 0.0;
      setpoint->acceleration.x = 0.0;
      setpoint->acceleration.y = 0.0;
      setpoint->acceleration.z = 0.0;
  }
  lastTime = xTaskGetTickCount();
  #endif
  #endif
}

/**
 * Update setpoint according to current situation
 *
 * Called by the stabilizer after state and setpoint update. This function
 * should update the setpoint accordig to the current state situation
 */
void sitAwUpdateSetpoint(setpoint_t *setpoint, const sensorData_t *sensorData,
                                               const state_t *state)
{
  sitAwCollisionAvoidance(setpoint, state);
  sitAwPostStateUpdateCallOut(sensorData, state);
  sitAwPreThrustUpdateCallOut(setpoint);
}

/**
 * Test values for a Free Fall situation.
 *
 * A free fall situation is considered identified when the vertical
 * acceleration of the crazyflie (regardless of orientation - given by
 * AccWZ) is approaching -1 (AccWZ is 0 when crazyflie is at rest). We
 * will look for when AccWZ is within SITAW_FF_THRESHOLD of -1.
 *
 * At the same time, there should be no other accelerations experienced
 * by the crazyflie.

 * This can be checked by looking at the accMAG (total acceleration). If
 * the accMAG is approaching 0, there are no other accelerations than accWZ.
 * This helps to distinguish free fall situations from other movements
 * such as shaking.
 *
 * @param accWZ  Vertical acceleration (regardless of orientation)
 * @param accMAG All experienced accelerations.
 *
 * @return True if the situation has been detected, otherwise false.
 */
bool sitAwFFTest(float accWZ, float accMAG)
{
  /* Check that the total acceleration is close to zero. */
  if(fabs(accMAG) > SITAW_FF_THRESHOLD) {
    /* If the total acceleration deviates from 0, this is not a free fall situation. */
    triggerReset(&sitAwFFAccWZ);
    return false;
  }

  /**
   * AccWZ approaches -1 in free fall. Check that the value stays within
   * SITAW_FF_THRESHOLD of -1 for the triggerCount specified.
   */
  return(triggerTestValue(&sitAwFFAccWZ, fabs(accWZ + 1)));
}

/**
 * Check if a Free Fall situation has been detected.
 *
 * @return True if the situation has been detected, otherwise false.
 */
bool sitAwFFDetected(void)
{
  return sitAwFFAccWZ.released;
}

/**
 * Initialize the At Rest detection.
 *
 * See the sitAwARTest() function for details.
 */
void sitAwARInit(void)
{
  triggerInit(&sitAwARAccZ, triggerFuncIsLE, SITAW_AR_THRESHOLD, SITAW_AR_TRIGGER_COUNT);
  triggerActivate(&sitAwARAccZ, true);
}

/**
 * Test values for an At Rest situation.
 *
 * An At Rest situation is considered identified when the crazyflie is
 * placed on its feet (accZ = 1) and with no horizontal accelerations
 * (accX = accY = 0).
 *
 * Since there is always some minor noise in the measurements, we use
 * a margin of SITAW_AR_THRESHOLD from the ideal values. Since this function
 * does not check for thrust, the SITAW_AR_THRESHOLD is assumed to be set
 * sufficiently close to the absolute resting values so that these values cannot
 * be achieved (over time) during hovering or flight.
 *
 * @param accX   Horizontal X acceleration (when crazyflie is placed on its feet)
 * @param accY   Horizontal Y acceleration (when crazyflie is placed on its feet)
 * @param accZ   Vertical Z acceleration (when crazyflie is placed on its feet)
 *
 * @return True if the situation has been detected, otherwise false.
 */
bool sitAwARTest(float accX, float accY, float accZ)
{
  /* Check that there are no horizontal accelerations. At rest, these are 0. */
  if((fabs(accX) > SITAW_AR_THRESHOLD) || (fabs(accY) > SITAW_AR_THRESHOLD)) {
    /* If the X or Y accelerations are different than 0, the crazyflie is not at rest. */
    triggerReset(&sitAwARAccZ);
    return(false);
  }

  /**
   * If the test above indicates that there are no horizontal movements, test the
   * vertical acceleration value against the trigger.
   *
   * The vertical acceleration must be close to 1, but is allowed to oscillate slightly
   * around 1. Testing that the deviation from 1 stays within SITAW_AR_THRESHOLD.
   */
  return(triggerTestValue(&sitAwARAccZ, fabs(accZ - 1)));
}

/**
 * Check if an At Rest situation has been detected.
 *
 * @return True if the situation has been detected, otherwise false.
 */
bool sitAwARDetected(void)
{
  return sitAwARAccZ.released;
}

/**
 * Initialize the Tumbled detection.
 *
 * See the sitAwTuTest() function for details.
 */
void sitAwTuInit(void)
{
  triggerInit(&sitAwTuAngle, triggerFuncIsGE, SITAW_TU_THRESHOLD, SITAW_TU_TRIGGER_COUNT);
  triggerActivate(&sitAwTuAngle, true);
}

/**
 * Test values for a Tumbled situation.
 *
 * A tumbled situation is considered identified when the roll or pitch has
 * exceeded +/- SITAW_TU_THRESHOLD degrees.
 *
 * For thresholds beyond +/- 90 degrees, this is only reported by the roll
 * value. The roll value is thus the only one of roll, pitch and yaw values
 * which can detect upside down situations.
 *
 * Once a tumbled situation is identified, this can be used for instance to
 * cut the thrust to the motors, avoiding the crazyflie from running
 * propellers at significant thrust when accidentially crashing into walls
 * or the ground.

 * @param The actual roll in degrees. +180/-180 degrees means upside down.
 * @param The actual pitch in degrees. 0 degrees means horizontal.
 *
 * @return True if the situation has been detected, otherwise false.
 */
bool sitAwTuTest(float eulerRollActual, float eulerPitchActual)
{
  /*
   * It is sufficient to use a single trigger object, we simply pass the
   * greatest of the roll and pitch absolute values to the trigger object
   * at any given time.
   */
  float fAbsRoll  = fabs(eulerRollActual);
  float fAbsPitch = fabs(eulerPitchActual);

  /* Only the roll value will report if the crazyflie is turning upside down. */
  return(triggerTestValue(&sitAwTuAngle, fAbsRoll >= fAbsPitch ? fAbsRoll : fAbsPitch));
}

/**
 * Check if a Tumbled situation has been detected.
 *
 * @return True if the situation has been detected, otherwise false.
 */
bool sitAwTuDetected(void)
{
  return sitAwTuAngle.released;
}

/**
 * Initialize the situation awareness subsystem.
 */
void sitAwInit(void)
{
#ifdef SITAW_FF_ENABLED
  sitAwFFInit();
#endif
#ifdef SITAW_AR_ENABLED
  sitAwARInit();
#endif
#ifdef SITAW_TU_ENABLED
  sitAwTuInit();
#endif
}
