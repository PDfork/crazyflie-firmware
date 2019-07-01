/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 *
 */
#ifndef POSITION_EXTERNAL_H_
#define POSITION_EXTERNAL_H_

#include <stdint.h>
#include "math3d.h"
#include "packetdef.h"

void positionExternalInit(void);
bool positionExternalTest(void);

void positionExternalGetLastData(
  float* x,
  float* y,
  float* z,
  float* q0,
  float* q1,
  float* q2,
  float* q3,
  float* vx,
  float* vy,
  float* vz,
  uint16_t* last_time_in_ms);

void positionExternalUpdateDt();

extern bool positionExternalFresh;
extern bool positionExternalFresh2;
extern uint8_t numExternalTarget;
extern uint8_t numExternalDrone;
extern struct data_start_avoid_target avoidTarget[5];
extern struct data_flocking neighborDrones[5];

static float const SEARCH_RADIUS = 5.0f;
static float const SEPARATION_RADIUS = 1.5f;
static float const TARGET_RADIUS = 0.1f;
static float const ANISOTROPY = 0.5f;
static float const REP_GAIN = 2.5f;

// position of the "interactive object" i.e. in "avoid human" demo
typedef void (*positionInteractiveCallback)(struct vec const *, struct quat const *);
void setPositionInteractiveCallback(positionInteractiveCallback cb);
int8_t checkID(int8_t id);
int8_t checkDistance(float dist);

#endif /* POSITION_EXTERNAL_H_ */
