/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * position_external.c: Module to receive current position and yaw from external source
 */

#include <errno.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "position_external.h"
#include "debug.h"
#include "num.h"
#include "configblock.h"
#include "log.h"
#include "quatcompress.h"
// ------------------ for qick kill fix ----------------------------------------
#include "crtp_commander_high_level.h"
#include "power_distribution.h"
#include "controller.h"
//------------------------------------------------------------------------------

// Global variables
bool positionExternalFresh = false;
bool positionExternalFresh2 = false;
static bool isInit = false;
static float lastX;
static float lastY;
static float lastZ;
static float lastQ0;
static float lastQ1;
static float lastQ2;
static float lastQ3;
struct vec posExtLastRPY;
struct vec posExtLastPos;
struct vec posExtLastVel;
static uint64_t lastTime = 0;
static uint8_t my_id;
static float v_x;
static float v_y;
static float v_z;
static uint16_t dt;
//-------------------------------------by FLW-----------------------------------
static uint16_t thresKill = 800;
static bool startFlag = false;
static bool fuckYou = false;
//------------------------------------------------------------------------------
struct data_start_avoid_target avoidTarget[5];
struct data_flocking neighborDrones[5];
//------------------------------------------------------------------------------

// #define MEASURE_PACKET_DROPS
#ifdef MEASURE_PACKET_DROPS
static uint32_t packet_drop_counts[10];
static uint32_t total_packet_count;
#endif

positionInteractiveCallback interactiveCallback = NULL;

//Private functions
static void positionExternalCrtpCB(CRTPPacket* pk);

void positionExternalInit(void)
{
  if(isInit) {
    return;
  }

  // crtpInit();
  crtpRegisterPortCB(CRTP_PORT_POSEXT, positionExternalCrtpCB);

  // init drone droneIDs
  neighborDrones[0].id = -1;
  neighborDrones[1].id = -1;
  neighborDrones[2].id = -1;
  neighborDrones[3].id = -1;
  neighborDrones[4].id = -1;

  isInit = true;

  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;
  DEBUG_PRINT("posextbrinup. initialized: %d\n", my_id);
}

bool positionExternalTest(void)
{
  return isInit;
}

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
  uint16_t* last_time_in_ms)
{
  *x = lastX;
  *y = lastY;
  *z = lastZ;
  *q0 = lastQ0;
  *q1 = lastQ1;
  *q2 = lastQ2;
  *q3 = lastQ3;
  *vx = v_x;
  *vy = v_y;
  *vz = v_z;
  positionExternalUpdateDt();
  *last_time_in_ms = dt;
}

void positionExternalUpdateDt()
{
  uint64_t tickDiff = xTaskGetTickCount() - lastTime;
  if (tickDiff < 10 * 1000) {
    dt = (uint16_t)tickDiff;
  } else {
    dt = 10 * 1000;
  }
  // shut off drone after connection lost ------------------by FLW ---------
  startFlag = droneHasStarted();
  fuckYou = (dt > thresKill) && droneHasStarted();
  if (fuckYou){
    powerStop();
    controllerInit(getControllerType());
    crtpCommanderHighLevelStop();
  }
  //------------------------------------------------------------------------
}

void setPositionInteractiveCallback(positionInteractiveCallback cb)
{
  interactiveCallback = cb;
}

int8_t checkID(int8_t id)
{
  uint8_t i;
  int8_t droneIDs[6] = {neighborDrones[0].id, neighborDrones[1].id, neighborDrones[2].id, neighborDrones[3].id, neighborDrones[4].id, id}; // 5 drone IDs + 1 temp id for comparison
  for (i = 0; droneIDs[i] != id && droneIDs[i] >= 0; i++);
  if (i == 5) {
    return -1;
  }
  else {
    return i;
  }
}

int8_t checkDistance(float dist)
{
  uint8_t i;
  float droneDist[6] = {neighborDrones[0].lastDistance, neighborDrones[1].lastDistance, neighborDrones[2].lastDistance, neighborDrones[3].lastDistance, neighborDrones[4].lastDistance, dist}; // 5 drones + 1 temp for comparison
  for (i = 0; droneDist[i] != dist; i++);
  if (i == 5) {
    return -1;
  }
  else {
    return i;
  }
}

static void positionExternalCrtpCB(CRTPPacket* pk)
{
#ifdef MEASURE_PACKET_DROPS
  static uint64_t lastSeq = 0;
  struct data_packed_drops* d = ((struct data_packed_drops*)pk->data);
  if (lastSeq != 0 && d->seq > lastSeq) {
    uint64_t diff = d->seq - lastSeq - 1;
    if (diff < 9) {
      ++packet_drop_counts[diff];
    } else {
      ++packet_drop_counts[9];
    }
    ++total_packet_count;
  }

  lastSeq = d->seq;
#else
  struct data_vicon* d = ((struct data_vicon*)pk->data);
  for (int i=0; i < 2; ++i) {
    if (d->pose[i].id == my_id) {
      float x = position_fix24_to_float(d->pose[i].x);
      float y = position_fix24_to_float(d->pose[i].y);
      float z = position_fix24_to_float(d->pose[i].z);

      if (lastTime != 0) {
        float dt = (xTaskGetTickCount() - lastTime) / 1000.0f;
        dt = fmax(dt, 0.005);
        v_x = (x - lastX) / dt;
        v_y = (y - lastY) / dt;
        v_z = (z - lastZ) / dt;
      }

      lastX = x;
      lastY = y;
      lastZ = z;

      float q[4];
      quatdecompress(d->pose[i].quat, q);
      lastQ0 = q[0];
      lastQ1 = q[1];
      lastQ2 = q[2];
      lastQ3 = q[3];

      posExtLastRPY = vscl(180 / M_PI, quat2rpy(mkquat(lastQ0, lastQ1, lastQ2, lastQ3)));
      posExtLastPos = mkvec(x, y, z);
      posExtLastVel = mkvec(v_x, v_y, v_z);

      lastTime = xTaskGetTickCount();
      positionExternalFresh = true;
      positionExternalFresh2 = true;
    }
    else if (d->pose[i].id == INTERACTIVE_ID) { // && interactiveCallback != NULL) {
      float x = position_fix24_to_float(d->pose[i].x);
      float y = position_fix24_to_float(d->pose[i].y);
      float z = position_fix24_to_float(d->pose[i].z);
      //struct vec pos = mkvec(x, y, z);

      //float q[4];
      //quatdecompress(d->pose[i].quat, q);
      //struct quat quat = qloadf(q);

      avoidTarget[0].x = x;
      avoidTarget[0].y = y;
      avoidTarget[0].z = z;
      avoidTarget[0].max_displacement = 1.5f;
      avoidTarget[0].max_speed = 0.3f;

      //positionExternalTarget = true;
      // (*interactiveCallback)(&pos, &quat);
    }
    /*else if (d->pose[i].id == DRONE_ID) {
      float x = position_fix24_to_float(d->pose[i].x);
      float y = position_fix24_to_float(d->pose[i].y);
      float z = position_fix24_to_float(d->pose[i].z);
      //struct vec pos = mkvec(x, y, z);

      //float q[4];
      //quatdecompress(d->pose[i].quat, q);
      //struct quat quat = qloadf(q);

      avoidDrone.x = x;
      avoidDrone.y = y;
      avoidDrone.z = z;
      avoidDrone.max_displacement = 1.5f;
      avoidDrone.max_speed = 0.5f;

      positionExternalDrone = true;
    }*/
    else { // Prototype of checking drone distances
      int8_t temp_id = d->pose[i].id;

      float x = position_fix24_to_float(d->pose[i].x);
      float y = position_fix24_to_float(d->pose[i].y);
      /*float q[4];
      quatdecompress(d->pose[i].quat, q);
      struct vec v = quat2rpy(mkquat(q[0],q[1],q[2],q[3]));
      float yaw = v.z;*/

      float dx = lastX - x;
      float dy = lastY - y;
      float dist = sqrtf(dx*dx + dy*dy);

      if (dist < SEARCH_RADIUS) {
        int8_t indx = checkID(temp_id); // check if ID is already in the list
        if (indx != -1) {
          if (neighborDrones[indx].id == temp_id) {
            neighborDrones[indx].velocity.x = (x - neighborDrones[indx].position.x)/0.005f;
            neighborDrones[indx].velocity.y = (y - neighborDrones[indx].position.y)/0.005f;
          }
          else {
            neighborDrones[indx].id = temp_id;
            neighborDrones[indx].velocity.x = 0.0f;
            neighborDrones[indx].velocity.y = 0.0f;
          }
          neighborDrones[indx].position.x = x;
          neighborDrones[indx].position.y = y;
          neighborDrones[indx].lastDistance = dist;
        }
        else {
          int8_t indx = checkDistance(dist); // check if distance is smaller than one of the list
          if (indx != -1) {
            neighborDrones[indx].id = temp_id;
            neighborDrones[indx].velocity.x = 0.0f;
            neighborDrones[indx].velocity.y = 0.0f;
            neighborDrones[indx].position.x = x;
            neighborDrones[indx].position.y = y;
            neighborDrones[indx].lastDistance = dist;
          }
        }
      }
    }
  }
#endif
}

LOG_GROUP_START(vicon)
LOG_ADD(LOG_FLOAT, v_x, &v_x)
LOG_ADD(LOG_FLOAT, v_y, &v_y)
LOG_ADD(LOG_FLOAT, v_z, &v_z)
LOG_ADD(LOG_UINT16, dt, &dt) // zueit seit letztem packet startFlag
LOG_ADD(LOG_UINT8, startFlag, &startFlag) // by FlW ----------------------------
LOG_ADD(LOG_UINT8, fuckYou, &fuckYou) // fuck you facebook filter by FLW -------
// neighborDrones Log:
LOG_ADD(LOG_UINT8, nID0, &neighborDrones[0].id)
LOG_ADD(LOG_FLOAT, nX0, &neighborDrones[0].position.x)
LOG_ADD(LOG_FLOAT, nY0, &neighborDrones[0].position.y)
LOG_ADD(LOG_FLOAT, nVX0, &neighborDrones[0].velocity.x)
LOG_ADD(LOG_FLOAT, nVY0, &neighborDrones[0].velocity.y)
LOG_ADD(LOG_FLOAT, nDist0, &neighborDrones[0].lastDistance)
// neighborDrones Log end
LOG_ADD(LOG_FLOAT, roll, &posExtLastRPY.x)
LOG_ADD(LOG_FLOAT, pitch, &posExtLastRPY.y)
LOG_ADD(LOG_FLOAT, yaw, &posExtLastRPY.z)
LOG_ADD(LOG_FLOAT, x, &lastX)
LOG_ADD(LOG_FLOAT, y, &lastY)
LOG_ADD(LOG_FLOAT, z, &lastZ)
LOG_GROUP_STOP(vicon)

/*#ifdef MEASURE_PACKET_DROPS
LOG_GROUP_START(pacDrop)
LOG_ADD(LOG_UINT32, d0, &packet_drop_counts[0])
LOG_ADD(LOG_UINT32, d1, &packet_drop_counts[1])
LOG_ADD(LOG_UINT32, d2, &packet_drop_counts[2])
LOG_ADD(LOG_UINT32, d3, &packet_drop_counts[3])
LOG_ADD(LOG_UINT32, d4, &packet_drop_counts[4])
LOG_ADD(LOG_UINT32, d5, &packet_drop_counts[5])
LOG_ADD(LOG_UINT32, d6, &packet_drop_counts[6])
LOG_ADD(LOG_UINT32, d7, &packet_drop_counts[7])
LOG_ADD(LOG_UINT32, d8, &packet_drop_counts[8])
LOG_ADD(LOG_UINT32, d9p, &packet_drop_counts[9])
LOG_ADD(LOG_UINT32, total, &total_packet_count)
LOG_GROUP_STOP(pacDrop)
#endif*/
