/*
 * state_space.c
 *
 *  Created on: Oct 28, 2021
 *      Author: BPaik
 */

#include "state_space.h"
#include "dsp.h"
#include "cmsis_os.h"

void init_state_space(STATE_SPACE *state_space)
{
  uint16_t i;

  state_space->x_limit[0] = PI / 2;
  state_space->x_limit[1] = 20;
  state_space->x_limit[2] = 40;

  // column 1
  state_space->A[0][0] = 1.0017;
  state_space->A[1][0] = 0.3315;
  state_space->A[2][0] = -0.1679;
  // column 2
  state_space->A[0][1] = 0.0096;
  state_space->A[1][1] = 0.9149;
  state_space->A[2][1] = -0.6727;
  // column 3
  state_space->A[0][2] = 0.0002;
  state_space->A[1][2] = 0.0245;
  state_space->A[2][2] = 0.0504;

  // vector B
  state_space->B[0] = 0.00048;
  state_space->B[1] = 0.1206;
  state_space->B[2] = 2.2819;

  for(i = 0; i < NUM_STATES; i++) {
    state_space->x[i] = 0;
    state_space->x_previous[i] = 0;
  }
}

void init_observer(STATE_OBSERVER *observer)
{
  init_state_space(&observer->ss);
  observer->L[0] = 1.4565;
  observer->L[1] = 48.5181;
  observer->L[2] = -38.5287;
}

void update_states(STATE_SPACE *state_space, float u)
{
  uint16_t i, j;
  // load states into temp buffer
  for(j = 0; j < NUM_STATES; j++) {
    state_space->x_previous[j] = state_space->x[j];
  }

  // calculate new states from previous x values
  for(i = 0; i < NUM_STATES; i++) {
    state_space->x[i] = 0;
    for(j = 0; j < NUM_STATES; j++) {
      state_space->x[i] += (state_space->A[i][j] * state_space->x_previous[j]);
    }
    state_space->x[i] += (state_space->B[i] * u);
  }
}

void update_observer(STATE_OBSERVER *observer, float feedback, float u)
{
  uint16_t i;
  // calculate observer correction terms
  observer->error = feedback - observer->ss.x[STATE_OUTPUT];
  for(i = 0; i < NUM_STATES; i++) {
    observer->correction[i] = observer->L[i] * observer->error;
  }
  // update model states
  update_states(&observer->ss, u);
  // add corrective term to state model
  for(i = 0; i < NUM_STATES; i++) {
    observer->ss.x[i] += observer->correction[i];
//    observer->ss.x[i] = LIMIT(observer->ss.x[i], observer->ss.x_limit[i], -observer->ss.x_limit[i]);
  }
}

void update_control_effort(STATE_CONTROLLER *controller, float command)
{
  uint16_t i;
  controller->r = controller->tracking_gain * command;
  controller->u = controller->r;
  for(i = 0; i < NUM_STATES; i++) {
    controller->u -= (controller->K[i] * controller->obs.ss.x[i]);
  }
}
