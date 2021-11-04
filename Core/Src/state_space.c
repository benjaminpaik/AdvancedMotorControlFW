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

  state_space->x_limit[0] = PI / 3;
  state_space->x_limit[1] = 20;
  state_space->x_limit[2] = 40;

  // column 1
  state_space->A[0][0] = 1.002;
  state_space->A[1][0] = 0.3315;
  state_space->A[2][0] = -0.1679;
  // column 2
  state_space->A[0][1] = 0.009606;
  state_space->A[1][1] = 0.9149;
  state_space->A[2][1] = -0.6727;
  // column 3
  state_space->A[0][2] = 0.0001773;
  state_space->A[1][2] = 0.02451;
  state_space->A[2][2] = 0.05037;

  // vector B
  state_space->B[0] = 0.00048;
  state_space->B[1] = 0.1206;
  state_space->B[2] = 2.282;

  for(i = 0; i < NUM_STATES; i++) {
    state_space->x[i] = 0;
    state_space->x_previous[i] = 0;
  }
}

void init_observer(STATE_OBSERVER *observer)
{
  init_state_space(&observer->ss);
  observer->L[0] = 1.4407;
  observer->L[1] = 47.3589;
  observer->L[2] = -37.6072;
}

void update_observer(STATE_OBSERVER *observer, float feedback)
{
  uint16_t i;
  // calculate observer correction terms
  observer->error = feedback - observer->ss.x[STATE_OUTPUT];
  for(i = 0; i < NUM_STATES; i++) {
    observer->correction[i] = observer->L[i] * observer->error;
  }
  // update model states
  update_states(&observer->ss, observer->u);
  // add corrective term to state model
  for(i = 0; i < NUM_STATES; i++) {
    observer->ss.x[i] += observer->correction[i];
//    observer->ss.x[i] = LIMIT(observer->ss.x[i], observer->ss.x_limit[i], -observer->ss.x_limit[i]);
  }
}

void update_control_effort(STATE_OBSERVER *observer, float reference)
{
  uint16_t i;
  observer->u = reference;
  for(i = 0; i < NUM_STATES; i++) {
    observer->u -= (observer->K[i] * observer->ss.x[i]);
  }
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
