/**
******************************************************************************
* @file    funclib.cpp
* @author  Ran Wang EDPLab@TAMU
* @version V1.0
* @date    2018-4-28
* @brief   This file provides the test functions to be integrated into the mujoco project.
******************************************************************************
* <h2><center>&copy; COPYRIGHT EDPLab@TAMU</center></h2>
******************************************************************************
*/

/* Includes -----------------------------------------------------------------*/

#include "funclib.h"

/* Extern variables -------------------------------------------------------*/
// constants
const int kMaxStep = 1510;   // max step number for one rollout
const int kMaxState = 40;	// max (state dimension, actuator number)

// model parameters and environment settings
int integration_per_step = 1;
int stepnum;
int actuatornum;
int statenum;
int modelid;
int rolloutnum_train;
mjtNum control_timestep;
mjtNum simulation_timestep;
mjtNum ctrl_upperlimit_train = 100;
mjtNum ctrl_lowerlimit_train = -100;
mjtNum state_nominal[kMaxStep][kMaxState] = { 0 };
mjtNum ctrl_nominal[kMaxStep * kMaxState] = { 0 };
mjtNum state_target[kMaxState] = { 0 };
mjtNum stabilizer_feedback_gain[kMaxState][kMaxState] = { 0 };

// hyperparameters 
mjtNum Q, QT, R;
mjtNum Qm[kMaxState][kMaxState], QTm[kMaxState][kMaxState];

/* Private typedef ----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Mujoco function prototypes------------------------------------------------*/

/**
* @brief  Cost function
* @note   needs to be modified according to different tasks
* @param  mjModel* m: model
*         mjData* d: data
*         mjtNum *ctrl: control value
*         mjtNum nu: number of control
*         mjtNum *Q: state error penalty
*         mjtNum R: control penalty
* @retval mjtNum: cost value
* @author rwang0417@tamu.edu
*/
mjtNum mjc_costFunction(mjModel* m, mjData* d, mjtNum *ctrl, mjtNum nu, mjtNum *Q, mjtNum R)
{
	mjtNum c = 0, s = 0;
	mjtNum res0[4], res1[4];

	for (int i = 0; i < nu; i++) c += R * ctrl[i] * ctrl[i];

	//modify
	//s = Q[0] * ((d->geom_xpos[6] - 1) * (d->geom_xpos[6] - 1) + (d->geom_xpos[7] - 1) * (d->geom_xpos[7] - 1));
	//mju_mulMatVec(res1, Q, res0, 4, 4);
	//s = mju_dot(res0, res1, 4);
	//modify

	return s + c;
}

/**
* @brief  Set the model to initial state
* @note   none
* @param  mjModel* m: model
*         mjData* d: data
* @retval none
* @author rwang0417@tamu.edu
*/
void mjc_modelInit(mjModel* m, mjData* d)
{
	mju_zero(d->qpos, m->nq);
	mju_zero(d->qvel, m->nv);
	mju_zero(d->act, m->na);
	mju_zero(d->ctrl, m->nu);
}

/* Private general function prototypes---------------------------------------*/
mjtNum angleModify(int model, mjtNum angle)
{
	if (model == 0)
		return (PI - fabs(angle - PI))*((PI - angle > 0) - (PI - angle < 0));
	return 0;
}

mjtNum randGauss(mjtNum mean, mjtNum var)
{
	mjtNum U, V, Z;

	U = rand() / (RAND_MAX + 1.0);
	V = rand() / (RAND_MAX + 1.0);
	if (U == 0) U = 0.0001;
	Z = sqrt(-2.0 * log(U)) * sin(2.0 * PI * V);

	return mean + sqrt(var) * Z;
}

int modelSelection(const char* model)
{
	if (_strcmpi(model, "pendulum") == 0) {
		modelid = 0;
		control_timestep = 0.1;
		simulation_timestep = 0.1;
		stepnum = 30;
		statenum = 2;
		actuatornum = 1;
		rolloutnum_train = 240;
		mjtNum temp[kMaxState][kMaxState] = { 11.0396, 3.0658 };// { 5.4995, 1.2227 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], statenum);
		mjtNum temp1[kMaxState] = { PI, 0.0 };
		mju_copy(state_nominal[0], temp1, statenum);
		mjtNum temp2[kMaxState] = { 2 * PI, 0.0 };
		mju_copy(state_target, temp2, statenum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "cheetah") == 0) {
		modelid = 1;
		control_timestep = 0.1;
		simulation_timestep = 0.05;
		stepnum = 50;
		statenum = 18;
		actuatornum = 6;
		rolloutnum_train = 300;
		ctrl_upperlimit_train = 0.5;
		ctrl_lowerlimit_train = -0.5;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], statenum);
		mjtNum temp1[kMaxState] = { 0 };
		mju_copy(state_nominal[0], temp1, statenum);
		mjtNum temp2[kMaxState] = { 0 };
		mju_copy(state_target, temp2, statenum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "swimmer6") == 0) {
		modelid = 2;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 1000;
		statenum = 16;
		actuatornum = 5;
		rolloutnum_train = 50;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], statenum);
		mjtNum temp1[kMaxState] = { 0 };
		mju_copy(state_nominal[0], temp1, statenum);
		mjtNum temp2[kMaxState] = { 0 };
		mju_copy(state_target, temp2, statenum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "acrobot") == 0) {
		modelid = 3;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 800;
		statenum = 4;
		actuatornum = 1;
		rolloutnum_train = 2000;
		mjtNum temp[kMaxState][kMaxState] = { -428.9630, -108.7071, -158.2817, -45.5430 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], statenum);
		mjtNum temp1[kMaxState] = { PI, 0.0, 0, 0 };
		mju_copy(state_nominal[0], temp1, statenum);
		mjtNum temp2[kMaxState] = { 0 };
		mju_copy(state_target, temp2, statenum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "dbar") == 0) {
		modelid = 4;
		control_timestep = 0.02;
		simulation_timestep = 0.02;
		stepnum = 50;
		statenum = 4;
		actuatornum = 4;
		rolloutnum_train = 10;
		ctrl_upperlimit_train = 100;
		ctrl_lowerlimit_train = 0;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], statenum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "finger") == 0) {
		modelid = 5;
		control_timestep = 0.04;
		simulation_timestep = 0.04;
		stepnum = 200;
		statenum = 10;
		actuatornum = 10;
		rolloutnum_train = 300;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], statenum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "arm") == 0) {
		modelid = 6;
		control_timestep = 0.02;
		simulation_timestep = 0.02;
		stepnum = 300;
		statenum = 18;
		actuatornum = 38;
		rolloutnum_train = 20;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], statenum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "swimmer6t") == 0) {
		modelid = 7;
		control_timestep = 0.006;
		simulation_timestep = 0.006;
		stepnum = 1500;
		statenum = 16;
		actuatornum = 22;
		rolloutnum_train = 50;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], statenum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "t1d1") == 0) {
		modelid = 8;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 300;
		statenum = 12;
		actuatornum = 10;
		rolloutnum_train = 20;
		ctrl_upperlimit_train = 100;
		ctrl_lowerlimit_train = 0;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], statenum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	return 0;
}

mjtNum stepCost(mjModel* m, mjData* d, int step_index)
{
	mjtNum state[kMaxState], res0[kMaxState] = { 0 }, res1[kMaxState] = { 0 }, cost;

	mju_copy(state, d->qpos, int(statenum / 2));
	mju_copy(&state[int(statenum / 2)], d->qvel, int(statenum / 2));
	if (modelid == 0) {
		if (state[0] < PI) state_target[0] = 0; else state_target[0] = 2 * PI;
		mju_sub(res0, state, state_target, statenum);
		if (step_index >= stepnum) {
			mju_mulMatVec(res1, *QTm, res0, kMaxState, kMaxState);
			mju_zero(d->ctrl, actuatornum);
		}
		else mju_mulMatVec(res1, *Qm, res0, kMaxState, kMaxState);
		cost = (mju_dot(res0, res1, statenum) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 3) {
		if (state[0] < PI) state_target[0] = 0; else state_target[0] = 2 * PI;
		mju_sub(res0, state, state_target, statenum);
		if (step_index >= stepnum) {
			mju_mulMatVec(res1, *QTm, res0, kMaxState, kMaxState);
			mju_zero(d->ctrl, actuatornum);
		}
		else mju_mulMatVec(res1, *Qm, res0, kMaxState, kMaxState);
		cost = (mju_dot(res0, res1, statenum) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 2) {
		if (step_index >= stepnum) cost = (QT * (1 * (d->qpos[0] - 0.6) * (d->qpos[0] - 0.6) + (d->qpos[1] + 0.6) * (d->qpos[1] + 0.6) + 3 * d->qvel[0] * d->qvel[0] + 3 * d->qvel[1] * d->qvel[1]));
		else cost = (Q * ((1.5 * (d->qpos[0] - 0.6) * (d->qpos[0] - 0.6) + 1.5*(d->qpos[1] + 0.6) * (d->qpos[1] + 0.6))) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 1) {
		res0[0] = d->qvel[0] - 3;
		if (res0[0] > 0) res0[0] = 0;
		if (step_index >= stepnum) cost = (QT * res0[0] * res0[0]);
		else cost = (Q * res0[0] * res0[0] + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 4) {
		if (step_index >= stepnum) cost = (QT * (4 * (d->site_xpos[19]) * (d->site_xpos[19]) + (d->site_xpos[20] - 2.5) * (d->site_xpos[20] - 2.5) + 0.5*mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * ((4 * (d->site_xpos[19]) * (d->site_xpos[19]) + 1 * (d->site_xpos[20] - 2.5) * (d->site_xpos[20] - 2.5)) + 0.01*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 5) {
		if (step_index >= stepnum) cost = (QT * (2 * (d->site_xpos[30] - d->site_xpos[0]) * (d->site_xpos[30] - d->site_xpos[0]) + 5 * (d->site_xpos[32] - d->site_xpos[2]) * (d->site_xpos[32] - d->site_xpos[2]) + 2*mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * ((2 * (d->site_xpos[30] - d->site_xpos[0]) * (d->site_xpos[30] - d->site_xpos[0]) + 4 * (d->site_xpos[32] - d->site_xpos[2]) * (d->site_xpos[32] - d->site_xpos[2])) + 0.1*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 6) {
		if (step_index >= stepnum) cost = (QT * (1 * (d->site_xpos[93] - d->site_xpos[0]) * (d->site_xpos[93] - d->site_xpos[0]) + 5 * (d->site_xpos[95] - d->site_xpos[2]) * (d->site_xpos[95] - d->site_xpos[2]) + 0.1 * mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * ((1 * (d->site_xpos[93] - d->site_xpos[0]) * (d->site_xpos[93] - d->site_xpos[0]) + 5 * (d->site_xpos[95] - d->site_xpos[2]) * (d->site_xpos[95] - d->site_xpos[2])) + 0.00*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
		//mju_sub(res0, state, state_target, int(statenum / 2));
		//if (step_index >= stepnum) {
		//	mju_mulMatVec(res1, *QTm, res0, kMaxState, kMaxState);
		//	mju_zero(d->ctrl, actuatornum);
		//}
		//else mju_mulMatVec(res1, *Qm, res0, kMaxState, kMaxState);
		//cost = (mju_dot(res0, res1, statenum) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));

		//// point track
		//if (step_index >= stepnum) cost = (QT * (0.8 * ((d->site_xpos[27] - 2.4) * (d->site_xpos[27] - 2.4)+ (d->site_xpos[36] - 3.58) * (d->site_xpos[36] - 3.58)+ (d->site_xpos[45] - 4.74) * (d->site_xpos[45] - 4.74)+ (d->site_xpos[54] - 5.86) * (d->site_xpos[54] - 5.86) + (d->site_xpos[63] - 6.95) * (d->site_xpos[63] - 6.95) + (d->site_xpos[72] - 7.99) * (d->site_xpos[72] - 7.99) + 1*(d->site_xpos[81] - 8.97) * (d->site_xpos[81] - 8.97) + 1*(d->site_xpos[90] - 9.89) * (d->site_xpos[90] - 9.89) + 1*(d->site_xpos[93] - 10.74) * (d->site_xpos[93] - 10.74)) + 4.5 * ((d->site_xpos[95] - 6.75) * (d->site_xpos[95] - 6.75) + 1*(d->site_xpos[92] - 5.9) * (d->site_xpos[92] - 5.9) + 1*(d->site_xpos[83] - 5.13) * (d->site_xpos[83] - 5.13) + 1*(d->site_xpos[74] - 4.44) * (d->site_xpos[74] - 4.44) + 1*(d->site_xpos[65] - 3.84) * (d->site_xpos[65] - 3.84) + 1.2*(d->site_xpos[56] - 3.33) * (d->site_xpos[56] - 3.33) + 1.5*(d->site_xpos[47] - 2.92) * (d->site_xpos[47] - 2.92) + 1.8*(d->site_xpos[38] - 2.61) * (d->site_xpos[38] - 2.61) + 2*(d->site_xpos[29] - 2.4) * (d->site_xpos[29] - 2.4))) + 0.8 * mju_dot(d->qvel, d->qvel, m->nv));
		//else cost = (Q * ((0.8 * ((d->site_xpos[27] - 2.4) * (d->site_xpos[27] - 2.4)+ (d->site_xpos[36] - 3.58) * (d->site_xpos[36] - 3.58)+ (d->site_xpos[45] - 4.74) * (d->site_xpos[45] - 4.74) + (d->site_xpos[54] - 5.86) * (d->site_xpos[54] - 5.86) + (d->site_xpos[63] - 6.95) * (d->site_xpos[63] - 6.95) + (d->site_xpos[72] - 7.99) * (d->site_xpos[72] - 7.99) + 1*(d->site_xpos[81] - 8.97) * (d->site_xpos[81] - 8.97) + 1*(d->site_xpos[90] - 9.89) * (d->site_xpos[90] - 9.89) + 1*(d->site_xpos[93] - 10.74) * (d->site_xpos[93] - 10.74)) + 4.5 * ((d->site_xpos[95] - 6.75) * (d->site_xpos[95] - 6.75) + 1*(d->site_xpos[92] - 5.9) * (d->site_xpos[92] - 5.9)+1*(d->site_xpos[83] - 5.13) * (d->site_xpos[83] - 5.13) + 1*(d->site_xpos[74] - 4.44) * (d->site_xpos[74] - 4.44) + 1*(d->site_xpos[65] - 3.84) * (d->site_xpos[65] - 3.84) + 1.2*(d->site_xpos[56] - 3.33) * (d->site_xpos[56] - 3.33) + 1.5*(d->site_xpos[47] - 2.92) * (d->site_xpos[47] - 2.92) + 1.8*(d->site_xpos[38] - 2.61) * (d->site_xpos[38] - 2.61) + 2*(d->site_xpos[29] - 2.4) * (d->site_xpos[29] - 2.4))) + 0.1*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
		
		//// height track
		//if (step_index >= stepnum) cost = (QT * (0.0 * ((d->site_xpos[27] - 2.4) * (d->site_xpos[27] - 2.4) + (d->site_xpos[36] - 3.58) * (d->site_xpos[36] - 3.58) + (d->site_xpos[45] - 4.74) * (d->site_xpos[45] - 4.74) + (d->site_xpos[54] - 5.86) * (d->site_xpos[54] - 5.86) + (d->site_xpos[63] - 6.95) * (d->site_xpos[63] - 6.95) + (d->site_xpos[72] - 7.99) * (d->site_xpos[72] - 7.99) + 1 * (d->site_xpos[81] - 8.97) * (d->site_xpos[81] - 8.97) + 1 * (d->site_xpos[90] - 9.89) * (d->site_xpos[90] - 9.89) + 1 * (d->site_xpos[93] - 10.74) * (d->site_xpos[93] - 10.74))+ .0 * (d->site_xpos[93] - 12) * (d->site_xpos[93] - 12) + 4. * (0.01*(d->site_xpos[95] - 6.75) * (d->site_xpos[95] - 6.75) + .04 * (d->site_xpos[92] - 5.9) * (d->site_xpos[92] - 5.9) + .1 * (d->site_xpos[83] - 5.13) * (d->site_xpos[83] - 5.13) + .5 * (d->site_xpos[74] - 4.44) * (d->site_xpos[74] - 4.44) + .8 * (d->site_xpos[65] - 3.84) * (d->site_xpos[65] - 3.84) + 1.2*(d->site_xpos[56] - 3.33) * (d->site_xpos[56] - 3.33) + 4*(d->site_xpos[47] - 2.92) * (d->site_xpos[47] - 2.92) + 8*(d->site_xpos[38] - 2.61) * (d->site_xpos[38] - 2.61) + 12 * (d->site_xpos[29] - 2.4) * (d->site_xpos[29] - 2.4))) + 0.2 * mju_dot(d->qvel, d->qvel, m->nv));
		//else cost = (Q * ((0.0 * ((d->site_xpos[27] - 2.4) * (d->site_xpos[27] - 2.4) + (d->site_xpos[36] - 3.58) * (d->site_xpos[36] - 3.58) + (d->site_xpos[45] - 4.74) * (d->site_xpos[45] - 4.74) + (d->site_xpos[54] - 5.86) * (d->site_xpos[54] - 5.86) + (d->site_xpos[63] - 6.95) * (d->site_xpos[63] - 6.95) + (d->site_xpos[72] - 7.99) * (d->site_xpos[72] - 7.99) + 1 * (d->site_xpos[81] - 8.97) * (d->site_xpos[81] - 8.97) + 1 * (d->site_xpos[90] - 9.89) * (d->site_xpos[90] - 9.89) + 1 * (d->site_xpos[93] - 10.74) * (d->site_xpos[93] - 10.74))+ .0 * (d->site_xpos[93] - 12) * (d->site_xpos[93] - 12) + 4. * (0.01*(d->site_xpos[95] - 6.75) * (d->site_xpos[95] - 6.75) + .04 * (d->site_xpos[92] - 5.9) * (d->site_xpos[92] - 5.9) + .1 * (d->site_xpos[83] - 5.13) * (d->site_xpos[83] - 5.13) + .5 * (d->site_xpos[74] - 4.44) * (d->site_xpos[74] - 4.44) + .8 * (d->site_xpos[65] - 3.84) * (d->site_xpos[65] - 3.84) + 1.2*(d->site_xpos[56] - 3.33) * (d->site_xpos[56] - 3.33) + 4*(d->site_xpos[47] - 2.92) * (d->site_xpos[47] - 2.92) + 8*(d->site_xpos[38] - 2.61) * (d->site_xpos[38] - 2.61) + 12 * (d->site_xpos[29] - 2.4) * (d->site_xpos[29] - 2.4))) + 0.2*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 7) {
		if (step_index >= stepnum) cost = (QT * (1 * (d->qpos[0] - d->site_xpos[0]) * (d->qpos[0] - d->site_xpos[0]) + 1 * (d->qpos[1] - d->site_xpos[1]) * (d->qpos[1] - d->site_xpos[1]) + 0.0 * mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * ((1 * (d->qpos[0] - d->site_xpos[0]) * (d->qpos[0] - d->site_xpos[0]) + 1 * (d->qpos[1] - d->site_xpos[1]) * (d->qpos[1] - d->site_xpos[1])) + 0.00*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 8) {
		if (step_index >= stepnum) cost = (QT * (1 * (d->site_xpos[39] - d->site_xpos[9]) * (d->site_xpos[39] - d->site_xpos[9]) + 1 * (d->site_xpos[41] - d->site_xpos[11]) * (d->site_xpos[41] - d->site_xpos[11]) + 1.5 * mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * ((1 * (d->site_xpos[39] - d->site_xpos[9]) * (d->site_xpos[39] - d->site_xpos[9]) + 1 * (d->site_xpos[41] - d->site_xpos[11]) * (d->site_xpos[41] - d->site_xpos[11])) + 0.00*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	return cost;
}

void stateNominal(mjModel* m, mjData* d)
{
	mju_copy(d->qpos, state_nominal[0], int(statenum / 2));
	mju_copy(d->qvel, &state_nominal[0][int(statenum / 2)], int(statenum / 2));
	mj_forward(m, d);
	for (int step_index = 0; step_index < stepnum; step_index++) {
		mju_copy(d->ctrl, &ctrl_nominal[step_index * actuatornum], m->nu);
		for (int i = 0; i < integration_per_step; i++) mj_step(m, d);
		mju_copy(state_nominal[step_index + 1], d->qpos, int(statenum / 2));
		mju_copy(&state_nominal[step_index + 1][int(statenum / 2)], d->qvel, int(statenum / 2));
	}
	mj_resetData(m, d);
}

/**
* @brief  Read parameters from a file stream
* @note   none
* @param  FILE *_filename: selected file name; 
*		  mjtNum ns: number of states in the cost function;
*         mjtNum *Q: running cost matrix;
*         mjtNum *QT: terminal cost matrix;
*         mjtNum *R: control cost coeficient;
*         mjtNum *ptb_coef: perturb coeficient;
*         mjtNum *step_coef_init: initial value of step coeficient;
* @retval none
* @author rwang0417@tamu.edu
*/
void get_para(const char *_filename, mjtNum ns, mjtNum *Q, mjtNum *QT, mjtNum *R, mjtNum *ptb_coef, mjtNum *step_coef_init)
{
	FILE *fop;

	if ((fop = fopen(_filename, "r")) != NULL) {
		while (!feof(fop))
		{
			fr_matrix(fop, "Q_diag:", Q, ns);
			fr_matrix(fop, "QT_diag:", QT, ns, ns, "full");
			fr_array(fop,"R:", R);
			fr_array(fop, "ptb_coef:", ptb_coef);
			fr_array(fop, "step_coef_init:", step_coef_init);
		}
		fclose(fop); 
	}
}

/**
* @brief  Read an matrix from a file stream
* @note   none
* @param  FILE *fstream: selected file stream or const char *_filename: selected file name;
*         const char *_name: array name for match check
*         mjtNum *prt: array pointer
*         mjtNum r: matrix row number
*         mjtNum c = 0: matrix column number
*         const char *_mode = "diag": matrix mode(
*         "full": read the matrix element by element
*         "diag": only read the diagonal elements
*         )
* @retval none
* @author rwang0417@tamu.edu
*/
void fr_matrix(FILE *fstream, const char *_name, mjtNum *prt, mjtNum r, mjtNum c, const char *_mode)
{
	char para_buff[50];
	char full[5] = "full", diag[5] = "diag";

	if (c == 0) c = r;
	fscanf(fstream, "%s", para_buff);
	if (strcmp(para_buff, _name) == 0)
	{
		if (strcmp(diag, _mode) == 0)
		{
			c = r;
			for (int i = 0; i < r; i++)
			{
				fscanf(fstream, "%s", para_buff);
				*(prt + (int)i + (int)r * i) = atof(para_buff);
			}
		}
		else if (strcmp(full, _mode) == 0){
			for (int i = 0; i < r; i++)
			{
				for (int j = 0; j < c; j++)
				{
					fscanf(fstream, "%s", para_buff);
					*(prt + (int)j + (int)r * i) = atof(para_buff);
				}
			}
		}
	}
}

void fr_matrix(const char *_filename, const char *_name, mjtNum *prt, mjtNum r, mjtNum c, const char *_mode)
{
	char para_buff[50];
	char full[5] = "full", diag[5] = "diag";
	FILE *fop;

	if ((fop = fopen(_filename, "r")) != NULL) {
		if (c == 0) c = r;
		fscanf(fop, "%s", para_buff);
		if (strcmp(para_buff, _name) == 0)
		{
			if (strcmp(diag, _mode) == 0)
			{
				c = r;
				for (int i = 0; i < r; i++)
				{
					fscanf(fop, "%s", para_buff);
					*(prt + (int)i + (int)r * i) = atof(para_buff);
				}
			}
			else if (strcmp(full, _mode) == 0) {
				for (int i = 0; i < r; i++)
				{
					for (int j = 0; j < c; j++)
					{
						fscanf(fop, "%s", para_buff);
						*(prt + (int)j + (int)r * i) = atof(para_buff);
					}
				}
			}
		}
	}
}

/**
* @brief  Read an array from a file stream
* @note   none
* @param  FILE *fstream: selected file stream or const char *_filename: selected file name;
*         mjtNum *prt: array pointer
*         const char *_name: array name for match check
*         mjtNum len = 0: array length
* @retval none
* @author rwang0417@tamu.edu
*/
void fr_array(FILE *fstream, const char *_name, mjtNum *prt, mjtNum len)
{
	char para_buff[50];

	fscanf(fstream, "%s", para_buff);
	if (strcmp(para_buff, _name) == 0)
	{
		for (int i = 0; i < len; i++)
		{
			fscanf(fstream, "%s", para_buff);
			*(prt + i) = atof(para_buff);
		}
	}
}

void fr_array(const char *_filename, const char *_name, mjtNum *prt, mjtNum len)
{
	char para_buff[50];
	FILE *fop;

	if ((fop = fopen(_filename, "r")) != NULL) {
		fscanf(fop, "%s", para_buff);
		if (strcmp(para_buff, _name) == 0)
		{
			for (int i = 0; i < len; i++)
			{
				fscanf(fop, "%s", para_buff);
				*(prt + i) = atof(para_buff);
			}
		}
	}
}

/**
* @brief  Save a matrix to a file stream
* @note   none
* @param  FILE *fstream: selected file stream or const char *_filename: selected file name;
*         mjtNum *prt: pointer of the first element
*         mjtNum r: matrix row number
*         mjtNum c = 0: matrix column number
*         const char *_name = "Matrix1: ": matrix name
* @retval none
* @author rwang0417@tamu.edu
*/
void fw_matrix(FILE *fstream, mjtNum *prt, mjtNum r, mjtNum c, const char *_name)
{
	char str[50];

	if (c == 0) c = r;

	fputs(_name, fstream);
	fputs("\n", fstream);
	for (int rr = 0; rr < r; rr++)
	{
		for (int cc = 0; cc < c; cc++)
		{
			sprintf(str, "%2.4f", *(prt+(int)c*rr+cc));
			fwrite(str, 6, 1, fstream);
			fputs(" ", fstream);
		}
		fputs("\n", fstream);
	}
}

void fw_matrix(const char *_filename, mjtNum *prt, mjtNum r, mjtNum c, const char *_name, const char *_mode)
{
	char str[50];
	FILE *fop;

	if ((fop = fopen(_filename, _mode)) != NULL)
	{
		if (c == 0) c = r;

		fputs(_name, fop);
		fputs("\n", fop);
		for (int rr = 0; rr < r; rr++)
		{
			for (int cc = 0; cc < c; cc++)
			{
				sprintf(str, "%2.4f", *(prt + (int)c*rr + cc));
				fwrite(str, 6, 1, fop);
				fputs(" ", fop);
			}
			fputs("\n", fop);
		}
	}
}

/**
* @brief  Save an array to a file stream
* @note   none
* @param  FILE *fstream: selected file stream or const char *_filename: selected file name;
*         mjtNum *prt: array pointer
*         mjtNum len = 1: array length;
*         const char *_mode: file operation mode(
*         r: read; w: write; +: build file if doesn't exist
*         a: add content starts from the end of file; t: .txt file; b: binary file
*         ) default = "wt+"
*         const char *_name = "Array1: ": array name
* @retval none
* @author rwang0417@tamu.edu
*/
void fw_array(FILE *fstream, mjtNum *prt, mjtNum len, const char *_name)
{
	char str[50];

	fputs(_name, fstream);
	for (int h = 0; h < len; h++)
	{
		sprintf(str, "%2.4f", prt[h]);
		fwrite(str, 6, 1, fstream);
		fputs(" ", fstream);
	}
	fputs("\n", fstream);
}

void fw_array(const char *_filename, mjtNum *prt, mjtNum len, const char *_name, const char *_mode)
{
	char str[50];
	FILE *fop;

	if ((fop = fopen(_filename, _mode)) != NULL)
	{
		fputs(_name, fop);
		for (int h = 0; h < len; h++)
		{
			sprintf(str, "%2.4f", prt[h]);
			fwrite(str, 6, 1, fop);
			fputs(" ", fop);
		}
		fputs("\n", fop);
	}
}

/**
* @brief  Save the result of simulation to selected file
* @note   none
* @param  const char *_filename: selected file name;
*         mjtNum *u: control result array;
*         mjtNum *u_init: initial control array;
*         mjtNum len: control array length;
*         mjtNum *Q: running cost matrix;
*         mjtNum *QT: terminal cost matrix;
*         mjtNum *R: control cost coeficient;
*         mjtNum *ptb_coef: perturb coeficient;
*         mjtNum *step_coef: final value of step coeficient;
*		  mjtNum ns: number of states in the cost function;
*         const char *_mode: file operation mode(
*         r: read; w: write; +: build file if doesn't exist
*         a: add content starts from the end of file; t: .txt file; b: binary file
*         )
* @retval none
* @author rwang0417@tamu.edu
*/
void save_result(const char *_filename, mjtNum *u, mjtNum *u_init, mjtNum len, mjtNum *Q, mjtNum *QT, mjtNum *R, mjtNum *ptb_coef, mjtNum *step_coef, mjtNum ns, const char *_mode)
{
	FILE *fop;

	if ((fop = fopen(_filename, _mode)) != NULL)
	{
		fw_array(fop, u, len, "Control: ");
		fw_array(fop, u_init, len, "Control Init: ");
		fw_matrix(fop, Q, ns, ns, "Q: ");
		fw_matrix(fop, QT, ns, ns, "QT: ");
		fw_array(fop, R, 1, "R: ");
		fw_array(fop, ptb_coef, 1, "ptb_coef: ");
		fw_array(fop, step_coef, 1, "step_coef: ");

		fclose(fop);
	}
}

/**
* @brief  Calculate the determinant of the input matrix
* @note   none
* @param  mjtNum *fh: input matrix; mjtNum r: rank of the matrix
* @retval mjtNum det: the determinant value of the input matrix
* @author rwang0417@tamu.edu
*/
mjtNum determinant(mjtNum *fh, mjtNum r)
{
	mjtNum k, det = 1.0;
	mjtNum *a = new mjtNum[r*r];

	/* make sure the input matrix won't be changed */
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < r; j++)
		{
			*(a + j + i * (int)r) = *(fh + j + i * (int)r);
		}
	}

	for (int z = 0; z < r - 1; z++)
		for (int i = z; i < r - 1; i++)
		{
			if (*(a + z + z * (int)r) == 0)
			{
				for (int j = 0; j < r; j++)
				{
					*(a + j + z * (int)r) = *(a + j + z * (int)r) + *(a + j + (i + 1) * (int)r);
				}
			}
			if (*(a + z + z * (int)r) != 0) {
				k = -(*(a + z + (i+1) * (int)r)) / (*(a + z + z * (int)r));
				for (int j = z; j < r; j++) *(a + j + (i + 1) * (int)r) = k * (*(a + j + z * (int)r)) + *(a + j + (i + 1) * (int)r);
			}
		}
	for (int z = 0; z < r; z++)
	{
		det = det * (*(a + z + z * (int)r));
	}
	return (det);
}

/**
* @brief  Transpose the input matrix
* @note   none
* @param  mjtNum *res: transposed matrix; mjtNum *mat: input matrix
*         mjtNum r: input matrix row number; mjtNum c: input matrix column number
* @retval none
* @author rwang0417@tamu.edu
*/
void transpose(mjtNum *res, mjtNum *mat, mjtNum rmat, mjtNum cmat)
{
	if (cmat == 0) cmat = rmat;

	for (int i = 0; i < cmat; i++)
	{
		for (int j = 0; j < rmat; j++)
		{
			*(res+j+(int)rmat*i) = *(mat+i+(int)cmat*j);
		}
	}
}

/**
* @brief  Find the inverse of the input matrix
* @note   none
* @param  mjtNum *res: inversed matrix; mjtNum arg[N][N]: input matrix; mjtNum r: rank of the input matrix
* @retval none
* @author rwang0417@tamu.edu
*/
void cofactor(mjtNum *res, mjtNum *arg, mjtNum r)
{
	mjtNum *b = new mjtNum[(r - 1)*(r - 1)];
	mjtNum *fac = new mjtNum[r*r];
	mjtNum *fdv = new mjtNum[r*r];
	mjtNum d;
	int  m, n;

	for (int q = 0; q < r; q++)
	{
		printf("%d ", q);
		for (int p = 0; p < r; p++)
		{
			m = 0;
			n = 0;
			for (int i = 0; i < r; i++)
			{
				for (int j = 0; j < r; j++)
				{
					if (i != q && j != p)
					{
						*(b + n + m * (int)(r - 1)) = *(arg + j + i * (int)r);
						if (n < (r - 2)) n++;
						else {
							n = 0;
							m++;
						}
					}
				}
			}
			*(fac + p + q * (int)r) = pow(mjtNum(-1), q + p) * determinant(b, r - 1);
		}
	}
	transpose(fdv, fac, r);
	d = determinant(arg, r);

	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < r; j++)
		{
			*(res + j + i * (int)r) = *(fdv + j + i * (int)r) / d;
		}
	}
}
