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

/* Extern variables ---------------------------------------------------------*/
// constants
const int kMaxStep = 3000; // max step number for one rollout
const int kMaxState = 60; // max (state dimension, actuator number)

// model parameters and environment settings
int integration_per_step = 1;
int stepnum;
int actuatornum;
int quatnum;
int dof;
int modelid;
int rolloutnum_train;
mjtNum control_timestep;
mjtNum simulation_timestep;
mjtNum ctrl_upperlimit = 100;
mjtNum ctrl_lowerlimit = -100;
mjtNum state_nominal[kMaxStep][kMaxState] = { 0 };
mjtNum ctrl_nominal[kMaxStep * kMaxState] = { 0 };
mjtNum state_target[kMaxState] = { 0 };
mjtNum stabilizer_feedback_gain[kMaxState][kMaxState] = { 0 };

// hyperparameters 
mjtNum Q, QT, R;
mjtNum Qm[kMaxState][kMaxState], QTm[kMaxState][kMaxState];

/* General function prototypes-----------------------------------------------*/
void modelInit(mjModel* m, mjData* d, mjtNum* state_init, int dof, int quatnum)
{
	mj_resetData(m, d);
	mju_copy(d->qpos, state_init, dof + quatnum);
	mju_copy(d->qvel, &state_init[dof + quatnum], dof);
	mj_forward(m, d);
}

mjtNum angleModify(int model, mjtNum angle)
{
	if (model == 0)
		return (PI - fabs(angle - PI))*((PI - angle > 0) - (PI - angle < 0));
	return 0;
}

void ctrlLimit(mjtNum* ctrl, int num)
{
	for (int i = 0; i < num; i++) {
		if (ctrl[i] > ctrl_upperlimit) ctrl[i] = ctrl_upperlimit;
		else if (ctrl[i] < ctrl_lowerlimit) ctrl[i] = ctrl_lowerlimit;
	}
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

// model-depedent settings
int modelSelection(const char* model)
{
	if (_strcmpi(model, "pendulum") == 0) {
		modelid = 0;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 200;
		dof = 1;
		quatnum = 0;
		actuatornum = 1;
		rolloutnum_train = 40;
		ctrl_upperlimit = 100;
		ctrl_lowerlimit = -100;
		mjtNum temp[kMaxState][kMaxState] = { 11.0396, 3.0658 };// { 5.4995, 1.2227 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2*dof+quatnum);
		mjtNum temp1[kMaxState] = { PI, 0.0 };
		mju_copy(state_nominal[0], temp1, 2*dof+quatnum);
		mjtNum temp2[kMaxState] = { 2 * PI, 0.0 };
		mju_copy(state_target, temp2, 2*dof+quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "cheetah") == 0) {
		modelid = 1;
		control_timestep = 0.002;
		simulation_timestep = 0.002;
		stepnum = 1500;
		dof = 9;
		quatnum = 0;
		actuatornum = 6;
		rolloutnum_train = 200;
		ctrl_upperlimit = 100;
		ctrl_lowerlimit = -100;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2*dof+quatnum);
		mjtNum temp1[kMaxState] = { 0 };
		mju_copy(state_nominal[0], temp1, 2*dof+quatnum);
		mjtNum temp2[kMaxState] = { 0 };
		mju_copy(state_target, temp2, 2*dof+quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "swimmer6") == 0) {
		modelid = 2;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 1000;
		dof = 8;
		quatnum = 0;
		actuatornum = 5;
		rolloutnum_train = 50;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2*dof+quatnum);
		mjtNum temp1[kMaxState] = { 0 };
		mju_copy(state_nominal[0], temp1, 2*dof+quatnum);
		mjtNum temp2[kMaxState] = { 0 };
		mju_copy(state_target, temp2, 2*dof+quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "acrobot") == 0) {
		modelid = 3;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 800;
		dof = 2;
		quatnum = 0;
		actuatornum = 1;
		rolloutnum_train = 2000;
		mjtNum temp[kMaxState][kMaxState] = { -428.9630, -108.7071, -158.2817, -45.5430 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2*dof+quatnum);
		mjtNum temp1[kMaxState] = { PI, 0.0, 0, 0 };
		mju_copy(state_nominal[0], temp1, 2*dof+quatnum);
		mjtNum temp2[kMaxState] = { 0 };
		mju_copy(state_target, temp2, 2*dof+quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "dbar") == 0) {
		modelid = 4;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 200;
		dof = 2;
		quatnum = 0;
		actuatornum = 4;
		rolloutnum_train = 10;
		ctrl_upperlimit = 0;
		ctrl_lowerlimit = -100;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2*dof+quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "finger") == 0) {
		modelid = 5;
		control_timestep = 0.04;
		simulation_timestep = 0.04;
		stepnum = 200;
		dof = 5;
		quatnum = 0;
		actuatornum = 10;
		rolloutnum_train = 300;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2*dof+quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "arm") == 0) {
		modelid = 6;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 400;
		dof = 9;
		quatnum = 0;
		actuatornum = 38;
		rolloutnum_train = 20;
		ctrl_upperlimit = 0;
		ctrl_lowerlimit = -100;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2*dof+quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "swimmer6t") == 0) {
		modelid = 7;
		control_timestep = 0.006;
		simulation_timestep = 0.006;
		stepnum = 1500;
		dof = 8;
		quatnum = 0;
		actuatornum = 22;
		rolloutnum_train = 30;
		ctrl_upperlimit = 100;
		ctrl_lowerlimit = -100;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2*dof+quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "t1d1") == 0) {
		modelid = 8;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 300;
		dof = 6;
		quatnum = 0;
		actuatornum = 10;
		rolloutnum_train = 20;
		ctrl_upperlimit = 0;
		ctrl_lowerlimit = -100;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2*dof+quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "t2d1") == 0) {
		modelid = 9;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 400;
		dof = 14;
		quatnum = 0;		
		actuatornum = 22;
		rolloutnum_train = 30;//300
		ctrl_upperlimit = 0;
		ctrl_lowerlimit = -1000;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2*dof+quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "fish") == 0) {
		modelid = 10;
		control_timestep = 0.004;
		simulation_timestep = 0.004;
		stepnum = 2000;
		dof = 13;
		quatnum = 1;
		actuatornum = 6;
		rolloutnum_train = 30;//300
		ctrl_upperlimit = 300;
		ctrl_lowerlimit = -300;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2*dof+quatnum);
		mjtNum temp1[kMaxState] = { 0.0, 0.0, 0, 0, 1, 0, 0 };
		mju_copy(state_nominal[0], temp1, 2 * dof + quatnum); 
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "dbar3d") == 0) {
		modelid = 11;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 400;
		dof = 9;
		quatnum = 3;
		actuatornum = 7;
		rolloutnum_train = 60;
		ctrl_upperlimit = 0;
		ctrl_lowerlimit = -1000;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2 * dof + quatnum);
		mjtNum temp1[kMaxState] = { 1.0, 0.0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 };
		mju_copy(state_nominal[0], temp1, 2 * dof + quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "pendulum3d") == 0) {
		modelid = 12;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 200;
		dof = 3;
		quatnum = 1;
		actuatornum = 3;
		rolloutnum_train = 20;
		ctrl_upperlimit = 0;
		ctrl_lowerlimit = -1000;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2 * dof + quatnum);
		mjtNum temp1[kMaxState] = { 1.0, 0.0, 0, 0 };
		mju_copy(state_nominal[0], temp1, 2 * dof + quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "swimmer3") == 0) {
		modelid = 13;
		control_timestep = 0.001;
		simulation_timestep = 0.001;
		stepnum = 100;
		dof = 5;
		quatnum = 0;
		actuatornum = 2;
		rolloutnum_train = 50;
		ctrl_upperlimit = 100;
		ctrl_lowerlimit = -100;
		mjtNum temp1[kMaxState] = { 0 };
		mju_copy(state_nominal[0], temp1, 2 * dof + quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	else if (_strcmpi(model, "t1d1_3d") == 0) {
		modelid = 14;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 300; // 300
		dof = 16;
		quatnum = 0;
		actuatornum = 20;
		rolloutnum_train = 50;
		ctrl_upperlimit = 1000;
		ctrl_lowerlimit = -1000;
		mjtNum temp[kMaxState][kMaxState] = { 0 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2 * dof + quatnum);
		mjtNum temp1[kMaxState] = { 0 };
		mju_copy(state_nominal[0], temp1, 2 * dof + quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	if (_strcmpi(model, "cartpole") == 0) {
		modelid = 15;
		control_timestep = 0.01;
		simulation_timestep = 0.01;
		stepnum = 300;
		dof = 2;
		quatnum = 0;
		actuatornum = 1;
		rolloutnum_train = 40;
		ctrl_upperlimit = 100;
		ctrl_lowerlimit = -100;
		mjtNum temp[kMaxState][kMaxState] = { -2.984962660388648, -41.858411893836880, -4.889496008438448, -10.918155495786175 };
		for (int i = 0; i < actuatornum; i++) mju_copy(stabilizer_feedback_gain[i], temp[i], 2 * dof + quatnum);
		mjtNum temp1[kMaxState] = { 0 };
		mju_copy(state_nominal[0], temp1, 2 * dof + quatnum);
		mjtNum temp2[kMaxState] = { 0, -PI, 0, 0.0, };
		mju_copy(state_target, temp2, 2 * dof + quatnum);
		integration_per_step = (int)(control_timestep / simulation_timestep);
		return 1;
	}
	return 0;
}

// return the cost value at the given step
mjtNum stepCost(mjModel* m, mjData* d, int step_index)
{
	mjtNum state[kMaxState], res0[kMaxState] = { 0 }, res1[kMaxState] = { 0 }, cost;

	mju_copy(state, d->qpos, dof + quatnum);
	mju_copy(&state[dof + quatnum], d->qvel, dof);
	
	if (modelid == 0) {
		if (state[0] < PI) state_target[0] = 0; else state_target[0] = 2 * PI;
		mju_sub(res0, state, state_target, 2*dof + quatnum);
		if (step_index >= stepnum) {
			mju_mulMatVec(res1, *QTm, res0, kMaxState, kMaxState);
			mju_zero(d->ctrl, actuatornum);
		}
		else mju_mulMatVec(res1, *Qm, res0, kMaxState, kMaxState);
		cost = (mju_dot(res0, res1, 2* dof + quatnum) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 3) {
		if (state[0] < PI) state_target[0] = 0; else state_target[0] = 2 * PI;
		mju_sub(res0, state, state_target, 2* dof + quatnum);
		if (step_index >= stepnum) {
			mju_mulMatVec(res1, *QTm, res0, kMaxState, kMaxState);
			mju_zero(d->ctrl, actuatornum);
		}
		else mju_mulMatVec(res1, *Qm, res0, kMaxState, kMaxState);
		cost = (mju_dot(res0, res1, 2* dof + quatnum) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
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
		if (step_index >= stepnum) cost = (QT * (1 * (d->site_xpos[6] - d->site_xpos[18]) * (d->site_xpos[6] - d->site_xpos[18]) + 2*(d->site_xpos[8] - d->site_xpos[20]) * (d->site_xpos[8] - d->site_xpos[20]) + 1*mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * ((1 * (d->site_xpos[6] - d->site_xpos[18]) * (d->site_xpos[6] - d->site_xpos[18]) + 2 * (d->site_xpos[8] - d->site_xpos[20]) * (d->site_xpos[8] - d->site_xpos[20])) + 0.8*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 5) {
		if (step_index >= stepnum) cost = (QT * (2 * (d->site_xpos[30] - d->site_xpos[0]) * (d->site_xpos[30] - d->site_xpos[0]) + 5 * (d->site_xpos[32] - d->site_xpos[2]) * (d->site_xpos[32] - d->site_xpos[2]) + 2*mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * ((2 * (d->site_xpos[30] - d->site_xpos[0]) * (d->site_xpos[30] - d->site_xpos[0]) + 4 * (d->site_xpos[32] - d->site_xpos[2]) * (d->site_xpos[32] - d->site_xpos[2])) + 0.1*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 6) {
		if (step_index >= stepnum) cost = (QT * (1 * (d->site_xpos[93] - d->site_xpos[0]) * (d->site_xpos[93] - d->site_xpos[0]) + 5 * (d->site_xpos[95] - d->site_xpos[2]) * (d->site_xpos[95] - d->site_xpos[2]) + 1.2 * mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * ((1 * (d->site_xpos[93] - d->site_xpos[0]) * (d->site_xpos[93] - d->site_xpos[0]) + 5 * (d->site_xpos[95] - d->site_xpos[2]) * (d->site_xpos[95] - d->site_xpos[2])) + 1.2*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
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
		//if (step_index >= stepnum) cost = (QT * (1 * (d->site_xpos[15] - d->site_xpos[33]) * (d->site_xpos[15] - d->site_xpos[33]) + 1 * (d->site_xpos[17] - d->site_xpos[35]) * (d->site_xpos[17] - d->site_xpos[35]) + .5* mju_dot(d->qvel, d->qvel, m->nv)));
		//else cost = (Q * ((1 * (d->site_xpos[15] - d->site_xpos[33]) * (d->site_xpos[15] - d->site_xpos[33]) + 1 * (d->site_xpos[17] - d->site_xpos[35]) * (d->site_xpos[17] - d->site_xpos[35])) + 0.4*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
		if (step_index >= stepnum) cost = (QT * (1.2 * (d->site_xpos[15] - d->site_xpos[33]) * (d->site_xpos[15] - d->site_xpos[33]) + 1 * (d->site_xpos[17] - d->site_xpos[35]) * (d->site_xpos[17] - d->site_xpos[35]) + 0.8* mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * ((1.2 * (d->site_xpos[15] - d->site_xpos[33]) * (d->site_xpos[15] - d->site_xpos[33]) + 1 * (d->site_xpos[17] - d->site_xpos[35]) * (d->site_xpos[17] - d->site_xpos[35])) + 0.5*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}					 
	else if (modelid == 9) {
		// original
		//if (step_index >= stepnum) cost = (QT * (1 * (d->site_xpos[33] - d->site_xpos[63]) * (d->site_xpos[33] - d->site_xpos[63]) + 1.8 * (d->site_xpos[35] - d->site_xpos[65]) * (d->site_xpos[35] - d->site_xpos[65]) + 1 * mju_dot(d->qvel, d->qvel, m->nv)));
		//else cost = (Q * ((1 * (d->site_xpos[33] - d->site_xpos[63]) * (d->site_xpos[33] - d->site_xpos[63]) + 1.8 * (d->site_xpos[35] - d->site_xpos[65]) * (d->site_xpos[35] - d->site_xpos[65])) + 0.8*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
		// big vel cost
		//if (step_index >= stepnum) cost = (QT * (1 * (d->site_xpos[33] - d->site_xpos[63]) * (d->site_xpos[33] - d->site_xpos[63]) + 1.5 * (d->site_xpos[35] - d->site_xpos[65]) * (d->site_xpos[35] - d->site_xpos[65]) + .8 * mju_dot(d->qvel, d->qvel, m->nv)));
		//else cost = (Q * ((1 * (d->site_xpos[33] - d->site_xpos[63]) * (d->site_xpos[33] - d->site_xpos[63]) + 1.5 * (d->site_xpos[35] - d->site_xpos[65]) * (d->site_xpos[35] - d->site_xpos[65])) + 0.00*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
		// small vel cost
		if (step_index >= stepnum) cost = (QT * (1 * (d->site_xpos[33] - d->site_xpos[63]) * (d->site_xpos[33] - d->site_xpos[63]) + 1.5 * (d->site_xpos[35] - d->site_xpos[65]) * (d->site_xpos[35] - d->site_xpos[65]) + .01 * mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * ((1 * (d->site_xpos[33] - d->site_xpos[63]) * (d->site_xpos[33] - d->site_xpos[63]) + 1.5 * (d->site_xpos[35] - d->site_xpos[65]) * (d->site_xpos[35] - d->site_xpos[65])) + 0.00*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 10) {
		if (step_index >= stepnum) cost = (QT * (120 * (2 * (d->geom_xpos[11] - d->geom_xpos[5]) * (d->geom_xpos[11] - d->geom_xpos[5]) + (d->geom_xpos[10] - d->geom_xpos[4]) * (d->geom_xpos[10] - d->geom_xpos[4]) + (d->geom_xpos[9] - d->geom_xpos[3]) * (d->geom_xpos[9] - d->geom_xpos[3])) + (d->xmat[17] - 1) * (d->xmat[17] - 1)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
		else cost = (Q * (120 * (2 * (d->geom_xpos[11] - d->geom_xpos[5]) * (d->geom_xpos[11] - d->geom_xpos[5]) + (d->geom_xpos[10] - d->geom_xpos[4]) * (d->geom_xpos[10] - d->geom_xpos[4]) + 1.2*(d->geom_xpos[9] - d->geom_xpos[3]) * (d->geom_xpos[9] - d->geom_xpos[3])) + (d->xmat[17] - 1) * (d->xmat[17] - 1)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 11) {
		if (step_index >= stepnum) cost = (QT * (1 * (d->site_xpos[6] - d->site_xpos[30]) * (d->site_xpos[6] - d->site_xpos[30]) + 1. * (d->site_xpos[7] - d->site_xpos[31]) * (d->site_xpos[7] - d->site_xpos[31]) + 1.5 * (d->site_xpos[8] - d->site_xpos[32]) * (d->site_xpos[8] - d->site_xpos[32]) + .05 * mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * (1 * (d->site_xpos[6] - d->site_xpos[30]) * (d->site_xpos[6] - d->site_xpos[30]) + 1. * (d->site_xpos[7] - d->site_xpos[31]) * (d->site_xpos[7] - d->site_xpos[31]) + 1.5 * (d->site_xpos[8] - d->site_xpos[32]) * (d->site_xpos[8] - d->site_xpos[32]) + 0.01*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 12) {
		if (step_index >= stepnum) cost = (QT * (1 * (d->site_xpos[3] - d->site_xpos[15]) * (d->site_xpos[3] - d->site_xpos[15]) + 1. * (d->site_xpos[4] - d->site_xpos[16]) * (d->site_xpos[4] - d->site_xpos[16]) + 1.5 * (d->site_xpos[5] - d->site_xpos[17]) * (d->site_xpos[5] - d->site_xpos[17]) + 1 * mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * (1 * (d->site_xpos[3] - d->site_xpos[15]) * (d->site_xpos[3] - d->site_xpos[15]) + 1. * (d->site_xpos[4] - d->site_xpos[16]) * (d->site_xpos[4] - d->site_xpos[16]) + 1.5 * (d->site_xpos[5] - d->site_xpos[17]) * (d->site_xpos[5] - d->site_xpos[17]) + 0.6*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 13) {
		if (step_index >= stepnum) cost = (QT * (1 * (d->qpos[0] - 0.6) * (d->qpos[0] - 0.6) + (d->qpos[1] + 0.6) * (d->qpos[1] + 0.6) + 3 * d->qvel[0] * d->qvel[0] + 3 * d->qvel[1] * d->qvel[1]) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
		else cost = (Q * ((1 * (d->qpos[0] - 0.6) * (d->qpos[0] - 0.6) + 1*(d->qpos[1] + 0.6) * (d->qpos[1] + 0.6))) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 14) {
		if (step_index >= stepnum) cost = (QT * (1 * (d->site_xpos[15] - d->site_xpos[45]) * (d->site_xpos[15] - d->site_xpos[45]) + 1. * (d->site_xpos[16] - d->site_xpos[46]) * (d->site_xpos[16] - d->site_xpos[46]) + 1.5 * (d->site_xpos[17] - d->site_xpos[47]) * (d->site_xpos[17] - d->site_xpos[47]) + 1 * mju_dot(d->qvel, d->qvel, m->nv)));
		else cost = (Q * (1 * (d->site_xpos[15] - d->site_xpos[45]) * (d->site_xpos[15] - d->site_xpos[45]) + 1. * (d->site_xpos[16] - d->site_xpos[46]) * (d->site_xpos[16] - d->site_xpos[46]) + 1.5 * (d->site_xpos[17] - d->site_xpos[47]) * (d->site_xpos[17] - d->site_xpos[47]) + 0.6*mju_dot(d->qvel, d->qvel, m->nv)) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	else if (modelid == 15) {
		if (state[1] < 0) state_target[1] = -PI; else state_target[1] = PI;
		mju_sub(res0, state, state_target, 2 * dof + quatnum);
		if (step_index >= stepnum) {
			mju_mulMatVec(res1, *QTm, res0, kMaxState, kMaxState);
			mju_zero(d->ctrl, actuatornum);
		}
		else mju_mulMatVec(res1, *Qm, res0, kMaxState, kMaxState);
		cost = (mju_dot(res0, res1, 2 * dof + quatnum) + R * mju_dot(d->ctrl, d->ctrl, actuatornum));
	}
	return cost;
}

// simulate and record the nominal trajectory
void stateNominal(mjModel* m, mjData* d)
{
	modelInit(m, d, state_nominal[0], dof, quatnum);
	for (int step_index = 0; step_index < stepnum; step_index++) {
		mju_copy(d->ctrl, &ctrl_nominal[step_index * actuatornum], m->nu);
		for (int i = 0; i < integration_per_step; i++) mj_step(m, d);
		mju_copy(state_nominal[step_index + 1], d->qpos, dof + quatnum);
		mju_copy(&state_nominal[step_index + 1][dof + quatnum], d->qvel, dof);
		if (modelid == 11) {
			mju_copy(state_nominal[step_index + 1], d->qpos, 4);
			mju_copy(&state_nominal[step_index + 1][4], &d->qpos[8], 4);
			mju_copy(&state_nominal[step_index + 1][8], &d->qpos[16], 4);
			mju_copy(&state_nominal[step_index + 1][dof + quatnum], d->qvel, 3);
			mju_copy(&state_nominal[step_index + 1][dof + quatnum + 3], &d->qvel[6], 3);
			mju_copy(&state_nominal[step_index + 1][dof + quatnum + 6], &d->qvel[12], 3);
		}
		else if (modelid == 14) {
			state_nominal[step_index + 1][13] = d->qpos[18];
			state_nominal[step_index + 1][14] = d->qpos[21];
			state_nominal[step_index + 1][29] = d->qvel[18];
			state_nominal[step_index + 1][30] = d->qvel[21];
		}
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
