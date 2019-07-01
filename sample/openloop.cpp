/*  Copyright  2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

#include "windows.h"
#include "mujoco.h"
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <string>
#include <chrono>
#include <thread>

#include <math.h> 
#include <time.h>
#include <funclib.h>

using namespace std;

//-------------------------------- user macros --------------------------------------
// model selection: PENDULUM ACROBOT SWIMMER3 CHEETAH
#define CHEETAH
#define CTRL_LIMITTED true
#define LOCAL true
#define PI 3.141592653
#define TRAINING_NUM 1

//-------------------------------- global variables -------------------------------------
// model parameters
#if defined(CHEETAH)
const mjtNum kControlTimeStep = 0.1;
const mjtNum kSimulationTimeStep = 0.05;
const int kStepNum = 50;
const int kRolloutNum = 300;
const int kActuatorNum = 6;
const int kStateNum = 18;
mjtNum ctrl_limit = 0.5;
mjtNum state_init[kStateNum] = { 0, 0 };
mjtNum state_target[kStateNum] = { 0 };
#elif defined(SWIMMER3)
const mjtNum kControlTimeStep = 0.005;
const mjtNum kSimulationTimeStep = 0.005;
const int kStepNum = 1600;
const int kRolloutNum = 800;
const int kActuatorNum = 2;
const int kStateNum = 10;
mjtNum state_init[kStateNum] = { 0.0 };
mjtNum state_target[kStateNum] = { 0 };
#elif defined(ACROBOT)
const mjtNum kControlTimeStep = 0.01;
const mjtNum kSimulationTimeStep = 0.01;
const int kStepNum = 700;
const int kRolloutNum = 2000;
const int kActuatorNum = 1;
const int kStateNum = 4;
mjtNum state_init[kStateNum] = { -PI, 0.0, 0, 0 };
// top is 0
mjtNum state_target[kStateNum] = {
	(mjtNum)(0),
	(mjtNum)(-2 * PI),
	(mjtNum)(0),
	(mjtNum)(0),
};
#elif defined(PENDULUM)
const mjtNum kControlTimeStep = 0.1;
const mjtNum kSimulationTimeStep = 0.1;
const int kStepNum = 30;
const int kRolloutNum = 240;
const int kActuatorNum = 1;
const int kStateNum = 2;
mjtNum state_init[kStateNum] = { PI, 0.0 };
// top is 0
mjtNum state_target[kStateNum] = {
	(mjtNum)(0),
	(mjtNum)(0),
};
#endif

mjtNum ctrl_current[64][kStepNum * kActuatorNum] = { 0 };
mjtNum ctrl_init[kStepNum * kActuatorNum] = { 0 };
FILE *filestream1[64], *filestream2, *filestream3;
static int kIterationNum[64] = { 0 };
int kIterationPerStep = (int)(kControlTimeStep / kSimulationTimeStep);
char para_buff[30], ctrl_buff[30];
char modelfilename[100];
char keyfilename[100];
char datafilename[100];
char costfilename[64][100];
char username[30];
#if LOCAL == false
char modelfilepre[20] = "../../../model/";
char keyfilepre[20] = "../../../doc/";
char datafilepre[20] = "../../../data/";
#else 
char modelfilepre[20] = "";
char keyfilepre[20] = "";
char datafilepre[20] = "";
#endif

/* hyperparameters */
mjtNum Q, QT, R;
mjtNum perturb_coefficient[64], update_coefficient[64];
mjtNum perturb_coefficient_init, update_coefficient_init;
mjtNum Qm[kStateNum][kStateNum], QTm[kStateNum][kStateNum];

// model and per-thread data
mjModel* m = NULL;
mjData* d[64];


// per-thread statistics
int contacts[64];
int constraints[64];
double simtime[64];
double printfraction = 0.1;


// timer
chrono::system_clock::time_point tm_start;
mjtNum gettm(void)
{
    chrono::duration<double> elapsed = chrono::system_clock::now() - tm_start;
    return elapsed.count();
}


// deallocate and print message
int finish(const char* msg = NULL, mjModel* m = NULL)
{
    // deallocate model
    if( m )
        mj_deleteModel(m);
    mj_deactivate();

    // print message
    if( msg )
        printf("%s\n", msg);

    return 0;
}

void train(mjData* d, int id)
{
	static mjtNum gradient[kStepNum * kActuatorNum] = { 0 };
	static mjtNum rollout_cost = 0, sum_cost = 0, nominal_cost = 0;
	static int step_index = 0, rollout_index = 0;
	mjtNum delta_u[kStepNum*kActuatorNum] = { 0 };
	mjtNum average_cost, state[kStateNum], res0[kStateNum];
	char str1[10];

	mju_copy(d->qpos, state_init, m->nq);
	mju_copy(d->qvel, &state_init[m->nq], m->nv);
	while (step_index < kStepNum) {
		mju_copy(state, d->qpos, m->nq);
		mju_copy(&state[m->nq], d->qvel, m->nv);
#if defined(PENDULUM)
		if (state[0] < PI) state_target[0] = 0; else state_target[0] = 2 * PI;
		mju_sub(res0, state, state_target, kStateNum);
		if (step_index >= kStepNum - 1) mju_mulMatVec(res1, *QTm, res0, kStateNum, kStateNum);
		else mju_mulMatVec(res1, *Qm, res0, kStateNum, kStateNum);
		rollout_cost += (mju_dot(res0, res1, kStateNum) + R * mju_dot(d->ctrl, d->ctrl, kActuatorNum));
#endif
#if defined(ACROBOT)
		if (state[0] < PI) state_target[0] = 0; else state_target[0] = 2 * PI;
		mju_sub(res0, state, state_target, kStateNum);
		if (step_index >= kStepNum - 1) mju_mulMatVec(res1, *QTm, res0, kStateNum, kStateNum);
		else mju_mulMatVec(res1, *Qm, res0, kStateNum, kStateNum);
		rollout_cost += (mju_dot(res0, res1, kStateNum) + R * mju_dot(d->ctrl, d->ctrl, kActuatorNum));
#endif
#if defined(SWIMMER3)
		if (step_index >= kStepNum - 1) rollout_cost += (QT * (1 * (d->qpos[0] - 0.6) * (d->qpos[0] - 0.6) + (d->qpos[1] + 0.6) * (d->qpos[1] + 0.6) + 3 * d->qvel[0] * d->qvel[0] + 3 * d->qvel[1] * d->qvel[1]) + R * mju_dot(d->ctrl, d->ctrl, kActuatorNum));
		else rollout_cost += (Q * ((1. * (d->qpos[0] - 0.6) * (d->qpos[0] - 0.6) + 1.*(d->qpos[1] + 0.6) * (d->qpos[1] + 0.6)) + R * mju_dot(d->ctrl, d->ctrl, kActuatorNum)));
#endif
#if defined(CHEETAH)
		res0[0] = d->qvel[0] - 3;
		if (res0[0] > 0) res0[0] = 0;
		if (step_index >= kStepNum - 1) rollout_cost += (QT*res0[0] * res0[0] + R * mju_dot(d->ctrl, d->ctrl, kActuatorNum));
		else rollout_cost += (Q*res0[0] * res0[0] + R * mju_dot(d->ctrl, d->ctrl, kActuatorNum));
#endif
		for (int i = 0; i < kActuatorNum; i++)
		{
			delta_u[step_index*kActuatorNum + i] = perturb_coefficient[id] * rand_gauss(0, 1);
			d->ctrl[i] = ctrl_current[id][step_index * kActuatorNum + i] + delta_u[step_index * kActuatorNum + i];
		}
		for (int i = 0; i < kIterationPerStep; i++) mj_step(m, d);
		step_index++;
	}
	
	step_index = 0;
	nominal_cost = rollout_cost / (rollout_index + 1.0) + (1 - 1 / (rollout_index + 1.0)) * nominal_cost;
	for (int i = 0; i < kActuatorNum * kStepNum; i++)
	{
		gradient[i] = gradient[i] * (1 - 1 / (rollout_index + 1.0)) + (rollout_cost - nominal_cost) * delta_u[i] / ((rollout_index + 1.0)*perturb_coefficient[id]*perturb_coefficient[id]);
	}

	if (id == 0) {
		sprintf(str1, "%3.3f", gradient[2]);
		fwrite(str1, 5, 1, filestream2);
		fputs(" ", filestream2);
	}

	sum_cost = sum_cost + rollout_cost;
	rollout_cost = 0;
	rollout_index++;
	if (rollout_index >= kRolloutNum)
	{
		rollout_index = 0;
		average_cost = sum_cost / (1.0 * kRolloutNum);
		sprintf(str1, "%5.0f", average_cost);
		fwrite(str1, 5, 1, filestream1[id]);
		fputs(" ", filestream1[id]);
		if (id == 0)fputs("\n", filestream2);
		sum_cost = 0;
		nominal_cost = 0;
		for (int i = 0; i < kActuatorNum * kStepNum; i++)
		{
			if (update_coefficient[id] * gradient[i] > 0.1) ctrl_current[id][i] -= 0.1;
			else if (update_coefficient[id] * gradient[i] < -0.1) ctrl_current[id][i] -= -0.1;
			else ctrl_current[id][i] -= update_coefficient[id] * gradient[i];
#if(CTRL_LIMITTED == true)
			if (ctrl_current[id][i] > ctrl_limit) ctrl_current[id][i] = ctrl_limit;
			else if (ctrl_current[id][i] < -ctrl_limit) ctrl_current[id][i] = -ctrl_limit;
#endif
			gradient[i] = 0;
		}
		kIterationNum[id]++;
		//if (kIterationNum > 150) { update_coefficient = 0.00001; perturb_coefficient = 0.01; }
		//if (kIterationNum > 600) { update_coefficient = 0.00001; perturb_coefficient = 0.01; }
	}
}

// thread function
void simulate(int id, int nite)
{
	char idstr[10];

    // clear statistics
    contacts[id] = 0;
    constraints[id] = 0;
    
	srand((unsigned)time(NULL));
	sprintf(idstr, "%d", id);
	snprintf(costfilename[id], sizeof(costfilename[id]), "%s%s%s%s", datafilepre, "cost", idstr, ".txt");
	if ((filestream1[id] = fopen(costfilename[id], "wt+")) == NULL) {
		printf("Could not open file: cost.txt");
	}
	
	for (int nominal_cost = 0; nominal_cost < TRAINING_NUM; nominal_cost++) {
		// task initialization
		for (int i = 0; i < kActuatorNum * kStepNum; i++) ctrl_current[id][i] = ctrl_init[i];
		kIterationNum[id] = 0;
		perturb_coefficient[id] = perturb_coefficient_init;
		update_coefficient[id] = update_coefficient_init;
		printfraction = 0.1;

		// run and time
		double start = gettm();
		while (kIterationNum[id] < nite)
		{
			train(d[id], id);

			// accumulate statistics
			contacts[id] += d[id]->ncon;
			constraints[id] += d[id]->nefc;

			// print '.' every printfraction of nite for thread 0
			if (kIterationNum[id] >= nite * printfraction && id == 0)
			{
				printf(".");
				printfraction += 0.1;
			}
		}
		fclose(filestream1[id]);
		simtime[id] = gettm() - start;
	}
}


// main function
int main(int argc, const char** argv)
{
	char idstr[10], str2[30];
    // print help if arguments are missing
    if( argc<3 || argc>5 )
        return finish("\n Usage:  testspeed modelfile nite [nthread [profile]]\n");

    // activate MuJoCo Pro license (this must be *your* activation key)
	DWORD usernamesize = 30;
	GetUserName(username, &usernamesize);
	if (username[0] == 'R') {
		strcpy(keyfilename, keyfilepre);
		strcat(keyfilename, "mjkeybig.txt");
		mj_activate(keyfilename);
	}
	else if (username[0] == '5') {
		strcpy(keyfilename, keyfilepre);
		strcat(keyfilename, "mjkeysmall.txt");
		mj_activate(keyfilename);
	}

    // read nite and nthread
    int nite = 0, nthread = 0, profile = 0;
    if( sscanf(argv[2], "%d", &nite)!=1 || nite<=0 )
        return finish("Invalid nite argument");
    if( argc>3 )
        if( sscanf(argv[3], "%d", &nthread)!=1 )
            return finish("Invalid nthread argument");
    if( argc>4 )
        if( sscanf(argv[4], "%d", &profile)!=1 )
            return finish("Invalid profile argument");

    // clamp nthread to [1, 64]
    nthread = mjMAX(1, mjMIN(64, nthread));

    // get filename, determine file type
    std::string filename(argv[1]);
    bool binary = (filename.find(".mjb")!=std::string::npos);
	strcpy(modelfilename, modelfilepre);
	strcat(modelfilename, argv[1]);

    // load model
    char error[500] = "Could not load binary model";
    if( binary )
        m = mj_loadModel(modelfilename, 0);
    else
        m = mj_loadXML(modelfilename, 0, error, 500);
    if( !m )
        return finish(error);

    // make per-thread data
    int testkey = mj_name2id(m, mjOBJ_KEY, "test");
    for( int id=0; id<nthread; id++ )
    {
        d[id] = mj_makeData(m);
        if( !d[id] )
            return finish("Could not allocate mjData", m);

        // init to keyframe "test" if present
        if( testkey>=0 )
        {
            mju_copy(d[id]->qpos, m->key_qpos + testkey*m->nq, m->nq);
            mju_copy(d[id]->qvel, m->key_qvel + testkey*m->nv, m->nv);
            mju_copy(d[id]->act,  m->key_act  + testkey*m->na, m->na);
        }
    }

	strcpy(datafilename, datafilepre);
	strcat(datafilename, "converge.txt");
	if ((filestream2 = fopen(datafilename, "wt+")) == NULL) {
		printf("Could not open file: converge.txt");
	}
	strcpy(datafilename, datafilepre);
	strcat(datafilename, "parameters.txt");
	if ((filestream3 = fopen(datafilename, "r")) != NULL) {
		while (!feof(filestream3))
		{
			fscanf(filestream3, "%s", para_buff);
			if (para_buff[1] == '_')
			{
				fscanf(filestream3, "%s", para_buff);
				Q = atof(para_buff);
			}
			if (para_buff[0] == 'R')
			{
				fscanf(filestream3, "%s", para_buff);
				R = atof(para_buff);
			}
			if (para_buff[1] == 'T')
			{
				fscanf(filestream3, "%s", para_buff);
				QT = atof(para_buff);
			}
			if (para_buff[0] == 'p')
			{
				fscanf(filestream3, "%s", para_buff);
				perturb_coefficient_init = atof(para_buff);
			}
			if (para_buff[0] == 's')
			{
				fscanf(filestream3, "%s", para_buff);
				update_coefficient_init = atof(para_buff);
			}
		}
		for (int kStateNum = 0; kStateNum < kStateNum; kStateNum++)
		{
			QTm[kStateNum][kStateNum] = 1 * QT;
			Qm[kStateNum][kStateNum] = 1 * Q;
		}
		//QTm[0][0] = 200; QTm[1][1] = 100; QTm[2][2] = 500; QTm[3][3] = 100;
		fclose(filestream3);
	}

	strcpy(datafilename, datafilepre);
	strcat(datafilename, "init.txt");
	if ((filestream3 = fopen(datafilename, "r")) != NULL)
	{
		for (int i = 0; i < kActuatorNum * kStepNum; i++)
		{
			fscanf(filestream3, "%s", ctrl_buff);
			ctrl_init[i] = atof(ctrl_buff);
		}
		fclose(filestream3);
	}

    // install timer callback for profiling if requested
    tm_start = chrono::system_clock::now();
    if( profile )
        mjcb_time = gettm;

    // print start
    if( nthread>1 )
        printf("\nRunning %d iterations per thread at dt_c = %g, dt_s = %g ...\n\n", nite, kControlTimeStep, m->opt.timestep);
    else
        printf("\nRunning %d iterations at dt_c = %g, dt_s = %g...\n\n", nite, kControlTimeStep, m->opt.timestep);

    // run simulation, record total time
    thread th[64];
    double starttime = gettm();
	for (int id = 0; id < nthread; id++) {
		th[id] = thread(simulate, id, nite);
	}
    for( int id=0; id<nthread; id++ )
        th[id].join();
    double tottime = gettm() - starttime;

    // all-thread summary
    if( nthread>1 )
    {
        printf("Summary for all %d threads\n\n", nthread);
        printf(" Total simulation time  : %.2f s\n", tottime);
        printf(" Total steps per second : %.0f\n", nthread*nite*kRolloutNum*kStepNum*kIterationPerStep /tottime);
        printf(" Total realtime factor  : %.2f x\n", nthread*nite*kRolloutNum*kStepNum*kIterationPerStep*m->opt.timestep/tottime);
        printf(" Total time per step    : %.4f ms\n\n", 1000*tottime/(nthread*nite*kRolloutNum*kStepNum*kIterationPerStep));

        printf("Details for thread 0\n\n");
    }

    // details for thread 0
    printf("\n Simulation time      : %.2f s\n", simtime[0]);
	printf(" Number of steps      : %d\n", nite*kRolloutNum*kStepNum*kIterationPerStep);
	printf(" Steps per second     : %.0f\n", nite*kRolloutNum*kStepNum*kIterationPerStep / simtime[0]);
	printf(" Realtime factor      : %.2f x\n", nite*kRolloutNum*kStepNum*kIterationPerStep*m->opt.timestep / simtime[0]);
	printf(" Time per step        : %.4f ms\n\n", 1000 * simtime[0] / (nite*kRolloutNum*kStepNum*kIterationPerStep));
	printf(" Contacts per step    : %d\n", contacts[0] / (nite*kRolloutNum*kStepNum*kIterationPerStep));
	printf(" Constraints per step : %d\n", constraints[0] / (nite*kRolloutNum*kStepNum*kIterationPerStep));
    printf(" Degrees of freedom   : %d\n\n", m->nv);

    // profiler results for thread 0
    if( profile )
    {
        printf(" Profiler phase (ms per step)\n");
        mjtNum tstep = d[0]->timer[mjTIMER_STEP].duration/d[0]->timer[mjTIMER_STEP].number;
        for( int i=0; i<mjNTIMER; i++ )
            if( d[0]->timer[i].number>0 )
            {
                mjtNum istep = d[0]->timer[i].duration/d[0]->timer[i].number;
                printf(" %16s : %.5f  (%6.2f %%)\n", mjTIMERSTRING[i],
                    1000*istep, 100*istep/tstep);
            }
    }

	// print control pos vel
	for (int id = 0; id < nthread; id++)
	{
		sprintf(idstr, "%d", id);
		snprintf(datafilename, sizeof(datafilename), "%s%s%s%s", datafilepre, "result", idstr, ".txt");
		if ((filestream3 = fopen(datafilename, "wt+")) != NULL)
		{
			for (int h = 0; h < kStepNum * kActuatorNum; h++)
			{
				sprintf(str2, "%4.8f", ctrl_current[id][h]);
				fwrite(str2, 10, 1, filestream3);
				fputs(" ", filestream3);
			}
			fputs("\n\n//////////// LOG ////////////\n", filestream3);
			fputs("Control Init:\n", filestream3);
			for (int d = 0; d < kStepNum * kActuatorNum; d++)
			{
				sprintf(str2, "%4.8f", ctrl_init[d]);
				fwrite(str2, 10, 1, filestream3);
				fputs(" ", filestream3);
			}
			fputs("\n", filestream3);

			fputs("Q: ", filestream3);
			sprintf(str2, "%2.4f", Q);
			fwrite(str2, 6, 1, filestream3);
			fputs("\n", filestream3);
			fputs("QT ", filestream3);
			sprintf(str2, "%2.4f", QT);
			fwrite(str2, 6, 1, filestream3);
			fputs("\n", filestream3);
			fputs("R: ", filestream3);
			sprintf(str2, "%2.4f", R);
			fwrite(str2, 6, 1, filestream3);
			fputs("\n", filestream3);
			fputs("perturb_coefficient: ", filestream3);
			sprintf(str2, "%2.4f", perturb_coefficient_init);
			fwrite(str2, 6, 1, filestream3);
			fputs("\nstep_coef: ", filestream3);
			sprintf(str2, "%2.4f", update_coefficient_init);
			fwrite(str2, 6, 1, filestream3);
			fclose(filestream3);
		}
	}

    // free per-thread data
    for( int id=0; id<nthread; id++ )
        mj_deleteData(d[id]);

    // finalize
	return finish(0, m);
}
