/*  Copyright  2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

#include "windows.h"
#include <thread>
#include <funclib.h>

//-------------------------------- user macros --------------------------------------
#define CTRL_LIMITTED true
#define PI 3.141592653
#define TRAINING_NUM 1

//-------------------------------- global variables -------------------------------------
// constants
extern const int kTestNum = 500;	// number of monte-carlo runs
extern const int kMaxStep = 1000;   // max step number for one rollout
extern const int kMaxState = 100;	// max state dimension
const int kMaxThread = 64;

// extern model specific parameters
extern int rolloutnum_train;
extern int integration_per_step;
extern int stepnum;
extern int actuatornum;
extern int statenum;
extern mjtNum control_timestep;
extern mjtNum simulation_timestep;
extern mjtNum ctrl_limit_train;
extern mjtNum state_nominal[kMaxStep][kMaxState];
extern mjtNum state_target[kMaxState];
extern char testmode[30];
extern char modelname[30];
extern char modelfilename[100];

// user data and other training settings
mjtNum ctrl_current[kMaxThread][kMaxStep * kMaxState] = { 0 };
mjtNum ctrl_init[kMaxStep * kMaxState] = { 0 };
FILE *filestream1[kMaxThread], *filestream2, *filestream3;
static int iterationnum[kMaxThread] = { 0 };
char para_buff[30], ctrl_buff[30], idstr[10];
char keyfilename[100];
char datafilename[100];
char costfilename[kMaxThread][100];
char username[30];
char keyfilepre[20] = "";

/* hyperparameters */
extern mjtNum Q, QT, R;
extern mjtNum Qm[kMaxState][kMaxState], QTm[kMaxState][kMaxState];
mjtNum perturb_coefficient_train[kMaxThread], update_coefficient[kMaxThread];
mjtNum perturb_coefficient_train_init, update_coefficient_init;

// model and per-thread data
mjModel* m = NULL;
mjData* d[kMaxThread];

// per-thread statistics
int contacts[kMaxThread];
int constraints[kMaxThread];
double simtime[kMaxThread];
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
	static mjtNum gradient[kMaxStep * kMaxState] = { 0 };
	static mjtNum rollout_cost = 0, sum_cost = 0, nominal_cost = 0;
	static int step_index = 0, rollout_index = 0;
	mjtNum delta_u[kMaxStep*kMaxState] = { 0 };
	mjtNum average_cost;
	char str1[10];

	mj_resetData(m, d);
	mju_copy(d->qpos, state_nominal[0], m->nq);
	mju_copy(d->qvel, &state_nominal[0][m->nq], m->nv);
	mj_forward(m, d); 
	while (step_index <= stepnum) {
		rollout_cost += stepCost(m, d, step_index);
		for (int i = 0; i < actuatornum; i++)
		{
			delta_u[step_index*actuatornum + i] = perturb_coefficient_train[id] * randGauss(0, 1);
			d->ctrl[i] = ctrl_current[id][step_index * actuatornum + i] + delta_u[step_index * actuatornum + i];
		}
		for (int i = 0; i < integration_per_step; i++) mj_step(m, d);
		step_index++;
	}
	
	step_index = 0;
	nominal_cost = rollout_cost / (rollout_index + 1.0) + (1 - 1 / (rollout_index + 1.0)) * nominal_cost;
	for (int i = 0; i < actuatornum * stepnum; i++)
	{
		gradient[i] = gradient[i] * (1 - 1 / (rollout_index + 1.0)) + (rollout_cost - nominal_cost) * delta_u[i] / ((rollout_index + 1.0)*perturb_coefficient_train[id]*perturb_coefficient_train[id]);
	}

	if (id == 0) {
		sprintf(str1, "%3.3f", gradient[2]);
		fwrite(str1, 5, 1, filestream2);
		fputs(" ", filestream2);
	}

	sum_cost = sum_cost + rollout_cost;
	rollout_cost = 0;
	rollout_index++;
	if (rollout_index >= rolloutnum_train)
	{
		rollout_index = 0;
		average_cost = sum_cost / (1.0 * rolloutnum_train);
		sprintf(str1, "%5.0f", average_cost);
		fwrite(str1, 5, 1, filestream1[id]);
		fputs(" ", filestream1[id]);
		if (id == 0)fputs("\n", filestream2);
		sum_cost = 0;
		nominal_cost = 0;
		for (int i = 0; i < actuatornum * stepnum; i++)
		{
			if (update_coefficient[id] * gradient[i] > 0.1) ctrl_current[id][i] -= 0.1;
			else if (update_coefficient[id] * gradient[i] < -0.1) ctrl_current[id][i] -= -0.1;
			else ctrl_current[id][i] -= update_coefficient[id] * gradient[i];
#if(CTRL_LIMITTED == true)
			if (ctrl_current[id][i] > ctrl_limit_train) ctrl_current[id][i] = ctrl_limit_train;
			else if (ctrl_current[id][i] < -ctrl_limit_train) ctrl_current[id][i] = -ctrl_limit_train;
#endif
			gradient[i] = 0;
		}
		iterationnum[id]++;
		//if (iterationnum > 150) { update_coefficient = 0.00001; perturb_coefficient_train = 0.01; }
		//if (iterationnum > 600) { update_coefficient = 0.00001; perturb_coefficient_train = 0.01; }
	}
}

// thread function
void simulate(int id, int niteration)
{
    // clear statistics
    contacts[id] = 0;
    constraints[id] = 0;
	
	srand((unsigned)time(NULL));
	sprintf(idstr, "%d", id);
	snprintf(costfilename[id], sizeof(costfilename[id]), "%s%s%s", "cost", idstr, ".txt");
	if ((filestream1[id] = fopen(costfilename[id], "wt+")) == NULL) {
		printf("Could not open file: cost.txt");
	}
	
	for (int i = 0; i < TRAINING_NUM; i++) {
		// task initialization
		for (int i = 0; i < actuatornum * stepnum; i++) ctrl_current[id][i] = ctrl_init[i];
		iterationnum[id] = 0;
		perturb_coefficient_train[id] = perturb_coefficient_train_init;
		update_coefficient[id] = update_coefficient_init;
		printfraction = 0.1;

		// run and time
		double start = gettm();
		while (iterationnum[id] < niteration)
		{
			train(d[id], id);
			
			// accumulate statistics
			contacts[id] += d[id]->ncon;
			constraints[id] += d[id]->nefc;

			// print '.' every printfraction of niteration for thread 0
			if (iterationnum[id] >= niteration * printfraction && id == 0)
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
	char str2[30];
    // print help if arguments are missing
    if( argc<3 || argc>5 )
        return finish("\n Usage:  testspeed modelfile niteration [nthread [profile]]\n");

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

    // read niteration and nthread
    int niteration = 0, nthread = 0, profile = 0;
    if( sscanf(argv[2], "%d", &niteration)!=1 || niteration<=0 )
        return finish("Invalid niteration argument");
    if( argc>3 )
        if( sscanf(argv[3], "%d", &nthread)!=1 )
            return finish("Invalid nthread argument");
    if( argc>4 )
        if( sscanf(argv[4], "%d", &profile)!=1 )
            return finish("Invalid profile argument");

    // clamp nthread to [1, kMaxThread]
    nthread = mjMAX(1, mjMIN(kMaxThread, nthread));

    // get filename, determine file type
    std::string filename(argv[1]);
    bool binary = (filename.find(".mjb")!=std::string::npos);
	modelSelection(argv[1]);

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

	strcpy(datafilename, "converge.txt");
	if ((filestream2 = fopen(datafilename, "wt+")) == NULL) {
		printf("Could not open file: converge.txt");
	}
	strcpy(datafilename, "parameters.txt");
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
				perturb_coefficient_train_init = atof(para_buff);
			}
			if (para_buff[0] == 's')
			{
				fscanf(filestream3, "%s", para_buff);
				update_coefficient_init = atof(para_buff);
			}
		}
		for (int i = 0; i < statenum; i++)
		{
			QTm[i][i] = 1 * QT;
			Qm[i][i] = 1 * Q;
		}
		//QTm[0][0] = 200; QTm[1][1] = 100; QTm[2][2] = 500; QTm[3][3] = 100;
		fclose(filestream3);
	}

	strcpy(datafilename, "init.txt");
	if ((filestream3 = fopen(datafilename, "r")) != NULL)
	{
		for (int i = 0; i < actuatornum * stepnum; i++)
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
        printf("\nRunning %d iterations per thread at dt_c = %g, dt_s = %g ...\n\n", niteration, control_timestep, m->opt.timestep);
    else
        printf("\nRunning %d iterations at dt_c = %g, dt_s = %g...\n\n", niteration, control_timestep, m->opt.timestep);

    // run simulation, record total time
    thread th[kMaxThread];
    double starttime = gettm();
	for (int id = 0; id < nthread; id++) {
		th[id] = thread(simulate, id, niteration);
	}
	for (int id = 0; id < nthread; id++)
		th[id].join();
    double tottime = gettm() - starttime;

    // all-thread summary
    if( nthread>1 )
    {
        printf("Summary for all %d threads\n\n", nthread);
        printf(" Total simulation time  : %.2f s\n", tottime);
        printf(" Total steps per second : %.0f\n", nthread*niteration*rolloutnum_train*stepnum*integration_per_step /tottime);
        printf(" Total realtime factor  : %.2f x\n", nthread*niteration*rolloutnum_train*stepnum*integration_per_step*m->opt.timestep/tottime);
        printf(" Total time per step    : %.4f ms\n\n", 1000*tottime/(nthread*niteration*rolloutnum_train*stepnum*integration_per_step));
        printf("Details for thread 0\n\n");
    }

    // details for thread 0
    printf("\n Simulation time      : %.2f s\n", simtime[0]);
	printf(" Number of steps      : %d\n", niteration*rolloutnum_train*stepnum*integration_per_step);
	printf(" Steps per second     : %.0f\n", niteration*rolloutnum_train*stepnum*integration_per_step / simtime[0]);
	printf(" Realtime factor      : %.2f x\n", niteration*rolloutnum_train*stepnum*integration_per_step*m->opt.timestep / simtime[0]);
	printf(" Time per step        : %.4f ms\n\n", 1000 * simtime[0] / (niteration*rolloutnum_train*stepnum*integration_per_step));
	printf(" Contacts per step    : %d\n", contacts[0] / (niteration*rolloutnum_train*stepnum*integration_per_step));
	printf(" Constraints per step : %d\n", constraints[0] / (niteration*rolloutnum_train*stepnum*integration_per_step));
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
		snprintf(datafilename, sizeof(datafilename), "%s%s%s", "result", idstr, ".txt");
		if ((filestream3 = fopen(datafilename, "wt+")) != NULL)
		{
			for (int h = 0; h < stepnum * actuatornum; h++)
			{
				sprintf(str2, "%4.8f", ctrl_current[id][h]);
				fwrite(str2, 10, 1, filestream3);
				fputs(" ", filestream3);
			}
			fputs("\n\n//////////// LOG ////////////\n", filestream3);
			fputs("Control Init:\n", filestream3);
			for (int d = 0; d < stepnum * actuatornum; d++)
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
			fputs("perturb_coefficient_train: ", filestream3);
			sprintf(str2, "%2.4f", perturb_coefficient_train_init);
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
