/*  Copyright  2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

#include <windows.h>
#include <thread>
#include "funclib.h"

//-------------------------------- global variables -------------------------------------
// constants
extern const int kTestNum = 100;	// number of monte-carlo runs
extern const int kMaxStep = 500;   // max step number for one rollout
extern const int kMaxState = 20;	// max state dimension
const int kMaxThread = 2;

// extern model specific parameters
extern int integration_per_step;
extern int stepnum;
extern int actuatornum;
extern int statenum;
extern int modelid;
extern mjtNum control_timestep;
extern mjtNum simulation_timestep;
extern mjtNum perturb_coefficient_sysid;
extern mjtNum state_nominal[kMaxStep][kMaxState];
extern mjtNum ctrl_nominal[kMaxStep * kMaxState];
extern char testmode[30];

// user data and other training settings
mjtNum matAB[kMaxThread][kMaxStep][kMaxState][kMaxState + kMaxState] = { 0 };
mjtNum ctrl_max = 0;
mjtNum sysiderr = 0;
FILE *filestream2, *filestream3;
static int iteration_index[kMaxThread] = { 0 };
char data_buff[30], idstr[10];
char keyfilename[100];
char datafilename[100];
char modelfilename[100];
char username[30];
char modelname[30];
char keyfilepre[20] = "";

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

void sysidCheck(mjModel* m, mjData* d)
{
	mjtNum dx_estimate[kMaxStep][kMaxState];
	mjtNum dx_input[kMaxStep][kMaxState + kMaxState] = { 0 };
	mjtNum dx_simulate[kMaxStep][kMaxState] = { 0 };

	for (int t = 0; t < kTestNum; t++)
	{
		for (int i = 0; i < statenum + actuatornum; i++)
		{
			for (int m = 0; m < stepnum; m++) dx_input[m][i] = 0.01 * ctrl_max * randGauss(0, 1);
		}
		for (int j = 0; j < stepnum; j++) mju_mulMatVec(dx_estimate[j], *matAB[0][j], dx_input[j], kMaxState, kMaxState+ kMaxState);
		for (int step_index = 0; step_index < stepnum; step_index++)
		{
			mju_add(d->qpos, dx_input[step_index], state_nominal[step_index], int(statenum/2));
			mju_add(d->qvel, &dx_input[step_index][int(statenum / 2)], &state_nominal[step_index][int(statenum / 2)], int(statenum / 2));
			mju_add(d->ctrl, &dx_input[step_index][statenum], &ctrl_nominal[step_index * actuatornum], m->nu);

			if (modelid == 4) {
				d->qpos[2] = -d->qpos[1];
				d->qpos[3] = d->qpos[1];
				d->qvel[2] = -d->qvel[1];
				d->qvel[3] = d->qvel[1];
			}

			for (int k = 0; k < integration_per_step; k++) mj_step(m, d);

			mju_sub(dx_simulate[step_index], d->qpos, state_nominal[step_index + 1], int(statenum / 2));
			mju_sub(&dx_simulate[step_index][int(statenum / 2)], d->qvel, &state_nominal[step_index + 1][int(statenum / 2)], int(statenum / 2));
		}
		for (int y = 0; y < statenum; y++)
		{
			for (int h = 0; h < stepnum; h++)
			{
				if (dx_simulate[h][y] != 0) sysiderr += fabs((dx_estimate[h][y] - dx_simulate[h][y]) / dx_simulate[h][y]);
			}
		}
	}
	sysiderr = sysiderr / (1.0*kTestNum*stepnum*statenum);
}

// thread function
void sysid(int id, int nite)
{
	mjtNum delta_x1[kMaxState + kMaxState] = { 0 };
	mjtNum delta_x2[kMaxState] = { 0 };
	char str1[30];

	// clear statistics
	contacts[id] = 0;
	constraints[id] = 0;
	printfraction = 0.1;
	srand((unsigned)time(NULL));

	// run and time
	double start = gettm();
	for(iteration_index[id] = 0; iteration_index[id] < nite; iteration_index[id]++)
	{
		for (int step_index = 0; step_index < stepnum; step_index++)
		{
			for (int y = 0; y < statenum + actuatornum; y++) delta_x1[y] = perturb_coefficient_sysid * ctrl_max * randGauss(0, 1);

			mju_add(d[id]->qpos, delta_x1, state_nominal[step_index], int(statenum / 2));
			mju_add(d[id]->qvel, &delta_x1[int(statenum / 2)], &state_nominal[step_index][int(statenum / 2)], int(statenum / 2));
			mju_add(d[id]->ctrl, &delta_x1[statenum], &ctrl_nominal[step_index * actuatornum], m->nu);

			if (modelid == 4) {
				d[id]->qpos[2] = -d[id]->qpos[1];
				d[id]->qpos[3] = d[id]->qpos[1];
				d[id]->qvel[2] = -d[id]->qvel[1];
				d[id]->qvel[3] = d[id]->qvel[1];
			}

			for (int i = 0; i < integration_per_step; i++) mj_step(m, d[id]);

			mju_sub(delta_x2, d[id]->qpos, state_nominal[step_index + 1], int(statenum / 2));
			mju_sub(&delta_x2[int(statenum / 2)], d[id]->qvel, &state_nominal[step_index + 1][int(statenum / 2)], int(statenum / 2));

			// option 1: iteration method
			for (int y = 0; y < statenum; y++)
			{
				for (int i = 0; i < statenum + actuatornum; i++)
				{
					matAB[id][step_index][y][i] = matAB[id][step_index][y][i] * (1 - 1 / (iteration_index[id] + 1.0)) + delta_x2[y] * delta_x1[i] / ((iteration_index[id] + 1.0) * perturb_coefficient_sysid * perturb_coefficient_sysid * ctrl_max * ctrl_max);
				}
			}
		}
		sprintf(str1, "%3.3f", matAB[id][0][0][1]);
		fwrite(str1, 5, 1, filestream2);
		fputs(" ", filestream2);

		// print '.' every printfraction of niteration for thread 0
		if (iteration_index[id] >= nite * printfraction && id == 0)
		{
			printf(".");
			printfraction += 0.1;
		}
		// accumulate statistics
		contacts[id] += d[id]->ncon;
		constraints[id] += d[id]->nefc;
		simtime[id] = gettm() - start;
	}
}

// main function
int main(int argc, const char** argv)
{
    // print help if arguments are missing
    if( argc<3 || argc>5 )
        return finish("\n Usage:  sysid modelfile niteration [nthread [profile]]\n");

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

	// get filename, determine file type
	std::string filename(argv[1]);
	bool binary = (filename.find(".mjb") != std::string::npos);
	strcpy(modelfilename, argv[1]);
	strncpy(modelname, modelfilename, strlen(modelfilename) - 4);
	modelSelection(modelname);

	// read niteration and nthread
	int niteration = 0, nthread = 0, profile = 0;
	if (sscanf(argv[2], "%d", &niteration) != 1 || niteration <= 0)
		return finish("Invalid niteration argument");
	if (argc > 3 && modelSelection(argv[3]) != 1) {
		if (sscanf(argv[3], "%d", &nthread) != 1)
			return finish("Invalid nthread argument");
		if (argc > 4)
			if (sscanf(argv[4], "%d", &profile) != 1)
				return finish("Invalid profile argument");
	}
	else if (argc > 4) {
		if (sscanf(argv[4], "%d", &nthread) != 1)
			return finish("Invalid nthread argument");
		if (argc > 5)
			if (sscanf(argv[5], "%d", &profile) != 1)
				return finish("Invalid profile argument");
	}

    // clamp nthread to [1, kMaxThread]
    nthread = mjMAX(1, mjMIN(kMaxThread, nthread));

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
	strcpy(datafilename, "result0.txt");
	if ((filestream3 = fopen(datafilename, "r")) != NULL) {
		for (int i = 0; i < actuatornum * stepnum; i++)
		{
			fscanf(filestream3, "%s", data_buff);
			ctrl_nominal[i] = atof(data_buff);
			if (fabs(ctrl_nominal[i]) > ctrl_max) ctrl_max = fabs(ctrl_nominal[i]);
		}
		fclose(filestream3);
	}
	else printf("Could not open file: result.txt");

    // install timer callback for profiling if requested
    tm_start = chrono::system_clock::now();
    if( profile )
        mjcb_time = gettm;

    // print start
	if (nthread > 1)
		printf("\nRunning %d iterations per thread at dt_c = %g, dt_s = %g for %d thread\n\n", int(niteration / nthread), control_timestep, m->opt.timestep, nthread);
    else
        printf("\nRunning %d iterations at dt_c = %g, dt_s = %g\n\n", niteration, control_timestep, m->opt.timestep);

    // run simulation, record total time
    thread th[kMaxThread];
    double starttime = gettm();
	stateNominal(m, d[0]);

	for (int id = 0; id < nthread; id++) {
		th[id] = thread(sysid, id, int(niteration / nthread));
	}
	for (int id = 0; id < nthread; id++)
		th[id].join();
	/*for (int idt = 1; idt < nthread; idt++)
	{
		for (int i = 0; i < stepnum; i++)
		{
			for (int j = 0; j < actuatornum; j++)
			{
				for (int k = 0; k < statenum + actuatornum; k++)
					matAB[0][i][j][k] += matAB[idt][i][j][k];
			}
		}
	}
	for (int i = 0; i < stepnum; i++)
	{
		for (int j = 0; j < actuatornum; j++)
		{
			for (int k = 0; k < statenum + actuatornum; k++)
				matAB[0][i][j][k] = matAB[0][i][j][k]/(nthread+1);
		}
	}*/
    double tottime = gettm() - starttime;
	sysidCheck(m, d[0]);
    // all-thread summary
    if( nthread>1 )
    {
        printf("Summary for all %d threads\n\n", nthread);
        printf(" Total simulation time  : %.2f s\n", tottime);
        printf(" Total steps per second : %.0f\n", nthread*niteration*stepnum*integration_per_step /tottime);
        printf(" Total realtime factor  : %.2f x\n", nthread*niteration*stepnum*integration_per_step*m->opt.timestep/tottime);
        printf(" Total time per step    : %.4f ms\n\n", 1000*tottime/(nthread*niteration*stepnum*integration_per_step));
        printf("Details for thread 0\n\n");
    }

    // details for thread 0
    printf("\n Simulation time      : %.2f s\n", simtime[0]);
	printf(" Number of steps      : %d\n", niteration*stepnum*integration_per_step);
	printf(" Steps per second     : %.0f\n", niteration*stepnum*integration_per_step / simtime[0]);
	printf(" Realtime factor      : %.2f x\n", niteration*stepnum*integration_per_step*m->opt.timestep / simtime[0]);
	printf(" Time per step        : %.4f ms\n\n", 1000 * simtime[0] / (niteration*stepnum*integration_per_step));
	printf(" Contacts per step    : %d\n", contacts[0] / (niteration*stepnum*integration_per_step));
	printf(" Constraints per step : %d\n", constraints[0] / (niteration*stepnum*integration_per_step));
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
	strcpy(datafilename, "lnr.txt");
	if ((filestream3 = fopen(datafilename, "wt+")) != NULL)
	{
		for (int t = 0; t < stepnum; t++)
		{
			for (int h = 0; h < statenum; h++)
			{
				for (int d = 0; d < statenum + actuatornum; d++)
				{
					sprintf(data_buff, "%4.8f", matAB[0][t][h][d]);
					fwrite(data_buff, 10, 1, filestream3);
					fputs(" ", filestream3);
				}
				fputs("\n", filestream3);
			}
			fputs("\n", filestream3);
		}
		fputs("sysiderr: ", filestream3);
		sprintf(data_buff, "%2.4f\n", sysiderr);
		fwrite(data_buff, 6, 1, filestream3);
		fputs("\nptb_coef: ", filestream3);
		sprintf(data_buff, "%2.4f\n", perturb_coefficient_sysid);
		fwrite(data_buff, 6, 1, filestream3);
		fclose(filestream3);
	}
	else printf("Could not open file: lnr.txt");

    // free per-thread data
    for( int id=0; id<nthread; id++ )
        mj_deleteData(d[id]);

    // finalize
	return finish(0, m);
}
