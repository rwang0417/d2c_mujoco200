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
#define CTRL_LIMITTED false
#define TRAINING_NUM 1

//-------------------------------- global variables -------------------------------------
// constants
extern const int kMaxStep = 500;   // max step number for one rollout
extern const int kMaxState = 20;	// max state dimension
const int kMaxThread = 64;
const mjtNum kMaxUpdate = 0.1;

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


// user data and other training settings
mjtNum ctrl_current[kMaxThread][kMaxStep * kMaxState] = { 0 };
mjtNum ctrl_init[kMaxStep * kMaxState] = { 0 };
FILE *filestream3;
static int iteration_index[kMaxThread] = { 0 };
char data_buff[30];
char keyfilename[100];
char datafilename[100];
char modelfilename[100];
char username[30];
char modelname[30];
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
double printfraction = 0.2;


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
void train(int id, int niteration)
{
	static char str1[30];
	char costfilename[100];
	FILE *filestream1;
	RowVectorXd nominal_cost(niteration);
	RowVectorXd rollout_cost(rolloutnum_train);
	RowVectorXd gradient(stepnum * actuatornum);
	MatrixXd delta_u(stepnum * actuatornum, rolloutnum_train);
	MatrixXd ttt(1,stepnum * actuatornum);
	srand((unsigned)time(NULL) + id);
	for (int train_index = 0; train_index < TRAINING_NUM; train_index++) {
		// task initialization
		mju_copy(ctrl_current[id], ctrl_init, actuatornum*stepnum);  
		perturb_coefficient_train[id] = perturb_coefficient_train_init;
		update_coefficient[id] = update_coefficient_init;
		printfraction = 0.2;

		// run and time
		double start = gettm();
		for (int iteration_index = 0; iteration_index < niteration; iteration_index++)
		{
			// clear statistics
			contacts[id] = 0;
			constraints[id] = 0;

			// nominal cost
			nominal_cost(iteration_index) = 0;
			mju_copy(d[id]->qpos, state_nominal[0], m->nq);
			mju_copy(d[id]->qvel, &state_nominal[0][m->nq], m->nv);
			for (int step_index = 0; step_index < stepnum; step_index++) {
				mju_copy(d[id]->ctrl, &ctrl_current[id][step_index * actuatornum], m->nu);
				nominal_cost(iteration_index) += stepCost(m, d[id], step_index);
				for (int i = 0; i < integration_per_step; i++) mj_step(m, d[id]);

				// accumulate statistics
				contacts[id] += d[id]->ncon;
				constraints[id] += d[id]->nefc;
			}
			nominal_cost(iteration_index) += stepCost(m, d[id], stepnum);

			// gradient
			for (int rollout_index = 0; rollout_index < rolloutnum_train; rollout_index++)
			{
				for (int i = 0; i < stepnum * actuatornum; i++) delta_u(i, rollout_index) = perturb_coefficient_train[id] * randGauss(0, 1);

				// plus
				mju_copy(d[id]->qpos, state_nominal[0], m->nq);
				mju_copy(d[id]->qvel, &state_nominal[0][m->nq], m->nv);
				for (int step_index = 0; step_index < stepnum; step_index++) {
					for (int i = 0; i < actuatornum; i++) d[id]->ctrl[i] = ctrl_current[id][step_index * actuatornum + i] + delta_u(step_index * actuatornum + i, rollout_index);
					rollout_cost(rollout_index) += stepCost(m, d[id], step_index);
					for (int i = 0; i < integration_per_step; i++) mj_step(m, d[id]);
				}
				rollout_cost(rollout_index) += stepCost(m, d[id], stepnum);

				// minus
				mju_copy(d[id]->qpos, state_nominal[0], m->nq);
				mju_copy(d[id]->qvel, &state_nominal[0][m->nq], m->nv);
				for (int step_index = 0; step_index < stepnum; step_index++) {
					for (int i = 0; i < actuatornum; i++) d[id]->ctrl[i] = ctrl_current[id][step_index * actuatornum + i] - delta_u(step_index * actuatornum + i, rollout_index);
					rollout_cost(rollout_index) -= stepCost(m, d[id], step_index);
					for (int i = 0; i < integration_per_step; i++) mj_step(m, d[id]);
				}
				rollout_cost(rollout_index) -= stepCost(m, d[id], stepnum);
			}
			ttt=rollout_cost * delta_u.transpose()*(delta_u*delta_u.transpose()).inverse()/2;// (rollout_cost * delta_u.transpose()*((delta_u*delta_u.transpose()).inverse())) / 2;
			
			// update
			for (int i = 0; i < actuatornum * stepnum; i++)
			{
				if (update_coefficient[id] * gradient(i) > kMaxUpdate) ctrl_current[id][i] -= kMaxUpdate;
				else if (update_coefficient[id] * gradient(i) < -kMaxUpdate) ctrl_current[id][i] -= -kMaxUpdate;
				else ctrl_current[id][i] -= update_coefficient[id] * gradient(i);
#if(CTRL_LIMITTED == true)
				if (ctrl_current[id][i] > ctrl_limit_train) ctrl_current[id][i] = ctrl_limit_train;
				else if (ctrl_current[id][i] < -ctrl_limit_train) ctrl_current[id][i] = -ctrl_limit_train;
#endif
			}
			for (int rollout_index = 0; rollout_index < rolloutnum_train; rollout_index++) rollout_cost(rollout_index) = 0;

			// print '.' every printfraction of niteration for thread 0
			if (id == 0 && iteration_index >= niteration * printfraction)
			{
				printf(".");
				printfraction += 0.2;
			}
			//if (iteration_index > 150) { update_coefficient = 0.00001; perturb_coefficient_train = 0.01; }
			//if (iteration_index > 600) { update_coefficient = 0.00001; perturb_coefficient_train = 0.01; }
		}
		simtime[id] = gettm() - start;

		// save result to file
		snprintf(costfilename, sizeof(costfilename), "%s%d%s", "cost", id, ".txt");
		if ((filestream1 = fopen(costfilename, "at+")) != NULL) {
			for (int iteration_index = 0; iteration_index < niteration; iteration_index++) {
				sprintf(str1, "%5.0f", nominal_cost(iteration_index));
				fwrite(str1, 5, 1, filestream1);
				fputs(" ", filestream1);
			}
			fputs("/n", filestream1);
			fclose(filestream1);
		}
		else printf("Could not open file: cost.txt\n");
	}
}

//// thread function
//void simulate(int id, int niteration)
//{
//	sprintf(idstr, "%d", id);
//	snprintf(costfilename[id], sizeof(costfilename[id]), "%s%s%s", "cost", idstr, ".txt");
//	if ((filestream1[id] = fopen(costfilename[id], "wt+")) == NULL) {
//		printf("Could not open file: cost.txt");
//	}
//	srand((unsigned)time(NULL));////////////////////////////////////// doesn't work with multi-thread
//	for (int i = 0; i < TRAINING_NUM; i++) {
//		// task initialization
//		mju_copy(ctrl_current[id], ctrl_init, actuatornum*stepnum);
//		iteration_index[id] = 0;
//		perturb_coefficient_train[id] = perturb_coefficient_train_init;
//		update_coefficient[id] = update_coefficient_init;
//		printfraction = 0.1;
//
//		// run and time
//		double start = gettm();
//		while (iteration_index[id] < niteration)
//		{
//			train(d[id], id);
//			
//			// accumulate statistics
//			contacts[id] += d[id]->ncon;
//			constraints[id] += d[id]->nefc;
//
//		}
//		
//		simtime[id] = gettm() - start;
//	}
//}


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

	// get filename, determine file type
	std::string filename(argv[1]);
	bool binary = (filename.find(".mjb") != std::string::npos);
	strcpy(modelfilename, argv[1]);
	strncpy(modelname, modelfilename, strlen(modelfilename) - 4);
	modelSelection(modelname);

    // read niteration and nthread
    int niteration = 0, nthread = 0, profile = 0;
    if( sscanf(argv[2], "%d", &niteration)!=1 || niteration<=0 )
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
	mju_add(state_nominal[0], state_nominal[0], d[0]->qpos, m->nq); // get initial position from key pos
	
	strcpy(datafilename, "parameters.txt");
	if ((filestream3 = fopen(datafilename, "r")) != NULL) {
		while (!feof(filestream3))
		{
			fscanf(filestream3, "%s", data_buff);
			if (data_buff[1] == '_')
			{
				fscanf(filestream3, "%s", data_buff);
				Q = atof(data_buff);
			}
			if (data_buff[0] == 'R')
			{
				fscanf(filestream3, "%s", data_buff);
				R = atof(data_buff);
			}
			if (data_buff[1] == 'T')
			{
				fscanf(filestream3, "%s", data_buff);
				QT = atof(data_buff);
			}
			if (data_buff[0] == 'p')
			{
				fscanf(filestream3, "%s", data_buff);
				perturb_coefficient_train_init = atof(data_buff);
			}
			if (data_buff[0] == 's')
			{
				fscanf(filestream3, "%s", data_buff);
				update_coefficient_init = atof(data_buff);
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
	else printf("Could not open file: parameters.txt\n");

	strcpy(datafilename, "init.txt");
	if ((filestream3 = fopen(datafilename, "r")) != NULL)
	{
		for (int i = 0; i < actuatornum * stepnum; i++)
		{
			fscanf(filestream3, "%s", data_buff);
			ctrl_init[i] = atof(data_buff);
		}
		fclose(filestream3);
	}
	else printf("Could not open file: init.txt\n");

    // install timer callback for profiling if requested
    tm_start = chrono::system_clock::now();
    if( profile )
        mjcb_time = gettm;

    // print start
    if( nthread>1 )
        printf("\nRunning %d iterations per thread at dt_c = %g, dt_s = %g, %d rollouts per iteration\n\n", niteration, control_timestep, m->opt.timestep, rolloutnum_train*2);
    else
        printf("\nRunning %d iterations at dt_c = %g, dt_s = %g, %d rollouts per iteration\n\n", niteration, control_timestep, m->opt.timestep, rolloutnum_train*2);
	
    // run simulation, record total time
    thread th[kMaxThread];
    double starttime = gettm();
	for (int id = 0; id < nthread; id++) {
		th[id] = thread(train, id, niteration);
	}
	for (int id = 0; id < nthread; id++)
		th[id].join();
    double tottime = gettm() - starttime;

    // all-thread summary
    if( nthread>1 )
    {
        printf("Summary for all %d threads\n\n", nthread);
        printf(" Total simulation time  : %.2f s\n", tottime);
        printf(" Total steps per second : %.0f\n", nthread*niteration*rolloutnum_train*2*stepnum*integration_per_step /tottime);
        printf(" Total realtime factor  : %.2f x\n", nthread*niteration*rolloutnum_train*2*stepnum*integration_per_step*m->opt.timestep/tottime);
        printf(" Total time per step    : %.4f ms\n\n", 1000*tottime/(nthread*niteration*rolloutnum_train*2*stepnum*integration_per_step));
        printf("Details for thread 0\n\n");
    }

    // details for thread 0
    printf("\n Simulation time      : %.2f s\n", simtime[0]);
	printf(" Number of steps      : %d\n", niteration*rolloutnum_train*2*stepnum*integration_per_step);
	printf(" Steps per second     : %.0f\n", niteration*rolloutnum_train*2*stepnum*integration_per_step / simtime[0]);
	printf(" Realtime factor      : %.2f x\n", niteration*rolloutnum_train*2*stepnum*integration_per_step*m->opt.timestep / simtime[0]);
	printf(" Time per step        : %.4f ms\n\n", 1000 * simtime[0] / (niteration*rolloutnum_train*2*stepnum*integration_per_step));
	printf(" Contacts per step    : %d\n", contacts[0] / (niteration*rolloutnum_train*2*stepnum*integration_per_step));
	printf(" Constraints per step : %d\n", constraints[0] / (niteration*rolloutnum_train*2*stepnum*integration_per_step));
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
		snprintf(datafilename, sizeof(datafilename), "%s%d%s", "result", id, ".txt");
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
