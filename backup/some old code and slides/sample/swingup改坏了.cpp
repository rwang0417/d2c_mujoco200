//---------------------------------//
//  This file is part of MuJoCo    //
//  Written by Emo Todorov         //
//  Copyright (C) 2017 Roboti LLC  //
//---------------------------------//


#include "mujoco.h"
#include "mjxmacro.h"
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <chrono>
#include <math.h> 
#include <time.h>

using namespace std;

//-------------------------------- macro variables --------------------------------------

// model selection: PENDULUM CARTPOLE CART2POLE CART3POLE CART7POLE ACROBOT FISH SWIMMER6 SWIMMER3 CHEETAH
#define PENDULUM
#define CTRL_LIMITTED false
#define LOCAL true
#define PI 3.141592653
#define NRUN 1

//-------------------------------- global variables -------------------------------------
// user customized parameters
#if defined(CHEETAH)
const mjtNum t_step = 0.005;
const mjtNum cal_step = 0.005;
const int step_max = 1000;
const int roll_max = 800;
const int ctrl_num = 6;
const int NS = 18;
mjtNum state_init[NS] = { 0, 0 };
mjtNum x_goal[NS] = { 0 };
#elif defined(SWIMMER3)
const mjtNum t_step = 0.005;
const mjtNum cal_step = 0.005;
const int step_max = 1600;
const int roll_max = 800;
const int ctrl_num = 2;
const int NS = 10;
mjtNum state_init[NS] = { 0, 0};
mjtNum x_goal[NS] = { 0 };
#elif defined(SWIMMER6)
const mjtNum t_step = 0.006;//0.006
const mjtNum cal_step = 0.006;
const int step_max = 1500;//1500
const int roll_max = 600;
const int ctrl_num = 5;
const int NS = 16;
mjtNum state_init[NS] = { 0.0 };
mjtNum x_goal[NS] = { 0 };
#elif defined(FISH)
const mjtNum t_step = 0.002;
const mjtNum cal_step = 0.002;
const int step_max = 3000;
const int roll_max = 600;
const int ctrl_num = 6;
const int NS = 27;
mjtNum umax = 1;
mjtNum state_init[NS] = { 0.0, 0.0, 0, 0, 0, 0, 1, 0, 0 };
mjtNum x_goal[NS] = { 0 };
#elif defined(CART2POLE)
const mjtNum t_step = 0.05;
const int step_max = 120;
const int roll_max = 600;
const int ctrl_num = 1;
const int NS = 6;
mjtNum state_init[NS] = { 0.0, 0, PI};
// top is 0
mjtNum x_goal[NS] = {
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
};
#elif defined(ACROBOT)
const mjtNum t_step = 0.01;
const int step_max = 800;
const int roll_max = 1500;
const int ctrl_num = 1;
const int NS = 4;
mjtNum state_init[NS] = { PI, 0.0, 0, 0 };
// top is 0
mjtNum x_goal[NS] = {
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
};
#elif defined(PENDULUM)
const mjtNum t_step = 0.1;
const mjtNum cal_step = 0.1;
const int step_max = 30;
const int roll_max = 300; 
const int ctrl_num = 1;
const int NS = 2;
mjtNum umax = 1;;
mjtNum state_init[NS] = { PI, 0.0 };
// top is 0
mjtNum x_goal[NS] = {
	(mjtNum)(0),
	(mjtNum)(0),
};
#elif defined(CARTPOLE)
const mjtNum t_step = 0.1;
const mjtNum cal_step = 0.1;
const int step_max = 30;
const int roll_max = 240;
const int ctrl_num = 1;
const int NS = 4;
mjtNum state_init[NS] = { 0, 0, 0, 0.0 };
mjtNum x_goal[NS] = {
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(-PI),
	(mjtNum)(0),
};
#endif

// model
mjModel* m = 0;
mjData* d = 0;
char lastfile[1000] = "";
char error[1000];
static int trial = 0;
static int tri_num = 0;
mjtNum J = 0;
mjtNum u[step_max * ctrl_num] = { 0 };
mjtNum gradient[step_max * ctrl_num] = { 0 };
mjtNum delta_u[step_max*ctrl_num] = { 0 };
mjtNum delta_j[roll_max] = { 0 };
mjtNum u_init[step_max * ctrl_num];
int stepite = (int)(t_step / cal_step);
char para_buff[20];
char ctrl_buff[30];
char glstr[10];
char *str3, *str;
FILE *fp, *fp1, *fp2;
FILE *fop;
FILE *fop1, *fop2;
char mfilename[30];
char kfilename[30];
char dfilename[30];
#if LOCAL == false
char mfilepre[17] = "../../../model/";
char kfilepre[17] = "../../../doc/";
char dfilepre[17] = "../../../data/";
#else 
char mfilepre[15] = "";
char kfilepre[15] = "";
char dfilepre[15] = "";
#endif

/* Parameters read from file fop */
mjtNum Q;
mjtNum QT;
mjtNum ptb_coef;
mjtNum step_coef;
mjtNum step_coef_init;
mjtNum R;
mjtNum Qm[NS][NS], QTm[NS][NS];

// timer
double gettm(void)
{
    static chrono::system_clock::time_point _start = chrono::system_clock::now();
    chrono::duration<double> elapsed = chrono::system_clock::now() - _start;
    return elapsed.count();
}

// help
const char helpstring[] = 
    "\n Usage:  test modelfile option [duration]\n"
    "   option can contain: 'x'- xml save/load, 's'- speed\n"
    "   if 's' is included, the simulation runs for the specified duration in seconds\n\n"
    " Example:  test model.xml xs 10\n";

// deallocate and print message
int finish(const char* msg = 0, mjModel* m = 0, mjData* d = 0)
{
    // deallocated everything
    if( d )
        mj_deleteData(d);
    if( m )
        mj_deleteModel(m);
    mj_deactivate();

    // print message
    if( msg )
        printf("%s\n", msg);

    return 0;
}

double gaussrand()
{
	static double U, V;
	static int phase = 0;
	double Z;

	if (phase == 0)
	{
		U = rand() / (RAND_MAX + 1.0);
		V = rand() / (RAND_MAX + 1.0);
		if (U == 0) U = 0.0001;
		Z = sqrt(-2.0 * log(U)) * sin(2.0 * PI * V);
	}
	else
	{
		Z = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);
	}
	phase = 1 - phase;
	return Z;
}

mjtNum cost_step(mjModel* m, mjData* d, int index)
{
	mjtNum state[NS], res0[NS], res1[NS], cost;

	for (int c = 0; c < m->nq; c++)
	{
		state[2 * c] = d->qpos[c];
	}
	for (int c = 0; c < m->nv; c++)
	{
		state[2 * c + 1] = d->qvel[c];
	}
#if defined(CARTPOLE)
	if (state[2] < 0) x_goal[2] = -PI; else x_goal[2] = PI;
	mju_sub(res0, state, x_goal, NS);
	if (index >= step_max) mju_mulMatVec(res1, *QTm, res0, NS, NS);
	else mju_mulMatVec(res1, *Qm, res0, NS, NS);
	cost = (mju_dot(res0, res1, NS) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num));
#endif
#if defined(CART2POLE)
	if (state[2] < PI) x_goal[2] = 0; else x_goal[2] = 2 * PI;
	mju_sub(res0, state, x_goal, NS);
	if (index >= step_max) mju_mulMatVec(res1, *QTm, res0, NS, NS);
	else mju_mulMatVec(res1, *Qm, res0, NS, NS);
	cost = (mju_dot(res0, res1, NS) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num));
#endif

#if defined(PENDULUM)
	if (state[0] < PI) x_goal[0] = 0; else x_goal[0] = 2 * PI;
	mju_sub(res0, state, x_goal, NS);
	if (index >= step_max) mju_mulMatVec(res1, *QTm, res0, NS, NS);
	else mju_mulMatVec(res1, *Qm, res0, NS, NS);
	cost = (mju_dot(res0, res1, NS) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num));
#endif
#if defined(ACROBOT)
	if (state[0] < PI) x_goal[0] = 0; else x_goal[0] = 2 * PI;
	mju_sub(res0, state, x_goal, NS);
	if (index >= step_max) mju_mulMatVec(res1, *QTm, res0, NS, NS);
	else mju_mulMatVec(res1, *Qm, res0, NS, NS);
	cost = (mju_dot(res0, res1, NS) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num));
#endif

#if defined(SWIMMER3)
	if (index >= step_max) cost = (QT * (1 * (d->qpos[0] - 0.6) * (d->qpos[0] - 0.6) + (d->qpos[1] - 0.7) * (d->qpos[1] - 0.7) + 3 * d->qvel[0] * d->qvel[0] + 3 * d->qvel[1] * d->qvel[1]) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num));
	else cost = (Q * ((1.5 * (d->qpos[0] - 0.6) * (d->qpos[0] - 0.6) + 1.5*(d->qpos[1] - 0.7) * (d->qpos[1] - 0.7)) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num)));
#endif
#if defined(SWIMMER6)
	if (index >= step_max) cost = (QT * (1 * (d->qpos[0] - 0.6) * (d->qpos[0] - 0.6) + (d->qpos[1] + 0.6) * (d->qpos[1] + 0.6) + 5 * d->qvel[0] * d->qvel[0] + 5 * d->qvel[1] * d->qvel[1]) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num));
	else cost = (Q * ((1 * (d->qpos[0] - 0.6) * (d->qpos[0] - 0.6) + (d->qpos[1] + 0.6) * (d->qpos[1] + 0.6)) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num)));
#endif
#if defined(FISH)
	if (index >= step_max) cost = (QT * (6 * (7 * (d->geom_xpos[11] - d->geom_xpos[5]) * (d->geom_xpos[11] - d->geom_xpos[5]) + 6 * (d->geom_xpos[10] - d->geom_xpos[4]) * (d->geom_xpos[10] - d->geom_xpos[4]) + 3 * (d->geom_xpos[9] - d->geom_xpos[3]) * (d->geom_xpos[9] - d->geom_xpos[3])) + 0.8*(d->xmat[17] - 1) * (d->xmat[17] - 1) + .1*d->qvel[1] * d->qvel[1]) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num));
	else cost = (Q * (2 * (6 * (d->geom_xpos[11] - d->geom_xpos[5]) * (d->geom_xpos[11] - d->geom_xpos[5]) + 7 * (d->geom_xpos[10] - d->geom_xpos[4]) * (d->geom_xpos[10] - d->geom_xpos[4]) + 3 * (d->geom_xpos[9] - d->geom_xpos[3]) * (d->geom_xpos[9] - d->geom_xpos[3])) + 0.8*(d->xmat[17] - 1) * (d->xmat[17] - 1)) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num));
#endif
#if defined(CHEETAH)
	if (index >= step_max) cost = (QT*(d->qvel[0] - 3)*(d->qvel[0] - 3) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num));
	else cost = (Q*(d->qvel[0] - 3) * (d->qvel[0] - 3) + R * mju_dot(d->ctrl, d->ctrl, ctrl_num));
#endif

	return cost;
}

mjtNum sim_rollout(mjModel* m, mjData* d, mjtNum* x0, mjtNum* un)
{
	int index = 0, i, y;
	mjtNum cost = 0;

	mju_zero(d->qpos, m->nq);
	mju_zero(d->qvel, m->nv);
	mju_zero(d->ctrl, m->nu);
	for (y = 0; y < m->nq; y++)
	{
		d->qpos[y] = x0[2 * y];
	}
	for (y = 0; y < m->nv; y++)
	{
		d->qvel[y] = x0[2 * y + 1];
	}
	while (index < step_max)
	{
		cost += cost_step(m, d, index);
		for (y = 0; y < ctrl_num; y++)
		{
			d->ctrl[y] = un[index * ctrl_num + y];
		}

		for (i = 0; i < stepite; i++) mj_step(m, d);

		index++;
	}
	cost += cost_step(m, d, step_max);

	return cost;
}

void setcontrol(mjModel* m, mjData* d)
{
	static int index = 0;
	static mjtNum sum = 0, t_init = -0.2, cost = 0;
	int c;

	if (d->time - t_init < t_step - 0.00001)
	{
		mj_step(m, d);
	}
	else {
		if (t_init < -0.1)
		{
			mju_zero(d->qpos, m->nq);
			mju_zero(d->qvel, m->nv);
			mju_zero(d->ctrl, m->nu);
			for (c = 0; c < m->nq; c++)
			{
				d->qpos[c] = state_init[2 * c];
			}
			for (c = 0; c < m->nv; c++)
			{
				d->qvel[c] = state_init[2 * c + 1];
			}
			cost += cost_step(m, d, 0);
			for (c = 0; c < ctrl_num; c++)
			{
				delta_u[c] = ptb_coef * gaussrand();
				d->ctrl[c] = u[0 * ctrl_num + c] + delta_u[0 * ctrl_num + c];
			}
		}else {
			index++;
			cost += cost_step(m, d, index);
			if (index < step_max)
			{
				for (c = 0; c < ctrl_num; c++)
				{
					delta_u[index*ctrl_num + c] = ptb_coef * gaussrand();
					d->ctrl[c] = u[index * ctrl_num + c] + delta_u[index * ctrl_num + c];
				}
			}else if (index >= step_max) {
				index = 0;
				for (int i = 0; i < ctrl_num * step_max; i++)
				{
					gradient[i] = gradient[i] * (1 - 1 / (trial + 1.0)) + (cost - J) * delta_u[i] / ((trial + 1.0)*ptb_coef*ptb_coef);
				}

				str = glstr;
				sprintf(str, "%3.3f", gradient[2]);
				fwrite(str, 5, 1, fp1);
				fputs(" ", fp1);

				cost = 0;
				trial++;
				if (trial >= roll_max)
				{
					trial = 0;
					str = glstr;
					sprintf(str, "%5.0f", J);
					fwrite(str, 5, 1, fp);
					fputs(" ", fp);
					fputs("\n", fp1);

					for (c = 0; c < ctrl_num * step_max; c++) 
					{
						if (step_coef * gradient[c] > .1) u[c] -= .1;
						else if (step_coef * gradient[c] < -.1) u[c] -= -.1;
						else u[c] -= step_coef * gradient[c];

#if(CTRL_LIMITTED == true)
						if (u[c] > umax) u[c] = umax;
						else if (u[c] < -umax) u[c] = -umax;
#endif
						gradient[c] = 0;
					}
					J = sim_rollout(m, d, state_init, u);
					tri_num++;
					/*if (tri_num > 2200) ptb_coef = 0.002;
					if (tri_num > 2200) step_coef = 0.00006;*/
					/*if (tri_num > 3000) ptb_coef = 0.0005;
					if (tri_num > 3000) step_coef = 0.00003;*/
					/*if (tri_num > 150) ptb_coef = 0.00005;
					if (tri_num > 150) step_coef = 0.0001;
					if (tri_num > 400) ptb_coef = 0.00005;
					if (tri_num > 400) step_coef = 0.00007;*/
				}

				//if (trial >= roll_max)
				//{
				//	// ptb_coef = 0.13 * pow(1, tri_num);
				//	//if ((step_coef < 0.0004) && (step_coef > 0) || tri_num > 1500) step_coef = step_coef_init/20;
				//}

				// reset
				t_init = -0.2;
			}
		}
		t_init = d->time;
	}
}

// main function
int main(int argc, const char** argv)
{
    // print help if arguments are missing
    if( argc<3 )
        return finish(helpstring);

    // activate MuJoCo Pro license (this must be *your* activation key)
	strcpy(kfilename, kfilepre);
	strcat(kfilename, "mjkeybig.txt");
    mj_activate(kfilename);

    // get filename, determine file type
    std::string filename(argv[1]);
    bool binary = (filename.find(".mjb")!=std::string::npos);

    // load model
	strcpy(mfilename, mfilepre);
	strcat(mfilename, argv[1]);
    char error[1000] = "Could not load binary model";
    if( binary )
        m = mj_loadModel(mfilename, 0);
    else
        m = mj_loadXML(mfilename, 0, error, 1000);
    if( !m )
        return finish(error);

    // make data
    d = mj_makeData(m);
    if( !d )
        return finish("Could not allocate mjData", m);

    // get option
    std::string option(argv[2]);

    // speed test
    if( option.find_first_of('s')!=std::string::npos )
    {
        // require duration
        if( argc<4 )
            return finish("Duration argument is required for speed test", m, d);

        // read duration
        double duration = 0;
        if( sscanf(argv[3], "%lf", &duration)!=1 || duration<=0 )
            return finish("Invalid duration argument", m, d);

		srand((unsigned)time(NULL));
		strcpy(dfilename, dfilepre);
		strcat(dfilename, "cost.txt");
		if ((fp = fopen(dfilename, "wt+")) == NULL) {
			return 0;
		}
		strcpy(dfilename, dfilepre);
		strcat(dfilename, "converge.txt");
		if ((fp1 = fopen(dfilename, "wt+")) == NULL) {
			return 0;
		}
		strcpy(dfilename, dfilepre);
		strcat(dfilename, "parameters.txt");
		if ((fop = fopen(dfilename, "r")) != NULL) {
			while (!feof(fop))
			{
				fscanf(fop, "%s", para_buff);
				if (para_buff[1] == '_')
				{
					fscanf(fop, "%s", para_buff);
					Q = atof(para_buff);
				}
				if (para_buff[0] == 'R')
				{
					fscanf(fop, "%s", para_buff);
					R = atof(para_buff);
				}
				if (para_buff[1] == 'T')
				{
					fscanf(fop, "%s", para_buff);
					QT = atof(para_buff);
				}
				if (para_buff[0] == 'p')
				{
					fscanf(fop, "%s", para_buff);
					ptb_coef = atof(para_buff);
				}
				if (para_buff[0] == 's')
				{
					fscanf(fop, "%s", para_buff);
					step_coef_init = atof(para_buff);
					step_coef = step_coef_init;
				}
			}
			//QTm[0][0] = 200; QTm[1][1] = 100; QTm[2][2] = 500; QTm[3][3] = 100;
			for (int ns = 0; ns < NS; ns++)
			{
				QTm[ns][ns] = 1 * QT;
				Qm[ns][ns] = 1 * Q;
			}
			//QTm[0][0] = 200; QTm[1][1] = 100; QTm[2][2] = 500; QTm[3][3] = 100;
			fclose(fop);
		}
	
		strcpy(dfilename, dfilepre);
		strcat(dfilename, "result.txt");
		if ((fp2 = fopen(dfilename, "r")) != NULL)
		{
			fscanf(fp2, "%s", ctrl_buff);
			if (ctrl_buff[0] == 'C') {
				for (int i = 0; i < ctrl_num * step_max; i++)
				{
					fscanf(fp2, "%s", ctrl_buff);
					u[i] = atof(ctrl_buff);
				}
			}
			fclose(fp2);
		}

        // time simulation
        int steps = 0, contacts = 0, constraints = 0;
        double printfraction = 0.1;
        printf("\nSimulation ");
        double start = gettm();
		//for (int j = 0; j < NRUN; j++) {
		//	m = mj_loadXML(mfilename, 0, error, 1000);
		//	d = mj_makeData(m);
			for (int i = 0; i < ctrl_num * step_max; i++) u[i] = 0;
			J = sim_rollout(m, d, state_init, u);
			while (d->time < duration)
			{
				setcontrol(m, d);

				// accumulate statistics
				steps++;
				contacts += d->ncon;
				constraints += d->nefc;

				// print '.' every 10% of duration
				if (d->time >= duration * printfraction)
				{
					printf(".");
					printfraction += 0.1;
				}
			}
		//	fputs("\n", fp);
		//}
        double end = gettm();

        // print results
        printf("\n Simulation time      : %.2f s\n", end-start);
        printf(" Realtime factor      : %.2f x\n", duration/mjMAX(1E-10,(end-start)));
        printf(" Time per step        : %.3f ms\n", 1000.0*(end-start)/mjMAX(1,steps));
        printf(" Contacts per step    : %d\n", contacts/mjMAX(1,steps));
        printf(" Constraints per step : %d\n", constraints/mjMAX(1,steps));
        printf(" Degrees of freedom   : %d\n\n", m->nv);
    }

	// print control pos vel
	strcpy(dfilename, dfilepre);
	strcat(dfilename, "result.txt");
	if ((fop2 = fopen(dfilename, "wt+")) != NULL)
	{
		str3 = glstr;
		fputs("Control:\n", fop2);
		for (int h = 0; h < step_max * ctrl_num; h++)
		{
			sprintf(str3, "%4.8f", u[h]);
			fwrite(str3, 10, 1, fop2);
			fputs(" ", fop2);
		}
		fputs("\n", fop2);
		//fputs("Control Init:\n", fop2);
		//for (int d = 0; d < step_max * ctrl_num; d++)
		//{
		//	sprintf(str3, "%4.8f", u_init[d]);
		//	fwrite(str3, 10, 1, fop2);
		//	fputs(" ", fop2);
		//}
		//fputs("\n", fop2);

		fputs("Q: ", fop2);
		sprintf(str3, "%2.4f", Q);
		fwrite(str3, 6, 1, fop2);
		fputs("\n", fop2);
		fputs("QT ", fop2);
		sprintf(str3, "%2.4f", QT);
		fwrite(str3, 6, 1, fop2);
		fputs("\n", fop2);
		fputs("R: ", fop2);
		sprintf(str3, "%2.4f", R);
		fwrite(str3, 6, 1, fop2);
		fputs("\n", fop2);
		fputs("ptb_coef: ", fop2);
		sprintf(str3, "%2.4f", ptb_coef);
		fwrite(str3, 6, 1, fop2);
		fputs("\nstep_coef: ", fop2);
		sprintf(str3, "%2.4f", step_coef);
		fwrite(str3, 6, 1, fop2);
		fclose(fop2);
	}

    // finalize
    return finish(0,m,d);
}
