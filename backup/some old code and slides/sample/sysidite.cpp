//---------------------------------//
//  This file is part of MuJoCo    //
//  Written by Emo Todorov         //
//  Copyright (C) 2017 Roboti LLC  //
//---------------------------------//


#include "mujoco.h"
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <chrono>
#include <math.h> 
#include <time.h>

using namespace std;
//-------------------------------- macro variables --------------------------------------

// model selection: PENDULUM CARTPOLE CART2POLE CART3POLE SWIMMER3 SWIMMER6
#define SWIMMER3
#define CTRL_LIMITTED false
#define STATE_LINEAR false
#define PI 3.141592653

//-------------------------------- global variables -------------------------------------

// user customized parameters
#if defined(SWIMMER6)
const mjtNum t_step = 0.02;
const mjtNum ptb_coef = 0.02;
const int step_max = 300;
const int32_t roll_max = 60000;
const int ctrl_num = 5;
const int NS = 16;
mjtNum state_nominal[step_max + 1][NS] = { 0 };
#elif defined(SWIMMER3)
const mjtNum t_step = 0.02;
const mjtNum ptb_coef = 0.1;
const int32_t step_max = 600;
const int32_t roll_max = 18000;
const int ctrl_num = 2;
const int NS = 10;
mjtNum state_nominal[step_max + 1][NS] = { 0 };
#elif defined(CART2POLE)
const mjtNum t_step = 0.05;
const int32_t roll_max = 20000; //2w
const int32_t step_max = 120;
const int ctrl_num = 1;
const int NS = 6;
mjtNum ptb_coef = 0.04;
mjtNum state_nominal[step_max + 1][NS] = { 0 };
#elif defined(CART7POLE)
const mjtNum t_step = 0.1;
const mjtNum ptb_coef = 1;
const int32_t step_max = 100;
const int32_t roll_max = 20000;
const int ctrl_num = 1;
const int NS = 16;
mjtNum state_nominal[step_max + 1][NS] = { 0 };
#elif defined(CART3POLE)
const mjtNum t_step = 0.05;
const mjtNum ptb_coef = 0;
const int32_t step_max = 140;
const int32_t roll_max = 20000;
const int ctrl_num = 1;
const int NS = 8;
mjtNum state_nominal[step_max + 1][NS] = { 0 };
#elif defined(ACROBOT)
const mjtNum t_step = 0.01;
const mjtNum ptb_coef = 0.02;
const int32_t step_max = 700;
const int32_t roll_max = 20000;
const int ctrl_num = 1;
const int NS = 4;
mjtNum state_nominal[step_max + 1][NS] = { -PI, 0, 0, 0 };
#elif defined(PENDULUM)
const mjtNum t_step = 0.1;
const mjtNum ptb_coef = 6;
const int32_t step_max = 80;
const int32_t roll_max = 20000;
const int ctrl_num = 1;
const int NS = 2;
mjtNum state_nominal[step_max + 1][NS] = { PI, 0.0 };
#elif defined(CARTPOLE)
const mjtNum t_step = 0.1;
const mjtNum ptb_coef = 2;
const int32_t step_max = 30;
const int32_t roll_max = 20000;
const int ctrl_num = 1;
const int NS = 4;
mjtNum state_nominal[step_max + 1][NS] = { 0, 0, 0, 0.0 };
#endif

// model
mjModel* m = 0;
mjData* d = 0;
mjtNum t_init = -0.2;
char lastfile[1000] = "";
char error[1000];
static int tri_num = 0;
static int32_t trial = 0;
static int index = 0;
static bool cal_flag = true;
const long N = NS + ctrl_num;
mjtNum delta_x1[NS + ctrl_num] = { 0 };
mjtNum delta_x2[NS] = { 0 };
mjtNum mat[step_max][NS][NS + ctrl_num] = { 0 };
mjtNum sum[step_max][NS][NS + ctrl_num] = { 0 };
mjtNum dx[step_max + 1][NS];
mjtNum delta_xc[step_max + 1][NS + ctrl_num] = { 0 };
mjtNum delta_xcs[step_max][NS] = { 0 };
mjtNum res[NS + ctrl_num][NS + ctrl_num], res6[NS + ctrl_num][NS + ctrl_num], res7[NS][NS + ctrl_num];
mjtNum ctrl_nominal[step_max * ctrl_num] = { 0 };
mjtNum buff = 0;
char ctrl_buff[20];
char para_buff[20];
char glstr[10];
char *str3, *str;
FILE *fp;
FILE *fop;
FILE *fop1, *fop2;

// timer
double gettm(void)
{
    static chrono::system_clock::time_point _start = chrono::system_clock::now();
    chrono::duration<double> elapsed = chrono::system_clock::now() - _start;
    return elapsed.count();
}

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

/* Calculate Determinant of the Matrix */
mjtNum determinant(mjtNum fh[N][N], mjtNum r)
{
	int z, j, i;
	mjtNum m = 1, k, det = 1.0;
	int rr = (int)r;
	mjtNum a[N][N];

	for (i = 0; i < rr; i++)
	{
		for (j = 0; j < r; j++)
		{
			a[i][j] = fh[i][j];
		}
	}

	for (z = 0; z<rr - 1; z++)
		for (i = z; i<rr - 1; i++)
		{
			if (a[z][z] == 0)
			{
				
				for (j = 0; j<rr; j++)
				{
					a[z][j] = a[z][j] + a[i + 1][j];
				}
			}
			if (a[z][z] != 0) {
				k = -a[i + 1][z] / a[z][z];
				for (j = z; j<rr; j++)a[i + 1][j] = k * a[z][j] + a[i + 1][j];
			}
		}
	
	for (z = 0; z<rr; z++)
	{
		det = det * (a[z][z]);
	}
	return (det);
}

/* Find transpose of matrix */
void transpose(mjtNum num[N][N], mjtNum fac[N][N], mjtNum r)
{
	int i, j, rr = (int)r;
	mjtNum b[N][N], d;

	for (i = 0; i < rr; i++)
	{
		for (j = 0; j < r; j++)
		{
			b[i][j] = fac[j][i];
		}
	}
	d = determinant(num, r);
	for (i = 0; i < rr; i++)
	{
		for (j = 0; j < r; j++)
		{
			res[i][j] = b[i][j] / d;
		}
	}
}

/* Inverse */
void cofactor(mjtNum num[N][N], mjtNum f)
{
	mjtNum b[N][N] = { 0 }, fac[N][N];
	int p, q, m, n, i, j;
	int ff = (int)f;

	for (q = 0; q < ff; q++)
	{
		for (p = 0; p < ff; p++)
		{
			m = 0;
			n = 0;
			for (i = 0; i < ff; i++)
			{
				for (j = 0; j < ff; j++)
				{
					if (i != q && j != p)
					{
						b[m][n] = num[i][j];
						if (n < (f - 2)) n++;
						else {
							n = 0;
							m++;
						}
					}
				}
			}
			fac[q][p] = pow(mjtNum(-1), q + p) * determinant(b, ff - 1);
		}
	}
	transpose(num, fac, ff);
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
		if (U == 0) U = 0.00001;
		Z = sqrt(-2.0 * log(U)) * sin(2.0 * PI * V);
	}
	else
	{
		Z = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);
	}
	phase = 1 - phase;
	return Z;
}

void setcontrol(mjtNum time, mjtNum* ctrl, int nu)
{
	if (cal_flag == true)
	{
		if (time - t_init < t_step - 0.0001)
		{
			mj_step(m, d);
		}
		else {
			if (t_init < -0.1)
			{
				// initialize
				mju_zero(d->qpos, m->nq);
				mju_zero(d->qvel, m->nv);
				for (int y = 0; y < m->nq; y++)
				{
					d->qpos[y] = state_nominal[0][2 * y];
				}
				for (int y = 0; y < m->nv; y++)
				{
					d->qvel[y] = state_nominal[0][2 * y + 1];
				}
				for (int ci = 0; ci < ctrl_num; ci++)
				{
					ctrl[ci] = ctrl_nominal[0 * ctrl_num + ci];
				}
			}
			else {
				index++;
				for (int y = 0; y < m->nq; y++)
				{
					state_nominal[index][2 * y] = d->qpos[y];
				}
				for (int y = 0; y < m->nv; y++)
				{
					state_nominal[index][2 * y + 1] = d->qvel[y];
				}
				if (index >= step_max) {
					index = 0;
					cal_flag = false;
					for (int y = 0; y < m->nq; y++)
					{
						delta_x1[2 * y] = ptb_coef * gaussrand();
						d->qpos[y] = delta_x1[2 * y] + state_nominal[0][2 * y];
					}
					for (int y = 0; y < m->nv; y++)
					{
						delta_x1[2 * y + 1] = ptb_coef * gaussrand();
						d->qvel[y] = delta_x1[2 * y + 1] + state_nominal[0][2 * y + 1];
					}
					for (int ci = 0; ci < ctrl_num; ci++)
					{
						delta_x1[ci + NS] = ptb_coef * gaussrand();
						ctrl[ci] = ctrl_nominal[0 * ctrl_num + ci] + delta_x1[ci + NS];
					}
				}
				else {
					for (int ci = 0; ci < ctrl_num; ci++)
					{
						ctrl[ci] = ctrl_nominal[index * ctrl_num + ci];
					}
				}
			}
			t_init = time;
		}
	}else{
		if (time - t_init < t_step - 0.0001)
		{
			mj_step(m, d);
		}
		else {
			for (int y = 0; y < m->nq; y++)
			{
				delta_x2[2 * y] = d->qpos[y] - state_nominal[index + 1][2 * y];
			}
			for (int y = 0; y < m->nv; y++)
			{
				delta_x2[2 * y + 1] = d->qvel[y] - state_nominal[index + 1][2 * y + 1];
			}

			// option 1: iteration method
			for (int r = 0; r < NS; r++)
			{
				for (int c = 0; c < NS + ctrl_num; c++)
				{
					mat[index][r][c] = mat[index][r][c] * (1 - 1 / (trial + 1.0)) + delta_x2[r] * delta_x1[c] / ((trial + 1.0) * ptb_coef * ptb_coef);
				}
			}

			if (index == 13)
			{
				str3 = glstr;
				sprintf(str3, "%3.3f", mat[index][2][1]);
				fwrite(str3, 5, 1, fop);
				fputs(" ", fop);
			}

			index++;
			t_init = time;

			//// select certain time steps
			//if (index == 1) index = 224;
			//if (index == 130) index = 228;

			if (index >= step_max)
			{
				index = 0;
				trial++;
				if (trial >= roll_max)
				{
					trial = 0;

					//// option 2: inversion method
					//for (int j = 0; j < step_max; j++)
					//{
					//	mju_mulMatTMat(*res6, *delta_x1[j], *delta_x1[j], roll_max, NS+ctrl_num, NS + ctrl_num);
					//	mju_mulMatMat(*res7, *delta_x2[j], *delta_x1[j], NS, roll_max, NS + ctrl_num);
					//	cofactor(res6, NS + ctrl_num);
					//	mju_mulMatMat(*mat[j], *res7, *res, NS, NS + ctrl_num, NS + ctrl_num);
					//}

					for (int h = 0; h < step_max; h++)
					{
						for (int u = 0; u < NS + ctrl_num; u++)
						{
							delta_xc[h][u] = ptb_coef * gaussrand();
						}
					}
					for (int j = 0; j < step_max; j++)
					{
						mju_mulMatVec(dx[j + 1], *mat[j], delta_xc[j], NS, NS + ctrl_num);
					}

					// simulation check
					index = 0;
					t_init = d->time;
					for (int y = 0; y < m->nq; y++)
					{
						d->qpos[y] = delta_xc[index][2 * y] + state_nominal[index][2 * y];
					}
					for (int y = 0; y < m->nv; y++)
					{
						d->qvel[y] = delta_xc[index][2 * y + 1] + state_nominal[index][2 * y + 1];
					}
					for (int ci = 0; ci < ctrl_num; ci++)
					{
						ctrl[ci] = ctrl_nominal[index * ctrl_num + ci] + delta_xc[index][ci + NS];
					}

					while (index < step_max - 1)
					{
						if (d->time - t_init < t_step - 0.0001)
						{
							mj_step(m, d);
						}
						else {
							for (int y = 0; y < m->nq; y++)
							{
								delta_xcs[index][2 * y] = d->qpos[y] - state_nominal[index + 1][2 * y];
							}
							for (int y = 0; y < m->nv; y++)
							{
								delta_xcs[index][2 * y + 1] = d->qvel[y] - state_nominal[index + 1][2 * y + 1];
							}

							index++;
							t_init = d->time;

							for (int y = 0; y < m->nq; y++)
							{
								d->qpos[y] = delta_xc[index][2 * y] + state_nominal[index][2 * y];
							}
							for (int y = 0; y < m->nv; y++)
							{
								d->qvel[y] = delta_xc[index][2 * y + 1] + state_nominal[index][2 * y + 1];
							}
							for (int ci = 0; ci < ctrl_num; ci++)
							{
								ctrl[ci] = ctrl_nominal[index * ctrl_num + ci] + delta_xc[index][ci + NS];
							}
						}
					}
					tri_num++;
				}
			}

			// reload
			for (int y = 0; y < m->nq; y++)
			{
				delta_x1[2 * y] = ptb_coef * gaussrand();
				d->qpos[y] = delta_x1[2 * y] + state_nominal[index][2 * y];
			}
			for (int y = 0; y < m->nv; y++)
			{
				delta_x1[2 * y + 1] = ptb_coef * gaussrand();
				d->qvel[y] = delta_x1[2 * y + 1] + state_nominal[index][2 * y + 1];
			}
			for (int ci = 0; ci < ctrl_num; ci++)
			{
				delta_x1[ci + NS] = ptb_coef * gaussrand();
				ctrl[ci] = ctrl_nominal[index * ctrl_num + ci] + delta_x1[ci + NS];
			}
		}
	}
}

// main function
int main(int argc, const char** argv)
{
	// print help if arguments are missing
	if (argc<2)
		return finish(0);

    // activate MuJoCo Pro license (this must be *your* activation key)
    mj_activate("mjkey.txt");

    // get filename, determine file type
    std::string filename(argv[1]);
    bool binary = (filename.find(".mjb")!=std::string::npos);

    // load model
    char error[1000] = "Could not load binary model";
    if( binary )
        m = mj_loadModel(argv[1], 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        return finish(error);

    // make data
    d = mj_makeData(m);
    if( !d )
        return finish("Could not allocate mjData", m);

	if (STATE_LINEAR == true)
	{
		#if defined(CART2POLE)||defined(CART3POLE)||defined(CART7POLE)
				state_nominal[0][2] = PI;
		#elif defined(PENDULUM)
				state_nominal[0][0] = 0;
		#elif defined(CARTPOLE)
				state_nominal[0][2] = -PI;
		#elif defined(ACROBOT)
				state_nominal[0][0] = 0;
				state_nominal[0][2] = -2 * PI;
		#endif
	}

	srand((unsigned)time(NULL));
	if ((fop = fopen("../../../data/converge.txt", "wt+")) == NULL) {
		return 0;
	}
	if ((fp = fopen("../../../data/result.txt", "r")) != NULL)
	{
		fscanf(fp, "%s", ctrl_buff);
		if (ctrl_buff[0] == 'C') {
			for (int i = 0; i < step_max * ctrl_num; i++)
			{
				fscanf(fp, "%s", ctrl_buff);
				ctrl_nominal[i] = atof(ctrl_buff);

				if (CTRL_LIMITTED == true)
				{
					if (ctrl_nominal[i] > m->actuator_ctrlrange[1]) ctrl_nominal[i] = m->actuator_ctrlrange[1];
					else if (ctrl_nominal[i] < m->actuator_ctrlrange[0]) ctrl_nominal[i] = m->actuator_ctrlrange[0];
				}
				if (STATE_LINEAR == true)
				{
					ctrl_nominal[i] = 0;
				}
			}
			fclose(fp); 
		}
	}

    // time simulation
    int steps = 0, contacts = 0, constraints = 0;
    double printfraction = 0.1;
    printf("\nSimulation ");
    double start = gettm();
	while (tri_num < 1)
    {
		setcontrol(d->time, d->ctrl, m->nu);

        // accumulate statistics
        steps++;
        contacts += d->ncon;
        constraints += d->nefc;

        // print '.' every 10% of duration
        if( trial >= roll_max*printfraction )
        {
            printf(".");
            printfraction += 0.1;
        }
    }
    double end = gettm();

    // print results
    printf("\n Simulation time      : %.2f s\n", end-start);
    printf(" Time per step        : %.3f ms\n", 1000.0*(end-start)/mjMAX(1,steps));
    printf(" Contacts per step    : %d\n", contacts/mjMAX(1,steps));
    printf(" Constraints per step : %d\n", constraints/mjMAX(1,steps));
    printf(" Degrees of freedom   : %d\n\n", m->nv);

	// print result
	if (STATE_LINEAR == true)
	{
		if ((fop2 = fopen("../../../data/linearization_top.txt", "wt+")) != NULL)
		{
			str3 = glstr;
			for (int t = 0; t < step_max; t++)
			{
				for (int h = 0; h < NS; h++)
				{
					for (int d = 0; d < NS + ctrl_num; d++)
					{
						sprintf(str3, "%2.8f", mat[t][h][d]);
						fwrite(str3, 10, 1, fop2);
						fputs(" ", fop2);
					}
					fputs("\n", fop2);
				}
				fputs("\n", fop2);
			}

			// for result checking
			for (int t = 0; t < NS; t++)
			{
				for (int h = 1; h <= step_max; h++)
				{
					sprintf(str3, "%2.4f", dx[h][t]);
					fwrite(str3, 6, 1, fop2);
					fputs(" ", fop2);
				}
				fputs("\n", fop2);
				for (int d = 0; d < step_max; d++)
				{
					sprintf(str3, "%2.4f", delta_xcs[d][t]);
					fwrite(str3, 6, 1, fop2);
					fputs(" ", fop2);
				}
				fputs("\n", fop2);
				fputs("\n", fop2);
			}
			fputs("\nptb_coef: ", fop2);
			sprintf(str3, "%2.4f\n", ptb_coef);
			fwrite(str3, 6, 1, fop2);
			fclose(fop2);
		}
	}
	else {
		if ((fop2 = fopen("../../../data/linearization.txt", "wt+")) != NULL)
		{
			str3 = glstr;
			for (int t = 0; t < step_max; t++)
			{
				for (int h = 0; h < NS; h++)
				{
					for (int d = 0; d < NS + ctrl_num; d++)
					{
						sprintf(str3, "%4.8f", mat[t][h][d]);
						fwrite(str3, 10, 1, fop2);
						fputs(" ", fop2);
					}
					fputs("\n", fop2);
				}
				fputs("\n", fop2);
			}

			// for result checking
			for (int t = 0; t < NS; t++)
			{
				for (int h = 1; h <= step_max; h++)
				{
					sprintf(str3, "%2.4f", dx[h][t]);
					fwrite(str3, 6, 1, fop2);
					fputs(" ", fop2);
				}
				fputs("\n", fop2);
				for (int d = 0; d < step_max; d++)
				{
					sprintf(str3, "%2.4f", delta_xcs[d][t]);
					fwrite(str3, 6, 1, fop2);
					fputs(" ", fop2);
				}
				fputs("\n", fop2);
				fputs("\n", fop2);
			}
			fputs("\nptb_coef: ", fop2);
			sprintf(str3, "%2.4f\n", ptb_coef);
			fwrite(str3, 6, 1, fop2);

			fclose(fop2);
		}
	}

    // finalize
    return finish(0, m, d);
}
