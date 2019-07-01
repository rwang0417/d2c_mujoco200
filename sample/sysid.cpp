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
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;
//-------------------------------- macro variables --------------------------------------

// model selection: PENDULUM CARTPOLE CART2POLE CART3POLE SWIMMER3 SWIMMER6 FISH
// EULER QUAT
#define FISH
#define EULER
#define CTRL_LIMITTED false
#define STATE_LINEAR false
#define LOCAL true
#define PI 3.141592653

//-------------------------------- global variables -------------------------------------

// user customized parameters
#if defined(SWIMMER6)
const mjtNum t_step = 0.006;
const mjtNum cal_step = 0.006;
const mjtNum ptb_coef = 0.005;
const int step_max = 1500;
const int32_t roll_max = 1400;
const char* mname = "swimmer6.xml";
const int ctrl_num = 5;
const int NS = 16;
mjtNum state_nominal[step_max + 1][NS] = { 0 };
#elif defined(SWIMMER3)
const mjtNum t_step = 0.005;//0.01
const mjtNum cal_step = 0.005;
const mjtNum ptb_coef = 0.008;
const int step_max = 1600;//800
const int32_t roll_max = 1200;
const char* mname = "swimmer3.xml";
const int ctrl_num = 2;
const int NS = 10;
mjtNum state_nominal[step_max + 1][NS] = { 0 };
#elif defined(FISH)
const mjtNum t_step = 0.002;
const mjtNum cal_step = 0.002;
const mjtNum ptb_coef = 0.005;
const int step_max = 3000;//16000
const int roll_max = 1600;
const char* mname = "fish.xml";
const int ctrl_num = 6;
const int NS = 26;
mjtNum state_nominal[step_max + 1][NS] = { 0.0, 0.0, 0, 0, 0, 0, 1, 0, 0 };
#elif defined(CART2POLE)
const mjtNum t_step = 0.05;
const int32_t roll_max = 20000; //2w
const char* mname = "cart2pole.xml";
const int step_max = 120;
const int ctrl_num = 1;
const int NS = 6;
mjtNum ptb_coef = 0.04;
mjtNum state_nominal[step_max + 1][NS] = { 0 };
#elif defined(CART7POLE)
const mjtNum t_step = 0.1;
const mjtNum ptb_coef = 1;
const int step_max = 100;
const int32_t roll_max = 20000;
const char* mname = "cart7pole.xml";
const int ctrl_num = 1;
const int NS = 16;
mjtNum state_nominal[step_max + 1][NS] = { 0 };
#elif defined(CART3POLE)
const mjtNum t_step = 0.05;
const mjtNum ptb_coef = 0;
const int step_max = 140;
const int32_t roll_max = 20000;
const char* mname = "cart3pole.xml";
const int ctrl_num = 1;
const int NS = 8;
mjtNum state_nominal[step_max + 1][NS] = { 0 };
#elif defined(ACROBOT)
const mjtNum t_step = 0.01;
const mjtNum ptb_coef = 0.02;
const int step_max = 700;
const int32_t roll_max = 20000;
const char* mname = "acrobot.xml";
const int ctrl_num = 1;
const int NS = 4;
mjtNum state_nominal[step_max + 1][NS] = { -PI, 0, 0, 0 };
#elif defined(PENDULUM)
const mjtNum t_step = 0.1;
const mjtNum cal_step = 0.1;
mjtNum ptb_coef = 0.005;
const int step_max = 30;
const int32_t roll_max = 500;
const char* mname = "pendulum.xml";
const int ctrl_num = 1;
const int NS = 2;
mjtNum state_nominal[step_max + 1][NS] = { PI, 0.0 };
#elif defined(CARTPOLE)
const mjtNum t_step = 0.1;
const mjtNum cal_step = 0.1;
mjtNum ptb_coef = 0.003;
const int step_max = 30;
const int32_t roll_max = 600;
const char* mname = "cartpole.xml";
const int ctrl_num = 1;
const int NS = 4;
mjtNum state_nominal[step_max + 1][NS] = { 0, 0, 0, 0.0 };
#endif

// model
mjModel* m = 0;
mjData* d = 0;
char lastfile[1000] = "";
char error[1000];
static int tri_num = 0;
static int32_t trial = 0;
static int index = 0;
static bool cal_flag = true;
const long N = NS + ctrl_num;
const int n_check = 1;
mjtNum sysiderr = 0;
mjtNum delta_x1[NS + ctrl_num] = { 0 };
mjtNum delta_x2[NS] = { 0 };
mjtNum mat[step_max][NS][NS + ctrl_num] = { 0 };
mjtNum sum[step_max][NS][NS + ctrl_num] = { 0 };
mjtNum dx[step_max + 1][NS];
mjtNum delta_xc[step_max + 1][NS + ctrl_num] = { 0 };
mjtNum delta_xcs[step_max][NS] = { 0 };
mjtNum res[NS + ctrl_num][NS + ctrl_num], res6[NS + ctrl_num][NS + ctrl_num], res7[NS][NS + ctrl_num];
mjtNum ctrl_nominal[step_max * ctrl_num] = { 0 };
mjtNum ulim = 0;
int stepite = (int)(t_step / cal_step);
char ctrl_buff[20];
char para_buff[20];
char glstr[10];
char *str3, *str;
FILE *fp, *fp1, *fop, *fop1, *fop2;
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

// w x y z to roll pitch yaw
void quat2smpl(double *res, double *quat)
{
	double b0, b1, b2, b3;

	b0 = *quat;
	b1 = *(quat + 1);
	b2 = *(quat + 2);
	b3 = *(quat + 3);

	Quaterniond q = Eigen::Quaterniond(b0, b1, b2, b3); // w x y z
	auto r = q.toRotationMatrix().eulerAngles(0, 1, 2);

	*res = r[0];
	*(res + 1) = r[1];
	*(res + 2) = r[2];
}

// roll pitch yaw to w x y z
void smpl2quat(double *res, double *smpl)
{
	double rx, ry, rz; // roll pitch yaw

	rx = *smpl;
	ry = *(smpl + 1);
	rz = *(smpl + 2);

	if (rx > PI) rx = PI - rx;

	Quaterniond q = AngleAxisd(rx, Vector3d::UnitX())
		* AngleAxisd(ry, Vector3d::UnitY())
		* AngleAxisd(rz, Vector3d::UnitZ());

	Eigen::Vector4d b = q.coeffs(); // x y z w

	*res = b[3];
	*(res + 1) = b[0];
	*(res + 2) = b[1];
	*(res + 3) = b[2];
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

void get_nominal(mjModel* m, mjData* d)
{
	int index = 0, i, y;
	mjtNum temp[4];
#if STATE_LINEAR == true
	static int step_max = 1;
#endif

	mju_zero(d->qpos, m->nq);
	mju_zero(d->qvel, m->nv);
	for (y = 0; y < NS/2; y++)
	{
		d->qpos[y] = state_nominal[index][2 * y];
	}
	for (y = 0; y < m->nv; y++)
	{
		d->qvel[y] = state_nominal[index][2 * y + 1];
	}
	for (y = 0; y < ctrl_num; y++)
	{
		d->ctrl[y] = ctrl_nominal[index * ctrl_num + y];
	}

#if defined(EULER)
	quat2smpl(temp, &(d->qpos[3]));
	for (int k = 0; k < 3; k++)state_nominal[index][2 * k + 6] = temp[k];
#elif defined(QUAT)
	for (int k = 0; k < 3; k++)state_nominal[index][2 * k + 6] = d->qpos[k + 4];
#endif

	while (index < step_max)
	{
		for (i = 0; i < stepite; i++) mj_step(m, d);

		index++;
		for (y = 0; y < 3; y++)
		{
			state_nominal[index][2 * y] = d->qpos[y];
		}

#if defined(EULER)
		quat2smpl(temp, &(d->qpos[3]));
		for (int k = 0; k < 3; k++)state_nominal[index][2 * k + 6] = temp[k];
#elif defined(QUAT)
		for (int k = 0; k < 3; k++)state_nominal[index][2 * k + 6] = d->qpos[k + 4];
#endif

		for (y = 7; y < m->nq; y++)
		{
			state_nominal[index][2 * (y-1)] = d->qpos[y];
		}
		for (y = 0; y < m->nv; y++)
		{
			state_nominal[index][2 * y + 1] = d->qvel[y];
		}
		for (y = 0; y < ctrl_num; y++)
		{
			d->ctrl[y] = ctrl_nominal[index * ctrl_num + y];
		}
	}
}

void sysidcheck(mjModel* m, mjData* d)
{
	static int index = 0;
	static mjtNum t_init;
	const int step_max = 1500;
	int i, j, k;
	mjtNum dx[n_check][step_max][NS];
	mjtNum delta_xc[n_check][step_max][NS + ctrl_num] = { 0 };
	mjtNum delta_xcs[n_check][step_max][NS] = { 0 };
	mjtNum temp[4];

	for (i = 0; i < NS + ctrl_num; i++)
	{
		for (j = 0; j < n_check; j++)
		{
			for (k = 0; k < step_max; k++) delta_xc[j][k][i] = 0.005 * ulim * gaussrand();
		}
	}
	for (i = 0; i < n_check; i++)
	{
		for (j = 0; j < step_max; j++) mju_mulMatVec(dx[i][j], *mat[j], delta_xc[i][j], NS, NS + ctrl_num);
	}

	for (i = 0; i < n_check; i++)
	{
		index = 0;
		t_init = d->time;
		for (j = 0; j < 3; j++)
		{
			d->qpos[j] = delta_xc[i][index][2 * j] + state_nominal[index][2 * j];
		}

#if defined(EULER)
		for (k = 0; k < 3; k++) temp[k] = delta_xc[i][index][2 * k + 6] + state_nominal[index][2 * k + 6];
		smpl2quat(&(d->qpos[3]), temp);
#elif defined(QUAT)
		for (k = 0; k < 3; k++) d->qpos[k + 4] = delta_xc[i][index][2 * k + 6] + state_nominal[index][2 * k + 6];
		d->qpos[3] = sqrt(1 - d->qpos[4] * d->qpos[4] - d->qpos[5] * d->qpos[5] - d->qpos[6] * d->qpos[6]);
#endif

		for (j = 7; j < m->nq; j++)
		{
			d->qpos[j] = delta_xc[i][index][2 * (j - 1)] + state_nominal[index][2 * (j - 1)];
		}
		for (j = 0; j < m->nv; j++)
		{
			d->qvel[j] = delta_xc[i][index][2 * j + 1] + state_nominal[index][2 * j + 1];
		}
		for (k = 0; k < ctrl_num; k++)
		{
			d->ctrl[k] = delta_xc[i][index][k + NS] + ctrl_nominal[index * ctrl_num + k];
		}
		while (index < step_max - 1)
		{
			if (d->time - t_init < t_step - 0.00001)
			{
				mj_step(m, d);
			}
			else {
				for (int y = 0; y < 3; y++)
				{
					delta_xcs[i][index][2 * y] = d->qpos[y] - state_nominal[index + 1][2 * y];
				}

#if defined(EULER)
				quat2smpl(temp, &(d->qpos[3]));
				for (int k = 0; k < 3; k++) delta_xcs[i][index][2 * k + 6] = temp[k] - state_nominal[index][2 * k + 6];
#elif defined(QUAT)
				for (int k = 0; k < 3; k++) delta_xcs[i][index][2 * k + 6] = d->qpos[k+4] - state_nominal[index][2 * k + 6];
#endif

				for (int y = 7; y < m->nq; y++)
				{
					delta_xcs[i][index][2 * (y - 1)] = d->qpos[y] - state_nominal[index + 1][2 * (y - 1)];
				}
				for (int y = 0; y < m->nv; y++)
				{
					delta_xcs[i][index][2 * y + 1] = d->qvel[y] - state_nominal[index + 1][2 * y + 1];
				}

				index++;
				t_init = d->time;

				for (int y = 0; y < 3; y++)
				{
					d->qpos[y] = delta_xc[i][index][2 * y] + state_nominal[index][2 * y];
				}

				
#if defined(EULER)
				for (int k = 0; k < 3; k++) temp[k] = delta_xc[i][index][2 * k + 6] + state_nominal[index][2 * k + 6];
				smpl2quat(&(d->qpos[3]), temp);
#elif defined(QUAT)
				for (int k = 0; k < 3; k++) d->qpos[k + 4] = delta_xc[i][index][2 * k + 6] + state_nominal[index][2 * k + 6];
				d->qpos[3] = sqrt(1 - d->qpos[4] * d->qpos[4] - d->qpos[5] * d->qpos[5] - d->qpos[6] * d->qpos[6]);
#endif

				for (j = 7; j < m->nq; j++)
				{
					d->qpos[j] = delta_xc[i][index][2 * (j - 1)] + state_nominal[index][2 * (j - 1)];
				}
				for (int y = 0; y < m->nv; y++)
				{
					d->qvel[y] = delta_xc[i][index][2 * y + 1] + state_nominal[index][2 * y + 1];
				}
				for (int ci = 0; ci < ctrl_num; ci++)
				{
					d->ctrl[ci] = delta_xc[i][index][ci + NS] + ctrl_nominal[index * ctrl_num + ci];
				}
			}
		}
		for (int t = 0; t < NS; t++)
		{
			for (int h = 0; h < step_max; h++)
			{
				if (delta_xcs[i][h][t] != 0) sysiderr += fabs((dx[i][h][t] - delta_xcs[i][h][t]) / delta_xcs[i][h][t]);
			}
		}
	}
	sysiderr = sysiderr / (1.0*n_check*step_max*NS);
}

void check(mjModel* m, mjData* d)
{
	static int index = 0, h, u, y;
#if STATE_LINEAR == true
	static int step_max = 1;
#endif

	for (u = 0; u < NS + ctrl_num; u++)
	{
		for (h = 0; h < step_max; h++) delta_xc[h][u] = ptb_coef * ulim * gaussrand();
	}
	for (h = 0; h < step_max; h++)
	{
		mju_mulMatVec(dx[h + 1], *mat[h], delta_xc[h], NS, NS + ctrl_num);
	}
	while (index < step_max)
	{
		for (y = 0; y < m->nq; y++)
		{
			d->qpos[y] = delta_xc[index][2 * y] + state_nominal[index][2 * y];
		}
		for (y = 0; y < m->nv; y++)
		{
			d->qvel[y] = delta_xc[index][2 * y + 1] + state_nominal[index][2 * y + 1];
		}
		for (h = 0; h < ctrl_num; h++)
		{
			d->ctrl[h] = ctrl_nominal[index * ctrl_num + h] + delta_xc[index][h + NS];
		}

		for (h = 0; h < stepite; h++) mj_step(m, d);

		for (y = 0; y < m->nq; y++)
		{
			delta_xcs[index][2 * y] = d->qpos[y] - state_nominal[index + 1][2 * y];
		}
		for (y = 0; y < m->nv; y++)
		{
			delta_xcs[index][2 * y + 1] = d->qvel[y] - state_nominal[index + 1][2 * y + 1];
		}
		index++;
	}
}

void setcontrol(mjModel* m, mjData* d)
{
	mjtNum temp[4];
	static int index = 0, i, y;
#if STATE_LINEAR == true
	static int step_max = 1;
#endif

	while (index < step_max)
	{
		for (y = 0; y < 3; y++)
		{
			delta_x1[2 * y] = ptb_coef * ulim * gaussrand();
			d->qpos[y] = delta_x1[2 * y] + state_nominal[index][2 * y];
		}
		for (int k = 0; k < 3; k++) {
			delta_x1[2 * k + 6] = ptb_coef * ulim * gaussrand();

			
#if defined(EULER)
			temp[k] = delta_x1[2 * k + 6] + state_nominal[index][2 * k + 6];
#elif defined(QUAT)
			d->qpos[k + 4] = delta_x1[2 * k + 6] + state_nominal[index][2 * k + 6];
#endif
		}
		
#if defined(EULER)
		smpl2quat(&(d->qpos[3]), temp);
#elif defined(QUAT)
		d->qpos[3] = sqrt(1 - d->qpos[4] * d->qpos[4] - d->qpos[5] * d->qpos[5] - d->qpos[6] * d->qpos[6]);
#endif

		for (int j = 7; j < m->nq; j++)
		{
			delta_x1[2 * (j - 1)] = ptb_coef * ulim * gaussrand();
			d->qpos[j] = delta_x1[2 * (j - 1)] + state_nominal[index][2 * (j - 1)];
		}
		for (y = 0; y < m->nv; y++)
		{
			delta_x1[2 * y + 1] = ptb_coef * ulim * gaussrand();
			d->qvel[y] = delta_x1[2 * y + 1] + state_nominal[index][2 * y + 1];
		}
		for (i = 0; i < ctrl_num; i++)
		{
			delta_x1[i + NS] = ptb_coef * ulim * gaussrand();
			d->ctrl[i] = ctrl_nominal[index * ctrl_num + i] + delta_x1[i + NS];
		}

		for (i = 0; i < stepite; i++) mj_step(m, d);

		for (int y = 0; y < 3; y++)
		{
			delta_x2[2 * y] = d->qpos[y] - state_nominal[index + 1][2 * y];
		}

#if defined(EULER)
		quat2smpl(temp, &(d->qpos[3]));
		for (int k = 0; k < 3; k++) delta_x2[2 * k + 6] = temp[k] - state_nominal[index][2 * k + 6];
#elif defined(QUAT)
		for (int k = 0; k < 3; k++) delta_x2[2 * k + 6] = d->qpos[k+4] - state_nominal[index][2 * k + 6];
#endif

		for (int y = 7; y < m->nq; y++)
		{
			delta_x2[2 * (y - 1)] = d->qpos[y] - state_nominal[index + 1][2 * (y - 1)];
		}
		for (int y = 0; y < m->nv; y++)
		{
			delta_x2[2 * y + 1] = d->qvel[y] - state_nominal[index + 1][2 * y + 1];
		} 

		// iteration method
		for (y = 0; y < NS; y++)
		{
			for (i = 0; i < NS + ctrl_num; i++)
			{
				mat[index][y][i] = mat[index][y][i] * (1 - 1 / (trial + 1.0)) + delta_x2[y] * delta_x1[i] / ((trial + 1.0) * ptb_coef * ptb_coef * ulim * ulim);
			}
		}

		if (index == 0)
		{
			str3 = glstr;
			sprintf(str3, "%3.3f", mat[index][0][1]);
			fwrite(str3, 5, 1, fop);
			fputs(" ", fop);
		}
		index++;
	}
	index = 0;
	trial++;
}

// main function
int main(int argc, const char** argv)
{
    // activate MuJoCo Pro license (this must be *your* activation key)
	strcpy(kfilename, kfilepre);
	strcat(kfilename, "mjkeybig.txt");
    mj_activate(kfilename);

    // get filename, determine file type
    std::string filename(mname);
    bool binary = (filename.find(".mjb")!=std::string::npos);
	strcpy(mfilename, mfilepre);
	strcat(mfilename, mname);

    // load model
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
	strcpy(dfilename, dfilepre);
	strcat(dfilename, "converge.txt");
	if ((fop = fopen(dfilename, "wt+")) == NULL) {
		return 0;
	}
	strcpy(dfilename, dfilepre);
	strcat(dfilename, "result.txt");
	if ((fp = fopen(dfilename, "r")) != NULL)
	{
		fscanf(fp, "%s", ctrl_buff);
		if (ctrl_buff[0] == 'C') {
			for (int i = 0; i < step_max * ctrl_num; i++)
			{
				fscanf(fp, "%s", ctrl_buff);
				ctrl_nominal[i] = atof(ctrl_buff);
				if (fabs(ctrl_nominal[i]) > ulim) ulim = fabs(ctrl_nominal[i]);

				#if CTRL_LIMITTED == true
					if (ctrl_nominal[i] > m->actuator_ctrlrange[1]) ctrl_nominal[i] = m->actuator_ctrlrange[1];
					else if (ctrl_nominal[i] < m->actuator_ctrlrange[0]) ctrl_nominal[i] = m->actuator_ctrlrange[0];
				#endif
				#if STATE_LINEAR == true
					ctrl_nominal[i] = 0;
				#endif
			}
			fclose(fp); 
		}
	}

    // time simulation
    int steps = 0, contacts = 0, constraints = 0;
    double printfraction = 0.1;
    printf("\nSimulation ");
    double start = gettm();
	get_nominal(m, d);
	while (trial < roll_max)
    {
		setcontrol(m, d);

        // accumulate statistics
        steps = steps + stepite;
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

	sysidcheck(m, d);

	// print result
	#if STATE_LINEAR == true
		strcpy(dfilename, dfilepre);
		strcat(dfilename, "linearization_top.txt");
		if ((fop2 = fopen(dfilename, "wt+")) != NULL)
		{
			str3 = glstr;
			for (int t = 0; t < 1; t++)
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
				for (int h = 1; h <= 1; h++)
				{
					sprintf(str3, "%2.4f", dx[h][t]);
					fwrite(str3, 6, 1, fop2);
					fputs(" ", fop2);
				}
				fputs("\n", fop2);
				for (int d = 0; d < 1; d++)
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
	#else
		strcpy(dfilename, dfilepre);
		strcat(dfilename, "lnr.txt");
		if ((fop2 = fopen(dfilename, "wt+")) != NULL)
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

			//// for result checking
			//for (int t = 0; t < NS; t++)
			//{
			//	for (int h = 1; h <= step_max; h++)
			//	{
			//		sprintf(str3, "%2.4f", dx[h][t]);
			//		fwrite(str3, 6, 1, fop2);
			//		fputs(" ", fop2);
			//	}
			//	fputs("\n", fop2);
			//	for (int d = 0; d < step_max; d++)
			//	{
			//		sprintf(str3, "%2.4f", delta_xcs[d][t]);
			//		fwrite(str3, 6, 1, fop2);
			//		fputs(" ", fop2);
			//	}
			//	fputs("\n", fop2);
			//	fputs("\n", fop2);
			//}
			fputs("sysiderr: ", fop2);
			sprintf(str3, "%2.4f\n", sysiderr);
			fwrite(str3, 6, 1, fop2);
			fputs("\nptb_coef: ", fop2);
			sprintf(str3, "%2.4f\n", ptb_coef);
			fwrite(str3, 6, 1, fop2);

			fclose(fop2);
		}

		//// state output
		//strcpy(dfilename, dfilepre);
		//strcat(dfilename, "states.txt");
		//if ((fp1 = fopen(dfilename, "at+")) != NULL)
		//{
		//	str3 = glstr;
		//	for (int h = 0; h < NS; h++)
		//	{
		//		for (int d = 1; d < step_max; d++)
		//		{
		//			sprintf(str3, "%4.8f", state_nominal[d][h]);
		//			fwrite(str3, 10, 1, fp1);
		//			fputs(" ", fp1);
		//		}
		//		fputs("\n", fp1);
		//	}
		//	fclose(fp1);
		//}
	#endif
	// hold cmd
	system("pause");
    // finalize
    return finish(0, m, d);
}
