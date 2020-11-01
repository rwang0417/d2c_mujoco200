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
//-------------------------------- global variables -------------------------------------

// model
mjModel* m = 0;
mjData* d = 0;
mjtNum t_init = -0.2;
static mjtNum top_flag = 0;
char lastfile[1000] = "";
char error[1000];
static int trial = 0;
static int tri_num = 0;
static mjtNum cal_flag = 1;
const mjtNum t_step = 0.1;
const int32_t step_max = 1;
const int N1 = 5;
const int ctrl_num = 4;
const int32_t roll_max = step_max * ctrl_num;
const long N = step_max * ctrl_num;
const double PI = 3.141592654;
mjtNum cost[roll_max+1] = { 0 };
mjtNum u[step_max * ctrl_num] = { 0 };
mjtNum gradient[step_max * ctrl_num] = { 0 };
mjtNum delta_u[roll_max][step_max*ctrl_num] = { 0 };
mjtNum delta_j[roll_max] = { 0 };
mjtNum res[step_max * ctrl_num][step_max * ctrl_num];
mjtNum resp[5][5];
mjtNum res4[step_max * ctrl_num][step_max * ctrl_num];
mjtNum record[step_max + 2][4];
mjtNum u_init[step_max * ctrl_num];
char para_buff[20];
char glstr[10];
FILE *fp;
FILE *fop;
FILE *fop1;

mjtNum x_goal[4] = {
	(mjtNum)(1.6),
	(mjtNum)(0.5),
	(mjtNum)(-3.14),
	(mjtNum)(0),
};
/* Parameters read from file fop */
mjtNum Q;
mjtNum QT;
mjtNum ptb_coef;
mjtNum step_coef;
mjtNum step_coef_init;
mjtNum R;

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

/* Calculate Determinant of the Matrix */
mjtNum determinant(mjtNum fh[N][N], mjtNum r)
{
	long z, j, i;
	mjtNum m = 1, k, a[N][N], det = 1.0;

	for (i = 0; i < r; i++)
	{
		for (j = 0; j < r; j++)
		{
			a[i][j] = fh[i][j];
		}
	}

	for (z = 0; z<r - 1; z++)
		for (i = z; i<r - 1; i++)
		{
			if (a[z][z] == 0)
			{
				for (j = 0; j<r; j++)
				{
					a[z][j] = a[z][j] + a[i + 1][j];
				}
			}
			if (a[z][z] != 0) {
				k = -a[i + 1][z] / a[z][z];
				for (j = z; j<r; j++)a[i + 1][j] = k * a[z][j] + a[i + 1][j];
			}
		}
	for (z = 0; z<r; z++)
	{
		det = det * (a[z][z]);
	}
	return (det);
}

/* Find transpose of matrix */
void transpose(mjtNum num[N][N], mjtNum fac[N][N], mjtNum r)
{
	long i, j;
	mjtNum b[N][N], d;

	for (i = 0; i < r; i++)
	{
		for (j = 0; j < r; j++)
		{
			b[i][j] = fac[j][i];
		}
	}
	d = determinant(num, r);
	for (i = 0; i < r; i++)
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
	long p, q, m, n, i, j;
	for (q = 0; q < f; q++)
	{
		for (p = 0; p < f; p++)
		{
			m = 0;
			n = 0;
			for (i = 0; i < f; i++)
			{
				for (j = 0; j < f; j++)
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
			fac[q][p] = pow(mjtNum(-1), q + p) * determinant(b, f - 1);
		}
	}
	transpose(num, fac, f);
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

void setcontrol(mjtNum time, mjtNum* ctrl, int nu)
{
	mjtNum res3[step_max * ctrl_num], avg_j;
	static int index = 0;
	static mjtNum sum = 0;
	static mjtNum gra_max = 3;
	static mjtNum last_tri = 6000;
	FILE *fop2;
	char *str, *str3;

	{
		if (time - t_init <= 0.1)
		{
			for (int ci = 0; ci < ctrl_num; ci++)
			{
				/*if ((index * ctrl_num + ci) == trial) ctrl[ci] = u[index * ctrl_num + ci] + ptb_coef;
				else ctrl[ci] = u[index * ctrl_num + ci];*/
				ctrl[ci] = 0;
			}
		}
		else {
			//record[index][1] = d->qpos[1] - x_goal[1];
			//record[index][0] = d->qvel[0] - x_goal[0];

			//if(index == step_max - 1) cost[trial] += QT * (record[index][1] * record[index][1]) + R * (ctrl[0] * ctrl[0] + ctrl[1] * ctrl[1] + ctrl[2] * ctrl[2] + ctrl[3] * ctrl[3]);
			////else if(index <= 15) cost[trial] += 2 * Q * (record[index][1] * record[index][1]) + R * (ctrl[0] * ctrl[0] + ctrl[1] * ctrl[1] + ctrl[2] * ctrl[2] + ctrl[3] * ctrl[3]);
			//else cost[trial] += Q * (record[index][1] * record[index][1]) + R * (ctrl[0] * ctrl[0] + ctrl[1] * ctrl[1] + ctrl[2] * ctrl[2] + ctrl[3] * ctrl[3]);
			cost[0] = 100000 * d->qpos[1];

			t_init = time;
			index++;

			if (index >= step_max)
			{
				str = glstr;
				sprintf(str, "%5.0f", cost[0]);
				fwrite(str, 5, 1, fp);
				fputs(" ", fp);
				index = 0;
				//trial++;
				//if (trial >= roll_max)
				//{
				//	trial = 0;
				//	/*str = glstr;
				//	sprintf(str, "%5.0f", cost[roll_max]);
				//	fwrite(str, 5, 1, fp);
				//	fputs(" ", fp);*/
				//	for (int j = 0; j < roll_max; j++)
				//	{
				//		//delta_j[j] = cost[j] - cost[roll_max];
				//		cost[j] = 0;
				//	}
				//	//cost[roll_max] = 0;
				//}
				//	// print control pos vel
				//	if ((fop2 = fopen("../../../data/result.txt", "wt+")) != NULL)
				//	{
				//		str3 = glstr;
				//		fputs("Control: ", fop2);
				//		for (long h = 0; h < step_max * ctrl_num; h++)
				//		{
				//			sprintf(str3, "%2.4f", u[h]);
				//			fwrite(str3, 6, 1, fop2);
				//			fputs(" ", fop2);
				//		}
				//		fputs("\n", fop2);
				//		fputs("Control Init: ", fop2);
				//		for (long d = 0; d < step_max * ctrl_num; d++)
				//		{
				//			sprintf(str3, "%2.4f", u_init[d]);
				//			fwrite(str3, 6, 1, fop2);
				//			fputs(" ", fop2);
				//		}
				//		fputs("\n", fop2);
				//		fputs("Height: ", fop2);
				//		for (int d = 0; d < step_max; d++)
				//		{
				//			sprintf(str3, "%2.4f", record[d][1]);
				//			fwrite(str3, 6, 1, fop2);
				//			fputs(" ", fop2);
				//		}
				//		fputs("\n", fop2);
				//			
				//		fputs("Q: ", fop2);
				//		sprintf(str3, "%2.4f", Q);
				//		fwrite(str3, 6, 1, fop2);
				//		fputs("\n", fop2);
				//		fputs("QT ", fop2);
				//		sprintf(str3, "%2.4f", QT);
				//		fwrite(str3, 6, 1, fop2);
				//		fputs("\n", fop2);
				//		fputs("R: ", fop2);
				//		sprintf(str3, "%2.4f", R);
				//		fwrite(str3, 6, 1, fop2);
				//		fputs("\n", fop2);
				//		fputs("ptb_coef: ", fop2);
				//		sprintf(str3, "%2.4f", ptb_coef);
				//		fwrite(str3, 6, 1, fop2);
				//		fputs("\nstep_coef: ", fop2);
				//		sprintf(str3, "%2.4f", step_coef);
				//		fwrite(str3, 6, 1, fop2);
				//		fclose(fop2);
				//	}

				//	/*for (int h = 0; h < step_max * ctrl_num; h++)
				//	{
				//		if (step_coef * delta_j[h] > 0.1) u[h] -= 0.1;
				//		else if (step_coef * delta_j[h] < -0.1) u[h] -= -0.1;
				//		else u[h] -= step_coef * delta_j[h];
				//	}*/
				//	tri_num++;

				//	step_coef = step_coef_init;// *pow(0.991, tri_num);
				//	// ptb_coef = 0.13 * pow(1, tri_num);
				//	//if ((step_coef < 0.0004) && (step_coef > 0) || tri_num > 1500) step_coef = step_coef_init/20;
				//	//if (tri_num > 400) step_coef = step_coef_init/20;
				//	/*if (tri_num > 1400) step_coef = step_coef_init/100;*/
				//}

				// reset
				t_init = time;
				mju_zero(d->qpos, m->nq);
				mju_zero(d->qvel, m->nv);
				mju_zero(d->act, m->na);
				//mj_forward(m, d);
			}
		}
	}
}

// main function
int main(int argc, const char** argv)
{
    // print help if arguments are missing
    if( argc<3 )
        return finish(helpstring);

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
		if ((fp = fopen("../../../data/cost.txt", "wt+")) == NULL) {
			return 0;
		}
		//if ((fop = fopen("../../../data/parameters.txt", "r")) != NULL) {					
		//	while (!feof(fop))
		//	{
		//		fscanf(fop, "%s", para_buff);
		//		if (para_buff[1] == '_')
		//		{
		//			fscanf(fop, "%s", para_buff);
		//			Q = atof(para_buff);
		//		}
		//		if (para_buff[0] == 'R')
		//		{
		//			fscanf(fop, "%s", para_buff);
		//			R = atof(para_buff);
		//		}
		//		if (para_buff[1] == 'T')
		//		{
		//			fscanf(fop, "%s", para_buff);
		//			QT = atof(para_buff);
		//		}
		//		if (para_buff[0] == 'p')
		//		{
		//			fscanf(fop, "%s", para_buff);
		//			ptb_coef = atof(para_buff);
		//		}
		//		if (para_buff[0] == 's')
		//		{
		//			fscanf(fop, "%s", para_buff);
		//			step_coef_init = atof(para_buff);
		//		}
		//	}
		//	fclose(fop);
		//}
			
		//for (int b = 0; b < step_max; b++)
		//{
		//	//u[b][0] = 1 * mju_sin(3.0*b / step_max * PI);
		//	//u_init[b] = 0 * gaussrand();
		//	//u[b][0] = u_init[b];
		//}

        // time simulation
        int steps = 0, contacts = 0, constraints = 0;
        double printfraction = 0.1;
        printf("\nSimulation ");
        double start = gettm();
		t_init = d->time;
        while( d->time<duration )
        {
			setcontrol(d->time, d->ctrl, m->nu);

            // advance simulation
            mj_step(m, d);

            // accumulate statistics
            steps++;
            contacts += d->ncon;
            constraints += d->nefc;

            // print '.' every 10% of duration
            if( d->time >= duration*printfraction )
            {
                printf(".");
                printfraction += 0.1;
            }
        }
        double end = gettm();

        // print results
        printf("\n Simulation time      : %.2f s\n", end-start);
        printf(" Realtime factor      : %.2f x\n", duration/mjMAX(1E-10,(end-start)));
        printf(" Time per step        : %.3f ms\n", 1000.0*(end-start)/mjMAX(1,steps));
        printf(" Contacts per step    : %d\n", contacts/mjMAX(1,steps));
        printf(" Constraints per step : %d\n", constraints/mjMAX(1,steps));
        printf(" Degrees of freedom   : %d\n\n", m->nv);
    }

    // finalize
    return finish();
}
