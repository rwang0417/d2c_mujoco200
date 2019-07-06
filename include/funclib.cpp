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

/* Exported variables -------------------------------------------------------*/
/* Private typedef ----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Mujoco function prototypes------------------------------------------------*/

/**
* @brief  Angle modification for pendulum, cartpole and acrobot to clamp angle value
* @note   different modification different model
* @param  mjModel* m: model
*         mjData* d: data
*         mjtNum *ctrl: control value
*         mjtNum nu: number of control
*         mjtNum *Q: state error penalty
*         mjtNum R: control penalty
* @retval mjtNum: cost value
* @author rwang0417@tamu.edu
*/
mjtNum angleModify(int model, mjtNum angle)
{
	switch (model) {
	case 0: 
		return (PI - fabs(angle - PI))*((PI - angle > 0) - (PI - angle < 0));
	case 1:
		return (PI - fabs(angle))*((angle < 0) - (angle > 0));
	case 2:
		return (PI - fabs(angle - PI))*((PI - angle > 0) - (PI - angle < 0));
	default: return 0;
	}
}

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

/**
* @brief  Generate Gaussian random value
* @note   none
* @param  mjtNum mean: mean
*         mjtNum var: variance
* @retval mjtNum: Gaussian random value
* @author rwang0417@tamu.edu
*/
mjtNum randGauss(mjtNum mean, mjtNum var)
{
	mjtNum U, V, Z;

	U = rand() / (RAND_MAX + 1.0);
	V = rand() / (RAND_MAX + 1.0);
	if (U == 0) U = 0.0001;
	Z = sqrt(-2.0 * log(U)) * sin(2.0 * PI * V);

	return mean + sqrt(var) * Z;
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
* @param  mjtNum *res: inversed matrix; mjtNum num[N][N]: input matrix; mjtNum r: rank of the input matrix
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
