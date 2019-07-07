/**
******************************************************************************
* @file    funclib.h
* @author  Ran Wang EDPLab@TAMU
* @version V1.0
* @date    2018-4-28
* @brief   This file contains the test functions prototypes for the mujoco project
******************************************************************************
* <h2><center>&copy; COPYRIGHT EDPLab@TAMU</center></h2>
******************************************************************************
*/

#pragma once
/* Includes -----------------------------------------------------------------*/

#include "mujoco.h"
#include "mjxmacro.h"
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <cstring>
#include <chrono>
#include <math.h> 
#include <time.h>
#include "Eigen/Geometry"
#include <iostream>

using namespace Eigen;
using namespace std;

/* Exported typedef ---------------------------------------------------------*/
/* Exported variables -------------------------------------------------------*/

const int N = 4;
const int N1 = 30;
const mjtNum PI = 3.141592653;

/* Exported functions ------------------------------------------------------- */

mjtNum mjc_costFunction(mjModel* m, mjData* d, mjtNum *ctrl, mjtNum nu, mjtNum *Q, mjtNum R);
void mjc_modelInit(mjModel* m, mjData* d);

/**
* @brief  Generate Gaussian random value
* @note   none
* @param  mjtNum mean: mean
*         mjtNum var: variance
* @retval mjtNum: Gaussian random value
* @author rwang0417@tamu.edu
*/
mjtNum randGauss(mjtNum mean, mjtNum var);

/**
* @brief  Angle modification for pendulum, cartpole and acrobot to clamp angle value
* @note   different modification different model
* @param  const char* model: name of the model
*         mjtNum angle: current angle value read from mujoco
* @retval mjtNum: modified angle value
* @author rwang0417@tamu.edu
*/
mjtNum angleModify(const char* model, mjtNum angle);

/**
* @brief  Select model parameters set
* @note   none
* @param  const char* model: name of the model whose parameters to select
* @retval none
* @author rwang0417@tamu.edu
*/
void modelSelection(const char* model);

/**
* @brief  calculate the cost at a step
* @note   none
* @param  mjData* d: mujoco simulation data at the specific step
		  mjModel* m: mujoco model
		  int step_index: the step number whose cost needs calculation
* @retval mjtNum: cost value at the specific step
* @author rwang0417@tamu.edu
*/
mjtNum stepCost(mjModel* m, mjData* d, int step_index);

mjtNum determinant(mjtNum *fh, mjtNum r);
void transpose(mjtNum *res, mjtNum *mat, mjtNum rmat, mjtNum cmat = 0);
void cofactor(mjtNum *res, mjtNum *arg, mjtNum r);

void save_result(const char *_filename, mjtNum *u, mjtNum *u_init, mjtNum len, mjtNum *Q, mjtNum *QT, mjtNum *R, mjtNum *ptb_coef, mjtNum *step_coef, mjtNum ns, const char *_mode = "wt+");
void fw_array(FILE *fstream, mjtNum *prt, mjtNum len = 1, const char *_name = "Array1: ");
void fw_array(const char *_filename, mjtNum *prt, mjtNum len = 1, const char *_name = "Array1: ", const char *_mode = "wt+");
void fw_matrix(FILE *fstream, mjtNum *prt, mjtNum r, mjtNum c = 0, const char *_name = "Matrix1: ");
void fw_matrix(const char *_filename, mjtNum *prt, mjtNum r, mjtNum c = 0, const char *_name = "Matrix1: ", const char *_mode = "wt+");

void get_para(const char *_filename, mjtNum ns, mjtNum *Q, mjtNum *QT, mjtNum *R, mjtNum *ptb_coef, mjtNum *step_coef_init);
void fr_array(FILE *fstream, const char *_name, mjtNum *prt, mjtNum len = 1);
void fr_array(const char *_filename, const char *_name, mjtNum *prt, mjtNum len = 1);
void fr_matrix(FILE *fstream, const char *_name, mjtNum *prt, mjtNum r, mjtNum c = 0, const char *_mode = "diag");
void fr_matrix(const char *_filename, const char *_name, mjtNum *prt, mjtNum r, mjtNum c = 0, const char *_mode = "diag");

/* __FUNCLIB_H */