//---------------------------------//
//  This file is part of MuJoCo    //
//  Written by Emo Todorov         //
//  Copyright (C) 2017 Roboti LLC  //
//---------------------------------//
// mex mexstep.c mujoco200.lib mujoco200nogl.lib
#pragma once

#include "windows.h"
#include "mex.h" 
#include "mujoco.h"
#include "mjxmacro.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

//-------------------------------- global variables -------------------------------------

// model
mjModel* m = 0;
mjData* d = 0;
char error[1000];
char username[30];

// deallocate and print message
int finish(const char* msg, mjModel* m, mjData* d)
{
	// deallocated everything
	if (d)
		mj_deleteData(d);
	if (m)
		mj_deleteModel(m);
	mj_deactivate();

	// print message
	if (msg)
		printf("%s\n", msg);
	return 0;
}

// find mjData field with given name, return pointer and dimensions
double* findfield(const char* name, int* nr, int* nc)
{
    // prepare constants for NC
    int nv = m->nv;
    int njmax = m->njmax;

    // find field
    #define X(TYPE, NAME, NR, NC)                               \
        if( !strcmp(#NAME, name) && !strcmp(#TYPE, "mjtNum") )  \
        {                                                       \
            *nr = m->NR;                                        \
            *nc = NC;                                           \
            return (double*)d->NAME;                            \
        }

        MJDATA_POINTERS
    #undef X

    // not found or type not mjtNum
    return NULL;
}

void mexFunction(int nout, mxArray* pout[], int nin, const mxArray* pin[])
{
    char filename[100], command[100], fieldname[100];
	mjtNum *x_out, *N, *N_dot, *u, *x_in;
	mjtNum buflen;
	mjtNum integration_per_step = 1;
	int statenum, actuatornum, nodenum;
	char *model;

   mxGetString(pin[0], command, 100);
   //---------------------------- initialize and load model file
    if( !strcmp(command, "load") )
    {
        mxGetString(pin[1], filename, 100);

        // activate MuJoCo license
        DWORD usernamesize = 30;
        GetUserName(username, &usernamesize);
        if (username[0] == 'R') {
            mj_activate("mjkeybig.txt");
        }
        else if (username[0] == '5') {
            mj_activate("mjkeysmall.txt");
        }

        // load and compile model
        char error[1000] = "Could not load binary model";
        m = mj_loadXML(filename, 0, error, 1000);
        if (!m)
		finish(error, 0, 0);

        // make data and update
        d = mj_makeData(m);
        mj_forward(m, d);
        return;
    }
    //---------------------------- terminate
    else if( !strcmp(command, "exit") || !strcmp(command, "quit") )
    {
        finish(0, m, d);
    }
   //---------------------------- get/set mjData field
    else if( !strcmp(command, "get") || !strcmp(command, "set") )
    {
        // get field name
        mxGetString(pin[1], fieldname, 100);

        // find field
        int nr = 0, nc = 0;
        double* fielddata = findfield(fieldname, &nr, &nc);
        if( !fielddata )
            printf("invalid field name");

        // get
        if( !strcmp(command, "get") )
        {   
            // create MATLAB matrix and copy data (assuming mjtNum is double)
            pout[0] = mxCreateDoubleMatrix(nc, nr, mxREAL);
            memcpy(mxGetPr(pout[0]), fielddata, nr*nc*sizeof(double));
        }
        // set
        else
        {
            nc=mxGetScalar(pin[3]);nr=1;
            // copy data (assuming mjtNum is double)
            memcpy(fielddata, mxGetPr(pin[2]), nr*nc*sizeof(double));
        }
    }
   //---------------------------- step [number]
    else if( !strcmp(command, "step") )
    {
        // no number
        if( nin<2 )
            mj_step(m, d);

        // number of steps specified
        else
        {
            // get number of steps
            int number = mju_round(mxGetScalar(pin[1]));
            if( number<0 )
                char error[1000] = "invalid nunber";

            // run for specified number of steps
            for( int i=0; i<number; i++ )
                mj_step(m, d);
        }
    }
   //---------------------------- forward
    else if( !strcmp(command, "forward") )
    {
        mj_forward(m, d);
    }
}