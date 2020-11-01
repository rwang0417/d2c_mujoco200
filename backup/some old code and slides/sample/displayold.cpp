//---------------------------------//
//  This file is part of MuJoCo    //
//  Written by Emo Todorov         //
//  Copyright (C) 2017 Roboti LLC  //
//---------------------------------//

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "string.h"
#include "malloc.h"
#include "math.h" 
#include "time.h"

//-------------------------------- macro variables --------------------------------------

// model selection: PENDULUM CARTPOLE CART2POLE CART3POLE CART7POLE ACROBOT FISH SWIMMER6 SWIMMER3
#define ACROBOT
#define CTRL_LIMITTED false
#define SYSIDCHECK false
#define ERRCOMPARE false
#define PI 3.141592653

//-------------------------------- global variables -------------------------------------

// user customized parameters
#if defined(SWIMMER3)
const mjtNum t_step = 0.006;
const mjtNum ptb_coef = 5.4;//5.6
const int32_t step_max = 2000;
const int ctrl_num = 2;
const int NS = 10;
const char* mfilename = "../../../model/swimmer3.xml";
static bool TOP_FLAG = false;
const mjtNum K[NS] = { 0 };
mjtNum state_nominal[step_max + 1][NS + 1] = { 0.0 };
mjtNum x_goal[NS] = { 0 };
#elif defined(SWIMMER6)
const mjtNum t_step = 0.006;
const mjtNum ptb_coef = 3.6;//4.5
const int32_t step_max = 2000;
const int ctrl_num = 5;
const int NS = 16;
const char* mfilename = "../../../model/swimmer6.xml";
static bool TOP_FLAG = false;
const mjtNum K[NS] = { 0 };
mjtNum state_nominal[step_max + 1][NS + 1] = { 0.0 };
mjtNum x_goal[NS] = { 0 };
#elif defined(FISH)
const mjtNum t_step = 0.004;
const mjtNum ptb_coef = 0.1;
const int32_t step_max = 2000;
const int ctrl_num = 5;
const int NS = 26;
const char* mfilename = "../../../model/fish.xml";
static bool TOP_FLAG = false;
const mjtNum K[NS-1] = { 0 };
mjtNum state_nominal[step_max + 1][NS+1] = { 0.0, 0.0, 0, 0, 0, 0, 0, 0, 1 };
mjtNum state_openlp[step_max + 1][NS+1] = { 0 };
mjtNum state_closedlp[step_max + 1][NS+1] = { 0 };
mjtNum TK[step_max][ctrl_num][NS-1] = { 0 };
mjtNum x_goal[NS] = { 0 };
#elif defined(CART7POLE)
const mjtNum t_step = 0.1;
const mjtNum ptb_coef = 1;
const int32_t step_max = 100;
const int ctrl_num = 1;
const int NS = 16;
const char* mfilename = "../../../model/cart7pole.xml";
static bool TOP_FLAG = false;
const mjtNum K[NS] = { 0 };
mjtNum state_nominal[step_max + 1][NS] = { 0.0, 0.0, 0, 0, 0, 0 };
// top is 0
mjtNum x_goal[NS] = {
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(PI),
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
};
#elif defined(CART3POLE)
const mjtNum t_step = 0.05;
const mjtNum ptb_coef = 0;
const int32_t step_max = 140;
const int ctrl_num = 1;
const int NS = 8;
const char* mfilename = "../../../model/cart3pole.xml";
static bool TOP_FLAG = false;
const mjtNum K[NS] = { 6.1164,   13.3886,  101.9535,   60.7007,  277.3286,   55.3521 };
mjtNum state_nominal[step_max + 1][NS] = { 0.0, 0.0, 0, 0, 0, 0 };
// top is 0
mjtNum x_goal[NS] = {
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(PI),
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
};
#elif defined(CART2POLE)
const mjtNum t_step = 0.05;
const mjtNum ptb_coef = 2;
const int32_t step_max = 120;
const int ctrl_num = 1;
const int NS = 6;
const char* mfilename = "../../../model/cart2pole.xml";
static bool TOP_FLAG = false;
const mjtNum K[NS] = { 6.1164,   13.3886,  101.9535,   60.7007,  277.3286,   55.3521 };
mjtNum state_nominal[step_max + 1][NS] = { 0.0, 0.0, 0, 0, 0, 0 };
// top is 0
mjtNum x_goal[NS] = {
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(PI),
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(0),
};
#elif defined(ACROBOT)
const mjtNum t_step = 0.01;
const mjtNum ptb_coef = 0.;
const int32_t step_max = 700;
const int ctrl_num = 1;
const int NS = 4;
mjtNum K[NS] = { -428.9630, -158.2817, -108.7071, -45.5430 };
const char* mfilename = "../../../model/acrobot.xml";
static bool TOP_FLAG = false;
mjtNum state_nominal[step_max + 1][NS] = { -PI, 0.0, 0, 0 };
// top is 0
mjtNum x_goal[NS] = {
	(mjtNum)(0),
	(mjtNum)(0),
	(mjtNum)(-2*PI),
	(mjtNum)(0),
};
#elif defined(PENDULUM)
const mjtNum t_step = 0.1;
const mjtNum ptb_coef = 6;
const int32_t step_max = 80;
const int ctrl_num = 1;
const int NS = 2;
const mjtNum K[NS] = { 11.0396, 3.0658 };
const char* mfilename = "../../../model/pendulum.xml";
static bool TOP_FLAG = false;
mjtNum state_nominal[step_max + 1][NS] = { PI, 0.0};
// top is 0
mjtNum x_goal[NS] = {
	(mjtNum)(0),
	(mjtNum)(0),
};
#elif defined(CARTPOLE)
const mjtNum t_step = 0.1;
const mjtNum ptb_coef = 3;
const int32_t step_max = 30;
const int ctrl_num = 1;
const int NS = 4;
const mjtNum K[NS] = { -1.0959, -2.3829, -33.7244, -8.8701 };
const char* mfilename = "../../../model/cartpole.xml";
static bool TOP_FLAG = false;
mjtNum state_nominal[step_max + 1][NS] = { 0, 0, 0, 0.0 };
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
mjModel* m1 = 0;
mjData* d1 = 0;
mjModel* m2 = 0;
mjData* d2 = 0;					
char lastfile[1000] = "";
char error[1000];
static int32_t trial = 0;
static int tri_num = 0;
const int n_check = 100;
const int N = step_max;
const mjtNum slowmotion1 = 1;
const mjtNum slowmotion2 = 0.2; // smaller faster
//mjtNum dx[n_check][step_max + 1][NS];
//mjtNum delta_xc[n_check][step_max][NS + ctrl_num] = { 0 };
//mjtNum delta_xcs[n_check][step_max][NS] = { 0 };
//mjtNum mat[step_max][NS][NS + ctrl_num] = { 0 };
mjtNum u[step_max * ctrl_num] = {0};
mjtNum res[step_max][step_max];
mjtNum delta_u[step_max * ctrl_num] = { 0 };
mjtNum ctrl_nominal[step_max * ctrl_num] = { 0 };
#if !defined(FISH)
mjtNum state_openlp[step_max + 1][NS] = { 0 };
mjtNum state_closedlp[step_max + 1][NS] = { 0 };
mjtNum TK[step_max][ctrl_num][NS] = { 0 };
#endif
char ctrl_buff[30], glstr[30];
char *str, *str3;
FILE *fp, *fp1, *fp2, *fop, *fop1, *fop2;

double gaussrand(void);

// user state
bool paused = false;
bool showoption = false;
bool showinfo = true;
bool showfullscreen = false;
bool slowmotion = false;
bool showdepth = false;
bool showsensor = false;
bool showprofiler = true;
int showhelp = 1;                   // 0: none; 1: brief; 2: full
int fontscale = mjFONTSCALE_150;    // can be 100, 150, 200
int keyreset = -1;                  // non-negative: reset to keyframe

// abstract visualization
mjvScene scn;
mjvCamera cam;
mjvOption vopt;
mjvPerturb pert;
mjvFigure figconstraint;
mjvFigure figcost;
mjvFigure figtimer;
mjvFigure figsize;
mjvFigure figsensor;
char status[1000] = "";

// OpenGL rendering
int refreshrate;
mjrContext con;
float depth_buffer[5120*2880];        // big enough for 5K screen
unsigned char depth_rgb[1280*720*3];  // 1/4th of screen

// selection and perturbation
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
double window2buffer = 1;           // framebuffersize / windowsize (for scaled video modes)

// help strings
const char help_title[] = 
"Help\n"
"Option\n"
"Info\n"
"Depth\n"
"Full screen\n"
"Stereo\n"
"Profiler\n"
"Slow motion\n"
"Key reset\n"
"Pause\n"
"Reset\n"
"Forward\n"
"Back\n"
"Forward 100\n"
"Back 100\n"
"Autoscale\n"
"Reload\n"
"Geoms\n"
"Sites\n"
"Select\n"
"Center\n"
"Track\n"
"Zoom\n"
"Translate\n"
"Rotate\n"
"Perturb\n"
"Free Camera\n"
"Camera\n"
"Frame\n"
"Label\n"
"Fontsize";


const char help_content[] = 
"F1\n"
"F2\n"
"F3\n"
"F4\n"
"F5\n"
"F6\n"
"F7\n"
"Enter\n"
"Page Up/Down\n"
"Space\n"
"BackSpace\n"
"Right arrow\n"
"Left arrow\n"
"Down arrow\n"
"Up arrow\n"
"Ctrl A\n"
"Ctrl L\n"
"0 - 4\n"
"Shift 0 - 4\n"
"L dblclick\n"
"R dblclick\n"
"Ctrl R dblclick\n"
"Scroll or M drag\n"
"[Shift] R drag\n"
"L drag\n"
"Ctrl [Shift] L/R drag\n"
"Esc\n"
"[ ]\n"
"; '\n"
". /\n"
"- =";

char opt_title[1000] = "";
char opt_content[1000];

// deallocate and print message
int finish(const char* msg = 0, mjModel* m = 0, mjData* d = 0)
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

//-------------------------------- profiler and sensor ----------------------------------

// init profiler
void profilerinit(void)
{
    int i, n;

    // set figures to default
    mjv_defaultFigure(&figconstraint);
    mjv_defaultFigure(&figcost);
    mjv_defaultFigure(&figtimer);
    mjv_defaultFigure(&figsize);

    // titles
    strcpy(figconstraint.title, "Control");
    strcpy(figcost.title, "Convergence (log 10)");
    strcpy(figsize.title, "UT*U");
    strcpy(figtimer.title, "Cost&State");

    // x-labels
    strcpy(figconstraint.xlabel, "Video frame");
    strcpy(figcost.xlabel, "Solver iteration");
    strcpy(figsize.xlabel, "Video frame");
    strcpy(figtimer.xlabel, "Video frame");

    // y-tick nubmer formats
    strcpy(figconstraint.yformat, "%2.2f");
    strcpy(figcost.yformat, "%.1f");
    strcpy(figsize.yformat, "%.0f");
    strcpy(figtimer.yformat, "%.0f");

    // colors
    figconstraint.figurergba[0]  = 0.1f;
    figcost.figurergba[2] =  0.2f;
    figsize.figurergba[0] =  0.1f;
    figtimer.figurergba[2] =  0.2f;

    // legends
    strcpy(figconstraint.linename[0], "RTcontrol");
    strcpy(figconstraint.linename[1], "control1");
    strcpy(figconstraint.linename[2], "control2");
    strcpy(figconstraint.linename[3], "control3");
    strcpy(figconstraint.linename[4], "control4");
    strcpy(figcost.linename[0], "improvement");
    strcpy(figcost.linename[1], "gradient");
    strcpy(figcost.linename[2], "lineslope");
    strcpy(figsize.linename[0], "[1 1]");
    strcpy(figsize.linename[1], "[1 15]");
    strcpy(figsize.linename[2], "[10 10]");
    strcpy(figsize.linename[3], "[19 19]");
    strcpy(figsize.linename[4], "[12 29]");
    strcpy(figsize.linename[5], "[2 12]");
    strcpy(figtimer.linename[0], "qpos0");
    strcpy(figtimer.linename[1], "qpos1");
    strcpy(figtimer.linename[2], "qvel0");
    strcpy(figtimer.linename[3], "qvel1");
    strcpy(figtimer.linename[4], "cost");

    // grid sizes
    figconstraint.gridsize[0] = 3;
    figconstraint.gridsize[1] = 5;
    figcost.gridsize[0] = 5;
    figcost.gridsize[1] = 5;
    figsize.gridsize[0] = 3;
    figsize.gridsize[1] = 5;
    figtimer.gridsize[0] = 3;
    figtimer.gridsize[1] = 5;

    // minimum ranges
    figconstraint.range[0][0] = -200;
    figconstraint.range[0][1] = 0;
    figconstraint.range[1][0] = 0;
    figconstraint.range[1][1] = 2.2f;
    figcost.range[0][0] = 0;
    figcost.range[0][1] = 20;
    figcost.range[1][0] = -15;
    figcost.range[1][1] = 5;
    figsize.range[0][0] = -200;
    figsize.range[0][1] = 0;
    figsize.range[1][0] = 0;
    figsize.range[1][1] = .0f;
    figtimer.range[0][0] = -100;
    figtimer.range[0][1] = 0;
    figtimer.range[1][0] = 0;
    figtimer.range[1][1] = 4.0f;

    // init x axis on history figures (do not show yet)
    for( n=0; n<6; n++ )
        for( i=0; i<mjMAXLINEPNT; i++ )
        {
            figtimer.linedata[n][2*i] = (float)-i;
            figsize.linedata[n][2*i] = (float)-i;
			figconstraint.linedata[n][2*i] = (float)-i;
        }
}


// show profiler
void profilerupdate(void)
{
    int i, n;

    //// update constraint figure
    //figconstraint.linepnt[0] = mjMIN(mjMIN(d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
    //for( i=1; i<5; i++ )
    //    figconstraint.linepnt[i] = figconstraint.linepnt[0];
    //if( m->opt.solver==mjSOL_PGS )
    //{
    //    figconstraint.linepnt[3] = 0;
    //    figconstraint.linepnt[4] = 0;
    //}
    //if( m->opt.solver==mjSOL_CG )
    //    figconstraint.linepnt[4] = 0;
    //for( i=0; i<figconstraint.linepnt[0]; i++ )
    //{
        //// x
        /*figconstraint.linedata[0][2*i] = (float)i;
        figconstraint.linedata[1][2*i] = (float)i;
        figconstraint.linedata[2][2*i] = (float)i;
        figconstraint.linedata[3][2*i] = (float)i;
        figconstraint.linedata[4][2*i] = (float)i;*/

        //// y
        /*figconstraint.linedata[0][2*i+1] = (float)d->nefc;
        figconstraint.linedata[1][2*i+1] = (float)d->solver[i].nactive;
        figconstraint.linedata[2][2*i+1] = (float)d->solver[i].nchange;
        figconstraint.linedata[3][2*i+1] = (float)d->solver[i].neval;
        figconstraint.linedata[4][2*i+1] = (float)d->solver[i].nupdate;*/
    //}

	int pntcst = mjMIN(201, figtimer.linepnt[0]+1);
    for( n=0; n<5; n++ )
    {
        // shift data
        for( i=pntcst-1; i>0; i-- )
            figconstraint.linedata[n][2*i+1] = figconstraint.linedata[n][2*i-1];

        // assign new
        figconstraint.linepnt[n] = pntcst;
        figconstraint.linedata[0][1] = (float)d->ctrl[0];
        figconstraint.linedata[1][1] = (float)d->ctrl[0];
        figconstraint.linedata[2][1] = (float)d->ctrl[0];
        figconstraint.linedata[3][1] = (float)d->ctrl[0];
        figconstraint.linedata[4][1] = (float)d->ctrl[0];
    }

    // update cost figure
    figcost.linepnt[0] = mjMIN(mjMIN(d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
    for( i=1; i<3; i++ )
        figcost.linepnt[i] = figcost.linepnt[0];
    if( m->opt.solver==mjSOL_PGS )
    {
        figcost.linepnt[1] = 0;
        figcost.linepnt[2] = 0;
    }

    for( i=0; i<figcost.linepnt[0]; i++ )
    {
        // x
        figcost.linedata[0][2*i] = (float)i;
        figcost.linedata[1][2*i] = (float)i;
        figcost.linedata[2][2*i] = (float)i;

        // y
        figcost.linedata[0][2*i+1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].improvement));
        figcost.linedata[1][2*i+1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].gradient));
        figcost.linedata[2][2*i+1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].lineslope));
    }

    // get timers: total, collision, prepare, solve, other
    int itotal = (d->timer[mjTIMER_STEP].duration > d->timer[mjTIMER_FORWARD].duration ?
                    mjTIMER_STEP : mjTIMER_FORWARD);
 /*   float tdata[5] = { 
        (float)(d->timer[itotal].duration/mjMAX(1,d->timer[itotal].number)),
        (float)(d->timer[mjTIMER_POS_COLLISION].duration/mjMAX(1,d->timer[mjTIMER_POS_COLLISION].number)),
        (float)(d->timer[mjTIMER_POS_MAKE].duration/mjMAX(1,d->timer[mjTIMER_POS_MAKE].number)) +
            (float)(d->timer[mjTIMER_POS_PROJECT].duration/mjMAX(1,d->timer[mjTIMER_POS_PROJECT].number)),
        (float)(d->timer[mjTIMER_CONSTRAINT].duration/mjMAX(1,d->timer[mjTIMER_CONSTRAINT].number)),
        0
    };*/
    //tdata[4] = tdata[0] - tdata[1] - tdata[2] - tdata[3];

	float tdata[5] = { 
        (float)(d->qpos[0]),
        (float)(d->qpos[1]),
        (float)(d->qpos[2]),
        (float)(d->qpos[3]),
        (float)(0),
    };

    // update figtimer
    int pnt = mjMIN(201, figtimer.linepnt[0]+1);
    for( n=0; n<5; n++ )
    {
        // shift data
        for( i=pnt-1; i>0; i-- )
            figtimer.linedata[n][2*i+1] = figtimer.linedata[n][2*i-1];

        // assign new
        figtimer.linepnt[n] = pnt;
        figtimer.linedata[n][1] = tdata[n];
    }

    // get sizes: nv, nbody, nefc, sqrt(nnz), ncont, iter
    float sdata[6] = {
        (float)m->nv,
        (float)m->nbody,
        (float)d->nefc,
        (float)mju_sqrt((mjtNum)d->solver_nnz),
        (float)d->ncon,
        (float)d->solver_iter
    };

    // update figsize
    pnt = mjMIN(201, figsize.linepnt[0]+1);
    for( n=0; n<6; n++ )
    {
        // shift data
        for( i=pnt-1; i>0; i-- )
            figsize.linedata[n][2*i+1] = figsize.linedata[n][2*i-1];

        // assign new
        figsize.linepnt[n] = pnt;
        figsize.linedata[n][1] = sdata[n];
    }
}



// show profiler
void profilershow(mjrRect rect)
{
    mjrRect viewport = {rect.width - rect.width/5, rect.bottom, rect.width/5, rect.height/4};
    mjr_figure(viewport, &figtimer, &con);
    viewport.bottom += rect.height/4;
    mjr_figure(viewport, &figsize, &con);
    viewport.bottom += rect.height/4;
    mjr_figure(viewport, &figcost, &con);
    viewport.bottom += rect.height/4;
    mjr_figure(viewport, &figconstraint, &con);
}



// init sensor figure
void sensorinit(void)
{
    // set figure to default
    mjv_defaultFigure(&figsensor);

    // set flags
    figsensor.flg_extend = 1;
    figsensor.flg_barplot = 1;

    // title
    strcpy(figsensor.title, "Sensor data");

    // y-tick nubmer format
    strcpy(figsensor.yformat, "%.0f");

    // grid size
    figsensor.gridsize[0] = 2;
    figsensor.gridsize[1] = 3;

    // minimum range
    figsensor.range[0][0] = 0;
    figsensor.range[0][1] = 0;
    figsensor.range[1][0] = -1;
    figsensor.range[1][1] = 1;
}


// update sensor figure
void sensorupdate(void)
{
    static const int maxline = 10;

    // clear linepnt
    for( int i=0; i<maxline; i++ )
        figsensor.linepnt[i] = 0;

    // start with line 0
    int lineid = 0;

    // loop over sensors
    for( int n=0; n<m->nsensor; n++ )
    {
        // go to next line if type is different
        if( n>0 && m->sensor_type[n]!=m->sensor_type[n-1] )
            lineid = mjMIN(lineid+1, maxline-1);

        // get info about this sensor
        mjtNum cutoff = (m->sensor_cutoff[n]>0 ? m->sensor_cutoff[n] : 1);
        int adr = m->sensor_adr[n];
        int dim = m->sensor_dim[n];

        // data pointer in line
        int p = figsensor.linepnt[lineid];

        // fill in data for this sensor
        for( int i=0; i<dim; i++ )
        {
            // check size
            if( (p+2*i)>=mjMAXLINEPNT/2 )
                break;

            // x
            figsensor.linedata[lineid][2*p+4*i] = (float)(adr+i);
            figsensor.linedata[lineid][2*p+4*i+2] = (float)(adr+i);

            // y
            figsensor.linedata[lineid][2*p+4*i+1] = 0;
            figsensor.linedata[lineid][2*p+4*i+3] = (float)(d->sensordata[adr+i]/cutoff);
        }

        // update linepnt
        figsensor.linepnt[lineid] = mjMIN(mjMAXLINEPNT-1, 
                                          figsensor.linepnt[lineid]+2*dim);
    }
}



// show sensor figure
void sensorshow(mjrRect rect)
{
    // render figure on the right
    mjrRect viewport = {rect.width - rect.width/4, rect.bottom, rect.width/4, rect.height/3};
    mjr_figure(viewport, &figsensor, &con);
}

//-------------------------------- utility functions ------------------------------------

// center and scale view
void autoscale(GLFWwindow* window)
{
    // autoscale
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;

    // set to free camera
    cam.type = mjCAMERA_FREE;
}


// load mjb or xml model
void loadmodel(GLFWwindow* window, const char* filename)
{
    // make sure filename is given
    if( !filename  )
        return;

    // load and compile
    char error[1000] = "could not load binary model";
    mjModel* mnew = 0;                 
    if( strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb") )
        mnew = mj_loadModel(filename, 0);
    else
        mnew = mj_loadXML(filename, 0, error, 1000);
    if( !mnew )
    {
        printf("%s\n", error);
        return;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);
    mj_forward(m, d);

    // save filename for reload
    strcpy(lastfile, filename);

    // re-create custom context
    mjr_makeContext(m, &con, fontscale);

    // clear perturbation state and keyreset
    pert.active = 0;
    pert.select = 0;
    keyreset = -1;

    // center and scale view, update scene
    autoscale(window);
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    //// set window title to mode name
    //if( window && m->names )
    //    glfwSetWindowTitle(window, m->names);
}

void loadmodel1(GLFWwindow* window, const char* filename)
{
	// make sure filename is given
	if (!filename)
		return;

	// load and compile
	char error[1000] = "could not load binary model";
	mjModel* mnew = 0;
	if (strlen(filename)>4 && !strcmp(filename + strlen(filename) - 4, ".mjb"))
		mnew = mj_loadModel(filename, 0);
	else
		mnew = mj_loadXML(filename, 0, error, 1000);
	if (!mnew)
	{
		printf("%s\n", error);
		return;
	}

	// delete old model, assign new
	mj_deleteData(d1);
	mj_deleteModel(m1);
	m1 = mnew;
	d1 = mj_makeData(m1);
	mj_forward(m1, d1);

	// save filename for reload
	strcpy(lastfile, filename);

	// re-create custom context
	mjr_makeContext(m1, &con, fontscale);

	// clear perturbation state and keyreset
	pert.active = 0;
	pert.select = 0;
	keyreset = -1;

	// center and scale view, update scene
	autoscale(window);
	mjv_updateScene(m1, d1, &vopt, &pert, &cam, mjCAT_ALL, &scn);

	//// set window title to mode name
	//if (window && m1->names)
	//	glfwSetWindowTitle(window, m1->names);
}

void loadmodel2(GLFWwindow* window, const char* filename)
{
	// make sure filename is given
	if (!filename)
		return;

	// load and compile
	char error[1000] = "could not load binary model";
	mjModel* mnew = 0;
	if (strlen(filename)>4 && !strcmp(filename + strlen(filename) - 4, ".mjb"))
		mnew = mj_loadModel(filename, 0);
	else
		mnew = mj_loadXML(filename, 0, error, 1000);
	if (!mnew)
	{
		printf("%s\n", error);
		return;
	}

	// delete old model, assign new
	mj_deleteData(d2);
	mj_deleteModel(m2);
	m2 = mnew;
	d2 = mj_makeData(m2);
	mj_forward(m2, d2);

	// save filename for reload
	strcpy(lastfile, filename);

	// re-create custom context
	mjr_makeContext(m2, &con, fontscale);

	// clear perturbation state and keyreset
	pert.active = 0;
	pert.select = 0;
	keyreset = -1;

	// center and scale view, update scene
	autoscale(window);
	mjv_updateScene(m2, d2, &vopt, &pert, &cam, mjCAT_ALL, &scn);

	//// set window title to mode name
	//if (window && m2->names)
	//	glfwSetWindowTitle(window, m2->names);
}

// timer in milliseconds
mjtNum timer(void)
{
    // save start time
    static double starttm = 0;
    if( starttm==0 )
        starttm = glfwGetTime();

    // return time since start
    return (mjtNum)(1000 * (glfwGetTime() - starttm));
}


// clear all times
void cleartimers(mjData* d)
{
    for( int i=0; i<mjNTIMER; i++ )
    {
        d->timer[i].duration = 0;
        d->timer[i].number = 0;
    }
}

//--------------------------------- GLFW callbacks --------------------------------------

// keyboard
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    int n;

    // require model
    if( !m )
        return;

    // do not act on release
    if( act==GLFW_RELEASE )
        return;

    switch( key )
    {
    case GLFW_KEY_F1:                   // help
        showhelp++;
        if( showhelp>2 )
            showhelp = 0;
        break;

    case GLFW_KEY_F2:                   // option
        showoption = !showoption;
        break;

    case GLFW_KEY_F3:                   // info
        showinfo = !showinfo;
        break;

    case GLFW_KEY_F4:                   // depth
        showdepth = !showdepth;
        break;

    case GLFW_KEY_F5:                   // toggle full screen
        showfullscreen = !showfullscreen;
        if( showfullscreen )
            glfwMaximizeWindow(window);
        else
            glfwRestoreWindow(window);
        break;

    case GLFW_KEY_F6:                   // stereo
        scn.stereo = (scn.stereo==mjSTEREO_NONE ? mjSTEREO_QUADBUFFERED : mjSTEREO_NONE);
        break;

    case GLFW_KEY_F7:                   // sensor figure
        showsensor = !showsensor;
        break;

    case GLFW_KEY_F8:                   // profiler
        showprofiler = !showprofiler;
        break;

    case GLFW_KEY_ENTER:                // slow motion
        slowmotion = !slowmotion;
        break;

    case GLFW_KEY_SPACE:                // pause
        paused = !paused;
        break;

    case GLFW_KEY_PAGE_UP:              // previous keyreset
    case GLFW_KEY_PAGE_DOWN:            // next keyreset
        if( key==GLFW_KEY_PAGE_UP )
            keyreset = mjMAX(-1, keyreset-1);
        else
            keyreset = mjMIN(m->nkey-1, keyreset+1);

        // continue with reset

    case GLFW_KEY_BACKSPACE:            // reset
        mj_resetData(m, d);
        if( keyreset>=0 && keyreset<m->nkey )
        {
            d->time = m->key_time[keyreset];
            mju_copy(d->qpos, m->key_qpos+keyreset*m->nq, m->nq);
            mju_copy(d->qvel, m->key_qvel+keyreset*m->nv, m->nv);
            mju_copy(d->act, m->key_act+keyreset*m->na, m->na);
        }
        mj_forward(m, d);
        profilerupdate();
        sensorupdate();
        break;

    case GLFW_KEY_RIGHT:                // step forward
        if( paused )
        {
            mj_step(m, d);
            profilerupdate();
            sensorupdate();
        }
        break;

    case GLFW_KEY_LEFT:                 // step back
        if( paused )
        {
            m->opt.timestep = -m->opt.timestep;
            cleartimers(d);
            mj_step(m, d);
            m->opt.timestep = -m->opt.timestep;
            profilerupdate();
            sensorupdate();
        }
        break;

    case GLFW_KEY_DOWN:                 // step forward 100
        if( paused )
        {
            cleartimers(d);
            for( n=0; n<100; n++ )
                mj_step(m,d);
            profilerupdate();
            sensorupdate();
        }
        break;

    case GLFW_KEY_UP:                   // step back 100
        if( paused )
        {
            m->opt.timestep = -m->opt.timestep;
            cleartimers(d);
            for( n=0; n<100; n++ )
                mj_step(m,d);
            m->opt.timestep = -m->opt.timestep;
            profilerupdate();
            sensorupdate();
        }
        break;

    case GLFW_KEY_ESCAPE:               // free camera
        cam.type = mjCAMERA_FREE;
        break;

    case '=':                           // bigger font
        if( fontscale<200 )
        {
            fontscale += 50;
            mjr_makeContext(m, &con, fontscale);
        }
        break;

    case '-':                           // smaller font
        if( fontscale>100 )
        {
            fontscale -= 50;
            mjr_makeContext(m, &con, fontscale);
        }
        break;

    case '[':                           // previous fixed camera or free
        if( m->ncam && cam.type==mjCAMERA_FIXED )
        {
            if( cam.fixedcamid>0 )
                cam.fixedcamid--;
            else
                cam.type = mjCAMERA_FREE;
        }
        break;

    case ']':                           // next fixed camera
        if( m->ncam )
        {
            if( cam.type!=mjCAMERA_FIXED )
            {
                cam.type = mjCAMERA_FIXED;
                cam.fixedcamid = 0;
            }
            else if( cam.fixedcamid<m->ncam-1 )
                cam.fixedcamid++;
        }
        break;

    case ';':                           // cycle over frame rendering modes
        vopt.frame = mjMAX(0, vopt.frame-1);
        break;

    case '\'':                          // cycle over frame rendering modes
        vopt.frame = mjMIN(mjNFRAME-1, vopt.frame+1);
        break;

    case '.':                           // cycle over label rendering modes
        vopt.label = mjMAX(0, vopt.label-1);
        break;

    case '/':                           // cycle over label rendering modes
        vopt.label = mjMIN(mjNLABEL-1, vopt.label+1);
        break;

    default:                            // toggle flag
        // control keys
        if( mods & GLFW_MOD_CONTROL )
        {
            if( key==GLFW_KEY_A )
                autoscale(window);
            else if( key==GLFW_KEY_L && lastfile[0] )
                loadmodel(window, lastfile);
			
            break;
        }

        // toggle visualization flag
        for( int i=0; i<mjNVISFLAG; i++ )
            if( key==mjVISSTRING[i][2][0] )
                vopt.flags[i] = !vopt.flags[i];

        // toggle rendering flag
        for( int i=0; i<mjNRNDFLAG; i++ )
            if( key==mjRNDSTRING[i][2][0] )
                scn.flags[i] = !scn.flags[i];

        // toggle geom/site group
        for( int i=0; i<mjNGROUP; i++ )
            if( key==i+'0')
            {
                if( mods & GLFW_MOD_SHIFT )
                    vopt.sitegroup[i] = !vopt.sitegroup[i];
                else
                    vopt.geomgroup[i] = !vopt.geomgroup[i];
            }
    }
}


// mouse button
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // past data for double-click detection
    static int lastbutton = 0;
    static double lastclicktm = 0;

    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // Alt: swap left and right
    if( (mods & GLFW_MOD_ALT) )
    {
        bool tmp = button_left;
        button_left = button_right;
        button_right = tmp;

        if( button==GLFW_MOUSE_BUTTON_LEFT )
            button = GLFW_MOUSE_BUTTON_RIGHT;
        else if( button==GLFW_MOUSE_BUTTON_RIGHT )
            button = GLFW_MOUSE_BUTTON_LEFT;
    }

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);

    // require model
    if( !m )
        return;

    // set perturbation
    int newperturb = 0;
    if( act==GLFW_PRESS && (mods & GLFW_MOD_CONTROL) && pert.select>0 )
    {
        // right: translate;  left: rotate
        if( button_right )
            newperturb = mjPERT_TRANSLATE;
        else if( button_left )
            newperturb = mjPERT_ROTATE;

        // perturbation onset: reset reference
        if( newperturb && !pert.active )
            mjv_initPerturb(m, d, &scn, &pert);
    }
    pert.active = newperturb;

    // detect double-click (250 msec)
    if( act==GLFW_PRESS && glfwGetTime()-lastclicktm<0.25 && button==lastbutton )
    {
        // determine selection mode
        int selmode;
        if( button==GLFW_MOUSE_BUTTON_LEFT )
            selmode = 1;
        else if( mods & GLFW_MOD_CONTROL )
            selmode = 3;
        else
            selmode = 2;

        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        // find geom and 3D click point, get corresponding body
        mjtNum selpnt[3];
        int selgeom = mjv_select(m, d, &vopt,
                                 (mjtNum)width/(mjtNum)height, 
                                 (mjtNum)lastx/(mjtNum)width, 
                                 (mjtNum)(height-lasty)/(mjtNum)height, 
                                 &scn, selpnt);
        int selbody = (selgeom>=0 ? m->geom_bodyid[selgeom] : 0);

        // set lookat point, start tracking is requested
        if( selmode==2 || selmode==3 )
        {
            // copy selpnt if geom clicked
            if( selgeom>=0 )
                mju_copy3(cam.lookat, selpnt);

            // switch to tracking camera
            if( selmode==3 && selbody )
            {
                cam.type = mjCAMERA_TRACKING;
                cam.trackbodyid = selbody;
                cam.fixedcamid = -1;
            }
        }

        // set body selection
        else
        {
            if( selbody )
            {
                // record selection
                pert.select = selbody;

                // compute localpos
                mjtNum tmp[3];
                mju_sub3(tmp, selpnt, d->xpos+3*pert.select);
                mju_mulMatTVec(pert.localpos, d->xmat+9*pert.select, tmp, 3, 3);
            }
            else
                pert.select = 0;
        }

        // stop perturbation on select
        pert.active = 0;
    }

    // save info
    if( act==GLFW_PRESS )
    {
        lastbutton = button;
        lastclicktm = glfwGetTime();
    }
}


// mouse move
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // require model
    if( !m )
        return;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move perturb or camera
    if( pert.active )
        mjv_movePerturb(m, d, action, dx/height, dy/height, &scn, &pert);
    else
        mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // require model
    if( !m )
        return;

    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// drop
void drop(GLFWwindow* window, int count, const char** paths)
{
    // make sure list is non-empty
    if( count>0 )
        loadmodel(window, paths[0]);
}


//-------------------------------- simulation and rendering -----------------------------

// make option string
void makeoptionstring(const char* name, char key, char* buf)
{
    int i=0, cnt=0;

    // copy non-& characters
    while( name[i] && i<50 )
    {
        if( name[i]!='&' )
            buf[cnt++] = name[i];

        i++;
    }

    // finish
    buf[cnt] = ' ';
    buf[cnt+1] = '(';
    buf[cnt+2] = key;
    buf[cnt+3] = ')';
    buf[cnt+4] = 0;
}

/*For calculating Determinant of the Matrix */
mjtNum determinant(mjtNum fh[N][N], mjtNum r)
{
	int z,j,i;
	mjtNum m=1,k,a[N][N],det = 1.0;

	for (i = 0;i < r; i++)
    {
     for (j = 0;j < r; j++)
       {
		a[i][j] = fh[i][j];
        }
    }

	for(z=0;z<r-1;z++)
	for(i=z;i<r-1;i++)
	{
		if(a[z][z]==0)
		{
			for(j=0;j<r;j++)
			{ 
				a[z][j]=a[z][j]+a[i+1][j];
			}
		}
		if(a[z][z]!=0){
			k=-a[i+1][z]/a[z][z];
			for(j=z;j<r;j++)a[i+1][j]=k*a[z][j]+a[i+1][j];
		}
	}
	for(z=0;z<r;z++)
	{
		det=det*(a[z][z]);
	}
	return (det);
}

/* Finding transpose of matrix */ 
void transpose(mjtNum num[N][N], mjtNum fac[N][N], mjtNum r)
{
	int i, j;
	mjtNum b[N][N], d;
 
	for (i = 0;i < r; i++)
	{
		for (j = 0;j < r; j++)
		{
			b[i][j] = fac[j][i];
		}
	}
	d = determinant(num, r);
	for (i = 0;i < r; i++)
	{
		for (j = 0;j < r; j++)
		{
			res[i][j] = b[i][j] / d;
		}
	}
}
 
void cofactor(mjtNum num[N][N], mjtNum f)
{
	mjtNum b[N][N]={0}, fac[N][N];
	int p, q, m, n, i, j;
	for (q = 0;q < f; q++)
	{
		for (p = 0;p < f; p++)
		{
			m = 0;
			n = 0;
			for (i = 0;i < f; i++)
			{
				for (j = 0;j < f; j++)
				{
					if (i != q && j != p)
					{
						b[m][n] = num[i][j];
						if (n < (f - 2)) n++;
						else{
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

    if(phase == 0)
    {
         U = rand() / (RAND_MAX + 1.0);
         V = rand() / (RAND_MAX + 1.0);
		 if(U == 0) U = 0.00001;
         Z = sqrt(-2.0 * log(U)) * sin(2.0 * PI * V);
    }
    else
    {
         Z = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);
    }
    phase = 1 - phase;
    return Z;
}

int get_nominal(mjModel* m, mjData* d)
{
	static int index = 0;
	static mjtNum t_init = -0.2;

	if (d->time - t_init < t_step - 0.00001)
	{
		mj_step(m, d);
	}
	else {
		if (t_init < -0.1)
		{
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
				d->ctrl[ci] = ctrl_nominal[0 * ctrl_num + ci];
			}
		}else {
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
				return 1;
			}
			else {
				for (int ci = 0; ci < ctrl_num; ci++)
				{
					d->ctrl[ci] = ctrl_nominal[index * ctrl_num + ci];
				}
			}
		}
		t_init = d->time;
	}
	return 0;
}

void sysidcheckfish(mjModel* m, mjData* d)
{
	static int index = 0;
	static mjtNum t_init = -0.2;
	static mjtNum dx[n_check][step_max + 1][NS-1];
	static mjtNum delta_xc[n_check][step_max][NS + ctrl_num-1] = { 0 };
	static mjtNum delta_xcs[n_check][step_max][NS-1] = { 0 };
	static mjtNum mat[step_max][NS-1][NS + ctrl_num-1] = { 0 };
	
	if ((fop1 = fopen("../../../data/linearization.txt", "r")) != NULL)
	{
		for (int i = 0; i < step_max; i++)
		{
			for (int j = 0; j < NS - 1; j++)
			{
				for (int k = 0; k < NS + ctrl_num - 1; k++)
				{
					fscanf(fop1, "%s", ctrl_buff);
					mat[i][j][k] = atof(ctrl_buff);
				}
			}
		}
		fclose(fop1);
	}
	
	for (int h = 0; h < step_max; h++)
	{
		for (int u = 0; u < NS + ctrl_num - 1; u++)
		{
			for (int c = 0; c < n_check; c++) delta_xc[c][h][u] = ptb_coef * gaussrand();
		}
	}
	for (int i = 0; i < n_check; i++)
	{
		for (int j = 0; j < step_max; j++) mju_mulMatVec(dx[i][j + 1], *mat[j], delta_xc[i][j], NS - 1, NS - 1 + ctrl_num);
	}

	for (int i = 0; i < n_check; i++)
	{
		index = 0;
		t_init = d->time;
		for (int y = 0; y < m->nq; y++)
		{
			if (y < 3) d->qpos[y] = delta_xc[i][index][2 * y] + state_nominal[index][2 * y];
			else if (y > 4) d->qpos[y] = delta_xc[i][index][2 * (y - 2)] + state_nominal[index][2 * y];
		}
		d->qpos[3] = state_nominal[index][6];
		d->qpos[4] = sqrt(1 - d->qpos[3] * d->qpos[3] - d->qpos[6] * d->qpos[6] - d->qpos[5] * d->qpos[5]);
		for (int y = 0; y < m->nv - 1; y++)
		{
			d->qvel[y] = delta_xc[i][index][2 * y + 1] + state_nominal[index][2 * y + 1];
		}
		d->qvel[12] = delta_xc[i][index][24] + state_nominal[index][25];
		for (int ci = 0; ci < ctrl_num; ci++)
		{
			d->ctrl[ci] = delta_xc[i][index][ci + NS - 1] + ctrl_nominal[index * ctrl_num + ci];
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
					if (y < 3)  delta_xcs[i][index][2 * y] = d->qpos[y] - state_nominal[index + 1][2 * y];
					else if (y > 4) delta_xcs[i][index][2 * (y - 2)] = d->qpos[y] - state_nominal[index + 1][2 * y];
				}
				for (int y = 0; y < m->nv - 1; y++)
				{
					delta_xcs[i][index][2 * y + 1] = d->qvel[y] - state_nominal[index + 1][2 * y + 1];
				}
				delta_xcs[i][index][24] = d->qvel[12] - state_nominal[index + 1][25];

				index++;
				t_init = d->time;

				for (int y = 0; y < m->nq; y++)
				{
					if (y < 3) d->qpos[y] = delta_xc[i][index][2 * y] + state_nominal[index][2 * y];
					else if (y > 4) d->qpos[y] = delta_xc[i][index][2 * (y - 2)] + state_nominal[index][2 * y];
				}
				d->qpos[3] = state_nominal[index][6];
				d->qpos[4] = sqrt(1 - d->qpos[3] * d->qpos[3] - d->qpos[6] * d->qpos[6] - d->qpos[5] * d->qpos[5]);
				for (int y = 0; y < m->nv - 1; y++)
				{
					d->qvel[y] = delta_xc[i][index][2 * y + 1] + state_nominal[index][2 * y + 1];
				}
				d->qvel[12] = delta_xc[i][index][24] + state_nominal[index][25];
				for (int ci = 0; ci < ctrl_num; ci++)
				{
					d->ctrl[ci] = delta_xc[i][index][ci + NS - 1] + ctrl_nominal[index * ctrl_num + ci];
				}
			}
		}
	}

	// for result checking
	if ((fp2 = fopen("../../../data/sysidcheck.txt", "wt+")) != NULL)
	{
		str = glstr;
		for (int t = 0; t < NS-1; t++)
		{
			for (int c = 0; c < n_check; c++)
			{
				for (int h = 0; h < step_max; h++)
				{
					sprintf(str, "%2.4f", delta_xcs[c][h][t]);
					fwrite(str, 6, 1, fp2);
					fputs(" ", fp2);
				}
			}
			fputs("\n", fp2);
			for (int c = 0; c < n_check; c++)
			{
				for (int d = 1; d <= step_max; d++)
				{
					sprintf(str, "%2.4f", dx[c][d][t]);
					fwrite(str, 6, 1, fp2);
					fputs(" ", fp2);
				}
			}
			fputs("\n", fp2);
			fputs("\n", fp2);
		}
		fclose(fp2);
	}
}

void sysidcheck(mjModel* m, mjData* d)
{
	static int index = 0;
	static mjtNum t_init = -0.2;
	mjtNum dx[n_check][step_max + 1][NS];
	mjtNum delta_xc[n_check][step_max][NS + ctrl_num] = { 0 };
	mjtNum delta_xcs[n_check][step_max][NS] = { 0 };
	mjtNum mat[step_max][NS][NS + ctrl_num] = { 0 };

	if ((fop1 = fopen("../../../data/linearization.txt", "r")) != NULL)
	{
		for (int i = 0; i < step_max; i++)
		{
			for (int j = 0; j < NS; j++)
			{
				for (int k = 0; k < NS + ctrl_num; k++)
				{
					fscanf(fop1, "%s", ctrl_buff);
					mat[i][j][k] = atof(ctrl_buff);
				}
			}
		}
		fclose(fop1);
	}

	for (int h = 0; h < step_max; h++)
	{
		for (int u = 0; u < NS + ctrl_num; u++)
		{
			for (int c = 0; c < n_check; c++) delta_xc[c][h][u] = ptb_coef * gaussrand();
		}
	}
	for (int i = 0; i < n_check; i++)
	{
		for (int j = 0; j < step_max; j++) mju_mulMatVec(dx[i][j + 1], *mat[j], delta_xc[i][j], NS, NS + ctrl_num);
	}

	for (int i = 0; i < n_check; i++)
	{
		index = 0;
		t_init = d->time;
		for (int y = 0; y < m->nq; y++)
		{
			d->qpos[y] = delta_xc[i][index][2 * y] + state_nominal[index][2 * y];
		}
		for (int y = 0; y < m->nv; y++)
		{
			d->qvel[y] = delta_xc[i][index][2 * y + 1] + state_nominal[index][2 * y + 1];
		}
		for (int ci = 0; ci < ctrl_num; ci++)
		{
			d->ctrl[ci] = delta_xc[i][index][ci + NS] + ctrl_nominal[index * ctrl_num + ci];
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
					delta_xcs[i][index][2 * y] = d->qpos[y] - state_nominal[index + 1][2 * y];
				}
				for (int y = 0; y < m->nv; y++)
				{
					delta_xcs[i][index][2 * y + 1] = d->qvel[y] - state_nominal[index + 1][2 * y + 1];
				}

				index++;
				t_init = d->time;

				for (int y = 0; y < m->nq; y++)
				{
					d->qpos[y] = delta_xc[i][index][2 * y] + state_nominal[index][2 * y];
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
	}

	// for result checking
	if ((fp2 = fopen("../../../data/sysidcheck.txt", "wt+")) != NULL)
	{
		str = glstr;
		for (int t = 0; t < NS; t++)
		{
			for (int c = 0; c < n_check; c++)
			{
				for (int h = 0; h < step_max; h++)
				{
					sprintf(str, "%2.4f", delta_xcs[c][h][t]);
					fwrite(str, 6, 1, fp2);
					fputs(" ", fp2);
				}
			}
			fputs("\n", fp2);
			for (int c = 0; c < n_check; c++)
			{
				for (int d = 1; d <= step_max; d++)
				{
					sprintf(str, "%2.4f", dx[c][d][t]);
					fwrite(str, 6, 1, fp2);
					fputs(" ", fp2);
				}
			}
			fputs("\n", fp2);
			fputs("\n", fp2);
		}
		fclose(fp2);
	}
}

int setcontrol(mjModel* m, mjData* d)
{
	static mjtNum res0[ctrl_num] = { 0 }, res1[NS];
	static mjtNum state[NS+1];
	static mjtNum rtctrl[ctrl_num] = { 0 };
	static int index = 0;
	static mjtNum t_init = -0.2;

	if (TOP_FLAG == true)
	{
		if (d->time - t_init < t_step - 0.000001)
		{
			mj_step(m, d);
		}
		else {
			if (t_init < -0.1 && t_init > -0.4)
			{
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
			}
			for (int y = 0; y < m->nq; y++)
			{
				state[2 * y] = d->qpos[y] - x_goal[2 * y];
			}
			for (int y = 0; y < m->nv; y++)
			{
				state[2 * y + 1] = d->qvel[y] - x_goal[2 * y + 1];
			}

			mju_mulMatVec(res0, K, state, ctrl_num, NS);
			for (int ci = 0; ci < ctrl_num; ci++)
			{
				d->ctrl[ci] = -res0[ci];
			}
			t_init = d->time;
		}
	}else {
		if (d->time - t_init < t_step - 0.000001)
		{
			mj_step(m, d);
		}
		else {
			if (t_init < -0.1)
			{
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
					d->ctrl[ci] = ctrl_nominal[0 * ctrl_num + ci];// +delta_u[0 * ctrl_num + ci];
				}
			}
			else {
				index++;
				if (index >= step_max) {
					mju_zero(d->ctrl, m->nu);
					t_init = -0.5;
					TOP_FLAG = true;
					if (ERRCOMPARE == true)
					{
						TOP_FLAG = false;
						index = 0;
					}
					return 1;
				}
				else {
					#if defined(FISH)
					for (int y = 0; y < m->nq; y++)
					{
						state_closedlp[index][2 * y] = d->qpos[y];
						if (y < 3) {
							state[2 * y] = d->qpos[y] - state_nominal[index][2 * y];
						}
						else if (y > 4) {
							state[2 * (y - 2)] = d->qpos[y] - state_nominal[index][2 * y];
						}
					}
					for (int y = 0; y < m->nv - 1; y++)
					{
						state[2 * y + 1] = d->qvel[y] - state_nominal[index][2 * y + 1];
						state_closedlp[index][2 * y + 1] = d->qvel[y];
					}
					state[24] = d->qvel[12] - state_nominal[index][25];
					state_closedlp[index][25] = d->qvel[12];
					mju_mulMatVec(res0, *TK[index - 1], state, ctrl_num, NS - 1);
					#else 
					for (int y = 0; y < m->nq; y++)
					{
						state[2 * y] = d->qpos[y] - state_nominal[index][2 * y];
						state_closedlp[index][2 * y] = d->qpos[y];
					}
					for (int y = 0; y < m->nv; y++)
					{
						state[2 * y + 1] = d->qvel[y] - state_nominal[index][2 * y + 1];
						state_closedlp[index][2 * y + 1] = d->qvel[y];
					}
					//state[2] = 0; state[4] = 0; state[6] = 0; state[8] = 0; state[10] = 0; state[12] = 0; state[14] = 0; state[16] = 0;
					mju_mulMatVec(res0, *TK[index - 1], state, ctrl_num, NS);
					#endif
					for (int ci = 0; ci < ctrl_num; ci++)
					{
						d->ctrl[ci] = ctrl_nominal[index * ctrl_num + ci] - res0[ci];// +delta_u[index * ctrl_num + ci];
					}
				}
			}
			t_init = d->time;
		}
	}
	return 0;
}

int setcontrol1(mjModel* m, mjData* d)
{
	static mjtNum res0[NS], res1[NS];
	static mjtNum state[NS+1];
	static mjtNum rtctrl[ctrl_num] = { 0 };
	static mjtNum t_init = -0.2;
	static int index = 0;

    if (TOP_FLAG == true)
	{
		if (d->time - t_init < t_step - 0.000001)
		{
			mj_step(m, d);
		}
		else {
			if (t_init < -0.1 && t_init > -0.4)
			{
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
			}
			for (int y = 0; y < m->nq; y++)
			{
				state[2 * y] = d->qpos[y] - x_goal[2 * y];
			}
			for (int y = 0; y < m->nv; y++)
			{
				state[2 * y + 1] = d->qvel[y] - x_goal[2 * y + 1];
			}
			mju_mulMatVec(res0, K, state, ctrl_num, NS);
			for (int ci = 0; ci < ctrl_num; ci++)
			{
				d->ctrl[ci] = -res0[ci];
			}
			t_init = d->time;
		}
	}
	else {
		if (d->time - t_init < t_step - 0.000001)
		{
			mj_step(m, d);
		}
		else {
			if (t_init < -0.1)
			{
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
					d->ctrl[ci] = ctrl_nominal[0 * ctrl_num + ci] +delta_u[0 * ctrl_num + ci];
				}
			}
			else {
				index++;
				if (index >= step_max) {
					mju_zero(d->ctrl, m->nu);
					t_init = -0.5;
					TOP_FLAG = true;
					if (ERRCOMPARE == true)
					{
						TOP_FLAG = false;
						index = 0;
					}
					return 1;
				}
				else {
					for (int y = 0; y < m->nq; y++)
					{
						state_openlp[index][2 * y] = d->qpos[y];
					}
					for (int y = 0; y < m->nv; y++)
					{
						state_openlp[index][2 * y + 1] = d->qvel[y];
					}
					for (int ci = 0; ci < ctrl_num; ci++)
					{
						d->ctrl[ci] = ctrl_nominal[index * ctrl_num + ci] + delta_u[index * ctrl_num + ci];
					}
				}
			}
			t_init = d->time;
		}
	}
	return 0;
}

int setcontrol2(mjModel* m, mjData* d)
{
	static mjtNum res0[NS], res1[NS];
	static mjtNum state[NS+1];
	static mjtNum rtctrl[ctrl_num] = { 0 };
	static mjtNum t_init = -0.2;
	static int index = 0;

	if (TOP_FLAG == true)
	{
		if (d->time - t_init < t_step - 0.000001)
		{
			mj_step(m, d);
		}
		else {
			if (t_init < -0.1 && t_init > -0.4)
			{
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
			}
			for (int y = 0; y < m->nq; y++)
			{
				state[2 * y] = d->qpos[y] - x_goal[2 * y];
			}
			for (int y = 0; y < m->nv; y++)
			{
				state[2 * y + 1] = d->qvel[y] - x_goal[2 * y + 1];
			}
			mju_mulMatVec(res0, K, state, ctrl_num, NS);
			for (int ci = 0; ci < ctrl_num; ci++)
			{
				d->ctrl[ci] = -res0[ci];
			}
			t_init = d->time;
		}
	}
	else {
		if (d->time - t_init < t_step - 0.000001)
		{
			mj_step(m, d);
		}
		else {
			if (t_init < -0.1)
			{
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
					d->ctrl[ci] = ctrl_nominal[0 * ctrl_num + ci];
				}
			}
			else {
				index++;
				if (index >= step_max) {
					mju_zero(d->ctrl, m->nu);
					t_init = -0.5;
					TOP_FLAG = true;
					return 1;
				}
				else {
					for (int ci = 0; ci < ctrl_num; ci++)
					{
						d->ctrl[ci] = ctrl_nominal[index * ctrl_num + ci];
					}
				}
			}
			t_init = d->time;
		}
	}
	return 0;
}


void errcompare()
{
	for (int i = 0; i < n_check; i++)
	{
		while (setcontrol(m, d) != 1);
		while (setcontrol1(m1, d1) != 1);

		// option: output data for checking
		if ((fp1 = fopen("../../../data/display.txt", "at+")) != NULL)
		{
			str3 = glstr;
			for (int h = 0; h < NS; h++)
			{
				for (int d = 1; d < step_max; d++)
				{
					sprintf(str3, "%4.8f", state_nominal[d][h]);
					fwrite(str3, 10, 1, fp1);
					fputs(" ", fp1);
				}
				fputs("\n", fp1);
				for (int d = 1; d < step_max; d++)
				{
					sprintf(str3, "%4.8f", state_closedlp[d][h]);
					fwrite(str3, 10, 1, fp1);
					fputs(" ", fp1);
				}
				fputs("\n", fp1);
				for (int d = 1; d < step_max; d++)
				{
					sprintf(str3, "%4.8f", state_openlp[d][h]);
					fwrite(str3, 10, 1, fp1);
					fputs(" ", fp1);
				}
				fputs("\n", fp1);
				fputs("\n", fp1);
			}
			fclose(fp1);
		}
	}
}

// advance simulation
void simulation(void)
{
    // no model
    if( !m )
        return;

    // clear timers
    cleartimers(d);

    // paused
    if( paused )
    {
        // apply pose perturbations, run mj_forward
        if( pert.active )
        {
            mjv_applyPerturbPose(m, d, &pert, 1);      // move mocap and dynamic bodies
            mj_forward(m, d);
        }
    }

    // running
    else
    {
        // slow motion factor: 10x
        mjtNum factor = (slowmotion ? slowmotion1 : slowmotion2); // smaller faster 0.04:1 slow motion 1:10

        // advance effective simulation time by 1/refreshrate
        mjtNum startsimtm = d->time;
        while( (d->time-startsimtm)*factor<1.0/refreshrate )
        {
            // clear old perturbations, apply new
            mju_zero(d->xfrc_applied, 6*m->nbody);
            if( pert.select>0 )
            {
                mjv_applyPerturbPose(m, d, &pert, 0);  // move mocap bodies only
                mjv_applyPerturbForce(m, d, &pert);
            }

			setcontrol(m, d);

            // run mj_step and count
           // mj_step(m, d);

            // break on reset
            if( d->time<startsimtm )
                break;
        }
    }
}

void simulation1(void)
{
	// no model
	if (!m1)
		return;

	// clear timers
	cleartimers(d1);

	// paused
	if (paused)
	{
		// apply pose perturbations, run mj_forward
		if (pert.active)
		{
			mjv_applyPerturbPose(m1, d1, &pert, 1);      // move mocap and dynamic bodies
			mj_forward(m1, d1);
		}
	}

	// running
	else
	{
		// slow motion factor: 10x
		mjtNum factor = (slowmotion ? slowmotion1 : slowmotion2); // smaller faster 0.04:1

												// advance effective simulation time by 1/refreshrate
		mjtNum startsimtm = d1->time;
		while ((d1->time - startsimtm)*factor<1.0 / refreshrate)
		{
			// clear old perturbations, apply new
			mju_zero(d1->xfrc_applied, 6 * m1->nbody);
			if (pert.select>0)
			{
				mjv_applyPerturbPose(m1, d1, &pert, 0);  // move mocap bodies only
				mjv_applyPerturbForce(m1, d1, &pert);
			}

			setcontrol1(m1, d1);

			// run mj_step and count
			//mj_step(m1, d1);

			// break on reset
			if (d1->time<startsimtm)
				break;
		}
	}
}

void simulation2(void)
{
	// no model
	if (!m2)
		return;

	// clear timers
	cleartimers(d2);

	// paused
	if (paused)
	{
		// apply pose perturbations, run mj_forward
		if (pert.active)
		{
			mjv_applyPerturbPose(m2, d2, &pert, 1);      // move mocap and dynamic bodies
			mj_forward(m2, d2);
		}
	}

	// running
	else
	{
		// slow motion factor: 10x
		mjtNum factor = (slowmotion ? slowmotion1 : slowmotion2); // smaller faster 0.04:1

												  // advance effective simulation time by 1/refreshrate
		mjtNum startsimtm = d2->time;
		while ((d2->time - startsimtm)*factor<1.0 / refreshrate)
		{
			// clear old perturbations, apply new
			mju_zero(d2->xfrc_applied, 6 * m2->nbody);
			if (pert.select>0)
			{
				mjv_applyPerturbPose(m2, d2, &pert, 0);  // move mocap bodies only
				mjv_applyPerturbForce(m2, d2, &pert);
			}

			setcontrol2(m2, d2);

			// run mj_step and count
			//mj_step(m2, d2);

			// break on reset
			if (d2->time<startsimtm)
				break;
		}
	}
}

// render
void render(GLFWwindow* window)
{
    // past data for FPS calculation
    static double lastrendertm = 0;

    // get current framebuffer rectangle
    mjrRect rect = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &rect.width, &rect.height);
    mjrRect smallrect = rect;

    // reduce rectangle when profiler is on
    if( showprofiler )
        smallrect.width = rect.width - rect.width/5;

    // no model: empty screen
    if( !m )
    {
        mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Drag-and-drop model file here", 0, &con);

        // swap buffers
        glfwSwapBuffers(window); 
        return;
    }

    // advance simulation
    simulation();

    // update simulation statistics
    if( !paused )
    {
        // camera string
        char camstr[20];
        if( cam.type==mjCAMERA_FREE )
            strcpy(camstr, "Free");
        else if( cam.type==mjCAMERA_TRACKING )
            strcpy(camstr, "Tracking");
        else
            sprintf(camstr, "Fixed %d", cam.fixedcamid);

        // keyreset string
        char keyresetstr[20];
        if( keyreset<0 )
            strcpy(keyresetstr, "qpos0");
        else 
            sprintf(keyresetstr, "Key %d", keyreset);

        // solver error
        mjtNum solerr = 0;
        if( d->solver_iter )
        {
            int ind = mjMIN(d->solver_iter-1,mjNSOLVER-1);
            solerr = mju_min(d->solver[ind].improvement, d->solver[ind].gradient);
            if( solerr==0 )
                solerr = mju_max(d->solver[ind].improvement, d->solver[ind].gradient);
        }
        solerr = mju_log10(mju_max(mjMINVAL, solerr));

        // status
		sprintf(status, "%-20.1f\n%d  (%d con)\n%.3f\n%.0f\n%.2f\n%.1f  (%d it)\n%.1f %.1f\n%f\n%f\n%f\n%f",
			d->time,
			d->nefc,
			d->ncon,
			d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number),
			1.0 / (glfwGetTime() - lastrendertm),
			d->energy[0] + d->energy[1],
			solerr,
			d->solver_iter,
			mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[0])),
			mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[1])),
			//camstr,
			//mjFRAMESTRING[vopt.frame], 
			//mjLABELSTRING[vopt.label],
			d->qpos[0],
			d->qpos[1],
			d->time,
			d->qpos[6]
            );
    }

    // FPS timing satistics
    lastrendertm = glfwGetTime();

    // update scene
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // render
    mjr_render(rect, &scn, &con);

    // show depth map
    if( showdepth )
    {
        // get the depth buffer
        mjr_readPixels(NULL, depth_buffer, rect, &con);

        // convert to RGB, subsample by 4
        for( int r=0; r<rect.height; r+=4 )
            for( int c=0; c<rect.width; c+=4 )
            {
                // get subsampled address
                int adr = (r/4)*(rect.width/4) + c/4;

                // assign rgb
                depth_rgb[3*adr] = depth_rgb[3*adr+1] = depth_rgb[3*adr+2] = 
                    (unsigned char)((1.0f-depth_buffer[r*rect.width+c])*255.0f);
            }

        // show in bottom-right corner, offset for profiler and sensor
        mjrRect bottomright = {
            smallrect.left+(3*smallrect.width)/4, 
            smallrect.bottom, 
            smallrect.width/4, 
            smallrect.height/4
        };
        if( showsensor )
            bottomright.left -= smallrect.width/4;
        mjr_drawPixels(depth_rgb, NULL, bottomright, &con);
    }

    // show overlays
    if( showhelp==1 )
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, "Help  ", "F1  ", &con);
    else if( showhelp==2 )
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, help_title, help_content, &con);

    // show info
    if( showinfo )
    {
        if( paused )
            mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect, "PAUSED", 0, &con);
        else
            mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect, 
			"Time\nSize\nCPU\nFPS\nEnergy\nSolver\nFwdInv\nCamera\nqvel[0]\nqpos[1]\nTrial#", status, &con);
    }

    // show options
    if( showoption )
    {
        int i;
        char buf[100];

        // fill titles on first pass
        if( !opt_title[0] )
        {
            for( i=0; i<mjNRNDFLAG; i++)
            {
                makeoptionstring(mjRNDSTRING[i][0], mjRNDSTRING[i][2][0], buf);
                strcat(opt_title, buf);
                strcat(opt_title, "\n");
            }
            for( i=0; i<mjNVISFLAG; i++)
            {
                makeoptionstring(mjVISSTRING[i][0], mjVISSTRING[i][2][0], buf);
                strcat(opt_title, buf);
                if( i<mjNVISFLAG-1 )
                    strcat(opt_title, "\n");
            }
        }

        // fill content
        opt_content[0] = 0;
        for( i=0; i<mjNRNDFLAG; i++)
        {
            strcat(opt_content, scn.flags[i] ? " + " : "   ");
            strcat(opt_content, "\n");
        }
        for( i=0; i<mjNVISFLAG; i++)
        {
            strcat(opt_content, vopt.flags[i] ? " + " : "   ");
            if( i<mjNVISFLAG-1 )
                strcat(opt_content, "\n");
        }

        // show
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, smallrect, opt_title, opt_content, &con);
    }

    // show profiler
    if( showprofiler )
    {
        if( !paused )
            profilerupdate();
        profilershow(rect);
    }

    // show sensor
    if( showsensor )
    {
        if( !paused )
            sensorupdate();
        sensorshow(smallrect);
    }

    // swap buffers
    glfwSwapBuffers(window);
}

void render1(GLFWwindow* window)
{
	// past data for FPS calculation
	static double lastrendertm = 0;

	// get current framebuffer rectangle
	mjrRect rect = { 0, 0, 0, 0 };
	glfwGetFramebufferSize(window, &rect.width, &rect.height);
	mjrRect smallrect = rect;

	// reduce rectangle when profiler is on
	if (showprofiler)
		smallrect.width = rect.width - rect.width / 5;

	// no model: empty screen
	if (!m1)
	{
		mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Drag-and-drop model file here", 0, &con);

		// swap buffers
		glfwSwapBuffers(window);
		return;
	}

	// advance simulation
	simulation1();

	// update simulation statistics
	if (!paused)
	{
		// camera string
		char camstr[20];
		if (cam.type == mjCAMERA_FREE)
			strcpy(camstr, "Free");
		else if (cam.type == mjCAMERA_TRACKING)
			strcpy(camstr, "Tracking");
		else
			sprintf(camstr, "Fixed %d", cam.fixedcamid);

		// keyreset string
		char keyresetstr[20];
		if (keyreset<0)
			strcpy(keyresetstr, "qpos0");
		else
			sprintf(keyresetstr, "Key %d", keyreset);

		// solver error
		mjtNum solerr = 0;
		if (d1->solver_iter)
		{
			int ind = mjMIN(d1->solver_iter - 1, mjNSOLVER - 1);
			solerr = mju_min(d1->solver[ind].improvement, d1->solver[ind].gradient);
			if (solerr == 0)
				solerr = mju_max(d1->solver[ind].improvement, d1->solver[ind].gradient);
		}
		solerr = mju_log10(mju_max(mjMINVAL, solerr));

		// status
		sprintf(status, "%-20.1f\n%d  (%d con)\n%.3f\n%.0f\n%.2f\n%.1f  (%d it)\n%.1f %.1f\n%s\n%f\n%f\n%f",
			d1->time,
			d1->nefc,
			d1->ncon,
			d1->timer[mjTIMER_STEP].duration / mjMAX(1, d1->timer[mjTIMER_STEP].number),
			1.0 / (glfwGetTime() - lastrendertm),
			d1->energy[0] + d1->energy[1],
			solerr,
			d1->solver_iter,
			mju_log10(mju_max(mjMINVAL, d1->solver_fwdinv[0])),
			mju_log10(mju_max(mjMINVAL, d1->solver_fwdinv[1])),
			camstr,
			d1->qpos[0],
			//mjFRAMESTRING[vopt.frame],
			d1->qpos[1],
			//mjLABELSTRING[vopt.label],
			0.5
		);
	}

	// FPS timing satistics
	lastrendertm = glfwGetTime();

	// update scene
	mjv_updateScene(m1, d1, &vopt, &pert, &cam, mjCAT_ALL, &scn);

	// render
	mjr_render(rect, &scn, &con);

	// show depth map
	if (showdepth)
	{
		// get the depth buffer
		mjr_readPixels(NULL, depth_buffer, rect, &con);

		// convert to RGB, subsample by 4
		for (int r = 0; r<rect.height; r += 4)
			for (int c = 0; c<rect.width; c += 4)
			{
				// get subsampled address
				int adr = (r / 4)*(rect.width / 4) + c / 4;

				// assign rgb
				depth_rgb[3 * adr] = depth_rgb[3 * adr + 1] = depth_rgb[3 * adr + 2] =
					(unsigned char)((1.0f - depth_buffer[r*rect.width + c])*255.0f);
			}

		// show in bottom-right corner, offset for profiler and sensor
		mjrRect bottomright = {
			smallrect.left + (3 * smallrect.width) / 4,
			smallrect.bottom,
			smallrect.width / 4,
			smallrect.height / 4
		};
		//	if (showsensor)
		//		bottomright.left -= smallrect.width / 4;
		mjr_drawPixels(depth_rgb, NULL, bottomright, &con);
	}

	// show overlays
	if (showhelp == 1)
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, "Help  ", "F1  ", &con);
	else if (showhelp == 2)
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, help_title, help_content, &con);

	// show info
	if (showinfo)
	{
		if (paused)
			mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect, "PAUSED", 0, &con);
		else
			mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect,
				"Time\nSize\nCPU\nFPS\nEnergy\nSolver\nFwdInv\nCamera\nFrame\nqpos0\nTrial#", status, &con);
	}

	// show options
	if (showoption)
	{
		int i;
		char buf[100];

		// fill titles on first pass
		if (!opt_title[0])
		{
			for (i = 0; i<mjNRNDFLAG; i++)
			{
				makeoptionstring(mjRNDSTRING[i][0], mjRNDSTRING[i][2][0], buf);
				strcat(opt_title, buf);
				strcat(opt_title, "\n");
			}
			for (i = 0; i<mjNVISFLAG; i++)
			{
				makeoptionstring(mjVISSTRING[i][0], mjVISSTRING[i][2][0], buf);
				strcat(opt_title, buf);
				if (i<mjNVISFLAG - 1)
					strcat(opt_title, "\n");
			}
		}

		// fill content
		opt_content[0] = 0;
		for (i = 0; i<mjNRNDFLAG; i++)
		{
			strcat(opt_content, scn.flags[i] ? " + " : "   ");
			strcat(opt_content, "\n");
		}
		for (i = 0; i<mjNVISFLAG; i++)
		{
			strcat(opt_content, vopt.flags[i] ? " + " : "   ");
			if (i<mjNVISFLAG - 1)
				strcat(opt_content, "\n");
		}

		// show
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, smallrect, opt_title, opt_content, &con);
	}

	//// show profiler
	//if (showprofiler)
	//{
	//	if (!paused)
	//		profilerupdate();
	//	profilershow(rect);
	//}

	//// show sensor
	//if (showsensor)
	//{
	//	if (!paused)
	//		sensorupdate();
	//	sensorshow(smallrect);
	//}

	// swap buffers
	glfwSwapBuffers(window);
}

void render2(GLFWwindow* window)
{
	// past data for FPS calculation
	static double lastrendertm = 0;

	// get current framebuffer rectangle
	mjrRect rect = { 0, 0, 0, 0 };
	glfwGetFramebufferSize(window, &rect.width, &rect.height);
	mjrRect smallrect = rect;

	// reduce rectangle when profiler is on
	if (showprofiler)
		smallrect.width = rect.width - rect.width / 5;

	// no model: empty screen
	if (!m2)
	{
		mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Drag-and-drop model file here", 0, &con);

		// swap buffers
		glfwSwapBuffers(window);
		return;
	}

	// advance simulation
	simulation2();

	// update simulation statistics
	if (!paused)
	{
		// camera string
		char camstr[20];
		if (cam.type == mjCAMERA_FREE)
			strcpy(camstr, "Free");
		else if (cam.type == mjCAMERA_TRACKING)
			strcpy(camstr, "Tracking");
		else
			sprintf(camstr, "Fixed %d", cam.fixedcamid);

		// keyreset string
		char keyresetstr[20];
		if (keyreset<0)
			strcpy(keyresetstr, "qpos0");
		else
			sprintf(keyresetstr, "Key %d", keyreset);

		// solver error
		mjtNum solerr = 0;
		if (d2->solver_iter)
		{
			int ind = mjMIN(d2->solver_iter - 1, mjNSOLVER - 1);
			solerr = mju_min(d2->solver[ind].improvement, d2->solver[ind].gradient);
			if (solerr == 0)
				solerr = mju_max(d2->solver[ind].improvement, d2->solver[ind].gradient);
		}
		solerr = mju_log10(mju_max(mjMINVAL, solerr));

		// status
		sprintf(status, "%-20.1f\n%d  (%d con)\n%.3f\n%.0f\n%.2f\n%.1f  (%d it)\n%.1f %.1f\n%s\n%f\n%f\n%d",
			d2->time,
			d2->nefc,
			d2->ncon,
			d2->timer[mjTIMER_STEP].duration / mjMAX(1, d2->timer[mjTIMER_STEP].number),
			1.0 / (glfwGetTime() - lastrendertm),
			d2->energy[0] + d2->energy[1],
			solerr,
			d2->solver_iter,
			mju_log10(mju_max(mjMINVAL, d2->solver_fwdinv[0])),
			mju_log10(mju_max(mjMINVAL, d2->solver_fwdinv[1])),
			camstr,
			//mjFRAMESTRING[vopt.frame],
			d2->qpos[0],
			d2->qpos[1],
			//mjLABELSTRING[vopt.label],
			tri_num
		);
	}

	// FPS timing satistics
	lastrendertm = glfwGetTime();

	// update scene
	mjv_updateScene(m2, d2, &vopt, &pert, &cam, mjCAT_ALL, &scn);

	// render
	mjr_render(rect, &scn, &con);

	// show depth map
	if (showdepth)
	{
		// get the depth buffer
		mjr_readPixels(NULL, depth_buffer, rect, &con);

		// convert to RGB, subsample by 4
		for (int r = 0; r<rect.height; r += 4)
			for (int c = 0; c<rect.width; c += 4)
			{
				// get subsampled address
				int adr = (r / 4)*(rect.width / 4) + c / 4;

				// assign rgb
				depth_rgb[3 * adr] = depth_rgb[3 * adr + 1] = depth_rgb[3 * adr + 2] =
					(unsigned char)((1.0f - depth_buffer[r*rect.width + c])*255.0f);
			}

		// show in bottom-right corner, offset for profiler and sensor
		mjrRect bottomright = {
			smallrect.left + (3 * smallrect.width) / 4,
			smallrect.bottom,
			smallrect.width / 4,
			smallrect.height / 4
		};
		//	if (showsensor)
		//		bottomright.left -= smallrect.width / 4;
		mjr_drawPixels(depth_rgb, NULL, bottomright, &con);
	}

	// show overlays
	if (showhelp == 1)
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, "Help  ", "F1  ", &con);
	else if (showhelp == 2)
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, help_title, help_content, &con);

	// show info
	if (showinfo)
	{
		if (paused)
			mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect, "PAUSED", 0, &con);
		else
			mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect,
				"Time\nSize\nCPU\nFPS\nEnergy\nSolver\nFwdInv\nCamera\nFrame\nqpos0\nTrial#", status, &con);
	}

	// show options
	if (showoption)
	{
		int i;
		char buf[100];

		// fill titles on first pass
		if (!opt_title[0])
		{
			for (i = 0; i<mjNRNDFLAG; i++)
			{
				makeoptionstring(mjRNDSTRING[i][0], mjRNDSTRING[i][2][0], buf);
				strcat(opt_title, buf);
				strcat(opt_title, "\n");
			}
			for (i = 0; i<mjNVISFLAG; i++)
			{
				makeoptionstring(mjVISSTRING[i][0], mjVISSTRING[i][2][0], buf);
				strcat(opt_title, buf);
				if (i<mjNVISFLAG - 1)
					strcat(opt_title, "\n");
			}
		}

		// fill content
		opt_content[0] = 0;
		for (i = 0; i<mjNRNDFLAG; i++)
		{
			strcat(opt_content, scn.flags[i] ? " + " : "   ");
			strcat(opt_content, "\n");
		}
		for (i = 0; i<mjNVISFLAG; i++)
		{
			strcat(opt_content, vopt.flags[i] ? " + " : "   ");
			if (i<mjNVISFLAG - 1)
				strcat(opt_content, "\n");
		}

		// show
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, smallrect, opt_title, opt_content, &con);
	}

	//// show profiler
	//if (showprofiler)
	//{
	//	if (!paused)
	//		profilerupdate();
	//	profilershow(rect);
	//}

	//// show sensor
	//if (showsensor)
	//{
	//	if (!paused)
	//		sensorupdate();
	//	sensorshow(smallrect);
	//}

	// swap buffers
	glfwSwapBuffers(window);
}

//-------------------------------- main function ----------------------------------------

int wWinMain(int argc, const char** argv)
{
	// print version, check compatibility
	printf("MuJoCo Pro library version %.2lf\n", 0.01*mj_version());
	if (mjVERSION_HEADER != mj_version())
		mju_error("Headers and library have different versions");

	// activate MuJoCo license
	mj_activate("../../../doc/mjkey.txt");

	// load model
	char error[1000] = "Could not load binary model";
	m = mj_loadXML(mfilename, 0, error, 1000);
	if (!m)
		return finish(error);

	// make data
	d = mj_makeData(m);
	if (!d)
		return finish("Could not allocate mjData", m);

	if (TOP_FLAG == true)
	{
		#if defined(CART2POLE)||defined(CART3POLE)||defined(CART7POLE)
				state_nominal[0][2] = PI;
		#elif defined(PENDULUM)
				state_nominal[0][0] = 0;
		#elif defined(CARTPOLE)
				state_nominal[0][2] = -PI;
		#elif defined(ACROBOT)
				state_nominal[0][0] = 0;
				state_nominal[0][2] = -2*PI;
		#endif
	}

	if ((fp = fopen("../../../data/result.txt", "r")) != NULL)
	{
		fscanf(fp, "%s", ctrl_buff);
		if (ctrl_buff[0] == 'C') {
			for (int i = 0; i < ctrl_num * step_max; i++)
			{
				fscanf(fp, "%s", ctrl_buff);
				ctrl_nominal[i] = atof(ctrl_buff);

				if (CTRL_LIMITTED == true)
				{
					if (ctrl_nominal[i] > m->actuator_ctrlrange[1]) ctrl_nominal[i] = m->actuator_ctrlrange[1];
					else if (ctrl_nominal[i] < m->actuator_ctrlrange[0]) ctrl_nominal[i] = m->actuator_ctrlrange[0];
				}
				if (TOP_FLAG == true)
				{
					ctrl_nominal[i] = 0;
				}
			}
		}
		fclose(fp);
	}

	if ((fop = fopen("../../../data/TK.txt", "r")) != NULL)
	{
		for (int i1 = 0; i1 < step_max - 1; i1++) {
			for (int i2 = 0; i2 < ctrl_num; i2++) {
				#if defined(FISH)
				for (int i = 0; i < NS - 1; i++) {
					fscanf(fop, "%s", ctrl_buff);
					TK[i1][i2][i] = atof(ctrl_buff);
				}
				#else
				for (int i = 0; i < NS; i++) {
					fscanf(fop, "%s", ctrl_buff);
					TK[i1][i2][i] = atof(ctrl_buff);
				}
				#endif	
			}
		}
		fclose(fop);
	}

	srand((unsigned)time(NULL));
	for (int e = 0; e < step_max * ctrl_num; e++)
	{
		delta_u[e] = ptb_coef * gaussrand();
	}

	while (get_nominal(m, d) != 1) ;
	
	if (SYSIDCHECK == true)
	{
#if defined(FISH)
		sysidcheckfish(m, d);
#else
		sysidcheck(m, d);
#endif
		return 0;
	}

	if (ERRCOMPARE == true)
	{
		m1 = mj_loadXML(mfilename, 0, error, 1000);
		if (!m1) return finish(error);

		d1 = mj_makeData(m1);
		if (!d1) return finish("Could not allocate mjData", m1);
		errcompare();
		return 0;
	}

	// init GLFW
	if (!glfwInit())
		return 1;

	// get refreshrate
	refreshrate = glfwGetVideoMode(glfwGetPrimaryMonitor())->refreshRate;

	// multisampling
	glfwWindowHint(GLFW_SAMPLES, 4);

	// create window
	GLFWwindow* window = glfwCreateWindow(600, 500, "closed loop with noise", NULL, NULL);
	GLFWwindow* window1 = glfwCreateWindow(600, 500, "openloop with noise", NULL, NULL);
	GLFWwindow* window2 = glfwCreateWindow(600, 500, "openloop with no noise", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		return 1;
	}
	glfwSetWindowPos(window, 1200, 100);
	glfwSetWindowPos(window1, 0, 100);
	glfwSetWindowPos(window2, 600, 100);	 
	// make context current, disable v-sync
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// save window-to-framebuffer pixel scaling (needed for OSX scaling)
	int width, width1, height;
	glfwGetWindowSize(window, &width, &height);
	glfwGetFramebufferSize(window, &width1, &height);

	// set GLFW callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetScrollCallback(window, scroll);
	glfwSetDropCallback(window, drop);
	glfwSetWindowRefreshCallback(window, render);

	// switch and do the same thing for window1
	glfwMakeContextCurrent(window1);
	glfwSwapInterval(1);
	glfwGetWindowSize(window1, &width, &height);
	glfwGetFramebufferSize(window1, &width1, &height);
	glfwSetKeyCallback(window1, keyboard);
	glfwSetCursorPosCallback(window1, mouse_move);
	glfwSetMouseButtonCallback(window1, mouse_button);
	glfwSetScrollCallback(window1, scroll);
	glfwSetDropCallback(window1, drop);
	glfwSetWindowRefreshCallback(window1, render1);

	// switch and do the same thing for window2
	glfwMakeContextCurrent(window2);
	glfwSwapInterval(1);
	glfwGetWindowSize(window2, &width, &height);
	glfwGetFramebufferSize(window2, &width1, &height);
	glfwSetKeyCallback(window2, keyboard);
	glfwSetCursorPosCallback(window2, mouse_move);
	glfwSetMouseButtonCallback(window2, mouse_button);
	glfwSetScrollCallback(window2, scroll);
	glfwSetDropCallback(window2, drop);
	glfwSetWindowRefreshCallback(window2, render2);
	window2buffer = (double)width1 / (double)width;

	// init MuJoCo rendering, get OpenGL info
	mjv_makeScene(&scn, 1000);
	mjv_defaultCamera(&cam);
	mjv_defaultOption(&vopt);
	mjr_defaultContext(&con);
	mjr_makeContext(m, &con, fontscale);
	profilerinit();
	sensorinit();

	// set MuJoCo time callback for profiling
	mjcb_time = timer;

	// load model if filename given as argument
	/*  if( argc==2 )
	loadmodel(window, argv[1]);*/
	glfwMakeContextCurrent(window);
	loadmodel(window, mfilename);
	glfwMakeContextCurrent(window1);
	loadmodel1(window1, mfilename);
	glfwMakeContextCurrent(window2);
	loadmodel2(window2, mfilename);

	// main loop
	while (!glfwWindowShouldClose(window) && !glfwWindowShouldClose(window1) && !glfwWindowShouldClose(window2))
	{
		// simulate and render
		glfwMakeContextCurrent(window);
		render(window);
		glfwMakeContextCurrent(window1);
		render1(window1);
		glfwMakeContextCurrent(window2);
		render2(window2);				  

		// handle events (this calls all callbacks)
		glfwPollEvents();
	}

	// option: output data for checking
	if ((fp1 = fopen("../../../data/display.txt", "at+")) != NULL)
	{
		str3 = glstr;
		for (int h = 0; h < NS; h++)
		{
			for (int d = 1; d < step_max; d++)
			{
				sprintf(str3, "%4.8f", state_nominal[d][h]);
				fwrite(str3, 10, 1, fp1);
				fputs(" ", fp1);
			}
			fputs("\n", fp1);
			/*for (int d = 1; d < step_max; d++)
			{
				sprintf(str3, "%4.8f", state_closedlp[d][h]);
				fwrite(str3, 10, 1, fp1);
				fputs(" ", fp1);
			}
			fputs("\n", fp1);*/
	/*		for (int d = 1; d < step_max; d++)
			{
				sprintf(str3, "%4.8f", state_openlp[d][h]);
				fwrite(str3, 10, 1, fp1);
				fputs(" ", fp1);
			}
			fputs("\n", fp1);*/
			fputs("\n", fp1);
		}
		fclose(fp1);
	}

	// delete everything we allocated
	mj_deleteData(d);
	mj_deleteModel(m);
	mj_deleteData(d1);
	mj_deleteModel(m1);
	mj_deleteData(d2);
	mj_deleteModel(m2);				
	mjr_freeContext(&con);
	mjv_freeScene(&scn);

	// terminate
	glfwTerminate();
	mj_deactivate();
	return 0;
}