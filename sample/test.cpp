/*  Copyrigh 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

#include <windows.h>
#include <thread>
#include <mutex>
#include "uitools.h"
#include "funclib.h"

//-------------------------------- global -----------------------------------------------
// constants
extern const int kMaxStep = 3000;   // max step number for one rollout
extern const int kMaxState = 160;	// max (state dimension, actuator number)

const int kTestNum = 1000;	        // number of monte-carlo runs
const int kMaxGeom = 5000;          // preallocated geom array in mjvScene
const double syncmisalign = 0.1;    // maximum time mis-alignment before re-sync
const double refreshfactor = 0.5;   // fraction of refresh available for simulation

// extern model specific parameters
extern int integration_per_step;
extern int stepnum;
extern int actuatornum;
extern int quatnum;
extern int dof;
extern int modelid;
extern mjtNum control_timestep;
extern mjtNum simulation_timestep;
extern mjtNum state_nominal[kMaxStep][kMaxState];
extern mjtNum ctrl_nominal[kMaxStep * kMaxState];
extern mjtNum ctrl_openloop[kMaxStep * kMaxState];
extern mjtNum rest_length[kMaxStep * kMaxState];
extern mjtNum delta_rest_length[kMaxStep*kMaxState];
extern mjtNum state_target[kMaxState];
extern mjtNum stabilizer_feedback_gain[kMaxState][kMaxState];
extern mjtNum ctrl_upperlimit;
extern mjtNum ctrl_lowerlimit;

// hyperparameters 
extern mjtNum Q, QT, R;
extern mjtNum Qm[kMaxState][kMaxState], QTm[kMaxState][kMaxState];

// user data
mjModel* m = NULL;
mjData* d = NULL;
mjData* d_closedloop = NULL;
mjData* d_openloop = NULL;
mjtNum ctrl_max = 0;
mjtNum perturb_coefficient_std = 0;
mjtNum cost_closedloop = 0, cost_openloop = 0;
mjtNum energy = 0;
mjtNum tracker_feedback_gain[kMaxStep][kMaxState][kMaxState] = { 0 };
FILE *filestream1, *filestream2;
char testmode[30];
char data_buff[30];
char keyfilename[100];
char datafilename[100];
char modelfilename[100];
char username[30];
char modelname[30];
char keyfilepre[20]  = "";
int step_index_nominal = 0;
int step_index_openloop = 0;
int step_index_closedloop = 0;
bool NFinal = false;
bool terminal_trigger = false;

// abstract visualization
mjvScene scn, scn_openloop, scn_closedloop;
mjvCamera cam;
mjvOption vopt;
mjvPerturb pert;
mjvFigure figconstraint;
mjvFigure figcost;
mjvFigure figtimer;
mjvFigure figsize;
mjvFigure figsensor;


// OpenGL rendering and UI
GLFWvidmode vmode;
int windowpos[2];
int windowsize[2];
mjrContext con, con_openloop, con_closedloop;
GLFWwindow *window = NULL, *window_openloop = NULL, *window_closedloop = NULL;
mjuiState uistate;
mjUI ui0, ui1;


// UI settings not contained in MuJoCo structures
struct
{
    // file
    int exitrequest = 0;

    // option
    int spacing = 0;
    int color = 0;
    int font = 0;
    int ui0 = 0;
    int ui1 = 0;
    int help = 0;               
    int info = 1;               
    int profiler = 0;           
    int sensor = 0;
    int fullscreen = 0;
    int vsync = 1;
    int busywait = 0;

    // simulation
    int run = 0;// 0 is pause, 1 is run
    int key = 0;
    int loadrequest = 0;

    // watch
    char field[mjMAXUITEXT] = "qpos";
    int index = 0;

    // physics: need sync
    int disable[mjNDISABLE];
    int enable[mjNENABLE];

    // rendering: need sync
    int camera = 0;
} settings;


// section ids
enum
{
    // left ui
    SECT_FILE   = 0,
    SECT_OPTION,
    SECT_SIMULATION,
    SECT_WATCH,
	SECT_PHYSICS,
    SECT_RENDERING,
    SECT_GROUP,
    NSECT0,

    // right ui
    SECT_JOINT = 0,
    SECT_CONTROL,
    NSECT1
};


// file section of UI
const mjuiDef defFile[] = 
{
    {mjITEM_SECTION,   "File",          1, NULL,                    "AF"},
    {mjITEM_BUTTON,    "Save xml",      2, NULL,                    ""},
    {mjITEM_BUTTON,    "Save mjb",      2, NULL,                    ""},
    {mjITEM_BUTTON,    "Print model",   2, NULL,                    "CM"},
    {mjITEM_BUTTON,    "Print data",    2, NULL,                    "CD"},
    {mjITEM_BUTTON,    "Quit",          1, NULL,                    "CQ"},
    {mjITEM_END}
};


// option section of UI
const mjuiDef defOption[] = 
{
    {mjITEM_SECTION,   "Option",        1, NULL,                    "AO"},
    {mjITEM_SELECT,    "Spacing",       1, &settings.spacing,       "Tight\nWide"},
    {mjITEM_SELECT,    "Color",         1, &settings.color,         "Default\nOrange\nWhite\nBlack"},
    {mjITEM_SELECT,    "Font",          1, &settings.font,          "50 %\n100 %\n150 %\n200 %\n250 %\n300 %"},
    {mjITEM_CHECKINT,  "Left UI (Tab)", 1, &settings.ui0,           " #258"},
    {mjITEM_CHECKINT,  "Right UI",      1, &settings.ui1,           "S#258"},
    {mjITEM_CHECKINT,  "Help",          2, &settings.help,          " #290"},
    {mjITEM_CHECKINT,  "Info",          2, &settings.info,          " #291"},
    {mjITEM_CHECKINT,  "Profiler",      2, &settings.profiler,      " #292"},
    {mjITEM_CHECKINT,  "Sensor",        2, &settings.sensor,        " #293"},
#ifdef __APPLE__
    {mjITEM_CHECKINT,  "Fullscreen",    0, &settings.fullscreen,    " #294"},
#else
    {mjITEM_CHECKINT,  "Fullscreen",    1, &settings.fullscreen,    " #294"},
#endif
    {mjITEM_CHECKINT,  "Vertical Sync", 1, &settings.vsync,         " #295"},
    {mjITEM_CHECKINT,  "Busy Wait",     1, &settings.busywait,      " #296"},
    {mjITEM_END}
};


// simulation section of UI
const mjuiDef defSimulation[] = 
{
    {mjITEM_SECTION,   "Simulation",    1, NULL,                    "AS"},
    {mjITEM_RADIO,     "",              2, &settings.run,           "Pause\nRun"},
    {mjITEM_BUTTON,    "Reset",         2, NULL,                    " #259"},
    {mjITEM_BUTTON,    "Reload",        2, NULL,                    "CL"},
    {mjITEM_BUTTON,    "Align",         2, NULL,                    "CA"},
    {mjITEM_BUTTON,    "Copy pose",     2, NULL,                    "CC"},
    {mjITEM_SLIDERINT, "Key",           3, &settings.key,           "0 0"},
    {mjITEM_BUTTON,    "Reset to key",  3},
    {mjITEM_BUTTON,    "Set key",       3},
    {mjITEM_END}
};


// watch section of UI
const mjuiDef defWatch[] = 
{
    {mjITEM_SECTION,   "Watch",         0, NULL,                    "AW"},
    {mjITEM_EDITTXT,   "Field",         2, settings.field,          "qpos"},
    {mjITEM_EDITINT,   "Index",         2, &settings.index,         "1"},
    {mjITEM_STATIC,    "Value",         2, NULL,                    " "},
    {mjITEM_END}
};


// help strings
const char help_content[] = 
"Alt mouse button\n"
"UI right hold\n"
"UI title double-click\n"
"Space\n"
"Esc\n"
"Right arrow\n"
"Left arrow\n"
"Down arrow\n"
"Up arrow\n"
"Page Up\n"
"Double-click\n"
"Right double-click\n"
"Ctrl Right double-click\n"
"Scroll, middle drag\n"
"Left drag\n"
"[Shift] right drag\n"
"Ctrl [Shift] drag\n"
"Ctrl [Shift] right drag";

const char help_title[] = 
"Swap left-right\n"
"Show UI shortcuts\n"
"Expand/collapse all  \n"
"Pause\n"
"Free camera\n"
"Step forward\n"
"Step back\n"
"Step forward 100\n"
"Step back 100\n"
"Select parent\n"
"Select\n"
"Center\n"
"Track camera\n"
"Zoom\n"
"View rotate\n"
"View translate\n"
"Object rotate\n"
"Object translate";


// info strings
char info_title[1000];
char info_content[1000];
char info_content_openloop[1000];
char info_content_closedloop[1000];


//----------------------- profiler, sensor, info, watch ---------------------------------

// init profiler figures
void profilerinit(void)
{
    int i, n;

    // set figures to default
    mjv_defaultFigure(&figconstraint);
    mjv_defaultFigure(&figcost);
    mjv_defaultFigure(&figtimer);
    mjv_defaultFigure(&figsize);

    // titles
    strcpy(figconstraint.title, "Counts");
    strcpy(figcost.title, "Convergence (log 10)");
    strcpy(figsize.title, "Dimensions");
    strcpy(figtimer.title, "CPU time (msec)");

    // x-labels
    strcpy(figconstraint.xlabel, "Solver iteration");
    strcpy(figcost.xlabel, "Solver iteration");
    strcpy(figsize.xlabel, "Video frame");
    strcpy(figtimer.xlabel, "Video frame");

    // y-tick nubmer formats
    strcpy(figconstraint.yformat, "%.0f");
    strcpy(figcost.yformat, "%.1f");
    strcpy(figsize.yformat, "%.0f");
    strcpy(figtimer.yformat, "%.2f");

    // colors
    figconstraint.figurergba[0] =   0.1f;
    figcost.figurergba[2] =         0.2f;
    figsize.figurergba[0] =         0.1f;
    figtimer.figurergba[2] =        0.2f;
    figconstraint.figurergba[3] =   0.5f;
    figcost.figurergba[3] =         0.5f;
    figsize.figurergba[3] =         0.5f;
    figtimer.figurergba[3] =        0.5f;

    // legends
    strcpy(figconstraint.linename[0], "total");
    strcpy(figconstraint.linename[1], "active");
    strcpy(figconstraint.linename[2], "changed");
    strcpy(figconstraint.linename[3], "evals");
    strcpy(figconstraint.linename[4], "updates");
    strcpy(figcost.linename[0], "improvement");
    strcpy(figcost.linename[1], "gradient");
    strcpy(figcost.linename[2], "lineslope");
    strcpy(figsize.linename[0], "dof");
    strcpy(figsize.linename[1], "body");
    strcpy(figsize.linename[2], "constraint");
    strcpy(figsize.linename[3], "sqrt(nnz)");
    strcpy(figsize.linename[4], "contact");
    strcpy(figsize.linename[5], "iteration");
    strcpy(figtimer.linename[0], "total");
    strcpy(figtimer.linename[1], "collision");
    strcpy(figtimer.linename[2], "prepare");
    strcpy(figtimer.linename[3], "solve");
    strcpy(figtimer.linename[4], "other");

    // grid sizes
    figconstraint.gridsize[0] = 5;
    figconstraint.gridsize[1] = 5;
    figcost.gridsize[0] = 5;
    figcost.gridsize[1] = 5;
    figsize.gridsize[0] = 3;
    figsize.gridsize[1] = 5;
    figtimer.gridsize[0] = 3;
    figtimer.gridsize[1] = 5;

    // minimum ranges
    figconstraint.range[0][0] = 0;
    figconstraint.range[0][1] = 20;
    figconstraint.range[1][0] = 0;
    figconstraint.range[1][1] = 80;
    figcost.range[0][0] = 0;
    figcost.range[0][1] = 20;
    figcost.range[1][0] = -15;
    figcost.range[1][1] = 5;
    figsize.range[0][0] = -200;
    figsize.range[0][1] = 0;
    figsize.range[1][0] = 0;
    figsize.range[1][1] = 100;
    figtimer.range[0][0] = -200;
    figtimer.range[0][1] = 0;
    figtimer.range[1][0] = 0;
    figtimer.range[1][1] = 0.4f;

    // init x axis on history figures (do not show yet)
    for( n=0; n<6; n++ )
        for( i=0; i<mjMAXLINEPNT; i++ )
        {
            figtimer.linedata[n][2*i] = (float)-i;
            figsize.linedata[n][2*i] = (float)-i;
        }
}



// update profiler figures
void profilerupdate(void)
{
    int i, n;

    // update constraint figure
    figconstraint.linepnt[0] = mjMIN(mjMIN(d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
    for( i=1; i<5; i++ )
        figconstraint.linepnt[i] = figconstraint.linepnt[0];
    if( m->opt.solver==mjSOL_PGS )
    {
        figconstraint.linepnt[3] = 0;
        figconstraint.linepnt[4] = 0;
    }
    if( m->opt.solver==mjSOL_CG )
        figconstraint.linepnt[4] = 0;
    for( i=0; i<figconstraint.linepnt[0]; i++ )
    {
        // x
        figconstraint.linedata[0][2*i] = (float)i;
        figconstraint.linedata[1][2*i] = (float)i;
        figconstraint.linedata[2][2*i] = (float)i;
        figconstraint.linedata[3][2*i] = (float)i;
        figconstraint.linedata[4][2*i] = (float)i;

        // y
        figconstraint.linedata[0][2*i+1] = (float)d->nefc;
        figconstraint.linedata[1][2*i+1] = (float)d->solver[i].nactive;
        figconstraint.linedata[2][2*i+1] = (float)d->solver[i].nchange;
        figconstraint.linedata[3][2*i+1] = (float)d->solver[i].neval;
        figconstraint.linedata[4][2*i+1] = (float)d->solver[i].nupdate;
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
    mjtNum total = d->timer[mjTIMER_STEP].duration;
    int number = d->timer[mjTIMER_STEP].number;
    if( !number )
    {
        total = d->timer[mjTIMER_FORWARD].duration;
        number = d->timer[mjTIMER_FORWARD].number;
    }
    number = mjMAX(1, number);
    float tdata[5] = { 
        (float)(total/number),
        (float)(d->timer[mjTIMER_POS_COLLISION].duration/number),
        (float)(d->timer[mjTIMER_POS_MAKE].duration/number) +
            (float)(d->timer[mjTIMER_POS_PROJECT].duration/number),
        (float)(d->timer[mjTIMER_CONSTRAINT].duration/number),
        0
    };
    tdata[4] = tdata[0] - tdata[1] - tdata[2] - tdata[3];

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



// show profiler figures
void profilershow(mjrRect rect)
{
    mjrRect viewport = {
        rect.left + rect.width - rect.width/4, 
        rect.bottom, 
        rect.width/4, 
        rect.height/4
    };
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
    figsensor.figurergba[3] = 0.5f;

    // set flags
    figsensor.flg_extend = 1;
    figsensor.flg_barplot = 1;
    figsensor.flg_symmetric = 1;

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
    // constant width with and without profiler
    int width = settings.profiler ? rect.width/3 : rect.width/4;

    // render figure on the right
    mjrRect viewport = {
        rect.left + rect.width - width, 
        rect.bottom, 
        width, 
        rect.height/3
    };
    mjr_figure(viewport, &figsensor, &con);
}



// prepare info text
void infotext(mjData* d, char* title, char* content, double interval)
{
    char tmp[30];

    // compute solver error
    mjtNum solerr = 0;
    if( d->solver_iter )
    {
        int ind = mjMIN(d->solver_iter-1,mjNSOLVER-1);
        solerr = mju_min(d->solver[ind].improvement, d->solver[ind].gradient);
        if( solerr==0 )
            solerr = mju_max(d->solver[ind].improvement, d->solver[ind].gradient);
    }
    solerr = mju_log10(mju_max(mjMINVAL, solerr));

    // prepare info text
    strcpy(title, "Time\nSize\nCPU\nSolver   \nFPS\nstack\nconbuf\nefcbuf");
    sprintf(content, "%-20.3f\n%d  (%d con)\n%.3f\n%.1f  (%d it)\n%.0f\n%.3f\n%.3f\n%.3f",
            d->time, 
            d->nefc, d->ncon,
            settings.run ? 
                d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number) :
                d->timer[mjTIMER_FORWARD].duration / mjMAX(1, d->timer[mjTIMER_FORWARD].number),
            solerr, d->solver_iter, 
            1/interval,
            d->maxuse_stack/(double)d->nstack,
            d->maxuse_con/(double)m->nconmax,
            d->maxuse_efc/(double)m->njmax);

    // add Energy if enabled
    if( mjENABLED(mjENBL_ENERGY) )
    {
        sprintf(tmp, "\n%.3f", d->energy[0]+d->energy[1]);
        strcat(content, tmp);
        strcat(title, "\nEnergy");
    }

    // add FwdInv if enabled
    if( mjENABLED(mjENBL_FWDINV) )
    {
        sprintf(tmp, "\n%.1f %.1f", 
            mju_log10(mju_max(mjMINVAL,d->solver_fwdinv[0])),
            mju_log10(mju_max(mjMINVAL,d->solver_fwdinv[1])));
        strcat(content, tmp);
        strcat(title, "\nFwdInv");
    }   
}



// sprintf forwarding, to avoid compiler warning in x-macro
void printfield(char* str, void* ptr)
{
    sprintf(str, "%g", *(mjtNum*)ptr);
}



// update watch
void watch(void)
{
    // clear
    ui0.sect[SECT_WATCH].item[2].multi.nelem = 1;
    strcpy(ui0.sect[SECT_WATCH].item[2].multi.name[0], "invalid field");

    // prepare constants for NC
    int nv = m->nv;
    int njmax = m->njmax;

    // find specified field in mjData arrays, update value
    #define X(TYPE, NAME, NR, NC)                                           \
        if( !strcmp(#NAME, settings.field) && !strcmp(#TYPE, "mjtNum") )    \
        {                                                                   \
            if( settings.index>=0 && settings.index<m->NR*NC )              \
                printfield(ui0.sect[SECT_WATCH].item[2].multi.name[0],      \
                           d->NAME + settings.index);                       \
            else                                                            \
                strcpy(ui0.sect[SECT_WATCH].item[2].multi.name[0],          \
                       "invalid index");                                    \
            return;                                                         \
        }

        MJDATA_POINTERS
    #undef X
}



//-------------------------------- UI construction --------------------------------------

// make physics section of UI
void makephysics(int oldstate)
{
    int i;

    mjuiDef defPhysics[] = 
    {
        {mjITEM_SECTION,   "Physics",       oldstate, NULL,                 "AP"},
        {mjITEM_SELECT,    "Integrator",    2, &(m->opt.integrator),        "Euler\nRK4"},
        {mjITEM_SELECT,    "Collision",     2, &(m->opt.collision),         "All\nPair\nDynamic"},
        {mjITEM_SELECT,    "Cone",          2, &(m->opt.cone),              "Pyramidal\nElliptic"},
        {mjITEM_SELECT,    "Jacobian",      2, &(m->opt.jacobian),          "Dense\nSparse\nAuto"},
        {mjITEM_SELECT,    "Solver",        2, &(m->opt.solver),            "PGS\nCG\nNewton"},
        {mjITEM_SEPARATOR, "Algorithmic Parameters", 1},
        {mjITEM_EDITNUM,   "Timestep",      2, &(m->opt.timestep),          "1 0 1"},
        {mjITEM_EDITINT,   "Iterations",    2, &(m->opt.iterations),        "1 0 1000"},
        {mjITEM_EDITNUM,   "Tolerance",     2, &(m->opt.tolerance),         "1 0 1"},
        {mjITEM_EDITINT,   "Noslip Iter",   2, &(m->opt.noslip_iterations), "1 0 1000"},
        {mjITEM_EDITNUM,   "Noslip Tol",    2, &(m->opt.noslip_tolerance),  "1 0 1"},
        {mjITEM_EDITINT,   "MRR Iter",      2, &(m->opt.mpr_iterations),    "1 0 1000"},
        {mjITEM_EDITNUM,   "MPR Tol",       2, &(m->opt.mpr_tolerance),     "1 0 1"},
        {mjITEM_EDITNUM,   "API Rate",      2, &(m->opt.apirate),           "1 0 1000"},
        {mjITEM_SEPARATOR, "Physical Parameters", 1},
        {mjITEM_EDITNUM,   "Gravity",       2, m->opt.gravity,              "3"},
        {mjITEM_EDITNUM,   "Wind",          2, m->opt.wind,                 "3"},
        {mjITEM_EDITNUM,   "Magnetic",      2, m->opt.magnetic,             "3"},
        {mjITEM_EDITNUM,   "Density",       2, &(m->opt.density),           "1"},
        {mjITEM_EDITNUM,   "Viscosity",     2, &(m->opt.viscosity),         "1"},
        {mjITEM_EDITNUM,   "Imp Ratio",     2, &(m->opt.impratio),          "1"},
        {mjITEM_SEPARATOR, "Disable Flags", 1},
        {mjITEM_END}
    };
    mjuiDef defEnableFlags[] = 
    {
        {mjITEM_SEPARATOR, "Enable Flags", 1},
        {mjITEM_END}
    };
    mjuiDef defOverride[] = 
    {
        {mjITEM_SEPARATOR, "Contact Override", 1},
        {mjITEM_EDITNUM,   "Margin",        2, &(m->opt.o_margin),          "1"},
        {mjITEM_EDITNUM,   "Sol Imp",       2, &(m->opt.o_solimp),          "5"},
        {mjITEM_EDITNUM,   "Sol Ref",       2, &(m->opt.o_solref),          "2"},
        {mjITEM_END}
    };

    // add physics
    mjui_add(&ui0, defPhysics);

    // add flags programmatically
    mjuiDef defFlag[] = 
    {
        {mjITEM_CHECKINT,  "", 2, NULL, ""}, 
        {mjITEM_END}
    };
    for( i=0; i<mjNDISABLE; i++ )
    {
        strcpy(defFlag[0].name, mjDISABLESTRING[i]);
        defFlag[0].pdata = settings.disable + i;
        mjui_add(&ui0, defFlag);
    }
    mjui_add(&ui0, defEnableFlags);
    for( i=0; i<mjNENABLE; i++ )
    {
        strcpy(defFlag[0].name, mjENABLESTRING[i]);
        defFlag[0].pdata = settings.enable + i;
        mjui_add(&ui0, defFlag);
    }

    // add contact override
    mjui_add(&ui0, defOverride);
}



// make rendering section of UI
void makerendering(int oldstate)
{
    int i, j;

    mjuiDef defRendering[] = 
    {
        {mjITEM_SECTION,    "Rendering",        oldstate, NULL,             "AR"},
        {mjITEM_SELECT,     "Camera",           2, &(settings.camera),      "Free\nTracking"},
        {mjITEM_SELECT,     "Label",            2, &(vopt.label),           
            "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\nActuator\nConstraint\nSkin\nSelection\nSel Pnt\nForce"},
        {mjITEM_SELECT,     "Frame",            2, &(vopt.frame),           
            "None\nBody\nGeom\nSite\nCamera\nLight\nWorld"},
        {mjITEM_SEPARATOR,  "Model Elements",   1},
        {mjITEM_END}
    };
    mjuiDef defOpenGL[] = 
    {
        {mjITEM_SEPARATOR, "OpenGL Effects", 1},
        {mjITEM_END}
    };

    // add model cameras, up to UI limit
    for( i=0; i<mjMIN(m->ncam, mjMAXUIMULTI-2); i++ )
    {
        // prepare name
        char camname[mjMAXUITEXT] = "\n";
        if( m->names[m->name_camadr[i]] )
            strcat(camname, m->names+m->name_camadr[i]);
        else
            sprintf(camname, "\nCamera %d", i);

        // check string length
        if( strlen(camname) + strlen(defRendering[1].other)>=mjMAXUITEXT-1 )
            break;

        // add camera
        strcat(defRendering[1].other, camname);
    }

    // add rendering standard
    mjui_add(&ui0, defRendering);

    // add flags programmatically
    mjuiDef defFlag[] = 
    {
        {mjITEM_CHECKBYTE,  "", 2, NULL, ""}, 
        {mjITEM_END}
    };
    for( i=0; i<mjNVISFLAG; i++ )
    {
        // set name, remove "&"
        strcpy(defFlag[0].name, mjVISSTRING[i][0]);
        for( j=0; j<strlen(mjVISSTRING[i][0]); j++ )
            if( mjVISSTRING[i][0][j]=='&' )
            {
                strcpy(defFlag[0].name+j, mjVISSTRING[i][0]+j+1);
                break;
            }

        // set shortcut and data
        sprintf(defFlag[0].other, " %s", mjVISSTRING[i][2]);
        defFlag[0].pdata = vopt.flags + i;
        mjui_add(&ui0, defFlag);
    }
    mjui_add(&ui0, defOpenGL);
    for( i=0; i<mjNRNDFLAG; i++ )
    {
        strcpy(defFlag[0].name, mjRNDSTRING[i][0]);
        sprintf(defFlag[0].other, " %s", mjRNDSTRING[i][2]);
        defFlag[0].pdata = scn.flags + i;
        mjui_add(&ui0, defFlag);
    }
}


// make group section of UI
void makegroup(int oldstate)
{
    mjuiDef defGroup[] = 
    {
        {mjITEM_SECTION,    "Group enable",     oldstate, NULL,             "AG"},
        {mjITEM_SEPARATOR,  "Geom groups",  1},
        {mjITEM_CHECKBYTE,  "Geom 0",           2, vopt.geomgroup,          " 0"},
        {mjITEM_CHECKBYTE,  "Geom 1",           2, vopt.geomgroup+1,        " 1"},
        {mjITEM_CHECKBYTE,  "Geom 2",           2, vopt.geomgroup+2,        " 2"},
        {mjITEM_CHECKBYTE,  "Geom 3",           2, vopt.geomgroup+3,        " 3"},
        {mjITEM_CHECKBYTE,  "Geom 4",           2, vopt.geomgroup+4,        " 4"},
        {mjITEM_CHECKBYTE,  "Geom 5",           2, vopt.geomgroup+5,        " 5"},
        {mjITEM_SEPARATOR,  "Site groups",  1},
        {mjITEM_CHECKBYTE,  "Site 0",           2, vopt.sitegroup,          "S0"},
        {mjITEM_CHECKBYTE,  "Site 1",           2, vopt.sitegroup+1,        "S1"},
        {mjITEM_CHECKBYTE,  "Site 2",           2, vopt.sitegroup+2,        "S2"},
        {mjITEM_CHECKBYTE,  "Site 3",           2, vopt.sitegroup+3,        "S3"},
        {mjITEM_CHECKBYTE,  "Site 4",           2, vopt.sitegroup+4,        "S4"},
        {mjITEM_CHECKBYTE,  "Site 5",           2, vopt.sitegroup+5,        "S5"},
        {mjITEM_SEPARATOR,  "Joint groups", 1},
        {mjITEM_CHECKBYTE,  "Joint 0",          2, vopt.jointgroup,         ""},
        {mjITEM_CHECKBYTE,  "Joint 1",          2, vopt.jointgroup+1,       ""},
        {mjITEM_CHECKBYTE,  "Joint 2",          2, vopt.jointgroup+2,       ""},
        {mjITEM_CHECKBYTE,  "Joint 3",          2, vopt.jointgroup+3,       ""},
        {mjITEM_CHECKBYTE,  "Joint 4",          2, vopt.jointgroup+4,       ""},
        {mjITEM_CHECKBYTE,  "Joint 5",          2, vopt.jointgroup+5,       ""},
        {mjITEM_SEPARATOR,  "Tendon groups",    1},
        {mjITEM_CHECKBYTE,  "Tendon 0",         2, vopt.tendongroup,        ""},
        {mjITEM_CHECKBYTE,  "Tendon 1",         2, vopt.tendongroup+1,      ""},
        {mjITEM_CHECKBYTE,  "Tendon 2",         2, vopt.tendongroup+2,      ""},
        {mjITEM_CHECKBYTE,  "Tendon 3",         2, vopt.tendongroup+3,      ""},
        {mjITEM_CHECKBYTE,  "Tendon 4",         2, vopt.tendongroup+4,      ""},
        {mjITEM_CHECKBYTE,  "Tendon 5",         2, vopt.tendongroup+5,      ""},
        {mjITEM_SEPARATOR,  "Actuator groups", 1},
        {mjITEM_CHECKBYTE,  "Actuator 0",       2, vopt.actuatorgroup,      ""},
        {mjITEM_CHECKBYTE,  "Actuator 1",       2, vopt.actuatorgroup+1,    ""},
        {mjITEM_CHECKBYTE,  "Actuator 2",       2, vopt.actuatorgroup+2,    ""},
        {mjITEM_CHECKBYTE,  "Actuator 3",       2, vopt.actuatorgroup+3,    ""},
        {mjITEM_CHECKBYTE,  "Actuator 4",       2, vopt.actuatorgroup+4,    ""},
        {mjITEM_CHECKBYTE,  "Actuator 5",       2, vopt.actuatorgroup+5,    ""},
        {mjITEM_END}
    };

    // add section
    mjui_add(&ui0, defGroup);
}



// make joint section of UI
void makejoint(int oldstate)
{
    int i;

    mjuiDef defJoint[] = 
    {
        {mjITEM_SECTION, "Joint", oldstate, NULL, "AJ"},
        {mjITEM_END}
    };
    mjuiDef defSlider[] = 
    {
        {mjITEM_SLIDERNUM, "", 2, NULL, "0 1"},
        {mjITEM_END}
    };

    // add section
    mjui_add(&ui1, defJoint);
    defSlider[0].state = 4;

    // add scalar joints, exit if UI limit reached
    int itemcnt = 0;
    for( i=0; i<m->njnt && itemcnt<mjMAXUIITEM; i++ )
        if( (m->jnt_type[i]==mjJNT_HINGE || m->jnt_type[i]==mjJNT_SLIDE) )
        {
            // skip if joint group is disabled
            if( !vopt.jointgroup[mjMAX(0, mjMIN(mjNGROUP-1, m->jnt_group[i]))] )
                continue;

            // set data and name
            defSlider[0].pdata = d->qpos + m->jnt_qposadr[i];
            if( m->names[m->name_jntadr[i]] )
                mju_strncpy(defSlider[0].name, m->names+m->name_jntadr[i], 
                            mjMAXUINAME);
            else
                sprintf(defSlider[0].name, "joint %d", i);

            // set range
            if( m->jnt_limited[i] )
                sprintf(defSlider[0].other, "%.4g %.4g", 
                    m->jnt_range[2*i], m->jnt_range[2*i+1]);
            else if( m->jnt_type[i]==mjJNT_SLIDE )
                strcpy(defSlider[0].other, "-1 1");
            else
                strcpy(defSlider[0].other, "-3.1416 3.1416");

            // add and count
            mjui_add(&ui1, defSlider);
            itemcnt++;
        }
}



// make control section of UI
void makecontrol(int oldstate)
{
    int i;

    mjuiDef defControl[] = 
    {
        {mjITEM_SECTION, "Control", oldstate, NULL, "AC"},
        {mjITEM_BUTTON,  "Clear all", 2},
        {mjITEM_END}
    };
    mjuiDef defSlider[] = 
    {
        {mjITEM_SLIDERNUM, "", 2, NULL, "0 1"},
        {mjITEM_END}
    };

    // add section
    mjui_add(&ui1, defControl);
    defSlider[0].state = 2;

    // add controls, exit if UI limit reached (Clear button already added)
    int itemcnt = 1;
    for( i=0; i<m->nu && itemcnt<mjMAXUIITEM; i++ )
    {
        // skip if actuator group is disabled
        if( !vopt.actuatorgroup[mjMAX(0, mjMIN(mjNGROUP-1, m->actuator_group[i]))] )
            continue;

        // set data and name
        defSlider[0].pdata = d->ctrl + i;
        if( m->names[m->name_actuatoradr[i]] )
            mju_strncpy(defSlider[0].name, m->names+m->name_actuatoradr[i], 
                        mjMAXUINAME);
        else
            sprintf(defSlider[0].name, "control %d", i);

        // set range
        if( m->actuator_ctrllimited[i] )
            sprintf(defSlider[0].other, "%.4g %.4g", 
                m->actuator_ctrlrange[2*i], m->actuator_ctrlrange[2*i+1]);
        else
            strcpy(defSlider[0].other, "-1 1");

        // add and count
        mjui_add(&ui1, defSlider);
        itemcnt++;
    }
}



// make model-dependent UI sections
void makesections(void)
{
    int i;

    // get section open-close state, UI 0
    int oldstate0[NSECT0];
    for( i=0; i<NSECT0; i++ )
    {
        oldstate0[i] = 0;
        if( ui0.nsect>i )
            oldstate0[i] = ui0.sect[i].state;
    }

    // get section open-close state, UI 1
    int oldstate1[NSECT1];
    for( i=0; i<NSECT1; i++ )
    {
        oldstate1[i] = 0;
        if( ui1.nsect>i )
            oldstate1[i] = ui1.sect[i].state;
    }

    // clear model-dependent sections of UI
    ui0.nsect = SECT_PHYSICS;
    ui1.nsect = 0;

    // make
    makephysics(oldstate0[SECT_PHYSICS]);
    makerendering(oldstate0[SECT_RENDERING]);
    makegroup(oldstate0[SECT_GROUP]);
    makejoint(oldstate1[SECT_JOINT]);
    makecontrol(oldstate1[SECT_CONTROL]);
}



//-------------------------------- utility functions ------------------------------------

// align and scale view
void alignscale(void)
{
    // autoscale
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;

    // set to free camera
    cam.type = mjCAMERA_FREE;
}



// copy qpos to clipboard as key
void copykey(void)
{
    char clipboard[5000] = "<key qpos='";
    char buf[200];

    // prepare string
    for( int i=0; i<m->nq; i++ )
    {
        sprintf(buf, i==m->nq-1 ? "%g" : "%g ", d->qpos[i]);
        strcat(clipboard, buf);
    }
    strcat(clipboard, "'/>");

    // copy to clipboard
    glfwSetClipboardString(window, clipboard);
}



// millisecond timer, for MuJoCo built-in profiler
mjtNum timer(void)
{
    return (mjtNum)(1000*glfwGetTime());
}



// clear all times
void cleartimers()
{
    for( int i=0; i<mjNTIMER; i++ )
    {
        d->timer[i].duration = 0;
        d->timer[i].number = 0;
		d_closedloop->timer[i].duration = 0;
		d_closedloop->timer[i].number = 0;
		d_openloop->timer[i].duration = 0;
		d_openloop->timer[i].number = 0;
    }
}



// update UI 0 when MuJoCo structures change (except for joint sliders)
void updatesettings(void)
{
    int i;

    // physics flags
    for( i=0; i<mjNDISABLE; i++ )
        settings.disable[i] = ((m->opt.disableflags & (1<<i)) !=0 );
    for( i=0; i<mjNENABLE; i++ )
        settings.enable[i] = ((m->opt.enableflags & (1<<i)) !=0 );

    // camera
    if( cam.type==mjCAMERA_FIXED )
        settings.camera = 2 + cam.fixedcamid;
    else if( cam.type==mjCAMERA_TRACKING )
        settings.camera = 1;
    else 
        settings.camera = 0;

    // update UI
    mjui_update(-1, -1, &ui0, &uistate, &con);
}



// drop file callback
void drop(GLFWwindow* window, int count, const char** paths)
{
    // make sure list is non-empty
    if( count>0 )
    {
        strcpy(modelfilename, paths[0]);
        settings.loadrequest = 1;
    }
}

void loadmodel(void)
{
	// clear request
	settings.loadrequest = 0;

	// make sure filename is not empty
	if (!modelfilename[0])
		return;

	// load and compile
	char error[500] = "";
	mjModel* mnew = 0;
	if (strlen(modelfilename)>4 && !strcmp(modelfilename + strlen(modelfilename) - 4, ".mjb"))
	{
		mnew = mj_loadModel(modelfilename, NULL);
		if (!mnew)
			strcpy(error, "could not load binary model");
	}
	else
		mnew = mj_loadXML(modelfilename, NULL, error, 500);
	if (!mnew)
	{
		printf("%s\n", error);
		return;
	}

	// compiler warning: print and pause
	if (error[0])
	{
		// mj_forward() below will print the warning message
		printf("Model compiled, but simulation warning (paused):\n  %s\n\n",
			error);
		settings.run = 0;
	}

	// delete old model, assign new
	mj_deleteData(d);
	mj_deleteData(d_closedloop);
	mj_deleteData(d_openloop);
	mj_deleteModel(m);
	m = mnew;
	d = mj_makeData(m);
	d_closedloop = mj_makeData(m);
	d_openloop = mj_makeData(m);
	mj_forward(m, d);
	mj_forward(m, d_closedloop);
	mj_forward(m, d_openloop);

	// re-create scene and context
	mjv_makeScene(m, &scn_openloop, kMaxGeom);
	mjv_makeScene(m, &scn_closedloop, kMaxGeom);
	mjv_makeScene(m, &scn, kMaxGeom);
	glfwMakeContextCurrent(window_openloop);
	mjr_makeContext(m, &con_openloop, 50 * (settings.font + 1));
	glfwMakeContextCurrent(window_closedloop);
	mjr_makeContext(m, &con_closedloop, 50 * (settings.font + 1));
	glfwMakeContextCurrent(window);
	mjr_makeContext(m, &con, 50 * (settings.font + 1));
	
	// clear perturbation state
	pert.active = 0;
	pert.select = 0;
	pert.skinselect = -1;
	
	// align and scale view, update scene
	alignscale();
	mjv_updateScene(m, d_openloop, &vopt, &pert, &cam, mjCAT_ALL, &scn_openloop);
	mjv_updateScene(m, d_closedloop, &vopt, &pert, &cam, mjCAT_ALL, &scn_closedloop);
	mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);
	// set window title to model name
	/*if (window && m->names)
	{
		char title[200] = "Simulate : ";
		strcat(title, m->names);
		glfwSetWindowTitle(window, title);
	}*/

	// set keyframe range and divisions
	ui0.sect[SECT_SIMULATION].item[6].slider.range[0] = 0;
	ui0.sect[SECT_SIMULATION].item[6].slider.range[1] = mjMAX(0, m->nkey - 1);
	ui0.sect[SECT_SIMULATION].item[6].slider.divisions = mjMAX(1, m->nkey - 1);
	
	// rebuild UI sections
	makesections();

	// full ui update
	uiModify(window, &ui0, &uistate, &con);
	uiModify(window, &ui1, &uistate, &con);
	updatesettings();
}

//--------------------------------- UI hooks (for uitools.c) ----------------------------

// determine enable/disable item state given category
int uiPredicate(int category, void* userdata)
{
    switch( category )
    {
    case 2:                 // require model
        return (m!=NULL);

    case 3:                 // require model and nkey
        return (m && m->nkey);

    case 4:                 // require model and paused
        return (m && !settings.run);

    default:
        return 1;
    }
}


// set window layout
void uiLayout(mjuiState* state)
{
    mjrRect* rect = state->rect;

    // set number of rectangles
    state->nrect = 6;

    // rect 0: entire framebuffer
    rect[0].left = 0;
    rect[0].bottom = 0;
    glfwGetFramebufferSize(window, &rect[0].width, &rect[0].height);

    // rect 1: UI 0
    rect[1].left = 0;
    rect[1].width = settings.ui0 ? ui0.width : 0;
    rect[1].bottom = 0;
    rect[1].height = rect[0].height;

    // rect 2: UI 1
    rect[2].width = settings.ui1 ? ui1.width : 0;
    rect[2].left = mjMAX(0, rect[0].width - rect[2].width);
    rect[2].bottom = 0;
    rect[2].height = rect[0].height;

    // rect 3: 3D plot (everything else is an overlay)
    rect[3].left = rect[1].width;
    rect[3].width = mjMAX(0, rect[0].width - rect[1].width - rect[2].width);
    rect[3].bottom = 0;
    rect[3].height = rect[0].height;

	// rect 4: entire framebuffer
	rect[4].left = 0;
	rect[4].bottom = 0;
	glfwGetFramebufferSize(window_openloop, &rect[4].width, &rect[4].height);

	// rect 5 entire framebuffer
	rect[5].left = 0;
	rect[5].bottom = 0;
	glfwGetFramebufferSize(window_closedloop, &rect[5].width, &rect[5].height);
}



// handle UI event
void uiEvent(mjuiState* state)
{
    int i;
    char err[200];

    // call UI 0 if event is directed to it
    if( (state->dragrect==ui0.rectid) ||
        (state->dragrect==0 && state->mouserect==ui0.rectid) ||
        state->type==mjEVENT_KEY )
    {
        // process UI event
        mjuiItem* it = mjui_event(&ui0, state, &con);

        // file section
        if( it && it->sectionid==SECT_FILE )
        {
            switch( it->itemid )
            {
            case 0:             // Save xml
                if( !mj_saveLastXML("mjmodel.xml", m, err, 200) )
                    printf("Save XML error: %s", err);
                break;

            case 1:             // Save mjb
                mj_saveModel(m, "mjmodel.mjb", NULL, 0);
                break;

            case 2:             // Print model
                mj_printModel(m, "MJMODEL.TXT");
                break;

            case 3:             // Print data
                mj_printData(m, d, "MJDATA.TXT");
                break;

            case 4:             // Quit
                settings.exitrequest = 1;
                break;
            }
        }

        // option section
        else if( it && it->sectionid==SECT_OPTION )
        {
            switch( it->itemid )
            {
            case 0:             // Spacing
                ui0.spacing = mjui_themeSpacing(settings.spacing);
                ui1.spacing = mjui_themeSpacing(settings.spacing);
                break;

            case 1:             // Color
                ui0.color = mjui_themeColor(settings.color);
                ui1.color = mjui_themeColor(settings.color);
                break;

            case 2:             // Font
				mjr_changeFont(50 * (settings.font + 1), &con);
				mjr_changeFont(50 * (settings.font + 1), &con_openloop);
				mjr_changeFont(50 * (settings.font + 1), &con_closedloop);
                break;

			case 9:             // Full screen
				if (glfwGetWindowMonitor(window))
				{
					// restore window from saved data
					glfwSetWindowMonitor(window, NULL, windowpos[0], windowpos[1],
						windowsize[0], windowsize[1], 0);
					glfwSetWindowMonitor(window_closedloop, NULL, windowpos[0] + (2 * vmode.width) / 6, windowpos[1],
						windowsize[0], windowsize[1], 0);
					glfwSetWindowMonitor(window_openloop, NULL, windowpos[0] - (2 * vmode.width) / 6, windowpos[1],
						windowsize[0], windowsize[1], 0);
				}

				// currently windowed: switch to full screen
				else
				{
					// save window data
					glfwGetWindowPos(window, windowpos, windowpos + 1);
					glfwGetWindowSize(window, windowsize, windowsize + 1);

					// switch
					glfwSetWindowMonitor(window_closedloop, glfwGetPrimaryMonitor(), 0, 0,
						vmode.width, vmode.height, vmode.refreshRate);
					glfwSetWindowMonitor(window_openloop, glfwGetPrimaryMonitor(), 0, 0,
						vmode.width, vmode.height, vmode.refreshRate);
					glfwSetWindowMonitor(window, glfwGetPrimaryMonitor(), 0, 0,
						vmode.width, vmode.height, vmode.refreshRate);
				}

				// reinstante vsync, just in case
				glfwSwapInterval(settings.vsync);
				break;

            case 10:            // Vertical sync
                glfwSwapInterval(settings.vsync);
                break;
            }

            // modify UI
			uiModify(window, &ui0, state, &con);
            uiModify(window, &ui1, state, &con);
        }

        // simulation section
        else if( it && it->sectionid==SECT_SIMULATION )
        {
            switch( it->itemid )
            {
            case 1:             // Reset
                if( m )
                {
					step_index_nominal = 0;
					step_index_closedloop = 0;
					step_index_openloop = 0;
					terminal_trigger = false;
                    mj_resetData(m, d);
                    mj_forward(m, d);
					mj_resetData(m, d_openloop);
					mj_forward(m, d_openloop);
					mj_resetData(m, d_closedloop);
					mj_forward(m, d_closedloop);
                    profilerupdate();
                    sensorupdate();
					updatesettings();
					for (int e = 0; e < stepnum * actuatornum; e++)
					{
						ctrl_openloop[e] = ctrl_nominal[e] + perturb_coefficient_std * ctrl_max * randGauss(0, 1);
						delta_rest_length[e] = perturb_coefficient_std * randGauss(0, 1);
					}
                }
                break;

            case 2:             // Reload
                settings.loadrequest = 1;
                break;

            case 3:             // Align
                alignscale();
                updatesettings();
                break;

            case 4:             // Copy pose
                copykey();
                break;

            case 5:             // Adjust key
            case 6:             // Reset to key
                i = settings.key;
                d->time = m->key_time[i];
                mju_copy(d->qpos, m->key_qpos+i*m->nq, m->nq);
                mju_copy(d->qvel, m->key_qvel+i*m->nv, m->nv);
                mju_copy(d->act, m->key_act+i*m->na, m->na);
                mj_forward(m, d);
                profilerupdate();
                sensorupdate();
                updatesettings();
                break;

            case 7:             // Set key
                i = settings.key;
                m->key_time[i] = d->time;
                mju_copy(m->key_qpos+i*m->nq, d->qpos, m->nq);
                mju_copy(m->key_qvel+i*m->nv, d->qvel, m->nv);
                mju_copy(m->key_act+i*m->na, d->act, m->na);
                break;
            }
        }

        // physics section
        else if( it && it->sectionid==SECT_PHYSICS )
        {
            // update disable flags in mjOption
            m->opt.disableflags = 0;
            for( i=0; i<mjNDISABLE; i++ )
                if( settings.disable[i] )
                    m->opt.disableflags |= (1<<i);

            // update enable flags in mjOption
            m->opt.enableflags = 0;
            for( i=0; i<mjNENABLE; i++ )
                if( settings.enable[i] )
                    m->opt.enableflags |= (1<<i);
        }

        // rendering section
        else if( it && it->sectionid==SECT_RENDERING )
        {
            // set camera in mjvCamera
            if( settings.camera==0 )
                cam.type = mjCAMERA_FREE;
            else if( settings.camera==1 )
            {
                if( pert.select>0 )
                {
                    cam.type = mjCAMERA_TRACKING;
                    cam.trackbodyid = pert.select;
                    cam.fixedcamid = -1;
                }
                else
                {
                    cam.type = mjCAMERA_FREE;
                    settings.camera = 0;
                    mjui_update(SECT_RENDERING, -1, &ui0, &uistate, &con);
                }
            }
            else
            {
                cam.type = mjCAMERA_FIXED;
                cam.fixedcamid = settings.camera - 2;
            }
        }

        // group section
        else if( it && it->sectionid==SECT_GROUP )
        {
            // remake joint section if joint group changed
            if( it->name[0]=='J' && it->name[1]=='o' )
            {
                ui1.nsect = SECT_JOINT;
                makejoint(ui1.sect[SECT_JOINT].state);
                ui1.nsect = NSECT1;
				uiModify(window, &ui1, state, &con);
            }

            // remake control section if actuator group changed
            if( it->name[0]=='A' && it->name[1]=='c' )
            {
                ui1.nsect = SECT_CONTROL;
                makecontrol(ui1.sect[SECT_CONTROL].state);
                ui1.nsect = NSECT1;
                uiModify(window, &ui1, state, &con);
            }
        }

        // stop if UI processed event
        if( it!=NULL || (state->type==mjEVENT_KEY && state->key==0) )
            return;
    }

    // call UI 1 if event is directed to it
    if( (state->dragrect==ui1.rectid) ||
        (state->dragrect==0 && state->mouserect==ui1.rectid) ||
        state->type==mjEVENT_KEY )
    {
        // process UI event
        mjuiItem* it = mjui_event(&ui1, state, &con);

        // control section
        if( it && it->sectionid==SECT_CONTROL )
        {
            // clear controls
            if( it->itemid==0 )
            {
                mju_zero(d->ctrl, m->nu);
				mju_zero(d_openloop->ctrl, m->nu);
				mju_zero(d_closedloop->ctrl, m->nu);
                mjui_update(SECT_CONTROL, -1, &ui1, &uistate, &con);
            }
        }

        // stop if UI processed event
        if( it!=NULL || (state->type==mjEVENT_KEY && state->key==0) )
            return;
    }

    // shortcut not handled by UI
    if( state->type==mjEVENT_KEY && state->key!=0 )
    {
        switch( state->key )
        {
        case ' ':                   // Mode
            if( m )
            {
                settings.run = 1 - settings.run;
                pert.active = 0;
                mjui_update(-1, -1, &ui0, state, &con);
            }
            break;

        case mjKEY_RIGHT:           // step forward
            if( m && !settings.run )
            {
                cleartimers();
                mj_step(m, d);
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_LEFT:            // step back
            if( m && !settings.run )
            {
                m->opt.timestep = -m->opt.timestep;
                cleartimers();
                mj_step(m, d);
                m->opt.timestep = -m->opt.timestep;
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_DOWN:            // step forward 100
            if( m && !settings.run )
            {
                cleartimers();
                for( i=0; i<100; i++ )
                    mj_step(m, d);
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_UP:              // step back 100
            if( m && !settings.run )
            {
                m->opt.timestep = -m->opt.timestep;
                cleartimers();
                for( i=0; i<100; i++ )
                    mj_step(m, d);
                m->opt.timestep = -m->opt.timestep;
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_PAGE_UP:         // select parent body
            if( m && pert.select>0 )
            {
                pert.select = m->body_parentid[pert.select];
                pert.skinselect = -1;

                // stop perturbation if world reached
                if( pert.select<=0 )
                    pert.active = 0;
            }

            break;

        case mjKEY_ESCAPE:          // free camera
            cam.type = mjCAMERA_FREE;
            settings.camera = 0;
            mjui_update(SECT_RENDERING, -1, &ui0, &uistate, &con);
            break;
        }

        return;
    }

    // 3D scroll
    if( state->type==mjEVENT_SCROLL && state->mouserect==3 && m )
    {
        // emulate vertical mouse motion = 5% of window height
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*state->sy, &scn, &cam);
		mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*state->sy, &scn_openloop, &cam);
		mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*state->sy, &scn_closedloop, &cam);

        return;
    }

    // 3D press
    if( state->type==mjEVENT_PRESS && state->mouserect==3 && m )
    {
        // set perturbation
        int newperturb = 0;
        if( state->control && pert.select>0 )
        {
            // right: translate;  left: rotate
            if( state->right )
                newperturb = mjPERT_TRANSLATE;
            else if( state->left )
                newperturb = mjPERT_ROTATE;

            // perturbation onset: reset reference
			if (newperturb && !pert.active) {
				mjv_initPerturb(m, d, &scn, &pert);
				mjv_initPerturb(m, d_openloop, &scn_openloop, &pert);
				mjv_initPerturb(m, d_closedloop, &scn_closedloop, &pert);
			}
        }
        pert.active = newperturb;

        // handle double-click
        if( state->doubleclick )
        {
            // determine selection mode
            int selmode;
            if( state->button==mjBUTTON_LEFT )
                selmode = 1;
            else if( state->control )
                selmode = 3;
            else
                selmode = 2;

            // find geom and 3D click point, get corresponding body
            mjrRect r = state->rect[3];
            mjtNum selpnt[3];
            int selgeom, selskin;
            int selbody = mjv_select(m, d, &vopt,
                                     (mjtNum)r.width/(mjtNum)r.height, 
                                     (mjtNum)(state->x-r.left)/(mjtNum)r.width, 
                                     (mjtNum)(state->y-r.bottom)/(mjtNum)r.height, 
                                     &scn, selpnt, &selgeom, &selskin);

            // set lookat point, start tracking is requested
            if( selmode==2 || selmode==3 )
            {
                // copy selpnt if anything clicked
                if( selbody>=0 )
                    mju_copy3(cam.lookat, selpnt);

                // switch to tracking camera if dynamic body clicked
                if( selmode==3 && selbody>0 )
                {
                    // mujoco camera
                    cam.type = mjCAMERA_TRACKING;
                    cam.trackbodyid = selbody;
                    cam.fixedcamid = -1;

                    // UI camera
                    settings.camera = 1;
                    mjui_update(SECT_RENDERING, -1, &ui0, &uistate, &con);
                }
            }
            // set body selection
            else
            {
                if( selbody>=0 )
                {
                    // record selection
                    pert.select = selbody;
                    pert.skinselect = selskin;

                    // compute localpos
                    mjtNum tmp[3];
                    mju_sub3(tmp, selpnt, d->xpos+3*pert.select);
                    mju_mulMatTVec(pert.localpos, d->xmat+9*pert.select, tmp, 3, 3);
                }
                else
                {
                    pert.select = 0;
                    pert.skinselect = -1;
                }
            }

            // stop perturbation on select
            pert.active = 0;
        }

        return;
    }

    // 3D release
    if( state->type==mjEVENT_RELEASE && state->dragrect==3 && m )
    {
        // stop perturbation
        pert.active = 0;

        return;
    }

    // 3D move
    if( state->type==mjEVENT_MOVE && state->dragrect==3 && m )
    {
        // determine action based on mouse button
        mjtMouse action;
        if( state->right )
            action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else if( state->left )
            action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else
            action = mjMOUSE_ZOOM;

        // move perturb or camera
        mjrRect r = state->rect[3];
        if( pert.active )
            mjv_movePerturb(m, d, action, state->dx/r.height, -state->dy/r.height, 
                            &scn, &pert);
        else
            mjv_moveCamera(m, action, state->dx/r.height, -state->dy/r.height, 
                           &scn, &cam);

        return;
    }
}

//--------------------------- control and testing ---------------------------------------
// simulate the nominal trajectory
void simulateNominal(void)
{
	if (step_index_nominal == 0) {
		modelInit(m, d, state_nominal[0]);
	}
	if (step_index_nominal >= stepnum)
	{
		terminalCtrl(m, d, step_index_nominal);

		// print N_final to be the target for the analytical shape control
		if (NFinal == true) {
			for (int i = 0; i < 3 * m->nsite; i++) printf("%.4f\n", d->site_xpos[i]);
			//for (int i = 0; i < m->nsensordata; i++) printf(" d->sensordata[%d]        : %.4f\n", i, d->sensordata[i]);
			//for (int i = 0; i < 3 * m->nsite; i++) printf(" d->site_xpos[%d]        : %.4f\n", i, d->site_xpos[i]);
			//for (int i = 0; i < m->nq; i++) printf(" d->qpos[%d]        : %.8f\n", i, d->qpos[i]);
			//for (int i = 0; i < m->nv; i++) printf(" d->qvel[%d]        : %.8f\n", i, d->qvel[i]);
			NFinal = false;
		}
	}
	else mju_copy(d->ctrl, &ctrl_nominal[step_index_nominal * actuatornum], m->nu);
	ctrlLimit(d->ctrl, m->nu);//for (int i = 0; i < m->nq; i++) printf(" d->qpos[%d]        : %.8f\n", i, d->qpos[i]);
    // state perturbation
	//d->qpos[0] += randGauss(0, 0.0001);
	//d->qpos[1] += randGauss(0, 0.000001);
	//d->qpos[3] += randGauss(0, 0.0001);
	//d->qpos[4] += randGauss(0, 0.0001);
	//d->qpos[5] += randGauss(0, 0.0001);
	//d->qpos[8] += randGauss(0, 0.000001);
	//d->qpos[9] += randGauss(0, 0.0001);
	//d->qpos[10] += randGauss(0, 0.0001);
	//d->qpos[11] += randGauss(0, 0.0001);
	//d->qpos[15] += randGauss(0, 0.0001);
	//d->qpos[21] += randGauss(0, 0.0001);
	//d->qpos[12] += randGauss(0, 0.0001);
	//d->qpos[18] += randGauss(0, 0.0001);
	//d->qpos[2] = -2 * d->qpos[1];
	//d->qpos[13] = -d->qpos[1];
	//d->qpos[19] = -d->qpos[1];
	//d->qpos[14] = -d->qpos[2];
	//d->qpos[20] = -d->qpos[2];
	//d->qpos[6] += randGauss(0, 0.000001);// d->qpos[1];
	//d->qpos[7] = -2*(d->qpos[6]-d->qpos[1]);
	//d->qpos[16] = -d->qpos[6];
	//d->qpos[22] = -d->qpos[6];
	//d->qpos[17] = -d->qpos[7];
	//d->qpos[23] = -d->qpos[7];

	for (int i = 0; i < integration_per_step; i++) mj_step(m, d);
	step_index_nominal++;
}

// simulate with closed-loop control policy under noise
bool simulateClosedloop(void)
{
	static mjtNum state_error[kMaxState], ctrl_feedback[kMaxState], ctrl_temp[kMaxState];

	if (step_index_closedloop == 0) {
		modelInit(m, d_closedloop, state_nominal[0]);
		cost_closedloop = 0;
		energy = 0;
	}

	if (terminal_trigger == false) terminal_trigger = terminalTrigger(m, d_closedloop, modelid, step_index_closedloop);
	
	if (terminal_trigger == true)
	{
		terminalCtrl(m, d_closedloop, step_index_closedloop);
		if (_strcmpi(testmode, "policy_compare") == 0 && step_index_closedloop >= stepnum) {
			cost_closedloop += stepCost(m, d_closedloop, stepnum);
			step_index_closedloop = 0; 
			terminal_trigger = false;
			return 1;
		}
		cost_closedloop += stepCost(m, d_closedloop, step_index_closedloop);
	}
	else {
		mju_sub(state_error, state_nominal[step_index_closedloop], d_closedloop->qpos, dof + quatnum);
		mju_sub(&state_error[dof + quatnum], &state_nominal[step_index_closedloop][dof + quatnum], d_closedloop->qvel, dof);
		if (modelid == 14) {
			state_error[2] = state_nominal[step_index_closedloop][2] - d_closedloop->qpos[15];
			state_error[7] = state_nominal[step_index_closedloop][7] - d_closedloop->qpos[18];
			state_error[13] = state_nominal[step_index_closedloop][13] - d_closedloop->qpos[21];
			state_error[16] = state_nominal[step_index_closedloop][16] - d_closedloop->qvel[15];
			state_error[21] = state_nominal[step_index_closedloop][21] - d_closedloop->qvel[18];
			state_error[27] = state_nominal[step_index_closedloop][27] - d_closedloop->qvel[21];
		}
		mju_mulMatVec(ctrl_feedback, *tracker_feedback_gain[step_index_closedloop], state_error, kMaxState, kMaxState);
		mju_add(d_closedloop->ctrl, &ctrl_openloop[step_index_closedloop * actuatornum], ctrl_feedback, m->nu);
		ctrlLimit(d_closedloop->ctrl, m->nu);
		mju_add(ctrl_temp, &ctrl_nominal[step_index_closedloop * actuatornum], ctrl_feedback, m->nu);
		ctrlLimit(ctrl_temp, m->nu);
		energy += mju_dot(ctrl_temp, ctrl_temp, m->nu);
		cost_closedloop += stepCost(m, d_closedloop, step_index_closedloop);
	}
	for (int i = 0; i < integration_per_step; i++) mj_step(m, d_closedloop); //printf("%f\t%f\t%f\t%f\t%f\t%f\t%d\n", d->qpos[0], d_closedloop->qpos[0], d->qpos[1],  d_closedloop->qpos[1], d->qpos[2], d_closedloop->qpos[2], terminal_trigger);
	step_index_closedloop++;																		 // process noise including state noise
	//mju_add(d_closedloop->qpos, d_closedloop->qpos, randGauss(0, 0.00023, dof + quatnum), dof + quatnum);
	//mju_add(d_closedloop->qvel, d_closedloop->qvel, randGauss(0, 0.00023, dof), dof);
	//mj_forward(m, d_closedloop);
	return 0;
}

// simulate with open-loop control policy under noise
bool simulateOpenloop(void)
{
	if (step_index_openloop == 0) {
		modelInit(m, d_openloop, state_nominal[0]);
		cost_openloop = 0;
	}
	if (step_index_openloop >= stepnum)
	{
		terminalCtrl(m, d_openloop, step_index_openloop);
		if (_strcmpi(testmode, "policy_compare") == 0) {
			cost_openloop += stepCost(m, d_openloop, stepnum);
			step_index_openloop = 0;
			return 1;
		}
	}
	else {
		mju_copy(d_openloop->ctrl, &ctrl_openloop[step_index_openloop * actuatornum], m->nu);
		ctrlLimit(d_openloop->ctrl, m->nu);
		cost_openloop += stepCost(m, d_openloop, step_index_openloop);
	}
	for (int i = 0; i < integration_per_step; i++) mj_step(m, d_openloop);
	step_index_openloop++;
	//printf("%f,%f,%f\n", d_openloop->qpos[0]-state_nominal[step_index_openloop][0], d_openloop->qpos[1]- state_nominal[step_index_openloop][1], d_openloop->qvel[1]- state_nominal[step_index_openloop][9]);
	return 0;
}

// simulate the rest length control trajectory
void simulateRestLength(void)
{
	if (step_index_nominal == 0) {
		modelInit(m, d, state_nominal[0]);
	}
	if (step_index_nominal >= stepnum)
	{
		for (int i = 0; i < actuatornum; i++)
			m->tendon_lengthspring[i] = rest_length[(stepnum - 1) * actuatornum + i];

		// print N_final to be the target for the analytical shape control
		if (NFinal == true) {
			for (int i = 0; i < 3 * m->nsite; i++) printf("%.4f\n", d->site_xpos[i]);
			//for (int i = 0; i < m->nsensordata; i++) printf(" d->sensordata[%d]        : %.4f\n", i, d->sensordata[i]);
			//for (int i = 0; i < 3 * m->nsite; i++) printf(" d->site_xpos[%d]        : %.4f\n", i, d->site_xpos[i]);
			//for (int i = 0; i < m->nq; i++) printf(" d->qpos[%d]        : %.8f\n", i, d->qpos[i]);
			//for (int i = 0; i < m->nv; i++) printf(" d->qvel[%d]        : %.8f\n", i, d->qvel[i]);
			NFinal = false;
		}
	}
	else {
		for (int i = 0; i < actuatornum; i++)
			m->tendon_lengthspring[i] = rest_length[step_index_nominal * actuatornum + i] + delta_rest_length[step_index_nominal * actuatornum + i];
	}
	for (int i = 0; i < integration_per_step; i++) mj_step(m, d);
	step_index_nominal++;
}

// calculate the distance from the target at the terminal step
mjtNum terminalError(mjtNum ptb, const char *type)
{
	mjtNum state_error[kMaxState], ctrl_feedback[kMaxState];

	modelInit(m, d_closedloop, state_nominal[0]);
	terminal_trigger = false;
	for (int e = 0; e < stepnum * actuatornum; e++) ctrl_openloop[e] = ctrl_nominal[e] + ptb * ctrl_max * randGauss(0, 1);

	for (int step_index = 0; step_index < stepnum; step_index++) {
		if (_strcmpi(type, "openloop") == 0) mju_copy(d_closedloop->ctrl, &ctrl_openloop[step_index * actuatornum], m->nu);
		else {
			if (terminal_trigger == false) terminal_trigger = terminalTrigger(m, d_closedloop, modelid, step_index);
			if (terminal_trigger == true) {
				terminalCtrl(m, d_closedloop, step_index);
				mju_add(d_closedloop->ctrl, d_closedloop->ctrl, ctrl_openloop, m->nu);
				mju_sub(d_closedloop->ctrl, d_closedloop->ctrl, ctrl_nominal, m->nu);
			}
			else {
				mju_sub(state_error, state_nominal[step_index], d_closedloop->qpos, dof + quatnum);
				mju_sub(&state_error[dof + quatnum], &state_nominal[step_index][dof + quatnum], d_closedloop->qvel, dof);
				mju_mulMatVec(ctrl_feedback, *tracker_feedback_gain[step_index], state_error, kMaxState, kMaxState);
				mju_add(d_closedloop->ctrl, &ctrl_openloop[step_index * actuatornum], ctrl_feedback, m->nu);
				ctrlLimit(d_closedloop->ctrl, m->nu);
			}
		}
		for (int i = 0; i < integration_per_step; i++) mj_step(m, d_closedloop);
	}
	if (modelid == 0)
		return sqrt(angleModify(modelid, d_closedloop->qpos[0])*angleModify(modelid, d_closedloop->qpos[0]) + d_closedloop->qvel[0] * d_closedloop->qvel[0]);
	else if (modelid == 2)
		return sqrt((d_closedloop->geom_xpos[6] - 0.6) * (d_closedloop->geom_xpos[6] - 0.6) + (d_closedloop->geom_xpos[7] + 0.6) * (d_closedloop->geom_xpos[7] + 0.6));
	else if (modelid == 4)			   
		return sqrt(d_closedloop->site_xpos[19] * d_closedloop->site_xpos[19] + (d_closedloop->site_xpos[20] - 2.5) * (d_closedloop->site_xpos[20] - 2.5));
	else if (modelid == 5)
		return sqrt((d_closedloop->site_xpos[30] - d_closedloop->site_xpos[0]) * (d_closedloop->site_xpos[30] - d_closedloop->site_xpos[0]) + (d_closedloop->site_xpos[32] - d_closedloop->site_xpos[2]) * (d_closedloop->site_xpos[32] - d_closedloop->site_xpos[2]));
	else if (modelid == 6)
		return sqrt((d_closedloop->site_xpos[93] - d_closedloop->site_xpos[0]) * (d_closedloop->site_xpos[93] - d_closedloop->site_xpos[0]) + (d_closedloop->site_xpos[95] - d_closedloop->site_xpos[2]) * (d_closedloop->site_xpos[95] - d_closedloop->site_xpos[2]));
	else if (modelid == 7)
		return sqrt((d_closedloop->qpos[0] - d_closedloop->site_xpos[0]) * (d_closedloop->qpos[0] - d_closedloop->site_xpos[0]) + (d_closedloop->qpos[1] - d_closedloop->site_xpos[1]) * (d_closedloop->qpos[1] - d_closedloop->site_xpos[1]));
	else if (modelid == 8)
		return sqrt((d_closedloop->site_xpos[39] - d_closedloop->site_xpos[9]) * (d_closedloop->site_xpos[39] - d_closedloop->site_xpos[9]) + (d_closedloop->site_xpos[41] - d_closedloop->site_xpos[11]) * (d_closedloop->site_xpos[41] - d_closedloop->site_xpos[11]));													   
	else if (modelid == 9)
		return sqrt((d_closedloop->site_xpos[75] - d_closedloop->site_xpos[9]) * (d_closedloop->site_xpos[75] - d_closedloop->site_xpos[9]) + (d_closedloop->site_xpos[77] - d_closedloop->site_xpos[11]) * (d_closedloop->site_xpos[77] - d_closedloop->site_xpos[11]));
	else if (modelid == 10)
		return sqrt((d_closedloop->geom_xpos[11] - d_closedloop->geom_xpos[5]) * (d_closedloop->geom_xpos[11] - d_closedloop->geom_xpos[5]) + (d_closedloop->geom_xpos[10] - d_closedloop->geom_xpos[4]) * (d_closedloop->geom_xpos[10] - d_closedloop->geom_xpos[4]) + (d_closedloop->geom_xpos[9] - d_closedloop->geom_xpos[3]) * (d_closedloop->geom_xpos[9] - d_closedloop->geom_xpos[3]));
	else if (modelid == 11)
		return sqrt((d_closedloop->site_xpos[3] - d_closedloop->site_xpos[12]) * (d_closedloop->site_xpos[3] - d_closedloop->site_xpos[12]) + (d_closedloop->site_xpos[4] - d_closedloop->site_xpos[13]) * (d_closedloop->site_xpos[4] - d_closedloop->site_xpos[13]) + (d_closedloop->site_xpos[5] - d_closedloop->site_xpos[14]) * (d_closedloop->site_xpos[5] - d_closedloop->site_xpos[14]));
	else if (modelid == 12)
		return sqrt((d_closedloop->site_xpos[3] - d_closedloop->site_xpos[15]) * (d_closedloop->site_xpos[3] - d_closedloop->site_xpos[15]) + (d_closedloop->site_xpos[4] - d_closedloop->site_xpos[16]) * (d_closedloop->site_xpos[4] - d_closedloop->site_xpos[16]) + (d_closedloop->site_xpos[5] - d_closedloop->site_xpos[17]) * (d_closedloop->site_xpos[5] - d_closedloop->site_xpos[17]));
	else if (modelid == 13)
		return sqrt((d_closedloop->geom_xpos[6] - 0.6) * (d_closedloop->geom_xpos[6] - 0.6) + (d_closedloop->geom_xpos[7] + 0.6) * (d_closedloop->geom_xpos[7] + 0.6));
	else if (modelid == 15)
		return sqrt(d_closedloop->qpos[0]* d_closedloop->qpos[0] + angleModify(modelid, d_closedloop->qpos[1])*angleModify(modelid, d_closedloop->qpos[1]) + mju_dot(d_closedloop->qvel, d_closedloop->qvel, dof));
	return 0;
}

// collect data for comparing the distance from the target at the terminal step between closed-loop policy and open-loop policy under different levels of noise
void performanceTest(void)
{
	double printfraction = 0.2;

	strcpy(datafilename, "perfcheck.txt");
	if ((filestream1 = fopen(datafilename, "wt+")) != NULL)
	{
		for (mjtNum ptb = 0; ptb < 1.0001; ptb += 0.05)
		{
			for (int i = 0; i < kTestNum; i++)
			{
				// data output for checking
				sprintf(data_buff, "%4.8f", terminalError(ptb, "closedloop"));
				fwrite(data_buff, 10, 1, filestream1);
				fputs(" ", filestream1);
			}
			fputs("\n", filestream1);

			//for (int i = 0; i < kTestNum; i++)
			//{
			//	sprintf(data_buff, "%4.8f", terminalError(ptb, "openloop"));
			//	fwrite(data_buff, 10, 1, filestream1);
			//	fputs(" ", filestream1);
			//}
			//fputs("\n", filestream1);

			if (ptb >= 1 * printfraction)
			{
				printf(".");
				printfraction += 0.2;
			}
		}
		fclose(filestream1);
	}
	else printf("Could not open file: perfcheck.txt");
}

// compare the expected cost between the closed-loop policy, the open-loop policy and the model-based shape control under different levels of noise 
void policyCompare()
{
	double printfraction = 0.2;
	
	strcpy(datafilename, "clopdata.txt");
	if ((filestream1 = fopen(datafilename, "wt+")) != NULL)
	{
		strcpy(datafilename, "energydata.txt");
		if ((filestream2 = fopen(datafilename, "wt+")) != NULL)
		{
			for (double ptb = 0; ptb <= 1.00001; ptb += 0.05)
			{
				for (int i = 0; i < kTestNum; i++)
				{
					for (int e = 0; e < stepnum * actuatornum; e++) ctrl_openloop[e] = ctrl_nominal[e] + ptb * ctrl_max * randGauss(0, 1);

					while (simulateOpenloop() != 1);
					while (simulateClosedloop() != 1);

					sprintf(data_buff, "%4.8f", cost_closedloop);
					fwrite(data_buff, 10, 1, filestream1);
					fputs("\n", filestream1);
					sprintf(data_buff, "%4.8f", cost_openloop);
					fwrite(data_buff, 10, 1, filestream1);
					fputs("\n", filestream1);

					sprintf(data_buff, "%4.8f", energy);
					fwrite(data_buff, 10, 1, filestream2);
					fputs(" ", filestream2);
				}
				fputs("\n", filestream2);

				if (ptb >= 1 * printfraction)
				{
					printf(".");
					printfraction += 0.2;
				}
			}
			fclose(filestream2);
		}
		else printf("Could not open file: energydata.txt\n");
		fclose(filestream1);
	}else printf("Could not open file: clopdata.txt\n");
}

// show info for the model
void modelTest(void)
{
	mj_resetData(m, d);
	mj_forward(m, d);
	printf("\n Model file name : %s\n", modelfilename);
	printf(" m->nq             : %d\n", m->nq);
	printf(" m->nv             : %d\n", m->nv);
	printf(" m->nu             : %d\n", m->nu);
	printf(" m->nsite          : %d\n", m->nsite);
	for (int i = 0; i < m->nq; i++) printf(" d->qpos[%d]        : %.2f\n", i, d->qpos[i]);
	for (int i = 0; i < 3*m->nsite; i++) printf(" d->site_xpos[%d]   : %.2f\n", i, d->site_xpos[i]);
}

// run a certain mode
void testModeSelection(const char* mode)
{
	strcpy(testmode, mode);
	if (_strcmpi(testmode, "policy_compare") == 0) {
		policyCompare();
		exit(0);
	}
	else if (_strcmpi(testmode, "performance_test") == 0) {
		performanceTest();
		exit(0);
	}
	else if (_strcmpi(testmode, "modeltest") == 0)
		modelTest();
	else if (_strcmpi(testmode, "restlength") == 0)
		printf("Controlling the rest length...\n");
	else if (_strcmpi(testmode, "nfinal") == 0)
		NFinal = true;
	else if (_strcmpi(testmode, "top") == 0) {
		stepnum = 0;
		mju_add(state_nominal[0], state_target, randGauss(0, 0.0001, 2 * dof + quatnum), 2 * dof + quatnum);
	}
}

//--------------------------- rendering and simulation ----------------------------------

// sim thread synchronization
std::mutex mtx;


// prepare to render
void prepare(void)
{
    // data for FPS calculation
    static double lastupdatetm = 0;

    // update interval, save update time
    double tmnow = glfwGetTime();
    double interval = tmnow - lastupdatetm;
    interval = mjMIN(1, mjMAX(0.0001, interval));
    lastupdatetm = tmnow;

    // no model: nothing to do
    if( !m )
        return;

    // update scene
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);
	mjv_updateScene(m, d_openloop, &vopt, &pert, &cam, mjCAT_ALL, &scn_openloop);
	mjv_updateScene(m, d_closedloop, &vopt, &pert, &cam, mjCAT_ALL, &scn_closedloop);
	
	// update watch 
    if( settings.ui0 && ui0.sect[SECT_WATCH].state )
    {
		watch();
		mjui_update(SECT_WATCH, -1, &ui0, &uistate, &con);
    }

    // update joint
	if (settings.ui1 && ui1.sect[SECT_JOINT].state) {
		mjui_update(SECT_JOINT, -1, &ui1, &uistate, &con);
	}

    // update info text
	if (settings.info) {
		infotext(d, info_title, info_content, interval);
		infotext(d_openloop, info_title, info_content_openloop, interval);
		infotext(d_closedloop, info_title, info_content_closedloop, interval);
	}

    // update profiler
    if( settings.profiler && settings.run )
        profilerupdate();

    // update sensor
    if( settings.sensor && settings.run )
        sensorupdate();

    // clear timers once profiler info has been copied
    cleartimers();
}

// render in main thread (while simulating in background thread)
// parameter window is for callback
void render(GLFWwindow* window)
{
    // get 3D rectangle and reduced for profiler
    mjrRect rect = uistate.rect[3];
    mjrRect smallrect = rect;
    if( settings.profiler )
        smallrect.width = rect.width - rect.width/4;

    // no model
    if( !m )
    {
        // blank screen
        mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);

        // label
        if( settings.loadrequest )
            mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect, 
                        "loading", NULL, &con);
        else
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, 
                        "Drag-and-drop model file here", 0, &con);

        // render uis
        if( settings.ui0 )
            mjui_render(&ui0, &uistate, &con);
        if( settings.ui1 )
            mjui_render(&ui1, &uistate, &con);

        // finalize
        glfwSwapBuffers(window); 

        return;
    }


    // render scene
    mjr_render(rect, &scn, &con);

    // show pause/loading label
    if( !settings.run || settings.loadrequest )
        mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect, 
                    settings.loadrequest ? "loading" : "pause", NULL, &con);

	// show ui 0
	if (settings.ui0)
		mjui_render(&ui0, &uistate, &con);

	// show ui 1
	if (settings.ui1)
		mjui_render(&ui1, &uistate, &con);

    // show help
    if( settings.help )
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title, help_content, &con);

    // show info
    if( settings.info )
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, 
                    info_title, info_content, &con);

    // show profiler
    if( settings.profiler )
        profilershow(rect);

    // show sensor
    if( settings.sensor )
        sensorshow(smallrect);

    // finalize
    glfwSwapBuffers(window);
}

void renderOpenloop(GLFWwindow* window)
{
	// get 3D rectangle and reduced for profiler
	mjrRect rect = uistate.rect[4];
	mjrRect smallrect = uistate.rect[3];

	// no model
	if (!m) return;
	
	// render scene
	mjr_render(rect, &scn_openloop, &con_openloop);

	//// show pause/loading label
	//if (!settings.run || settings.loadrequest)
	//	mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect,
	//		settings.loadrequest ? "loading" : "pause", NULL, &con_openloop);

	// show info
	if (settings.info)
		mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect,
			info_title, info_content_openloop, &con_openloop);

	// finalize
	glfwSwapBuffers(window);
}

void renderClosedloop(GLFWwindow* window)
{
	// get 3D rectangle and reduced for profiler
	mjrRect rect = uistate.rect[5];
	mjrRect smallrect = uistate.rect[3];

	// no model
	if (!m) return;

	// render scene
	mjr_render(rect, &scn_closedloop, &con_closedloop);

	//// show pause/loading label
	//if (!settings.run || settings.loadrequest)
	//	mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect,
	//		settings.loadrequest ? "loading" : "pause", NULL, &con_closedloop);

	// show info
	if (settings.info)
		mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect,
			info_title, info_content_closedloop, &con_closedloop);

	// finalize
	glfwSwapBuffers(window);
}


// simulate in background thread (while rendering in main thread)
void simulate(void)
{
    // cpu-sim syncronization point
    double cpusync = 0;
    mjtNum simsync = 0;

    // run until asked to exit
	srand((unsigned)time(NULL));
    while( !settings.exitrequest )
    {
        // sleep for 1 ms or yield, to let main thread run
        //  yield results in busy wait - which has better timing but kills battery life
        if( settings.run && settings.busywait )
            std::this_thread::yield();
		else
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // start exclusive access
        mtx.lock();

        // run only if model is present
        if( m )
        {
            // record start time
            double startwalltm = glfwGetTime();

            // running
            if( settings.run )
            {
                // record cpu time at start of iteration
                double tmstart = glfwGetTime();

                // out-of-sync (for any reason)
                if( d->time<simsync || tmstart<cpusync || cpusync==0 ||
                    mju_abs((d->time-simsync)-(tmstart-cpusync))>syncmisalign )
                {
                    // re-sync
                    cpusync = tmstart;
                    simsync = d->time;

                    // clear old perturbations, apply new
                    mju_zero(d->xfrc_applied, 6*m->nbody);
					mjv_applyPerturbPose(m, d, &pert, 0);  // move mocap bodies only
					mjv_applyPerturbForce(m, d, &pert);

                    // run single step, let next iteration deal with timing
                    //mj_step(m, d);
                }

                // in-sync
                else
                {
                    // step while simtime lags behind cputime, and within safefactor
					while ((d_openloop->time - simsync) < (glfwGetTime() - cpusync) && (d_closedloop->time - simsync) < (glfwGetTime() - cpusync) && (d->time - simsync) < (glfwGetTime() - cpusync) &&
						(glfwGetTime() - tmstart) < refreshfactor / vmode.refreshRate)
					{
						// clear old perturbations, apply new
						mju_zero(d->xfrc_applied, 6 * m->nbody);
						mjv_applyPerturbPose(m, d, &pert, 0);  // move mocap bodies only
						mjv_applyPerturbForce(m, d, &pert);
						mju_zero(d_closedloop->xfrc_applied, 6 * m->nbody);
						mjv_applyPerturbPose(m, d_closedloop, &pert, 0);
						mjv_applyPerturbForce(m, d_closedloop, &pert);
						mju_zero(d_openloop->xfrc_applied, 6 * m->nbody);
						mjv_applyPerturbPose(m, d_openloop, &pert, 0);
						mjv_applyPerturbForce(m, d_openloop, &pert);

						// run mj_step
						mjtNum prevtm = d->time;
						if (_strcmpi(testmode, "modeltest") == 0) {
							mj_step(m, d);
						}
						else if (_strcmpi(testmode, "restlength") == 0) {
							simulateRestLength();
							//for (int i = 0; i < actuatornum; i++)
							//	m->tendon_lengthspring[i] = rest_length[i];
							////m->tendon_lengthspring[0] = 1.79999999995395;
							////m->tendon_lengthspring[1] = 1.69707951914804;
							////m->tendon_lengthspring[2] = 2.05316229103641;
							////m->tendon_lengthspring[3] = 2.05316229103641;
							////m->tendon_lengthspring[4] = 2.20454076848204;
							////m->tendon_lengthspring[5] = 2.20454076846339;
							////m->tendon_lengthspring[6] = 2.10292140654738;
							//mj_step(m, d);
						}
						else {
							simulateNominal();
							simulateOpenloop();
							simulateClosedloop();
						}

						//m->tendon_lengthspring[0] = 0.5;
						//for (int i = 0; i < m->nq; i++) d->qpos[i] = -PI / 36;
						//for (int i = 0; i < m->nq; i++) printf(" d->qpos[%d]        : %.2f\n", i, d->qpos[i]);
						//printf("%f \n", d->qpos[20] - (d->qpos[9]+d->qpos[7] + d->qpos[10]));
						//for (int i = 0; i < m->nv; i++) d->ctrl[i] = -5;
						//mjtNum tempq[4] = { 0 };
						//for (int i = 0; i < 4; i++) tempq[i] = d->qpos[3+i]+randGauss(0, 0.00005);
						//mju_normalize4(tempq);
						//for (int i = 0; i < 4; i++) d->qpos[3 + i] = tempq[i];
						//mj_step(m, d); printf("%f ", d->qpos[3]); printf("%f ", d->qpos[4]); printf("%f ", d->qpos[5]); printf("%f\n", d->qpos[6]);
						//for (int i = 0; i < 4; i++) d->xquat[4 + i] = state_nominal[0][3+i];
						//for (int i = 3; i < 7; i++) d->qpos[i] = state_nominal[0][i]; 
						//printf("%f ", d->qpos[3]- state_nominal[0][3]); printf("%f ", d->qpos[4] - state_nominal[0][4]); printf("%f ", d->qpos[5] - state_nominal[0][5]); printf("%f\n", d->qpos[6] - state_nominal[0][6]);
						//printf("%f ", state_nominal[0][3]); printf("%f ", state_nominal[0][4]); printf("%f ", state_nominal[0][5]); printf("%f\n", state_nominal[0][6]);
						//d->efc_force[0] += 10; 
						//mj_step(m, d); mj_forward(m, d); 
						//printf("%f, ", d->efc_force[0]);
						//printf("%f ", d->xquat[4]); printf("%f ", d->xquat[5]); printf("%f ", d->xquat[6]); printf("%f\n", d->xquat[7]);
						//mjtNum temp1[kMaxState] = { 0.9534626,0.0953463,0,0.2860388, 0.9534626,0.0953463,0,0.2860388, 0.92123,0.20498,0.33023,0.01662, 0.9534626,0.0953463,0, 0.92123,0.20498,0.33023 };
						//for (int y = 0; y < 4; y++) d->qpos[y] = temp1[y];
						//for (int y = 4; y < 8; y++) d->qpos[y+4] = temp1[y];
						//for (int y = 8; y < 12; y++) d->qpos[y+8] = temp1[y];
						//for (int y = 0; y < 3; y++) d->qvel[y] = temp1[y + dof + quatnum];
						//for (int y = 3; y < 6; y++) d->qvel[y+3] = temp1[y + dof + quatnum];
						//mj_step(m, d); 
						////mj_forward(m, d);
						//for (int i = 8; i < 12; i++) printf(" d->qpos[%d]        : %.5f\n", i, d->qpos[i]);
						//for (int i = 0; i < 3; i++) printf(" d->qvel[%d]        : %.5f\n", i, d->qvel[i]);
						//static int c = 0;
						//if (c == 0) { d->ctrl[0] = -1; d->ctrl[1] = -2.41; d->ctrl[2] = -1; d->ctrl[3] = -1; }
						//else { d->ctrl[0] = -1; d->ctrl[1] = -2.42; d->ctrl[2] = -1; d->ctrl[3] = -1; }
						//c++;
						//mj_step(m, d);
						
                        // break on reset
                        if( d->time<prevtm )
                            break;
                    }
                }
            }

            // paused
            else
            {
                // apply pose perturbation
                mjv_applyPerturbPose(m, d, &pert, 1);      // move mocap and dynamic bodies
				mjv_applyPerturbPose(m, d_closedloop, &pert, 1);
				mjv_applyPerturbPose(m, d_openloop, &pert, 1);

                // run mj_forward, to update rendering and joint sliders
                mj_forward(m, d);
				mj_forward(m, d_closedloop);
				mj_forward(m, d_openloop);
            }
        }

        // end exclusive access
        mtx.unlock();
    }
}

//-------------------------------- init and main ----------------------------------------

// initalize
void init(void)
{
	// print version, check compatibility
	printf("MuJoCo Pro version %.2lf\n", 0.01*mj_version());
	if (mjVERSION_HEADER != mj_version())
		mju_error("Headers and library have different versions");

	// activate MuJoCo license
	DWORD usernamesize = 30;
	GetUserName(username, &usernamesize);
	if (username[0] == 'R') {
		strcpy(keyfilename, keyfilepre);
		strcat(keyfilename, "mjkeybig.txt");
		mj_activate(keyfilename);
	}
	else if (username[0] == 'r') {
		strcpy(keyfilename, keyfilepre);
		strcat(keyfilename, "mjkeyda.txt");
		mj_activate(keyfilename);
	}
    else {
        strcpy(keyfilename, keyfilepre);
        strcat(keyfilename, "mjkeysmall.txt");
        mj_activate(keyfilename);
    }

	// read nominal control values
	strcpy(datafilename, "result0.txt");
	if ((filestream1 = fopen(datafilename, "r")) != NULL)
	{
		for (int i = 0; i < actuatornum * stepnum; i++)
		{
			fscanf(filestream1, "%s", data_buff);
			ctrl_nominal[i] = atof(data_buff);
			if (fabs(ctrl_nominal[i]) > ctrl_max) ctrl_max = fabs(ctrl_nominal[i]); // find umax
		}
		fclose(filestream1);
	}
	else printf("Could not open file: result0.txt\n");

	// read rest length values
	strcpy(datafilename, "length0.txt");
	if ((filestream1 = fopen(datafilename, "r")) != NULL)
	{
		for (int i = 0; i < actuatornum * stepnum; i++)
		{
			fscanf(filestream1, "%s", data_buff);
			rest_length[i] = atof(data_buff);
			if (fabs(rest_length[i]) > ctrl_max) ctrl_max = fabs(rest_length[i]); // find umax
		}
		fclose(filestream1);
	}
	else printf("Could not open file: length0.txt\n");

	// read feedback gain K
	strcpy(datafilename, "TK.txt");
	if ((filestream1 = fopen(datafilename, "r")) != NULL)
	{
		for (int i1 = 0; i1 < stepnum; i1++) {
			for (int i2 = 0; i2 < actuatornum; i2++) {
				for (int i = 0; i < 2*dof+quatnum; i++) {
					fscanf(filestream1, "%s", data_buff);
					tracker_feedback_gain[i1][i2][i] = atof(data_buff);
				}
			}
		}
		fclose(filestream1);
	}
	else printf("Could not open file: TK.txt\n");

	strcpy(datafilename, "TK_top.txt");
	if ((filestream1 = fopen(datafilename, "r")) != NULL)
	{
			for (int i2 = 0; i2 < actuatornum; i2++) {
				for (int i = 0; i < 2 * dof + quatnum; i++) {
					fscanf(filestream1, "%s", data_buff);
					stabilizer_feedback_gain[i2][i] = atof(data_buff);
				}
			}
		fclose(filestream1);
	}

	// read open-loop training cost parameters
	strcpy(datafilename, "parameters.txt");
	if ((filestream1 = fopen(datafilename, "r")) != NULL) {
		while (!feof(filestream1))
		{
			fscanf(filestream1, "%s", data_buff);
			if (data_buff[1] == '_')
			{
				fscanf(filestream1, "%s", data_buff);
				Q = atof(data_buff);
			}
			if (data_buff[0] == 'R')
			{
				fscanf(filestream1, "%s", data_buff);
				R = atof(data_buff);
			}
			if (data_buff[1] == 'T')
			{
				fscanf(filestream1, "%s", data_buff);
				QT = atof(data_buff);
			}
		}
		for (int i = 0; i < 2*dof+quatnum; i++)
		{
			QTm[i][i] = 1 * QT;
			Qm[i][i] = 1 * Q;
		}
		fclose(filestream1);
	}
	else printf("Could not open file: parameters.txt\n");

	// init GLFW, set timer callback (milliseconds)
	if (!glfwInit())
		mju_error("Could not initialize GLFW");
	mjcb_time = timer;

	// multisampling
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_VISIBLE, 1);

	// get videomode and save
	vmode = *glfwGetVideoMode(glfwGetPrimaryMonitor());

	// create window
	window = glfwCreateWindow((2 * vmode.width) / 6, (2 * vmode.height) / 4,
		"Nominal", NULL, NULL);
	window_openloop = glfwCreateWindow((2 * vmode.width) / 6, (2 * vmode.height) / 4,
		"Openloop with noise", NULL, NULL);
	window_closedloop = glfwCreateWindow((2 * vmode.width) / 6, (2 * vmode.height) / 4,
		"Closedloop with noise", NULL, NULL);

	if (!window)
	{
		glfwTerminate();
		mju_error("Could not create window");
	}

	glfwSetWindowPos(window, (2 * vmode.width) / 6, 100);
	glfwSetWindowPos(window_openloop, 0, 100);
	glfwSetWindowPos(window_closedloop, (4 * vmode.width) / 6, 100);

	// save window position and size
	glfwGetWindowPos(window, windowpos, windowpos + 1);
	glfwGetWindowSize(window, windowsize, windowsize + 1);

	// make context current, set v-sync
	glfwMakeContextCurrent(window);
	glfwSwapInterval(settings.vsync);

	// init abstract visualization
	mjv_defaultCamera(&cam);
	mjv_defaultOption(&vopt);
	profilerinit();
	sensorinit();

	// make empty scene
	mjv_defaultScene(&scn);
	mjv_makeScene(NULL, &scn, kMaxGeom);

	// select default font
	int fontscale = uiFontScale(window);
	settings.font = fontscale / 50 - 1;

	// make empty context
	mjr_defaultContext(&con);
	mjr_makeContext(NULL, &con, fontscale);

	// set GLFW callbacks
	uiSetCallback(window, &uistate, uiEvent, uiLayout);
	glfwSetWindowRefreshCallback(window, render);
	glfwSetDropCallback(window, drop);

	// switch window and do the same thing
	uiSetCallback(window_closedloop, &uistate, uiEvent, uiLayout);
	glfwSetWindowRefreshCallback(window_closedloop, renderClosedloop);
	glfwSetDropCallback(window_closedloop, drop);
	uiSetCallback(window_openloop, &uistate, uiEvent, uiLayout);
	glfwSetWindowRefreshCallback(window_openloop, renderOpenloop);
	glfwSetDropCallback(window_openloop, drop);

	// init state and uis
	memset(&uistate, 0, sizeof(mjuiState));
	memset(&ui0, 0, sizeof(mjUI));
	memset(&ui1, 0, sizeof(mjUI));
	ui0.spacing = mjui_themeSpacing(settings.spacing);
	ui0.color = mjui_themeColor(settings.color);
	ui0.predicate = uiPredicate;
	ui0.rectid = 1;
	ui0.auxid = 0;
	ui1.spacing = mjui_themeSpacing(settings.spacing);
	ui1.color = mjui_themeColor(settings.color);
	ui1.predicate = uiPredicate;
	ui1.rectid = 2;
	ui1.auxid = 1;

	// populate uis with standard sections
	mjui_add(&ui0, defFile);
	mjui_add(&ui0, defOption);
	mjui_add(&ui0, defSimulation);
	mjui_add(&ui0, defWatch);
	uiModify(window, &ui0, &uistate, &con);
	uiModify(window, &ui1, &uistate, &con);
}


// run event loop
int main(int argc, const char** argv)
{
	// print help if arguments are missing
	if (argc < 4 || argc > 7) {
		printf("\n Usage: test2d modelfile control_timestep stepnum [modeltype [mode [noiselevel]]]\n");
		return 0;
	}

	// process input from command window4
	if (argc > 1)
	{
		strcpy(modelfilename, argv[1]);
		strncpy(modelname, modelfilename, strlen(modelfilename) - 4);
		settings.loadrequest = 1;
	}
	if (argc > 4 && modelSelection(argv[4]) == 1);
	else modelSelection(modelname);
	if (sscanf(argv[2], "%lf", &control_timestep) != 1 || control_timestep <= 0) {
		printf("Invalid control_timestep argument");
		return 0;
	}
	if (sscanf(argv[3], "%d", &stepnum) != 1 || stepnum <= 0) {
		printf("Invalid stepnum argument");
		return 0;
	}
	srand((unsigned)time(NULL));

	// initialize
	init();
	loadmodel();

	// check timestep setting
	simulation_timestep = m->opt.timestep;
	integration_per_step = (int)(control_timestep / simulation_timestep);
	if (integration_per_step <= 0) {
		printf("Invalid timestep setting");
		return 0;
	}

	stateNominal(m, d);

	if (argc > 4) if (sscanf(argv[4], "%lf", &perturb_coefficient_std) != 1) testModeSelection(argv[4]);

	if (argc > 5) if (sscanf(argv[5], "%lf", &perturb_coefficient_std) != 1) testModeSelection(argv[5]);

	if (argc > 6) sscanf(argv[6], "%lf", &perturb_coefficient_std);
	for (int e = 0; e < stepnum * actuatornum; e++)
	{
		ctrl_openloop[e] = ctrl_nominal[e] + perturb_coefficient_std * ctrl_max * randGauss(0, 1);
		delta_rest_length[e] = perturb_coefficient_std * randGauss(0, 1);
	}
	
    // start simulation thread
    std::thread simthread(simulate);

    // event loop
    while( !glfwWindowShouldClose(window) && !glfwWindowShouldClose(window_closedloop) && !glfwWindowShouldClose(window_openloop) && !settings.exitrequest )
    {
        // start exclusive access (block simulation thread)
        mtx.lock();
		
        // load model (not on first pass, to show "loading" label)
		if (settings.loadrequest == 1) {
			loadmodel();
		}
        else if( settings.loadrequest>1 )
            settings.loadrequest = 1;

        // handle events (calls all callbacks)
        glfwPollEvents();
		
        // prepare to render
        prepare();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

		// render while simulation is running
		glfwMakeContextCurrent(window_openloop);
		renderOpenloop(window_openloop);

		glfwMakeContextCurrent(window_closedloop);
		renderClosedloop(window_closedloop);

		glfwMakeContextCurrent(window);
		render(window);
    }
	
    // stop simulation thread
    settings.exitrequest = 1;
    simthread.join();

    // delete everything we allocated
    uiClearCallback(window);
	uiClearCallback(window_closedloop);
	uiClearCallback(window_openloop);
    mj_deleteData(d); 
	mj_deleteData(d_openloop);
	mj_deleteData(d_closedloop);
    mj_deleteModel(m);
    mjv_freeScene(&scn); 
	mjv_freeScene(&scn_openloop);
	mjv_freeScene(&scn_closedloop);
    mjr_freeContext(&con); 
	mjr_freeContext(&con_openloop);
	mjr_freeContext(&con_closedloop);

    // deactivate MuJoCo
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 0;
}
