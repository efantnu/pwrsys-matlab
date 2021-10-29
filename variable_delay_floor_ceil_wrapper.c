
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include <stdlib.h>
#include <string.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void variable_delay_floor_ceil_Start_wrapper(real_T *xD,
			void **pW,
			const real_T *windowSize, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Allocate the temporal window, the window will be accessed like a circular
 * queue FIFO structure. discrete state xD[0] is used to store the current
 * index.
 */
    pW[0] = malloc( windowSize[0] * sizeof(real_T) );
    memset( pW[0], '0', windowSize[0] * sizeof(real_T) );
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void variable_delay_floor_ceil_Outputs_wrapper(const real_T *in,
			const real_T *n,
			real_T *delayed_f,
			real_T *delayed_c,
			const real_T *xD,
			void **pW,
			const real_T *windowSize, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
//insertion pointer
int idx = (int)xD[0];
    
//circular buffer
real_T *window = (real_T *)pW[0];
    
//retrieval pointer : floor
int nf = (int)(floor(n[0]));
int idxf = (idx - nf) % (int)windowSize[0];
// -4 % 10 = -4; 14 % 10 = 4 
while (idxf < 0)
{
    idxf += (int)windowSize[0];
}    
    
//retrieval pointer : ceiling
int nc = (int)(ceil(n[0]));
int idxc = (idx - nc) % (int)windowSize[0];
// -4 % 10 = -4; 14 % 10 = 4 
while (idxc < 0)
{
    idxc += (int)windowSize[0];
}    
    
//update circular buffer with new value
window[idx] = in[0];
       
//retrieve old values of circular buffer
delayed_f[0] = (real_T)window[idxf];
delayed_c[0] = (real_T)window[idxc];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void variable_delay_floor_ceil_Update_wrapper(const real_T *in,
			const real_T *n,
			real_T *delayed_f,
			real_T *delayed_c,
			real_T *xD,
			void **pW,
			const real_T *windowSize, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * The discrete state xD[0] is used to store the index in the circular queue
 * FIFO structure, and is updated every step by circular increment.
 */

    uint64_T idx = (uint64_T)xD[0];
    
    idx = (idx+1) % (uint64_T)windowSize[0];
    xD[0] = (real_T)idx;
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Terminate function
 *
 */
void variable_delay_floor_ceil_Terminate_wrapper(real_T *xD,
			void **pW,
			const real_T *windowSize, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Free the memory allocated in Start.
 */
    free(pW[0]);
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

