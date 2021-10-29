

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
 * Output functions
 *
 */
void pi_std_antiwindup_Outputs_wrapper(real_T *out,
			real_T *sOutAtMax,
			real_T *sOutAtMin,
			real_T *sIAtMax,
			real_T *sIAtMin,
			real_T *Punlim,
			real_T *I,
			const real_T *xD,
			const real_T  *Ts, const int_T  p_width0,
			const real_T  *Kg, const int_T  p_width1,
			const real_T  *Kp, const int_T  p_width2,
			const real_T  *Ti, const int_T  p_width3,
			const real_T  *outMax, const int_T  p_width4,
			const real_T  *outMin, const int_T  p_width5)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* Auxiliary variables */
real_T outUnlim, outLim;
real_T Icontrib, Pcontrib;

Pcontrib = xD[0];
Icontrib = xD[1];

/* Calculate unlimited output */
outUnlim = Pcontrib + Icontrib;

/* Limit output */
if (outUnlim > *outMax) {
    outLim = *outMax;
}
else if (outUnlim < *outMin) {
    outLim = *outMin;
}
else {
    outLim = outUnlim;
}


/* Write outputs */
*out = outLim;

if (outLim == *outMax) {
    *sOutAtMax = 1;
}
else {
    *sOutAtMax = 0;
}

if (outLim == *outMin) {
    *sOutAtMin = 1;
}
else {
    *sOutAtMin = 0;
}

if (Icontrib == *outMax) {
    *sIAtMax = 1;
}
else {
    *sIAtMax = 0;
}

if (Icontrib == *outMin) {
    *sIAtMin = 1;
}
else {
    *sIAtMin = 0;
}

*Punlim = Pcontrib;
*I = Icontrib;
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
  * Updates function
  *
  */
void pi_std_antiwindup_Update_wrapper(const real_T *error,
			const real_T *cPoff,
			const real_T *cIhold,
			const real_T *cIforce,
			const real_T *Ivalue,
			real_T *out,
			real_T *sOutAtMax,
			real_T *sOutAtMin,
			real_T *sIAtMax,
			real_T *sIAtMin,
			real_T *Punlim,
			real_T *I,
			real_T *xD,
			const real_T  *Ts,  const int_T  p_width0,
			const real_T  *Kg,  const int_T  p_width1,
			const real_T  *Kp,  const int_T  p_width2,
			const real_T  *Ti,  const int_T  p_width3,
			const real_T  *outMax,  const int_T  p_width4,
			const real_T  *outMin,  const int_T  p_width5)
{
  /* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
/* Auxiliary variables */
real_T Igain, Icontrib;
real_T Pgain, Pcontrib;

/* Initialize P contribution */
Pcontrib = 0;

/* Initialize I contribution */
if (*cIforce > 0) {
    Icontrib = *Ivalue;
}
else {
    Icontrib = xD[1];
}

/* Calculate P gain */
Pgain = *Kg * *Kp;

/* Calculate I gain */
Igain = *Kg / *Ti;

/* Calculate proportional contribution */
if(*cPoff == 0) {
    Pcontrib = Pgain * *error;
}

/* Calculate integral contribution */
if((*cIhold == 0) && (*cIforce == 0) && 
    (Icontrib >= *outMin) && (Icontrib <= *outMax)) {
    Icontrib += Igain * *error * *Ts;
}

/* Limit integral contribution */
if (Icontrib > *outMax) {
    Icontrib = *outMax;
}  
if (Icontrib < *outMin) {
    Icontrib = *outMin;
}  

/* Update states */
xD[0] = Pcontrib;
xD[1] = Icontrib;
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}
