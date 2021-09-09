

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
void notch_Outputs_wrapper(real_T *out,
			const real_T *xD,
			const real_T  *Ts, const int_T  p_width0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
*out = xD[0];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
  * Updates function
  *
  */
void notch_Update_wrapper(const real_T *in,
			const real_T *freq,
			const real_T *gmin,
			const real_T *damp,
			real_T *out,
			real_T *xD,
			const real_T  *Ts,  const int_T  p_width0)
{
  /* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
/* Auxiliary variables */
real_T wt, p, q, r, s, t, u, v;
real_T a, b, c, d;

wt = *freq * *Ts / 2;
p = 2 * *damp * wt;
q = wt * wt;
r = 1 + q;
s = p + r;
t = r / s; // (1 + wt^2) / (1 + 2*damp*wt + wt^2)
u = p / s; // 2*damp*wt / (1 + 2*damp*wt + wt^2)
v = *gmin * u;

a = t + v;
b = 2 * (1 - q) / s;
c = t - v;
d = t - u;

xD[0] = *in * a + xD[1];
xD[1] = (xD[0] - *in) * b + xD[2];
xD[2] = *in * c - xD[0] * d;
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}
