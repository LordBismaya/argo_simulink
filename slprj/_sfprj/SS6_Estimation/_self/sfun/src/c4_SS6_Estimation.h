#ifndef __c4_SS6_Estimation_h__
#define __c4_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc4_SS6_EstimationInstanceStruct
#define typedef_SFc4_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c4_sfEvent;
  boolean_T c4_isStable;
  boolean_T c4_doneDoubleBufferReInit;
  uint8_T c4_is_active_c4_SS6_Estimation;
  real_T *c4_U;
  real_T *c4_V;
  real_T *c4_U_dot;
  real_T *c4_V_dot;
  real_T *c4_r;
  real_T *c4_F_tL;
  real_T *c4_F_tR;
  real_T *c4_F_sL;
  real_T *c4_F_sR;
  real_T *c4_r_dot;
  real_T *c4_F_SL_f;
  real_T *c4_F_SL_r;
  real_T *c4_F_SL_m;
  real_T *c4_F_SR_f;
  real_T *c4_F_SR_r;
  real_T *c4_F_SR_m;
} SFc4_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc4_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c4_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c4_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c4_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
