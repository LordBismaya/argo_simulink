#ifndef __c3_SS6_Estimation_h__
#define __c3_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc3_SS6_EstimationInstanceStruct
#define typedef_SFc3_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  boolean_T c3_isStable;
  boolean_T c3_doneDoubleBufferReInit;
  uint8_T c3_is_active_c3_SS6_Estimation;
  real_T *c3_FzR;
  real_T *c3_F_SR;
  real_T *c3_F_tR;
  real_T *c3_U;
  real_T *c3_V;
  real_T *c3_Slip;
  real_T *c3_r;
  real_T *c3_a_fR;
  real_T *c3_a_rR;
  real_T *c3_a_mR;
  real_T *c3_F_SR_f;
  real_T *c3_F_SR_r;
  real_T *c3_F_SR_m;
} SFc3_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc3_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c3_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c3_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif