#ifndef __c14_SS6_Estimation_h__
#define __c14_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc14_SS6_EstimationInstanceStruct
#define typedef_SFc14_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c14_sfEvent;
  boolean_T c14_isStable;
  boolean_T c14_doneDoubleBufferReInit;
  uint8_T c14_is_active_c14_SS6_Estimation;
  real_T *c14_FzL;
  real_T *c14_F_SL;
  real_T *c14_F_tL;
  real_T *c14_U;
  real_T *c14_V;
  real_T *c14_Slip;
  real_T *c14_r;
  real_T *c14_a_fL;
  real_T *c14_a_rL;
  real_T *c14_a_mL;
  real_T *c14_F_SL_f;
  real_T *c14_F_SL_r;
  real_T *c14_F_SL_m;
} SFc14_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc14_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c14_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c14_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c14_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
