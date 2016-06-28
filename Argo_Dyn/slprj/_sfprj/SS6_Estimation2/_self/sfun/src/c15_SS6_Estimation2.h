#ifndef __c15_SS6_Estimation2_h__
#define __c15_SS6_Estimation2_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc15_SS6_Estimation2InstanceStruct
#define typedef_SFc15_SS6_Estimation2InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c15_sfEvent;
  boolean_T c15_isStable;
  boolean_T c15_doneDoubleBufferReInit;
  uint8_T c15_is_active_c15_SS6_Estimation2;
  real_T *c15_FzR;
  real_T *c15_F_SR;
  real_T *c15_F_tR;
  real_T *c15_U;
  real_T *c15_V;
  real_T *c15_Slip;
  real_T *c15_r;
  real_T *c15_a_fR;
  real_T *c15_a_rR;
  real_T *c15_a_mR;
  real_T *c15_F_SR_f;
  real_T *c15_F_SR_r;
  real_T *c15_F_SR_m;
} SFc15_SS6_Estimation2InstanceStruct;

#endif                                 /*typedef_SFc15_SS6_Estimation2InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c15_SS6_Estimation2_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c15_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
extern void c15_SS6_Estimation2_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
