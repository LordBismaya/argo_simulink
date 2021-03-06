#ifndef __c17_SS6_Estimation2_h__
#define __c17_SS6_Estimation2_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc17_SS6_Estimation2InstanceStruct
#define typedef_SFc17_SS6_Estimation2InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c17_sfEvent;
  boolean_T c17_isStable;
  boolean_T c17_doneDoubleBufferReInit;
  uint8_T c17_is_active_c17_SS6_Estimation2;
  real_T *c17_U;
  real_T *c17_V;
  real_T *c17_U_dot;
  real_T *c17_V_dot;
  real_T *c17_r;
  real_T *c17_F_tL;
  real_T *c17_F_tR;
  real_T *c17_F_sL;
  real_T *c17_F_sR;
  real_T *c17_r_dot;
  real_T *c17_F_SL_f;
  real_T *c17_F_SL_r;
  real_T *c17_F_SL_m;
  real_T *c17_F_SR_f;
  real_T *c17_F_SR_r;
  real_T *c17_F_SR_m;
} SFc17_SS6_Estimation2InstanceStruct;

#endif                                 /*typedef_SFc17_SS6_Estimation2InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c17_SS6_Estimation2_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c17_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
extern void c17_SS6_Estimation2_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
