#ifndef __c18_SS6_Estimation_h__
#define __c18_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc18_SS6_EstimationInstanceStruct
#define typedef_SFc18_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c18_sfEvent;
  boolean_T c18_isStable;
  boolean_T c18_doneDoubleBufferReInit;
  uint8_T c18_is_active_c18_SS6_Estimation;
  real_T *c18_aXM_IMU;
  real_T *c18_aYM_IMU;
  real_T *c18_rM_IMU;
  real_T *c18_U_dot;
  real_T *c18_aX;
  real_T *c18_aY;
  real_T *c18_r;
  real_T *c18_V_dot;
  real_T *c18_r_eom;
  real_T *c18_UM_dot;
  real_T *c18_VM_dot;
  real_T *c18_rM_eom;
  real_T *c18_V1;
  real_T *c18_V2;
  real_T *c18_V3;
  real_T *c18_V4;
  real_T *c18_V5;
  real_T *c18_V6;
} SFc18_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc18_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c18_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c18_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c18_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
