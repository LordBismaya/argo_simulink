#ifndef __c11_SS6_Estimation_h__
#define __c11_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc11_SS6_EstimationInstanceStruct
#define typedef_SFc11_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c11_sfEvent;
  boolean_T c11_isStable;
  boolean_T c11_doneDoubleBufferReInit;
  uint8_T c11_is_active_c11_SS6_Estimation;
  real_T c11_cov[49];
  boolean_T c11_cov_not_empty;
  real_T c11_State[7];
  boolean_T c11_State_not_empty;
  real_T *c11_X_1;
  real_T *c11_Y_1;
  real_T *c11_X_GPS;
  real_T *c11_Y_GPS;
  real_T *c11_aX_IMU;
  real_T *c11_aY_IMU;
  real_T *c11_r_IMU;
  real_T *c11_V_x;
  real_T *c11_V_y;
} SFc11_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc11_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c11_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c11_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c11_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
