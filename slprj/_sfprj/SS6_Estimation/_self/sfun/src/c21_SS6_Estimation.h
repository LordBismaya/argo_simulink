#ifndef __c21_SS6_Estimation_h__
#define __c21_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc21_SS6_EstimationInstanceStruct
#define typedef_SFc21_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c21_sfEvent;
  boolean_T c21_isStable;
  boolean_T c21_doneDoubleBufferReInit;
  uint8_T c21_is_active_c21_SS6_Estimation;
  real_T c21_cov[49];
  boolean_T c21_cov_not_empty;
  real_T c21_State[7];
  boolean_T c21_State_not_empty;
  real_T *c21_X_GPS;
  real_T *c21_Y_GPS;
  real_T *c21_aX_IMU;
  real_T *c21_aY_IMU;
  real_T *c21_r_IMU;
  real_T *c21_X_2;
  real_T *c21_Y_2;
} SFc21_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc21_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c21_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c21_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c21_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
