#ifndef __c9_SS6_Estimation_h__
#define __c9_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc9_SS6_EstimationInstanceStruct
#define typedef_SFc9_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c9_sfEvent;
  boolean_T c9_isStable;
  boolean_T c9_doneDoubleBufferReInit;
  uint8_T c9_is_active_c9_SS6_Estimation;
  uint32_T c9_method;
  boolean_T c9_method_not_empty;
  uint32_T c9_state[2];
  boolean_T c9_state_not_empty;
  uint32_T c9_b_method;
  boolean_T c9_b_method_not_empty;
  uint32_T c9_b_state;
  boolean_T c9_b_state_not_empty;
  uint32_T c9_c_state[2];
  boolean_T c9_c_state_not_empty;
  uint32_T c9_d_state[625];
  boolean_T c9_d_state_not_empty;
  real_T *c9_aX;
  real_T *c9_aY;
  real_T *c9_aX_IMU;
  real_T *c9_aY_IMU;
  real_T *c9_yaw_rate;
  real_T *c9_yawRate_IMU;
} SFc9_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc9_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c9_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c9_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c9_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
