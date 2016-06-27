#ifndef __c1_SS6_Estimation_h__
#define __c1_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_SS6_EstimationInstanceStruct
#define typedef_SFc1_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_SS6_Estimation;
  uint32_T c1_method;
  boolean_T c1_method_not_empty;
  uint32_T c1_state[2];
  boolean_T c1_state_not_empty;
  uint32_T c1_b_method;
  boolean_T c1_b_method_not_empty;
  uint32_T c1_b_state;
  boolean_T c1_b_state_not_empty;
  uint32_T c1_c_state[2];
  boolean_T c1_c_state_not_empty;
  uint32_T c1_d_state[625];
  boolean_T c1_d_state_not_empty;
  real_T *c1_x;
  real_T *c1_y;
  real_T *c1_X_GPS;
  real_T *c1_Y_GPS;
} SFc1_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc1_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c1_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
