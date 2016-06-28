#ifndef __c10_SS6_Estimation2_h__
#define __c10_SS6_Estimation2_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc10_SS6_Estimation2InstanceStruct
#define typedef_SFc10_SS6_Estimation2InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c10_sfEvent;
  boolean_T c10_isStable;
  boolean_T c10_doneDoubleBufferReInit;
  uint8_T c10_is_active_c10_SS6_Estimation2;
  uint32_T c10_method;
  boolean_T c10_method_not_empty;
  uint32_T c10_state[2];
  boolean_T c10_state_not_empty;
  uint32_T c10_b_method;
  boolean_T c10_b_method_not_empty;
  uint32_T c10_b_state;
  boolean_T c10_b_state_not_empty;
  uint32_T c10_c_state[2];
  boolean_T c10_c_state_not_empty;
  uint32_T c10_d_state[625];
  boolean_T c10_d_state_not_empty;
  real_T *c10_w_L;
  real_T *c10_w_R;
  real_T *c10_count_L;
  real_T *c10_count_R;
} SFc10_SS6_Estimation2InstanceStruct;

#endif                                 /*typedef_SFc10_SS6_Estimation2InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c10_SS6_Estimation2_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c10_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
extern void c10_SS6_Estimation2_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
