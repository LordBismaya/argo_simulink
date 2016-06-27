#ifndef __c8_SS6_Estimation_h__
#define __c8_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc8_SS6_EstimationInstanceStruct
#define typedef_SFc8_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c8_sfEvent;
  boolean_T c8_isStable;
  boolean_T c8_doneDoubleBufferReInit;
  uint8_T c8_is_active_c8_SS6_Estimation;
  real_T *c8_U;
  real_T *c8_V;
  real_T *c8_theta;
  real_T *c8_X_dot;
  real_T *c8_Y_dot;
  real_T *c8_V_SS;
} SFc8_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc8_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c8_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c8_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c8_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
