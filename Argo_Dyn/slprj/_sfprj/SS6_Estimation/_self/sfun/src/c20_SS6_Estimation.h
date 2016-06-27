#ifndef __c20_SS6_Estimation_h__
#define __c20_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc20_SS6_EstimationInstanceStruct
#define typedef_SFc20_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c20_sfEvent;
  boolean_T c20_isStable;
  boolean_T c20_doneDoubleBufferReInit;
  uint8_T c20_is_active_c20_SS6_Estimation;
  real_T *c20_U;
  real_T *c20_V;
  real_T *c20_theta;
  real_T *c20_X_dot;
  real_T *c20_Y_dot;
  real_T *c20_V_SS;
} SFc20_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc20_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c20_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c20_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c20_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
