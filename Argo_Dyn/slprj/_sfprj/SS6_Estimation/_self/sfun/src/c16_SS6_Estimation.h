#ifndef __c16_SS6_Estimation_h__
#define __c16_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc16_SS6_EstimationInstanceStruct
#define typedef_SFc16_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c16_sfEvent;
  boolean_T c16_isStable;
  boolean_T c16_doneDoubleBufferReInit;
  uint8_T c16_is_active_c16_SS6_Estimation;
  real_T *c16_U_dot;
  real_T *c16_V;
  real_T *c16_Fz_L;
  real_T *c16_Fz_R;
  real_T *c16_r;
} SFc16_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc16_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c16_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c16_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c16_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
