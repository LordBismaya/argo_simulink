#ifndef __c19_SS6_Estimation_h__
#define __c19_SS6_Estimation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc19_SS6_EstimationInstanceStruct
#define typedef_SFc19_SS6_EstimationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c19_sfEvent;
  boolean_T c19_isStable;
  boolean_T c19_doneDoubleBufferReInit;
  uint8_T c19_is_active_c19_SS6_Estimation;
  real_T c19_Vx;
  boolean_T c19_Vx_not_empty;
  real_T c19_Vy;
  boolean_T c19_Vy_not_empty;
  real_T c19_yaw;
  boolean_T c19_yaw_not_empty;
  real_T c19_X;
  boolean_T c19_X_not_empty;
  real_T c19_Y;
  boolean_T c19_Y_not_empty;
  real_T *c19_aX;
  real_T *c19_aY;
  real_T *c19_X_pos;
  real_T *c19_Y_pos;
  real_T *c19_r;
} SFc19_SS6_EstimationInstanceStruct;

#endif                                 /*typedef_SFc19_SS6_EstimationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c19_SS6_Estimation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c19_SS6_Estimation_get_check_sum(mxArray *plhs[]);
extern void c19_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
