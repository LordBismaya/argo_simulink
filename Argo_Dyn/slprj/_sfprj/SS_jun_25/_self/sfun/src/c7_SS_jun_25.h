#ifndef __c7_SS_jun_25_h__
#define __c7_SS_jun_25_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc7_SS_jun_25InstanceStruct
#define typedef_SFc7_SS_jun_25InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c7_sfEvent;
  boolean_T c7_isStable;
  boolean_T c7_doneDoubleBufferReInit;
  uint8_T c7_is_active_c7_SS_jun_25;
  real_T *c7_FzL;
  real_T *c7_F_SL;
  real_T *c7_F_tL;
  real_T *c7_U;
  real_T *c7_V;
  real_T *c7_Slip;
  real_T *c7_delta;
  real_T *c7_r;
  real_T *c7_a_fL;
  real_T *c7_a_rL;
  real_T *c7_a_mL;
  real_T *c7_F_SL_f;
  real_T *c7_F_SL_r;
  real_T *c7_F_SL_m;
} SFc7_SS_jun_25InstanceStruct;

#endif                                 /*typedef_SFc7_SS_jun_25InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c7_SS_jun_25_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c7_SS_jun_25_get_check_sum(mxArray *plhs[]);
extern void c7_SS_jun_25_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
