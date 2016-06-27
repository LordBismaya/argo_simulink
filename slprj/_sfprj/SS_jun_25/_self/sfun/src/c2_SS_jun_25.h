#ifndef __c2_SS_jun_25_h__
#define __c2_SS_jun_25_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_SS_jun_25InstanceStruct
#define typedef_SFc2_SS_jun_25InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_SS_jun_25;
  real_T *c2_WOT;
  real_T *c2_K_brk_L;
  real_T *c2_K_brk_R;
  real_T *c2_F_tl;
  real_T *c2_F_tr;
  real_T *c2_FzL;
  real_T *c2_FzR;
  real_T *c2_w_l;
  real_T *c2_w_r;
  real_T *c2_w_dot_l;
  real_T *c2_w_dot_r;
} SFc2_SS_jun_25InstanceStruct;

#endif                                 /*typedef_SFc2_SS_jun_25InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_SS_jun_25_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_SS_jun_25_get_check_sum(mxArray *plhs[]);
extern void c2_SS_jun_25_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
