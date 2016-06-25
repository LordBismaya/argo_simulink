#ifndef __c6_SS_jun_25_h__
#define __c6_SS_jun_25_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc6_SS_jun_25InstanceStruct
#define typedef_SFc6_SS_jun_25InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c6_sfEvent;
  boolean_T c6_isStable;
  boolean_T c6_doneDoubleBufferReInit;
  uint8_T c6_is_active_c6_SS_jun_25;
  real_T *c6_U_dot;
  real_T *c6_V;
  real_T *c6_Fz_L;
  real_T *c6_Fz_R;
  real_T *c6_r;
} SFc6_SS_jun_25InstanceStruct;

#endif                                 /*typedef_SFc6_SS_jun_25InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c6_SS_jun_25_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c6_SS_jun_25_get_check_sum(mxArray *plhs[]);
extern void c6_SS_jun_25_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
