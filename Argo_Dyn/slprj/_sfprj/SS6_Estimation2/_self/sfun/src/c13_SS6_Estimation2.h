#ifndef __c13_SS6_Estimation2_h__
#define __c13_SS6_Estimation2_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc13_SS6_Estimation2InstanceStruct
#define typedef_SFc13_SS6_Estimation2InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c13_sfEvent;
  boolean_T c13_isStable;
  boolean_T c13_doneDoubleBufferReInit;
  uint8_T c13_is_active_c13_SS6_Estimation2;
  real_T *c13_U_tL;
  real_T *c13_U_tR;
  real_T *c13_SlipL;
  real_T *c13_SlipR;
  real_T *c13_countL;
  real_T *c13_countR;
} SFc13_SS6_Estimation2InstanceStruct;

#endif                                 /*typedef_SFc13_SS6_Estimation2InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c13_SS6_Estimation2_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c13_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
extern void c13_SS6_Estimation2_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
