#ifndef __c18_SS6_Estimation2_h__
#define __c18_SS6_Estimation2_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc18_SS6_Estimation2InstanceStruct
#define typedef_SFc18_SS6_Estimation2InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c18_sfEvent;
  boolean_T c18_isStable;
  boolean_T c18_doneDoubleBufferReInit;
  uint8_T c18_is_active_c18_SS6_Estimation2;
  real_T *c18_aX_IMU;
  real_T *c18_aY_IMU;
  real_T *c18_aX;
  real_T *c18_aY;
  real_T *c18_r_IMU;
  real_T *c18_U_dot;
  real_T *c18_V_dot;
  real_T *c18_r_EOM;
  real_T *c18_UM_dot;
  real_T *c18_VM_dot;
  real_T *c18_rM;
  real_T *c18_V1;
  real_T *c18_V2;
  real_T *c18_V3;
  real_T *c18_V4;
  real_T *c18_V5;
  real_T *c18_V6;
  real_T *c18_r;
} SFc18_SS6_Estimation2InstanceStruct;

#endif                                 /*typedef_SFc18_SS6_Estimation2InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c18_SS6_Estimation2_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c18_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
extern void c18_SS6_Estimation2_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
