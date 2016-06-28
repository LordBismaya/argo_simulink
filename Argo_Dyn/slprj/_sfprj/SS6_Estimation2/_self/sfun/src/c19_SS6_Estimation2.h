#ifndef __c19_SS6_Estimation2_h__
#define __c19_SS6_Estimation2_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc19_SS6_Estimation2InstanceStruct
#define typedef_SFc19_SS6_Estimation2InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c19_sfEvent;
  boolean_T c19_isStable;
  boolean_T c19_doneDoubleBufferReInit;
  uint8_T c19_is_active_c19_SS6_Estimation2;
  real_T c19_cov[49];
  boolean_T c19_cov_not_empty;
  real_T c19_State[7];
  boolean_T c19_State_not_empty;
  real_T *c19_X_1;
  real_T *c19_Y_1;
  real_T *c19_X_GPS;
  real_T *c19_Y_GPS;
  real_T *c19_aX_IMU;
  real_T *c19_aY_IMU;
  real_T *c19_r_IMU;
  real_T *c19_V_x;
  real_T *c19_V_y;
} SFc19_SS6_Estimation2InstanceStruct;

#endif                                 /*typedef_SFc19_SS6_Estimation2InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c19_SS6_Estimation2_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c19_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
extern void c19_SS6_Estimation2_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
