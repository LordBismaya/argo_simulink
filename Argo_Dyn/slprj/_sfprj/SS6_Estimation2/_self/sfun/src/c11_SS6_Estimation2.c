/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SS6_Estimation2_sfun.h"
#include "c11_SS6_Estimation2.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SS6_Estimation2_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c11_debug_family_names[23] = { "Q", "R", "Ts", "B", "G", "g",
  "State_p", "H", "y_meas", "K", "nargin", "nargout", "X_GPS", "Y_GPS", "aX_IMU",
  "aY_IMU", "r_IMU", "X_1", "Y_1", "V_x", "V_y", "cov", "State" };

/* Function Declarations */
static void initialize_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void initialize_params_c11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static void enable_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void disable_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_update_debugger_state_c11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static void set_sim_state_c11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance, const mxArray *c11_st);
static void finalize_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void sf_gateway_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void mdl_start_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_chartstep_c11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static void initSimStructsc11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c11_machineNumber, uint32_T
  c11_chartNumber, uint32_T c11_instanceNumber);
static const mxArray *c11_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static void c11_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_b_State, const char_T *c11_identifier,
  real_T c11_y[7]);
static void c11_b_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[7]);
static void c11_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static const mxArray *c11_b_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static void c11_c_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_b_cov, const char_T *c11_identifier, real_T
  c11_y[49]);
static void c11_d_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[49]);
static void c11_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static const mxArray *c11_c_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static real_T c11_e_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_b_V_y, const char_T *c11_identifier);
static real_T c11_f_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId);
static void c11_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static const mxArray *c11_d_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static void c11_g_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[14]);
static void c11_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static const mxArray *c11_e_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static void c11_h_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[2]);
static void c11_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static const mxArray *c11_f_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static const mxArray *c11_g_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static void c11_i_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[7]);
static void c11_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static const mxArray *c11_h_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static void c11_j_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[49]);
static void c11_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static const mxArray *c11_i_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static const mxArray *c11_j_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static void c11_k_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[4]);
static void c11_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static void c11_info_helper(const mxArray **c11_info);
static const mxArray *c11_emlrt_marshallOut(const char * c11_u);
static const mxArray *c11_b_emlrt_marshallOut(const uint32_T c11_u);
static void c11_b_info_helper(const mxArray **c11_info);
static void c11_c_info_helper(const mxArray **c11_info);
static void c11_d_info_helper(const mxArray **c11_info);
static void c11_eye(SFc11_SS6_Estimation2InstanceStruct *chartInstance, real_T
                    c11_I[49]);
static void c11_eml_switch_helper(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static boolean_T c11_use_refblas(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_threshold(SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static void c11_b_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_c_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_d_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_e_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_pinv(SFc11_SS6_Estimation2InstanceStruct *chartInstance, real_T
                     c11_A[4], real_T c11_X[4]);
static void c11_f_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_eml_error(SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static void c11_eml_xgesvd(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_A[4], real_T c11_U[4], real_T c11_S[2], real_T c11_V[4]);
static real_T c11_eml_xnrm2(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4]);
static void c11_below_threshold(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_realmin(SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static real_T c11_abs(SFc11_SS6_Estimation2InstanceStruct *chartInstance, real_T
                      c11_x);
static void c11_eps(SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static void c11_b_below_threshold(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_b_eml_switch_helper(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static real_T c11_eml_xdotc(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4], real_T c11_y[4]);
static void c11_b_threshold(SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static void c11_eml_xaxpy(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_a, real_T c11_y[4], real_T c11_b_y[4]);
static void c11_c_threshold(SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static void c11_check_forloop_overflow_error(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, boolean_T c11_overflow);
static real_T c11_b_eml_xdotc(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c11_n, real_T c11_x[4], int32_T c11_ix0, real_T c11_y[4], int32_T
  c11_iy0);
static void c11_b_eml_xaxpy(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c11_n, real_T c11_a, int32_T c11_ix0, real_T c11_y[4], int32_T c11_iy0,
  real_T c11_b_y[4]);
static void c11_eml_xscal(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_a, real_T c11_x[4], int32_T c11_ix0, real_T c11_b_x[4]);
static void c11_g_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_b_eml_error(SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static real_T c11_sqrt(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x);
static void c11_c_eml_error(SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static void c11_eml_xrotg(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_a, real_T c11_b, real_T *c11_b_a, real_T *c11_b_b, real_T *c11_c,
  real_T *c11_s);
static void c11_eml_xrot(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4], int32_T c11_ix0, int32_T c11_iy0, real_T c11_c, real_T c11_s,
  real_T c11_b_x[4]);
static void c11_d_threshold(SFc11_SS6_Estimation2InstanceStruct *chartInstance);
static void c11_eml_xswap(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4], int32_T c11_ix0, int32_T c11_iy0, real_T c11_b_x[4]);
static void c11_c_eml_switch_helper(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_eml_xgemm(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c11_k, real_T c11_A[4], real_T c11_B[4], real_T c11_C[4], real_T
  c11_b_C[4]);
static void c11_h_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_i_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_j_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c11_k_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static const mxArray *c11_k_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static int32_T c11_l_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId);
static void c11_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static uint8_T c11_m_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_b_is_active_c11_SS6_Estimation2, const
  char_T *c11_identifier);
static uint8_T c11_n_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId);
static void c11_c_eml_xaxpy(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_a, real_T c11_y[4]);
static void c11_d_eml_xaxpy(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c11_n, real_T c11_a, int32_T c11_ix0, real_T c11_y[4], int32_T c11_iy0);
static void c11_b_eml_xscal(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_a, real_T c11_x[4], int32_T c11_ix0);
static void c11_b_sqrt(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T *c11_x);
static void c11_b_eml_xrotg(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T *c11_a, real_T *c11_b, real_T *c11_c, real_T *c11_s);
static void c11_b_eml_xrot(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4], int32_T c11_ix0, int32_T c11_iy0, real_T c11_c, real_T c11_s);
static void c11_b_eml_xswap(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4], int32_T c11_ix0, int32_T c11_iy0);
static void c11_b_eml_xgemm(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c11_k, real_T c11_A[4], real_T c11_B[4], real_T c11_C[4]);
static void init_dsm_address_info(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  chartInstance->c11_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c11_cov_not_empty = false;
  chartInstance->c11_State_not_empty = false;
  chartInstance->c11_is_active_c11_SS6_Estimation2 = 0U;
}

static void initialize_params_c11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c11_update_debugger_state_c11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  const mxArray *c11_st;
  const mxArray *c11_y = NULL;
  real_T c11_hoistedGlobal;
  real_T c11_u;
  const mxArray *c11_b_y = NULL;
  real_T c11_b_hoistedGlobal;
  real_T c11_b_u;
  const mxArray *c11_c_y = NULL;
  real_T c11_c_hoistedGlobal;
  real_T c11_c_u;
  const mxArray *c11_d_y = NULL;
  real_T c11_d_hoistedGlobal;
  real_T c11_d_u;
  const mxArray *c11_e_y = NULL;
  int32_T c11_i0;
  real_T c11_e_u[7];
  const mxArray *c11_f_y = NULL;
  int32_T c11_i1;
  real_T c11_f_u[49];
  const mxArray *c11_g_y = NULL;
  uint8_T c11_e_hoistedGlobal;
  uint8_T c11_g_u;
  const mxArray *c11_h_y = NULL;
  c11_st = NULL;
  c11_st = NULL;
  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_createcellmatrix(7, 1), false);
  c11_hoistedGlobal = *chartInstance->c11_V_x;
  c11_u = c11_hoistedGlobal;
  c11_b_y = NULL;
  sf_mex_assign(&c11_b_y, sf_mex_create("y", &c11_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c11_y, 0, c11_b_y);
  c11_b_hoistedGlobal = *chartInstance->c11_V_y;
  c11_b_u = c11_b_hoistedGlobal;
  c11_c_y = NULL;
  sf_mex_assign(&c11_c_y, sf_mex_create("y", &c11_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c11_y, 1, c11_c_y);
  c11_c_hoistedGlobal = *chartInstance->c11_X_1;
  c11_c_u = c11_c_hoistedGlobal;
  c11_d_y = NULL;
  sf_mex_assign(&c11_d_y, sf_mex_create("y", &c11_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c11_y, 2, c11_d_y);
  c11_d_hoistedGlobal = *chartInstance->c11_Y_1;
  c11_d_u = c11_d_hoistedGlobal;
  c11_e_y = NULL;
  sf_mex_assign(&c11_e_y, sf_mex_create("y", &c11_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c11_y, 3, c11_e_y);
  for (c11_i0 = 0; c11_i0 < 7; c11_i0++) {
    c11_e_u[c11_i0] = chartInstance->c11_State[c11_i0];
  }

  c11_f_y = NULL;
  if (!chartInstance->c11_State_not_empty) {
    sf_mex_assign(&c11_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c11_f_y, sf_mex_create("y", c11_e_u, 0, 0U, 1U, 0U, 1, 7),
                  false);
  }

  sf_mex_setcell(c11_y, 4, c11_f_y);
  for (c11_i1 = 0; c11_i1 < 49; c11_i1++) {
    c11_f_u[c11_i1] = chartInstance->c11_cov[c11_i1];
  }

  c11_g_y = NULL;
  if (!chartInstance->c11_cov_not_empty) {
    sf_mex_assign(&c11_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c11_g_y, sf_mex_create("y", c11_f_u, 0, 0U, 1U, 0U, 2, 7, 7),
                  false);
  }

  sf_mex_setcell(c11_y, 5, c11_g_y);
  c11_e_hoistedGlobal = chartInstance->c11_is_active_c11_SS6_Estimation2;
  c11_g_u = c11_e_hoistedGlobal;
  c11_h_y = NULL;
  sf_mex_assign(&c11_h_y, sf_mex_create("y", &c11_g_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c11_y, 6, c11_h_y);
  sf_mex_assign(&c11_st, c11_y, false);
  return c11_st;
}

static void set_sim_state_c11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance, const mxArray *c11_st)
{
  const mxArray *c11_u;
  real_T c11_dv0[7];
  int32_T c11_i2;
  real_T c11_dv1[49];
  int32_T c11_i3;
  chartInstance->c11_doneDoubleBufferReInit = true;
  c11_u = sf_mex_dup(c11_st);
  *chartInstance->c11_V_x = c11_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c11_u, 0)), "V_x");
  *chartInstance->c11_V_y = c11_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c11_u, 1)), "V_y");
  *chartInstance->c11_X_1 = c11_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c11_u, 2)), "X_1");
  *chartInstance->c11_Y_1 = c11_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c11_u, 3)), "Y_1");
  c11_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c11_u, 4)),
                       "State", c11_dv0);
  for (c11_i2 = 0; c11_i2 < 7; c11_i2++) {
    chartInstance->c11_State[c11_i2] = c11_dv0[c11_i2];
  }

  c11_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c11_u, 5)),
    "cov", c11_dv1);
  for (c11_i3 = 0; c11_i3 < 49; c11_i3++) {
    chartInstance->c11_cov[c11_i3] = c11_dv1[c11_i3];
  }

  chartInstance->c11_is_active_c11_SS6_Estimation2 = c11_m_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c11_u, 6)),
     "is_active_c11_SS6_Estimation2");
  sf_mex_destroy(&c11_u);
  c11_update_debugger_state_c11_SS6_Estimation2(chartInstance);
  sf_mex_destroy(&c11_st);
}

static void finalize_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 10U, chartInstance->c11_sfEvent);
  chartInstance->c11_sfEvent = CALL_EVENT;
  c11_chartstep_c11_SS6_Estimation2(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SS6_Estimation2MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c11_X_1, 0U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c11_Y_1, 1U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c11_X_GPS, 2U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c11_Y_GPS, 3U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c11_aX_IMU, 4U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c11_aY_IMU, 5U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c11_r_IMU, 6U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c11_V_x, 7U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c11_V_y, 8U);
}

static void mdl_start_c11_SS6_Estimation2(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_chartstep_c11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  real_T c11_hoistedGlobal;
  real_T c11_b_hoistedGlobal;
  real_T c11_c_hoistedGlobal;
  real_T c11_d_hoistedGlobal;
  real_T c11_e_hoistedGlobal;
  real_T c11_b_X_GPS;
  real_T c11_b_Y_GPS;
  real_T c11_b_aX_IMU;
  real_T c11_b_aY_IMU;
  real_T c11_b_r_IMU;
  uint32_T c11_debug_family_var_map[23];
  real_T c11_Q[49];
  real_T c11_R[4];
  real_T c11_Ts;
  real_T c11_B[21];
  real_T c11_G[49];
  real_T c11_g[7];
  real_T c11_State_p[7];
  real_T c11_H[14];
  real_T c11_y_meas[2];
  real_T c11_K[14];
  real_T c11_nargin = 5.0;
  real_T c11_nargout = 4.0;
  real_T c11_b_X_1;
  real_T c11_b_Y_1;
  real_T c11_b_V_x;
  real_T c11_b_V_y;
  int32_T c11_i4;
  int32_T c11_i5;
  int32_T c11_i6;
  static real_T c11_dv2[49] = { 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001 };

  int32_T c11_i7;
  int32_T c11_i8;
  int32_T c11_i9;
  int32_T c11_i10;
  static real_T c11_dv3[4] = { 0.0001, 0.0, 0.0, 0.0001 };

  int32_T c11_i11;
  static real_T c11_a[21] = { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0 };

  int32_T c11_i12;
  real_T c11_f_hoistedGlobal[7];
  real_T c11_x;
  real_T c11_b_x;
  int32_T c11_i13;
  real_T c11_c_x;
  real_T c11_d_x;
  int32_T c11_i14;
  int32_T c11_i15;
  real_T c11_g_hoistedGlobal[7];
  real_T c11_e_x;
  real_T c11_f_x;
  int32_T c11_i16;
  int32_T c11_i17;
  real_T c11_h_hoistedGlobal[7];
  real_T c11_g_x;
  real_T c11_h_x;
  int32_T c11_i18;
  real_T c11_i_x;
  real_T c11_j_x;
  int32_T c11_i19;
  real_T c11_k_x;
  real_T c11_l_x;
  int32_T c11_i20;
  int32_T c11_i21;
  real_T c11_i_hoistedGlobal[7];
  real_T c11_m_x;
  real_T c11_n_x;
  int32_T c11_i22;
  int32_T c11_i23;
  real_T c11_j_hoistedGlobal[7];
  real_T c11_o_x;
  real_T c11_p_x;
  int32_T c11_i24;
  int32_T c11_i25;
  static real_T c11_dv4[7] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T c11_i26;
  int32_T c11_i27;
  static real_T c11_dv5[7] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T c11_i28;
  int32_T c11_i29;
  static real_T c11_dv6[7] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T c11_i30;
  int32_T c11_i31;
  int32_T c11_i32;
  int32_T c11_i33;
  int32_T c11_i34;
  real_T c11_q_x;
  real_T c11_r_x;
  int32_T c11_i35;
  int32_T c11_i36;
  real_T c11_k_hoistedGlobal[7];
  real_T c11_s_x;
  real_T c11_t_x;
  int32_T c11_i37;
  int32_T c11_i38;
  real_T c11_l_hoistedGlobal[7];
  real_T c11_u_x;
  real_T c11_v_x;
  int32_T c11_i39;
  int32_T c11_i40;
  real_T c11_m_hoistedGlobal[7];
  real_T c11_w_x;
  real_T c11_x_x;
  real_T c11_b[3];
  int32_T c11_i41;
  int32_T c11_i42;
  int32_T c11_i43;
  int32_T c11_i44;
  int32_T c11_i45;
  real_T c11_n_hoistedGlobal[49];
  int32_T c11_i46;
  real_T c11_b_a[49];
  int32_T c11_i47;
  int32_T c11_i48;
  int32_T c11_i49;
  real_T c11_y[49];
  int32_T c11_i50;
  int32_T c11_i51;
  int32_T c11_i52;
  int32_T c11_i53;
  int32_T c11_i54;
  int32_T c11_i55;
  int32_T c11_i56;
  int32_T c11_i57;
  int32_T c11_i58;
  int32_T c11_i59;
  int32_T c11_i60;
  int32_T c11_i61;
  int32_T c11_i62;
  static real_T c11_c_a[14] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 1.0 };

  int32_T c11_i63;
  int32_T c11_i64;
  int32_T c11_i65;
  int32_T c11_i66;
  real_T c11_b_y[14];
  int32_T c11_i67;
  int32_T c11_i68;
  static real_T c11_b_b[14] = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0 };

  int32_T c11_i69;
  int32_T c11_i70;
  int32_T c11_i71;
  int32_T c11_i72;
  int32_T c11_i73;
  real_T c11_c_y[14];
  int32_T c11_i74;
  int32_T c11_i75;
  int32_T c11_i76;
  int32_T c11_i77;
  int32_T c11_i78;
  int32_T c11_i79;
  real_T c11_d_y[4];
  int32_T c11_i80;
  int32_T c11_i81;
  int32_T c11_i82;
  real_T c11_e_y[4];
  int32_T c11_i83;
  int32_T c11_i84;
  int32_T c11_i85;
  real_T c11_C[14];
  int32_T c11_i86;
  int32_T c11_i87;
  int32_T c11_i88;
  int32_T c11_i89;
  int32_T c11_i90;
  int32_T c11_i91;
  int32_T c11_i92;
  int32_T c11_i93;
  int32_T c11_i94;
  int32_T c11_i95;
  int32_T c11_i96;
  real_T c11_f_y[2];
  int32_T c11_i97;
  int32_T c11_i98;
  int32_T c11_i99;
  int32_T c11_i100;
  int32_T c11_i101;
  int32_T c11_i102;
  int32_T c11_i103;
  int32_T c11_i104;
  int32_T c11_i105;
  int32_T c11_i106;
  int32_T c11_i107;
  int32_T c11_i108;
  int32_T c11_i109;
  int32_T c11_i110;
  int32_T c11_i111;
  int32_T c11_i112;
  int32_T c11_i113;
  int32_T c11_i114;
  int32_T c11_i115;
  int32_T c11_i116;
  int32_T c11_i117;
  int32_T c11_i118;
  int32_T c11_i119;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 10U, chartInstance->c11_sfEvent);
  c11_hoistedGlobal = *chartInstance->c11_X_GPS;
  c11_b_hoistedGlobal = *chartInstance->c11_Y_GPS;
  c11_c_hoistedGlobal = *chartInstance->c11_aX_IMU;
  c11_d_hoistedGlobal = *chartInstance->c11_aY_IMU;
  c11_e_hoistedGlobal = *chartInstance->c11_r_IMU;
  c11_b_X_GPS = c11_hoistedGlobal;
  c11_b_Y_GPS = c11_b_hoistedGlobal;
  c11_b_aX_IMU = c11_c_hoistedGlobal;
  c11_b_aY_IMU = c11_d_hoistedGlobal;
  c11_b_r_IMU = c11_e_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 23U, 23U, c11_debug_family_names,
    c11_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c11_Q, 0U, c11_h_sf_marshallOut,
    c11_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c11_R, 1U, c11_j_sf_marshallOut,
    c11_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c11_Ts, 2U, c11_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c11_B, 3U, c11_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c11_G, 4U, c11_h_sf_marshallOut,
    c11_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c11_g, 5U, c11_g_sf_marshallOut,
    c11_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c11_State_p, 6U, c11_g_sf_marshallOut,
    c11_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c11_H, 7U, c11_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c11_y_meas, 8U, c11_e_sf_marshallOut,
    c11_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c11_K, 9U, c11_d_sf_marshallOut,
    c11_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c11_nargin, 10U, c11_c_sf_marshallOut,
    c11_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c11_nargout, 11U, c11_c_sf_marshallOut,
    c11_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c11_b_X_GPS, 12U, c11_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c11_b_Y_GPS, 13U, c11_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c11_b_aX_IMU, 14U, c11_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c11_b_aY_IMU, 15U, c11_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c11_b_r_IMU, 16U, c11_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c11_b_X_1, 17U, c11_c_sf_marshallOut,
    c11_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c11_b_Y_1, 18U, c11_c_sf_marshallOut,
    c11_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c11_b_V_x, 19U, c11_c_sf_marshallOut,
    c11_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c11_b_V_y, 20U, c11_c_sf_marshallOut,
    c11_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c11_cov, 21U,
    c11_b_sf_marshallOut, c11_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c11_State, 22U,
    c11_sf_marshallOut, c11_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 3);
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 5);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c11_cov_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 6);
    for (c11_i4 = 0; c11_i4 < 7; c11_i4++) {
      chartInstance->c11_State[c11_i4] = 0.0;
    }

    chartInstance->c11_State_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 7);
    for (c11_i5 = 0; c11_i5 < 49; c11_i5++) {
      chartInstance->c11_cov[c11_i5] = 0.0;
    }

    chartInstance->c11_cov_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 11);
  for (c11_i6 = 0; c11_i6 < 49; c11_i6++) {
    c11_Q[c11_i6] = c11_dv2[c11_i6];
  }

  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 12);
  c11_i7 = 0;
  for (c11_i8 = 0; c11_i8 < 4; c11_i8++) {
    for (c11_i9 = 0; c11_i9 < 4; c11_i9++) {
      c11_Q[(c11_i9 + c11_i7) + 24] = 0.0;
    }

    c11_i7 += 7;
  }

  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 14);
  for (c11_i10 = 0; c11_i10 < 4; c11_i10++) {
    c11_R[c11_i10] = c11_dv3[c11_i10];
  }

  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 16);
  c11_Ts = 0.01;
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 18);
  for (c11_i11 = 0; c11_i11 < 21; c11_i11++) {
    c11_B[c11_i11] = c11_a[c11_i11];
  }

  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 27);
  for (c11_i12 = 0; c11_i12 < 7; c11_i12++) {
    c11_f_hoistedGlobal[c11_i12] = chartInstance->c11_State[c11_i12];
  }

  c11_x = c11_f_hoistedGlobal[2];
  c11_b_x = c11_x;
  c11_b_x = muDoubleScalarCos(c11_b_x);
  for (c11_i13 = 0; c11_i13 < 7; c11_i13++) {
    c11_f_hoistedGlobal[c11_i13] = chartInstance->c11_State[c11_i13];
  }

  c11_c_x = c11_f_hoistedGlobal[2];
  c11_d_x = c11_c_x;
  c11_d_x = muDoubleScalarSin(c11_d_x);
  for (c11_i14 = 0; c11_i14 < 7; c11_i14++) {
    c11_f_hoistedGlobal[c11_i14] = chartInstance->c11_State[c11_i14];
  }

  for (c11_i15 = 0; c11_i15 < 7; c11_i15++) {
    c11_g_hoistedGlobal[c11_i15] = chartInstance->c11_State[c11_i15];
  }

  c11_e_x = c11_g_hoistedGlobal[2];
  c11_f_x = c11_e_x;
  c11_f_x = muDoubleScalarSin(c11_f_x);
  for (c11_i16 = 0; c11_i16 < 7; c11_i16++) {
    c11_g_hoistedGlobal[c11_i16] = chartInstance->c11_State[c11_i16];
  }

  for (c11_i17 = 0; c11_i17 < 7; c11_i17++) {
    c11_h_hoistedGlobal[c11_i17] = chartInstance->c11_State[c11_i17];
  }

  c11_g_x = c11_h_hoistedGlobal[2];
  c11_h_x = c11_g_x;
  c11_h_x = muDoubleScalarCos(c11_h_x);
  for (c11_i18 = 0; c11_i18 < 7; c11_i18++) {
    c11_h_hoistedGlobal[c11_i18] = chartInstance->c11_State[c11_i18];
  }

  c11_i_x = c11_h_hoistedGlobal[2];
  c11_j_x = c11_i_x;
  c11_j_x = muDoubleScalarSin(c11_j_x);
  for (c11_i19 = 0; c11_i19 < 7; c11_i19++) {
    c11_h_hoistedGlobal[c11_i19] = chartInstance->c11_State[c11_i19];
  }

  c11_k_x = c11_h_hoistedGlobal[2];
  c11_l_x = c11_k_x;
  c11_l_x = muDoubleScalarCos(c11_l_x);
  for (c11_i20 = 0; c11_i20 < 7; c11_i20++) {
    c11_h_hoistedGlobal[c11_i20] = chartInstance->c11_State[c11_i20];
  }

  for (c11_i21 = 0; c11_i21 < 7; c11_i21++) {
    c11_i_hoistedGlobal[c11_i21] = chartInstance->c11_State[c11_i21];
  }

  c11_m_x = c11_i_hoistedGlobal[2];
  c11_n_x = c11_m_x;
  c11_n_x = muDoubleScalarCos(c11_n_x);
  for (c11_i22 = 0; c11_i22 < 7; c11_i22++) {
    c11_i_hoistedGlobal[c11_i22] = chartInstance->c11_State[c11_i22];
  }

  for (c11_i23 = 0; c11_i23 < 7; c11_i23++) {
    c11_j_hoistedGlobal[c11_i23] = chartInstance->c11_State[c11_i23];
  }

  c11_o_x = c11_j_hoistedGlobal[2];
  c11_p_x = c11_o_x;
  c11_p_x = muDoubleScalarSin(c11_p_x);
  c11_i24 = 0;
  for (c11_i25 = 0; c11_i25 < 7; c11_i25++) {
    c11_G[c11_i24] = c11_dv4[c11_i25];
    c11_i24 += 7;
  }

  c11_i26 = 0;
  for (c11_i27 = 0; c11_i27 < 7; c11_i27++) {
    c11_G[c11_i26 + 1] = c11_dv5[c11_i27];
    c11_i26 += 7;
  }

  c11_i28 = 0;
  for (c11_i29 = 0; c11_i29 < 7; c11_i29++) {
    c11_G[c11_i28 + 2] = c11_dv6[c11_i29];
    c11_i28 += 7;
  }

  c11_G[3] = c11_b_x;
  c11_G[10] = -c11_d_x;
  c11_G[17] = -c11_f_hoistedGlobal[0] * c11_f_x - c11_g_hoistedGlobal[1] *
    c11_h_x;
  c11_G[24] = 0.0;
  c11_G[31] = 0.0;
  c11_G[38] = 0.0;
  c11_G[45] = 0.0;
  c11_G[4] = c11_j_x;
  c11_G[11] = c11_l_x;
  c11_G[18] = c11_h_hoistedGlobal[0] * c11_n_x - c11_i_hoistedGlobal[1] *
    c11_p_x;
  c11_G[25] = 0.0;
  c11_G[32] = 0.0;
  c11_G[39] = 0.0;
  c11_G[46] = 0.0;
  c11_G[5] = 0.0;
  c11_G[12] = 0.0;
  c11_G[19] = 0.0;
  c11_G[26] = c11_Ts;
  c11_G[33] = 0.0;
  c11_G[40] = 1.0;
  c11_G[47] = 0.0;
  c11_G[6] = 0.0;
  c11_G[13] = 0.0;
  c11_G[20] = 0.0;
  c11_G[27] = 0.0;
  c11_G[34] = c11_Ts;
  c11_G[41] = 0.0;
  c11_G[48] = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 36);
  for (c11_i30 = 0; c11_i30 < 7; c11_i30++) {
    c11_f_hoistedGlobal[c11_i30] = chartInstance->c11_State[c11_i30];
  }

  for (c11_i31 = 0; c11_i31 < 7; c11_i31++) {
    c11_g_hoistedGlobal[c11_i31] = chartInstance->c11_State[c11_i31];
  }

  for (c11_i32 = 0; c11_i32 < 7; c11_i32++) {
    c11_h_hoistedGlobal[c11_i32] = chartInstance->c11_State[c11_i32];
  }

  for (c11_i33 = 0; c11_i33 < 7; c11_i33++) {
    c11_i_hoistedGlobal[c11_i33] = chartInstance->c11_State[c11_i33];
  }

  for (c11_i34 = 0; c11_i34 < 7; c11_i34++) {
    c11_j_hoistedGlobal[c11_i34] = chartInstance->c11_State[c11_i34];
  }

  c11_q_x = c11_j_hoistedGlobal[2];
  c11_r_x = c11_q_x;
  c11_r_x = muDoubleScalarCos(c11_r_x);
  for (c11_i35 = 0; c11_i35 < 7; c11_i35++) {
    c11_j_hoistedGlobal[c11_i35] = chartInstance->c11_State[c11_i35];
  }

  for (c11_i36 = 0; c11_i36 < 7; c11_i36++) {
    c11_k_hoistedGlobal[c11_i36] = chartInstance->c11_State[c11_i36];
  }

  c11_s_x = c11_k_hoistedGlobal[2];
  c11_t_x = c11_s_x;
  c11_t_x = muDoubleScalarSin(c11_t_x);
  for (c11_i37 = 0; c11_i37 < 7; c11_i37++) {
    c11_k_hoistedGlobal[c11_i37] = chartInstance->c11_State[c11_i37];
  }

  for (c11_i38 = 0; c11_i38 < 7; c11_i38++) {
    c11_l_hoistedGlobal[c11_i38] = chartInstance->c11_State[c11_i38];
  }

  c11_u_x = c11_l_hoistedGlobal[2];
  c11_v_x = c11_u_x;
  c11_v_x = muDoubleScalarSin(c11_v_x);
  for (c11_i39 = 0; c11_i39 < 7; c11_i39++) {
    c11_l_hoistedGlobal[c11_i39] = chartInstance->c11_State[c11_i39];
  }

  for (c11_i40 = 0; c11_i40 < 7; c11_i40++) {
    c11_m_hoistedGlobal[c11_i40] = chartInstance->c11_State[c11_i40];
  }

  c11_w_x = c11_m_hoistedGlobal[2];
  c11_x_x = c11_w_x;
  c11_x_x = muDoubleScalarCos(c11_x_x);
  c11_g[0] = c11_f_hoistedGlobal[0];
  c11_g[1] = c11_g_hoistedGlobal[1];
  c11_g[2] = c11_h_hoistedGlobal[2];
  c11_g[3] = c11_i_hoistedGlobal[0] * c11_r_x - c11_j_hoistedGlobal[1] * c11_t_x;
  c11_g[4] = c11_k_hoistedGlobal[0] * c11_v_x + c11_l_hoistedGlobal[1] * c11_x_x;
  c11_g[5] = 0.01 * chartInstance->c11_State[3] + chartInstance->c11_State[5];
  c11_g[6] = 0.01 * chartInstance->c11_State[4] + chartInstance->c11_State[6];
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 45);
  c11_b[0] = c11_b_aX_IMU;
  c11_b[1] = c11_b_aY_IMU;
  c11_b[2] = c11_b_r_IMU;
  c11_eml_scalar_eg(chartInstance);
  c11_eml_scalar_eg(chartInstance);
  c11_threshold(chartInstance);
  for (c11_i41 = 0; c11_i41 < 7; c11_i41++) {
    c11_f_hoistedGlobal[c11_i41] = 0.0;
    c11_i42 = 0;
    for (c11_i43 = 0; c11_i43 < 3; c11_i43++) {
      c11_f_hoistedGlobal[c11_i41] += c11_a[c11_i42 + c11_i41] * c11_b[c11_i43];
      c11_i42 += 7;
    }
  }

  for (c11_i44 = 0; c11_i44 < 7; c11_i44++) {
    c11_State_p[c11_i44] = c11_g[c11_i44] + c11_f_hoistedGlobal[c11_i44];
  }

  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 48);
  for (c11_i45 = 0; c11_i45 < 49; c11_i45++) {
    c11_n_hoistedGlobal[c11_i45] = chartInstance->c11_cov[c11_i45];
  }

  for (c11_i46 = 0; c11_i46 < 49; c11_i46++) {
    c11_b_a[c11_i46] = c11_G[c11_i46];
  }

  c11_b_eml_scalar_eg(chartInstance);
  c11_b_eml_scalar_eg(chartInstance);
  c11_threshold(chartInstance);
  for (c11_i47 = 0; c11_i47 < 7; c11_i47++) {
    c11_i48 = 0;
    for (c11_i49 = 0; c11_i49 < 7; c11_i49++) {
      c11_y[c11_i48 + c11_i47] = 0.0;
      c11_i50 = 0;
      for (c11_i51 = 0; c11_i51 < 7; c11_i51++) {
        c11_y[c11_i48 + c11_i47] += c11_b_a[c11_i50 + c11_i47] *
          c11_n_hoistedGlobal[c11_i51 + c11_i48];
        c11_i50 += 7;
      }

      c11_i48 += 7;
    }
  }

  c11_i52 = 0;
  for (c11_i53 = 0; c11_i53 < 7; c11_i53++) {
    c11_i54 = 0;
    for (c11_i55 = 0; c11_i55 < 7; c11_i55++) {
      c11_n_hoistedGlobal[c11_i55 + c11_i52] = c11_G[c11_i54 + c11_i53];
      c11_i54 += 7;
    }

    c11_i52 += 7;
  }

  c11_b_eml_scalar_eg(chartInstance);
  c11_b_eml_scalar_eg(chartInstance);
  c11_threshold(chartInstance);
  for (c11_i56 = 0; c11_i56 < 7; c11_i56++) {
    c11_i57 = 0;
    for (c11_i58 = 0; c11_i58 < 7; c11_i58++) {
      c11_b_a[c11_i57 + c11_i56] = 0.0;
      c11_i59 = 0;
      for (c11_i60 = 0; c11_i60 < 7; c11_i60++) {
        c11_b_a[c11_i57 + c11_i56] += c11_y[c11_i59 + c11_i56] *
          c11_n_hoistedGlobal[c11_i60 + c11_i57];
        c11_i59 += 7;
      }

      c11_i57 += 7;
    }
  }

  for (c11_i61 = 0; c11_i61 < 49; c11_i61++) {
    chartInstance->c11_cov[c11_i61] = c11_b_a[c11_i61] + c11_Q[c11_i61];
  }

  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 50);
  for (c11_i62 = 0; c11_i62 < 14; c11_i62++) {
    c11_H[c11_i62] = c11_c_a[c11_i62];
  }

  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 54);
  c11_y_meas[0] = c11_b_X_GPS;
  c11_y_meas[1] = c11_b_Y_GPS;
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 57);
  for (c11_i63 = 0; c11_i63 < 49; c11_i63++) {
    c11_n_hoistedGlobal[c11_i63] = chartInstance->c11_cov[c11_i63];
  }

  c11_c_eml_scalar_eg(chartInstance);
  c11_c_eml_scalar_eg(chartInstance);
  c11_threshold(chartInstance);
  for (c11_i64 = 0; c11_i64 < 7; c11_i64++) {
    c11_i65 = 0;
    for (c11_i66 = 0; c11_i66 < 2; c11_i66++) {
      c11_b_y[c11_i65 + c11_i64] = 0.0;
      c11_i67 = 0;
      for (c11_i68 = 0; c11_i68 < 7; c11_i68++) {
        c11_b_y[c11_i65 + c11_i64] += c11_n_hoistedGlobal[c11_i67 + c11_i64] *
          c11_b_b[c11_i68 + c11_i65];
        c11_i67 += 7;
      }

      c11_i65 += 7;
    }
  }

  for (c11_i69 = 0; c11_i69 < 49; c11_i69++) {
    c11_n_hoistedGlobal[c11_i69] = chartInstance->c11_cov[c11_i69];
  }

  c11_d_eml_scalar_eg(chartInstance);
  c11_d_eml_scalar_eg(chartInstance);
  c11_threshold(chartInstance);
  for (c11_i70 = 0; c11_i70 < 2; c11_i70++) {
    c11_i71 = 0;
    c11_i72 = 0;
    for (c11_i73 = 0; c11_i73 < 7; c11_i73++) {
      c11_c_y[c11_i71 + c11_i70] = 0.0;
      c11_i74 = 0;
      for (c11_i75 = 0; c11_i75 < 7; c11_i75++) {
        c11_c_y[c11_i71 + c11_i70] += c11_c_a[c11_i74 + c11_i70] *
          c11_n_hoistedGlobal[c11_i75 + c11_i72];
        c11_i74 += 2;
      }

      c11_i71 += 2;
      c11_i72 += 7;
    }
  }

  c11_e_eml_scalar_eg(chartInstance);
  c11_e_eml_scalar_eg(chartInstance);
  c11_threshold(chartInstance);
  for (c11_i76 = 0; c11_i76 < 2; c11_i76++) {
    c11_i77 = 0;
    c11_i78 = 0;
    for (c11_i79 = 0; c11_i79 < 2; c11_i79++) {
      c11_d_y[c11_i77 + c11_i76] = 0.0;
      c11_i80 = 0;
      for (c11_i81 = 0; c11_i81 < 7; c11_i81++) {
        c11_d_y[c11_i77 + c11_i76] += c11_c_y[c11_i80 + c11_i76] *
          c11_b_b[c11_i81 + c11_i78];
        c11_i80 += 2;
      }

      c11_i77 += 2;
      c11_i78 += 7;
    }
  }

  for (c11_i82 = 0; c11_i82 < 4; c11_i82++) {
    c11_e_y[c11_i82] = c11_d_y[c11_i82] + c11_R[c11_i82];
  }

  c11_pinv(chartInstance, c11_e_y, c11_d_y);
  c11_h_eml_scalar_eg(chartInstance);
  c11_h_eml_scalar_eg(chartInstance);
  for (c11_i83 = 0; c11_i83 < 14; c11_i83++) {
    c11_K[c11_i83] = 0.0;
  }

  for (c11_i84 = 0; c11_i84 < 14; c11_i84++) {
    c11_K[c11_i84] = 0.0;
  }

  for (c11_i85 = 0; c11_i85 < 14; c11_i85++) {
    c11_C[c11_i85] = c11_K[c11_i85];
  }

  for (c11_i86 = 0; c11_i86 < 14; c11_i86++) {
    c11_K[c11_i86] = c11_C[c11_i86];
  }

  c11_threshold(chartInstance);
  for (c11_i87 = 0; c11_i87 < 14; c11_i87++) {
    c11_C[c11_i87] = c11_K[c11_i87];
  }

  for (c11_i88 = 0; c11_i88 < 14; c11_i88++) {
    c11_K[c11_i88] = c11_C[c11_i88];
  }

  for (c11_i89 = 0; c11_i89 < 7; c11_i89++) {
    c11_i90 = 0;
    c11_i91 = 0;
    for (c11_i92 = 0; c11_i92 < 2; c11_i92++) {
      c11_K[c11_i90 + c11_i89] = 0.0;
      c11_i93 = 0;
      for (c11_i94 = 0; c11_i94 < 2; c11_i94++) {
        c11_K[c11_i90 + c11_i89] += c11_b_y[c11_i93 + c11_i89] * c11_d_y[c11_i94
          + c11_i91];
        c11_i93 += 7;
      }

      c11_i90 += 7;
      c11_i91 += 2;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 58);
  for (c11_i95 = 0; c11_i95 < 7; c11_i95++) {
    c11_f_hoistedGlobal[c11_i95] = c11_State_p[c11_i95];
  }

  c11_i_eml_scalar_eg(chartInstance);
  c11_i_eml_scalar_eg(chartInstance);
  c11_threshold(chartInstance);
  for (c11_i96 = 0; c11_i96 < 2; c11_i96++) {
    c11_f_y[c11_i96] = 0.0;
    c11_i97 = 0;
    for (c11_i98 = 0; c11_i98 < 7; c11_i98++) {
      c11_f_y[c11_i96] += c11_c_a[c11_i97 + c11_i96] *
        c11_f_hoistedGlobal[c11_i98];
      c11_i97 += 2;
    }
  }

  for (c11_i99 = 0; c11_i99 < 14; c11_i99++) {
    c11_b_y[c11_i99] = c11_K[c11_i99];
  }

  for (c11_i100 = 0; c11_i100 < 2; c11_i100++) {
    c11_f_y[c11_i100] = c11_y_meas[c11_i100] - c11_f_y[c11_i100];
  }

  c11_j_eml_scalar_eg(chartInstance);
  c11_j_eml_scalar_eg(chartInstance);
  c11_threshold(chartInstance);
  for (c11_i101 = 0; c11_i101 < 7; c11_i101++) {
    c11_f_hoistedGlobal[c11_i101] = 0.0;
    c11_i102 = 0;
    for (c11_i103 = 0; c11_i103 < 2; c11_i103++) {
      c11_f_hoistedGlobal[c11_i101] += c11_b_y[c11_i102 + c11_i101] *
        c11_f_y[c11_i103];
      c11_i102 += 7;
    }
  }

  for (c11_i104 = 0; c11_i104 < 7; c11_i104++) {
    chartInstance->c11_State[c11_i104] = c11_State_p[c11_i104] +
      c11_f_hoistedGlobal[c11_i104];
  }

  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 59);
  for (c11_i105 = 0; c11_i105 < 14; c11_i105++) {
    c11_b_y[c11_i105] = c11_K[c11_i105];
  }

  c11_k_eml_scalar_eg(chartInstance);
  c11_k_eml_scalar_eg(chartInstance);
  c11_threshold(chartInstance);
  for (c11_i106 = 0; c11_i106 < 7; c11_i106++) {
    c11_i107 = 0;
    c11_i108 = 0;
    for (c11_i109 = 0; c11_i109 < 7; c11_i109++) {
      c11_y[c11_i107 + c11_i106] = 0.0;
      c11_i110 = 0;
      for (c11_i111 = 0; c11_i111 < 2; c11_i111++) {
        c11_y[c11_i107 + c11_i106] += c11_b_y[c11_i110 + c11_i106] *
          c11_c_a[c11_i111 + c11_i108];
        c11_i110 += 7;
      }

      c11_i107 += 7;
      c11_i108 += 2;
    }
  }

  for (c11_i112 = 0; c11_i112 < 49; c11_i112++) {
    c11_n_hoistedGlobal[c11_i112] = chartInstance->c11_cov[c11_i112];
  }

  c11_eye(chartInstance, c11_b_a);
  for (c11_i113 = 0; c11_i113 < 49; c11_i113++) {
    c11_b_a[c11_i113] -= c11_y[c11_i113];
  }

  c11_b_eml_scalar_eg(chartInstance);
  c11_b_eml_scalar_eg(chartInstance);
  c11_threshold(chartInstance);
  for (c11_i114 = 0; c11_i114 < 7; c11_i114++) {
    c11_i115 = 0;
    for (c11_i116 = 0; c11_i116 < 7; c11_i116++) {
      c11_y[c11_i115 + c11_i114] = 0.0;
      c11_i117 = 0;
      for (c11_i118 = 0; c11_i118 < 7; c11_i118++) {
        c11_y[c11_i115 + c11_i114] += c11_b_a[c11_i117 + c11_i114] *
          c11_n_hoistedGlobal[c11_i118 + c11_i115];
        c11_i117 += 7;
      }

      c11_i115 += 7;
    }
  }

  for (c11_i119 = 0; c11_i119 < 49; c11_i119++) {
    chartInstance->c11_cov[c11_i119] = c11_y[c11_i119];
  }

  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 60);
  c11_b_X_1 = chartInstance->c11_State[5];
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 61);
  c11_b_Y_1 = chartInstance->c11_State[6];
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 62);
  c11_b_V_x = chartInstance->c11_State[0];
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 63);
  c11_b_V_y = chartInstance->c11_State[1];
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, -63);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c11_X_1 = c11_b_X_1;
  *chartInstance->c11_Y_1 = c11_b_Y_1;
  *chartInstance->c11_V_x = c11_b_V_x;
  *chartInstance->c11_V_y = c11_b_V_y;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 10U, chartInstance->c11_sfEvent);
}

static void initSimStructsc11_SS6_Estimation2
  (SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c11_machineNumber, uint32_T
  c11_chartNumber, uint32_T c11_instanceNumber)
{
  (void)c11_machineNumber;
  (void)c11_chartNumber;
  (void)c11_instanceNumber;
}

static const mxArray *c11_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  int32_T c11_i120;
  real_T c11_b_inData[7];
  int32_T c11_i121;
  real_T c11_u[7];
  const mxArray *c11_y = NULL;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  for (c11_i120 = 0; c11_i120 < 7; c11_i120++) {
    c11_b_inData[c11_i120] = (*(real_T (*)[7])c11_inData)[c11_i120];
  }

  for (c11_i121 = 0; c11_i121 < 7; c11_i121++) {
    c11_u[c11_i121] = c11_b_inData[c11_i121];
  }

  c11_y = NULL;
  if (!chartInstance->c11_State_not_empty) {
    sf_mex_assign(&c11_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 0, 0U, 1U, 0U, 1, 7), false);
  }

  sf_mex_assign(&c11_mxArrayOutData, c11_y, false);
  return c11_mxArrayOutData;
}

static void c11_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_b_State, const char_T *c11_identifier,
  real_T c11_y[7])
{
  emlrtMsgIdentifier c11_thisId;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_b_State), &c11_thisId,
    c11_y);
  sf_mex_destroy(&c11_b_State);
}

static void c11_b_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[7])
{
  real_T c11_dv7[7];
  int32_T c11_i122;
  if (mxIsEmpty(c11_u)) {
    chartInstance->c11_State_not_empty = false;
  } else {
    chartInstance->c11_State_not_empty = true;
    sf_mex_import(c11_parentId, sf_mex_dup(c11_u), c11_dv7, 1, 0, 0U, 1, 0U, 1,
                  7);
    for (c11_i122 = 0; c11_i122 < 7; c11_i122++) {
      c11_y[c11_i122] = c11_dv7[c11_i122];
    }
  }

  sf_mex_destroy(&c11_u);
}

static void c11_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_b_State;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  real_T c11_y[7];
  int32_T c11_i123;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_b_State = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_b_State), &c11_thisId,
    c11_y);
  sf_mex_destroy(&c11_b_State);
  for (c11_i123 = 0; c11_i123 < 7; c11_i123++) {
    (*(real_T (*)[7])c11_outData)[c11_i123] = c11_y[c11_i123];
  }

  sf_mex_destroy(&c11_mxArrayInData);
}

static const mxArray *c11_b_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  int32_T c11_i124;
  int32_T c11_i125;
  int32_T c11_i126;
  real_T c11_b_inData[49];
  int32_T c11_i127;
  int32_T c11_i128;
  int32_T c11_i129;
  real_T c11_u[49];
  const mxArray *c11_y = NULL;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  c11_i124 = 0;
  for (c11_i125 = 0; c11_i125 < 7; c11_i125++) {
    for (c11_i126 = 0; c11_i126 < 7; c11_i126++) {
      c11_b_inData[c11_i126 + c11_i124] = (*(real_T (*)[49])c11_inData)[c11_i126
        + c11_i124];
    }

    c11_i124 += 7;
  }

  c11_i127 = 0;
  for (c11_i128 = 0; c11_i128 < 7; c11_i128++) {
    for (c11_i129 = 0; c11_i129 < 7; c11_i129++) {
      c11_u[c11_i129 + c11_i127] = c11_b_inData[c11_i129 + c11_i127];
    }

    c11_i127 += 7;
  }

  c11_y = NULL;
  if (!chartInstance->c11_cov_not_empty) {
    sf_mex_assign(&c11_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 0, 0U, 1U, 0U, 2, 7, 7),
                  false);
  }

  sf_mex_assign(&c11_mxArrayOutData, c11_y, false);
  return c11_mxArrayOutData;
}

static void c11_c_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_b_cov, const char_T *c11_identifier, real_T
  c11_y[49])
{
  emlrtMsgIdentifier c11_thisId;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_b_cov), &c11_thisId,
    c11_y);
  sf_mex_destroy(&c11_b_cov);
}

static void c11_d_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[49])
{
  real_T c11_dv8[49];
  int32_T c11_i130;
  if (mxIsEmpty(c11_u)) {
    chartInstance->c11_cov_not_empty = false;
  } else {
    chartInstance->c11_cov_not_empty = true;
    sf_mex_import(c11_parentId, sf_mex_dup(c11_u), c11_dv8, 1, 0, 0U, 1, 0U, 2,
                  7, 7);
    for (c11_i130 = 0; c11_i130 < 49; c11_i130++) {
      c11_y[c11_i130] = c11_dv8[c11_i130];
    }
  }

  sf_mex_destroy(&c11_u);
}

static void c11_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_b_cov;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  real_T c11_y[49];
  int32_T c11_i131;
  int32_T c11_i132;
  int32_T c11_i133;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_b_cov = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_b_cov), &c11_thisId,
    c11_y);
  sf_mex_destroy(&c11_b_cov);
  c11_i131 = 0;
  for (c11_i132 = 0; c11_i132 < 7; c11_i132++) {
    for (c11_i133 = 0; c11_i133 < 7; c11_i133++) {
      (*(real_T (*)[49])c11_outData)[c11_i133 + c11_i131] = c11_y[c11_i133 +
        c11_i131];
    }

    c11_i131 += 7;
  }

  sf_mex_destroy(&c11_mxArrayInData);
}

static const mxArray *c11_c_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  real_T c11_u;
  const mxArray *c11_y = NULL;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  c11_u = *(real_T *)c11_inData;
  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", &c11_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c11_mxArrayOutData, c11_y, false);
  return c11_mxArrayOutData;
}

static real_T c11_e_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_b_V_y, const char_T *c11_identifier)
{
  real_T c11_y;
  emlrtMsgIdentifier c11_thisId;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_y = c11_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_b_V_y),
    &c11_thisId);
  sf_mex_destroy(&c11_b_V_y);
  return c11_y;
}

static real_T c11_f_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId)
{
  real_T c11_y;
  real_T c11_d0;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_u), &c11_d0, 1, 0, 0U, 0, 0U, 0);
  c11_y = c11_d0;
  sf_mex_destroy(&c11_u);
  return c11_y;
}

static void c11_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_b_V_y;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  real_T c11_y;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_b_V_y = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_y = c11_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_b_V_y),
    &c11_thisId);
  sf_mex_destroy(&c11_b_V_y);
  *(real_T *)c11_outData = c11_y;
  sf_mex_destroy(&c11_mxArrayInData);
}

static const mxArray *c11_d_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  int32_T c11_i134;
  int32_T c11_i135;
  int32_T c11_i136;
  real_T c11_b_inData[14];
  int32_T c11_i137;
  int32_T c11_i138;
  int32_T c11_i139;
  real_T c11_u[14];
  const mxArray *c11_y = NULL;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  c11_i134 = 0;
  for (c11_i135 = 0; c11_i135 < 2; c11_i135++) {
    for (c11_i136 = 0; c11_i136 < 7; c11_i136++) {
      c11_b_inData[c11_i136 + c11_i134] = (*(real_T (*)[14])c11_inData)[c11_i136
        + c11_i134];
    }

    c11_i134 += 7;
  }

  c11_i137 = 0;
  for (c11_i138 = 0; c11_i138 < 2; c11_i138++) {
    for (c11_i139 = 0; c11_i139 < 7; c11_i139++) {
      c11_u[c11_i139 + c11_i137] = c11_b_inData[c11_i139 + c11_i137];
    }

    c11_i137 += 7;
  }

  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 0, 0U, 1U, 0U, 2, 7, 2), false);
  sf_mex_assign(&c11_mxArrayOutData, c11_y, false);
  return c11_mxArrayOutData;
}

static void c11_g_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[14])
{
  real_T c11_dv9[14];
  int32_T c11_i140;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_u), c11_dv9, 1, 0, 0U, 1, 0U, 2, 7,
                2);
  for (c11_i140 = 0; c11_i140 < 14; c11_i140++) {
    c11_y[c11_i140] = c11_dv9[c11_i140];
  }

  sf_mex_destroy(&c11_u);
}

static void c11_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_K;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  real_T c11_y[14];
  int32_T c11_i141;
  int32_T c11_i142;
  int32_T c11_i143;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_K = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_K), &c11_thisId, c11_y);
  sf_mex_destroy(&c11_K);
  c11_i141 = 0;
  for (c11_i142 = 0; c11_i142 < 2; c11_i142++) {
    for (c11_i143 = 0; c11_i143 < 7; c11_i143++) {
      (*(real_T (*)[14])c11_outData)[c11_i143 + c11_i141] = c11_y[c11_i143 +
        c11_i141];
    }

    c11_i141 += 7;
  }

  sf_mex_destroy(&c11_mxArrayInData);
}

static const mxArray *c11_e_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  int32_T c11_i144;
  real_T c11_b_inData[2];
  int32_T c11_i145;
  real_T c11_u[2];
  const mxArray *c11_y = NULL;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  for (c11_i144 = 0; c11_i144 < 2; c11_i144++) {
    c11_b_inData[c11_i144] = (*(real_T (*)[2])c11_inData)[c11_i144];
  }

  for (c11_i145 = 0; c11_i145 < 2; c11_i145++) {
    c11_u[c11_i145] = c11_b_inData[c11_i145];
  }

  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c11_mxArrayOutData, c11_y, false);
  return c11_mxArrayOutData;
}

static void c11_h_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[2])
{
  real_T c11_dv10[2];
  int32_T c11_i146;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_u), c11_dv10, 1, 0, 0U, 1, 0U, 1, 2);
  for (c11_i146 = 0; c11_i146 < 2; c11_i146++) {
    c11_y[c11_i146] = c11_dv10[c11_i146];
  }

  sf_mex_destroy(&c11_u);
}

static void c11_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_y_meas;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  real_T c11_y[2];
  int32_T c11_i147;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_y_meas = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_y_meas), &c11_thisId,
    c11_y);
  sf_mex_destroy(&c11_y_meas);
  for (c11_i147 = 0; c11_i147 < 2; c11_i147++) {
    (*(real_T (*)[2])c11_outData)[c11_i147] = c11_y[c11_i147];
  }

  sf_mex_destroy(&c11_mxArrayInData);
}

static const mxArray *c11_f_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  int32_T c11_i148;
  int32_T c11_i149;
  int32_T c11_i150;
  real_T c11_b_inData[14];
  int32_T c11_i151;
  int32_T c11_i152;
  int32_T c11_i153;
  real_T c11_u[14];
  const mxArray *c11_y = NULL;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  c11_i148 = 0;
  for (c11_i149 = 0; c11_i149 < 7; c11_i149++) {
    for (c11_i150 = 0; c11_i150 < 2; c11_i150++) {
      c11_b_inData[c11_i150 + c11_i148] = (*(real_T (*)[14])c11_inData)[c11_i150
        + c11_i148];
    }

    c11_i148 += 2;
  }

  c11_i151 = 0;
  for (c11_i152 = 0; c11_i152 < 7; c11_i152++) {
    for (c11_i153 = 0; c11_i153 < 2; c11_i153++) {
      c11_u[c11_i153 + c11_i151] = c11_b_inData[c11_i153 + c11_i151];
    }

    c11_i151 += 2;
  }

  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 0, 0U, 1U, 0U, 2, 2, 7), false);
  sf_mex_assign(&c11_mxArrayOutData, c11_y, false);
  return c11_mxArrayOutData;
}

static const mxArray *c11_g_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  int32_T c11_i154;
  real_T c11_b_inData[7];
  int32_T c11_i155;
  real_T c11_u[7];
  const mxArray *c11_y = NULL;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  for (c11_i154 = 0; c11_i154 < 7; c11_i154++) {
    c11_b_inData[c11_i154] = (*(real_T (*)[7])c11_inData)[c11_i154];
  }

  for (c11_i155 = 0; c11_i155 < 7; c11_i155++) {
    c11_u[c11_i155] = c11_b_inData[c11_i155];
  }

  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 0, 0U, 1U, 0U, 1, 7), false);
  sf_mex_assign(&c11_mxArrayOutData, c11_y, false);
  return c11_mxArrayOutData;
}

static void c11_i_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[7])
{
  real_T c11_dv11[7];
  int32_T c11_i156;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_u), c11_dv11, 1, 0, 0U, 1, 0U, 1, 7);
  for (c11_i156 = 0; c11_i156 < 7; c11_i156++) {
    c11_y[c11_i156] = c11_dv11[c11_i156];
  }

  sf_mex_destroy(&c11_u);
}

static void c11_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_State_p;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  real_T c11_y[7];
  int32_T c11_i157;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_State_p = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_State_p), &c11_thisId,
    c11_y);
  sf_mex_destroy(&c11_State_p);
  for (c11_i157 = 0; c11_i157 < 7; c11_i157++) {
    (*(real_T (*)[7])c11_outData)[c11_i157] = c11_y[c11_i157];
  }

  sf_mex_destroy(&c11_mxArrayInData);
}

static const mxArray *c11_h_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  int32_T c11_i158;
  int32_T c11_i159;
  int32_T c11_i160;
  real_T c11_b_inData[49];
  int32_T c11_i161;
  int32_T c11_i162;
  int32_T c11_i163;
  real_T c11_u[49];
  const mxArray *c11_y = NULL;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  c11_i158 = 0;
  for (c11_i159 = 0; c11_i159 < 7; c11_i159++) {
    for (c11_i160 = 0; c11_i160 < 7; c11_i160++) {
      c11_b_inData[c11_i160 + c11_i158] = (*(real_T (*)[49])c11_inData)[c11_i160
        + c11_i158];
    }

    c11_i158 += 7;
  }

  c11_i161 = 0;
  for (c11_i162 = 0; c11_i162 < 7; c11_i162++) {
    for (c11_i163 = 0; c11_i163 < 7; c11_i163++) {
      c11_u[c11_i163 + c11_i161] = c11_b_inData[c11_i163 + c11_i161];
    }

    c11_i161 += 7;
  }

  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 0, 0U, 1U, 0U, 2, 7, 7), false);
  sf_mex_assign(&c11_mxArrayOutData, c11_y, false);
  return c11_mxArrayOutData;
}

static void c11_j_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[49])
{
  real_T c11_dv12[49];
  int32_T c11_i164;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_u), c11_dv12, 1, 0, 0U, 1, 0U, 2, 7,
                7);
  for (c11_i164 = 0; c11_i164 < 49; c11_i164++) {
    c11_y[c11_i164] = c11_dv12[c11_i164];
  }

  sf_mex_destroy(&c11_u);
}

static void c11_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_G;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  real_T c11_y[49];
  int32_T c11_i165;
  int32_T c11_i166;
  int32_T c11_i167;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_G = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_G), &c11_thisId, c11_y);
  sf_mex_destroy(&c11_G);
  c11_i165 = 0;
  for (c11_i166 = 0; c11_i166 < 7; c11_i166++) {
    for (c11_i167 = 0; c11_i167 < 7; c11_i167++) {
      (*(real_T (*)[49])c11_outData)[c11_i167 + c11_i165] = c11_y[c11_i167 +
        c11_i165];
    }

    c11_i165 += 7;
  }

  sf_mex_destroy(&c11_mxArrayInData);
}

static const mxArray *c11_i_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  int32_T c11_i168;
  int32_T c11_i169;
  int32_T c11_i170;
  real_T c11_b_inData[21];
  int32_T c11_i171;
  int32_T c11_i172;
  int32_T c11_i173;
  real_T c11_u[21];
  const mxArray *c11_y = NULL;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  c11_i168 = 0;
  for (c11_i169 = 0; c11_i169 < 3; c11_i169++) {
    for (c11_i170 = 0; c11_i170 < 7; c11_i170++) {
      c11_b_inData[c11_i170 + c11_i168] = (*(real_T (*)[21])c11_inData)[c11_i170
        + c11_i168];
    }

    c11_i168 += 7;
  }

  c11_i171 = 0;
  for (c11_i172 = 0; c11_i172 < 3; c11_i172++) {
    for (c11_i173 = 0; c11_i173 < 7; c11_i173++) {
      c11_u[c11_i173 + c11_i171] = c11_b_inData[c11_i173 + c11_i171];
    }

    c11_i171 += 7;
  }

  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 0, 0U, 1U, 0U, 2, 7, 3), false);
  sf_mex_assign(&c11_mxArrayOutData, c11_y, false);
  return c11_mxArrayOutData;
}

static const mxArray *c11_j_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  int32_T c11_i174;
  int32_T c11_i175;
  int32_T c11_i176;
  real_T c11_b_inData[4];
  int32_T c11_i177;
  int32_T c11_i178;
  int32_T c11_i179;
  real_T c11_u[4];
  const mxArray *c11_y = NULL;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  c11_i174 = 0;
  for (c11_i175 = 0; c11_i175 < 2; c11_i175++) {
    for (c11_i176 = 0; c11_i176 < 2; c11_i176++) {
      c11_b_inData[c11_i176 + c11_i174] = (*(real_T (*)[4])c11_inData)[c11_i176
        + c11_i174];
    }

    c11_i174 += 2;
  }

  c11_i177 = 0;
  for (c11_i178 = 0; c11_i178 < 2; c11_i178++) {
    for (c11_i179 = 0; c11_i179 < 2; c11_i179++) {
      c11_u[c11_i179 + c11_i177] = c11_b_inData[c11_i179 + c11_i177];
    }

    c11_i177 += 2;
  }

  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 0, 0U, 1U, 0U, 2, 2, 2), false);
  sf_mex_assign(&c11_mxArrayOutData, c11_y, false);
  return c11_mxArrayOutData;
}

static void c11_k_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId,
  real_T c11_y[4])
{
  real_T c11_dv13[4];
  int32_T c11_i180;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_u), c11_dv13, 1, 0, 0U, 1, 0U, 2, 2,
                2);
  for (c11_i180 = 0; c11_i180 < 4; c11_i180++) {
    c11_y[c11_i180] = c11_dv13[c11_i180];
  }

  sf_mex_destroy(&c11_u);
}

static void c11_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_R;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  real_T c11_y[4];
  int32_T c11_i181;
  int32_T c11_i182;
  int32_T c11_i183;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_R = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_R), &c11_thisId, c11_y);
  sf_mex_destroy(&c11_R);
  c11_i181 = 0;
  for (c11_i182 = 0; c11_i182 < 2; c11_i182++) {
    for (c11_i183 = 0; c11_i183 < 2; c11_i183++) {
      (*(real_T (*)[4])c11_outData)[c11_i183 + c11_i181] = c11_y[c11_i183 +
        c11_i181];
    }

    c11_i181 += 2;
  }

  sf_mex_destroy(&c11_mxArrayInData);
}

const mxArray *sf_c11_SS6_Estimation2_get_eml_resolved_functions_info(void)
{
  const mxArray *c11_nameCaptureInfo = NULL;
  c11_nameCaptureInfo = NULL;
  sf_mex_assign(&c11_nameCaptureInfo, sf_mex_createstruct("structure", 2, 210, 1),
                false);
  c11_info_helper(&c11_nameCaptureInfo);
  c11_b_info_helper(&c11_nameCaptureInfo);
  c11_c_info_helper(&c11_nameCaptureInfo);
  c11_d_info_helper(&c11_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c11_nameCaptureInfo);
  return c11_nameCaptureInfo;
}

static void c11_info_helper(const mxArray **c11_info)
{
  const mxArray *c11_rhs0 = NULL;
  const mxArray *c11_lhs0 = NULL;
  const mxArray *c11_rhs1 = NULL;
  const mxArray *c11_lhs1 = NULL;
  const mxArray *c11_rhs2 = NULL;
  const mxArray *c11_lhs2 = NULL;
  const mxArray *c11_rhs3 = NULL;
  const mxArray *c11_lhs3 = NULL;
  const mxArray *c11_rhs4 = NULL;
  const mxArray *c11_lhs4 = NULL;
  const mxArray *c11_rhs5 = NULL;
  const mxArray *c11_lhs5 = NULL;
  const mxArray *c11_rhs6 = NULL;
  const mxArray *c11_lhs6 = NULL;
  const mxArray *c11_rhs7 = NULL;
  const mxArray *c11_lhs7 = NULL;
  const mxArray *c11_rhs8 = NULL;
  const mxArray *c11_lhs8 = NULL;
  const mxArray *c11_rhs9 = NULL;
  const mxArray *c11_lhs9 = NULL;
  const mxArray *c11_rhs10 = NULL;
  const mxArray *c11_lhs10 = NULL;
  const mxArray *c11_rhs11 = NULL;
  const mxArray *c11_lhs11 = NULL;
  const mxArray *c11_rhs12 = NULL;
  const mxArray *c11_lhs12 = NULL;
  const mxArray *c11_rhs13 = NULL;
  const mxArray *c11_lhs13 = NULL;
  const mxArray *c11_rhs14 = NULL;
  const mxArray *c11_lhs14 = NULL;
  const mxArray *c11_rhs15 = NULL;
  const mxArray *c11_lhs15 = NULL;
  const mxArray *c11_rhs16 = NULL;
  const mxArray *c11_lhs16 = NULL;
  const mxArray *c11_rhs17 = NULL;
  const mxArray *c11_lhs17 = NULL;
  const mxArray *c11_rhs18 = NULL;
  const mxArray *c11_lhs18 = NULL;
  const mxArray *c11_rhs19 = NULL;
  const mxArray *c11_lhs19 = NULL;
  const mxArray *c11_rhs20 = NULL;
  const mxArray *c11_lhs20 = NULL;
  const mxArray *c11_rhs21 = NULL;
  const mxArray *c11_lhs21 = NULL;
  const mxArray *c11_rhs22 = NULL;
  const mxArray *c11_lhs22 = NULL;
  const mxArray *c11_rhs23 = NULL;
  const mxArray *c11_lhs23 = NULL;
  const mxArray *c11_rhs24 = NULL;
  const mxArray *c11_lhs24 = NULL;
  const mxArray *c11_rhs25 = NULL;
  const mxArray *c11_lhs25 = NULL;
  const mxArray *c11_rhs26 = NULL;
  const mxArray *c11_lhs26 = NULL;
  const mxArray *c11_rhs27 = NULL;
  const mxArray *c11_lhs27 = NULL;
  const mxArray *c11_rhs28 = NULL;
  const mxArray *c11_lhs28 = NULL;
  const mxArray *c11_rhs29 = NULL;
  const mxArray *c11_lhs29 = NULL;
  const mxArray *c11_rhs30 = NULL;
  const mxArray *c11_lhs30 = NULL;
  const mxArray *c11_rhs31 = NULL;
  const mxArray *c11_lhs31 = NULL;
  const mxArray *c11_rhs32 = NULL;
  const mxArray *c11_lhs32 = NULL;
  const mxArray *c11_rhs33 = NULL;
  const mxArray *c11_lhs33 = NULL;
  const mxArray *c11_rhs34 = NULL;
  const mxArray *c11_lhs34 = NULL;
  const mxArray *c11_rhs35 = NULL;
  const mxArray *c11_lhs35 = NULL;
  const mxArray *c11_rhs36 = NULL;
  const mxArray *c11_lhs36 = NULL;
  const mxArray *c11_rhs37 = NULL;
  const mxArray *c11_lhs37 = NULL;
  const mxArray *c11_rhs38 = NULL;
  const mxArray *c11_lhs38 = NULL;
  const mxArray *c11_rhs39 = NULL;
  const mxArray *c11_lhs39 = NULL;
  const mxArray *c11_rhs40 = NULL;
  const mxArray *c11_lhs40 = NULL;
  const mxArray *c11_rhs41 = NULL;
  const mxArray *c11_lhs41 = NULL;
  const mxArray *c11_rhs42 = NULL;
  const mxArray *c11_lhs42 = NULL;
  const mxArray *c11_rhs43 = NULL;
  const mxArray *c11_lhs43 = NULL;
  const mxArray *c11_rhs44 = NULL;
  const mxArray *c11_lhs44 = NULL;
  const mxArray *c11_rhs45 = NULL;
  const mxArray *c11_lhs45 = NULL;
  const mxArray *c11_rhs46 = NULL;
  const mxArray *c11_lhs46 = NULL;
  const mxArray *c11_rhs47 = NULL;
  const mxArray *c11_lhs47 = NULL;
  const mxArray *c11_rhs48 = NULL;
  const mxArray *c11_lhs48 = NULL;
  const mxArray *c11_rhs49 = NULL;
  const mxArray *c11_lhs49 = NULL;
  const mxArray *c11_rhs50 = NULL;
  const mxArray *c11_lhs50 = NULL;
  const mxArray *c11_rhs51 = NULL;
  const mxArray *c11_lhs51 = NULL;
  const mxArray *c11_rhs52 = NULL;
  const mxArray *c11_lhs52 = NULL;
  const mxArray *c11_rhs53 = NULL;
  const mxArray *c11_lhs53 = NULL;
  const mxArray *c11_rhs54 = NULL;
  const mxArray *c11_lhs54 = NULL;
  const mxArray *c11_rhs55 = NULL;
  const mxArray *c11_lhs55 = NULL;
  const mxArray *c11_rhs56 = NULL;
  const mxArray *c11_lhs56 = NULL;
  const mxArray *c11_rhs57 = NULL;
  const mxArray *c11_lhs57 = NULL;
  const mxArray *c11_rhs58 = NULL;
  const mxArray *c11_lhs58 = NULL;
  const mxArray *c11_rhs59 = NULL;
  const mxArray *c11_lhs59 = NULL;
  const mxArray *c11_rhs60 = NULL;
  const mxArray *c11_lhs60 = NULL;
  const mxArray *c11_rhs61 = NULL;
  const mxArray *c11_lhs61 = NULL;
  const mxArray *c11_rhs62 = NULL;
  const mxArray *c11_lhs62 = NULL;
  const mxArray *c11_rhs63 = NULL;
  const mxArray *c11_lhs63 = NULL;
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eye"), "name", "name", 0);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1406834748U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c11_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs0), "rhs", "rhs",
                  0);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs0), "lhs", "lhs",
                  0);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 1);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1368204630U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c11_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs1), "rhs", "rhs",
                  1);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs1), "lhs", "lhs",
                  1);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 2);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 2);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c11_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs2), "rhs", "rhs",
                  2);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs2), "lhs", "lhs",
                  2);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral"),
                  "context", "context", 3);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("isinf"), "name", "name", 3);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 3);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731856U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c11_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs3), "rhs", "rhs",
                  3);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs3), "lhs", "lhs",
                  3);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 4);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c11_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs4), "rhs", "rhs",
                  4);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs4), "lhs", "lhs",
                  4);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 5);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_is_integer_class"),
                  "name", "name", 5);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_integer_class.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840382U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c11_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs5), "rhs", "rhs",
                  5);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs5), "lhs", "lhs",
                  5);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 6);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("intmax"), "name", "name", 6);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c11_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs6), "rhs", "rhs",
                  6);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs6), "lhs", "lhs",
                  6);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 7);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1393352458U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c11_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs7), "rhs", "rhs",
                  7);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs7), "lhs", "lhs",
                  7);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 8);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("intmin"), "name", "name", 8);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 8);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c11_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs8), "rhs", "rhs",
                  8);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs8), "lhs", "lhs",
                  8);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 9);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1393352458U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c11_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs9), "rhs", "rhs",
                  9);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs9), "lhs", "lhs",
                  9);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 10);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.indexIntRelop"), "name", "name", 10);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1326749922U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c11_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!apply_float_relop"),
                  "context", "context", 11);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 11);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1393352458U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c11_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 12);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 12);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c11_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 13);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("intmin"), "name", "name", 13);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c11_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 14);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 14);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c11_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 15);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("intmax"), "name", "name", 15);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c11_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 16);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c11_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 17);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("isfi"), "name", "name", 17);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 17);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isfi.m"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1346531958U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c11_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isfi.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("isnumerictype"), "name",
                  "name", 18);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isnumerictype.m"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1398897198U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c11_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 19);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("intmax"), "name", "name", 19);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c11_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 20);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("intmin"), "name", "name", 20);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 20);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c11_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "context", "context", 21);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 21);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1383898894U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c11_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 22);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 22);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c11_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "context", "context", 23);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("cos"), "name", "name", 23);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 23);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395346496U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c11_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 24);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840322U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c11_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "context", "context", 25);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("sin"), "name", "name", 25);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 25);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395346504U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c11_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 26);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 26);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840336U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c11_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 27);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 27);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c11_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 28);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 28);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 28);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c11_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 29);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 29);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c11_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 30);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  30);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c11_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 31);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 31);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c11_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 32);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 32);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c11_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 33);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 33);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c11_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 34);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 34);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c11_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 35);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 35);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1393352458U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c11_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 36);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 36);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c11_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 37);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.refblas.xgemm"), "name", "name", 37);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c11_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "context", "context", 38);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("pinv"), "name", "name", 38);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840428U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c11_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 39);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 39);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c11_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 40);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 40);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 40);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c11_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 41);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("svd"), "name", "name", 41);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "resolved",
                  "resolved", 41);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840432U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c11_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 42);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 42);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c11_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 43);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 43);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c11_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 44);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("isfinite"), "name", "name",
                  44);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "resolved",
                  "resolved", 44);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731856U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c11_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 45);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 45);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c11_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 46);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("isinf"), "name", "name", 46);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 46);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731856U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c11_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 47);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("isnan"), "name", "name", 47);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 47);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c11_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 48);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 48);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 48);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c11_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 49);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_error"), "name", "name",
                  49);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 49);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c11_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 50);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_xgesvd"), "name", "name",
                  50);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m"),
                  "resolved", "resolved", 50);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840406U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c11_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m"),
                  "context", "context", 51);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_lapack_xgesvd"), "name",
                  "name", 51);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840410U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c11_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m"),
                  "context", "context", 52);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_matlab_zsvdc"), "name",
                  "name", 52);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "resolved", "resolved", 52);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1398897206U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c11_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 53);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 53);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 53);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c11_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 54);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("min"), "name", "name", 54);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 54);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 54);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1311276918U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c11_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "context",
                  "context", 55);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 55);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 55);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1378317584U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c11_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 56);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 56);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 56);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 56);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c11_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 57);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 57);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 57);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c11_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 58);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 58);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 58);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 58);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c11_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 59);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 59);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 59);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 59);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c11_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 60);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 60);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c11_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 61);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 61);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 61);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 61);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c11_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 62);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 62);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 62);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 62);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c11_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 63);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("max"), "name", "name", 63);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 63);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1311276916U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c11_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c11_rhs0);
  sf_mex_destroy(&c11_lhs0);
  sf_mex_destroy(&c11_rhs1);
  sf_mex_destroy(&c11_lhs1);
  sf_mex_destroy(&c11_rhs2);
  sf_mex_destroy(&c11_lhs2);
  sf_mex_destroy(&c11_rhs3);
  sf_mex_destroy(&c11_lhs3);
  sf_mex_destroy(&c11_rhs4);
  sf_mex_destroy(&c11_lhs4);
  sf_mex_destroy(&c11_rhs5);
  sf_mex_destroy(&c11_lhs5);
  sf_mex_destroy(&c11_rhs6);
  sf_mex_destroy(&c11_lhs6);
  sf_mex_destroy(&c11_rhs7);
  sf_mex_destroy(&c11_lhs7);
  sf_mex_destroy(&c11_rhs8);
  sf_mex_destroy(&c11_lhs8);
  sf_mex_destroy(&c11_rhs9);
  sf_mex_destroy(&c11_lhs9);
  sf_mex_destroy(&c11_rhs10);
  sf_mex_destroy(&c11_lhs10);
  sf_mex_destroy(&c11_rhs11);
  sf_mex_destroy(&c11_lhs11);
  sf_mex_destroy(&c11_rhs12);
  sf_mex_destroy(&c11_lhs12);
  sf_mex_destroy(&c11_rhs13);
  sf_mex_destroy(&c11_lhs13);
  sf_mex_destroy(&c11_rhs14);
  sf_mex_destroy(&c11_lhs14);
  sf_mex_destroy(&c11_rhs15);
  sf_mex_destroy(&c11_lhs15);
  sf_mex_destroy(&c11_rhs16);
  sf_mex_destroy(&c11_lhs16);
  sf_mex_destroy(&c11_rhs17);
  sf_mex_destroy(&c11_lhs17);
  sf_mex_destroy(&c11_rhs18);
  sf_mex_destroy(&c11_lhs18);
  sf_mex_destroy(&c11_rhs19);
  sf_mex_destroy(&c11_lhs19);
  sf_mex_destroy(&c11_rhs20);
  sf_mex_destroy(&c11_lhs20);
  sf_mex_destroy(&c11_rhs21);
  sf_mex_destroy(&c11_lhs21);
  sf_mex_destroy(&c11_rhs22);
  sf_mex_destroy(&c11_lhs22);
  sf_mex_destroy(&c11_rhs23);
  sf_mex_destroy(&c11_lhs23);
  sf_mex_destroy(&c11_rhs24);
  sf_mex_destroy(&c11_lhs24);
  sf_mex_destroy(&c11_rhs25);
  sf_mex_destroy(&c11_lhs25);
  sf_mex_destroy(&c11_rhs26);
  sf_mex_destroy(&c11_lhs26);
  sf_mex_destroy(&c11_rhs27);
  sf_mex_destroy(&c11_lhs27);
  sf_mex_destroy(&c11_rhs28);
  sf_mex_destroy(&c11_lhs28);
  sf_mex_destroy(&c11_rhs29);
  sf_mex_destroy(&c11_lhs29);
  sf_mex_destroy(&c11_rhs30);
  sf_mex_destroy(&c11_lhs30);
  sf_mex_destroy(&c11_rhs31);
  sf_mex_destroy(&c11_lhs31);
  sf_mex_destroy(&c11_rhs32);
  sf_mex_destroy(&c11_lhs32);
  sf_mex_destroy(&c11_rhs33);
  sf_mex_destroy(&c11_lhs33);
  sf_mex_destroy(&c11_rhs34);
  sf_mex_destroy(&c11_lhs34);
  sf_mex_destroy(&c11_rhs35);
  sf_mex_destroy(&c11_lhs35);
  sf_mex_destroy(&c11_rhs36);
  sf_mex_destroy(&c11_lhs36);
  sf_mex_destroy(&c11_rhs37);
  sf_mex_destroy(&c11_lhs37);
  sf_mex_destroy(&c11_rhs38);
  sf_mex_destroy(&c11_lhs38);
  sf_mex_destroy(&c11_rhs39);
  sf_mex_destroy(&c11_lhs39);
  sf_mex_destroy(&c11_rhs40);
  sf_mex_destroy(&c11_lhs40);
  sf_mex_destroy(&c11_rhs41);
  sf_mex_destroy(&c11_lhs41);
  sf_mex_destroy(&c11_rhs42);
  sf_mex_destroy(&c11_lhs42);
  sf_mex_destroy(&c11_rhs43);
  sf_mex_destroy(&c11_lhs43);
  sf_mex_destroy(&c11_rhs44);
  sf_mex_destroy(&c11_lhs44);
  sf_mex_destroy(&c11_rhs45);
  sf_mex_destroy(&c11_lhs45);
  sf_mex_destroy(&c11_rhs46);
  sf_mex_destroy(&c11_lhs46);
  sf_mex_destroy(&c11_rhs47);
  sf_mex_destroy(&c11_lhs47);
  sf_mex_destroy(&c11_rhs48);
  sf_mex_destroy(&c11_lhs48);
  sf_mex_destroy(&c11_rhs49);
  sf_mex_destroy(&c11_lhs49);
  sf_mex_destroy(&c11_rhs50);
  sf_mex_destroy(&c11_lhs50);
  sf_mex_destroy(&c11_rhs51);
  sf_mex_destroy(&c11_lhs51);
  sf_mex_destroy(&c11_rhs52);
  sf_mex_destroy(&c11_lhs52);
  sf_mex_destroy(&c11_rhs53);
  sf_mex_destroy(&c11_lhs53);
  sf_mex_destroy(&c11_rhs54);
  sf_mex_destroy(&c11_lhs54);
  sf_mex_destroy(&c11_rhs55);
  sf_mex_destroy(&c11_lhs55);
  sf_mex_destroy(&c11_rhs56);
  sf_mex_destroy(&c11_lhs56);
  sf_mex_destroy(&c11_rhs57);
  sf_mex_destroy(&c11_lhs57);
  sf_mex_destroy(&c11_rhs58);
  sf_mex_destroy(&c11_lhs58);
  sf_mex_destroy(&c11_rhs59);
  sf_mex_destroy(&c11_lhs59);
  sf_mex_destroy(&c11_rhs60);
  sf_mex_destroy(&c11_lhs60);
  sf_mex_destroy(&c11_rhs61);
  sf_mex_destroy(&c11_lhs61);
  sf_mex_destroy(&c11_rhs62);
  sf_mex_destroy(&c11_lhs62);
  sf_mex_destroy(&c11_rhs63);
  sf_mex_destroy(&c11_lhs63);
}

static const mxArray *c11_emlrt_marshallOut(const char * c11_u)
{
  const mxArray *c11_y = NULL;
  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c11_u)), false);
  return c11_y;
}

static const mxArray *c11_b_emlrt_marshallOut(const uint32_T c11_u)
{
  const mxArray *c11_y = NULL;
  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", &c11_u, 7, 0U, 0U, 0U, 0), false);
  return c11_y;
}

static void c11_b_info_helper(const mxArray **c11_info)
{
  const mxArray *c11_rhs64 = NULL;
  const mxArray *c11_lhs64 = NULL;
  const mxArray *c11_rhs65 = NULL;
  const mxArray *c11_lhs65 = NULL;
  const mxArray *c11_rhs66 = NULL;
  const mxArray *c11_lhs66 = NULL;
  const mxArray *c11_rhs67 = NULL;
  const mxArray *c11_lhs67 = NULL;
  const mxArray *c11_rhs68 = NULL;
  const mxArray *c11_lhs68 = NULL;
  const mxArray *c11_rhs69 = NULL;
  const mxArray *c11_lhs69 = NULL;
  const mxArray *c11_rhs70 = NULL;
  const mxArray *c11_lhs70 = NULL;
  const mxArray *c11_rhs71 = NULL;
  const mxArray *c11_lhs71 = NULL;
  const mxArray *c11_rhs72 = NULL;
  const mxArray *c11_lhs72 = NULL;
  const mxArray *c11_rhs73 = NULL;
  const mxArray *c11_lhs73 = NULL;
  const mxArray *c11_rhs74 = NULL;
  const mxArray *c11_lhs74 = NULL;
  const mxArray *c11_rhs75 = NULL;
  const mxArray *c11_lhs75 = NULL;
  const mxArray *c11_rhs76 = NULL;
  const mxArray *c11_lhs76 = NULL;
  const mxArray *c11_rhs77 = NULL;
  const mxArray *c11_lhs77 = NULL;
  const mxArray *c11_rhs78 = NULL;
  const mxArray *c11_lhs78 = NULL;
  const mxArray *c11_rhs79 = NULL;
  const mxArray *c11_lhs79 = NULL;
  const mxArray *c11_rhs80 = NULL;
  const mxArray *c11_lhs80 = NULL;
  const mxArray *c11_rhs81 = NULL;
  const mxArray *c11_lhs81 = NULL;
  const mxArray *c11_rhs82 = NULL;
  const mxArray *c11_lhs82 = NULL;
  const mxArray *c11_rhs83 = NULL;
  const mxArray *c11_lhs83 = NULL;
  const mxArray *c11_rhs84 = NULL;
  const mxArray *c11_lhs84 = NULL;
  const mxArray *c11_rhs85 = NULL;
  const mxArray *c11_lhs85 = NULL;
  const mxArray *c11_rhs86 = NULL;
  const mxArray *c11_lhs86 = NULL;
  const mxArray *c11_rhs87 = NULL;
  const mxArray *c11_lhs87 = NULL;
  const mxArray *c11_rhs88 = NULL;
  const mxArray *c11_lhs88 = NULL;
  const mxArray *c11_rhs89 = NULL;
  const mxArray *c11_lhs89 = NULL;
  const mxArray *c11_rhs90 = NULL;
  const mxArray *c11_lhs90 = NULL;
  const mxArray *c11_rhs91 = NULL;
  const mxArray *c11_lhs91 = NULL;
  const mxArray *c11_rhs92 = NULL;
  const mxArray *c11_lhs92 = NULL;
  const mxArray *c11_rhs93 = NULL;
  const mxArray *c11_lhs93 = NULL;
  const mxArray *c11_rhs94 = NULL;
  const mxArray *c11_lhs94 = NULL;
  const mxArray *c11_rhs95 = NULL;
  const mxArray *c11_lhs95 = NULL;
  const mxArray *c11_rhs96 = NULL;
  const mxArray *c11_lhs96 = NULL;
  const mxArray *c11_rhs97 = NULL;
  const mxArray *c11_lhs97 = NULL;
  const mxArray *c11_rhs98 = NULL;
  const mxArray *c11_lhs98 = NULL;
  const mxArray *c11_rhs99 = NULL;
  const mxArray *c11_lhs99 = NULL;
  const mxArray *c11_rhs100 = NULL;
  const mxArray *c11_lhs100 = NULL;
  const mxArray *c11_rhs101 = NULL;
  const mxArray *c11_lhs101 = NULL;
  const mxArray *c11_rhs102 = NULL;
  const mxArray *c11_lhs102 = NULL;
  const mxArray *c11_rhs103 = NULL;
  const mxArray *c11_lhs103 = NULL;
  const mxArray *c11_rhs104 = NULL;
  const mxArray *c11_lhs104 = NULL;
  const mxArray *c11_rhs105 = NULL;
  const mxArray *c11_lhs105 = NULL;
  const mxArray *c11_rhs106 = NULL;
  const mxArray *c11_lhs106 = NULL;
  const mxArray *c11_rhs107 = NULL;
  const mxArray *c11_lhs107 = NULL;
  const mxArray *c11_rhs108 = NULL;
  const mxArray *c11_lhs108 = NULL;
  const mxArray *c11_rhs109 = NULL;
  const mxArray *c11_lhs109 = NULL;
  const mxArray *c11_rhs110 = NULL;
  const mxArray *c11_lhs110 = NULL;
  const mxArray *c11_rhs111 = NULL;
  const mxArray *c11_lhs111 = NULL;
  const mxArray *c11_rhs112 = NULL;
  const mxArray *c11_lhs112 = NULL;
  const mxArray *c11_rhs113 = NULL;
  const mxArray *c11_lhs113 = NULL;
  const mxArray *c11_rhs114 = NULL;
  const mxArray *c11_lhs114 = NULL;
  const mxArray *c11_rhs115 = NULL;
  const mxArray *c11_lhs115 = NULL;
  const mxArray *c11_rhs116 = NULL;
  const mxArray *c11_lhs116 = NULL;
  const mxArray *c11_rhs117 = NULL;
  const mxArray *c11_lhs117 = NULL;
  const mxArray *c11_rhs118 = NULL;
  const mxArray *c11_lhs118 = NULL;
  const mxArray *c11_rhs119 = NULL;
  const mxArray *c11_lhs119 = NULL;
  const mxArray *c11_rhs120 = NULL;
  const mxArray *c11_lhs120 = NULL;
  const mxArray *c11_rhs121 = NULL;
  const mxArray *c11_lhs121 = NULL;
  const mxArray *c11_rhs122 = NULL;
  const mxArray *c11_lhs122 = NULL;
  const mxArray *c11_rhs123 = NULL;
  const mxArray *c11_lhs123 = NULL;
  const mxArray *c11_rhs124 = NULL;
  const mxArray *c11_lhs124 = NULL;
  const mxArray *c11_rhs125 = NULL;
  const mxArray *c11_lhs125 = NULL;
  const mxArray *c11_rhs126 = NULL;
  const mxArray *c11_lhs126 = NULL;
  const mxArray *c11_rhs127 = NULL;
  const mxArray *c11_lhs127 = NULL;
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "context",
                  "context", 64);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 64);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 64);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1378317584U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c11_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 65);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 65);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 65);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c11_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 66);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 66);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 66);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c11_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 67);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 67);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 67);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c11_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 68);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 68);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 68);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c11_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 69);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_relop"), "name", "name",
                  69);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 69);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 69);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1342472782U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c11_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "context",
                  "context", 70);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.indexIntRelop"), "name", "name", 70);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 70);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 70);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1326749922U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c11_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 71);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("isnan"), "name", "name", 71);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 71);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 71);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c11_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 72);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 72);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 72);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c11_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 73);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("max"), "name", "name", 73);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 73);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 73);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1311276916U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c11_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 74);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_xnrm2"), "name", "name",
                  74);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 74);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c11_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 75);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 75);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 75);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c11_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 76);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.xnrm2"),
                  "name", "name", 76);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "resolved", "resolved", 76);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c11_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 77);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 77);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 77);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c11_rhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 78);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 78);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 78);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 78);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c11_rhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 79);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.refblas.xnrm2"), "name", "name", 79);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 79);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "resolved", "resolved", 79);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c11_rhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 80);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("realmin"), "name", "name",
                  80);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 80);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 80);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1307672842U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c11_rhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "context",
                  "context", 81);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_realmin"), "name",
                  "name", 81);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "resolved",
                  "resolved", 81);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1307672844U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c11_rhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "context",
                  "context", 82);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 82);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 82);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 82);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c11_rhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 83);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 83);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 83);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 83);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c11_rhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 84);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 84);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 84);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 84);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c11_rhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 85);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 85);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 85);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 85);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c11_rhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs85), "lhs", "lhs",
                  85);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 86);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 86);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 86);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 86);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 86);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 86);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 86);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 86);
  sf_mex_assign(&c11_rhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs86), "rhs", "rhs",
                  86);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs86), "lhs", "lhs",
                  86);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 87);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("abs"), "name", "name", 87);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 87);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 87);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 87);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 87);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 87);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 87);
  sf_mex_assign(&c11_rhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs87), "rhs", "rhs",
                  87);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs87), "lhs", "lhs",
                  87);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 88);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 88);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 88);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 88);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 88);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 88);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 88);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 88);
  sf_mex_assign(&c11_rhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs88), "rhs", "rhs",
                  88);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs88), "lhs", "lhs",
                  88);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 89);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 89);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 89);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 89);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840312U), "fileTimeLo",
                  "fileTimeLo", 89);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 89);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 89);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 89);
  sf_mex_assign(&c11_rhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs89), "rhs", "rhs",
                  89);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs89), "lhs", "lhs",
                  89);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 90);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.scaleVectorByRecip"), "name", "name", 90);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 90);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "resolved", "resolved", 90);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 90);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 90);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 90);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 90);
  sf_mex_assign(&c11_rhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs90), "rhs", "rhs",
                  90);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs90), "lhs", "lhs",
                  90);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 91);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("realmin"), "name", "name",
                  91);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 91);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 91);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1307672842U), "fileTimeLo",
                  "fileTimeLo", 91);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 91);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 91);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 91);
  sf_mex_assign(&c11_rhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs91), "rhs", "rhs",
                  91);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs91), "lhs", "lhs",
                  91);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 92);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eps"), "name", "name", 92);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 92);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 92);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 92);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 92);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 92);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 92);
  sf_mex_assign(&c11_rhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs92), "rhs", "rhs",
                  92);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs92), "lhs", "lhs",
                  92);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 93);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 93);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 93);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 93);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840382U), "fileTimeLo",
                  "fileTimeLo", 93);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 93);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 93);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 93);
  sf_mex_assign(&c11_rhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs93), "rhs", "rhs",
                  93);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs93), "lhs", "lhs",
                  93);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 94);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_eps"), "name", "name",
                  94);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 94);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 94);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 94);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 94);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 94);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 94);
  sf_mex_assign(&c11_rhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs94), "rhs", "rhs",
                  94);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs94), "lhs", "lhs",
                  94);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 95);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 95);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 95);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 95);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 95);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 95);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 95);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 95);
  sf_mex_assign(&c11_rhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs95), "rhs", "rhs",
                  95);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs95), "lhs", "lhs",
                  95);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 96);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("mrdivide"), "name", "name",
                  96);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 96);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 96);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 96);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 96);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 96);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 96);
  sf_mex_assign(&c11_rhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs96), "rhs", "rhs",
                  96);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs96), "lhs", "lhs",
                  96);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 97);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 97);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 97);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 97);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1389739374U), "fileTimeLo",
                  "fileTimeLo", 97);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 97);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 97);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 97);
  sf_mex_assign(&c11_rhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs97), "rhs", "rhs",
                  97);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs97), "lhs", "lhs",
                  97);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 98);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("rdivide"), "name", "name",
                  98);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 98);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 98);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 98);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 98);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 98);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 98);
  sf_mex_assign(&c11_rhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs98), "rhs", "rhs",
                  98);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs98), "lhs", "lhs",
                  98);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 99);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 99);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 99);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 99);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 99);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 99);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 99);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 99);
  sf_mex_assign(&c11_rhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs99), "rhs", "rhs",
                  99);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs99), "lhs", "lhs",
                  99);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 100);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 100);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 100);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 100);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 100);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 100);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 100);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 100);
  sf_mex_assign(&c11_rhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs100), "rhs",
                  "rhs", 100);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs100), "lhs",
                  "lhs", 100);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 101);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_div"), "name", "name",
                  101);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 101);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 101);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 101);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 101);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 101);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 101);
  sf_mex_assign(&c11_rhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs101), "rhs",
                  "rhs", 101);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs101), "lhs",
                  "lhs", 101);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 102);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 102);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 102);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 102);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 102);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 102);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 102);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 102);
  sf_mex_assign(&c11_rhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs102), "rhs",
                  "rhs", 102);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs102), "lhs",
                  "lhs", 102);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 103);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("abs"), "name", "name", 103);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 103);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 103);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 103);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 103);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 103);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 103);
  sf_mex_assign(&c11_rhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs103), "rhs",
                  "rhs", 103);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs103), "lhs",
                  "lhs", 103);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 104);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.xscal"),
                  "name", "name", 104);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 104);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "resolved", "resolved", 104);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 104);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 104);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 104);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 104);
  sf_mex_assign(&c11_rhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs104), "rhs",
                  "rhs", 104);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs104), "lhs",
                  "lhs", 104);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 105);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 105);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 105);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 105);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 105);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 105);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 105);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 105);
  sf_mex_assign(&c11_rhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs105), "rhs",
                  "rhs", 105);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs105), "lhs",
                  "lhs", 105);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 106);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 106);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 106);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 106);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 106);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 106);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 106);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 106);
  sf_mex_assign(&c11_rhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs106), "rhs",
                  "rhs", 106);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs106), "lhs",
                  "lhs", 106);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 107);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 107);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 107);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 107);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 107);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 107);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 107);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 107);
  sf_mex_assign(&c11_rhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs107), "rhs",
                  "rhs", 107);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs107), "lhs",
                  "lhs", 107);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 108);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.refblas.xscal"), "name", "name", 108);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 108);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "resolved", "resolved", 108);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 108);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 108);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 108);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 108);
  sf_mex_assign(&c11_rhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs108), "rhs",
                  "rhs", 108);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs108), "lhs",
                  "lhs", 108);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 109);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 109);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 109);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 109);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 109);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 109);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 109);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 109);
  sf_mex_assign(&c11_rhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs109), "rhs",
                  "rhs", 109);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs109), "lhs",
                  "lhs", 109);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 110);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 110);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 110);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 110);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 110);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 110);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 110);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 110);
  sf_mex_assign(&c11_rhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs110), "rhs",
                  "rhs", 110);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs110), "lhs",
                  "lhs", 110);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 111);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 111);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 111);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 111);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 111);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 111);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 111);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 111);
  sf_mex_assign(&c11_rhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs111), "rhs",
                  "rhs", 111);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs111), "lhs",
                  "lhs", 111);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 112);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 112);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 112);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 112);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 112);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 112);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 112);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 112);
  sf_mex_assign(&c11_rhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs112), "rhs",
                  "rhs", 112);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs112), "lhs",
                  "lhs", 112);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 113);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 113);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 113);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 113);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 113);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 113);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 113);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 113);
  sf_mex_assign(&c11_rhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs113), "rhs",
                  "rhs", 113);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs113), "lhs",
                  "lhs", 113);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 114);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 114);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 114);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 114);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 114);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 114);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 114);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 114);
  sf_mex_assign(&c11_rhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs114), "rhs",
                  "rhs", 114);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs114), "lhs",
                  "lhs", 114);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 115);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 115);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 115);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 115);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 115);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 115);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 115);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 115);
  sf_mex_assign(&c11_rhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs115), "rhs",
                  "rhs", 115);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs115), "lhs",
                  "lhs", 115);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 116);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 116);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 116);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 116);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 116);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 116);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 116);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 116);
  sf_mex_assign(&c11_rhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs116), "rhs",
                  "rhs", 116);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs116), "lhs",
                  "lhs", 116);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 117);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_xdotc"), "name", "name",
                  117);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 117);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"),
                  "resolved", "resolved", 117);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 117);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 117);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 117);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 117);
  sf_mex_assign(&c11_rhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs117), "rhs",
                  "rhs", 117);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs117), "lhs",
                  "lhs", 117);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"), "context",
                  "context", 118);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 118);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 118);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 118);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 118);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 118);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 118);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 118);
  sf_mex_assign(&c11_rhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs118), "rhs",
                  "rhs", 118);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs118), "lhs",
                  "lhs", 118);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"), "context",
                  "context", 119);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.xdotc"),
                  "name", "name", 119);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 119);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotc.p"),
                  "resolved", "resolved", 119);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 119);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 119);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 119);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 119);
  sf_mex_assign(&c11_rhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs119), "rhs",
                  "rhs", 119);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs119), "lhs",
                  "lhs", 119);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotc.p"),
                  "context", "context", 120);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.xdot"),
                  "name", "name", 120);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 120);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "resolved", "resolved", 120);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 120);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 120);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 120);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 120);
  sf_mex_assign(&c11_rhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs120), "rhs",
                  "rhs", 120);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs120), "lhs",
                  "lhs", 120);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 121);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 121);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 121);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 121);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 121);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 121);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 121);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 121);
  sf_mex_assign(&c11_rhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs121), "rhs",
                  "rhs", 121);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs121), "lhs",
                  "lhs", 121);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!below_threshold"),
                  "context", "context", 122);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 122);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 122);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 122);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 122);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 122);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 122);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 122);
  sf_mex_assign(&c11_rhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs122), "rhs",
                  "rhs", 122);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs122), "lhs",
                  "lhs", 122);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 123);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.refblas.xdot"),
                  "name", "name", 123);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 123);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "resolved", "resolved", 123);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 123);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 123);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 123);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 123);
  sf_mex_assign(&c11_rhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs123), "rhs",
                  "rhs", 123);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs123), "lhs",
                  "lhs", 123);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "context", "context", 124);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.refblas.xdotx"), "name", "name", 124);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 124);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "resolved", "resolved", 124);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 124);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 124);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 124);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 124);
  sf_mex_assign(&c11_rhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs124), "rhs",
                  "rhs", 124);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs124), "lhs",
                  "lhs", 124);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 125);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 125);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 125);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 125);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 125);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 125);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 125);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 125);
  sf_mex_assign(&c11_rhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs125), "rhs",
                  "rhs", 125);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs125), "lhs",
                  "lhs", 125);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 126);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 126);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 126);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 126);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 126);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 126);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 126);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 126);
  sf_mex_assign(&c11_rhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs126), "rhs",
                  "rhs", 126);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs126), "lhs",
                  "lhs", 126);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 127);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 127);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 127);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 127);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 127);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 127);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 127);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 127);
  sf_mex_assign(&c11_rhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs127), "rhs",
                  "rhs", 127);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs127), "lhs",
                  "lhs", 127);
  sf_mex_destroy(&c11_rhs64);
  sf_mex_destroy(&c11_lhs64);
  sf_mex_destroy(&c11_rhs65);
  sf_mex_destroy(&c11_lhs65);
  sf_mex_destroy(&c11_rhs66);
  sf_mex_destroy(&c11_lhs66);
  sf_mex_destroy(&c11_rhs67);
  sf_mex_destroy(&c11_lhs67);
  sf_mex_destroy(&c11_rhs68);
  sf_mex_destroy(&c11_lhs68);
  sf_mex_destroy(&c11_rhs69);
  sf_mex_destroy(&c11_lhs69);
  sf_mex_destroy(&c11_rhs70);
  sf_mex_destroy(&c11_lhs70);
  sf_mex_destroy(&c11_rhs71);
  sf_mex_destroy(&c11_lhs71);
  sf_mex_destroy(&c11_rhs72);
  sf_mex_destroy(&c11_lhs72);
  sf_mex_destroy(&c11_rhs73);
  sf_mex_destroy(&c11_lhs73);
  sf_mex_destroy(&c11_rhs74);
  sf_mex_destroy(&c11_lhs74);
  sf_mex_destroy(&c11_rhs75);
  sf_mex_destroy(&c11_lhs75);
  sf_mex_destroy(&c11_rhs76);
  sf_mex_destroy(&c11_lhs76);
  sf_mex_destroy(&c11_rhs77);
  sf_mex_destroy(&c11_lhs77);
  sf_mex_destroy(&c11_rhs78);
  sf_mex_destroy(&c11_lhs78);
  sf_mex_destroy(&c11_rhs79);
  sf_mex_destroy(&c11_lhs79);
  sf_mex_destroy(&c11_rhs80);
  sf_mex_destroy(&c11_lhs80);
  sf_mex_destroy(&c11_rhs81);
  sf_mex_destroy(&c11_lhs81);
  sf_mex_destroy(&c11_rhs82);
  sf_mex_destroy(&c11_lhs82);
  sf_mex_destroy(&c11_rhs83);
  sf_mex_destroy(&c11_lhs83);
  sf_mex_destroy(&c11_rhs84);
  sf_mex_destroy(&c11_lhs84);
  sf_mex_destroy(&c11_rhs85);
  sf_mex_destroy(&c11_lhs85);
  sf_mex_destroy(&c11_rhs86);
  sf_mex_destroy(&c11_lhs86);
  sf_mex_destroy(&c11_rhs87);
  sf_mex_destroy(&c11_lhs87);
  sf_mex_destroy(&c11_rhs88);
  sf_mex_destroy(&c11_lhs88);
  sf_mex_destroy(&c11_rhs89);
  sf_mex_destroy(&c11_lhs89);
  sf_mex_destroy(&c11_rhs90);
  sf_mex_destroy(&c11_lhs90);
  sf_mex_destroy(&c11_rhs91);
  sf_mex_destroy(&c11_lhs91);
  sf_mex_destroy(&c11_rhs92);
  sf_mex_destroy(&c11_lhs92);
  sf_mex_destroy(&c11_rhs93);
  sf_mex_destroy(&c11_lhs93);
  sf_mex_destroy(&c11_rhs94);
  sf_mex_destroy(&c11_lhs94);
  sf_mex_destroy(&c11_rhs95);
  sf_mex_destroy(&c11_lhs95);
  sf_mex_destroy(&c11_rhs96);
  sf_mex_destroy(&c11_lhs96);
  sf_mex_destroy(&c11_rhs97);
  sf_mex_destroy(&c11_lhs97);
  sf_mex_destroy(&c11_rhs98);
  sf_mex_destroy(&c11_lhs98);
  sf_mex_destroy(&c11_rhs99);
  sf_mex_destroy(&c11_lhs99);
  sf_mex_destroy(&c11_rhs100);
  sf_mex_destroy(&c11_lhs100);
  sf_mex_destroy(&c11_rhs101);
  sf_mex_destroy(&c11_lhs101);
  sf_mex_destroy(&c11_rhs102);
  sf_mex_destroy(&c11_lhs102);
  sf_mex_destroy(&c11_rhs103);
  sf_mex_destroy(&c11_lhs103);
  sf_mex_destroy(&c11_rhs104);
  sf_mex_destroy(&c11_lhs104);
  sf_mex_destroy(&c11_rhs105);
  sf_mex_destroy(&c11_lhs105);
  sf_mex_destroy(&c11_rhs106);
  sf_mex_destroy(&c11_lhs106);
  sf_mex_destroy(&c11_rhs107);
  sf_mex_destroy(&c11_lhs107);
  sf_mex_destroy(&c11_rhs108);
  sf_mex_destroy(&c11_lhs108);
  sf_mex_destroy(&c11_rhs109);
  sf_mex_destroy(&c11_lhs109);
  sf_mex_destroy(&c11_rhs110);
  sf_mex_destroy(&c11_lhs110);
  sf_mex_destroy(&c11_rhs111);
  sf_mex_destroy(&c11_lhs111);
  sf_mex_destroy(&c11_rhs112);
  sf_mex_destroy(&c11_lhs112);
  sf_mex_destroy(&c11_rhs113);
  sf_mex_destroy(&c11_lhs113);
  sf_mex_destroy(&c11_rhs114);
  sf_mex_destroy(&c11_lhs114);
  sf_mex_destroy(&c11_rhs115);
  sf_mex_destroy(&c11_lhs115);
  sf_mex_destroy(&c11_rhs116);
  sf_mex_destroy(&c11_lhs116);
  sf_mex_destroy(&c11_rhs117);
  sf_mex_destroy(&c11_lhs117);
  sf_mex_destroy(&c11_rhs118);
  sf_mex_destroy(&c11_lhs118);
  sf_mex_destroy(&c11_rhs119);
  sf_mex_destroy(&c11_lhs119);
  sf_mex_destroy(&c11_rhs120);
  sf_mex_destroy(&c11_lhs120);
  sf_mex_destroy(&c11_rhs121);
  sf_mex_destroy(&c11_lhs121);
  sf_mex_destroy(&c11_rhs122);
  sf_mex_destroy(&c11_lhs122);
  sf_mex_destroy(&c11_rhs123);
  sf_mex_destroy(&c11_lhs123);
  sf_mex_destroy(&c11_rhs124);
  sf_mex_destroy(&c11_lhs124);
  sf_mex_destroy(&c11_rhs125);
  sf_mex_destroy(&c11_lhs125);
  sf_mex_destroy(&c11_rhs126);
  sf_mex_destroy(&c11_lhs126);
  sf_mex_destroy(&c11_rhs127);
  sf_mex_destroy(&c11_lhs127);
}

static void c11_c_info_helper(const mxArray **c11_info)
{
  const mxArray *c11_rhs128 = NULL;
  const mxArray *c11_lhs128 = NULL;
  const mxArray *c11_rhs129 = NULL;
  const mxArray *c11_lhs129 = NULL;
  const mxArray *c11_rhs130 = NULL;
  const mxArray *c11_lhs130 = NULL;
  const mxArray *c11_rhs131 = NULL;
  const mxArray *c11_lhs131 = NULL;
  const mxArray *c11_rhs132 = NULL;
  const mxArray *c11_lhs132 = NULL;
  const mxArray *c11_rhs133 = NULL;
  const mxArray *c11_lhs133 = NULL;
  const mxArray *c11_rhs134 = NULL;
  const mxArray *c11_lhs134 = NULL;
  const mxArray *c11_rhs135 = NULL;
  const mxArray *c11_lhs135 = NULL;
  const mxArray *c11_rhs136 = NULL;
  const mxArray *c11_lhs136 = NULL;
  const mxArray *c11_rhs137 = NULL;
  const mxArray *c11_lhs137 = NULL;
  const mxArray *c11_rhs138 = NULL;
  const mxArray *c11_lhs138 = NULL;
  const mxArray *c11_rhs139 = NULL;
  const mxArray *c11_lhs139 = NULL;
  const mxArray *c11_rhs140 = NULL;
  const mxArray *c11_lhs140 = NULL;
  const mxArray *c11_rhs141 = NULL;
  const mxArray *c11_lhs141 = NULL;
  const mxArray *c11_rhs142 = NULL;
  const mxArray *c11_lhs142 = NULL;
  const mxArray *c11_rhs143 = NULL;
  const mxArray *c11_lhs143 = NULL;
  const mxArray *c11_rhs144 = NULL;
  const mxArray *c11_lhs144 = NULL;
  const mxArray *c11_rhs145 = NULL;
  const mxArray *c11_lhs145 = NULL;
  const mxArray *c11_rhs146 = NULL;
  const mxArray *c11_lhs146 = NULL;
  const mxArray *c11_rhs147 = NULL;
  const mxArray *c11_lhs147 = NULL;
  const mxArray *c11_rhs148 = NULL;
  const mxArray *c11_lhs148 = NULL;
  const mxArray *c11_rhs149 = NULL;
  const mxArray *c11_lhs149 = NULL;
  const mxArray *c11_rhs150 = NULL;
  const mxArray *c11_lhs150 = NULL;
  const mxArray *c11_rhs151 = NULL;
  const mxArray *c11_lhs151 = NULL;
  const mxArray *c11_rhs152 = NULL;
  const mxArray *c11_lhs152 = NULL;
  const mxArray *c11_rhs153 = NULL;
  const mxArray *c11_lhs153 = NULL;
  const mxArray *c11_rhs154 = NULL;
  const mxArray *c11_lhs154 = NULL;
  const mxArray *c11_rhs155 = NULL;
  const mxArray *c11_lhs155 = NULL;
  const mxArray *c11_rhs156 = NULL;
  const mxArray *c11_lhs156 = NULL;
  const mxArray *c11_rhs157 = NULL;
  const mxArray *c11_lhs157 = NULL;
  const mxArray *c11_rhs158 = NULL;
  const mxArray *c11_lhs158 = NULL;
  const mxArray *c11_rhs159 = NULL;
  const mxArray *c11_lhs159 = NULL;
  const mxArray *c11_rhs160 = NULL;
  const mxArray *c11_lhs160 = NULL;
  const mxArray *c11_rhs161 = NULL;
  const mxArray *c11_lhs161 = NULL;
  const mxArray *c11_rhs162 = NULL;
  const mxArray *c11_lhs162 = NULL;
  const mxArray *c11_rhs163 = NULL;
  const mxArray *c11_lhs163 = NULL;
  const mxArray *c11_rhs164 = NULL;
  const mxArray *c11_lhs164 = NULL;
  const mxArray *c11_rhs165 = NULL;
  const mxArray *c11_lhs165 = NULL;
  const mxArray *c11_rhs166 = NULL;
  const mxArray *c11_lhs166 = NULL;
  const mxArray *c11_rhs167 = NULL;
  const mxArray *c11_lhs167 = NULL;
  const mxArray *c11_rhs168 = NULL;
  const mxArray *c11_lhs168 = NULL;
  const mxArray *c11_rhs169 = NULL;
  const mxArray *c11_lhs169 = NULL;
  const mxArray *c11_rhs170 = NULL;
  const mxArray *c11_lhs170 = NULL;
  const mxArray *c11_rhs171 = NULL;
  const mxArray *c11_lhs171 = NULL;
  const mxArray *c11_rhs172 = NULL;
  const mxArray *c11_lhs172 = NULL;
  const mxArray *c11_rhs173 = NULL;
  const mxArray *c11_lhs173 = NULL;
  const mxArray *c11_rhs174 = NULL;
  const mxArray *c11_lhs174 = NULL;
  const mxArray *c11_rhs175 = NULL;
  const mxArray *c11_lhs175 = NULL;
  const mxArray *c11_rhs176 = NULL;
  const mxArray *c11_lhs176 = NULL;
  const mxArray *c11_rhs177 = NULL;
  const mxArray *c11_lhs177 = NULL;
  const mxArray *c11_rhs178 = NULL;
  const mxArray *c11_lhs178 = NULL;
  const mxArray *c11_rhs179 = NULL;
  const mxArray *c11_lhs179 = NULL;
  const mxArray *c11_rhs180 = NULL;
  const mxArray *c11_lhs180 = NULL;
  const mxArray *c11_rhs181 = NULL;
  const mxArray *c11_lhs181 = NULL;
  const mxArray *c11_rhs182 = NULL;
  const mxArray *c11_lhs182 = NULL;
  const mxArray *c11_rhs183 = NULL;
  const mxArray *c11_lhs183 = NULL;
  const mxArray *c11_rhs184 = NULL;
  const mxArray *c11_lhs184 = NULL;
  const mxArray *c11_rhs185 = NULL;
  const mxArray *c11_lhs185 = NULL;
  const mxArray *c11_rhs186 = NULL;
  const mxArray *c11_lhs186 = NULL;
  const mxArray *c11_rhs187 = NULL;
  const mxArray *c11_lhs187 = NULL;
  const mxArray *c11_rhs188 = NULL;
  const mxArray *c11_lhs188 = NULL;
  const mxArray *c11_rhs189 = NULL;
  const mxArray *c11_lhs189 = NULL;
  const mxArray *c11_rhs190 = NULL;
  const mxArray *c11_lhs190 = NULL;
  const mxArray *c11_rhs191 = NULL;
  const mxArray *c11_lhs191 = NULL;
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 128);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("mrdivide"), "name", "name",
                  128);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 128);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 128);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 128);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 128);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 128);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 128);
  sf_mex_assign(&c11_rhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs128), "rhs",
                  "rhs", 128);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs128), "lhs",
                  "lhs", 128);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 129);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_xaxpy"), "name", "name",
                  129);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 129);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"),
                  "resolved", "resolved", 129);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 129);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 129);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 129);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 129);
  sf_mex_assign(&c11_rhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs129), "rhs",
                  "rhs", 129);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs129), "lhs",
                  "lhs", 129);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"), "context",
                  "context", 130);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 130);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 130);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 130);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 130);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 130);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 130);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 130);
  sf_mex_assign(&c11_rhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs130), "rhs",
                  "rhs", 130);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs130), "lhs",
                  "lhs", 130);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"), "context",
                  "context", 131);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.xaxpy"),
                  "name", "name", 131);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 131);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "resolved", "resolved", 131);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 131);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 131);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 131);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 131);
  sf_mex_assign(&c11_rhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs131), "rhs",
                  "rhs", 131);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs131), "lhs",
                  "lhs", 131);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 132);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 132);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 132);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 132);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 132);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 132);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 132);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 132);
  sf_mex_assign(&c11_rhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs132), "rhs",
                  "rhs", 132);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs132), "lhs",
                  "lhs", 132);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p!below_threshold"),
                  "context", "context", 133);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 133);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 133);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 133);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 133);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 133);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 133);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 133);
  sf_mex_assign(&c11_rhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs133), "rhs",
                  "rhs", 133);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs133), "lhs",
                  "lhs", 133);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 134);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 134);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 134);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 134);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 134);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 134);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 134);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 134);
  sf_mex_assign(&c11_rhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs134), "rhs",
                  "rhs", 134);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs134), "lhs",
                  "lhs", 134);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 135);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.refblas.xaxpy"), "name", "name", 135);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 135);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "resolved", "resolved", 135);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 135);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 135);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 135);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 135);
  sf_mex_assign(&c11_rhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs135), "rhs",
                  "rhs", 135);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs135), "lhs",
                  "lhs", 135);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 136);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.isaUint"),
                  "name", "name", 136);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 136);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/isaUint.p"),
                  "resolved", "resolved", 136);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 136);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 136);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 136);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 136);
  sf_mex_assign(&c11_rhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs136), "rhs",
                  "rhs", 136);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs136), "lhs",
                  "lhs", 136);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 137);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 137);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 137);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 137);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 137);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 137);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 137);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 137);
  sf_mex_assign(&c11_rhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs137), "rhs",
                  "rhs", 137);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs137), "lhs",
                  "lhs", 137);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 138);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 138);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 138);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 138);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 138);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 138);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 138);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 138);
  sf_mex_assign(&c11_rhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs138), "rhs",
                  "rhs", 138);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs138), "lhs",
                  "lhs", 138);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 139);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 139);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 139);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 139);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 139);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 139);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 139);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 139);
  sf_mex_assign(&c11_rhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs139), "rhs",
                  "rhs", 139);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs139), "lhs",
                  "lhs", 139);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 140);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 140);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 140);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 140);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 140);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 140);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 140);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 140);
  sf_mex_assign(&c11_rhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs140), "rhs",
                  "rhs", 140);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs140), "lhs",
                  "lhs", 140);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 141);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 141);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 141);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 141);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 141);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 141);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 141);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 141);
  sf_mex_assign(&c11_rhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs141), "rhs",
                  "rhs", 141);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs141), "lhs",
                  "lhs", 141);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!below_threshold"),
                  "context", "context", 142);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("length"), "name", "name",
                  142);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 142);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 142);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1303167806U), "fileTimeLo",
                  "fileTimeLo", 142);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 142);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 142);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 142);
  sf_mex_assign(&c11_rhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs142), "rhs",
                  "rhs", 142);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs142), "lhs",
                  "lhs", 142);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength"),
                  "context", "context", 143);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 143);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 143);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 143);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 143);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 143);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 143);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 143);
  sf_mex_assign(&c11_rhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs143), "rhs",
                  "rhs", 143);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs143), "lhs",
                  "lhs", 143);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p!below_threshold"),
                  "context", "context", 144);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("length"), "name", "name",
                  144);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 144);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 144);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1303167806U), "fileTimeLo",
                  "fileTimeLo", 144);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 144);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 144);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 144);
  sf_mex_assign(&c11_rhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs144), "rhs",
                  "rhs", 144);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs144), "lhs",
                  "lhs", 144);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 145);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("abs"), "name", "name", 145);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 145);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 145);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 145);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 145);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 145);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 145);
  sf_mex_assign(&c11_rhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs145), "rhs",
                  "rhs", 145);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs145), "lhs",
                  "lhs", 145);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 146);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_xscal"), "name", "name",
                  146);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 146);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"),
                  "resolved", "resolved", 146);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 146);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 146);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 146);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 146);
  sf_mex_assign(&c11_rhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs146), "rhs",
                  "rhs", 146);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs146), "lhs",
                  "lhs", 146);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 147);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 147);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 147);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 147);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 147);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 147);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 147);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 147);
  sf_mex_assign(&c11_rhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs147), "rhs",
                  "rhs", 147);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs147), "lhs",
                  "lhs", 147);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 148);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.xscal"),
                  "name", "name", 148);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 148);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "resolved", "resolved", 148);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 148);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 148);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 148);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 148);
  sf_mex_assign(&c11_rhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs148), "rhs",
                  "rhs", 148);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs148), "lhs",
                  "lhs", 148);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 149);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("realmin"), "name", "name",
                  149);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 149);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 149);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1307672842U), "fileTimeLo",
                  "fileTimeLo", 149);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 149);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 149);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 149);
  sf_mex_assign(&c11_rhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs149), "rhs",
                  "rhs", 149);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs149), "lhs",
                  "lhs", 149);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 150);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eps"), "name", "name", 150);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 150);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 150);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 150);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 150);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 150);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 150);
  sf_mex_assign(&c11_rhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs150), "rhs",
                  "rhs", 150);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs150), "lhs",
                  "lhs", 150);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 151);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 151);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 151);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 151);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 151);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 151);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 151);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 151);
  sf_mex_assign(&c11_rhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs151), "rhs",
                  "rhs", 151);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs151), "lhs",
                  "lhs", 151);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 152);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_error"), "name", "name",
                  152);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 152);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 152);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 152);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 152);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 152);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 152);
  sf_mex_assign(&c11_rhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs152), "rhs",
                  "rhs", 152);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs152), "lhs",
                  "lhs", 152);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 153);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_const_nonsingleton_dim"),
                  "name", "name", 153);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 153);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "resolved", "resolved", 153);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 153);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 153);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 153);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 153);
  sf_mex_assign(&c11_rhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs153), "rhs",
                  "rhs", 153);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs153), "lhs",
                  "lhs", 153);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "context", "context", 154);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 154);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 154);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 154);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 154);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 154);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 154);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 154);
  sf_mex_assign(&c11_rhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs154), "rhs",
                  "rhs", 154);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs154), "lhs",
                  "lhs", 154);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 155);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 155);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 155);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 155);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 155);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 155);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 155);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 155);
  sf_mex_assign(&c11_rhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs155), "rhs",
                  "rhs", 155);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs155), "lhs",
                  "lhs", 155);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 156);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 156);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 156);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 156);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 156);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 156);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 156);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 156);
  sf_mex_assign(&c11_rhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs156), "rhs",
                  "rhs", 156);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs156), "lhs",
                  "lhs", 156);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 157);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 157);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 157);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 157);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 157);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 157);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 157);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 157);
  sf_mex_assign(&c11_rhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs157), "rhs",
                  "rhs", 157);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs157), "lhs",
                  "lhs", 157);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 158);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("isnan"), "name", "name", 158);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 158);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 158);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 158);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 158);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 158);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 158);
  sf_mex_assign(&c11_rhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs158), "rhs",
                  "rhs", 158);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs158), "lhs",
                  "lhs", 158);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 159);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 159);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 159);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 159);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 159);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 159);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 159);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 159);
  sf_mex_assign(&c11_rhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs159), "rhs",
                  "rhs", 159);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs159), "lhs",
                  "lhs", 159);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 160);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 160);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 160);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 160);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 160);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 160);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 160);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 160);
  sf_mex_assign(&c11_rhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs160), "rhs",
                  "rhs", 160);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs160), "lhs",
                  "lhs", 160);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 161);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 161);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 161);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 161);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 161);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 161);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 161);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 161);
  sf_mex_assign(&c11_rhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs161), "rhs",
                  "rhs", 161);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs161), "lhs",
                  "lhs", 161);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 162);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_relop"), "name", "name",
                  162);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 162);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 162);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1342472782U), "fileTimeLo",
                  "fileTimeLo", 162);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 162);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 162);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 162);
  sf_mex_assign(&c11_rhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs162), "rhs",
                  "rhs", 162);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs162), "lhs",
                  "lhs", 162);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 163);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("sqrt"), "name", "name", 163);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 163);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 163);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 163);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 163);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 163);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 163);
  sf_mex_assign(&c11_rhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs163), "rhs",
                  "rhs", 163);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs163), "lhs",
                  "lhs", 163);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 164);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_error"), "name", "name",
                  164);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 164);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 164);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 164);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 164);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 164);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 164);
  sf_mex_assign(&c11_rhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs164), "rhs",
                  "rhs", 164);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs164), "lhs",
                  "lhs", 164);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 165);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 165);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 165);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 165);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 165);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 165);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 165);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 165);
  sf_mex_assign(&c11_rhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs165), "rhs",
                  "rhs", 165);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs165), "lhs",
                  "lhs", 165);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 166);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_xrotg"), "name", "name",
                  166);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 166);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"),
                  "resolved", "resolved", 166);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 166);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 166);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 166);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 166);
  sf_mex_assign(&c11_rhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs166), "rhs",
                  "rhs", 166);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs166), "lhs",
                  "lhs", 166);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"), "context",
                  "context", 167);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 167);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 167);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 167);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 167);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 167);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 167);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 167);
  sf_mex_assign(&c11_rhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs167), "rhs",
                  "rhs", 167);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs167), "lhs",
                  "lhs", 167);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"), "context",
                  "context", 168);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.xrotg"),
                  "name", "name", 168);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 168);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "resolved", "resolved", 168);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 168);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 168);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 168);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 168);
  sf_mex_assign(&c11_rhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs168), "rhs",
                  "rhs", 168);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs168), "lhs",
                  "lhs", 168);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "context", "context", 169);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 169);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 169);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 169);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 169);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 169);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 169);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 169);
  sf_mex_assign(&c11_rhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs169), "rhs",
                  "rhs", 169);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs169), "lhs",
                  "lhs", 169);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "context", "context", 170);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.refblas.xrotg"), "name", "name", 170);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 170);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "resolved", "resolved", 170);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 170);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 170);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 170);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 170);
  sf_mex_assign(&c11_rhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs170), "rhs",
                  "rhs", 170);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs170), "lhs",
                  "lhs", 170);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 171);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("abs"), "name", "name", 171);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 171);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 171);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 171);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 171);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 171);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 171);
  sf_mex_assign(&c11_rhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs171), "rhs",
                  "rhs", 171);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs171), "lhs",
                  "lhs", 171);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 172);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("mrdivide"), "name", "name",
                  172);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 172);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 172);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 172);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 172);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 172);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 172);
  sf_mex_assign(&c11_rhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs172), "rhs",
                  "rhs", 172);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs172), "lhs",
                  "lhs", 172);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 173);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("sqrt"), "name", "name", 173);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 173);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 173);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 173);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 173);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 173);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 173);
  sf_mex_assign(&c11_rhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs173), "rhs",
                  "rhs", 173);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs173), "lhs",
                  "lhs", 173);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p!eml_ceval_xrotg"),
                  "context", "context", 174);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 174);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 174);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 174);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 174);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 174);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 174);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 174);
  sf_mex_assign(&c11_rhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs174), "rhs",
                  "rhs", 174);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs174), "lhs",
                  "lhs", 174);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 175);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_xrot"), "name", "name",
                  175);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 175);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "resolved",
                  "resolved", 175);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 175);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 175);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 175);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 175);
  sf_mex_assign(&c11_rhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs175), "rhs",
                  "rhs", 175);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs175), "lhs",
                  "lhs", 175);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "context",
                  "context", 176);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 176);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 176);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 176);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 176);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 176);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 176);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 176);
  sf_mex_assign(&c11_rhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs176), "rhs",
                  "rhs", 176);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs176), "lhs",
                  "lhs", 176);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "context",
                  "context", 177);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.xrot"),
                  "name", "name", 177);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 177);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "resolved", "resolved", 177);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 177);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 177);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 177);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 177);
  sf_mex_assign(&c11_rhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs177), "rhs",
                  "rhs", 177);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs177), "lhs",
                  "lhs", 177);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 178);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 178);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 178);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 178);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 178);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 178);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 178);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 178);
  sf_mex_assign(&c11_rhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs178), "rhs",
                  "rhs", 178);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs178), "lhs",
                  "lhs", 178);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p!below_threshold"),
                  "context", "context", 179);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 179);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 179);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 179);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 179);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 179);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 179);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 179);
  sf_mex_assign(&c11_rhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs179), "rhs",
                  "rhs", 179);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs179), "lhs",
                  "lhs", 179);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 180);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 180);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 180);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 180);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 180);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 180);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 180);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 180);
  sf_mex_assign(&c11_rhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs180), "rhs",
                  "rhs", 180);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs180), "lhs",
                  "lhs", 180);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 181);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.refblas.xrot"),
                  "name", "name", 181);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 181);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "resolved", "resolved", 181);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 181);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 181);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 181);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 181);
  sf_mex_assign(&c11_rhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs181), "rhs",
                  "rhs", 181);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs181), "lhs",
                  "lhs", 181);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "context", "context", 182);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 182);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 182);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 182);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 182);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 182);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 182);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 182);
  sf_mex_assign(&c11_rhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs182), "rhs",
                  "rhs", 182);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs182), "lhs",
                  "lhs", 182);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "context", "context", 183);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 183);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 183);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 183);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 183);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 183);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 183);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 183);
  sf_mex_assign(&c11_rhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs183), "rhs",
                  "rhs", 183);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs183), "lhs",
                  "lhs", 183);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 184);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_xswap"), "name", "name",
                  184);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 184);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"),
                  "resolved", "resolved", 184);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 184);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 184);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 184);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 184);
  sf_mex_assign(&c11_rhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs184), "rhs",
                  "rhs", 184);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs184), "lhs",
                  "lhs", 184);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 185);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 185);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 185);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 185);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 185);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 185);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 185);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 185);
  sf_mex_assign(&c11_rhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs185), "rhs",
                  "rhs", 185);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs185), "lhs",
                  "lhs", 185);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 186);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.blas.xswap"),
                  "name", "name", 186);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 186);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "resolved", "resolved", 186);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 186);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 186);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 186);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 186);
  sf_mex_assign(&c11_rhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs186), "rhs",
                  "rhs", 186);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs186), "lhs",
                  "lhs", 186);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 187);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 187);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 187);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 187);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 187);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 187);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 187);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 187);
  sf_mex_assign(&c11_rhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs187), "rhs",
                  "rhs", 187);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs187), "lhs",
                  "lhs", 187);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p!below_threshold"),
                  "context", "context", 188);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 188);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 188);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 188);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 188);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 188);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 188);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 188);
  sf_mex_assign(&c11_rhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs188), "rhs",
                  "rhs", 188);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs188), "lhs",
                  "lhs", 188);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 189);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.refblas.xswap"), "name", "name", 189);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 189);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "resolved", "resolved", 189);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 189);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 189);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 189);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 189);
  sf_mex_assign(&c11_rhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs189), "rhs",
                  "rhs", 189);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs189), "lhs",
                  "lhs", 189);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 190);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("abs"), "name", "name", 190);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 190);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 190);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 190);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 190);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 190);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 190);
  sf_mex_assign(&c11_rhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs190), "rhs",
                  "rhs", 190);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs190), "lhs",
                  "lhs", 190);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 191);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 191);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 191);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 191);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 191);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 191);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 191);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 191);
  sf_mex_assign(&c11_rhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs191), "rhs",
                  "rhs", 191);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs191), "lhs",
                  "lhs", 191);
  sf_mex_destroy(&c11_rhs128);
  sf_mex_destroy(&c11_lhs128);
  sf_mex_destroy(&c11_rhs129);
  sf_mex_destroy(&c11_lhs129);
  sf_mex_destroy(&c11_rhs130);
  sf_mex_destroy(&c11_lhs130);
  sf_mex_destroy(&c11_rhs131);
  sf_mex_destroy(&c11_lhs131);
  sf_mex_destroy(&c11_rhs132);
  sf_mex_destroy(&c11_lhs132);
  sf_mex_destroy(&c11_rhs133);
  sf_mex_destroy(&c11_lhs133);
  sf_mex_destroy(&c11_rhs134);
  sf_mex_destroy(&c11_lhs134);
  sf_mex_destroy(&c11_rhs135);
  sf_mex_destroy(&c11_lhs135);
  sf_mex_destroy(&c11_rhs136);
  sf_mex_destroy(&c11_lhs136);
  sf_mex_destroy(&c11_rhs137);
  sf_mex_destroy(&c11_lhs137);
  sf_mex_destroy(&c11_rhs138);
  sf_mex_destroy(&c11_lhs138);
  sf_mex_destroy(&c11_rhs139);
  sf_mex_destroy(&c11_lhs139);
  sf_mex_destroy(&c11_rhs140);
  sf_mex_destroy(&c11_lhs140);
  sf_mex_destroy(&c11_rhs141);
  sf_mex_destroy(&c11_lhs141);
  sf_mex_destroy(&c11_rhs142);
  sf_mex_destroy(&c11_lhs142);
  sf_mex_destroy(&c11_rhs143);
  sf_mex_destroy(&c11_lhs143);
  sf_mex_destroy(&c11_rhs144);
  sf_mex_destroy(&c11_lhs144);
  sf_mex_destroy(&c11_rhs145);
  sf_mex_destroy(&c11_lhs145);
  sf_mex_destroy(&c11_rhs146);
  sf_mex_destroy(&c11_lhs146);
  sf_mex_destroy(&c11_rhs147);
  sf_mex_destroy(&c11_lhs147);
  sf_mex_destroy(&c11_rhs148);
  sf_mex_destroy(&c11_lhs148);
  sf_mex_destroy(&c11_rhs149);
  sf_mex_destroy(&c11_lhs149);
  sf_mex_destroy(&c11_rhs150);
  sf_mex_destroy(&c11_lhs150);
  sf_mex_destroy(&c11_rhs151);
  sf_mex_destroy(&c11_lhs151);
  sf_mex_destroy(&c11_rhs152);
  sf_mex_destroy(&c11_lhs152);
  sf_mex_destroy(&c11_rhs153);
  sf_mex_destroy(&c11_lhs153);
  sf_mex_destroy(&c11_rhs154);
  sf_mex_destroy(&c11_lhs154);
  sf_mex_destroy(&c11_rhs155);
  sf_mex_destroy(&c11_lhs155);
  sf_mex_destroy(&c11_rhs156);
  sf_mex_destroy(&c11_lhs156);
  sf_mex_destroy(&c11_rhs157);
  sf_mex_destroy(&c11_lhs157);
  sf_mex_destroy(&c11_rhs158);
  sf_mex_destroy(&c11_lhs158);
  sf_mex_destroy(&c11_rhs159);
  sf_mex_destroy(&c11_lhs159);
  sf_mex_destroy(&c11_rhs160);
  sf_mex_destroy(&c11_lhs160);
  sf_mex_destroy(&c11_rhs161);
  sf_mex_destroy(&c11_lhs161);
  sf_mex_destroy(&c11_rhs162);
  sf_mex_destroy(&c11_lhs162);
  sf_mex_destroy(&c11_rhs163);
  sf_mex_destroy(&c11_lhs163);
  sf_mex_destroy(&c11_rhs164);
  sf_mex_destroy(&c11_lhs164);
  sf_mex_destroy(&c11_rhs165);
  sf_mex_destroy(&c11_lhs165);
  sf_mex_destroy(&c11_rhs166);
  sf_mex_destroy(&c11_lhs166);
  sf_mex_destroy(&c11_rhs167);
  sf_mex_destroy(&c11_lhs167);
  sf_mex_destroy(&c11_rhs168);
  sf_mex_destroy(&c11_lhs168);
  sf_mex_destroy(&c11_rhs169);
  sf_mex_destroy(&c11_lhs169);
  sf_mex_destroy(&c11_rhs170);
  sf_mex_destroy(&c11_lhs170);
  sf_mex_destroy(&c11_rhs171);
  sf_mex_destroy(&c11_lhs171);
  sf_mex_destroy(&c11_rhs172);
  sf_mex_destroy(&c11_lhs172);
  sf_mex_destroy(&c11_rhs173);
  sf_mex_destroy(&c11_lhs173);
  sf_mex_destroy(&c11_rhs174);
  sf_mex_destroy(&c11_lhs174);
  sf_mex_destroy(&c11_rhs175);
  sf_mex_destroy(&c11_lhs175);
  sf_mex_destroy(&c11_rhs176);
  sf_mex_destroy(&c11_lhs176);
  sf_mex_destroy(&c11_rhs177);
  sf_mex_destroy(&c11_lhs177);
  sf_mex_destroy(&c11_rhs178);
  sf_mex_destroy(&c11_lhs178);
  sf_mex_destroy(&c11_rhs179);
  sf_mex_destroy(&c11_lhs179);
  sf_mex_destroy(&c11_rhs180);
  sf_mex_destroy(&c11_lhs180);
  sf_mex_destroy(&c11_rhs181);
  sf_mex_destroy(&c11_lhs181);
  sf_mex_destroy(&c11_rhs182);
  sf_mex_destroy(&c11_lhs182);
  sf_mex_destroy(&c11_rhs183);
  sf_mex_destroy(&c11_lhs183);
  sf_mex_destroy(&c11_rhs184);
  sf_mex_destroy(&c11_lhs184);
  sf_mex_destroy(&c11_rhs185);
  sf_mex_destroy(&c11_lhs185);
  sf_mex_destroy(&c11_rhs186);
  sf_mex_destroy(&c11_lhs186);
  sf_mex_destroy(&c11_rhs187);
  sf_mex_destroy(&c11_lhs187);
  sf_mex_destroy(&c11_rhs188);
  sf_mex_destroy(&c11_lhs188);
  sf_mex_destroy(&c11_rhs189);
  sf_mex_destroy(&c11_lhs189);
  sf_mex_destroy(&c11_rhs190);
  sf_mex_destroy(&c11_lhs190);
  sf_mex_destroy(&c11_rhs191);
  sf_mex_destroy(&c11_lhs191);
}

static void c11_d_info_helper(const mxArray **c11_info)
{
  const mxArray *c11_rhs192 = NULL;
  const mxArray *c11_lhs192 = NULL;
  const mxArray *c11_rhs193 = NULL;
  const mxArray *c11_lhs193 = NULL;
  const mxArray *c11_rhs194 = NULL;
  const mxArray *c11_lhs194 = NULL;
  const mxArray *c11_rhs195 = NULL;
  const mxArray *c11_lhs195 = NULL;
  const mxArray *c11_rhs196 = NULL;
  const mxArray *c11_lhs196 = NULL;
  const mxArray *c11_rhs197 = NULL;
  const mxArray *c11_lhs197 = NULL;
  const mxArray *c11_rhs198 = NULL;
  const mxArray *c11_lhs198 = NULL;
  const mxArray *c11_rhs199 = NULL;
  const mxArray *c11_lhs199 = NULL;
  const mxArray *c11_rhs200 = NULL;
  const mxArray *c11_lhs200 = NULL;
  const mxArray *c11_rhs201 = NULL;
  const mxArray *c11_lhs201 = NULL;
  const mxArray *c11_rhs202 = NULL;
  const mxArray *c11_lhs202 = NULL;
  const mxArray *c11_rhs203 = NULL;
  const mxArray *c11_lhs203 = NULL;
  const mxArray *c11_rhs204 = NULL;
  const mxArray *c11_lhs204 = NULL;
  const mxArray *c11_rhs205 = NULL;
  const mxArray *c11_lhs205 = NULL;
  const mxArray *c11_rhs206 = NULL;
  const mxArray *c11_lhs206 = NULL;
  const mxArray *c11_rhs207 = NULL;
  const mxArray *c11_lhs207 = NULL;
  const mxArray *c11_rhs208 = NULL;
  const mxArray *c11_lhs208 = NULL;
  const mxArray *c11_rhs209 = NULL;
  const mxArray *c11_lhs209 = NULL;
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 192);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 192);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 192);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 192);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1286840312U), "fileTimeLo",
                  "fileTimeLo", 192);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 192);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 192);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 192);
  sf_mex_assign(&c11_rhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs192), "rhs",
                  "rhs", 192);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs192), "lhs",
                  "lhs", 192);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 193);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 193);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 193);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 193);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 193);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 193);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 193);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 193);
  sf_mex_assign(&c11_rhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs193), "rhs",
                  "rhs", 193);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs193), "lhs",
                  "lhs", 193);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 194);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 194);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 194);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 194);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 194);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 194);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 194);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 194);
  sf_mex_assign(&c11_rhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs194), "rhs",
                  "rhs", 194);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs194), "lhs",
                  "lhs", 194);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 195);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eps"), "name", "name", 195);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 195);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 195);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 195);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 195);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 195);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 195);
  sf_mex_assign(&c11_rhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs195), "rhs",
                  "rhs", 195);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs195), "lhs",
                  "lhs", 195);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 196);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 196);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 196);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 196);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 196);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 196);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 196);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 196);
  sf_mex_assign(&c11_rhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs196), "rhs",
                  "rhs", 196);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs196), "lhs",
                  "lhs", 196);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 197);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 197);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 197);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 197);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 197);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 197);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 197);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 197);
  sf_mex_assign(&c11_rhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs197), "rhs",
                  "rhs", 197);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs197), "lhs",
                  "lhs", 197);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 198);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 198);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 198);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 198);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 198);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 198);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 198);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 198);
  sf_mex_assign(&c11_rhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs198), "rhs",
                  "rhs", 198);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs198), "lhs",
                  "lhs", 198);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 199);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_div"), "name", "name",
                  199);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 199);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 199);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 199);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 199);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 199);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 199);
  sf_mex_assign(&c11_rhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs199), "rhs",
                  "rhs", 199);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs199), "lhs",
                  "lhs", 199);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 200);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_xscal"), "name", "name",
                  200);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 200);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"),
                  "resolved", "resolved", 200);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 200);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 200);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 200);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 200);
  sf_mex_assign(&c11_rhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs200), "rhs",
                  "rhs", 200);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs200), "lhs",
                  "lhs", 200);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 201);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 201);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 201);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 201);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 201);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 201);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 201);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 201);
  sf_mex_assign(&c11_rhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs201), "rhs",
                  "rhs", 201);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs201), "lhs",
                  "lhs", 201);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 202);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  202);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 202);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 202);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 202);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 202);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 202);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 202);
  sf_mex_assign(&c11_rhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs202), "rhs",
                  "rhs", 202);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs202), "lhs",
                  "lhs", 202);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 203);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("min"), "name", "name", 203);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 203);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 203);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1311276918U), "fileTimeLo",
                  "fileTimeLo", 203);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 203);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 203);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 203);
  sf_mex_assign(&c11_rhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs203), "rhs",
                  "rhs", 203);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs203), "lhs",
                  "lhs", 203);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 204);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 204);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 204);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 204);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 204);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 204);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 204);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 204);
  sf_mex_assign(&c11_rhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs204), "rhs",
                  "rhs", 204);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs204), "lhs",
                  "lhs", 204);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 205);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 205);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 205);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 205);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 205);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 205);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 205);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 205);
  sf_mex_assign(&c11_rhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs205), "rhs",
                  "rhs", 205);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs205), "lhs",
                  "lhs", 205);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 206);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 206);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 206);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 206);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 206);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 206);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 206);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 206);
  sf_mex_assign(&c11_rhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs206), "rhs",
                  "rhs", 206);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs206), "lhs",
                  "lhs", 206);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 207);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 207);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 207);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 207);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 207);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 207);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 207);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 207);
  sf_mex_assign(&c11_rhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs207), "rhs",
                  "rhs", 207);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs207), "lhs",
                  "lhs", 207);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 208);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 208);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 208);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 208);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 208);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 208);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 208);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 208);
  sf_mex_assign(&c11_rhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs208), "rhs",
                  "rhs", 208);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs208), "lhs",
                  "lhs", 208);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 209);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 209);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 209);
  sf_mex_addfield(*c11_info, c11_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 209);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 209);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 209);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 209);
  sf_mex_addfield(*c11_info, c11_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 209);
  sf_mex_assign(&c11_rhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c11_lhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_rhs209), "rhs",
                  "rhs", 209);
  sf_mex_addfield(*c11_info, sf_mex_duplicatearraysafe(&c11_lhs209), "lhs",
                  "lhs", 209);
  sf_mex_destroy(&c11_rhs192);
  sf_mex_destroy(&c11_lhs192);
  sf_mex_destroy(&c11_rhs193);
  sf_mex_destroy(&c11_lhs193);
  sf_mex_destroy(&c11_rhs194);
  sf_mex_destroy(&c11_lhs194);
  sf_mex_destroy(&c11_rhs195);
  sf_mex_destroy(&c11_lhs195);
  sf_mex_destroy(&c11_rhs196);
  sf_mex_destroy(&c11_lhs196);
  sf_mex_destroy(&c11_rhs197);
  sf_mex_destroy(&c11_lhs197);
  sf_mex_destroy(&c11_rhs198);
  sf_mex_destroy(&c11_lhs198);
  sf_mex_destroy(&c11_rhs199);
  sf_mex_destroy(&c11_lhs199);
  sf_mex_destroy(&c11_rhs200);
  sf_mex_destroy(&c11_lhs200);
  sf_mex_destroy(&c11_rhs201);
  sf_mex_destroy(&c11_lhs201);
  sf_mex_destroy(&c11_rhs202);
  sf_mex_destroy(&c11_lhs202);
  sf_mex_destroy(&c11_rhs203);
  sf_mex_destroy(&c11_lhs203);
  sf_mex_destroy(&c11_rhs204);
  sf_mex_destroy(&c11_lhs204);
  sf_mex_destroy(&c11_rhs205);
  sf_mex_destroy(&c11_lhs205);
  sf_mex_destroy(&c11_rhs206);
  sf_mex_destroy(&c11_lhs206);
  sf_mex_destroy(&c11_rhs207);
  sf_mex_destroy(&c11_lhs207);
  sf_mex_destroy(&c11_rhs208);
  sf_mex_destroy(&c11_lhs208);
  sf_mex_destroy(&c11_rhs209);
  sf_mex_destroy(&c11_lhs209);
}

static void c11_eye(SFc11_SS6_Estimation2InstanceStruct *chartInstance, real_T
                    c11_I[49])
{
  int32_T c11_i184;
  int32_T c11_k;
  int32_T c11_b_k;
  for (c11_i184 = 0; c11_i184 < 49; c11_i184++) {
    c11_I[c11_i184] = 0.0;
  }

  c11_eml_switch_helper(chartInstance);
  for (c11_k = 1; c11_k < 8; c11_k++) {
    c11_b_k = c11_k;
    c11_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c11_b_k), 1, 7, 1, 0) + 7 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_k), 1, 7, 2, 0) - 1))
      - 1] = 1.0;
  }
}

static void c11_eml_switch_helper(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static boolean_T c11_use_refblas(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
  return false;
}

static void c11_threshold(SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c11_b_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_c_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_d_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_e_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_pinv(SFc11_SS6_Estimation2InstanceStruct *chartInstance, real_T
                     c11_A[4], real_T c11_X[4])
{
  int32_T c11_i185;
  int32_T c11_k;
  int32_T c11_b_k;
  real_T c11_x;
  real_T c11_b_x;
  boolean_T c11_b;
  boolean_T c11_b0;
  real_T c11_c_x;
  boolean_T c11_b_b;
  boolean_T c11_b1;
  boolean_T c11_c_b;
  int32_T c11_i186;
  real_T c11_b_A[4];
  real_T c11_V[4];
  real_T c11_s[2];
  real_T c11_U[4];
  int32_T c11_i187;
  real_T c11_S[4];
  int32_T c11_c_k;
  real_T c11_d_k;
  real_T c11_tol;
  int32_T c11_r;
  int32_T c11_e_k;
  int32_T c11_f_k;
  int32_T c11_a;
  int32_T c11_b_a;
  int32_T c11_vcol;
  int32_T c11_b_r;
  int32_T c11_d_b;
  int32_T c11_e_b;
  boolean_T c11_overflow;
  int32_T c11_j;
  int32_T c11_b_j;
  real_T c11_y;
  real_T c11_b_y;
  real_T c11_z;
  int32_T c11_c_a;
  int32_T c11_d_a;
  int32_T c11_i188;
  real_T c11_b_V[4];
  int32_T c11_i189;
  real_T c11_b_U[4];
  boolean_T exitg1;
  for (c11_i185 = 0; c11_i185 < 4; c11_i185++) {
    c11_X[c11_i185] = 0.0;
  }

  c11_eml_switch_helper(chartInstance);
  for (c11_k = 1; c11_k < 5; c11_k++) {
    c11_b_k = c11_k;
    c11_x = c11_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_b_k), 1, 4, 1, 0) - 1];
    c11_b_x = c11_x;
    c11_b = muDoubleScalarIsInf(c11_b_x);
    c11_b0 = !c11_b;
    c11_c_x = c11_x;
    c11_b_b = muDoubleScalarIsNaN(c11_c_x);
    c11_b1 = !c11_b_b;
    c11_c_b = (c11_b0 && c11_b1);
    if (!c11_c_b) {
      c11_eml_error(chartInstance);
    }
  }

  for (c11_i186 = 0; c11_i186 < 4; c11_i186++) {
    c11_b_A[c11_i186] = c11_A[c11_i186];
  }

  c11_eml_xgesvd(chartInstance, c11_b_A, c11_U, c11_s, c11_V);
  for (c11_i187 = 0; c11_i187 < 4; c11_i187++) {
    c11_S[c11_i187] = 0.0;
  }

  for (c11_c_k = 0; c11_c_k < 2; c11_c_k++) {
    c11_d_k = 1.0 + (real_T)c11_c_k;
    c11_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             c11_d_k), 1, 2, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", c11_d_k), 1, 2, 2, 0) - 1) << 1)) - 1] =
      c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c11_d_k), 1, 2, 1, 0) - 1];
  }

  c11_eps(chartInstance);
  c11_tol = 2.0 * c11_S[0] * 2.2204460492503131E-16;
  c11_r = 0;
  c11_eml_switch_helper(chartInstance);
  c11_e_k = 1;
  exitg1 = false;
  while ((exitg1 == false) && (c11_e_k < 3)) {
    c11_f_k = c11_e_k;
    if (!(c11_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_f_k), 1, 2, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_f_k), 1, 2, 2, 0) - 1) <<
           1)) - 1] > c11_tol)) {
      exitg1 = true;
    } else {
      c11_a = c11_r;
      c11_b_a = c11_a + 1;
      c11_r = c11_b_a;
      c11_e_k++;
    }
  }

  if (c11_r > 0) {
    c11_vcol = 1;
    c11_b_r = c11_r;
    c11_d_b = c11_b_r;
    c11_e_b = c11_d_b;
    if (1 > c11_e_b) {
      c11_overflow = false;
    } else {
      c11_eml_switch_helper(chartInstance);
      c11_eml_switch_helper(chartInstance);
      c11_overflow = (c11_e_b > 2147483646);
    }

    if (c11_overflow) {
      c11_check_forloop_overflow_error(chartInstance, c11_overflow);
    }

    for (c11_j = 1; c11_j <= c11_b_r; c11_j++) {
      c11_b_j = c11_j;
      c11_y = c11_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", (real_T)c11_b_j), 1, 2, 1, 0) +
                     ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c11_b_j), 1, 2, 2, 0) - 1) << 1)) - 1];
      c11_b_y = c11_y;
      c11_z = 1.0 / c11_b_y;
      c11_b_eml_xscal(chartInstance, c11_z, c11_V, c11_vcol);
      c11_c_a = c11_vcol;
      c11_d_a = c11_c_a + 2;
      c11_vcol = c11_d_a;
    }

    for (c11_i188 = 0; c11_i188 < 4; c11_i188++) {
      c11_b_V[c11_i188] = c11_V[c11_i188];
    }

    for (c11_i189 = 0; c11_i189 < 4; c11_i189++) {
      c11_b_U[c11_i189] = c11_U[c11_i189];
    }

    c11_b_eml_xgemm(chartInstance, c11_r, c11_b_V, c11_b_U, c11_X);
  }
}

static void c11_f_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_eml_error(SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  int32_T c11_i190;
  static char_T c11_cv0[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 's', 'v', 'd', '_', 'm', 'a', 't', 'r', 'i', 'x', 'W', 'i',
    't', 'h', 'N', 'a', 'N', 'I', 'n', 'f' };

  char_T c11_u[33];
  const mxArray *c11_y = NULL;
  (void)chartInstance;
  for (c11_i190 = 0; c11_i190 < 33; c11_i190++) {
    c11_u[c11_i190] = c11_cv0[c11_i190];
  }

  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 10, 0U, 1U, 0U, 2, 1, 33),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c11_y));
}

static void c11_eml_xgesvd(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_A[4], real_T c11_U[4], real_T c11_S[2], real_T c11_V[4])
{
  int32_T c11_i191;
  real_T c11_b_A[4];
  int32_T c11_i192;
  int32_T c11_i193;
  real_T c11_Vf[4];
  boolean_T c11_apply_transform;
  int32_T c11_i194;
  real_T c11_c_A[4];
  real_T c11_nrm;
  real_T c11_absx;
  real_T c11_d;
  real_T c11_y;
  real_T c11_s[2];
  real_T c11_a;
  real_T c11_x;
  real_T c11_b_x;
  real_T c11_b_y;
  real_T c11_B;
  real_T c11_c_y;
  real_T c11_d_y;
  real_T c11_e_y;
  real_T c11_f_y;
  real_T c11_b_a;
  real_T c11_c_a;
  int32_T c11_k;
  int32_T c11_b_k;
  int32_T c11_c_k;
  int32_T c11_d_k;
  real_T c11_d_A;
  real_T c11_b_B;
  real_T c11_c_x;
  real_T c11_g_y;
  real_T c11_d_x;
  real_T c11_h_y;
  real_T c11_e_x;
  real_T c11_i_y;
  real_T c11_j_y;
  int32_T c11_i195;
  real_T c11_e_A[4];
  int32_T c11_i196;
  real_T c11_f_A[4];
  real_T c11_t;
  real_T c11_g_A;
  real_T c11_c_B;
  real_T c11_f_x;
  real_T c11_k_y;
  real_T c11_g_x;
  real_T c11_l_y;
  real_T c11_h_x;
  real_T c11_m_y;
  real_T c11_n_y;
  int32_T c11_ii;
  int32_T c11_b_ii;
  int32_T c11_m;
  real_T c11_e[2];
  int32_T c11_c_ii;
  int32_T c11_i197;
  real_T c11_b_U[4];
  int32_T c11_i198;
  real_T c11_c_U[4];
  real_T c11_h_A;
  real_T c11_d_B;
  real_T c11_i_x;
  real_T c11_o_y;
  real_T c11_j_x;
  real_T c11_p_y;
  real_T c11_k_x;
  real_T c11_q_y;
  real_T c11_r_y;
  int32_T c11_d_ii;
  int32_T c11_e_ii;
  int32_T c11_q;
  int32_T c11_b_q;
  int32_T c11_qp1;
  int32_T c11_pmq;
  int32_T c11_qp1q;
  int32_T c11_b_qp1;
  int32_T c11_d_a;
  int32_T c11_e_a;
  boolean_T c11_overflow;
  int32_T c11_jj;
  int32_T c11_b_jj;
  int32_T c11_qp1jj;
  int32_T c11_i199;
  real_T c11_b_Vf[4];
  int32_T c11_i200;
  real_T c11_c_Vf[4];
  real_T c11_i_A;
  real_T c11_e_B;
  real_T c11_l_x;
  real_T c11_s_y;
  real_T c11_m_x;
  real_T c11_t_y;
  real_T c11_n_x;
  real_T c11_u_y;
  real_T c11_v_y;
  int32_T c11_f_ii;
  int32_T c11_c_q;
  real_T c11_rt;
  real_T c11_j_A;
  real_T c11_f_B;
  real_T c11_o_x;
  real_T c11_w_y;
  real_T c11_p_x;
  real_T c11_x_y;
  real_T c11_q_x;
  real_T c11_y_y;
  real_T c11_r;
  real_T c11_k_A;
  real_T c11_g_B;
  real_T c11_r_x;
  real_T c11_ab_y;
  real_T c11_s_x;
  real_T c11_bb_y;
  real_T c11_t_x;
  real_T c11_cb_y;
  real_T c11_db_y;
  int32_T c11_colq;
  real_T c11_l_A;
  real_T c11_h_B;
  real_T c11_u_x;
  real_T c11_eb_y;
  real_T c11_v_x;
  real_T c11_fb_y;
  real_T c11_w_x;
  real_T c11_gb_y;
  int32_T c11_colqp1;
  real_T c11_iter;
  real_T c11_snorm;
  int32_T c11_g_ii;
  real_T c11_varargin_1;
  real_T c11_varargin_2;
  real_T c11_b_varargin_2;
  real_T c11_varargin_3;
  real_T c11_x_x;
  real_T c11_hb_y;
  real_T c11_y_x;
  real_T c11_ib_y;
  real_T c11_xk;
  real_T c11_yk;
  real_T c11_ab_x;
  real_T c11_jb_y;
  real_T c11_maxval;
  real_T c11_b_varargin_1;
  real_T c11_c_varargin_2;
  real_T c11_d_varargin_2;
  real_T c11_b_varargin_3;
  real_T c11_bb_x;
  real_T c11_kb_y;
  real_T c11_cb_x;
  real_T c11_lb_y;
  real_T c11_b_xk;
  real_T c11_b_yk;
  real_T c11_db_x;
  real_T c11_mb_y;
  int32_T c11_i201;
  int32_T c11_f_a;
  int32_T c11_g_a;
  boolean_T c11_b_overflow;
  int32_T c11_h_ii;
  real_T c11_test0;
  real_T c11_ztest0;
  real_T c11_kase;
  int32_T c11_qs;
  int32_T c11_b_m;
  int32_T c11_d_q;
  int32_T c11_h_a;
  int32_T c11_b;
  int32_T c11_i_a;
  int32_T c11_b_b;
  boolean_T c11_c_overflow;
  int32_T c11_i_ii;
  real_T c11_test;
  real_T c11_ztest;
  real_T c11_f;
  int32_T c11_i202;
  int32_T c11_e_q;
  int32_T c11_j_a;
  int32_T c11_c_b;
  int32_T c11_k_a;
  int32_T c11_d_b;
  boolean_T c11_d_overflow;
  int32_T c11_e_k;
  int32_T c11_f_k;
  real_T c11_t1;
  real_T c11_b_t1;
  real_T c11_b_f;
  real_T c11_sn;
  real_T c11_cs;
  real_T c11_b_cs;
  real_T c11_b_sn;
  int32_T c11_km1;
  int32_T c11_colk;
  int32_T c11_colm;
  int32_T c11_qm1;
  int32_T c11_f_q;
  int32_T c11_c_m;
  int32_T c11_l_a;
  int32_T c11_e_b;
  int32_T c11_m_a;
  int32_T c11_f_b;
  boolean_T c11_e_overflow;
  int32_T c11_g_k;
  real_T c11_c_t1;
  real_T c11_unusedU0;
  real_T c11_c_sn;
  real_T c11_c_cs;
  int32_T c11_colqm1;
  int32_T c11_mm1;
  real_T c11_d1;
  real_T c11_d2;
  real_T c11_d3;
  real_T c11_d4;
  real_T c11_d5;
  real_T c11_c_varargin_1[5];
  int32_T c11_ixstart;
  real_T c11_mtmp;
  real_T c11_eb_x;
  boolean_T c11_g_b;
  int32_T c11_ix;
  int32_T c11_b_ix;
  real_T c11_fb_x;
  boolean_T c11_h_b;
  int32_T c11_n_a;
  int32_T c11_o_a;
  int32_T c11_i203;
  int32_T c11_p_a;
  int32_T c11_q_a;
  boolean_T c11_f_overflow;
  int32_T c11_c_ix;
  real_T c11_r_a;
  real_T c11_i_b;
  boolean_T c11_p;
  real_T c11_b_mtmp;
  real_T c11_scale;
  real_T c11_m_A;
  real_T c11_i_B;
  real_T c11_gb_x;
  real_T c11_nb_y;
  real_T c11_hb_x;
  real_T c11_ob_y;
  real_T c11_ib_x;
  real_T c11_pb_y;
  real_T c11_sm;
  real_T c11_n_A;
  real_T c11_j_B;
  real_T c11_jb_x;
  real_T c11_qb_y;
  real_T c11_kb_x;
  real_T c11_rb_y;
  real_T c11_lb_x;
  real_T c11_sb_y;
  real_T c11_smm1;
  real_T c11_o_A;
  real_T c11_k_B;
  real_T c11_mb_x;
  real_T c11_tb_y;
  real_T c11_nb_x;
  real_T c11_ub_y;
  real_T c11_ob_x;
  real_T c11_vb_y;
  real_T c11_emm1;
  real_T c11_p_A;
  real_T c11_l_B;
  real_T c11_pb_x;
  real_T c11_wb_y;
  real_T c11_qb_x;
  real_T c11_xb_y;
  real_T c11_rb_x;
  real_T c11_yb_y;
  real_T c11_sqds;
  real_T c11_q_A;
  real_T c11_m_B;
  real_T c11_sb_x;
  real_T c11_ac_y;
  real_T c11_tb_x;
  real_T c11_bc_y;
  real_T c11_ub_x;
  real_T c11_cc_y;
  real_T c11_eqds;
  real_T c11_r_A;
  real_T c11_vb_x;
  real_T c11_wb_x;
  real_T c11_xb_x;
  real_T c11_j_b;
  real_T c11_c;
  real_T c11_shift;
  real_T c11_s_A;
  real_T c11_n_B;
  real_T c11_yb_x;
  real_T c11_dc_y;
  real_T c11_ac_x;
  real_T c11_ec_y;
  real_T c11_bc_x;
  real_T c11_fc_y;
  real_T c11_g;
  int32_T c11_g_q;
  int32_T c11_b_mm1;
  int32_T c11_s_a;
  int32_T c11_k_b;
  int32_T c11_t_a;
  int32_T c11_l_b;
  boolean_T c11_g_overflow;
  int32_T c11_h_k;
  int32_T c11_kp1;
  real_T c11_c_f;
  real_T c11_unusedU1;
  real_T c11_d_sn;
  real_T c11_d_cs;
  int32_T c11_colkp1;
  real_T c11_d_f;
  real_T c11_unusedU2;
  real_T c11_e_sn;
  real_T c11_e_cs;
  int32_T c11_i_k;
  int32_T c11_j;
  int32_T c11_b_j;
  int32_T c11_i;
  int32_T c11_b_i;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T exitg5;
  boolean_T guard11 = false;
  for (c11_i191 = 0; c11_i191 < 4; c11_i191++) {
    c11_b_A[c11_i191] = c11_A[c11_i191];
  }

  c11_f_eml_scalar_eg(chartInstance);
  for (c11_i192 = 0; c11_i192 < 4; c11_i192++) {
    c11_U[c11_i192] = 0.0;
  }

  for (c11_i193 = 0; c11_i193 < 4; c11_i193++) {
    c11_Vf[c11_i193] = 0.0;
  }

  c11_apply_transform = false;
  for (c11_i194 = 0; c11_i194 < 4; c11_i194++) {
    c11_c_A[c11_i194] = c11_b_A[c11_i194];
  }

  c11_nrm = c11_eml_xnrm2(chartInstance, c11_c_A);
  if (c11_nrm > 0.0) {
    c11_apply_transform = true;
    c11_absx = c11_nrm;
    c11_d = c11_b_A[0];
    if (c11_d < 0.0) {
      c11_y = -c11_absx;
    } else {
      c11_y = c11_absx;
    }

    c11_s[0] = c11_y;
    c11_a = c11_s[0];
    c11_realmin(chartInstance);
    c11_eps(chartInstance);
    c11_x = c11_a;
    c11_b_x = c11_x;
    c11_b_y = muDoubleScalarAbs(c11_b_x);
    if (c11_b_y >= 1.0020841800044864E-292) {
      c11_B = c11_a;
      c11_c_y = c11_B;
      c11_d_y = c11_c_y;
      c11_e_y = c11_d_y;
      c11_f_y = 1.0 / c11_e_y;
      c11_b_a = c11_f_y;
      c11_b_below_threshold(chartInstance);
      c11_c_a = c11_b_a;
      c11_eml_switch_helper(chartInstance);
      for (c11_k = 1; c11_k < 3; c11_k++) {
        c11_b_k = c11_k;
        c11_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c11_b_k), 1, 4, 1, 0) - 1] = c11_c_a *
          c11_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c11_b_k), 1, 4, 1, 0) - 1];
      }
    } else {
      c11_eml_switch_helper(chartInstance);
      for (c11_c_k = 1; c11_c_k < 3; c11_c_k++) {
        c11_d_k = c11_c_k;
        c11_d_A = c11_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c11_d_k), 1, 4, 1, 0) - 1];
        c11_b_B = c11_a;
        c11_c_x = c11_d_A;
        c11_g_y = c11_b_B;
        c11_d_x = c11_c_x;
        c11_h_y = c11_g_y;
        c11_e_x = c11_d_x;
        c11_i_y = c11_h_y;
        c11_j_y = c11_e_x / c11_i_y;
        c11_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c11_d_k), 1, 4, 1, 0) - 1] = c11_j_y;
      }
    }

    c11_b_A[0]++;
    c11_s[0] = -c11_s[0];
  } else {
    c11_s[0] = 0.0;
  }

  if (c11_apply_transform) {
    for (c11_i195 = 0; c11_i195 < 4; c11_i195++) {
      c11_e_A[c11_i195] = c11_b_A[c11_i195];
    }

    for (c11_i196 = 0; c11_i196 < 4; c11_i196++) {
      c11_f_A[c11_i196] = c11_b_A[c11_i196];
    }

    c11_t = c11_eml_xdotc(chartInstance, c11_e_A, c11_f_A);
    c11_g_A = c11_t;
    c11_c_B = c11_b_A[0];
    c11_f_x = c11_g_A;
    c11_k_y = c11_c_B;
    c11_g_x = c11_f_x;
    c11_l_y = c11_k_y;
    c11_h_x = c11_g_x;
    c11_m_y = c11_l_y;
    c11_n_y = c11_h_x / c11_m_y;
    c11_t = -c11_n_y;
    c11_c_eml_xaxpy(chartInstance, c11_t, c11_b_A);
  }

  c11_eml_switch_helper(chartInstance);
  for (c11_ii = 1; c11_ii < 3; c11_ii++) {
    c11_b_ii = c11_ii;
    c11_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_b_ii), 1, 2, 1, 0) - 1] = c11_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_ii), 1, 2, 1, 0) - 1];
  }

  c11_m = 1;
  c11_s[1] = c11_b_A[3];
  c11_e[0] = c11_b_A[2];
  c11_e[1] = 0.0;
  c11_eml_switch_helper(chartInstance);
  for (c11_c_ii = 1; c11_c_ii < 3; c11_c_ii++) {
    c11_b_ii = c11_c_ii;
    c11_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_b_ii), 1, 2, 1, 0) + 1] = 0.0;
  }

  c11_U[3] = 1.0;
  if (c11_s[0] != 0.0) {
    for (c11_i197 = 0; c11_i197 < 4; c11_i197++) {
      c11_b_U[c11_i197] = c11_U[c11_i197];
    }

    for (c11_i198 = 0; c11_i198 < 4; c11_i198++) {
      c11_c_U[c11_i198] = c11_U[c11_i198];
    }

    c11_t = c11_eml_xdotc(chartInstance, c11_b_U, c11_c_U);
    c11_h_A = c11_t;
    c11_d_B = c11_U[0];
    c11_i_x = c11_h_A;
    c11_o_y = c11_d_B;
    c11_j_x = c11_i_x;
    c11_p_y = c11_o_y;
    c11_k_x = c11_j_x;
    c11_q_y = c11_p_y;
    c11_r_y = c11_k_x / c11_q_y;
    c11_t = -c11_r_y;
    c11_c_eml_xaxpy(chartInstance, c11_t, c11_U);
    c11_eml_switch_helper(chartInstance);
    for (c11_d_ii = 1; c11_d_ii < 3; c11_d_ii++) {
      c11_b_ii = c11_d_ii;
      c11_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c11_b_ii), 1, 2, 1, 0) - 1] = -c11_U[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_ii), 1, 2, 1, 0) - 1];
    }

    c11_U[0]++;
  } else {
    c11_eml_switch_helper(chartInstance);
    for (c11_e_ii = 1; c11_e_ii < 3; c11_e_ii++) {
      c11_b_ii = c11_e_ii;
      c11_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c11_b_ii), 1, 2, 1, 0) - 1] = 0.0;
    }

    c11_U[0] = 1.0;
  }

  c11_eml_switch_helper(chartInstance);
  for (c11_q = 2; c11_q > 0; c11_q--) {
    c11_b_q = c11_q;
    if (c11_b_q <= 0) {
      if (c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_b_q), 1, 2, 1, 0) - 1] != 0.0) {
        c11_qp1 = c11_b_q + 1;
        c11_pmq = 2 - c11_b_q;
        c11_qp1q = c11_qp1 + ((c11_b_q - 1) << 1);
        c11_b_qp1 = c11_qp1;
        c11_d_a = c11_b_qp1;
        c11_e_a = c11_d_a;
        if (c11_e_a > 2) {
          c11_overflow = false;
        } else {
          c11_eml_switch_helper(chartInstance);
          c11_eml_switch_helper(chartInstance);
          c11_overflow = false;
        }

        if (c11_overflow) {
          c11_check_forloop_overflow_error(chartInstance, c11_overflow);
        }

        for (c11_jj = c11_b_qp1; c11_jj < 3; c11_jj++) {
          c11_b_jj = c11_jj - 1;
          c11_qp1jj = c11_qp1 + (c11_b_jj << 1);
          for (c11_i199 = 0; c11_i199 < 4; c11_i199++) {
            c11_b_Vf[c11_i199] = c11_Vf[c11_i199];
          }

          for (c11_i200 = 0; c11_i200 < 4; c11_i200++) {
            c11_c_Vf[c11_i200] = c11_Vf[c11_i200];
          }

          c11_t = c11_b_eml_xdotc(chartInstance, c11_pmq, c11_b_Vf, c11_qp1q,
            c11_c_Vf, c11_qp1jj);
          c11_i_A = c11_t;
          c11_e_B = c11_Vf[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c11_qp1q), 1, 4, 1, 0) - 1];
          c11_l_x = c11_i_A;
          c11_s_y = c11_e_B;
          c11_m_x = c11_l_x;
          c11_t_y = c11_s_y;
          c11_n_x = c11_m_x;
          c11_u_y = c11_t_y;
          c11_v_y = c11_n_x / c11_u_y;
          c11_t = -c11_v_y;
          c11_d_eml_xaxpy(chartInstance, c11_pmq, c11_t, c11_qp1q, c11_Vf,
                          c11_qp1jj);
        }
      }
    }

    c11_eml_switch_helper(chartInstance);
    for (c11_f_ii = 1; c11_f_ii < 3; c11_f_ii++) {
      c11_b_ii = c11_f_ii;
      c11_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c11_b_ii), 1, 2, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
                 "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 2,
                 0) - 1) << 1)) - 1] = 0.0;
    }

    c11_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c11_b_q), 1, 2, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 2, 0) - 1)
             << 1)) - 1] = 1.0;
  }

  c11_eml_switch_helper(chartInstance);
  for (c11_c_q = 1; c11_c_q < 3; c11_c_q++) {
    c11_b_q = c11_c_q;
    if (c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c11_b_q), 1, 2, 1, 0) - 1] != 0.0) {
      c11_rt = c11_abs(chartInstance, c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 1, 0) - 1]);
      c11_j_A = c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 1, 0) - 1];
      c11_f_B = c11_rt;
      c11_o_x = c11_j_A;
      c11_w_y = c11_f_B;
      c11_p_x = c11_o_x;
      c11_x_y = c11_w_y;
      c11_q_x = c11_p_x;
      c11_y_y = c11_x_y;
      c11_r = c11_q_x / c11_y_y;
      c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c11_b_q), 1, 2, 1, 0) - 1] = c11_rt;
      if (c11_b_q < 2) {
        c11_k_A = c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 1, 0) - 1];
        c11_g_B = c11_r;
        c11_r_x = c11_k_A;
        c11_ab_y = c11_g_B;
        c11_s_x = c11_r_x;
        c11_bb_y = c11_ab_y;
        c11_t_x = c11_s_x;
        c11_cb_y = c11_bb_y;
        c11_db_y = c11_t_x / c11_cb_y;
        c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c11_b_q), 1, 2, 1, 0) - 1] = c11_db_y;
      }

      if (c11_b_q <= 2) {
        c11_colq = (c11_b_q - 1) << 1;
        c11_b_eml_xscal(chartInstance, c11_r, c11_U, c11_colq + 1);
      }
    }

    if (c11_b_q < 2) {
      if (c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_b_q), 1, 2, 1, 0) - 1] != 0.0) {
        c11_rt = c11_abs(chartInstance, c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 1, 0) - 1]);
        c11_l_A = c11_rt;
        c11_h_B = c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 1, 0) - 1];
        c11_u_x = c11_l_A;
        c11_eb_y = c11_h_B;
        c11_v_x = c11_u_x;
        c11_fb_y = c11_eb_y;
        c11_w_x = c11_v_x;
        c11_gb_y = c11_fb_y;
        c11_r = c11_w_x / c11_gb_y;
        c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c11_b_q), 1, 2, 1, 0) - 1] = c11_rt;
        c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c11_b_q + 1)), 1, 2, 1, 0) - 1] =
          c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c11_b_q + 1)), 1, 2, 1, 0) - 1] * c11_r;
        c11_colqp1 = c11_b_q << 1;
        c11_b_eml_xscal(chartInstance, c11_r, c11_Vf, c11_colqp1 + 1);
      }
    }
  }

  c11_iter = 0.0;
  c11_realmin(chartInstance);
  c11_eps(chartInstance);
  c11_snorm = 0.0;
  c11_eml_switch_helper(chartInstance);
  for (c11_g_ii = 1; c11_g_ii < 3; c11_g_ii++) {
    c11_b_ii = c11_g_ii;
    c11_varargin_1 = c11_abs(chartInstance, c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_ii),
      1, 2, 1, 0) - 1]);
    c11_varargin_2 = c11_abs(chartInstance, c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_ii),
      1, 2, 1, 0) - 1]);
    c11_b_varargin_2 = c11_varargin_1;
    c11_varargin_3 = c11_varargin_2;
    c11_x_x = c11_b_varargin_2;
    c11_hb_y = c11_varargin_3;
    c11_y_x = c11_x_x;
    c11_ib_y = c11_hb_y;
    c11_g_eml_scalar_eg(chartInstance);
    c11_xk = c11_y_x;
    c11_yk = c11_ib_y;
    c11_ab_x = c11_xk;
    c11_jb_y = c11_yk;
    c11_g_eml_scalar_eg(chartInstance);
    c11_maxval = muDoubleScalarMax(c11_ab_x, c11_jb_y);
    c11_b_varargin_1 = c11_snorm;
    c11_c_varargin_2 = c11_maxval;
    c11_d_varargin_2 = c11_b_varargin_1;
    c11_b_varargin_3 = c11_c_varargin_2;
    c11_bb_x = c11_d_varargin_2;
    c11_kb_y = c11_b_varargin_3;
    c11_cb_x = c11_bb_x;
    c11_lb_y = c11_kb_y;
    c11_g_eml_scalar_eg(chartInstance);
    c11_b_xk = c11_cb_x;
    c11_b_yk = c11_lb_y;
    c11_db_x = c11_b_xk;
    c11_mb_y = c11_b_yk;
    c11_g_eml_scalar_eg(chartInstance);
    c11_snorm = muDoubleScalarMax(c11_db_x, c11_mb_y);
  }

  exitg1 = false;
  while ((exitg1 == false) && (c11_m + 1 > 0)) {
    if (c11_iter >= 75.0) {
      c11_b_eml_error(chartInstance);
      exitg1 = true;
    } else {
      c11_b_q = c11_m;
      c11_i201 = c11_m;
      c11_f_a = c11_i201;
      c11_g_a = c11_f_a;
      if (c11_g_a < 0) {
        c11_b_overflow = false;
      } else {
        c11_eml_switch_helper(chartInstance);
        c11_eml_switch_helper(chartInstance);
        c11_b_overflow = false;
      }

      if (c11_b_overflow) {
        c11_check_forloop_overflow_error(chartInstance, c11_b_overflow);
      }

      c11_h_ii = c11_i201;
      guard3 = false;
      guard4 = false;
      exitg5 = false;
      while ((exitg5 == false) && (c11_h_ii > -1)) {
        c11_b_ii = c11_h_ii;
        c11_b_q = c11_b_ii;
        if (c11_b_ii == 0) {
          exitg5 = true;
        } else {
          c11_test0 = c11_abs(chartInstance, c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
            "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_ii), 1, 2, 1, 0) -
                              1]) + c11_abs(chartInstance,
            c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c11_b_ii + 1)), 1, 2, 1, 0) - 1]);
          c11_ztest0 = c11_abs(chartInstance, c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
            "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_ii), 1, 2, 1, 0) -
                               1]);
          c11_eps(chartInstance);
          if (c11_ztest0 <= 2.2204460492503131E-16 * c11_test0) {
            guard4 = true;
            exitg5 = true;
          } else if (c11_ztest0 <= 1.0020841800044864E-292) {
            guard4 = true;
            exitg5 = true;
          } else {
            guard11 = false;
            if (c11_iter > 20.0) {
              c11_eps(chartInstance);
              if (c11_ztest0 <= 2.2204460492503131E-16 * c11_snorm) {
                guard3 = true;
                exitg5 = true;
              } else {
                guard11 = true;
              }
            } else {
              guard11 = true;
            }

            if (guard11 == true) {
              c11_h_ii--;
              guard3 = false;
              guard4 = false;
            }
          }
        }
      }

      if (guard4 == true) {
        guard3 = true;
      }

      if (guard3 == true) {
        c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c11_b_ii), 1, 2, 1, 0) - 1] = 0.0;
      }

      if (c11_b_q == c11_m) {
        c11_kase = 4.0;
      } else {
        c11_qs = c11_m + 1;
        c11_b_m = c11_m + 1;
        c11_d_q = c11_b_q;
        c11_h_a = c11_b_m;
        c11_b = c11_d_q;
        c11_i_a = c11_h_a;
        c11_b_b = c11_b;
        if (c11_i_a < c11_b_b) {
          c11_c_overflow = false;
        } else {
          c11_eml_switch_helper(chartInstance);
          c11_eml_switch_helper(chartInstance);
          c11_c_overflow = (c11_b_b < -2147483647);
        }

        if (c11_c_overflow) {
          c11_check_forloop_overflow_error(chartInstance, c11_c_overflow);
        }

        c11_i_ii = c11_b_m;
        guard2 = false;
        exitg4 = false;
        while ((exitg4 == false) && (c11_i_ii >= c11_d_q)) {
          c11_b_ii = c11_i_ii;
          c11_qs = c11_b_ii;
          if (c11_b_ii == c11_b_q) {
            exitg4 = true;
          } else {
            c11_test = 0.0;
            if (c11_b_ii < c11_m + 1) {
              c11_test = c11_abs(chartInstance,
                                 c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c11_b_ii), 1, 2, 1, 0) - 1]);
            }

            if (c11_b_ii > c11_b_q + 1) {
              c11_test += c11_abs(chartInstance,
                                  c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)(c11_b_ii - 1)), 1, 2, 1, 0) - 1]);
            }

            c11_ztest = c11_abs(chartInstance, c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK
                                ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
              c11_b_ii), 1, 2, 1, 0) - 1]);
            c11_eps(chartInstance);
            if (c11_ztest <= 2.2204460492503131E-16 * c11_test) {
              guard2 = true;
              exitg4 = true;
            } else if (c11_ztest <= 1.0020841800044864E-292) {
              guard2 = true;
              exitg4 = true;
            } else {
              c11_i_ii--;
              guard2 = false;
            }
          }
        }

        if (guard2 == true) {
          c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_b_ii), 1, 2, 1, 0) - 1] = 0.0;
        }

        if (c11_qs == c11_b_q) {
          c11_kase = 3.0;
        } else if (c11_qs == c11_m + 1) {
          c11_kase = 1.0;
        } else {
          c11_kase = 2.0;
          c11_b_q = c11_qs;
        }
      }

      c11_b_q++;
      switch ((int32_T)_SFD_INTEGER_CHECK("", c11_kase)) {
       case 1:
        c11_f = c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c11_m), 1, 2, 1, 0) - 1];
        c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c11_m), 1, 2, 1, 0) - 1] = 0.0;
        c11_i202 = c11_m;
        c11_e_q = c11_b_q;
        c11_j_a = c11_i202;
        c11_c_b = c11_e_q;
        c11_k_a = c11_j_a;
        c11_d_b = c11_c_b;
        if (c11_k_a < c11_d_b) {
          c11_d_overflow = false;
        } else {
          c11_eml_switch_helper(chartInstance);
          c11_eml_switch_helper(chartInstance);
          c11_d_overflow = (c11_d_b < -2147483647);
        }

        if (c11_d_overflow) {
          c11_check_forloop_overflow_error(chartInstance, c11_d_overflow);
        }

        for (c11_e_k = c11_i202; c11_e_k >= c11_e_q; c11_e_k--) {
          c11_f_k = c11_e_k;
          c11_t1 = c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c11_f_k), 1, 2, 1, 0) - 1];
          c11_b_t1 = c11_t1;
          c11_b_f = c11_f;
          c11_b_eml_xrotg(chartInstance, &c11_b_t1, &c11_b_f, &c11_cs, &c11_sn);
          c11_t1 = c11_b_t1;
          c11_f = c11_b_f;
          c11_b_cs = c11_cs;
          c11_b_sn = c11_sn;
          c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_f_k), 1, 2, 1, 0) - 1] = c11_t1;
          if (c11_f_k > c11_b_q) {
            c11_km1 = c11_f_k - 1;
            c11_f = -c11_b_sn * c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c11_km1), 1, 2, 1, 0) - 1];
            c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c11_km1), 1, 2, 1, 0) - 1] =
              c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c11_km1), 1, 2, 1, 0) - 1] * c11_b_cs;
          }

          c11_colk = (c11_f_k - 1) << 1;
          c11_colm = c11_m << 1;
          c11_b_eml_xrot(chartInstance, c11_Vf, c11_colk + 1, c11_colm + 1,
                         c11_b_cs, c11_b_sn);
        }
        break;

       case 2:
        c11_qm1 = c11_b_q - 1;
        c11_f = c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c11_qm1), 1, 2, 1, 0) - 1];
        c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c11_qm1), 1, 2, 1, 0) - 1] = 0.0;
        c11_f_q = c11_b_q;
        c11_c_m = c11_m + 1;
        c11_l_a = c11_f_q;
        c11_e_b = c11_c_m;
        c11_m_a = c11_l_a;
        c11_f_b = c11_e_b;
        if (c11_m_a > c11_f_b) {
          c11_e_overflow = false;
        } else {
          c11_eml_switch_helper(chartInstance);
          c11_eml_switch_helper(chartInstance);
          c11_e_overflow = (c11_f_b > 2147483646);
        }

        if (c11_e_overflow) {
          c11_check_forloop_overflow_error(chartInstance, c11_e_overflow);
        }

        for (c11_g_k = c11_f_q; c11_g_k <= c11_c_m; c11_g_k++) {
          c11_f_k = c11_g_k;
          c11_t1 = c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c11_f_k), 1, 2, 1, 0) - 1];
          c11_c_t1 = c11_t1;
          c11_unusedU0 = c11_f;
          c11_b_eml_xrotg(chartInstance, &c11_c_t1, &c11_unusedU0, &c11_c_cs,
                          &c11_c_sn);
          c11_t1 = c11_c_t1;
          c11_b_cs = c11_c_cs;
          c11_b_sn = c11_c_sn;
          c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_f_k), 1, 2, 1, 0) - 1] = c11_t1;
          c11_f = -c11_b_sn * c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c11_f_k), 1, 2, 1, 0) - 1];
          c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_f_k), 1, 2, 1, 0) - 1] =
            c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_f_k), 1, 2, 1, 0) - 1] * c11_b_cs;
          c11_colk = (c11_f_k - 1) << 1;
          c11_colqm1 = (c11_qm1 - 1) << 1;
          c11_b_eml_xrot(chartInstance, c11_U, c11_colk + 1, c11_colqm1 + 1,
                         c11_b_cs, c11_b_sn);
        }
        break;

       case 3:
        c11_mm1 = c11_m;
        c11_d1 = c11_abs(chartInstance, c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)(c11_m + 1)), 1, 2, 1, 0) - 1]);
        c11_d2 = c11_abs(chartInstance, c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_mm1), 1, 2, 1, 0) - 1]);
        c11_d3 = c11_abs(chartInstance, c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_mm1), 1, 2, 1, 0) - 1]);
        c11_d4 = c11_abs(chartInstance, c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 1, 0) - 1]);
        c11_d5 = c11_abs(chartInstance, c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 1, 0) - 1]);
        c11_c_varargin_1[0] = c11_d1;
        c11_c_varargin_1[1] = c11_d2;
        c11_c_varargin_1[2] = c11_d3;
        c11_c_varargin_1[3] = c11_d4;
        c11_c_varargin_1[4] = c11_d5;
        c11_ixstart = 1;
        c11_mtmp = c11_c_varargin_1[0];
        c11_eb_x = c11_mtmp;
        c11_g_b = muDoubleScalarIsNaN(c11_eb_x);
        if (c11_g_b) {
          c11_eml_switch_helper(chartInstance);
          c11_eml_switch_helper(chartInstance);
          c11_ix = 2;
          exitg2 = false;
          while ((exitg2 == false) && (c11_ix < 6)) {
            c11_b_ix = c11_ix;
            c11_ixstart = c11_b_ix;
            c11_fb_x = c11_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c11_b_ix), 1, 5, 1, 0) - 1];
            c11_h_b = muDoubleScalarIsNaN(c11_fb_x);
            if (!c11_h_b) {
              c11_mtmp = c11_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_ix), 1, 5, 1, 0) -
                1];
              exitg2 = true;
            } else {
              c11_ix++;
            }
          }
        }

        if (c11_ixstart < 5) {
          c11_n_a = c11_ixstart;
          c11_o_a = c11_n_a + 1;
          c11_i203 = c11_o_a;
          c11_p_a = c11_i203;
          c11_q_a = c11_p_a;
          if (c11_q_a > 5) {
            c11_f_overflow = false;
          } else {
            c11_eml_switch_helper(chartInstance);
            c11_eml_switch_helper(chartInstance);
            c11_f_overflow = false;
          }

          if (c11_f_overflow) {
            c11_check_forloop_overflow_error(chartInstance, c11_f_overflow);
          }

          for (c11_c_ix = c11_i203; c11_c_ix < 6; c11_c_ix++) {
            c11_b_ix = c11_c_ix;
            c11_r_a = c11_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c11_b_ix), 1, 5, 1, 0) - 1];
            c11_i_b = c11_mtmp;
            c11_p = (c11_r_a > c11_i_b);
            if (c11_p) {
              c11_mtmp = c11_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_ix), 1, 5, 1, 0) -
                1];
            }
          }
        }

        c11_b_mtmp = c11_mtmp;
        c11_scale = c11_b_mtmp;
        c11_m_A = c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)(c11_m + 1)), 1, 2, 1, 0) - 1];
        c11_i_B = c11_scale;
        c11_gb_x = c11_m_A;
        c11_nb_y = c11_i_B;
        c11_hb_x = c11_gb_x;
        c11_ob_y = c11_nb_y;
        c11_ib_x = c11_hb_x;
        c11_pb_y = c11_ob_y;
        c11_sm = c11_ib_x / c11_pb_y;
        c11_n_A = c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c11_mm1), 1, 2, 1, 0) - 1];
        c11_j_B = c11_scale;
        c11_jb_x = c11_n_A;
        c11_qb_y = c11_j_B;
        c11_kb_x = c11_jb_x;
        c11_rb_y = c11_qb_y;
        c11_lb_x = c11_kb_x;
        c11_sb_y = c11_rb_y;
        c11_smm1 = c11_lb_x / c11_sb_y;
        c11_o_A = c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c11_mm1), 1, 2, 1, 0) - 1];
        c11_k_B = c11_scale;
        c11_mb_x = c11_o_A;
        c11_tb_y = c11_k_B;
        c11_nb_x = c11_mb_x;
        c11_ub_y = c11_tb_y;
        c11_ob_x = c11_nb_x;
        c11_vb_y = c11_ub_y;
        c11_emm1 = c11_ob_x / c11_vb_y;
        c11_p_A = c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 1, 0) - 1];
        c11_l_B = c11_scale;
        c11_pb_x = c11_p_A;
        c11_wb_y = c11_l_B;
        c11_qb_x = c11_pb_x;
        c11_xb_y = c11_wb_y;
        c11_rb_x = c11_qb_x;
        c11_yb_y = c11_xb_y;
        c11_sqds = c11_rb_x / c11_yb_y;
        c11_q_A = c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 1, 0) - 1];
        c11_m_B = c11_scale;
        c11_sb_x = c11_q_A;
        c11_ac_y = c11_m_B;
        c11_tb_x = c11_sb_x;
        c11_bc_y = c11_ac_y;
        c11_ub_x = c11_tb_x;
        c11_cc_y = c11_bc_y;
        c11_eqds = c11_ub_x / c11_cc_y;
        c11_r_A = (c11_smm1 + c11_sm) * (c11_smm1 - c11_sm) + c11_emm1 *
          c11_emm1;
        c11_vb_x = c11_r_A;
        c11_wb_x = c11_vb_x;
        c11_xb_x = c11_wb_x;
        c11_j_b = c11_xb_x / 2.0;
        c11_c = c11_sm * c11_emm1;
        c11_c *= c11_c;
        guard1 = false;
        if (c11_j_b != 0.0) {
          guard1 = true;
        } else if (c11_c != 0.0) {
          guard1 = true;
        } else {
          c11_shift = 0.0;
        }

        if (guard1 == true) {
          c11_shift = c11_j_b * c11_j_b + c11_c;
          c11_b_sqrt(chartInstance, &c11_shift);
          if (c11_j_b < 0.0) {
            c11_shift = -c11_shift;
          }

          c11_s_A = c11_c;
          c11_n_B = c11_j_b + c11_shift;
          c11_yb_x = c11_s_A;
          c11_dc_y = c11_n_B;
          c11_ac_x = c11_yb_x;
          c11_ec_y = c11_dc_y;
          c11_bc_x = c11_ac_x;
          c11_fc_y = c11_ec_y;
          c11_shift = c11_bc_x / c11_fc_y;
        }

        c11_f = (c11_sqds + c11_sm) * (c11_sqds - c11_sm) + c11_shift;
        c11_g = c11_sqds * c11_eqds;
        c11_g_q = c11_b_q;
        c11_b_mm1 = c11_mm1;
        c11_s_a = c11_g_q;
        c11_k_b = c11_b_mm1;
        c11_t_a = c11_s_a;
        c11_l_b = c11_k_b;
        if (c11_t_a > c11_l_b) {
          c11_g_overflow = false;
        } else {
          c11_eml_switch_helper(chartInstance);
          c11_eml_switch_helper(chartInstance);
          c11_g_overflow = (c11_l_b > 2147483646);
        }

        if (c11_g_overflow) {
          c11_check_forloop_overflow_error(chartInstance, c11_g_overflow);
        }

        for (c11_h_k = c11_g_q; c11_h_k <= c11_b_mm1; c11_h_k++) {
          c11_f_k = c11_h_k;
          c11_km1 = c11_f_k;
          c11_kp1 = c11_f_k + 1;
          c11_c_f = c11_f;
          c11_unusedU1 = c11_g;
          c11_b_eml_xrotg(chartInstance, &c11_c_f, &c11_unusedU1, &c11_d_cs,
                          &c11_d_sn);
          c11_f = c11_c_f;
          c11_b_cs = c11_d_cs;
          c11_b_sn = c11_d_sn;
          if (c11_f_k > c11_b_q) {
            c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)(c11_km1 - 1)), 1, 2, 1, 0) - 1] = c11_f;
          }

          c11_f = c11_b_cs * c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c11_f_k), 1, 2, 1, 0) - 1] + c11_b_sn
            * c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", (real_T)c11_f_k), 1, 2, 1, 0) - 1];
          c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_f_k), 1, 2, 1, 0) - 1] = c11_b_cs *
            c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_f_k), 1, 2, 1, 0) - 1] - c11_b_sn *
            c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_f_k), 1, 2, 1, 0) - 1];
          c11_g = c11_b_sn * c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c11_kp1), 1, 2, 1, 0) - 1];
          c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_kp1), 1, 2, 1, 0) - 1] =
            c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_kp1), 1, 2, 1, 0) - 1] * c11_b_cs;
          c11_colk = 1 + ((c11_f_k - 1) << 1);
          c11_colkp1 = 1 + (c11_f_k << 1);
          c11_b_eml_xrot(chartInstance, c11_Vf, c11_colk, c11_colkp1, c11_b_cs,
                         c11_b_sn);
          c11_d_f = c11_f;
          c11_unusedU2 = c11_g;
          c11_b_eml_xrotg(chartInstance, &c11_d_f, &c11_unusedU2, &c11_e_cs,
                          &c11_e_sn);
          c11_f = c11_d_f;
          c11_b_cs = c11_e_cs;
          c11_b_sn = c11_e_sn;
          c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_f_k), 1, 2, 1, 0) - 1] = c11_f;
          c11_f = c11_b_cs * c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c11_f_k), 1, 2, 1, 0) - 1] + c11_b_sn
            * c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", (real_T)c11_kp1), 1, 2, 1, 0) - 1];
          c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_kp1), 1, 2, 1, 0) - 1] = -c11_b_sn *
            c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_f_k), 1, 2, 1, 0) - 1] + c11_b_cs *
            c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_kp1), 1, 2, 1, 0) - 1];
          c11_g = c11_b_sn * c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c11_kp1), 1, 2, 1, 0) - 1];
          c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_kp1), 1, 2, 1, 0) - 1] =
            c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_kp1), 1, 2, 1, 0) - 1] * c11_b_cs;
          if (c11_f_k < 2) {
            c11_b_eml_xrot(chartInstance, c11_U, c11_colk, c11_colkp1, c11_b_cs,
                           c11_b_sn);
          }
        }

        c11_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c11_m), 1, 2, 1, 0) - 1] = c11_f;
        c11_iter++;
        break;

       default:
        if (c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c11_b_q), 1, 2, 1, 0) - 1] < 0.0) {
          c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_b_q), 1, 2, 1, 0) - 1] =
            -c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", (real_T)c11_b_q), 1, 2, 1, 0) - 1];
          c11_colq = (c11_b_q - 1) << 1;
          c11_f_eml_scalar_eg(chartInstance);
          c11_b_eml_xscal(chartInstance, -1.0, c11_Vf, c11_colq + 1);
        }

        c11_qp1 = c11_b_q + 1;
        exitg3 = false;
        while ((exitg3 == false) && (c11_b_q < 2)) {
          if (c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c11_b_q), 1, 2, 1, 0) - 1] <
              c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c11_qp1), 1, 2, 1, 0) - 1]) {
            c11_rt = c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c11_b_q), 1, 2, 1, 0) - 1];
            c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c11_b_q), 1, 2, 1, 0) - 1] =
              c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c11_qp1), 1, 2, 1, 0) - 1];
            c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c11_qp1), 1, 2, 1, 0) - 1] = c11_rt;
            if (c11_b_q < 2) {
              c11_colq = (c11_b_q - 1) << 1;
              c11_colqp1 = c11_b_q << 1;
              c11_b_eml_xswap(chartInstance, c11_Vf, c11_colq + 1, c11_colqp1 +
                              1);
            }

            if (c11_b_q < 2) {
              c11_colq = (c11_b_q - 1) << 1;
              c11_colqp1 = c11_b_q << 1;
              c11_b_eml_xswap(chartInstance, c11_U, c11_colq + 1, c11_colqp1 + 1);
            }

            c11_b_q = c11_qp1;
            c11_qp1 = c11_b_q + 1;
          } else {
            exitg3 = true;
          }
        }

        c11_iter = 0.0;
        c11_m--;
        break;
      }
    }
  }

  c11_eml_switch_helper(chartInstance);
  for (c11_i_k = 1; c11_i_k < 3; c11_i_k++) {
    c11_f_k = c11_i_k;
    c11_S[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_f_k), 1, 2, 1, 0) - 1] = c11_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_f_k), 1, 2, 1, 0) - 1];
  }

  c11_eml_switch_helper(chartInstance);
  for (c11_j = 1; c11_j < 3; c11_j++) {
    c11_b_j = c11_j;
    c11_eml_switch_helper(chartInstance);
    for (c11_i = 1; c11_i < 3; c11_i++) {
      c11_b_i = c11_i;
      c11_V[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c11_b_i), 1, 2, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
                (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_b_j), 1, 2, 2, 0) -
               1) << 1)) - 1] = c11_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c11_b_i), 1, 2, 1, 0) +
        ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c11_b_j), 1, 2, 2, 0) - 1) << 1)) - 1];
    }
  }
}

static real_T c11_eml_xnrm2(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4])
{
  real_T c11_y;
  real_T c11_scale;
  int32_T c11_k;
  int32_T c11_b_k;
  real_T c11_b_x;
  real_T c11_c_x;
  real_T c11_absxk;
  real_T c11_t;
  c11_below_threshold(chartInstance);
  c11_y = 0.0;
  c11_realmin(chartInstance);
  c11_scale = 2.2250738585072014E-308;
  c11_eml_switch_helper(chartInstance);
  for (c11_k = 1; c11_k < 3; c11_k++) {
    c11_b_k = c11_k;
    c11_b_x = c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c11_b_k), 1, 4, 1, 0) - 1];
    c11_c_x = c11_b_x;
    c11_absxk = muDoubleScalarAbs(c11_c_x);
    if (c11_absxk > c11_scale) {
      c11_t = c11_scale / c11_absxk;
      c11_y = 1.0 + c11_y * c11_t * c11_t;
      c11_scale = c11_absxk;
    } else {
      c11_t = c11_absxk / c11_scale;
      c11_y += c11_t * c11_t;
    }
  }

  return c11_scale * muDoubleScalarSqrt(c11_y);
}

static void c11_below_threshold(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_realmin(SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c11_abs(SFc11_SS6_Estimation2InstanceStruct *chartInstance, real_T
                      c11_x)
{
  real_T c11_b_x;
  (void)chartInstance;
  c11_b_x = c11_x;
  return muDoubleScalarAbs(c11_b_x);
}

static void c11_eps(SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c11_b_below_threshold(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  c11_b_eml_switch_helper(chartInstance);
}

static void c11_b_eml_switch_helper(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c11_eml_xdotc(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4], real_T c11_y[4])
{
  real_T c11_d;
  int32_T c11_ix;
  int32_T c11_iy;
  int32_T c11_k;
  int32_T c11_a;
  int32_T c11_b_a;
  c11_b_threshold(chartInstance);
  c11_d = 0.0;
  c11_ix = 1;
  c11_iy = 3;
  c11_eml_switch_helper(chartInstance);
  for (c11_k = 1; c11_k < 3; c11_k++) {
    c11_d += c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c11_ix), 1, 4, 1, 0) - 1] * c11_y[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_iy), 1, 4, 1, 0) - 1];
    c11_a = c11_ix + 1;
    c11_ix = c11_a;
    c11_b_a = c11_iy + 1;
    c11_iy = c11_b_a;
  }

  return c11_d;
}

static void c11_b_threshold(SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c11_eml_xaxpy(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_a, real_T c11_y[4], real_T c11_b_y[4])
{
  int32_T c11_i204;
  for (c11_i204 = 0; c11_i204 < 4; c11_i204++) {
    c11_b_y[c11_i204] = c11_y[c11_i204];
  }

  c11_c_eml_xaxpy(chartInstance, c11_a, c11_b_y);
}

static void c11_c_threshold(SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c11_check_forloop_overflow_error(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, boolean_T c11_overflow)
{
  int32_T c11_i205;
  static char_T c11_cv1[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c11_u[34];
  const mxArray *c11_y = NULL;
  int32_T c11_i206;
  static char_T c11_cv2[5] = { 'i', 'n', 't', '3', '2' };

  char_T c11_b_u[5];
  const mxArray *c11_b_y = NULL;
  (void)chartInstance;
  if (!c11_overflow) {
  } else {
    for (c11_i205 = 0; c11_i205 < 34; c11_i205++) {
      c11_u[c11_i205] = c11_cv1[c11_i205];
    }

    c11_y = NULL;
    sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  false);
    for (c11_i206 = 0; c11_i206 < 5; c11_i206++) {
      c11_b_u[c11_i206] = c11_cv2[c11_i206];
    }

    c11_b_y = NULL;
    sf_mex_assign(&c11_b_y, sf_mex_create("y", c11_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 2U, 14, c11_y, 14, c11_b_y));
  }
}

static real_T c11_b_eml_xdotc(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c11_n, real_T c11_x[4], int32_T c11_ix0, real_T c11_y[4], int32_T
  c11_iy0)
{
  real_T c11_d;
  int32_T c11_b_n;
  int32_T c11_b_ix0;
  int32_T c11_b_iy0;
  int32_T c11_c_n;
  int32_T c11_c_ix0;
  int32_T c11_c_iy0;
  int32_T c11_d_n;
  int32_T c11_d_ix0;
  int32_T c11_d_iy0;
  int32_T c11_e_n;
  int32_T c11_e_ix0;
  int32_T c11_e_iy0;
  int32_T c11_ix;
  int32_T c11_iy;
  int32_T c11_f_n;
  int32_T c11_b;
  int32_T c11_b_b;
  boolean_T c11_overflow;
  int32_T c11_k;
  int32_T c11_a;
  int32_T c11_b_a;
  c11_b_n = c11_n;
  c11_b_ix0 = c11_ix0;
  c11_b_iy0 = c11_iy0;
  c11_c_n = c11_b_n;
  c11_c_ix0 = c11_b_ix0;
  c11_c_iy0 = c11_b_iy0;
  c11_b_threshold(chartInstance);
  c11_d_n = c11_c_n;
  c11_d_ix0 = c11_c_ix0;
  c11_d_iy0 = c11_c_iy0;
  c11_e_n = c11_d_n;
  c11_e_ix0 = c11_d_ix0;
  c11_e_iy0 = c11_d_iy0;
  c11_d = 0.0;
  if (c11_e_n < 1) {
  } else {
    c11_ix = c11_e_ix0;
    c11_iy = c11_e_iy0;
    c11_f_n = c11_e_n;
    c11_b = c11_f_n;
    c11_b_b = c11_b;
    if (1 > c11_b_b) {
      c11_overflow = false;
    } else {
      c11_eml_switch_helper(chartInstance);
      c11_eml_switch_helper(chartInstance);
      c11_overflow = (c11_b_b > 2147483646);
    }

    if (c11_overflow) {
      c11_check_forloop_overflow_error(chartInstance, c11_overflow);
    }

    for (c11_k = 1; c11_k <= c11_f_n; c11_k++) {
      c11_d += c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c11_ix), 1, 4, 1, 0) - 1] *
        c11_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c11_iy), 1, 4, 1, 0) - 1];
      c11_a = c11_ix + 1;
      c11_ix = c11_a;
      c11_b_a = c11_iy + 1;
      c11_iy = c11_b_a;
    }
  }

  return c11_d;
}

static void c11_b_eml_xaxpy(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c11_n, real_T c11_a, int32_T c11_ix0, real_T c11_y[4], int32_T c11_iy0,
  real_T c11_b_y[4])
{
  int32_T c11_i207;
  for (c11_i207 = 0; c11_i207 < 4; c11_i207++) {
    c11_b_y[c11_i207] = c11_y[c11_i207];
  }

  c11_d_eml_xaxpy(chartInstance, c11_n, c11_a, c11_ix0, c11_b_y, c11_iy0);
}

static void c11_eml_xscal(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_a, real_T c11_x[4], int32_T c11_ix0, real_T c11_b_x[4])
{
  int32_T c11_i208;
  for (c11_i208 = 0; c11_i208 < 4; c11_i208++) {
    c11_b_x[c11_i208] = c11_x[c11_i208];
  }

  c11_b_eml_xscal(chartInstance, c11_a, c11_b_x, c11_ix0);
}

static void c11_g_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_b_eml_error(SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  int32_T c11_i209;
  static char_T c11_cv3[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 's', 'v', 'd', '_', 'N', 'o', 'C', 'o', 'n', 'v', 'e', 'r',
    'g', 'e', 'n', 'c', 'e' };

  char_T c11_u[30];
  const mxArray *c11_y = NULL;
  (void)chartInstance;
  for (c11_i209 = 0; c11_i209 < 30; c11_i209++) {
    c11_u[c11_i209] = c11_cv3[c11_i209];
  }

  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 10, 0U, 1U, 0U, 2, 1, 30),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c11_y));
}

static real_T c11_sqrt(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x)
{
  real_T c11_b_x;
  c11_b_x = c11_x;
  c11_b_sqrt(chartInstance, &c11_b_x);
  return c11_b_x;
}

static void c11_c_eml_error(SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  int32_T c11_i210;
  static char_T c11_cv4[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c11_u[30];
  const mxArray *c11_y = NULL;
  int32_T c11_i211;
  static char_T c11_cv5[4] = { 's', 'q', 'r', 't' };

  char_T c11_b_u[4];
  const mxArray *c11_b_y = NULL;
  (void)chartInstance;
  for (c11_i210 = 0; c11_i210 < 30; c11_i210++) {
    c11_u[c11_i210] = c11_cv4[c11_i210];
  }

  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", c11_u, 10, 0U, 1U, 0U, 2, 1, 30),
                false);
  for (c11_i211 = 0; c11_i211 < 4; c11_i211++) {
    c11_b_u[c11_i211] = c11_cv5[c11_i211];
  }

  c11_b_y = NULL;
  sf_mex_assign(&c11_b_y, sf_mex_create("y", c11_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c11_y, 14, c11_b_y));
}

static void c11_eml_xrotg(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_a, real_T c11_b, real_T *c11_b_a, real_T *c11_b_b, real_T *c11_c,
  real_T *c11_s)
{
  *c11_b_a = c11_a;
  *c11_b_b = c11_b;
  c11_b_eml_xrotg(chartInstance, c11_b_a, c11_b_b, c11_c, c11_s);
}

static void c11_eml_xrot(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4], int32_T c11_ix0, int32_T c11_iy0, real_T c11_c, real_T c11_s,
  real_T c11_b_x[4])
{
  int32_T c11_i212;
  for (c11_i212 = 0; c11_i212 < 4; c11_i212++) {
    c11_b_x[c11_i212] = c11_x[c11_i212];
  }

  c11_b_eml_xrot(chartInstance, c11_b_x, c11_ix0, c11_iy0, c11_c, c11_s);
}

static void c11_d_threshold(SFc11_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c11_eml_xswap(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4], int32_T c11_ix0, int32_T c11_iy0, real_T c11_b_x[4])
{
  int32_T c11_i213;
  for (c11_i213 = 0; c11_i213 < 4; c11_i213++) {
    c11_b_x[c11_i213] = c11_x[c11_i213];
  }

  c11_b_eml_xswap(chartInstance, c11_b_x, c11_ix0, c11_iy0);
}

static void c11_c_eml_switch_helper(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_eml_xgemm(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c11_k, real_T c11_A[4], real_T c11_B[4], real_T c11_C[4], real_T
  c11_b_C[4])
{
  int32_T c11_i214;
  int32_T c11_i215;
  real_T c11_b_A[4];
  int32_T c11_i216;
  real_T c11_b_B[4];
  for (c11_i214 = 0; c11_i214 < 4; c11_i214++) {
    c11_b_C[c11_i214] = c11_C[c11_i214];
  }

  for (c11_i215 = 0; c11_i215 < 4; c11_i215++) {
    c11_b_A[c11_i215] = c11_A[c11_i215];
  }

  for (c11_i216 = 0; c11_i216 < 4; c11_i216++) {
    c11_b_B[c11_i216] = c11_B[c11_i216];
  }

  c11_b_eml_xgemm(chartInstance, c11_k, c11_b_A, c11_b_B, c11_b_C);
}

static void c11_h_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_i_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_j_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c11_k_eml_scalar_eg(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c11_k_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  int32_T c11_u;
  const mxArray *c11_y = NULL;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  c11_u = *(int32_T *)c11_inData;
  c11_y = NULL;
  sf_mex_assign(&c11_y, sf_mex_create("y", &c11_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c11_mxArrayOutData, c11_y, false);
  return c11_mxArrayOutData;
}

static int32_T c11_l_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId)
{
  int32_T c11_y;
  int32_T c11_i217;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_u), &c11_i217, 1, 6, 0U, 0, 0U, 0);
  c11_y = c11_i217;
  sf_mex_destroy(&c11_u);
  return c11_y;
}

static void c11_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_b_sfEvent;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  int32_T c11_y;
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c11_b_sfEvent = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_y = c11_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_b_sfEvent),
    &c11_thisId);
  sf_mex_destroy(&c11_b_sfEvent);
  *(int32_T *)c11_outData = c11_y;
  sf_mex_destroy(&c11_mxArrayInData);
}

static uint8_T c11_m_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_b_is_active_c11_SS6_Estimation2, const
  char_T *c11_identifier)
{
  uint8_T c11_y;
  emlrtMsgIdentifier c11_thisId;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_y = c11_n_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c11_b_is_active_c11_SS6_Estimation2), &c11_thisId);
  sf_mex_destroy(&c11_b_is_active_c11_SS6_Estimation2);
  return c11_y;
}

static uint8_T c11_n_emlrt_marshallIn(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c11_u, const emlrtMsgIdentifier *c11_parentId)
{
  uint8_T c11_y;
  uint8_T c11_u0;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_u), &c11_u0, 1, 3, 0U, 0, 0U, 0);
  c11_y = c11_u0;
  sf_mex_destroy(&c11_u);
  return c11_y;
}

static void c11_c_eml_xaxpy(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_a, real_T c11_y[4])
{
  real_T c11_b_a;
  real_T c11_c_a;
  int32_T c11_ix;
  int32_T c11_iy;
  int32_T c11_k;
  int32_T c11_d_a;
  int32_T c11_c;
  int32_T c11_e_a;
  int32_T c11_b_c;
  int32_T c11_f_a;
  int32_T c11_c_c;
  int32_T c11_g_a;
  int32_T c11_h_a;
  c11_b_a = c11_a;
  c11_c_threshold(chartInstance);
  c11_c_a = c11_b_a;
  if (c11_c_a == 0.0) {
  } else {
    c11_ix = 0;
    c11_iy = 2;
    c11_eml_switch_helper(chartInstance);
    for (c11_k = 0; c11_k < 2; c11_k++) {
      c11_d_a = c11_iy;
      c11_c = c11_d_a;
      c11_e_a = c11_iy;
      c11_b_c = c11_e_a;
      c11_f_a = c11_ix;
      c11_c_c = c11_f_a;
      c11_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c11_c + 1)), 1, 4, 1, 0) - 1] =
        c11_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c11_b_c + 1)), 1, 4, 1, 0) - 1] + c11_c_a *
        c11_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c11_c_c + 1)), 1, 4, 1, 0) - 1];
      c11_g_a = c11_ix + 1;
      c11_ix = c11_g_a;
      c11_h_a = c11_iy + 1;
      c11_iy = c11_h_a;
    }
  }
}

static void c11_d_eml_xaxpy(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c11_n, real_T c11_a, int32_T c11_ix0, real_T c11_y[4], int32_T c11_iy0)
{
  int32_T c11_b_n;
  real_T c11_b_a;
  int32_T c11_b_ix0;
  int32_T c11_b_iy0;
  int32_T c11_c_n;
  real_T c11_c_a;
  int32_T c11_c_ix0;
  int32_T c11_c_iy0;
  int32_T c11_d_a;
  int32_T c11_ix;
  int32_T c11_e_a;
  int32_T c11_iy;
  int32_T c11_f_a;
  int32_T c11_i218;
  int32_T c11_b;
  int32_T c11_b_b;
  boolean_T c11_overflow;
  int32_T c11_k;
  int32_T c11_g_a;
  int32_T c11_c;
  int32_T c11_h_a;
  int32_T c11_b_c;
  int32_T c11_i_a;
  int32_T c11_c_c;
  int32_T c11_j_a;
  int32_T c11_k_a;
  c11_b_n = c11_n;
  c11_b_a = c11_a;
  c11_b_ix0 = c11_ix0;
  c11_b_iy0 = c11_iy0;
  c11_c_threshold(chartInstance);
  c11_c_n = c11_b_n;
  c11_c_a = c11_b_a;
  c11_c_ix0 = c11_b_ix0;
  c11_c_iy0 = c11_b_iy0;
  if (c11_c_n < 1) {
  } else if (c11_c_a == 0.0) {
  } else {
    c11_d_a = c11_c_ix0 - 1;
    c11_ix = c11_d_a;
    c11_e_a = c11_c_iy0 - 1;
    c11_iy = c11_e_a;
    c11_f_a = c11_c_n - 1;
    c11_i218 = c11_f_a;
    c11_b = c11_i218;
    c11_b_b = c11_b;
    if (0 > c11_b_b) {
      c11_overflow = false;
    } else {
      c11_eml_switch_helper(chartInstance);
      c11_eml_switch_helper(chartInstance);
      c11_overflow = (c11_b_b > 2147483646);
    }

    if (c11_overflow) {
      c11_check_forloop_overflow_error(chartInstance, c11_overflow);
    }

    for (c11_k = 0; c11_k <= c11_i218; c11_k++) {
      c11_g_a = c11_iy;
      c11_c = c11_g_a;
      c11_h_a = c11_iy;
      c11_b_c = c11_h_a;
      c11_i_a = c11_ix;
      c11_c_c = c11_i_a;
      c11_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c11_c + 1)), 1, 4, 1, 0) - 1] =
        c11_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c11_b_c + 1)), 1, 4, 1, 0) - 1] + c11_c_a *
        c11_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c11_c_c + 1)), 1, 4, 1, 0) - 1];
      c11_j_a = c11_ix + 1;
      c11_ix = c11_j_a;
      c11_k_a = c11_iy + 1;
      c11_iy = c11_k_a;
    }
  }
}

static void c11_b_eml_xscal(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_a, real_T c11_x[4], int32_T c11_ix0)
{
  real_T c11_b_a;
  int32_T c11_b_ix0;
  real_T c11_c_a;
  int32_T c11_c_ix0;
  int32_T c11_d_ix0;
  int32_T c11_d_a;
  int32_T c11_i219;
  int32_T c11_e_a;
  int32_T c11_b;
  int32_T c11_f_a;
  int32_T c11_b_b;
  boolean_T c11_overflow;
  int32_T c11_k;
  int32_T c11_b_k;
  c11_b_a = c11_a;
  c11_b_ix0 = c11_ix0;
  c11_b_below_threshold(chartInstance);
  c11_c_a = c11_b_a;
  c11_c_ix0 = c11_b_ix0;
  c11_d_ix0 = c11_c_ix0;
  c11_d_a = c11_c_ix0 + 1;
  c11_i219 = c11_d_a;
  c11_e_a = c11_d_ix0;
  c11_b = c11_i219;
  c11_f_a = c11_e_a;
  c11_b_b = c11_b;
  if (c11_f_a > c11_b_b) {
    c11_overflow = false;
  } else {
    c11_eml_switch_helper(chartInstance);
    c11_eml_switch_helper(chartInstance);
    c11_overflow = (c11_b_b > 2147483646);
  }

  if (c11_overflow) {
    c11_check_forloop_overflow_error(chartInstance, c11_overflow);
  }

  for (c11_k = c11_d_ix0; c11_k <= c11_i219; c11_k++) {
    c11_b_k = c11_k;
    c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_b_k), 1, 4, 1, 0) - 1] = c11_c_a *
      c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_b_k), 1, 4, 1, 0) - 1];
  }
}

static void c11_b_sqrt(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T *c11_x)
{
  if (*c11_x < 0.0) {
    c11_c_eml_error(chartInstance);
  }

  *c11_x = muDoubleScalarSqrt(*c11_x);
}

static void c11_b_eml_xrotg(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T *c11_a, real_T *c11_b, real_T *c11_c, real_T *c11_s)
{
  real_T c11_b_a;
  real_T c11_b_b;
  real_T c11_c_b;
  real_T c11_c_a;
  real_T c11_d_a;
  real_T c11_d_b;
  real_T c11_e_b;
  real_T c11_e_a;
  real_T c11_b_c;
  real_T c11_b_s;
  double * c11_a_t;
  double * c11_b_t;
  double * c11_c_t;
  double * c11_s_t;
  real_T c11_c_c;
  real_T c11_c_s;
  (void)chartInstance;
  c11_b_a = *c11_a;
  c11_b_b = *c11_b;
  c11_c_b = c11_b_b;
  c11_c_a = c11_b_a;
  c11_d_a = c11_c_a;
  c11_d_b = c11_c_b;
  c11_e_b = c11_d_b;
  c11_e_a = c11_d_a;
  c11_b_c = 0.0;
  c11_b_s = 0.0;
  c11_a_t = (double *)(&c11_e_a);
  c11_b_t = (double *)(&c11_e_b);
  c11_c_t = (double *)(&c11_b_c);
  c11_s_t = (double *)(&c11_b_s);
  drotg(c11_a_t, c11_b_t, c11_c_t, c11_s_t);
  c11_c_a = c11_e_a;
  c11_c_b = c11_e_b;
  c11_c_c = c11_b_c;
  c11_c_s = c11_b_s;
  *c11_a = c11_c_a;
  *c11_b = c11_c_b;
  *c11_c = c11_c_c;
  *c11_s = c11_c_s;
}

static void c11_b_eml_xrot(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4], int32_T c11_ix0, int32_T c11_iy0, real_T c11_c, real_T c11_s)
{
  int32_T c11_b_ix0;
  int32_T c11_b_iy0;
  real_T c11_b_c;
  real_T c11_b_s;
  int32_T c11_c_ix0;
  int32_T c11_c_iy0;
  real_T c11_c_c;
  real_T c11_c_s;
  int32_T c11_ix;
  int32_T c11_iy;
  int32_T c11_k;
  real_T c11_temp;
  int32_T c11_a;
  int32_T c11_b_a;
  c11_b_ix0 = c11_ix0;
  c11_b_iy0 = c11_iy0;
  c11_b_c = c11_c;
  c11_b_s = c11_s;
  c11_d_threshold(chartInstance);
  c11_c_ix0 = c11_b_ix0;
  c11_c_iy0 = c11_b_iy0;
  c11_c_c = c11_b_c;
  c11_c_s = c11_b_s;
  c11_ix = c11_c_ix0;
  c11_iy = c11_c_iy0;
  c11_eml_switch_helper(chartInstance);
  for (c11_k = 1; c11_k < 3; c11_k++) {
    c11_temp = c11_c_c * c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c11_ix), 1, 4, 1, 0) - 1] + c11_c_s *
      c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_iy), 1, 4, 1, 0) - 1];
    c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_iy), 1, 4, 1, 0) - 1] = c11_c_c *
      c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_iy), 1, 4, 1, 0) - 1] - c11_c_s *
      c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_ix), 1, 4, 1, 0) - 1];
    c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_ix), 1, 4, 1, 0) - 1] = c11_temp;
    c11_a = c11_iy + 1;
    c11_iy = c11_a;
    c11_b_a = c11_ix + 1;
    c11_ix = c11_b_a;
  }
}

static void c11_b_eml_xswap(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c11_x[4], int32_T c11_ix0, int32_T c11_iy0)
{
  int32_T c11_b_ix0;
  int32_T c11_b_iy0;
  int32_T c11_c_ix0;
  int32_T c11_c_iy0;
  int32_T c11_ix;
  int32_T c11_iy;
  int32_T c11_k;
  real_T c11_temp;
  int32_T c11_a;
  int32_T c11_b_a;
  c11_b_ix0 = c11_ix0;
  c11_b_iy0 = c11_iy0;
  c11_c_eml_switch_helper(chartInstance);
  c11_c_ix0 = c11_b_ix0;
  c11_c_iy0 = c11_b_iy0;
  c11_ix = c11_c_ix0;
  c11_iy = c11_c_iy0;
  c11_eml_switch_helper(chartInstance);
  for (c11_k = 1; c11_k < 3; c11_k++) {
    c11_temp = c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c11_ix), 1, 4, 1, 0) - 1];
    c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_ix), 1, 4, 1, 0) - 1] = c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c11_iy), 1, 4, 1, 0) - 1];
    c11_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c11_iy), 1, 4, 1, 0) - 1] = c11_temp;
    c11_a = c11_ix + 1;
    c11_ix = c11_a;
    c11_b_a = c11_iy + 1;
    c11_iy = c11_b_a;
  }
}

static void c11_b_eml_xgemm(SFc11_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c11_k, real_T c11_A[4], real_T c11_B[4], real_T c11_C[4])
{
  int32_T c11_b_k;
  int32_T c11_c_k;
  int32_T c11_a;
  int32_T c11_km1;
  int32_T c11_cr;
  int32_T c11_b_cr;
  int32_T c11_b_a;
  int32_T c11_i220;
  int32_T c11_c_a;
  int32_T c11_i221;
  int32_T c11_d_a;
  int32_T c11_b;
  int32_T c11_e_a;
  int32_T c11_b_b;
  boolean_T c11_overflow;
  int32_T c11_ic;
  int32_T c11_b_ic;
  int32_T c11_br;
  int32_T c11_c_cr;
  int32_T c11_ar;
  int32_T c11_f_a;
  int32_T c11_b_br;
  int32_T c11_c_b;
  int32_T c11_c;
  int32_T c11_g_a;
  int32_T c11_d_b;
  int32_T c11_i222;
  int32_T c11_h_a;
  int32_T c11_e_b;
  int32_T c11_i_a;
  int32_T c11_f_b;
  boolean_T c11_b_overflow;
  int32_T c11_ib;
  int32_T c11_b_ib;
  real_T c11_temp;
  int32_T c11_ia;
  int32_T c11_j_a;
  int32_T c11_i223;
  int32_T c11_k_a;
  int32_T c11_i224;
  int32_T c11_l_a;
  int32_T c11_g_b;
  int32_T c11_m_a;
  int32_T c11_h_b;
  boolean_T c11_c_overflow;
  int32_T c11_c_ic;
  int32_T c11_n_a;
  int32_T c11_o_a;
  c11_b_k = c11_k;
  if (c11_use_refblas(chartInstance)) {
  } else {
    c11_threshold(chartInstance);
  }

  c11_c_k = c11_b_k;
  c11_a = c11_c_k;
  c11_km1 = c11_a;
  c11_eml_switch_helper(chartInstance);
  for (c11_cr = 0; c11_cr < 3; c11_cr += 2) {
    c11_b_cr = c11_cr;
    c11_b_a = c11_b_cr + 1;
    c11_i220 = c11_b_a;
    c11_c_a = c11_b_cr + 2;
    c11_i221 = c11_c_a;
    c11_d_a = c11_i220;
    c11_b = c11_i221;
    c11_e_a = c11_d_a;
    c11_b_b = c11_b;
    if (c11_e_a > c11_b_b) {
      c11_overflow = false;
    } else {
      c11_eml_switch_helper(chartInstance);
      c11_eml_switch_helper(chartInstance);
      c11_overflow = (c11_b_b > 2147483646);
    }

    if (c11_overflow) {
      c11_check_forloop_overflow_error(chartInstance, c11_overflow);
    }

    for (c11_ic = c11_i220; c11_ic <= c11_i221; c11_ic++) {
      c11_b_ic = c11_ic;
      c11_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c11_b_ic), 1, 4, 1, 0) - 1] = 0.0;
    }
  }

  c11_br = 0;
  c11_eml_switch_helper(chartInstance);
  for (c11_c_cr = 0; c11_c_cr < 3; c11_c_cr += 2) {
    c11_b_cr = c11_c_cr;
    c11_ar = 0;
    c11_f_a = c11_br + 1;
    c11_br = c11_f_a;
    c11_b_br = c11_br;
    c11_c_b = c11_km1 - 1;
    c11_c = c11_c_b << 1;
    c11_g_a = c11_br;
    c11_d_b = c11_c;
    c11_i222 = c11_g_a + c11_d_b;
    c11_h_a = c11_b_br;
    c11_e_b = c11_i222;
    c11_i_a = c11_h_a;
    c11_f_b = c11_e_b;
    if (c11_i_a > c11_f_b) {
      c11_b_overflow = false;
    } else {
      c11_eml_switch_helper(chartInstance);
      c11_eml_switch_helper(chartInstance);
      c11_b_overflow = (c11_f_b > 2147483645);
    }

    if (c11_b_overflow) {
      c11_check_forloop_overflow_error(chartInstance, c11_b_overflow);
    }

    for (c11_ib = c11_b_br; c11_ib <= c11_i222; c11_ib += 2) {
      c11_b_ib = c11_ib;
      if (c11_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_b_ib), 1, 4, 1, 0) - 1] != 0.0) {
        c11_temp = c11_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c11_b_ib), 1, 4, 1, 0) - 1];
        c11_ia = c11_ar;
        c11_j_a = c11_b_cr + 1;
        c11_i223 = c11_j_a;
        c11_k_a = c11_b_cr + 2;
        c11_i224 = c11_k_a;
        c11_l_a = c11_i223;
        c11_g_b = c11_i224;
        c11_m_a = c11_l_a;
        c11_h_b = c11_g_b;
        if (c11_m_a > c11_h_b) {
          c11_c_overflow = false;
        } else {
          c11_eml_switch_helper(chartInstance);
          c11_eml_switch_helper(chartInstance);
          c11_c_overflow = (c11_h_b > 2147483646);
        }

        if (c11_c_overflow) {
          c11_check_forloop_overflow_error(chartInstance, c11_c_overflow);
        }

        for (c11_c_ic = c11_i223; c11_c_ic <= c11_i224; c11_c_ic++) {
          c11_b_ic = c11_c_ic;
          c11_n_a = c11_ia + 1;
          c11_ia = c11_n_a;
          c11_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_b_ic), 1, 4, 1, 0) - 1] =
            c11_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_b_ic), 1, 4, 1, 0) - 1] + c11_temp *
            c11_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c11_ia), 1, 4, 1, 0) - 1];
        }
      }

      c11_o_a = c11_ar + 2;
      c11_ar = c11_o_a;
    }
  }
}

static void init_dsm_address_info(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc11_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  chartInstance->c11_X_1 = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c11_Y_1 = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c11_X_GPS = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c11_Y_GPS = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c11_aX_IMU = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c11_aY_IMU = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c11_r_IMU = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c11_V_x = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c11_V_y = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c11_SS6_Estimation2_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1121954543U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4036565571U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(170429564U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3070783999U);
}

mxArray* sf_c11_SS6_Estimation2_get_post_codegen_info(void);
mxArray *sf_c11_SS6_Estimation2_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("ynfZF7f68KRX4SVzhqPVsD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c11_SS6_Estimation2_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c11_SS6_Estimation2_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c11_SS6_Estimation2_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "incompatibleSymbol", };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 3, infoFields);
  mxArray *fallbackReason = mxCreateString("feature_off");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxArray *fallbackType = mxCreateString("early");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c11_SS6_Estimation2_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c11_SS6_Estimation2_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c11_SS6_Estimation2(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x7'type','srcId','name','auxInfo'{{M[1],M[14],T\"V_x\",},{M[1],M[18],T\"V_y\",},{M[1],M[19],T\"X_1\",},{M[1],M[20],T\"Y_1\",},{M[4],M[0],T\"State\",S'l','i','p'{{M1x2[87 92],M[0],}}},{M[4],M[0],T\"cov\",S'l','i','p'{{M1x2[83 86],M[0],}}},{M[8],M[0],T\"is_active_c11_SS6_Estimation2\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 7, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c11_SS6_Estimation2_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc11_SS6_Estimation2InstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SS6_Estimation2MachineNumber_,
           11,
           1,
           1,
           0,
           9,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_SS6_Estimation2MachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_SS6_Estimation2MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _SS6_Estimation2MachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,2,0,1,"X_1");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Y_1");
          _SFD_SET_DATA_PROPS(2,1,1,0,"X_GPS");
          _SFD_SET_DATA_PROPS(3,1,1,0,"Y_GPS");
          _SFD_SET_DATA_PROPS(4,1,1,0,"aX_IMU");
          _SFD_SET_DATA_PROPS(5,1,1,0,"aY_IMU");
          _SFD_SET_DATA_PROPS(6,1,1,0,"r_IMU");
          _SFD_SET_DATA_PROPS(7,2,0,1,"V_x");
          _SFD_SET_DATA_PROPS(8,2,0,1,"V_y");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1244);
        _SFD_CV_INIT_EML_IF(0,1,0,95,110,-1,146);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c11_c_sf_marshallOut,(MexInFcnForType)
          c11_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c11_c_sf_marshallOut,(MexInFcnForType)
          c11_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c11_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c11_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c11_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c11_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c11_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c11_c_sf_marshallOut,(MexInFcnForType)
          c11_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c11_c_sf_marshallOut,(MexInFcnForType)
          c11_c_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c11_X_1);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c11_Y_1);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c11_X_GPS);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c11_Y_GPS);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c11_aX_IMU);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c11_aY_IMU);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c11_r_IMU);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c11_V_x);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c11_V_y);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _SS6_Estimation2MachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "9cAESTshQvVEjJm7tcP3jD";
}

static void sf_opaque_initialize_c11_SS6_Estimation2(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc11_SS6_Estimation2InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c11_SS6_Estimation2((SFc11_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
  initialize_c11_SS6_Estimation2((SFc11_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c11_SS6_Estimation2(void *chartInstanceVar)
{
  enable_c11_SS6_Estimation2((SFc11_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c11_SS6_Estimation2(void *chartInstanceVar)
{
  disable_c11_SS6_Estimation2((SFc11_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c11_SS6_Estimation2(void *chartInstanceVar)
{
  sf_gateway_c11_SS6_Estimation2((SFc11_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c11_SS6_Estimation2(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c11_SS6_Estimation2((SFc11_SS6_Estimation2InstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c11_SS6_Estimation2(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c11_SS6_Estimation2((SFc11_SS6_Estimation2InstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c11_SS6_Estimation2(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc11_SS6_Estimation2InstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SS6_Estimation2_optimization_info();
    }

    finalize_c11_SS6_Estimation2((SFc11_SS6_Estimation2InstanceStruct*)
      chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc11_SS6_Estimation2((SFc11_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c11_SS6_Estimation2(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c11_SS6_Estimation2((SFc11_SS6_Estimation2InstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c11_SS6_Estimation2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SS6_Estimation2_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      11);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,11,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,11,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,11);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,11,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,11,4);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=4; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,11);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2218938675U));
  ssSetChecksum1(S,(2191342533U));
  ssSetChecksum2(S,(45057731U));
  ssSetChecksum3(S,(3822829524U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c11_SS6_Estimation2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c11_SS6_Estimation2(SimStruct *S)
{
  SFc11_SS6_Estimation2InstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc11_SS6_Estimation2InstanceStruct *)utMalloc(sizeof
    (SFc11_SS6_Estimation2InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc11_SS6_Estimation2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c11_SS6_Estimation2;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c11_SS6_Estimation2;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c11_SS6_Estimation2;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c11_SS6_Estimation2;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c11_SS6_Estimation2;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c11_SS6_Estimation2;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c11_SS6_Estimation2;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c11_SS6_Estimation2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c11_SS6_Estimation2;
  chartInstance->chartInfo.mdlStart = mdlStart_c11_SS6_Estimation2;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c11_SS6_Estimation2;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->checksum = SF_RUNTIME_INFO_CHECKSUM;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  crtInfo->compiledInfo = NULL;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c11_SS6_Estimation2_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c11_SS6_Estimation2(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c11_SS6_Estimation2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c11_SS6_Estimation2(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c11_SS6_Estimation2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
