/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SS6_Estimation2_sfun.h"
#include "c19_SS6_Estimation2.h"
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
static const char * c19_debug_family_names[23] = { "Q", "R", "Ts", "B", "G", "g",
  "State_p", "H", "y_meas", "K", "nargin", "nargout", "X_GPS", "Y_GPS", "aX_IMU",
  "aY_IMU", "r_IMU", "X_1", "Y_1", "V_x", "V_y", "cov", "State" };

/* Function Declarations */
static void initialize_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void initialize_params_c19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static void enable_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void disable_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_update_debugger_state_c19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static void set_sim_state_c19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance, const mxArray *c19_st);
static void finalize_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void sf_gateway_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void mdl_start_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_chartstep_c19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static void initSimStructsc19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c19_machineNumber, uint32_T
  c19_chartNumber, uint32_T c19_instanceNumber);
static const mxArray *c19_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_b_State, const char_T *c19_identifier,
  real_T c19_y[7]);
static void c19_b_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[7]);
static void c19_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_b_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_c_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_b_cov, const char_T *c19_identifier, real_T
  c19_y[49]);
static void c19_d_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[49]);
static void c19_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_c_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static real_T c19_e_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_b_V_y, const char_T *c19_identifier);
static real_T c19_f_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_d_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_g_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[14]);
static void c19_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_e_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_h_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[2]);
static void c19_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_f_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static const mxArray *c19_g_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_i_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[7]);
static void c19_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_h_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_j_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[49]);
static void c19_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_i_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static const mxArray *c19_j_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_k_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[4]);
static void c19_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static void c19_info_helper(const mxArray **c19_info);
static const mxArray *c19_emlrt_marshallOut(const char * c19_u);
static const mxArray *c19_b_emlrt_marshallOut(const uint32_T c19_u);
static void c19_b_info_helper(const mxArray **c19_info);
static void c19_c_info_helper(const mxArray **c19_info);
static void c19_d_info_helper(const mxArray **c19_info);
static void c19_eye(SFc19_SS6_Estimation2InstanceStruct *chartInstance, real_T
                    c19_I[49]);
static void c19_eml_switch_helper(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static boolean_T c19_use_refblas(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_threshold(SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static void c19_b_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_c_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_d_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_e_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_pinv(SFc19_SS6_Estimation2InstanceStruct *chartInstance, real_T
                     c19_A[4], real_T c19_X[4]);
static void c19_f_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_eml_error(SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static void c19_eml_xgesvd(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_A[4], real_T c19_U[4], real_T c19_S[2], real_T c19_V[4]);
static real_T c19_eml_xnrm2(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4]);
static void c19_below_threshold(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_realmin(SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static real_T c19_abs(SFc19_SS6_Estimation2InstanceStruct *chartInstance, real_T
                      c19_x);
static void c19_eps(SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static void c19_b_below_threshold(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_b_eml_switch_helper(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static real_T c19_eml_xdotc(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4], real_T c19_y[4]);
static void c19_b_threshold(SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static void c19_eml_xaxpy(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_a, real_T c19_y[4], real_T c19_b_y[4]);
static void c19_c_threshold(SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static void c19_check_forloop_overflow_error(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, boolean_T c19_overflow);
static real_T c19_b_eml_xdotc(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c19_n, real_T c19_x[4], int32_T c19_ix0, real_T c19_y[4], int32_T
  c19_iy0);
static void c19_b_eml_xaxpy(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c19_n, real_T c19_a, int32_T c19_ix0, real_T c19_y[4], int32_T c19_iy0,
  real_T c19_b_y[4]);
static void c19_eml_xscal(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_a, real_T c19_x[4], int32_T c19_ix0, real_T c19_b_x[4]);
static void c19_g_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_b_eml_error(SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static real_T c19_sqrt(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x);
static void c19_c_eml_error(SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static void c19_eml_xrotg(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_a, real_T c19_b, real_T *c19_b_a, real_T *c19_b_b, real_T *c19_c,
  real_T *c19_s);
static void c19_eml_xrot(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4], int32_T c19_ix0, int32_T c19_iy0, real_T c19_c, real_T c19_s,
  real_T c19_b_x[4]);
static void c19_d_threshold(SFc19_SS6_Estimation2InstanceStruct *chartInstance);
static void c19_eml_xswap(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4], int32_T c19_ix0, int32_T c19_iy0, real_T c19_b_x[4]);
static void c19_c_eml_switch_helper(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_eml_xgemm(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c19_k, real_T c19_A[4], real_T c19_B[4], real_T c19_C[4], real_T
  c19_b_C[4]);
static void c19_h_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_i_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_j_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c19_k_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static const mxArray *c19_k_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static int32_T c19_l_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static uint8_T c19_m_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_b_is_active_c19_SS6_Estimation2, const
  char_T *c19_identifier);
static uint8_T c19_n_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_c_eml_xaxpy(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_a, real_T c19_y[4]);
static void c19_d_eml_xaxpy(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c19_n, real_T c19_a, int32_T c19_ix0, real_T c19_y[4], int32_T c19_iy0);
static void c19_b_eml_xscal(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_a, real_T c19_x[4], int32_T c19_ix0);
static void c19_b_sqrt(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T *c19_x);
static void c19_b_eml_xrotg(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T *c19_a, real_T *c19_b, real_T *c19_c, real_T *c19_s);
static void c19_b_eml_xrot(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4], int32_T c19_ix0, int32_T c19_iy0, real_T c19_c, real_T c19_s);
static void c19_b_eml_xswap(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4], int32_T c19_ix0, int32_T c19_iy0);
static void c19_b_eml_xgemm(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c19_k, real_T c19_A[4], real_T c19_B[4], real_T c19_C[4]);
static void init_dsm_address_info(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  chartInstance->c19_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c19_cov_not_empty = false;
  chartInstance->c19_State_not_empty = false;
  chartInstance->c19_is_active_c19_SS6_Estimation2 = 0U;
}

static void initialize_params_c19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c19_update_debugger_state_c19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  const mxArray *c19_st;
  const mxArray *c19_y = NULL;
  real_T c19_hoistedGlobal;
  real_T c19_u;
  const mxArray *c19_b_y = NULL;
  real_T c19_b_hoistedGlobal;
  real_T c19_b_u;
  const mxArray *c19_c_y = NULL;
  real_T c19_c_hoistedGlobal;
  real_T c19_c_u;
  const mxArray *c19_d_y = NULL;
  real_T c19_d_hoistedGlobal;
  real_T c19_d_u;
  const mxArray *c19_e_y = NULL;
  int32_T c19_i0;
  real_T c19_e_u[7];
  const mxArray *c19_f_y = NULL;
  int32_T c19_i1;
  real_T c19_f_u[49];
  const mxArray *c19_g_y = NULL;
  uint8_T c19_e_hoistedGlobal;
  uint8_T c19_g_u;
  const mxArray *c19_h_y = NULL;
  c19_st = NULL;
  c19_st = NULL;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_createcellmatrix(7, 1), false);
  c19_hoistedGlobal = *chartInstance->c19_V_x;
  c19_u = c19_hoistedGlobal;
  c19_b_y = NULL;
  sf_mex_assign(&c19_b_y, sf_mex_create("y", &c19_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c19_y, 0, c19_b_y);
  c19_b_hoistedGlobal = *chartInstance->c19_V_y;
  c19_b_u = c19_b_hoistedGlobal;
  c19_c_y = NULL;
  sf_mex_assign(&c19_c_y, sf_mex_create("y", &c19_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c19_y, 1, c19_c_y);
  c19_c_hoistedGlobal = *chartInstance->c19_X_1;
  c19_c_u = c19_c_hoistedGlobal;
  c19_d_y = NULL;
  sf_mex_assign(&c19_d_y, sf_mex_create("y", &c19_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c19_y, 2, c19_d_y);
  c19_d_hoistedGlobal = *chartInstance->c19_Y_1;
  c19_d_u = c19_d_hoistedGlobal;
  c19_e_y = NULL;
  sf_mex_assign(&c19_e_y, sf_mex_create("y", &c19_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c19_y, 3, c19_e_y);
  for (c19_i0 = 0; c19_i0 < 7; c19_i0++) {
    c19_e_u[c19_i0] = chartInstance->c19_State[c19_i0];
  }

  c19_f_y = NULL;
  if (!chartInstance->c19_State_not_empty) {
    sf_mex_assign(&c19_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_f_y, sf_mex_create("y", c19_e_u, 0, 0U, 1U, 0U, 1, 7),
                  false);
  }

  sf_mex_setcell(c19_y, 4, c19_f_y);
  for (c19_i1 = 0; c19_i1 < 49; c19_i1++) {
    c19_f_u[c19_i1] = chartInstance->c19_cov[c19_i1];
  }

  c19_g_y = NULL;
  if (!chartInstance->c19_cov_not_empty) {
    sf_mex_assign(&c19_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_g_y, sf_mex_create("y", c19_f_u, 0, 0U, 1U, 0U, 2, 7, 7),
                  false);
  }

  sf_mex_setcell(c19_y, 5, c19_g_y);
  c19_e_hoistedGlobal = chartInstance->c19_is_active_c19_SS6_Estimation2;
  c19_g_u = c19_e_hoistedGlobal;
  c19_h_y = NULL;
  sf_mex_assign(&c19_h_y, sf_mex_create("y", &c19_g_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c19_y, 6, c19_h_y);
  sf_mex_assign(&c19_st, c19_y, false);
  return c19_st;
}

static void set_sim_state_c19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance, const mxArray *c19_st)
{
  const mxArray *c19_u;
  real_T c19_dv0[7];
  int32_T c19_i2;
  real_T c19_dv1[49];
  int32_T c19_i3;
  chartInstance->c19_doneDoubleBufferReInit = true;
  c19_u = sf_mex_dup(c19_st);
  *chartInstance->c19_V_x = c19_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c19_u, 0)), "V_x");
  *chartInstance->c19_V_y = c19_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c19_u, 1)), "V_y");
  *chartInstance->c19_X_1 = c19_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c19_u, 2)), "X_1");
  *chartInstance->c19_Y_1 = c19_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c19_u, 3)), "Y_1");
  c19_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c19_u, 4)),
                       "State", c19_dv0);
  for (c19_i2 = 0; c19_i2 < 7; c19_i2++) {
    chartInstance->c19_State[c19_i2] = c19_dv0[c19_i2];
  }

  c19_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c19_u, 5)),
    "cov", c19_dv1);
  for (c19_i3 = 0; c19_i3 < 49; c19_i3++) {
    chartInstance->c19_cov[c19_i3] = c19_dv1[c19_i3];
  }

  chartInstance->c19_is_active_c19_SS6_Estimation2 = c19_m_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c19_u, 6)),
     "is_active_c19_SS6_Estimation2");
  sf_mex_destroy(&c19_u);
  c19_update_debugger_state_c19_SS6_Estimation2(chartInstance);
  sf_mex_destroy(&c19_st);
}

static void finalize_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 18U, chartInstance->c19_sfEvent);
  chartInstance->c19_sfEvent = CALL_EVENT;
  c19_chartstep_c19_SS6_Estimation2(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SS6_Estimation2MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_X_1, 0U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_Y_1, 1U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_X_GPS, 2U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_Y_GPS, 3U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_aX_IMU, 4U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_aY_IMU, 5U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_r_IMU, 6U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_V_x, 7U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_V_y, 8U);
}

static void mdl_start_c19_SS6_Estimation2(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_chartstep_c19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  real_T c19_hoistedGlobal;
  real_T c19_b_hoistedGlobal;
  real_T c19_c_hoistedGlobal;
  real_T c19_d_hoistedGlobal;
  real_T c19_e_hoistedGlobal;
  real_T c19_b_X_GPS;
  real_T c19_b_Y_GPS;
  real_T c19_b_aX_IMU;
  real_T c19_b_aY_IMU;
  real_T c19_b_r_IMU;
  uint32_T c19_debug_family_var_map[23];
  real_T c19_Q[49];
  real_T c19_R[4];
  real_T c19_Ts;
  real_T c19_B[21];
  real_T c19_G[49];
  real_T c19_g[7];
  real_T c19_State_p[7];
  real_T c19_H[14];
  real_T c19_y_meas[2];
  real_T c19_K[14];
  real_T c19_nargin = 5.0;
  real_T c19_nargout = 4.0;
  real_T c19_b_X_1;
  real_T c19_b_Y_1;
  real_T c19_b_V_x;
  real_T c19_b_V_y;
  int32_T c19_i4;
  int32_T c19_i5;
  int32_T c19_i6;
  static real_T c19_dv2[49] = { 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001 };

  int32_T c19_i7;
  int32_T c19_i8;
  int32_T c19_i9;
  int32_T c19_i10;
  static real_T c19_dv3[4] = { 0.0001, 0.0, 0.0, 0.0001 };

  int32_T c19_i11;
  static real_T c19_a[21] = { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0 };

  int32_T c19_i12;
  real_T c19_f_hoistedGlobal[7];
  real_T c19_x;
  real_T c19_b_x;
  int32_T c19_i13;
  real_T c19_c_x;
  real_T c19_d_x;
  int32_T c19_i14;
  int32_T c19_i15;
  real_T c19_g_hoistedGlobal[7];
  real_T c19_e_x;
  real_T c19_f_x;
  int32_T c19_i16;
  int32_T c19_i17;
  real_T c19_h_hoistedGlobal[7];
  real_T c19_g_x;
  real_T c19_h_x;
  int32_T c19_i18;
  real_T c19_i_x;
  real_T c19_j_x;
  int32_T c19_i19;
  real_T c19_k_x;
  real_T c19_l_x;
  int32_T c19_i20;
  int32_T c19_i21;
  real_T c19_i_hoistedGlobal[7];
  real_T c19_m_x;
  real_T c19_n_x;
  int32_T c19_i22;
  int32_T c19_i23;
  real_T c19_j_hoistedGlobal[7];
  real_T c19_o_x;
  real_T c19_p_x;
  int32_T c19_i24;
  int32_T c19_i25;
  static real_T c19_dv4[7] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T c19_i26;
  int32_T c19_i27;
  static real_T c19_dv5[7] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T c19_i28;
  int32_T c19_i29;
  static real_T c19_dv6[7] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T c19_i30;
  int32_T c19_i31;
  int32_T c19_i32;
  int32_T c19_i33;
  int32_T c19_i34;
  real_T c19_q_x;
  real_T c19_r_x;
  int32_T c19_i35;
  int32_T c19_i36;
  real_T c19_k_hoistedGlobal[7];
  real_T c19_s_x;
  real_T c19_t_x;
  int32_T c19_i37;
  int32_T c19_i38;
  real_T c19_l_hoistedGlobal[7];
  real_T c19_u_x;
  real_T c19_v_x;
  int32_T c19_i39;
  int32_T c19_i40;
  real_T c19_m_hoistedGlobal[7];
  real_T c19_w_x;
  real_T c19_x_x;
  real_T c19_b[3];
  int32_T c19_i41;
  int32_T c19_i42;
  int32_T c19_i43;
  int32_T c19_i44;
  int32_T c19_i45;
  real_T c19_n_hoistedGlobal[49];
  int32_T c19_i46;
  real_T c19_b_a[49];
  int32_T c19_i47;
  int32_T c19_i48;
  int32_T c19_i49;
  real_T c19_y[49];
  int32_T c19_i50;
  int32_T c19_i51;
  int32_T c19_i52;
  int32_T c19_i53;
  int32_T c19_i54;
  int32_T c19_i55;
  int32_T c19_i56;
  int32_T c19_i57;
  int32_T c19_i58;
  int32_T c19_i59;
  int32_T c19_i60;
  int32_T c19_i61;
  int32_T c19_i62;
  static real_T c19_c_a[14] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 1.0 };

  int32_T c19_i63;
  int32_T c19_i64;
  int32_T c19_i65;
  int32_T c19_i66;
  real_T c19_b_y[14];
  int32_T c19_i67;
  int32_T c19_i68;
  static real_T c19_b_b[14] = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0 };

  int32_T c19_i69;
  int32_T c19_i70;
  int32_T c19_i71;
  int32_T c19_i72;
  int32_T c19_i73;
  real_T c19_c_y[14];
  int32_T c19_i74;
  int32_T c19_i75;
  int32_T c19_i76;
  int32_T c19_i77;
  int32_T c19_i78;
  int32_T c19_i79;
  real_T c19_d_y[4];
  int32_T c19_i80;
  int32_T c19_i81;
  int32_T c19_i82;
  real_T c19_e_y[4];
  int32_T c19_i83;
  int32_T c19_i84;
  int32_T c19_i85;
  real_T c19_C[14];
  int32_T c19_i86;
  int32_T c19_i87;
  int32_T c19_i88;
  int32_T c19_i89;
  int32_T c19_i90;
  int32_T c19_i91;
  int32_T c19_i92;
  int32_T c19_i93;
  int32_T c19_i94;
  int32_T c19_i95;
  int32_T c19_i96;
  real_T c19_f_y[2];
  int32_T c19_i97;
  int32_T c19_i98;
  int32_T c19_i99;
  int32_T c19_i100;
  int32_T c19_i101;
  int32_T c19_i102;
  int32_T c19_i103;
  int32_T c19_i104;
  int32_T c19_i105;
  int32_T c19_i106;
  int32_T c19_i107;
  int32_T c19_i108;
  int32_T c19_i109;
  int32_T c19_i110;
  int32_T c19_i111;
  int32_T c19_i112;
  int32_T c19_i113;
  int32_T c19_i114;
  int32_T c19_i115;
  int32_T c19_i116;
  int32_T c19_i117;
  int32_T c19_i118;
  int32_T c19_i119;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 18U, chartInstance->c19_sfEvent);
  c19_hoistedGlobal = *chartInstance->c19_X_GPS;
  c19_b_hoistedGlobal = *chartInstance->c19_Y_GPS;
  c19_c_hoistedGlobal = *chartInstance->c19_aX_IMU;
  c19_d_hoistedGlobal = *chartInstance->c19_aY_IMU;
  c19_e_hoistedGlobal = *chartInstance->c19_r_IMU;
  c19_b_X_GPS = c19_hoistedGlobal;
  c19_b_Y_GPS = c19_b_hoistedGlobal;
  c19_b_aX_IMU = c19_c_hoistedGlobal;
  c19_b_aY_IMU = c19_d_hoistedGlobal;
  c19_b_r_IMU = c19_e_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 23U, 23U, c19_debug_family_names,
    c19_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_Q, 0U, c19_h_sf_marshallOut,
    c19_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_R, 1U, c19_j_sf_marshallOut,
    c19_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c19_Ts, 2U, c19_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c19_B, 3U, c19_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_G, 4U, c19_h_sf_marshallOut,
    c19_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_g, 5U, c19_g_sf_marshallOut,
    c19_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_State_p, 6U, c19_g_sf_marshallOut,
    c19_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c19_H, 7U, c19_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_y_meas, 8U, c19_e_sf_marshallOut,
    c19_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_K, 9U, c19_d_sf_marshallOut,
    c19_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_nargin, 10U, c19_c_sf_marshallOut,
    c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_nargout, 11U, c19_c_sf_marshallOut,
    c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c19_b_X_GPS, 12U, c19_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c19_b_Y_GPS, 13U, c19_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c19_b_aX_IMU, 14U, c19_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c19_b_aY_IMU, 15U, c19_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c19_b_r_IMU, 16U, c19_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_b_X_1, 17U, c19_c_sf_marshallOut,
    c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_b_Y_1, 18U, c19_c_sf_marshallOut,
    c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_b_V_x, 19U, c19_c_sf_marshallOut,
    c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_b_V_y, 20U, c19_c_sf_marshallOut,
    c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c19_cov, 21U,
    c19_b_sf_marshallOut, c19_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c19_State, 22U,
    c19_sf_marshallOut, c19_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 3);
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 5);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c19_cov_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 6);
    for (c19_i4 = 0; c19_i4 < 7; c19_i4++) {
      chartInstance->c19_State[c19_i4] = 0.0;
    }

    chartInstance->c19_State_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 7);
    for (c19_i5 = 0; c19_i5 < 49; c19_i5++) {
      chartInstance->c19_cov[c19_i5] = 0.0;
    }

    chartInstance->c19_cov_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 11);
  for (c19_i6 = 0; c19_i6 < 49; c19_i6++) {
    c19_Q[c19_i6] = c19_dv2[c19_i6];
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 12);
  c19_i7 = 0;
  for (c19_i8 = 0; c19_i8 < 4; c19_i8++) {
    for (c19_i9 = 0; c19_i9 < 4; c19_i9++) {
      c19_Q[(c19_i9 + c19_i7) + 24] = 0.0;
    }

    c19_i7 += 7;
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 14);
  for (c19_i10 = 0; c19_i10 < 4; c19_i10++) {
    c19_R[c19_i10] = c19_dv3[c19_i10];
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 16);
  c19_Ts = 0.01;
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 18);
  for (c19_i11 = 0; c19_i11 < 21; c19_i11++) {
    c19_B[c19_i11] = c19_a[c19_i11];
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 27);
  for (c19_i12 = 0; c19_i12 < 7; c19_i12++) {
    c19_f_hoistedGlobal[c19_i12] = chartInstance->c19_State[c19_i12];
  }

  c19_x = c19_f_hoistedGlobal[2];
  c19_b_x = c19_x;
  c19_b_x = muDoubleScalarCos(c19_b_x);
  for (c19_i13 = 0; c19_i13 < 7; c19_i13++) {
    c19_f_hoistedGlobal[c19_i13] = chartInstance->c19_State[c19_i13];
  }

  c19_c_x = c19_f_hoistedGlobal[2];
  c19_d_x = c19_c_x;
  c19_d_x = muDoubleScalarSin(c19_d_x);
  for (c19_i14 = 0; c19_i14 < 7; c19_i14++) {
    c19_f_hoistedGlobal[c19_i14] = chartInstance->c19_State[c19_i14];
  }

  for (c19_i15 = 0; c19_i15 < 7; c19_i15++) {
    c19_g_hoistedGlobal[c19_i15] = chartInstance->c19_State[c19_i15];
  }

  c19_e_x = c19_g_hoistedGlobal[2];
  c19_f_x = c19_e_x;
  c19_f_x = muDoubleScalarSin(c19_f_x);
  for (c19_i16 = 0; c19_i16 < 7; c19_i16++) {
    c19_g_hoistedGlobal[c19_i16] = chartInstance->c19_State[c19_i16];
  }

  for (c19_i17 = 0; c19_i17 < 7; c19_i17++) {
    c19_h_hoistedGlobal[c19_i17] = chartInstance->c19_State[c19_i17];
  }

  c19_g_x = c19_h_hoistedGlobal[2];
  c19_h_x = c19_g_x;
  c19_h_x = muDoubleScalarCos(c19_h_x);
  for (c19_i18 = 0; c19_i18 < 7; c19_i18++) {
    c19_h_hoistedGlobal[c19_i18] = chartInstance->c19_State[c19_i18];
  }

  c19_i_x = c19_h_hoistedGlobal[2];
  c19_j_x = c19_i_x;
  c19_j_x = muDoubleScalarSin(c19_j_x);
  for (c19_i19 = 0; c19_i19 < 7; c19_i19++) {
    c19_h_hoistedGlobal[c19_i19] = chartInstance->c19_State[c19_i19];
  }

  c19_k_x = c19_h_hoistedGlobal[2];
  c19_l_x = c19_k_x;
  c19_l_x = muDoubleScalarCos(c19_l_x);
  for (c19_i20 = 0; c19_i20 < 7; c19_i20++) {
    c19_h_hoistedGlobal[c19_i20] = chartInstance->c19_State[c19_i20];
  }

  for (c19_i21 = 0; c19_i21 < 7; c19_i21++) {
    c19_i_hoistedGlobal[c19_i21] = chartInstance->c19_State[c19_i21];
  }

  c19_m_x = c19_i_hoistedGlobal[2];
  c19_n_x = c19_m_x;
  c19_n_x = muDoubleScalarCos(c19_n_x);
  for (c19_i22 = 0; c19_i22 < 7; c19_i22++) {
    c19_i_hoistedGlobal[c19_i22] = chartInstance->c19_State[c19_i22];
  }

  for (c19_i23 = 0; c19_i23 < 7; c19_i23++) {
    c19_j_hoistedGlobal[c19_i23] = chartInstance->c19_State[c19_i23];
  }

  c19_o_x = c19_j_hoistedGlobal[2];
  c19_p_x = c19_o_x;
  c19_p_x = muDoubleScalarSin(c19_p_x);
  c19_i24 = 0;
  for (c19_i25 = 0; c19_i25 < 7; c19_i25++) {
    c19_G[c19_i24] = c19_dv4[c19_i25];
    c19_i24 += 7;
  }

  c19_i26 = 0;
  for (c19_i27 = 0; c19_i27 < 7; c19_i27++) {
    c19_G[c19_i26 + 1] = c19_dv5[c19_i27];
    c19_i26 += 7;
  }

  c19_i28 = 0;
  for (c19_i29 = 0; c19_i29 < 7; c19_i29++) {
    c19_G[c19_i28 + 2] = c19_dv6[c19_i29];
    c19_i28 += 7;
  }

  c19_G[3] = c19_b_x;
  c19_G[10] = -c19_d_x;
  c19_G[17] = -c19_f_hoistedGlobal[0] * c19_f_x - c19_g_hoistedGlobal[1] *
    c19_h_x;
  c19_G[24] = 0.0;
  c19_G[31] = 0.0;
  c19_G[38] = 0.0;
  c19_G[45] = 0.0;
  c19_G[4] = c19_j_x;
  c19_G[11] = c19_l_x;
  c19_G[18] = c19_h_hoistedGlobal[0] * c19_n_x - c19_i_hoistedGlobal[1] *
    c19_p_x;
  c19_G[25] = 0.0;
  c19_G[32] = 0.0;
  c19_G[39] = 0.0;
  c19_G[46] = 0.0;
  c19_G[5] = 0.0;
  c19_G[12] = 0.0;
  c19_G[19] = 0.0;
  c19_G[26] = c19_Ts;
  c19_G[33] = 0.0;
  c19_G[40] = 1.0;
  c19_G[47] = 0.0;
  c19_G[6] = 0.0;
  c19_G[13] = 0.0;
  c19_G[20] = 0.0;
  c19_G[27] = 0.0;
  c19_G[34] = c19_Ts;
  c19_G[41] = 0.0;
  c19_G[48] = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 36);
  for (c19_i30 = 0; c19_i30 < 7; c19_i30++) {
    c19_f_hoistedGlobal[c19_i30] = chartInstance->c19_State[c19_i30];
  }

  for (c19_i31 = 0; c19_i31 < 7; c19_i31++) {
    c19_g_hoistedGlobal[c19_i31] = chartInstance->c19_State[c19_i31];
  }

  for (c19_i32 = 0; c19_i32 < 7; c19_i32++) {
    c19_h_hoistedGlobal[c19_i32] = chartInstance->c19_State[c19_i32];
  }

  for (c19_i33 = 0; c19_i33 < 7; c19_i33++) {
    c19_i_hoistedGlobal[c19_i33] = chartInstance->c19_State[c19_i33];
  }

  for (c19_i34 = 0; c19_i34 < 7; c19_i34++) {
    c19_j_hoistedGlobal[c19_i34] = chartInstance->c19_State[c19_i34];
  }

  c19_q_x = c19_j_hoistedGlobal[2];
  c19_r_x = c19_q_x;
  c19_r_x = muDoubleScalarCos(c19_r_x);
  for (c19_i35 = 0; c19_i35 < 7; c19_i35++) {
    c19_j_hoistedGlobal[c19_i35] = chartInstance->c19_State[c19_i35];
  }

  for (c19_i36 = 0; c19_i36 < 7; c19_i36++) {
    c19_k_hoistedGlobal[c19_i36] = chartInstance->c19_State[c19_i36];
  }

  c19_s_x = c19_k_hoistedGlobal[2];
  c19_t_x = c19_s_x;
  c19_t_x = muDoubleScalarSin(c19_t_x);
  for (c19_i37 = 0; c19_i37 < 7; c19_i37++) {
    c19_k_hoistedGlobal[c19_i37] = chartInstance->c19_State[c19_i37];
  }

  for (c19_i38 = 0; c19_i38 < 7; c19_i38++) {
    c19_l_hoistedGlobal[c19_i38] = chartInstance->c19_State[c19_i38];
  }

  c19_u_x = c19_l_hoistedGlobal[2];
  c19_v_x = c19_u_x;
  c19_v_x = muDoubleScalarSin(c19_v_x);
  for (c19_i39 = 0; c19_i39 < 7; c19_i39++) {
    c19_l_hoistedGlobal[c19_i39] = chartInstance->c19_State[c19_i39];
  }

  for (c19_i40 = 0; c19_i40 < 7; c19_i40++) {
    c19_m_hoistedGlobal[c19_i40] = chartInstance->c19_State[c19_i40];
  }

  c19_w_x = c19_m_hoistedGlobal[2];
  c19_x_x = c19_w_x;
  c19_x_x = muDoubleScalarCos(c19_x_x);
  c19_g[0] = c19_f_hoistedGlobal[0];
  c19_g[1] = c19_g_hoistedGlobal[1];
  c19_g[2] = c19_h_hoistedGlobal[2];
  c19_g[3] = c19_i_hoistedGlobal[0] * c19_r_x - c19_j_hoistedGlobal[1] * c19_t_x;
  c19_g[4] = c19_k_hoistedGlobal[0] * c19_v_x + c19_l_hoistedGlobal[1] * c19_x_x;
  c19_g[5] = 0.01 * chartInstance->c19_State[3] + chartInstance->c19_State[5];
  c19_g[6] = 0.01 * chartInstance->c19_State[4] + chartInstance->c19_State[6];
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 45);
  c19_b[0] = c19_b_aX_IMU;
  c19_b[1] = c19_b_aY_IMU;
  c19_b[2] = c19_b_r_IMU;
  c19_eml_scalar_eg(chartInstance);
  c19_eml_scalar_eg(chartInstance);
  c19_threshold(chartInstance);
  for (c19_i41 = 0; c19_i41 < 7; c19_i41++) {
    c19_f_hoistedGlobal[c19_i41] = 0.0;
    c19_i42 = 0;
    for (c19_i43 = 0; c19_i43 < 3; c19_i43++) {
      c19_f_hoistedGlobal[c19_i41] += c19_a[c19_i42 + c19_i41] * c19_b[c19_i43];
      c19_i42 += 7;
    }
  }

  for (c19_i44 = 0; c19_i44 < 7; c19_i44++) {
    c19_State_p[c19_i44] = c19_g[c19_i44] + c19_f_hoistedGlobal[c19_i44];
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 48);
  for (c19_i45 = 0; c19_i45 < 49; c19_i45++) {
    c19_n_hoistedGlobal[c19_i45] = chartInstance->c19_cov[c19_i45];
  }

  for (c19_i46 = 0; c19_i46 < 49; c19_i46++) {
    c19_b_a[c19_i46] = c19_G[c19_i46];
  }

  c19_b_eml_scalar_eg(chartInstance);
  c19_b_eml_scalar_eg(chartInstance);
  c19_threshold(chartInstance);
  for (c19_i47 = 0; c19_i47 < 7; c19_i47++) {
    c19_i48 = 0;
    for (c19_i49 = 0; c19_i49 < 7; c19_i49++) {
      c19_y[c19_i48 + c19_i47] = 0.0;
      c19_i50 = 0;
      for (c19_i51 = 0; c19_i51 < 7; c19_i51++) {
        c19_y[c19_i48 + c19_i47] += c19_b_a[c19_i50 + c19_i47] *
          c19_n_hoistedGlobal[c19_i51 + c19_i48];
        c19_i50 += 7;
      }

      c19_i48 += 7;
    }
  }

  c19_i52 = 0;
  for (c19_i53 = 0; c19_i53 < 7; c19_i53++) {
    c19_i54 = 0;
    for (c19_i55 = 0; c19_i55 < 7; c19_i55++) {
      c19_n_hoistedGlobal[c19_i55 + c19_i52] = c19_G[c19_i54 + c19_i53];
      c19_i54 += 7;
    }

    c19_i52 += 7;
  }

  c19_b_eml_scalar_eg(chartInstance);
  c19_b_eml_scalar_eg(chartInstance);
  c19_threshold(chartInstance);
  for (c19_i56 = 0; c19_i56 < 7; c19_i56++) {
    c19_i57 = 0;
    for (c19_i58 = 0; c19_i58 < 7; c19_i58++) {
      c19_b_a[c19_i57 + c19_i56] = 0.0;
      c19_i59 = 0;
      for (c19_i60 = 0; c19_i60 < 7; c19_i60++) {
        c19_b_a[c19_i57 + c19_i56] += c19_y[c19_i59 + c19_i56] *
          c19_n_hoistedGlobal[c19_i60 + c19_i57];
        c19_i59 += 7;
      }

      c19_i57 += 7;
    }
  }

  for (c19_i61 = 0; c19_i61 < 49; c19_i61++) {
    chartInstance->c19_cov[c19_i61] = c19_b_a[c19_i61] + c19_Q[c19_i61];
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 50);
  for (c19_i62 = 0; c19_i62 < 14; c19_i62++) {
    c19_H[c19_i62] = c19_c_a[c19_i62];
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 54);
  c19_y_meas[0] = c19_b_X_GPS;
  c19_y_meas[1] = c19_b_Y_GPS;
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 57);
  for (c19_i63 = 0; c19_i63 < 49; c19_i63++) {
    c19_n_hoistedGlobal[c19_i63] = chartInstance->c19_cov[c19_i63];
  }

  c19_c_eml_scalar_eg(chartInstance);
  c19_c_eml_scalar_eg(chartInstance);
  c19_threshold(chartInstance);
  for (c19_i64 = 0; c19_i64 < 7; c19_i64++) {
    c19_i65 = 0;
    for (c19_i66 = 0; c19_i66 < 2; c19_i66++) {
      c19_b_y[c19_i65 + c19_i64] = 0.0;
      c19_i67 = 0;
      for (c19_i68 = 0; c19_i68 < 7; c19_i68++) {
        c19_b_y[c19_i65 + c19_i64] += c19_n_hoistedGlobal[c19_i67 + c19_i64] *
          c19_b_b[c19_i68 + c19_i65];
        c19_i67 += 7;
      }

      c19_i65 += 7;
    }
  }

  for (c19_i69 = 0; c19_i69 < 49; c19_i69++) {
    c19_n_hoistedGlobal[c19_i69] = chartInstance->c19_cov[c19_i69];
  }

  c19_d_eml_scalar_eg(chartInstance);
  c19_d_eml_scalar_eg(chartInstance);
  c19_threshold(chartInstance);
  for (c19_i70 = 0; c19_i70 < 2; c19_i70++) {
    c19_i71 = 0;
    c19_i72 = 0;
    for (c19_i73 = 0; c19_i73 < 7; c19_i73++) {
      c19_c_y[c19_i71 + c19_i70] = 0.0;
      c19_i74 = 0;
      for (c19_i75 = 0; c19_i75 < 7; c19_i75++) {
        c19_c_y[c19_i71 + c19_i70] += c19_c_a[c19_i74 + c19_i70] *
          c19_n_hoistedGlobal[c19_i75 + c19_i72];
        c19_i74 += 2;
      }

      c19_i71 += 2;
      c19_i72 += 7;
    }
  }

  c19_e_eml_scalar_eg(chartInstance);
  c19_e_eml_scalar_eg(chartInstance);
  c19_threshold(chartInstance);
  for (c19_i76 = 0; c19_i76 < 2; c19_i76++) {
    c19_i77 = 0;
    c19_i78 = 0;
    for (c19_i79 = 0; c19_i79 < 2; c19_i79++) {
      c19_d_y[c19_i77 + c19_i76] = 0.0;
      c19_i80 = 0;
      for (c19_i81 = 0; c19_i81 < 7; c19_i81++) {
        c19_d_y[c19_i77 + c19_i76] += c19_c_y[c19_i80 + c19_i76] *
          c19_b_b[c19_i81 + c19_i78];
        c19_i80 += 2;
      }

      c19_i77 += 2;
      c19_i78 += 7;
    }
  }

  for (c19_i82 = 0; c19_i82 < 4; c19_i82++) {
    c19_e_y[c19_i82] = c19_d_y[c19_i82] + c19_R[c19_i82];
  }

  c19_pinv(chartInstance, c19_e_y, c19_d_y);
  c19_h_eml_scalar_eg(chartInstance);
  c19_h_eml_scalar_eg(chartInstance);
  for (c19_i83 = 0; c19_i83 < 14; c19_i83++) {
    c19_K[c19_i83] = 0.0;
  }

  for (c19_i84 = 0; c19_i84 < 14; c19_i84++) {
    c19_K[c19_i84] = 0.0;
  }

  for (c19_i85 = 0; c19_i85 < 14; c19_i85++) {
    c19_C[c19_i85] = c19_K[c19_i85];
  }

  for (c19_i86 = 0; c19_i86 < 14; c19_i86++) {
    c19_K[c19_i86] = c19_C[c19_i86];
  }

  c19_threshold(chartInstance);
  for (c19_i87 = 0; c19_i87 < 14; c19_i87++) {
    c19_C[c19_i87] = c19_K[c19_i87];
  }

  for (c19_i88 = 0; c19_i88 < 14; c19_i88++) {
    c19_K[c19_i88] = c19_C[c19_i88];
  }

  for (c19_i89 = 0; c19_i89 < 7; c19_i89++) {
    c19_i90 = 0;
    c19_i91 = 0;
    for (c19_i92 = 0; c19_i92 < 2; c19_i92++) {
      c19_K[c19_i90 + c19_i89] = 0.0;
      c19_i93 = 0;
      for (c19_i94 = 0; c19_i94 < 2; c19_i94++) {
        c19_K[c19_i90 + c19_i89] += c19_b_y[c19_i93 + c19_i89] * c19_d_y[c19_i94
          + c19_i91];
        c19_i93 += 7;
      }

      c19_i90 += 7;
      c19_i91 += 2;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 58);
  for (c19_i95 = 0; c19_i95 < 7; c19_i95++) {
    c19_f_hoistedGlobal[c19_i95] = c19_State_p[c19_i95];
  }

  c19_i_eml_scalar_eg(chartInstance);
  c19_i_eml_scalar_eg(chartInstance);
  c19_threshold(chartInstance);
  for (c19_i96 = 0; c19_i96 < 2; c19_i96++) {
    c19_f_y[c19_i96] = 0.0;
    c19_i97 = 0;
    for (c19_i98 = 0; c19_i98 < 7; c19_i98++) {
      c19_f_y[c19_i96] += c19_c_a[c19_i97 + c19_i96] *
        c19_f_hoistedGlobal[c19_i98];
      c19_i97 += 2;
    }
  }

  for (c19_i99 = 0; c19_i99 < 14; c19_i99++) {
    c19_b_y[c19_i99] = c19_K[c19_i99];
  }

  for (c19_i100 = 0; c19_i100 < 2; c19_i100++) {
    c19_f_y[c19_i100] = c19_y_meas[c19_i100] - c19_f_y[c19_i100];
  }

  c19_j_eml_scalar_eg(chartInstance);
  c19_j_eml_scalar_eg(chartInstance);
  c19_threshold(chartInstance);
  for (c19_i101 = 0; c19_i101 < 7; c19_i101++) {
    c19_f_hoistedGlobal[c19_i101] = 0.0;
    c19_i102 = 0;
    for (c19_i103 = 0; c19_i103 < 2; c19_i103++) {
      c19_f_hoistedGlobal[c19_i101] += c19_b_y[c19_i102 + c19_i101] *
        c19_f_y[c19_i103];
      c19_i102 += 7;
    }
  }

  for (c19_i104 = 0; c19_i104 < 7; c19_i104++) {
    chartInstance->c19_State[c19_i104] = c19_State_p[c19_i104] +
      c19_f_hoistedGlobal[c19_i104];
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 59);
  for (c19_i105 = 0; c19_i105 < 14; c19_i105++) {
    c19_b_y[c19_i105] = c19_K[c19_i105];
  }

  c19_k_eml_scalar_eg(chartInstance);
  c19_k_eml_scalar_eg(chartInstance);
  c19_threshold(chartInstance);
  for (c19_i106 = 0; c19_i106 < 7; c19_i106++) {
    c19_i107 = 0;
    c19_i108 = 0;
    for (c19_i109 = 0; c19_i109 < 7; c19_i109++) {
      c19_y[c19_i107 + c19_i106] = 0.0;
      c19_i110 = 0;
      for (c19_i111 = 0; c19_i111 < 2; c19_i111++) {
        c19_y[c19_i107 + c19_i106] += c19_b_y[c19_i110 + c19_i106] *
          c19_c_a[c19_i111 + c19_i108];
        c19_i110 += 7;
      }

      c19_i107 += 7;
      c19_i108 += 2;
    }
  }

  for (c19_i112 = 0; c19_i112 < 49; c19_i112++) {
    c19_n_hoistedGlobal[c19_i112] = chartInstance->c19_cov[c19_i112];
  }

  c19_eye(chartInstance, c19_b_a);
  for (c19_i113 = 0; c19_i113 < 49; c19_i113++) {
    c19_b_a[c19_i113] -= c19_y[c19_i113];
  }

  c19_b_eml_scalar_eg(chartInstance);
  c19_b_eml_scalar_eg(chartInstance);
  c19_threshold(chartInstance);
  for (c19_i114 = 0; c19_i114 < 7; c19_i114++) {
    c19_i115 = 0;
    for (c19_i116 = 0; c19_i116 < 7; c19_i116++) {
      c19_y[c19_i115 + c19_i114] = 0.0;
      c19_i117 = 0;
      for (c19_i118 = 0; c19_i118 < 7; c19_i118++) {
        c19_y[c19_i115 + c19_i114] += c19_b_a[c19_i117 + c19_i114] *
          c19_n_hoistedGlobal[c19_i118 + c19_i115];
        c19_i117 += 7;
      }

      c19_i115 += 7;
    }
  }

  for (c19_i119 = 0; c19_i119 < 49; c19_i119++) {
    chartInstance->c19_cov[c19_i119] = c19_y[c19_i119];
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 60);
  c19_b_X_1 = chartInstance->c19_State[5];
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 61);
  c19_b_Y_1 = chartInstance->c19_State[6];
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 62);
  c19_b_V_x = chartInstance->c19_State[0];
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 63);
  c19_b_V_y = chartInstance->c19_State[1];
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, -63);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c19_X_1 = c19_b_X_1;
  *chartInstance->c19_Y_1 = c19_b_Y_1;
  *chartInstance->c19_V_x = c19_b_V_x;
  *chartInstance->c19_V_y = c19_b_V_y;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 18U, chartInstance->c19_sfEvent);
}

static void initSimStructsc19_SS6_Estimation2
  (SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c19_machineNumber, uint32_T
  c19_chartNumber, uint32_T c19_instanceNumber)
{
  (void)c19_machineNumber;
  (void)c19_chartNumber;
  (void)c19_instanceNumber;
}

static const mxArray *c19_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i120;
  real_T c19_b_inData[7];
  int32_T c19_i121;
  real_T c19_u[7];
  const mxArray *c19_y = NULL;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  for (c19_i120 = 0; c19_i120 < 7; c19_i120++) {
    c19_b_inData[c19_i120] = (*(real_T (*)[7])c19_inData)[c19_i120];
  }

  for (c19_i121 = 0; c19_i121 < 7; c19_i121++) {
    c19_u[c19_i121] = c19_b_inData[c19_i121];
  }

  c19_y = NULL;
  if (!chartInstance->c19_State_not_empty) {
    sf_mex_assign(&c19_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 1, 7), false);
  }

  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static void c19_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_b_State, const char_T *c19_identifier,
  real_T c19_y[7])
{
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_State), &c19_thisId,
    c19_y);
  sf_mex_destroy(&c19_b_State);
}

static void c19_b_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[7])
{
  real_T c19_dv7[7];
  int32_T c19_i122;
  if (mxIsEmpty(c19_u)) {
    chartInstance->c19_State_not_empty = false;
  } else {
    chartInstance->c19_State_not_empty = true;
    sf_mex_import(c19_parentId, sf_mex_dup(c19_u), c19_dv7, 1, 0, 0U, 1, 0U, 1,
                  7);
    for (c19_i122 = 0; c19_i122 < 7; c19_i122++) {
      c19_y[c19_i122] = c19_dv7[c19_i122];
    }
  }

  sf_mex_destroy(&c19_u);
}

static void c19_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_State;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y[7];
  int32_T c19_i123;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_b_State = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_State), &c19_thisId,
    c19_y);
  sf_mex_destroy(&c19_b_State);
  for (c19_i123 = 0; c19_i123 < 7; c19_i123++) {
    (*(real_T (*)[7])c19_outData)[c19_i123] = c19_y[c19_i123];
  }

  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_b_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i124;
  int32_T c19_i125;
  int32_T c19_i126;
  real_T c19_b_inData[49];
  int32_T c19_i127;
  int32_T c19_i128;
  int32_T c19_i129;
  real_T c19_u[49];
  const mxArray *c19_y = NULL;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_i124 = 0;
  for (c19_i125 = 0; c19_i125 < 7; c19_i125++) {
    for (c19_i126 = 0; c19_i126 < 7; c19_i126++) {
      c19_b_inData[c19_i126 + c19_i124] = (*(real_T (*)[49])c19_inData)[c19_i126
        + c19_i124];
    }

    c19_i124 += 7;
  }

  c19_i127 = 0;
  for (c19_i128 = 0; c19_i128 < 7; c19_i128++) {
    for (c19_i129 = 0; c19_i129 < 7; c19_i129++) {
      c19_u[c19_i129 + c19_i127] = c19_b_inData[c19_i129 + c19_i127];
    }

    c19_i127 += 7;
  }

  c19_y = NULL;
  if (!chartInstance->c19_cov_not_empty) {
    sf_mex_assign(&c19_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 2, 7, 7),
                  false);
  }

  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static void c19_c_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_b_cov, const char_T *c19_identifier, real_T
  c19_y[49])
{
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_cov), &c19_thisId,
    c19_y);
  sf_mex_destroy(&c19_b_cov);
}

static void c19_d_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[49])
{
  real_T c19_dv8[49];
  int32_T c19_i130;
  if (mxIsEmpty(c19_u)) {
    chartInstance->c19_cov_not_empty = false;
  } else {
    chartInstance->c19_cov_not_empty = true;
    sf_mex_import(c19_parentId, sf_mex_dup(c19_u), c19_dv8, 1, 0, 0U, 1, 0U, 2,
                  7, 7);
    for (c19_i130 = 0; c19_i130 < 49; c19_i130++) {
      c19_y[c19_i130] = c19_dv8[c19_i130];
    }
  }

  sf_mex_destroy(&c19_u);
}

static void c19_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_cov;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y[49];
  int32_T c19_i131;
  int32_T c19_i132;
  int32_T c19_i133;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_b_cov = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_cov), &c19_thisId,
    c19_y);
  sf_mex_destroy(&c19_b_cov);
  c19_i131 = 0;
  for (c19_i132 = 0; c19_i132 < 7; c19_i132++) {
    for (c19_i133 = 0; c19_i133 < 7; c19_i133++) {
      (*(real_T (*)[49])c19_outData)[c19_i133 + c19_i131] = c19_y[c19_i133 +
        c19_i131];
    }

    c19_i131 += 7;
  }

  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_c_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  real_T c19_u;
  const mxArray *c19_y = NULL;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(real_T *)c19_inData;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static real_T c19_e_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_b_V_y, const char_T *c19_identifier)
{
  real_T c19_y;
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_V_y),
    &c19_thisId);
  sf_mex_destroy(&c19_b_V_y);
  return c19_y;
}

static real_T c19_f_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  real_T c19_y;
  real_T c19_d0;
  (void)chartInstance;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_d0, 1, 0, 0U, 0, 0U, 0);
  c19_y = c19_d0;
  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_V_y;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_b_V_y = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_V_y),
    &c19_thisId);
  sf_mex_destroy(&c19_b_V_y);
  *(real_T *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_d_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i134;
  int32_T c19_i135;
  int32_T c19_i136;
  real_T c19_b_inData[14];
  int32_T c19_i137;
  int32_T c19_i138;
  int32_T c19_i139;
  real_T c19_u[14];
  const mxArray *c19_y = NULL;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_i134 = 0;
  for (c19_i135 = 0; c19_i135 < 2; c19_i135++) {
    for (c19_i136 = 0; c19_i136 < 7; c19_i136++) {
      c19_b_inData[c19_i136 + c19_i134] = (*(real_T (*)[14])c19_inData)[c19_i136
        + c19_i134];
    }

    c19_i134 += 7;
  }

  c19_i137 = 0;
  for (c19_i138 = 0; c19_i138 < 2; c19_i138++) {
    for (c19_i139 = 0; c19_i139 < 7; c19_i139++) {
      c19_u[c19_i139 + c19_i137] = c19_b_inData[c19_i139 + c19_i137];
    }

    c19_i137 += 7;
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 2, 7, 2), false);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static void c19_g_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[14])
{
  real_T c19_dv9[14];
  int32_T c19_i140;
  (void)chartInstance;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), c19_dv9, 1, 0, 0U, 1, 0U, 2, 7,
                2);
  for (c19_i140 = 0; c19_i140 < 14; c19_i140++) {
    c19_y[c19_i140] = c19_dv9[c19_i140];
  }

  sf_mex_destroy(&c19_u);
}

static void c19_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_K;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y[14];
  int32_T c19_i141;
  int32_T c19_i142;
  int32_T c19_i143;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_K = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_K), &c19_thisId, c19_y);
  sf_mex_destroy(&c19_K);
  c19_i141 = 0;
  for (c19_i142 = 0; c19_i142 < 2; c19_i142++) {
    for (c19_i143 = 0; c19_i143 < 7; c19_i143++) {
      (*(real_T (*)[14])c19_outData)[c19_i143 + c19_i141] = c19_y[c19_i143 +
        c19_i141];
    }

    c19_i141 += 7;
  }

  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_e_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i144;
  real_T c19_b_inData[2];
  int32_T c19_i145;
  real_T c19_u[2];
  const mxArray *c19_y = NULL;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  for (c19_i144 = 0; c19_i144 < 2; c19_i144++) {
    c19_b_inData[c19_i144] = (*(real_T (*)[2])c19_inData)[c19_i144];
  }

  for (c19_i145 = 0; c19_i145 < 2; c19_i145++) {
    c19_u[c19_i145] = c19_b_inData[c19_i145];
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static void c19_h_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[2])
{
  real_T c19_dv10[2];
  int32_T c19_i146;
  (void)chartInstance;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), c19_dv10, 1, 0, 0U, 1, 0U, 1, 2);
  for (c19_i146 = 0; c19_i146 < 2; c19_i146++) {
    c19_y[c19_i146] = c19_dv10[c19_i146];
  }

  sf_mex_destroy(&c19_u);
}

static void c19_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_y_meas;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y[2];
  int32_T c19_i147;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_y_meas = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_y_meas), &c19_thisId,
    c19_y);
  sf_mex_destroy(&c19_y_meas);
  for (c19_i147 = 0; c19_i147 < 2; c19_i147++) {
    (*(real_T (*)[2])c19_outData)[c19_i147] = c19_y[c19_i147];
  }

  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_f_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i148;
  int32_T c19_i149;
  int32_T c19_i150;
  real_T c19_b_inData[14];
  int32_T c19_i151;
  int32_T c19_i152;
  int32_T c19_i153;
  real_T c19_u[14];
  const mxArray *c19_y = NULL;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_i148 = 0;
  for (c19_i149 = 0; c19_i149 < 7; c19_i149++) {
    for (c19_i150 = 0; c19_i150 < 2; c19_i150++) {
      c19_b_inData[c19_i150 + c19_i148] = (*(real_T (*)[14])c19_inData)[c19_i150
        + c19_i148];
    }

    c19_i148 += 2;
  }

  c19_i151 = 0;
  for (c19_i152 = 0; c19_i152 < 7; c19_i152++) {
    for (c19_i153 = 0; c19_i153 < 2; c19_i153++) {
      c19_u[c19_i153 + c19_i151] = c19_b_inData[c19_i153 + c19_i151];
    }

    c19_i151 += 2;
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 2, 2, 7), false);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static const mxArray *c19_g_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i154;
  real_T c19_b_inData[7];
  int32_T c19_i155;
  real_T c19_u[7];
  const mxArray *c19_y = NULL;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  for (c19_i154 = 0; c19_i154 < 7; c19_i154++) {
    c19_b_inData[c19_i154] = (*(real_T (*)[7])c19_inData)[c19_i154];
  }

  for (c19_i155 = 0; c19_i155 < 7; c19_i155++) {
    c19_u[c19_i155] = c19_b_inData[c19_i155];
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 1, 7), false);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static void c19_i_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[7])
{
  real_T c19_dv11[7];
  int32_T c19_i156;
  (void)chartInstance;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), c19_dv11, 1, 0, 0U, 1, 0U, 1, 7);
  for (c19_i156 = 0; c19_i156 < 7; c19_i156++) {
    c19_y[c19_i156] = c19_dv11[c19_i156];
  }

  sf_mex_destroy(&c19_u);
}

static void c19_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_State_p;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y[7];
  int32_T c19_i157;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_State_p = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_State_p), &c19_thisId,
    c19_y);
  sf_mex_destroy(&c19_State_p);
  for (c19_i157 = 0; c19_i157 < 7; c19_i157++) {
    (*(real_T (*)[7])c19_outData)[c19_i157] = c19_y[c19_i157];
  }

  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_h_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i158;
  int32_T c19_i159;
  int32_T c19_i160;
  real_T c19_b_inData[49];
  int32_T c19_i161;
  int32_T c19_i162;
  int32_T c19_i163;
  real_T c19_u[49];
  const mxArray *c19_y = NULL;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_i158 = 0;
  for (c19_i159 = 0; c19_i159 < 7; c19_i159++) {
    for (c19_i160 = 0; c19_i160 < 7; c19_i160++) {
      c19_b_inData[c19_i160 + c19_i158] = (*(real_T (*)[49])c19_inData)[c19_i160
        + c19_i158];
    }

    c19_i158 += 7;
  }

  c19_i161 = 0;
  for (c19_i162 = 0; c19_i162 < 7; c19_i162++) {
    for (c19_i163 = 0; c19_i163 < 7; c19_i163++) {
      c19_u[c19_i163 + c19_i161] = c19_b_inData[c19_i163 + c19_i161];
    }

    c19_i161 += 7;
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 2, 7, 7), false);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static void c19_j_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[49])
{
  real_T c19_dv12[49];
  int32_T c19_i164;
  (void)chartInstance;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), c19_dv12, 1, 0, 0U, 1, 0U, 2, 7,
                7);
  for (c19_i164 = 0; c19_i164 < 49; c19_i164++) {
    c19_y[c19_i164] = c19_dv12[c19_i164];
  }

  sf_mex_destroy(&c19_u);
}

static void c19_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_G;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y[49];
  int32_T c19_i165;
  int32_T c19_i166;
  int32_T c19_i167;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_G = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_G), &c19_thisId, c19_y);
  sf_mex_destroy(&c19_G);
  c19_i165 = 0;
  for (c19_i166 = 0; c19_i166 < 7; c19_i166++) {
    for (c19_i167 = 0; c19_i167 < 7; c19_i167++) {
      (*(real_T (*)[49])c19_outData)[c19_i167 + c19_i165] = c19_y[c19_i167 +
        c19_i165];
    }

    c19_i165 += 7;
  }

  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_i_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i168;
  int32_T c19_i169;
  int32_T c19_i170;
  real_T c19_b_inData[21];
  int32_T c19_i171;
  int32_T c19_i172;
  int32_T c19_i173;
  real_T c19_u[21];
  const mxArray *c19_y = NULL;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_i168 = 0;
  for (c19_i169 = 0; c19_i169 < 3; c19_i169++) {
    for (c19_i170 = 0; c19_i170 < 7; c19_i170++) {
      c19_b_inData[c19_i170 + c19_i168] = (*(real_T (*)[21])c19_inData)[c19_i170
        + c19_i168];
    }

    c19_i168 += 7;
  }

  c19_i171 = 0;
  for (c19_i172 = 0; c19_i172 < 3; c19_i172++) {
    for (c19_i173 = 0; c19_i173 < 7; c19_i173++) {
      c19_u[c19_i173 + c19_i171] = c19_b_inData[c19_i173 + c19_i171];
    }

    c19_i171 += 7;
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 2, 7, 3), false);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static const mxArray *c19_j_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i174;
  int32_T c19_i175;
  int32_T c19_i176;
  real_T c19_b_inData[4];
  int32_T c19_i177;
  int32_T c19_i178;
  int32_T c19_i179;
  real_T c19_u[4];
  const mxArray *c19_y = NULL;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_i174 = 0;
  for (c19_i175 = 0; c19_i175 < 2; c19_i175++) {
    for (c19_i176 = 0; c19_i176 < 2; c19_i176++) {
      c19_b_inData[c19_i176 + c19_i174] = (*(real_T (*)[4])c19_inData)[c19_i176
        + c19_i174];
    }

    c19_i174 += 2;
  }

  c19_i177 = 0;
  for (c19_i178 = 0; c19_i178 < 2; c19_i178++) {
    for (c19_i179 = 0; c19_i179 < 2; c19_i179++) {
      c19_u[c19_i179 + c19_i177] = c19_b_inData[c19_i179 + c19_i177];
    }

    c19_i177 += 2;
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 2, 2, 2), false);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static void c19_k_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[4])
{
  real_T c19_dv13[4];
  int32_T c19_i180;
  (void)chartInstance;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), c19_dv13, 1, 0, 0U, 1, 0U, 2, 2,
                2);
  for (c19_i180 = 0; c19_i180 < 4; c19_i180++) {
    c19_y[c19_i180] = c19_dv13[c19_i180];
  }

  sf_mex_destroy(&c19_u);
}

static void c19_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_R;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y[4];
  int32_T c19_i181;
  int32_T c19_i182;
  int32_T c19_i183;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_R = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_R), &c19_thisId, c19_y);
  sf_mex_destroy(&c19_R);
  c19_i181 = 0;
  for (c19_i182 = 0; c19_i182 < 2; c19_i182++) {
    for (c19_i183 = 0; c19_i183 < 2; c19_i183++) {
      (*(real_T (*)[4])c19_outData)[c19_i183 + c19_i181] = c19_y[c19_i183 +
        c19_i181];
    }

    c19_i181 += 2;
  }

  sf_mex_destroy(&c19_mxArrayInData);
}

const mxArray *sf_c19_SS6_Estimation2_get_eml_resolved_functions_info(void)
{
  const mxArray *c19_nameCaptureInfo = NULL;
  c19_nameCaptureInfo = NULL;
  sf_mex_assign(&c19_nameCaptureInfo, sf_mex_createstruct("structure", 2, 210, 1),
                false);
  c19_info_helper(&c19_nameCaptureInfo);
  c19_b_info_helper(&c19_nameCaptureInfo);
  c19_c_info_helper(&c19_nameCaptureInfo);
  c19_d_info_helper(&c19_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c19_nameCaptureInfo);
  return c19_nameCaptureInfo;
}

static void c19_info_helper(const mxArray **c19_info)
{
  const mxArray *c19_rhs0 = NULL;
  const mxArray *c19_lhs0 = NULL;
  const mxArray *c19_rhs1 = NULL;
  const mxArray *c19_lhs1 = NULL;
  const mxArray *c19_rhs2 = NULL;
  const mxArray *c19_lhs2 = NULL;
  const mxArray *c19_rhs3 = NULL;
  const mxArray *c19_lhs3 = NULL;
  const mxArray *c19_rhs4 = NULL;
  const mxArray *c19_lhs4 = NULL;
  const mxArray *c19_rhs5 = NULL;
  const mxArray *c19_lhs5 = NULL;
  const mxArray *c19_rhs6 = NULL;
  const mxArray *c19_lhs6 = NULL;
  const mxArray *c19_rhs7 = NULL;
  const mxArray *c19_lhs7 = NULL;
  const mxArray *c19_rhs8 = NULL;
  const mxArray *c19_lhs8 = NULL;
  const mxArray *c19_rhs9 = NULL;
  const mxArray *c19_lhs9 = NULL;
  const mxArray *c19_rhs10 = NULL;
  const mxArray *c19_lhs10 = NULL;
  const mxArray *c19_rhs11 = NULL;
  const mxArray *c19_lhs11 = NULL;
  const mxArray *c19_rhs12 = NULL;
  const mxArray *c19_lhs12 = NULL;
  const mxArray *c19_rhs13 = NULL;
  const mxArray *c19_lhs13 = NULL;
  const mxArray *c19_rhs14 = NULL;
  const mxArray *c19_lhs14 = NULL;
  const mxArray *c19_rhs15 = NULL;
  const mxArray *c19_lhs15 = NULL;
  const mxArray *c19_rhs16 = NULL;
  const mxArray *c19_lhs16 = NULL;
  const mxArray *c19_rhs17 = NULL;
  const mxArray *c19_lhs17 = NULL;
  const mxArray *c19_rhs18 = NULL;
  const mxArray *c19_lhs18 = NULL;
  const mxArray *c19_rhs19 = NULL;
  const mxArray *c19_lhs19 = NULL;
  const mxArray *c19_rhs20 = NULL;
  const mxArray *c19_lhs20 = NULL;
  const mxArray *c19_rhs21 = NULL;
  const mxArray *c19_lhs21 = NULL;
  const mxArray *c19_rhs22 = NULL;
  const mxArray *c19_lhs22 = NULL;
  const mxArray *c19_rhs23 = NULL;
  const mxArray *c19_lhs23 = NULL;
  const mxArray *c19_rhs24 = NULL;
  const mxArray *c19_lhs24 = NULL;
  const mxArray *c19_rhs25 = NULL;
  const mxArray *c19_lhs25 = NULL;
  const mxArray *c19_rhs26 = NULL;
  const mxArray *c19_lhs26 = NULL;
  const mxArray *c19_rhs27 = NULL;
  const mxArray *c19_lhs27 = NULL;
  const mxArray *c19_rhs28 = NULL;
  const mxArray *c19_lhs28 = NULL;
  const mxArray *c19_rhs29 = NULL;
  const mxArray *c19_lhs29 = NULL;
  const mxArray *c19_rhs30 = NULL;
  const mxArray *c19_lhs30 = NULL;
  const mxArray *c19_rhs31 = NULL;
  const mxArray *c19_lhs31 = NULL;
  const mxArray *c19_rhs32 = NULL;
  const mxArray *c19_lhs32 = NULL;
  const mxArray *c19_rhs33 = NULL;
  const mxArray *c19_lhs33 = NULL;
  const mxArray *c19_rhs34 = NULL;
  const mxArray *c19_lhs34 = NULL;
  const mxArray *c19_rhs35 = NULL;
  const mxArray *c19_lhs35 = NULL;
  const mxArray *c19_rhs36 = NULL;
  const mxArray *c19_lhs36 = NULL;
  const mxArray *c19_rhs37 = NULL;
  const mxArray *c19_lhs37 = NULL;
  const mxArray *c19_rhs38 = NULL;
  const mxArray *c19_lhs38 = NULL;
  const mxArray *c19_rhs39 = NULL;
  const mxArray *c19_lhs39 = NULL;
  const mxArray *c19_rhs40 = NULL;
  const mxArray *c19_lhs40 = NULL;
  const mxArray *c19_rhs41 = NULL;
  const mxArray *c19_lhs41 = NULL;
  const mxArray *c19_rhs42 = NULL;
  const mxArray *c19_lhs42 = NULL;
  const mxArray *c19_rhs43 = NULL;
  const mxArray *c19_lhs43 = NULL;
  const mxArray *c19_rhs44 = NULL;
  const mxArray *c19_lhs44 = NULL;
  const mxArray *c19_rhs45 = NULL;
  const mxArray *c19_lhs45 = NULL;
  const mxArray *c19_rhs46 = NULL;
  const mxArray *c19_lhs46 = NULL;
  const mxArray *c19_rhs47 = NULL;
  const mxArray *c19_lhs47 = NULL;
  const mxArray *c19_rhs48 = NULL;
  const mxArray *c19_lhs48 = NULL;
  const mxArray *c19_rhs49 = NULL;
  const mxArray *c19_lhs49 = NULL;
  const mxArray *c19_rhs50 = NULL;
  const mxArray *c19_lhs50 = NULL;
  const mxArray *c19_rhs51 = NULL;
  const mxArray *c19_lhs51 = NULL;
  const mxArray *c19_rhs52 = NULL;
  const mxArray *c19_lhs52 = NULL;
  const mxArray *c19_rhs53 = NULL;
  const mxArray *c19_lhs53 = NULL;
  const mxArray *c19_rhs54 = NULL;
  const mxArray *c19_lhs54 = NULL;
  const mxArray *c19_rhs55 = NULL;
  const mxArray *c19_lhs55 = NULL;
  const mxArray *c19_rhs56 = NULL;
  const mxArray *c19_lhs56 = NULL;
  const mxArray *c19_rhs57 = NULL;
  const mxArray *c19_lhs57 = NULL;
  const mxArray *c19_rhs58 = NULL;
  const mxArray *c19_lhs58 = NULL;
  const mxArray *c19_rhs59 = NULL;
  const mxArray *c19_lhs59 = NULL;
  const mxArray *c19_rhs60 = NULL;
  const mxArray *c19_lhs60 = NULL;
  const mxArray *c19_rhs61 = NULL;
  const mxArray *c19_lhs61 = NULL;
  const mxArray *c19_rhs62 = NULL;
  const mxArray *c19_lhs62 = NULL;
  const mxArray *c19_rhs63 = NULL;
  const mxArray *c19_lhs63 = NULL;
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eye"), "name", "name", 0);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1406834748U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c19_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs0), "rhs", "rhs",
                  0);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs0), "lhs", "lhs",
                  0);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 1);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1368204630U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c19_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs1), "rhs", "rhs",
                  1);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs1), "lhs", "lhs",
                  1);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 2);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 2);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c19_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs2), "rhs", "rhs",
                  2);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs2), "lhs", "lhs",
                  2);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral"),
                  "context", "context", 3);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("isinf"), "name", "name", 3);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 3);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731856U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c19_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs3), "rhs", "rhs",
                  3);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs3), "lhs", "lhs",
                  3);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 4);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c19_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs4), "rhs", "rhs",
                  4);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs4), "lhs", "lhs",
                  4);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 5);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_is_integer_class"),
                  "name", "name", 5);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_integer_class.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840382U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c19_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs5), "rhs", "rhs",
                  5);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs5), "lhs", "lhs",
                  5);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 6);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("intmax"), "name", "name", 6);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c19_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs6), "rhs", "rhs",
                  6);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs6), "lhs", "lhs",
                  6);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 7);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1393352458U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c19_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs7), "rhs", "rhs",
                  7);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs7), "lhs", "lhs",
                  7);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 8);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("intmin"), "name", "name", 8);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 8);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c19_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs8), "rhs", "rhs",
                  8);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs8), "lhs", "lhs",
                  8);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 9);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1393352458U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c19_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs9), "rhs", "rhs",
                  9);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs9), "lhs", "lhs",
                  9);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 10);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.indexIntRelop"), "name", "name", 10);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1326749922U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c19_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!apply_float_relop"),
                  "context", "context", 11);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 11);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1393352458U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c19_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 12);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 12);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c19_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 13);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("intmin"), "name", "name", 13);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c19_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 14);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 14);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c19_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 15);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("intmax"), "name", "name", 15);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c19_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 16);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c19_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 17);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("isfi"), "name", "name", 17);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 17);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isfi.m"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1346531958U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c19_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isfi.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("isnumerictype"), "name",
                  "name", 18);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isnumerictype.m"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1398897198U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c19_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 19);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("intmax"), "name", "name", 19);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c19_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 20);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("intmin"), "name", "name", 20);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 20);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c19_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "context", "context", 21);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 21);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1383898894U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c19_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 22);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 22);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c19_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "context", "context", 23);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("cos"), "name", "name", 23);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 23);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395346496U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c19_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 24);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840322U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c19_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "context", "context", 25);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("sin"), "name", "name", 25);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 25);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395346504U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c19_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 26);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 26);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840336U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c19_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 27);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 27);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c19_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 28);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 28);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 28);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c19_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 29);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 29);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c19_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 30);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  30);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c19_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 31);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 31);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c19_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 32);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 32);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c19_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 33);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 33);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c19_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 34);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 34);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c19_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 35);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 35);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1393352458U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c19_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 36);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 36);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c19_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 37);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.refblas.xgemm"), "name", "name", 37);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c19_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "context", "context", 38);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("pinv"), "name", "name", 38);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840428U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c19_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 39);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 39);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c19_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 40);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 40);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 40);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c19_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 41);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("svd"), "name", "name", 41);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "resolved",
                  "resolved", 41);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840432U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c19_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 42);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 42);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c19_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 43);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 43);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c19_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 44);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("isfinite"), "name", "name",
                  44);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "resolved",
                  "resolved", 44);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731856U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c19_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 45);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 45);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c19_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 46);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("isinf"), "name", "name", 46);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 46);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731856U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c19_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 47);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("isnan"), "name", "name", 47);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 47);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c19_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 48);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 48);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 48);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c19_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 49);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_error"), "name", "name",
                  49);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 49);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c19_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 50);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_xgesvd"), "name", "name",
                  50);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m"),
                  "resolved", "resolved", 50);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840406U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c19_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m"),
                  "context", "context", 51);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_lapack_xgesvd"), "name",
                  "name", 51);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840410U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c19_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m"),
                  "context", "context", 52);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_matlab_zsvdc"), "name",
                  "name", 52);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "resolved", "resolved", 52);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1398897206U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c19_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 53);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 53);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 53);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c19_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 54);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("min"), "name", "name", 54);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 54);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 54);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1311276918U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c19_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "context",
                  "context", 55);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 55);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 55);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1378317584U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c19_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 56);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 56);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 56);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 56);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c19_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 57);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 57);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 57);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c19_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 58);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 58);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 58);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 58);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c19_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 59);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 59);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 59);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 59);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c19_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 60);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 60);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c19_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 61);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 61);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 61);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 61);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c19_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 62);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 62);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 62);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 62);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c19_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 63);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("max"), "name", "name", 63);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 63);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1311276916U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c19_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c19_rhs0);
  sf_mex_destroy(&c19_lhs0);
  sf_mex_destroy(&c19_rhs1);
  sf_mex_destroy(&c19_lhs1);
  sf_mex_destroy(&c19_rhs2);
  sf_mex_destroy(&c19_lhs2);
  sf_mex_destroy(&c19_rhs3);
  sf_mex_destroy(&c19_lhs3);
  sf_mex_destroy(&c19_rhs4);
  sf_mex_destroy(&c19_lhs4);
  sf_mex_destroy(&c19_rhs5);
  sf_mex_destroy(&c19_lhs5);
  sf_mex_destroy(&c19_rhs6);
  sf_mex_destroy(&c19_lhs6);
  sf_mex_destroy(&c19_rhs7);
  sf_mex_destroy(&c19_lhs7);
  sf_mex_destroy(&c19_rhs8);
  sf_mex_destroy(&c19_lhs8);
  sf_mex_destroy(&c19_rhs9);
  sf_mex_destroy(&c19_lhs9);
  sf_mex_destroy(&c19_rhs10);
  sf_mex_destroy(&c19_lhs10);
  sf_mex_destroy(&c19_rhs11);
  sf_mex_destroy(&c19_lhs11);
  sf_mex_destroy(&c19_rhs12);
  sf_mex_destroy(&c19_lhs12);
  sf_mex_destroy(&c19_rhs13);
  sf_mex_destroy(&c19_lhs13);
  sf_mex_destroy(&c19_rhs14);
  sf_mex_destroy(&c19_lhs14);
  sf_mex_destroy(&c19_rhs15);
  sf_mex_destroy(&c19_lhs15);
  sf_mex_destroy(&c19_rhs16);
  sf_mex_destroy(&c19_lhs16);
  sf_mex_destroy(&c19_rhs17);
  sf_mex_destroy(&c19_lhs17);
  sf_mex_destroy(&c19_rhs18);
  sf_mex_destroy(&c19_lhs18);
  sf_mex_destroy(&c19_rhs19);
  sf_mex_destroy(&c19_lhs19);
  sf_mex_destroy(&c19_rhs20);
  sf_mex_destroy(&c19_lhs20);
  sf_mex_destroy(&c19_rhs21);
  sf_mex_destroy(&c19_lhs21);
  sf_mex_destroy(&c19_rhs22);
  sf_mex_destroy(&c19_lhs22);
  sf_mex_destroy(&c19_rhs23);
  sf_mex_destroy(&c19_lhs23);
  sf_mex_destroy(&c19_rhs24);
  sf_mex_destroy(&c19_lhs24);
  sf_mex_destroy(&c19_rhs25);
  sf_mex_destroy(&c19_lhs25);
  sf_mex_destroy(&c19_rhs26);
  sf_mex_destroy(&c19_lhs26);
  sf_mex_destroy(&c19_rhs27);
  sf_mex_destroy(&c19_lhs27);
  sf_mex_destroy(&c19_rhs28);
  sf_mex_destroy(&c19_lhs28);
  sf_mex_destroy(&c19_rhs29);
  sf_mex_destroy(&c19_lhs29);
  sf_mex_destroy(&c19_rhs30);
  sf_mex_destroy(&c19_lhs30);
  sf_mex_destroy(&c19_rhs31);
  sf_mex_destroy(&c19_lhs31);
  sf_mex_destroy(&c19_rhs32);
  sf_mex_destroy(&c19_lhs32);
  sf_mex_destroy(&c19_rhs33);
  sf_mex_destroy(&c19_lhs33);
  sf_mex_destroy(&c19_rhs34);
  sf_mex_destroy(&c19_lhs34);
  sf_mex_destroy(&c19_rhs35);
  sf_mex_destroy(&c19_lhs35);
  sf_mex_destroy(&c19_rhs36);
  sf_mex_destroy(&c19_lhs36);
  sf_mex_destroy(&c19_rhs37);
  sf_mex_destroy(&c19_lhs37);
  sf_mex_destroy(&c19_rhs38);
  sf_mex_destroy(&c19_lhs38);
  sf_mex_destroy(&c19_rhs39);
  sf_mex_destroy(&c19_lhs39);
  sf_mex_destroy(&c19_rhs40);
  sf_mex_destroy(&c19_lhs40);
  sf_mex_destroy(&c19_rhs41);
  sf_mex_destroy(&c19_lhs41);
  sf_mex_destroy(&c19_rhs42);
  sf_mex_destroy(&c19_lhs42);
  sf_mex_destroy(&c19_rhs43);
  sf_mex_destroy(&c19_lhs43);
  sf_mex_destroy(&c19_rhs44);
  sf_mex_destroy(&c19_lhs44);
  sf_mex_destroy(&c19_rhs45);
  sf_mex_destroy(&c19_lhs45);
  sf_mex_destroy(&c19_rhs46);
  sf_mex_destroy(&c19_lhs46);
  sf_mex_destroy(&c19_rhs47);
  sf_mex_destroy(&c19_lhs47);
  sf_mex_destroy(&c19_rhs48);
  sf_mex_destroy(&c19_lhs48);
  sf_mex_destroy(&c19_rhs49);
  sf_mex_destroy(&c19_lhs49);
  sf_mex_destroy(&c19_rhs50);
  sf_mex_destroy(&c19_lhs50);
  sf_mex_destroy(&c19_rhs51);
  sf_mex_destroy(&c19_lhs51);
  sf_mex_destroy(&c19_rhs52);
  sf_mex_destroy(&c19_lhs52);
  sf_mex_destroy(&c19_rhs53);
  sf_mex_destroy(&c19_lhs53);
  sf_mex_destroy(&c19_rhs54);
  sf_mex_destroy(&c19_lhs54);
  sf_mex_destroy(&c19_rhs55);
  sf_mex_destroy(&c19_lhs55);
  sf_mex_destroy(&c19_rhs56);
  sf_mex_destroy(&c19_lhs56);
  sf_mex_destroy(&c19_rhs57);
  sf_mex_destroy(&c19_lhs57);
  sf_mex_destroy(&c19_rhs58);
  sf_mex_destroy(&c19_lhs58);
  sf_mex_destroy(&c19_rhs59);
  sf_mex_destroy(&c19_lhs59);
  sf_mex_destroy(&c19_rhs60);
  sf_mex_destroy(&c19_lhs60);
  sf_mex_destroy(&c19_rhs61);
  sf_mex_destroy(&c19_lhs61);
  sf_mex_destroy(&c19_rhs62);
  sf_mex_destroy(&c19_lhs62);
  sf_mex_destroy(&c19_rhs63);
  sf_mex_destroy(&c19_lhs63);
}

static const mxArray *c19_emlrt_marshallOut(const char * c19_u)
{
  const mxArray *c19_y = NULL;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c19_u)), false);
  return c19_y;
}

static const mxArray *c19_b_emlrt_marshallOut(const uint32_T c19_u)
{
  const mxArray *c19_y = NULL;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 7, 0U, 0U, 0U, 0), false);
  return c19_y;
}

static void c19_b_info_helper(const mxArray **c19_info)
{
  const mxArray *c19_rhs64 = NULL;
  const mxArray *c19_lhs64 = NULL;
  const mxArray *c19_rhs65 = NULL;
  const mxArray *c19_lhs65 = NULL;
  const mxArray *c19_rhs66 = NULL;
  const mxArray *c19_lhs66 = NULL;
  const mxArray *c19_rhs67 = NULL;
  const mxArray *c19_lhs67 = NULL;
  const mxArray *c19_rhs68 = NULL;
  const mxArray *c19_lhs68 = NULL;
  const mxArray *c19_rhs69 = NULL;
  const mxArray *c19_lhs69 = NULL;
  const mxArray *c19_rhs70 = NULL;
  const mxArray *c19_lhs70 = NULL;
  const mxArray *c19_rhs71 = NULL;
  const mxArray *c19_lhs71 = NULL;
  const mxArray *c19_rhs72 = NULL;
  const mxArray *c19_lhs72 = NULL;
  const mxArray *c19_rhs73 = NULL;
  const mxArray *c19_lhs73 = NULL;
  const mxArray *c19_rhs74 = NULL;
  const mxArray *c19_lhs74 = NULL;
  const mxArray *c19_rhs75 = NULL;
  const mxArray *c19_lhs75 = NULL;
  const mxArray *c19_rhs76 = NULL;
  const mxArray *c19_lhs76 = NULL;
  const mxArray *c19_rhs77 = NULL;
  const mxArray *c19_lhs77 = NULL;
  const mxArray *c19_rhs78 = NULL;
  const mxArray *c19_lhs78 = NULL;
  const mxArray *c19_rhs79 = NULL;
  const mxArray *c19_lhs79 = NULL;
  const mxArray *c19_rhs80 = NULL;
  const mxArray *c19_lhs80 = NULL;
  const mxArray *c19_rhs81 = NULL;
  const mxArray *c19_lhs81 = NULL;
  const mxArray *c19_rhs82 = NULL;
  const mxArray *c19_lhs82 = NULL;
  const mxArray *c19_rhs83 = NULL;
  const mxArray *c19_lhs83 = NULL;
  const mxArray *c19_rhs84 = NULL;
  const mxArray *c19_lhs84 = NULL;
  const mxArray *c19_rhs85 = NULL;
  const mxArray *c19_lhs85 = NULL;
  const mxArray *c19_rhs86 = NULL;
  const mxArray *c19_lhs86 = NULL;
  const mxArray *c19_rhs87 = NULL;
  const mxArray *c19_lhs87 = NULL;
  const mxArray *c19_rhs88 = NULL;
  const mxArray *c19_lhs88 = NULL;
  const mxArray *c19_rhs89 = NULL;
  const mxArray *c19_lhs89 = NULL;
  const mxArray *c19_rhs90 = NULL;
  const mxArray *c19_lhs90 = NULL;
  const mxArray *c19_rhs91 = NULL;
  const mxArray *c19_lhs91 = NULL;
  const mxArray *c19_rhs92 = NULL;
  const mxArray *c19_lhs92 = NULL;
  const mxArray *c19_rhs93 = NULL;
  const mxArray *c19_lhs93 = NULL;
  const mxArray *c19_rhs94 = NULL;
  const mxArray *c19_lhs94 = NULL;
  const mxArray *c19_rhs95 = NULL;
  const mxArray *c19_lhs95 = NULL;
  const mxArray *c19_rhs96 = NULL;
  const mxArray *c19_lhs96 = NULL;
  const mxArray *c19_rhs97 = NULL;
  const mxArray *c19_lhs97 = NULL;
  const mxArray *c19_rhs98 = NULL;
  const mxArray *c19_lhs98 = NULL;
  const mxArray *c19_rhs99 = NULL;
  const mxArray *c19_lhs99 = NULL;
  const mxArray *c19_rhs100 = NULL;
  const mxArray *c19_lhs100 = NULL;
  const mxArray *c19_rhs101 = NULL;
  const mxArray *c19_lhs101 = NULL;
  const mxArray *c19_rhs102 = NULL;
  const mxArray *c19_lhs102 = NULL;
  const mxArray *c19_rhs103 = NULL;
  const mxArray *c19_lhs103 = NULL;
  const mxArray *c19_rhs104 = NULL;
  const mxArray *c19_lhs104 = NULL;
  const mxArray *c19_rhs105 = NULL;
  const mxArray *c19_lhs105 = NULL;
  const mxArray *c19_rhs106 = NULL;
  const mxArray *c19_lhs106 = NULL;
  const mxArray *c19_rhs107 = NULL;
  const mxArray *c19_lhs107 = NULL;
  const mxArray *c19_rhs108 = NULL;
  const mxArray *c19_lhs108 = NULL;
  const mxArray *c19_rhs109 = NULL;
  const mxArray *c19_lhs109 = NULL;
  const mxArray *c19_rhs110 = NULL;
  const mxArray *c19_lhs110 = NULL;
  const mxArray *c19_rhs111 = NULL;
  const mxArray *c19_lhs111 = NULL;
  const mxArray *c19_rhs112 = NULL;
  const mxArray *c19_lhs112 = NULL;
  const mxArray *c19_rhs113 = NULL;
  const mxArray *c19_lhs113 = NULL;
  const mxArray *c19_rhs114 = NULL;
  const mxArray *c19_lhs114 = NULL;
  const mxArray *c19_rhs115 = NULL;
  const mxArray *c19_lhs115 = NULL;
  const mxArray *c19_rhs116 = NULL;
  const mxArray *c19_lhs116 = NULL;
  const mxArray *c19_rhs117 = NULL;
  const mxArray *c19_lhs117 = NULL;
  const mxArray *c19_rhs118 = NULL;
  const mxArray *c19_lhs118 = NULL;
  const mxArray *c19_rhs119 = NULL;
  const mxArray *c19_lhs119 = NULL;
  const mxArray *c19_rhs120 = NULL;
  const mxArray *c19_lhs120 = NULL;
  const mxArray *c19_rhs121 = NULL;
  const mxArray *c19_lhs121 = NULL;
  const mxArray *c19_rhs122 = NULL;
  const mxArray *c19_lhs122 = NULL;
  const mxArray *c19_rhs123 = NULL;
  const mxArray *c19_lhs123 = NULL;
  const mxArray *c19_rhs124 = NULL;
  const mxArray *c19_lhs124 = NULL;
  const mxArray *c19_rhs125 = NULL;
  const mxArray *c19_lhs125 = NULL;
  const mxArray *c19_rhs126 = NULL;
  const mxArray *c19_lhs126 = NULL;
  const mxArray *c19_rhs127 = NULL;
  const mxArray *c19_lhs127 = NULL;
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "context",
                  "context", 64);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 64);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 64);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1378317584U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c19_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 65);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 65);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 65);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c19_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 66);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 66);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 66);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c19_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 67);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 67);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 67);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c19_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 68);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 68);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 68);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c19_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 69);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_relop"), "name", "name",
                  69);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 69);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 69);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1342472782U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c19_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "context",
                  "context", 70);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.indexIntRelop"), "name", "name", 70);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 70);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 70);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1326749922U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c19_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 71);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("isnan"), "name", "name", 71);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 71);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 71);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c19_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 72);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 72);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 72);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c19_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 73);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("max"), "name", "name", 73);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 73);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 73);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1311276916U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c19_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 74);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_xnrm2"), "name", "name",
                  74);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 74);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c19_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 75);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 75);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 75);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c19_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 76);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.xnrm2"),
                  "name", "name", 76);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "resolved", "resolved", 76);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c19_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 77);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 77);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 77);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c19_rhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 78);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 78);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 78);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 78);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c19_rhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 79);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.refblas.xnrm2"), "name", "name", 79);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 79);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "resolved", "resolved", 79);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c19_rhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 80);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("realmin"), "name", "name",
                  80);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 80);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 80);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1307672842U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c19_rhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "context",
                  "context", 81);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_realmin"), "name",
                  "name", 81);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "resolved",
                  "resolved", 81);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1307672844U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c19_rhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "context",
                  "context", 82);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 82);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 82);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 82);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c19_rhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 83);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 83);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 83);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 83);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c19_rhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 84);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 84);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 84);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 84);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c19_rhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 85);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 85);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 85);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 85);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c19_rhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs85), "lhs", "lhs",
                  85);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 86);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 86);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 86);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 86);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 86);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 86);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 86);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 86);
  sf_mex_assign(&c19_rhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs86), "rhs", "rhs",
                  86);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs86), "lhs", "lhs",
                  86);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 87);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("abs"), "name", "name", 87);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 87);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 87);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 87);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 87);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 87);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 87);
  sf_mex_assign(&c19_rhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs87), "rhs", "rhs",
                  87);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs87), "lhs", "lhs",
                  87);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 88);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 88);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 88);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 88);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 88);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 88);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 88);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 88);
  sf_mex_assign(&c19_rhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs88), "rhs", "rhs",
                  88);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs88), "lhs", "lhs",
                  88);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 89);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 89);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 89);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 89);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840312U), "fileTimeLo",
                  "fileTimeLo", 89);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 89);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 89);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 89);
  sf_mex_assign(&c19_rhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs89), "rhs", "rhs",
                  89);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs89), "lhs", "lhs",
                  89);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 90);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.scaleVectorByRecip"), "name", "name", 90);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 90);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "resolved", "resolved", 90);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 90);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 90);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 90);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 90);
  sf_mex_assign(&c19_rhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs90), "rhs", "rhs",
                  90);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs90), "lhs", "lhs",
                  90);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 91);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("realmin"), "name", "name",
                  91);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 91);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 91);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1307672842U), "fileTimeLo",
                  "fileTimeLo", 91);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 91);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 91);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 91);
  sf_mex_assign(&c19_rhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs91), "rhs", "rhs",
                  91);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs91), "lhs", "lhs",
                  91);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 92);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eps"), "name", "name", 92);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 92);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 92);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 92);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 92);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 92);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 92);
  sf_mex_assign(&c19_rhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs92), "rhs", "rhs",
                  92);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs92), "lhs", "lhs",
                  92);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 93);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 93);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 93);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 93);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840382U), "fileTimeLo",
                  "fileTimeLo", 93);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 93);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 93);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 93);
  sf_mex_assign(&c19_rhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs93), "rhs", "rhs",
                  93);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs93), "lhs", "lhs",
                  93);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 94);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_eps"), "name", "name",
                  94);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 94);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 94);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 94);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 94);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 94);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 94);
  sf_mex_assign(&c19_rhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs94), "rhs", "rhs",
                  94);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs94), "lhs", "lhs",
                  94);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 95);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 95);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 95);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 95);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 95);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 95);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 95);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 95);
  sf_mex_assign(&c19_rhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs95), "rhs", "rhs",
                  95);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs95), "lhs", "lhs",
                  95);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 96);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("mrdivide"), "name", "name",
                  96);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 96);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 96);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 96);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 96);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 96);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 96);
  sf_mex_assign(&c19_rhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs96), "rhs", "rhs",
                  96);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs96), "lhs", "lhs",
                  96);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 97);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 97);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 97);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 97);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1389739374U), "fileTimeLo",
                  "fileTimeLo", 97);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 97);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 97);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 97);
  sf_mex_assign(&c19_rhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs97), "rhs", "rhs",
                  97);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs97), "lhs", "lhs",
                  97);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 98);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("rdivide"), "name", "name",
                  98);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 98);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 98);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 98);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 98);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 98);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 98);
  sf_mex_assign(&c19_rhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs98), "rhs", "rhs",
                  98);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs98), "lhs", "lhs",
                  98);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 99);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 99);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 99);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 99);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 99);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 99);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 99);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 99);
  sf_mex_assign(&c19_rhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs99), "rhs", "rhs",
                  99);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs99), "lhs", "lhs",
                  99);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 100);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 100);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 100);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 100);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 100);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 100);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 100);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 100);
  sf_mex_assign(&c19_rhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs100), "rhs",
                  "rhs", 100);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs100), "lhs",
                  "lhs", 100);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 101);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_div"), "name", "name",
                  101);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 101);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 101);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 101);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 101);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 101);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 101);
  sf_mex_assign(&c19_rhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs101), "rhs",
                  "rhs", 101);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs101), "lhs",
                  "lhs", 101);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 102);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 102);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 102);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 102);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 102);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 102);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 102);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 102);
  sf_mex_assign(&c19_rhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs102), "rhs",
                  "rhs", 102);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs102), "lhs",
                  "lhs", 102);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 103);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("abs"), "name", "name", 103);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 103);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 103);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 103);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 103);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 103);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 103);
  sf_mex_assign(&c19_rhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs103), "rhs",
                  "rhs", 103);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs103), "lhs",
                  "lhs", 103);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 104);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.xscal"),
                  "name", "name", 104);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 104);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "resolved", "resolved", 104);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 104);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 104);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 104);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 104);
  sf_mex_assign(&c19_rhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs104), "rhs",
                  "rhs", 104);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs104), "lhs",
                  "lhs", 104);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 105);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 105);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 105);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 105);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 105);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 105);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 105);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 105);
  sf_mex_assign(&c19_rhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs105), "rhs",
                  "rhs", 105);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs105), "lhs",
                  "lhs", 105);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 106);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 106);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 106);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 106);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 106);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 106);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 106);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 106);
  sf_mex_assign(&c19_rhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs106), "rhs",
                  "rhs", 106);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs106), "lhs",
                  "lhs", 106);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 107);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 107);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 107);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 107);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 107);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 107);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 107);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 107);
  sf_mex_assign(&c19_rhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs107), "rhs",
                  "rhs", 107);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs107), "lhs",
                  "lhs", 107);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 108);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.refblas.xscal"), "name", "name", 108);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 108);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "resolved", "resolved", 108);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 108);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 108);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 108);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 108);
  sf_mex_assign(&c19_rhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs108), "rhs",
                  "rhs", 108);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs108), "lhs",
                  "lhs", 108);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 109);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 109);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 109);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 109);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 109);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 109);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 109);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 109);
  sf_mex_assign(&c19_rhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs109), "rhs",
                  "rhs", 109);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs109), "lhs",
                  "lhs", 109);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 110);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 110);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 110);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 110);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 110);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 110);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 110);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 110);
  sf_mex_assign(&c19_rhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs110), "rhs",
                  "rhs", 110);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs110), "lhs",
                  "lhs", 110);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 111);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 111);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 111);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 111);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 111);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 111);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 111);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 111);
  sf_mex_assign(&c19_rhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs111), "rhs",
                  "rhs", 111);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs111), "lhs",
                  "lhs", 111);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 112);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 112);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 112);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 112);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 112);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 112);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 112);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 112);
  sf_mex_assign(&c19_rhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs112), "rhs",
                  "rhs", 112);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs112), "lhs",
                  "lhs", 112);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 113);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 113);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 113);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 113);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 113);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 113);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 113);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 113);
  sf_mex_assign(&c19_rhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs113), "rhs",
                  "rhs", 113);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs113), "lhs",
                  "lhs", 113);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 114);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 114);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 114);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 114);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 114);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 114);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 114);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 114);
  sf_mex_assign(&c19_rhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs114), "rhs",
                  "rhs", 114);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs114), "lhs",
                  "lhs", 114);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 115);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 115);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 115);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 115);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 115);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 115);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 115);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 115);
  sf_mex_assign(&c19_rhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs115), "rhs",
                  "rhs", 115);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs115), "lhs",
                  "lhs", 115);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scaleVectorByRecip.p"),
                  "context", "context", 116);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 116);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 116);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 116);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 116);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 116);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 116);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 116);
  sf_mex_assign(&c19_rhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs116), "rhs",
                  "rhs", 116);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs116), "lhs",
                  "lhs", 116);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 117);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_xdotc"), "name", "name",
                  117);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 117);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"),
                  "resolved", "resolved", 117);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 117);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 117);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 117);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 117);
  sf_mex_assign(&c19_rhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs117), "rhs",
                  "rhs", 117);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs117), "lhs",
                  "lhs", 117);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"), "context",
                  "context", 118);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 118);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 118);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 118);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 118);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 118);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 118);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 118);
  sf_mex_assign(&c19_rhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs118), "rhs",
                  "rhs", 118);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs118), "lhs",
                  "lhs", 118);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"), "context",
                  "context", 119);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.xdotc"),
                  "name", "name", 119);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 119);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotc.p"),
                  "resolved", "resolved", 119);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 119);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 119);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 119);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 119);
  sf_mex_assign(&c19_rhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs119), "rhs",
                  "rhs", 119);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs119), "lhs",
                  "lhs", 119);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotc.p"),
                  "context", "context", 120);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.xdot"),
                  "name", "name", 120);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 120);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "resolved", "resolved", 120);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 120);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 120);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 120);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 120);
  sf_mex_assign(&c19_rhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs120), "rhs",
                  "rhs", 120);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs120), "lhs",
                  "lhs", 120);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 121);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 121);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 121);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 121);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 121);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 121);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 121);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 121);
  sf_mex_assign(&c19_rhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs121), "rhs",
                  "rhs", 121);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs121), "lhs",
                  "lhs", 121);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!below_threshold"),
                  "context", "context", 122);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 122);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 122);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 122);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 122);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 122);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 122);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 122);
  sf_mex_assign(&c19_rhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs122), "rhs",
                  "rhs", 122);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs122), "lhs",
                  "lhs", 122);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 123);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.refblas.xdot"),
                  "name", "name", 123);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 123);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "resolved", "resolved", 123);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 123);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 123);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 123);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 123);
  sf_mex_assign(&c19_rhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs123), "rhs",
                  "rhs", 123);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs123), "lhs",
                  "lhs", 123);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "context", "context", 124);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.refblas.xdotx"), "name", "name", 124);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 124);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "resolved", "resolved", 124);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 124);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 124);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 124);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 124);
  sf_mex_assign(&c19_rhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs124), "rhs",
                  "rhs", 124);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs124), "lhs",
                  "lhs", 124);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 125);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 125);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 125);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 125);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 125);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 125);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 125);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 125);
  sf_mex_assign(&c19_rhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs125), "rhs",
                  "rhs", 125);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs125), "lhs",
                  "lhs", 125);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 126);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 126);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 126);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 126);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 126);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 126);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 126);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 126);
  sf_mex_assign(&c19_rhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs126), "rhs",
                  "rhs", 126);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs126), "lhs",
                  "lhs", 126);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 127);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 127);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 127);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 127);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 127);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 127);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 127);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 127);
  sf_mex_assign(&c19_rhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs127), "rhs",
                  "rhs", 127);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs127), "lhs",
                  "lhs", 127);
  sf_mex_destroy(&c19_rhs64);
  sf_mex_destroy(&c19_lhs64);
  sf_mex_destroy(&c19_rhs65);
  sf_mex_destroy(&c19_lhs65);
  sf_mex_destroy(&c19_rhs66);
  sf_mex_destroy(&c19_lhs66);
  sf_mex_destroy(&c19_rhs67);
  sf_mex_destroy(&c19_lhs67);
  sf_mex_destroy(&c19_rhs68);
  sf_mex_destroy(&c19_lhs68);
  sf_mex_destroy(&c19_rhs69);
  sf_mex_destroy(&c19_lhs69);
  sf_mex_destroy(&c19_rhs70);
  sf_mex_destroy(&c19_lhs70);
  sf_mex_destroy(&c19_rhs71);
  sf_mex_destroy(&c19_lhs71);
  sf_mex_destroy(&c19_rhs72);
  sf_mex_destroy(&c19_lhs72);
  sf_mex_destroy(&c19_rhs73);
  sf_mex_destroy(&c19_lhs73);
  sf_mex_destroy(&c19_rhs74);
  sf_mex_destroy(&c19_lhs74);
  sf_mex_destroy(&c19_rhs75);
  sf_mex_destroy(&c19_lhs75);
  sf_mex_destroy(&c19_rhs76);
  sf_mex_destroy(&c19_lhs76);
  sf_mex_destroy(&c19_rhs77);
  sf_mex_destroy(&c19_lhs77);
  sf_mex_destroy(&c19_rhs78);
  sf_mex_destroy(&c19_lhs78);
  sf_mex_destroy(&c19_rhs79);
  sf_mex_destroy(&c19_lhs79);
  sf_mex_destroy(&c19_rhs80);
  sf_mex_destroy(&c19_lhs80);
  sf_mex_destroy(&c19_rhs81);
  sf_mex_destroy(&c19_lhs81);
  sf_mex_destroy(&c19_rhs82);
  sf_mex_destroy(&c19_lhs82);
  sf_mex_destroy(&c19_rhs83);
  sf_mex_destroy(&c19_lhs83);
  sf_mex_destroy(&c19_rhs84);
  sf_mex_destroy(&c19_lhs84);
  sf_mex_destroy(&c19_rhs85);
  sf_mex_destroy(&c19_lhs85);
  sf_mex_destroy(&c19_rhs86);
  sf_mex_destroy(&c19_lhs86);
  sf_mex_destroy(&c19_rhs87);
  sf_mex_destroy(&c19_lhs87);
  sf_mex_destroy(&c19_rhs88);
  sf_mex_destroy(&c19_lhs88);
  sf_mex_destroy(&c19_rhs89);
  sf_mex_destroy(&c19_lhs89);
  sf_mex_destroy(&c19_rhs90);
  sf_mex_destroy(&c19_lhs90);
  sf_mex_destroy(&c19_rhs91);
  sf_mex_destroy(&c19_lhs91);
  sf_mex_destroy(&c19_rhs92);
  sf_mex_destroy(&c19_lhs92);
  sf_mex_destroy(&c19_rhs93);
  sf_mex_destroy(&c19_lhs93);
  sf_mex_destroy(&c19_rhs94);
  sf_mex_destroy(&c19_lhs94);
  sf_mex_destroy(&c19_rhs95);
  sf_mex_destroy(&c19_lhs95);
  sf_mex_destroy(&c19_rhs96);
  sf_mex_destroy(&c19_lhs96);
  sf_mex_destroy(&c19_rhs97);
  sf_mex_destroy(&c19_lhs97);
  sf_mex_destroy(&c19_rhs98);
  sf_mex_destroy(&c19_lhs98);
  sf_mex_destroy(&c19_rhs99);
  sf_mex_destroy(&c19_lhs99);
  sf_mex_destroy(&c19_rhs100);
  sf_mex_destroy(&c19_lhs100);
  sf_mex_destroy(&c19_rhs101);
  sf_mex_destroy(&c19_lhs101);
  sf_mex_destroy(&c19_rhs102);
  sf_mex_destroy(&c19_lhs102);
  sf_mex_destroy(&c19_rhs103);
  sf_mex_destroy(&c19_lhs103);
  sf_mex_destroy(&c19_rhs104);
  sf_mex_destroy(&c19_lhs104);
  sf_mex_destroy(&c19_rhs105);
  sf_mex_destroy(&c19_lhs105);
  sf_mex_destroy(&c19_rhs106);
  sf_mex_destroy(&c19_lhs106);
  sf_mex_destroy(&c19_rhs107);
  sf_mex_destroy(&c19_lhs107);
  sf_mex_destroy(&c19_rhs108);
  sf_mex_destroy(&c19_lhs108);
  sf_mex_destroy(&c19_rhs109);
  sf_mex_destroy(&c19_lhs109);
  sf_mex_destroy(&c19_rhs110);
  sf_mex_destroy(&c19_lhs110);
  sf_mex_destroy(&c19_rhs111);
  sf_mex_destroy(&c19_lhs111);
  sf_mex_destroy(&c19_rhs112);
  sf_mex_destroy(&c19_lhs112);
  sf_mex_destroy(&c19_rhs113);
  sf_mex_destroy(&c19_lhs113);
  sf_mex_destroy(&c19_rhs114);
  sf_mex_destroy(&c19_lhs114);
  sf_mex_destroy(&c19_rhs115);
  sf_mex_destroy(&c19_lhs115);
  sf_mex_destroy(&c19_rhs116);
  sf_mex_destroy(&c19_lhs116);
  sf_mex_destroy(&c19_rhs117);
  sf_mex_destroy(&c19_lhs117);
  sf_mex_destroy(&c19_rhs118);
  sf_mex_destroy(&c19_lhs118);
  sf_mex_destroy(&c19_rhs119);
  sf_mex_destroy(&c19_lhs119);
  sf_mex_destroy(&c19_rhs120);
  sf_mex_destroy(&c19_lhs120);
  sf_mex_destroy(&c19_rhs121);
  sf_mex_destroy(&c19_lhs121);
  sf_mex_destroy(&c19_rhs122);
  sf_mex_destroy(&c19_lhs122);
  sf_mex_destroy(&c19_rhs123);
  sf_mex_destroy(&c19_lhs123);
  sf_mex_destroy(&c19_rhs124);
  sf_mex_destroy(&c19_lhs124);
  sf_mex_destroy(&c19_rhs125);
  sf_mex_destroy(&c19_lhs125);
  sf_mex_destroy(&c19_rhs126);
  sf_mex_destroy(&c19_lhs126);
  sf_mex_destroy(&c19_rhs127);
  sf_mex_destroy(&c19_lhs127);
}

static void c19_c_info_helper(const mxArray **c19_info)
{
  const mxArray *c19_rhs128 = NULL;
  const mxArray *c19_lhs128 = NULL;
  const mxArray *c19_rhs129 = NULL;
  const mxArray *c19_lhs129 = NULL;
  const mxArray *c19_rhs130 = NULL;
  const mxArray *c19_lhs130 = NULL;
  const mxArray *c19_rhs131 = NULL;
  const mxArray *c19_lhs131 = NULL;
  const mxArray *c19_rhs132 = NULL;
  const mxArray *c19_lhs132 = NULL;
  const mxArray *c19_rhs133 = NULL;
  const mxArray *c19_lhs133 = NULL;
  const mxArray *c19_rhs134 = NULL;
  const mxArray *c19_lhs134 = NULL;
  const mxArray *c19_rhs135 = NULL;
  const mxArray *c19_lhs135 = NULL;
  const mxArray *c19_rhs136 = NULL;
  const mxArray *c19_lhs136 = NULL;
  const mxArray *c19_rhs137 = NULL;
  const mxArray *c19_lhs137 = NULL;
  const mxArray *c19_rhs138 = NULL;
  const mxArray *c19_lhs138 = NULL;
  const mxArray *c19_rhs139 = NULL;
  const mxArray *c19_lhs139 = NULL;
  const mxArray *c19_rhs140 = NULL;
  const mxArray *c19_lhs140 = NULL;
  const mxArray *c19_rhs141 = NULL;
  const mxArray *c19_lhs141 = NULL;
  const mxArray *c19_rhs142 = NULL;
  const mxArray *c19_lhs142 = NULL;
  const mxArray *c19_rhs143 = NULL;
  const mxArray *c19_lhs143 = NULL;
  const mxArray *c19_rhs144 = NULL;
  const mxArray *c19_lhs144 = NULL;
  const mxArray *c19_rhs145 = NULL;
  const mxArray *c19_lhs145 = NULL;
  const mxArray *c19_rhs146 = NULL;
  const mxArray *c19_lhs146 = NULL;
  const mxArray *c19_rhs147 = NULL;
  const mxArray *c19_lhs147 = NULL;
  const mxArray *c19_rhs148 = NULL;
  const mxArray *c19_lhs148 = NULL;
  const mxArray *c19_rhs149 = NULL;
  const mxArray *c19_lhs149 = NULL;
  const mxArray *c19_rhs150 = NULL;
  const mxArray *c19_lhs150 = NULL;
  const mxArray *c19_rhs151 = NULL;
  const mxArray *c19_lhs151 = NULL;
  const mxArray *c19_rhs152 = NULL;
  const mxArray *c19_lhs152 = NULL;
  const mxArray *c19_rhs153 = NULL;
  const mxArray *c19_lhs153 = NULL;
  const mxArray *c19_rhs154 = NULL;
  const mxArray *c19_lhs154 = NULL;
  const mxArray *c19_rhs155 = NULL;
  const mxArray *c19_lhs155 = NULL;
  const mxArray *c19_rhs156 = NULL;
  const mxArray *c19_lhs156 = NULL;
  const mxArray *c19_rhs157 = NULL;
  const mxArray *c19_lhs157 = NULL;
  const mxArray *c19_rhs158 = NULL;
  const mxArray *c19_lhs158 = NULL;
  const mxArray *c19_rhs159 = NULL;
  const mxArray *c19_lhs159 = NULL;
  const mxArray *c19_rhs160 = NULL;
  const mxArray *c19_lhs160 = NULL;
  const mxArray *c19_rhs161 = NULL;
  const mxArray *c19_lhs161 = NULL;
  const mxArray *c19_rhs162 = NULL;
  const mxArray *c19_lhs162 = NULL;
  const mxArray *c19_rhs163 = NULL;
  const mxArray *c19_lhs163 = NULL;
  const mxArray *c19_rhs164 = NULL;
  const mxArray *c19_lhs164 = NULL;
  const mxArray *c19_rhs165 = NULL;
  const mxArray *c19_lhs165 = NULL;
  const mxArray *c19_rhs166 = NULL;
  const mxArray *c19_lhs166 = NULL;
  const mxArray *c19_rhs167 = NULL;
  const mxArray *c19_lhs167 = NULL;
  const mxArray *c19_rhs168 = NULL;
  const mxArray *c19_lhs168 = NULL;
  const mxArray *c19_rhs169 = NULL;
  const mxArray *c19_lhs169 = NULL;
  const mxArray *c19_rhs170 = NULL;
  const mxArray *c19_lhs170 = NULL;
  const mxArray *c19_rhs171 = NULL;
  const mxArray *c19_lhs171 = NULL;
  const mxArray *c19_rhs172 = NULL;
  const mxArray *c19_lhs172 = NULL;
  const mxArray *c19_rhs173 = NULL;
  const mxArray *c19_lhs173 = NULL;
  const mxArray *c19_rhs174 = NULL;
  const mxArray *c19_lhs174 = NULL;
  const mxArray *c19_rhs175 = NULL;
  const mxArray *c19_lhs175 = NULL;
  const mxArray *c19_rhs176 = NULL;
  const mxArray *c19_lhs176 = NULL;
  const mxArray *c19_rhs177 = NULL;
  const mxArray *c19_lhs177 = NULL;
  const mxArray *c19_rhs178 = NULL;
  const mxArray *c19_lhs178 = NULL;
  const mxArray *c19_rhs179 = NULL;
  const mxArray *c19_lhs179 = NULL;
  const mxArray *c19_rhs180 = NULL;
  const mxArray *c19_lhs180 = NULL;
  const mxArray *c19_rhs181 = NULL;
  const mxArray *c19_lhs181 = NULL;
  const mxArray *c19_rhs182 = NULL;
  const mxArray *c19_lhs182 = NULL;
  const mxArray *c19_rhs183 = NULL;
  const mxArray *c19_lhs183 = NULL;
  const mxArray *c19_rhs184 = NULL;
  const mxArray *c19_lhs184 = NULL;
  const mxArray *c19_rhs185 = NULL;
  const mxArray *c19_lhs185 = NULL;
  const mxArray *c19_rhs186 = NULL;
  const mxArray *c19_lhs186 = NULL;
  const mxArray *c19_rhs187 = NULL;
  const mxArray *c19_lhs187 = NULL;
  const mxArray *c19_rhs188 = NULL;
  const mxArray *c19_lhs188 = NULL;
  const mxArray *c19_rhs189 = NULL;
  const mxArray *c19_lhs189 = NULL;
  const mxArray *c19_rhs190 = NULL;
  const mxArray *c19_lhs190 = NULL;
  const mxArray *c19_rhs191 = NULL;
  const mxArray *c19_lhs191 = NULL;
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 128);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("mrdivide"), "name", "name",
                  128);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 128);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 128);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 128);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 128);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 128);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 128);
  sf_mex_assign(&c19_rhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs128), "rhs",
                  "rhs", 128);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs128), "lhs",
                  "lhs", 128);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 129);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_xaxpy"), "name", "name",
                  129);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 129);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"),
                  "resolved", "resolved", 129);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 129);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 129);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 129);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 129);
  sf_mex_assign(&c19_rhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs129), "rhs",
                  "rhs", 129);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs129), "lhs",
                  "lhs", 129);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"), "context",
                  "context", 130);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 130);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 130);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 130);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 130);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 130);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 130);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 130);
  sf_mex_assign(&c19_rhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs130), "rhs",
                  "rhs", 130);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs130), "lhs",
                  "lhs", 130);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"), "context",
                  "context", 131);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.xaxpy"),
                  "name", "name", 131);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 131);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "resolved", "resolved", 131);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 131);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 131);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 131);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 131);
  sf_mex_assign(&c19_rhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs131), "rhs",
                  "rhs", 131);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs131), "lhs",
                  "lhs", 131);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 132);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 132);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 132);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 132);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 132);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 132);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 132);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 132);
  sf_mex_assign(&c19_rhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs132), "rhs",
                  "rhs", 132);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs132), "lhs",
                  "lhs", 132);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p!below_threshold"),
                  "context", "context", 133);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 133);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 133);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 133);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 133);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 133);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 133);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 133);
  sf_mex_assign(&c19_rhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs133), "rhs",
                  "rhs", 133);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs133), "lhs",
                  "lhs", 133);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 134);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 134);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 134);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 134);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 134);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 134);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 134);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 134);
  sf_mex_assign(&c19_rhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs134), "rhs",
                  "rhs", 134);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs134), "lhs",
                  "lhs", 134);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 135);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.refblas.xaxpy"), "name", "name", 135);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 135);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "resolved", "resolved", 135);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 135);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 135);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 135);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 135);
  sf_mex_assign(&c19_rhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs135), "rhs",
                  "rhs", 135);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs135), "lhs",
                  "lhs", 135);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 136);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.isaUint"),
                  "name", "name", 136);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 136);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/isaUint.p"),
                  "resolved", "resolved", 136);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 136);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 136);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 136);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 136);
  sf_mex_assign(&c19_rhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs136), "rhs",
                  "rhs", 136);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs136), "lhs",
                  "lhs", 136);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 137);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 137);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 137);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 137);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 137);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 137);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 137);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 137);
  sf_mex_assign(&c19_rhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs137), "rhs",
                  "rhs", 137);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs137), "lhs",
                  "lhs", 137);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 138);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 138);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 138);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 138);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 138);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 138);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 138);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 138);
  sf_mex_assign(&c19_rhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs138), "rhs",
                  "rhs", 138);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs138), "lhs",
                  "lhs", 138);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 139);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 139);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 139);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 139);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 139);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 139);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 139);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 139);
  sf_mex_assign(&c19_rhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs139), "rhs",
                  "rhs", 139);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs139), "lhs",
                  "lhs", 139);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 140);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 140);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 140);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 140);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 140);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 140);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 140);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 140);
  sf_mex_assign(&c19_rhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs140), "rhs",
                  "rhs", 140);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs140), "lhs",
                  "lhs", 140);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 141);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 141);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 141);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 141);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 141);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 141);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 141);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 141);
  sf_mex_assign(&c19_rhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs141), "rhs",
                  "rhs", 141);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs141), "lhs",
                  "lhs", 141);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!below_threshold"),
                  "context", "context", 142);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("length"), "name", "name",
                  142);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 142);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 142);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1303167806U), "fileTimeLo",
                  "fileTimeLo", 142);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 142);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 142);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 142);
  sf_mex_assign(&c19_rhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs142), "rhs",
                  "rhs", 142);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs142), "lhs",
                  "lhs", 142);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength"),
                  "context", "context", 143);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 143);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 143);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 143);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 143);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 143);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 143);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 143);
  sf_mex_assign(&c19_rhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs143), "rhs",
                  "rhs", 143);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs143), "lhs",
                  "lhs", 143);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p!below_threshold"),
                  "context", "context", 144);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("length"), "name", "name",
                  144);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 144);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 144);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1303167806U), "fileTimeLo",
                  "fileTimeLo", 144);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 144);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 144);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 144);
  sf_mex_assign(&c19_rhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs144), "rhs",
                  "rhs", 144);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs144), "lhs",
                  "lhs", 144);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 145);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("abs"), "name", "name", 145);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 145);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 145);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 145);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 145);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 145);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 145);
  sf_mex_assign(&c19_rhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs145), "rhs",
                  "rhs", 145);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs145), "lhs",
                  "lhs", 145);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 146);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_xscal"), "name", "name",
                  146);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 146);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"),
                  "resolved", "resolved", 146);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 146);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 146);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 146);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 146);
  sf_mex_assign(&c19_rhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs146), "rhs",
                  "rhs", 146);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs146), "lhs",
                  "lhs", 146);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 147);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 147);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 147);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 147);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 147);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 147);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 147);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 147);
  sf_mex_assign(&c19_rhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs147), "rhs",
                  "rhs", 147);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs147), "lhs",
                  "lhs", 147);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 148);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.xscal"),
                  "name", "name", 148);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 148);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "resolved", "resolved", 148);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 148);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 148);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 148);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 148);
  sf_mex_assign(&c19_rhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs148), "rhs",
                  "rhs", 148);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs148), "lhs",
                  "lhs", 148);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 149);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("realmin"), "name", "name",
                  149);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 149);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 149);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1307672842U), "fileTimeLo",
                  "fileTimeLo", 149);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 149);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 149);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 149);
  sf_mex_assign(&c19_rhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs149), "rhs",
                  "rhs", 149);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs149), "lhs",
                  "lhs", 149);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 150);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eps"), "name", "name", 150);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 150);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 150);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 150);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 150);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 150);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 150);
  sf_mex_assign(&c19_rhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs150), "rhs",
                  "rhs", 150);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs150), "lhs",
                  "lhs", 150);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 151);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 151);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 151);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 151);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 151);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 151);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 151);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 151);
  sf_mex_assign(&c19_rhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs151), "rhs",
                  "rhs", 151);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs151), "lhs",
                  "lhs", 151);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 152);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_error"), "name", "name",
                  152);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 152);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 152);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 152);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 152);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 152);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 152);
  sf_mex_assign(&c19_rhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs152), "rhs",
                  "rhs", 152);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs152), "lhs",
                  "lhs", 152);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 153);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_const_nonsingleton_dim"),
                  "name", "name", 153);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 153);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "resolved", "resolved", 153);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 153);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 153);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 153);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 153);
  sf_mex_assign(&c19_rhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs153), "rhs",
                  "rhs", 153);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs153), "lhs",
                  "lhs", 153);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "context", "context", 154);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 154);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 154);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 154);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 154);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 154);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 154);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 154);
  sf_mex_assign(&c19_rhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs154), "rhs",
                  "rhs", 154);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs154), "lhs",
                  "lhs", 154);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 155);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 155);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 155);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 155);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 155);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 155);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 155);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 155);
  sf_mex_assign(&c19_rhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs155), "rhs",
                  "rhs", 155);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs155), "lhs",
                  "lhs", 155);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 156);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 156);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 156);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 156);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 156);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 156);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 156);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 156);
  sf_mex_assign(&c19_rhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs156), "rhs",
                  "rhs", 156);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs156), "lhs",
                  "lhs", 156);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 157);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 157);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 157);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 157);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 157);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 157);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 157);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 157);
  sf_mex_assign(&c19_rhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs157), "rhs",
                  "rhs", 157);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs157), "lhs",
                  "lhs", 157);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 158);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("isnan"), "name", "name", 158);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 158);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 158);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 158);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 158);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 158);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 158);
  sf_mex_assign(&c19_rhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs158), "rhs",
                  "rhs", 158);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs158), "lhs",
                  "lhs", 158);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 159);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 159);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 159);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 159);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 159);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 159);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 159);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 159);
  sf_mex_assign(&c19_rhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs159), "rhs",
                  "rhs", 159);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs159), "lhs",
                  "lhs", 159);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 160);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 160);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 160);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 160);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 160);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 160);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 160);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 160);
  sf_mex_assign(&c19_rhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs160), "rhs",
                  "rhs", 160);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs160), "lhs",
                  "lhs", 160);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 161);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 161);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 161);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 161);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 161);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 161);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 161);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 161);
  sf_mex_assign(&c19_rhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs161), "rhs",
                  "rhs", 161);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs161), "lhs",
                  "lhs", 161);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 162);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_relop"), "name", "name",
                  162);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 162);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 162);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1342472782U), "fileTimeLo",
                  "fileTimeLo", 162);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 162);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 162);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 162);
  sf_mex_assign(&c19_rhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs162), "rhs",
                  "rhs", 162);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs162), "lhs",
                  "lhs", 162);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 163);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("sqrt"), "name", "name", 163);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 163);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 163);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 163);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 163);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 163);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 163);
  sf_mex_assign(&c19_rhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs163), "rhs",
                  "rhs", 163);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs163), "lhs",
                  "lhs", 163);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 164);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_error"), "name", "name",
                  164);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 164);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 164);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 164);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 164);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 164);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 164);
  sf_mex_assign(&c19_rhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs164), "rhs",
                  "rhs", 164);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs164), "lhs",
                  "lhs", 164);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 165);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 165);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 165);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 165);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 165);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 165);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 165);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 165);
  sf_mex_assign(&c19_rhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs165), "rhs",
                  "rhs", 165);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs165), "lhs",
                  "lhs", 165);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 166);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_xrotg"), "name", "name",
                  166);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 166);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"),
                  "resolved", "resolved", 166);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 166);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 166);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 166);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 166);
  sf_mex_assign(&c19_rhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs166), "rhs",
                  "rhs", 166);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs166), "lhs",
                  "lhs", 166);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"), "context",
                  "context", 167);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 167);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 167);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 167);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 167);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 167);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 167);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 167);
  sf_mex_assign(&c19_rhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs167), "rhs",
                  "rhs", 167);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs167), "lhs",
                  "lhs", 167);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"), "context",
                  "context", 168);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.xrotg"),
                  "name", "name", 168);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 168);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "resolved", "resolved", 168);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 168);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 168);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 168);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 168);
  sf_mex_assign(&c19_rhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs168), "rhs",
                  "rhs", 168);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs168), "lhs",
                  "lhs", 168);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "context", "context", 169);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 169);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 169);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 169);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 169);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 169);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 169);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 169);
  sf_mex_assign(&c19_rhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs169), "rhs",
                  "rhs", 169);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs169), "lhs",
                  "lhs", 169);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "context", "context", 170);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.refblas.xrotg"), "name", "name", 170);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 170);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "resolved", "resolved", 170);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 170);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 170);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 170);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 170);
  sf_mex_assign(&c19_rhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs170), "rhs",
                  "rhs", 170);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs170), "lhs",
                  "lhs", 170);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 171);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("abs"), "name", "name", 171);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 171);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 171);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 171);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 171);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 171);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 171);
  sf_mex_assign(&c19_rhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs171), "rhs",
                  "rhs", 171);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs171), "lhs",
                  "lhs", 171);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 172);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("mrdivide"), "name", "name",
                  172);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 172);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 172);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 172);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 172);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 172);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 172);
  sf_mex_assign(&c19_rhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs172), "rhs",
                  "rhs", 172);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs172), "lhs",
                  "lhs", 172);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 173);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("sqrt"), "name", "name", 173);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 173);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 173);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 173);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 173);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 173);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 173);
  sf_mex_assign(&c19_rhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs173), "rhs",
                  "rhs", 173);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs173), "lhs",
                  "lhs", 173);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p!eml_ceval_xrotg"),
                  "context", "context", 174);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 174);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 174);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 174);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 174);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 174);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 174);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 174);
  sf_mex_assign(&c19_rhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs174), "rhs",
                  "rhs", 174);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs174), "lhs",
                  "lhs", 174);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 175);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_xrot"), "name", "name",
                  175);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 175);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "resolved",
                  "resolved", 175);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 175);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 175);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 175);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 175);
  sf_mex_assign(&c19_rhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs175), "rhs",
                  "rhs", 175);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs175), "lhs",
                  "lhs", 175);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "context",
                  "context", 176);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 176);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 176);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 176);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 176);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 176);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 176);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 176);
  sf_mex_assign(&c19_rhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs176), "rhs",
                  "rhs", 176);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs176), "lhs",
                  "lhs", 176);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "context",
                  "context", 177);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.xrot"),
                  "name", "name", 177);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 177);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "resolved", "resolved", 177);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 177);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 177);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 177);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 177);
  sf_mex_assign(&c19_rhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs177), "rhs",
                  "rhs", 177);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs177), "lhs",
                  "lhs", 177);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 178);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 178);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 178);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 178);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 178);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 178);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 178);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 178);
  sf_mex_assign(&c19_rhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs178), "rhs",
                  "rhs", 178);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs178), "lhs",
                  "lhs", 178);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p!below_threshold"),
                  "context", "context", 179);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 179);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 179);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 179);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 179);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 179);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 179);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 179);
  sf_mex_assign(&c19_rhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs179), "rhs",
                  "rhs", 179);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs179), "lhs",
                  "lhs", 179);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 180);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 180);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 180);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 180);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 180);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 180);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 180);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 180);
  sf_mex_assign(&c19_rhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs180), "rhs",
                  "rhs", 180);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs180), "lhs",
                  "lhs", 180);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 181);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.refblas.xrot"),
                  "name", "name", 181);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 181);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "resolved", "resolved", 181);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 181);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 181);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 181);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 181);
  sf_mex_assign(&c19_rhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs181), "rhs",
                  "rhs", 181);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs181), "lhs",
                  "lhs", 181);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "context", "context", 182);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 182);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 182);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 182);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 182);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 182);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 182);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 182);
  sf_mex_assign(&c19_rhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs182), "rhs",
                  "rhs", 182);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs182), "lhs",
                  "lhs", 182);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "context", "context", 183);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 183);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 183);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 183);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 183);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 183);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 183);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 183);
  sf_mex_assign(&c19_rhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs183), "rhs",
                  "rhs", 183);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs183), "lhs",
                  "lhs", 183);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 184);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_xswap"), "name", "name",
                  184);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 184);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"),
                  "resolved", "resolved", 184);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 184);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 184);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 184);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 184);
  sf_mex_assign(&c19_rhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs184), "rhs",
                  "rhs", 184);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs184), "lhs",
                  "lhs", 184);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 185);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 185);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 185);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 185);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 185);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 185);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 185);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 185);
  sf_mex_assign(&c19_rhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs185), "rhs",
                  "rhs", 185);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs185), "lhs",
                  "lhs", 185);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 186);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.blas.xswap"),
                  "name", "name", 186);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 186);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "resolved", "resolved", 186);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 186);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 186);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 186);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 186);
  sf_mex_assign(&c19_rhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs186), "rhs",
                  "rhs", 186);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs186), "lhs",
                  "lhs", 186);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 187);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 187);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 187);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 187);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 187);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 187);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 187);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 187);
  sf_mex_assign(&c19_rhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs187), "rhs",
                  "rhs", 187);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs187), "lhs",
                  "lhs", 187);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p!below_threshold"),
                  "context", "context", 188);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 188);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 188);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 188);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 188);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 188);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 188);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 188);
  sf_mex_assign(&c19_rhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs188), "rhs",
                  "rhs", 188);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs188), "lhs",
                  "lhs", 188);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 189);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.refblas.xswap"), "name", "name", 189);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 189);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "resolved", "resolved", 189);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829372U), "fileTimeLo",
                  "fileTimeLo", 189);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 189);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 189);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 189);
  sf_mex_assign(&c19_rhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs189), "rhs",
                  "rhs", 189);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs189), "lhs",
                  "lhs", 189);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 190);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("abs"), "name", "name", 190);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 190);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 190);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 190);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 190);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 190);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 190);
  sf_mex_assign(&c19_rhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs190), "rhs",
                  "rhs", 190);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs190), "lhs",
                  "lhs", 190);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 191);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 191);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 191);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 191);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 191);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 191);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 191);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 191);
  sf_mex_assign(&c19_rhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs191), "rhs",
                  "rhs", 191);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs191), "lhs",
                  "lhs", 191);
  sf_mex_destroy(&c19_rhs128);
  sf_mex_destroy(&c19_lhs128);
  sf_mex_destroy(&c19_rhs129);
  sf_mex_destroy(&c19_lhs129);
  sf_mex_destroy(&c19_rhs130);
  sf_mex_destroy(&c19_lhs130);
  sf_mex_destroy(&c19_rhs131);
  sf_mex_destroy(&c19_lhs131);
  sf_mex_destroy(&c19_rhs132);
  sf_mex_destroy(&c19_lhs132);
  sf_mex_destroy(&c19_rhs133);
  sf_mex_destroy(&c19_lhs133);
  sf_mex_destroy(&c19_rhs134);
  sf_mex_destroy(&c19_lhs134);
  sf_mex_destroy(&c19_rhs135);
  sf_mex_destroy(&c19_lhs135);
  sf_mex_destroy(&c19_rhs136);
  sf_mex_destroy(&c19_lhs136);
  sf_mex_destroy(&c19_rhs137);
  sf_mex_destroy(&c19_lhs137);
  sf_mex_destroy(&c19_rhs138);
  sf_mex_destroy(&c19_lhs138);
  sf_mex_destroy(&c19_rhs139);
  sf_mex_destroy(&c19_lhs139);
  sf_mex_destroy(&c19_rhs140);
  sf_mex_destroy(&c19_lhs140);
  sf_mex_destroy(&c19_rhs141);
  sf_mex_destroy(&c19_lhs141);
  sf_mex_destroy(&c19_rhs142);
  sf_mex_destroy(&c19_lhs142);
  sf_mex_destroy(&c19_rhs143);
  sf_mex_destroy(&c19_lhs143);
  sf_mex_destroy(&c19_rhs144);
  sf_mex_destroy(&c19_lhs144);
  sf_mex_destroy(&c19_rhs145);
  sf_mex_destroy(&c19_lhs145);
  sf_mex_destroy(&c19_rhs146);
  sf_mex_destroy(&c19_lhs146);
  sf_mex_destroy(&c19_rhs147);
  sf_mex_destroy(&c19_lhs147);
  sf_mex_destroy(&c19_rhs148);
  sf_mex_destroy(&c19_lhs148);
  sf_mex_destroy(&c19_rhs149);
  sf_mex_destroy(&c19_lhs149);
  sf_mex_destroy(&c19_rhs150);
  sf_mex_destroy(&c19_lhs150);
  sf_mex_destroy(&c19_rhs151);
  sf_mex_destroy(&c19_lhs151);
  sf_mex_destroy(&c19_rhs152);
  sf_mex_destroy(&c19_lhs152);
  sf_mex_destroy(&c19_rhs153);
  sf_mex_destroy(&c19_lhs153);
  sf_mex_destroy(&c19_rhs154);
  sf_mex_destroy(&c19_lhs154);
  sf_mex_destroy(&c19_rhs155);
  sf_mex_destroy(&c19_lhs155);
  sf_mex_destroy(&c19_rhs156);
  sf_mex_destroy(&c19_lhs156);
  sf_mex_destroy(&c19_rhs157);
  sf_mex_destroy(&c19_lhs157);
  sf_mex_destroy(&c19_rhs158);
  sf_mex_destroy(&c19_lhs158);
  sf_mex_destroy(&c19_rhs159);
  sf_mex_destroy(&c19_lhs159);
  sf_mex_destroy(&c19_rhs160);
  sf_mex_destroy(&c19_lhs160);
  sf_mex_destroy(&c19_rhs161);
  sf_mex_destroy(&c19_lhs161);
  sf_mex_destroy(&c19_rhs162);
  sf_mex_destroy(&c19_lhs162);
  sf_mex_destroy(&c19_rhs163);
  sf_mex_destroy(&c19_lhs163);
  sf_mex_destroy(&c19_rhs164);
  sf_mex_destroy(&c19_lhs164);
  sf_mex_destroy(&c19_rhs165);
  sf_mex_destroy(&c19_lhs165);
  sf_mex_destroy(&c19_rhs166);
  sf_mex_destroy(&c19_lhs166);
  sf_mex_destroy(&c19_rhs167);
  sf_mex_destroy(&c19_lhs167);
  sf_mex_destroy(&c19_rhs168);
  sf_mex_destroy(&c19_lhs168);
  sf_mex_destroy(&c19_rhs169);
  sf_mex_destroy(&c19_lhs169);
  sf_mex_destroy(&c19_rhs170);
  sf_mex_destroy(&c19_lhs170);
  sf_mex_destroy(&c19_rhs171);
  sf_mex_destroy(&c19_lhs171);
  sf_mex_destroy(&c19_rhs172);
  sf_mex_destroy(&c19_lhs172);
  sf_mex_destroy(&c19_rhs173);
  sf_mex_destroy(&c19_lhs173);
  sf_mex_destroy(&c19_rhs174);
  sf_mex_destroy(&c19_lhs174);
  sf_mex_destroy(&c19_rhs175);
  sf_mex_destroy(&c19_lhs175);
  sf_mex_destroy(&c19_rhs176);
  sf_mex_destroy(&c19_lhs176);
  sf_mex_destroy(&c19_rhs177);
  sf_mex_destroy(&c19_lhs177);
  sf_mex_destroy(&c19_rhs178);
  sf_mex_destroy(&c19_lhs178);
  sf_mex_destroy(&c19_rhs179);
  sf_mex_destroy(&c19_lhs179);
  sf_mex_destroy(&c19_rhs180);
  sf_mex_destroy(&c19_lhs180);
  sf_mex_destroy(&c19_rhs181);
  sf_mex_destroy(&c19_lhs181);
  sf_mex_destroy(&c19_rhs182);
  sf_mex_destroy(&c19_lhs182);
  sf_mex_destroy(&c19_rhs183);
  sf_mex_destroy(&c19_lhs183);
  sf_mex_destroy(&c19_rhs184);
  sf_mex_destroy(&c19_lhs184);
  sf_mex_destroy(&c19_rhs185);
  sf_mex_destroy(&c19_lhs185);
  sf_mex_destroy(&c19_rhs186);
  sf_mex_destroy(&c19_lhs186);
  sf_mex_destroy(&c19_rhs187);
  sf_mex_destroy(&c19_lhs187);
  sf_mex_destroy(&c19_rhs188);
  sf_mex_destroy(&c19_lhs188);
  sf_mex_destroy(&c19_rhs189);
  sf_mex_destroy(&c19_lhs189);
  sf_mex_destroy(&c19_rhs190);
  sf_mex_destroy(&c19_lhs190);
  sf_mex_destroy(&c19_rhs191);
  sf_mex_destroy(&c19_lhs191);
}

static void c19_d_info_helper(const mxArray **c19_info)
{
  const mxArray *c19_rhs192 = NULL;
  const mxArray *c19_lhs192 = NULL;
  const mxArray *c19_rhs193 = NULL;
  const mxArray *c19_lhs193 = NULL;
  const mxArray *c19_rhs194 = NULL;
  const mxArray *c19_lhs194 = NULL;
  const mxArray *c19_rhs195 = NULL;
  const mxArray *c19_lhs195 = NULL;
  const mxArray *c19_rhs196 = NULL;
  const mxArray *c19_lhs196 = NULL;
  const mxArray *c19_rhs197 = NULL;
  const mxArray *c19_lhs197 = NULL;
  const mxArray *c19_rhs198 = NULL;
  const mxArray *c19_lhs198 = NULL;
  const mxArray *c19_rhs199 = NULL;
  const mxArray *c19_lhs199 = NULL;
  const mxArray *c19_rhs200 = NULL;
  const mxArray *c19_lhs200 = NULL;
  const mxArray *c19_rhs201 = NULL;
  const mxArray *c19_lhs201 = NULL;
  const mxArray *c19_rhs202 = NULL;
  const mxArray *c19_lhs202 = NULL;
  const mxArray *c19_rhs203 = NULL;
  const mxArray *c19_lhs203 = NULL;
  const mxArray *c19_rhs204 = NULL;
  const mxArray *c19_lhs204 = NULL;
  const mxArray *c19_rhs205 = NULL;
  const mxArray *c19_lhs205 = NULL;
  const mxArray *c19_rhs206 = NULL;
  const mxArray *c19_lhs206 = NULL;
  const mxArray *c19_rhs207 = NULL;
  const mxArray *c19_lhs207 = NULL;
  const mxArray *c19_rhs208 = NULL;
  const mxArray *c19_lhs208 = NULL;
  const mxArray *c19_rhs209 = NULL;
  const mxArray *c19_lhs209 = NULL;
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 192);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 192);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 192);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 192);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1286840312U), "fileTimeLo",
                  "fileTimeLo", 192);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 192);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 192);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 192);
  sf_mex_assign(&c19_rhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs192), "rhs",
                  "rhs", 192);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs192), "lhs",
                  "lhs", 192);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 193);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 193);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 193);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 193);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 193);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 193);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 193);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 193);
  sf_mex_assign(&c19_rhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs193), "rhs",
                  "rhs", 193);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs193), "lhs",
                  "lhs", 193);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 194);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 194);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 194);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 194);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 194);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 194);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 194);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 194);
  sf_mex_assign(&c19_rhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs194), "rhs",
                  "rhs", 194);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs194), "lhs",
                  "lhs", 194);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 195);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eps"), "name", "name", 195);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 195);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 195);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 195);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 195);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 195);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 195);
  sf_mex_assign(&c19_rhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs195), "rhs",
                  "rhs", 195);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs195), "lhs",
                  "lhs", 195);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 196);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 196);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 196);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 196);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 196);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 196);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 196);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 196);
  sf_mex_assign(&c19_rhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs196), "rhs",
                  "rhs", 196);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs196), "lhs",
                  "lhs", 196);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 197);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 197);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 197);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 197);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 197);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 197);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 197);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 197);
  sf_mex_assign(&c19_rhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs197), "rhs",
                  "rhs", 197);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs197), "lhs",
                  "lhs", 197);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 198);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 198);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 198);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 198);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 198);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 198);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 198);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 198);
  sf_mex_assign(&c19_rhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs198), "rhs",
                  "rhs", 198);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs198), "lhs",
                  "lhs", 198);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 199);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_div"), "name", "name",
                  199);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 199);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 199);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 199);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 199);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 199);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 199);
  sf_mex_assign(&c19_rhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs199), "rhs",
                  "rhs", 199);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs199), "lhs",
                  "lhs", 199);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 200);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_xscal"), "name", "name",
                  200);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 200);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"),
                  "resolved", "resolved", 200);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 200);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 200);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 200);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 200);
  sf_mex_assign(&c19_rhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs200), "rhs",
                  "rhs", 200);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs200), "lhs",
                  "lhs", 200);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 201);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 201);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 201);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 201);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 201);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 201);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 201);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 201);
  sf_mex_assign(&c19_rhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs201), "rhs",
                  "rhs", 201);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs201), "lhs",
                  "lhs", 201);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 202);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  202);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 202);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 202);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 202);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 202);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 202);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 202);
  sf_mex_assign(&c19_rhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs202), "rhs",
                  "rhs", 202);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs202), "lhs",
                  "lhs", 202);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 203);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("min"), "name", "name", 203);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 203);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 203);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1311276918U), "fileTimeLo",
                  "fileTimeLo", 203);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 203);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 203);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 203);
  sf_mex_assign(&c19_rhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs203), "rhs",
                  "rhs", 203);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs203), "lhs",
                  "lhs", 203);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 204);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 204);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 204);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 204);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 204);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 204);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 204);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 204);
  sf_mex_assign(&c19_rhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs204), "rhs",
                  "rhs", 204);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs204), "lhs",
                  "lhs", 204);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 205);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 205);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 205);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 205);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 205);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 205);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 205);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 205);
  sf_mex_assign(&c19_rhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs205), "rhs",
                  "rhs", 205);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs205), "lhs",
                  "lhs", 205);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 206);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 206);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 206);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 206);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 206);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 206);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 206);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 206);
  sf_mex_assign(&c19_rhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs206), "rhs",
                  "rhs", 206);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs206), "lhs",
                  "lhs", 206);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 207);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 207);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 207);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 207);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 207);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 207);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 207);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 207);
  sf_mex_assign(&c19_rhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs207), "rhs",
                  "rhs", 207);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs207), "lhs",
                  "lhs", 207);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 208);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 208);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 208);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 208);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1397279022U), "fileTimeLo",
                  "fileTimeLo", 208);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 208);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 208);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 208);
  sf_mex_assign(&c19_rhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs208), "rhs",
                  "rhs", 208);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs208), "lhs",
                  "lhs", 208);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 209);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 209);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 209);
  sf_mex_addfield(*c19_info, c19_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 209);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 209);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 209);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 209);
  sf_mex_addfield(*c19_info, c19_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 209);
  sf_mex_assign(&c19_rhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c19_lhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_rhs209), "rhs",
                  "rhs", 209);
  sf_mex_addfield(*c19_info, sf_mex_duplicatearraysafe(&c19_lhs209), "lhs",
                  "lhs", 209);
  sf_mex_destroy(&c19_rhs192);
  sf_mex_destroy(&c19_lhs192);
  sf_mex_destroy(&c19_rhs193);
  sf_mex_destroy(&c19_lhs193);
  sf_mex_destroy(&c19_rhs194);
  sf_mex_destroy(&c19_lhs194);
  sf_mex_destroy(&c19_rhs195);
  sf_mex_destroy(&c19_lhs195);
  sf_mex_destroy(&c19_rhs196);
  sf_mex_destroy(&c19_lhs196);
  sf_mex_destroy(&c19_rhs197);
  sf_mex_destroy(&c19_lhs197);
  sf_mex_destroy(&c19_rhs198);
  sf_mex_destroy(&c19_lhs198);
  sf_mex_destroy(&c19_rhs199);
  sf_mex_destroy(&c19_lhs199);
  sf_mex_destroy(&c19_rhs200);
  sf_mex_destroy(&c19_lhs200);
  sf_mex_destroy(&c19_rhs201);
  sf_mex_destroy(&c19_lhs201);
  sf_mex_destroy(&c19_rhs202);
  sf_mex_destroy(&c19_lhs202);
  sf_mex_destroy(&c19_rhs203);
  sf_mex_destroy(&c19_lhs203);
  sf_mex_destroy(&c19_rhs204);
  sf_mex_destroy(&c19_lhs204);
  sf_mex_destroy(&c19_rhs205);
  sf_mex_destroy(&c19_lhs205);
  sf_mex_destroy(&c19_rhs206);
  sf_mex_destroy(&c19_lhs206);
  sf_mex_destroy(&c19_rhs207);
  sf_mex_destroy(&c19_lhs207);
  sf_mex_destroy(&c19_rhs208);
  sf_mex_destroy(&c19_lhs208);
  sf_mex_destroy(&c19_rhs209);
  sf_mex_destroy(&c19_lhs209);
}

static void c19_eye(SFc19_SS6_Estimation2InstanceStruct *chartInstance, real_T
                    c19_I[49])
{
  int32_T c19_i184;
  int32_T c19_k;
  int32_T c19_b_k;
  for (c19_i184 = 0; c19_i184 < 49; c19_i184++) {
    c19_I[c19_i184] = 0.0;
  }

  c19_eml_switch_helper(chartInstance);
  for (c19_k = 1; c19_k < 8; c19_k++) {
    c19_b_k = c19_k;
    c19_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c19_b_k), 1, 7, 1, 0) + 7 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_k), 1, 7, 2, 0) - 1))
      - 1] = 1.0;
  }
}

static void c19_eml_switch_helper(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static boolean_T c19_use_refblas(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
  return false;
}

static void c19_threshold(SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c19_b_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_c_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_d_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_e_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_pinv(SFc19_SS6_Estimation2InstanceStruct *chartInstance, real_T
                     c19_A[4], real_T c19_X[4])
{
  int32_T c19_i185;
  int32_T c19_k;
  int32_T c19_b_k;
  real_T c19_x;
  real_T c19_b_x;
  boolean_T c19_b;
  boolean_T c19_b0;
  real_T c19_c_x;
  boolean_T c19_b_b;
  boolean_T c19_b1;
  boolean_T c19_c_b;
  int32_T c19_i186;
  real_T c19_b_A[4];
  real_T c19_V[4];
  real_T c19_s[2];
  real_T c19_U[4];
  int32_T c19_i187;
  real_T c19_S[4];
  int32_T c19_c_k;
  real_T c19_d_k;
  real_T c19_tol;
  int32_T c19_r;
  int32_T c19_e_k;
  int32_T c19_f_k;
  int32_T c19_a;
  int32_T c19_b_a;
  int32_T c19_vcol;
  int32_T c19_b_r;
  int32_T c19_d_b;
  int32_T c19_e_b;
  boolean_T c19_overflow;
  int32_T c19_j;
  int32_T c19_b_j;
  real_T c19_y;
  real_T c19_b_y;
  real_T c19_z;
  int32_T c19_c_a;
  int32_T c19_d_a;
  int32_T c19_i188;
  real_T c19_b_V[4];
  int32_T c19_i189;
  real_T c19_b_U[4];
  boolean_T exitg1;
  for (c19_i185 = 0; c19_i185 < 4; c19_i185++) {
    c19_X[c19_i185] = 0.0;
  }

  c19_eml_switch_helper(chartInstance);
  for (c19_k = 1; c19_k < 5; c19_k++) {
    c19_b_k = c19_k;
    c19_x = c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_b_k), 1, 4, 1, 0) - 1];
    c19_b_x = c19_x;
    c19_b = muDoubleScalarIsInf(c19_b_x);
    c19_b0 = !c19_b;
    c19_c_x = c19_x;
    c19_b_b = muDoubleScalarIsNaN(c19_c_x);
    c19_b1 = !c19_b_b;
    c19_c_b = (c19_b0 && c19_b1);
    if (!c19_c_b) {
      c19_eml_error(chartInstance);
    }
  }

  for (c19_i186 = 0; c19_i186 < 4; c19_i186++) {
    c19_b_A[c19_i186] = c19_A[c19_i186];
  }

  c19_eml_xgesvd(chartInstance, c19_b_A, c19_U, c19_s, c19_V);
  for (c19_i187 = 0; c19_i187 < 4; c19_i187++) {
    c19_S[c19_i187] = 0.0;
  }

  for (c19_c_k = 0; c19_c_k < 2; c19_c_k++) {
    c19_d_k = 1.0 + (real_T)c19_c_k;
    c19_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             c19_d_k), 1, 2, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", c19_d_k), 1, 2, 2, 0) - 1) << 1)) - 1] =
      c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c19_d_k), 1, 2, 1, 0) - 1];
  }

  c19_eps(chartInstance);
  c19_tol = 2.0 * c19_S[0] * 2.2204460492503131E-16;
  c19_r = 0;
  c19_eml_switch_helper(chartInstance);
  c19_e_k = 1;
  exitg1 = false;
  while ((exitg1 == false) && (c19_e_k < 3)) {
    c19_f_k = c19_e_k;
    if (!(c19_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_f_k), 1, 2, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_f_k), 1, 2, 2, 0) - 1) <<
           1)) - 1] > c19_tol)) {
      exitg1 = true;
    } else {
      c19_a = c19_r;
      c19_b_a = c19_a + 1;
      c19_r = c19_b_a;
      c19_e_k++;
    }
  }

  if (c19_r > 0) {
    c19_vcol = 1;
    c19_b_r = c19_r;
    c19_d_b = c19_b_r;
    c19_e_b = c19_d_b;
    if (1 > c19_e_b) {
      c19_overflow = false;
    } else {
      c19_eml_switch_helper(chartInstance);
      c19_eml_switch_helper(chartInstance);
      c19_overflow = (c19_e_b > 2147483646);
    }

    if (c19_overflow) {
      c19_check_forloop_overflow_error(chartInstance, c19_overflow);
    }

    for (c19_j = 1; c19_j <= c19_b_r; c19_j++) {
      c19_b_j = c19_j;
      c19_y = c19_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", (real_T)c19_b_j), 1, 2, 1, 0) +
                     ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c19_b_j), 1, 2, 2, 0) - 1) << 1)) - 1];
      c19_b_y = c19_y;
      c19_z = 1.0 / c19_b_y;
      c19_b_eml_xscal(chartInstance, c19_z, c19_V, c19_vcol);
      c19_c_a = c19_vcol;
      c19_d_a = c19_c_a + 2;
      c19_vcol = c19_d_a;
    }

    for (c19_i188 = 0; c19_i188 < 4; c19_i188++) {
      c19_b_V[c19_i188] = c19_V[c19_i188];
    }

    for (c19_i189 = 0; c19_i189 < 4; c19_i189++) {
      c19_b_U[c19_i189] = c19_U[c19_i189];
    }

    c19_b_eml_xgemm(chartInstance, c19_r, c19_b_V, c19_b_U, c19_X);
  }
}

static void c19_f_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_eml_error(SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  int32_T c19_i190;
  static char_T c19_cv0[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 's', 'v', 'd', '_', 'm', 'a', 't', 'r', 'i', 'x', 'W', 'i',
    't', 'h', 'N', 'a', 'N', 'I', 'n', 'f' };

  char_T c19_u[33];
  const mxArray *c19_y = NULL;
  (void)chartInstance;
  for (c19_i190 = 0; c19_i190 < 33; c19_i190++) {
    c19_u[c19_i190] = c19_cv0[c19_i190];
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 10, 0U, 1U, 0U, 2, 1, 33),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c19_y));
}

static void c19_eml_xgesvd(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_A[4], real_T c19_U[4], real_T c19_S[2], real_T c19_V[4])
{
  int32_T c19_i191;
  real_T c19_b_A[4];
  int32_T c19_i192;
  int32_T c19_i193;
  real_T c19_Vf[4];
  boolean_T c19_apply_transform;
  int32_T c19_i194;
  real_T c19_c_A[4];
  real_T c19_nrm;
  real_T c19_absx;
  real_T c19_d;
  real_T c19_y;
  real_T c19_s[2];
  real_T c19_a;
  real_T c19_x;
  real_T c19_b_x;
  real_T c19_b_y;
  real_T c19_B;
  real_T c19_c_y;
  real_T c19_d_y;
  real_T c19_e_y;
  real_T c19_f_y;
  real_T c19_b_a;
  real_T c19_c_a;
  int32_T c19_k;
  int32_T c19_b_k;
  int32_T c19_c_k;
  int32_T c19_d_k;
  real_T c19_d_A;
  real_T c19_b_B;
  real_T c19_c_x;
  real_T c19_g_y;
  real_T c19_d_x;
  real_T c19_h_y;
  real_T c19_e_x;
  real_T c19_i_y;
  real_T c19_j_y;
  int32_T c19_i195;
  real_T c19_e_A[4];
  int32_T c19_i196;
  real_T c19_f_A[4];
  real_T c19_t;
  real_T c19_g_A;
  real_T c19_c_B;
  real_T c19_f_x;
  real_T c19_k_y;
  real_T c19_g_x;
  real_T c19_l_y;
  real_T c19_h_x;
  real_T c19_m_y;
  real_T c19_n_y;
  int32_T c19_ii;
  int32_T c19_b_ii;
  int32_T c19_m;
  real_T c19_e[2];
  int32_T c19_c_ii;
  int32_T c19_i197;
  real_T c19_b_U[4];
  int32_T c19_i198;
  real_T c19_c_U[4];
  real_T c19_h_A;
  real_T c19_d_B;
  real_T c19_i_x;
  real_T c19_o_y;
  real_T c19_j_x;
  real_T c19_p_y;
  real_T c19_k_x;
  real_T c19_q_y;
  real_T c19_r_y;
  int32_T c19_d_ii;
  int32_T c19_e_ii;
  int32_T c19_q;
  int32_T c19_b_q;
  int32_T c19_qp1;
  int32_T c19_pmq;
  int32_T c19_qp1q;
  int32_T c19_b_qp1;
  int32_T c19_d_a;
  int32_T c19_e_a;
  boolean_T c19_overflow;
  int32_T c19_jj;
  int32_T c19_b_jj;
  int32_T c19_qp1jj;
  int32_T c19_i199;
  real_T c19_b_Vf[4];
  int32_T c19_i200;
  real_T c19_c_Vf[4];
  real_T c19_i_A;
  real_T c19_e_B;
  real_T c19_l_x;
  real_T c19_s_y;
  real_T c19_m_x;
  real_T c19_t_y;
  real_T c19_n_x;
  real_T c19_u_y;
  real_T c19_v_y;
  int32_T c19_f_ii;
  int32_T c19_c_q;
  real_T c19_rt;
  real_T c19_j_A;
  real_T c19_f_B;
  real_T c19_o_x;
  real_T c19_w_y;
  real_T c19_p_x;
  real_T c19_x_y;
  real_T c19_q_x;
  real_T c19_y_y;
  real_T c19_r;
  real_T c19_k_A;
  real_T c19_g_B;
  real_T c19_r_x;
  real_T c19_ab_y;
  real_T c19_s_x;
  real_T c19_bb_y;
  real_T c19_t_x;
  real_T c19_cb_y;
  real_T c19_db_y;
  int32_T c19_colq;
  real_T c19_l_A;
  real_T c19_h_B;
  real_T c19_u_x;
  real_T c19_eb_y;
  real_T c19_v_x;
  real_T c19_fb_y;
  real_T c19_w_x;
  real_T c19_gb_y;
  int32_T c19_colqp1;
  real_T c19_iter;
  real_T c19_snorm;
  int32_T c19_g_ii;
  real_T c19_varargin_1;
  real_T c19_varargin_2;
  real_T c19_b_varargin_2;
  real_T c19_varargin_3;
  real_T c19_x_x;
  real_T c19_hb_y;
  real_T c19_y_x;
  real_T c19_ib_y;
  real_T c19_xk;
  real_T c19_yk;
  real_T c19_ab_x;
  real_T c19_jb_y;
  real_T c19_maxval;
  real_T c19_b_varargin_1;
  real_T c19_c_varargin_2;
  real_T c19_d_varargin_2;
  real_T c19_b_varargin_3;
  real_T c19_bb_x;
  real_T c19_kb_y;
  real_T c19_cb_x;
  real_T c19_lb_y;
  real_T c19_b_xk;
  real_T c19_b_yk;
  real_T c19_db_x;
  real_T c19_mb_y;
  int32_T c19_i201;
  int32_T c19_f_a;
  int32_T c19_g_a;
  boolean_T c19_b_overflow;
  int32_T c19_h_ii;
  real_T c19_test0;
  real_T c19_ztest0;
  real_T c19_kase;
  int32_T c19_qs;
  int32_T c19_b_m;
  int32_T c19_d_q;
  int32_T c19_h_a;
  int32_T c19_b;
  int32_T c19_i_a;
  int32_T c19_b_b;
  boolean_T c19_c_overflow;
  int32_T c19_i_ii;
  real_T c19_test;
  real_T c19_ztest;
  real_T c19_f;
  int32_T c19_i202;
  int32_T c19_e_q;
  int32_T c19_j_a;
  int32_T c19_c_b;
  int32_T c19_k_a;
  int32_T c19_d_b;
  boolean_T c19_d_overflow;
  int32_T c19_e_k;
  int32_T c19_f_k;
  real_T c19_t1;
  real_T c19_b_t1;
  real_T c19_b_f;
  real_T c19_sn;
  real_T c19_cs;
  real_T c19_b_cs;
  real_T c19_b_sn;
  int32_T c19_km1;
  int32_T c19_colk;
  int32_T c19_colm;
  int32_T c19_qm1;
  int32_T c19_f_q;
  int32_T c19_c_m;
  int32_T c19_l_a;
  int32_T c19_e_b;
  int32_T c19_m_a;
  int32_T c19_f_b;
  boolean_T c19_e_overflow;
  int32_T c19_g_k;
  real_T c19_c_t1;
  real_T c19_unusedU0;
  real_T c19_c_sn;
  real_T c19_c_cs;
  int32_T c19_colqm1;
  int32_T c19_mm1;
  real_T c19_d1;
  real_T c19_d2;
  real_T c19_d3;
  real_T c19_d4;
  real_T c19_d5;
  real_T c19_c_varargin_1[5];
  int32_T c19_ixstart;
  real_T c19_mtmp;
  real_T c19_eb_x;
  boolean_T c19_g_b;
  int32_T c19_ix;
  int32_T c19_b_ix;
  real_T c19_fb_x;
  boolean_T c19_h_b;
  int32_T c19_n_a;
  int32_T c19_o_a;
  int32_T c19_i203;
  int32_T c19_p_a;
  int32_T c19_q_a;
  boolean_T c19_f_overflow;
  int32_T c19_c_ix;
  real_T c19_r_a;
  real_T c19_i_b;
  boolean_T c19_p;
  real_T c19_b_mtmp;
  real_T c19_scale;
  real_T c19_m_A;
  real_T c19_i_B;
  real_T c19_gb_x;
  real_T c19_nb_y;
  real_T c19_hb_x;
  real_T c19_ob_y;
  real_T c19_ib_x;
  real_T c19_pb_y;
  real_T c19_sm;
  real_T c19_n_A;
  real_T c19_j_B;
  real_T c19_jb_x;
  real_T c19_qb_y;
  real_T c19_kb_x;
  real_T c19_rb_y;
  real_T c19_lb_x;
  real_T c19_sb_y;
  real_T c19_smm1;
  real_T c19_o_A;
  real_T c19_k_B;
  real_T c19_mb_x;
  real_T c19_tb_y;
  real_T c19_nb_x;
  real_T c19_ub_y;
  real_T c19_ob_x;
  real_T c19_vb_y;
  real_T c19_emm1;
  real_T c19_p_A;
  real_T c19_l_B;
  real_T c19_pb_x;
  real_T c19_wb_y;
  real_T c19_qb_x;
  real_T c19_xb_y;
  real_T c19_rb_x;
  real_T c19_yb_y;
  real_T c19_sqds;
  real_T c19_q_A;
  real_T c19_m_B;
  real_T c19_sb_x;
  real_T c19_ac_y;
  real_T c19_tb_x;
  real_T c19_bc_y;
  real_T c19_ub_x;
  real_T c19_cc_y;
  real_T c19_eqds;
  real_T c19_r_A;
  real_T c19_vb_x;
  real_T c19_wb_x;
  real_T c19_xb_x;
  real_T c19_j_b;
  real_T c19_c;
  real_T c19_shift;
  real_T c19_s_A;
  real_T c19_n_B;
  real_T c19_yb_x;
  real_T c19_dc_y;
  real_T c19_ac_x;
  real_T c19_ec_y;
  real_T c19_bc_x;
  real_T c19_fc_y;
  real_T c19_g;
  int32_T c19_g_q;
  int32_T c19_b_mm1;
  int32_T c19_s_a;
  int32_T c19_k_b;
  int32_T c19_t_a;
  int32_T c19_l_b;
  boolean_T c19_g_overflow;
  int32_T c19_h_k;
  int32_T c19_kp1;
  real_T c19_c_f;
  real_T c19_unusedU1;
  real_T c19_d_sn;
  real_T c19_d_cs;
  int32_T c19_colkp1;
  real_T c19_d_f;
  real_T c19_unusedU2;
  real_T c19_e_sn;
  real_T c19_e_cs;
  int32_T c19_i_k;
  int32_T c19_j;
  int32_T c19_b_j;
  int32_T c19_i;
  int32_T c19_b_i;
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
  for (c19_i191 = 0; c19_i191 < 4; c19_i191++) {
    c19_b_A[c19_i191] = c19_A[c19_i191];
  }

  c19_f_eml_scalar_eg(chartInstance);
  for (c19_i192 = 0; c19_i192 < 4; c19_i192++) {
    c19_U[c19_i192] = 0.0;
  }

  for (c19_i193 = 0; c19_i193 < 4; c19_i193++) {
    c19_Vf[c19_i193] = 0.0;
  }

  c19_apply_transform = false;
  for (c19_i194 = 0; c19_i194 < 4; c19_i194++) {
    c19_c_A[c19_i194] = c19_b_A[c19_i194];
  }

  c19_nrm = c19_eml_xnrm2(chartInstance, c19_c_A);
  if (c19_nrm > 0.0) {
    c19_apply_transform = true;
    c19_absx = c19_nrm;
    c19_d = c19_b_A[0];
    if (c19_d < 0.0) {
      c19_y = -c19_absx;
    } else {
      c19_y = c19_absx;
    }

    c19_s[0] = c19_y;
    c19_a = c19_s[0];
    c19_realmin(chartInstance);
    c19_eps(chartInstance);
    c19_x = c19_a;
    c19_b_x = c19_x;
    c19_b_y = muDoubleScalarAbs(c19_b_x);
    if (c19_b_y >= 1.0020841800044864E-292) {
      c19_B = c19_a;
      c19_c_y = c19_B;
      c19_d_y = c19_c_y;
      c19_e_y = c19_d_y;
      c19_f_y = 1.0 / c19_e_y;
      c19_b_a = c19_f_y;
      c19_b_below_threshold(chartInstance);
      c19_c_a = c19_b_a;
      c19_eml_switch_helper(chartInstance);
      for (c19_k = 1; c19_k < 3; c19_k++) {
        c19_b_k = c19_k;
        c19_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_b_k), 1, 4, 1, 0) - 1] = c19_c_a *
          c19_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_b_k), 1, 4, 1, 0) - 1];
      }
    } else {
      c19_eml_switch_helper(chartInstance);
      for (c19_c_k = 1; c19_c_k < 3; c19_c_k++) {
        c19_d_k = c19_c_k;
        c19_d_A = c19_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_d_k), 1, 4, 1, 0) - 1];
        c19_b_B = c19_a;
        c19_c_x = c19_d_A;
        c19_g_y = c19_b_B;
        c19_d_x = c19_c_x;
        c19_h_y = c19_g_y;
        c19_e_x = c19_d_x;
        c19_i_y = c19_h_y;
        c19_j_y = c19_e_x / c19_i_y;
        c19_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_d_k), 1, 4, 1, 0) - 1] = c19_j_y;
      }
    }

    c19_b_A[0]++;
    c19_s[0] = -c19_s[0];
  } else {
    c19_s[0] = 0.0;
  }

  if (c19_apply_transform) {
    for (c19_i195 = 0; c19_i195 < 4; c19_i195++) {
      c19_e_A[c19_i195] = c19_b_A[c19_i195];
    }

    for (c19_i196 = 0; c19_i196 < 4; c19_i196++) {
      c19_f_A[c19_i196] = c19_b_A[c19_i196];
    }

    c19_t = c19_eml_xdotc(chartInstance, c19_e_A, c19_f_A);
    c19_g_A = c19_t;
    c19_c_B = c19_b_A[0];
    c19_f_x = c19_g_A;
    c19_k_y = c19_c_B;
    c19_g_x = c19_f_x;
    c19_l_y = c19_k_y;
    c19_h_x = c19_g_x;
    c19_m_y = c19_l_y;
    c19_n_y = c19_h_x / c19_m_y;
    c19_t = -c19_n_y;
    c19_c_eml_xaxpy(chartInstance, c19_t, c19_b_A);
  }

  c19_eml_switch_helper(chartInstance);
  for (c19_ii = 1; c19_ii < 3; c19_ii++) {
    c19_b_ii = c19_ii;
    c19_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_b_ii), 1, 2, 1, 0) - 1] = c19_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_ii), 1, 2, 1, 0) - 1];
  }

  c19_m = 1;
  c19_s[1] = c19_b_A[3];
  c19_e[0] = c19_b_A[2];
  c19_e[1] = 0.0;
  c19_eml_switch_helper(chartInstance);
  for (c19_c_ii = 1; c19_c_ii < 3; c19_c_ii++) {
    c19_b_ii = c19_c_ii;
    c19_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_b_ii), 1, 2, 1, 0) + 1] = 0.0;
  }

  c19_U[3] = 1.0;
  if (c19_s[0] != 0.0) {
    for (c19_i197 = 0; c19_i197 < 4; c19_i197++) {
      c19_b_U[c19_i197] = c19_U[c19_i197];
    }

    for (c19_i198 = 0; c19_i198 < 4; c19_i198++) {
      c19_c_U[c19_i198] = c19_U[c19_i198];
    }

    c19_t = c19_eml_xdotc(chartInstance, c19_b_U, c19_c_U);
    c19_h_A = c19_t;
    c19_d_B = c19_U[0];
    c19_i_x = c19_h_A;
    c19_o_y = c19_d_B;
    c19_j_x = c19_i_x;
    c19_p_y = c19_o_y;
    c19_k_x = c19_j_x;
    c19_q_y = c19_p_y;
    c19_r_y = c19_k_x / c19_q_y;
    c19_t = -c19_r_y;
    c19_c_eml_xaxpy(chartInstance, c19_t, c19_U);
    c19_eml_switch_helper(chartInstance);
    for (c19_d_ii = 1; c19_d_ii < 3; c19_d_ii++) {
      c19_b_ii = c19_d_ii;
      c19_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c19_b_ii), 1, 2, 1, 0) - 1] = -c19_U[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_ii), 1, 2, 1, 0) - 1];
    }

    c19_U[0]++;
  } else {
    c19_eml_switch_helper(chartInstance);
    for (c19_e_ii = 1; c19_e_ii < 3; c19_e_ii++) {
      c19_b_ii = c19_e_ii;
      c19_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c19_b_ii), 1, 2, 1, 0) - 1] = 0.0;
    }

    c19_U[0] = 1.0;
  }

  c19_eml_switch_helper(chartInstance);
  for (c19_q = 2; c19_q > 0; c19_q--) {
    c19_b_q = c19_q;
    if (c19_b_q <= 0) {
      if (c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_b_q), 1, 2, 1, 0) - 1] != 0.0) {
        c19_qp1 = c19_b_q + 1;
        c19_pmq = 2 - c19_b_q;
        c19_qp1q = c19_qp1 + ((c19_b_q - 1) << 1);
        c19_b_qp1 = c19_qp1;
        c19_d_a = c19_b_qp1;
        c19_e_a = c19_d_a;
        if (c19_e_a > 2) {
          c19_overflow = false;
        } else {
          c19_eml_switch_helper(chartInstance);
          c19_eml_switch_helper(chartInstance);
          c19_overflow = false;
        }

        if (c19_overflow) {
          c19_check_forloop_overflow_error(chartInstance, c19_overflow);
        }

        for (c19_jj = c19_b_qp1; c19_jj < 3; c19_jj++) {
          c19_b_jj = c19_jj - 1;
          c19_qp1jj = c19_qp1 + (c19_b_jj << 1);
          for (c19_i199 = 0; c19_i199 < 4; c19_i199++) {
            c19_b_Vf[c19_i199] = c19_Vf[c19_i199];
          }

          for (c19_i200 = 0; c19_i200 < 4; c19_i200++) {
            c19_c_Vf[c19_i200] = c19_Vf[c19_i200];
          }

          c19_t = c19_b_eml_xdotc(chartInstance, c19_pmq, c19_b_Vf, c19_qp1q,
            c19_c_Vf, c19_qp1jj);
          c19_i_A = c19_t;
          c19_e_B = c19_Vf[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c19_qp1q), 1, 4, 1, 0) - 1];
          c19_l_x = c19_i_A;
          c19_s_y = c19_e_B;
          c19_m_x = c19_l_x;
          c19_t_y = c19_s_y;
          c19_n_x = c19_m_x;
          c19_u_y = c19_t_y;
          c19_v_y = c19_n_x / c19_u_y;
          c19_t = -c19_v_y;
          c19_d_eml_xaxpy(chartInstance, c19_pmq, c19_t, c19_qp1q, c19_Vf,
                          c19_qp1jj);
        }
      }
    }

    c19_eml_switch_helper(chartInstance);
    for (c19_f_ii = 1; c19_f_ii < 3; c19_f_ii++) {
      c19_b_ii = c19_f_ii;
      c19_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c19_b_ii), 1, 2, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
                 "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 2,
                 0) - 1) << 1)) - 1] = 0.0;
    }

    c19_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c19_b_q), 1, 2, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 2, 0) - 1)
             << 1)) - 1] = 1.0;
  }

  c19_eml_switch_helper(chartInstance);
  for (c19_c_q = 1; c19_c_q < 3; c19_c_q++) {
    c19_b_q = c19_c_q;
    if (c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_b_q), 1, 2, 1, 0) - 1] != 0.0) {
      c19_rt = c19_abs(chartInstance, c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 1, 0) - 1]);
      c19_j_A = c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 1, 0) - 1];
      c19_f_B = c19_rt;
      c19_o_x = c19_j_A;
      c19_w_y = c19_f_B;
      c19_p_x = c19_o_x;
      c19_x_y = c19_w_y;
      c19_q_x = c19_p_x;
      c19_y_y = c19_x_y;
      c19_r = c19_q_x / c19_y_y;
      c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c19_b_q), 1, 2, 1, 0) - 1] = c19_rt;
      if (c19_b_q < 2) {
        c19_k_A = c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 1, 0) - 1];
        c19_g_B = c19_r;
        c19_r_x = c19_k_A;
        c19_ab_y = c19_g_B;
        c19_s_x = c19_r_x;
        c19_bb_y = c19_ab_y;
        c19_t_x = c19_s_x;
        c19_cb_y = c19_bb_y;
        c19_db_y = c19_t_x / c19_cb_y;
        c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_b_q), 1, 2, 1, 0) - 1] = c19_db_y;
      }

      if (c19_b_q <= 2) {
        c19_colq = (c19_b_q - 1) << 1;
        c19_b_eml_xscal(chartInstance, c19_r, c19_U, c19_colq + 1);
      }
    }

    if (c19_b_q < 2) {
      if (c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_b_q), 1, 2, 1, 0) - 1] != 0.0) {
        c19_rt = c19_abs(chartInstance, c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 1, 0) - 1]);
        c19_l_A = c19_rt;
        c19_h_B = c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 1, 0) - 1];
        c19_u_x = c19_l_A;
        c19_eb_y = c19_h_B;
        c19_v_x = c19_u_x;
        c19_fb_y = c19_eb_y;
        c19_w_x = c19_v_x;
        c19_gb_y = c19_fb_y;
        c19_r = c19_w_x / c19_gb_y;
        c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_b_q), 1, 2, 1, 0) - 1] = c19_rt;
        c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c19_b_q + 1)), 1, 2, 1, 0) - 1] =
          c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c19_b_q + 1)), 1, 2, 1, 0) - 1] * c19_r;
        c19_colqp1 = c19_b_q << 1;
        c19_b_eml_xscal(chartInstance, c19_r, c19_Vf, c19_colqp1 + 1);
      }
    }
  }

  c19_iter = 0.0;
  c19_realmin(chartInstance);
  c19_eps(chartInstance);
  c19_snorm = 0.0;
  c19_eml_switch_helper(chartInstance);
  for (c19_g_ii = 1; c19_g_ii < 3; c19_g_ii++) {
    c19_b_ii = c19_g_ii;
    c19_varargin_1 = c19_abs(chartInstance, c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_ii),
      1, 2, 1, 0) - 1]);
    c19_varargin_2 = c19_abs(chartInstance, c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_ii),
      1, 2, 1, 0) - 1]);
    c19_b_varargin_2 = c19_varargin_1;
    c19_varargin_3 = c19_varargin_2;
    c19_x_x = c19_b_varargin_2;
    c19_hb_y = c19_varargin_3;
    c19_y_x = c19_x_x;
    c19_ib_y = c19_hb_y;
    c19_g_eml_scalar_eg(chartInstance);
    c19_xk = c19_y_x;
    c19_yk = c19_ib_y;
    c19_ab_x = c19_xk;
    c19_jb_y = c19_yk;
    c19_g_eml_scalar_eg(chartInstance);
    c19_maxval = muDoubleScalarMax(c19_ab_x, c19_jb_y);
    c19_b_varargin_1 = c19_snorm;
    c19_c_varargin_2 = c19_maxval;
    c19_d_varargin_2 = c19_b_varargin_1;
    c19_b_varargin_3 = c19_c_varargin_2;
    c19_bb_x = c19_d_varargin_2;
    c19_kb_y = c19_b_varargin_3;
    c19_cb_x = c19_bb_x;
    c19_lb_y = c19_kb_y;
    c19_g_eml_scalar_eg(chartInstance);
    c19_b_xk = c19_cb_x;
    c19_b_yk = c19_lb_y;
    c19_db_x = c19_b_xk;
    c19_mb_y = c19_b_yk;
    c19_g_eml_scalar_eg(chartInstance);
    c19_snorm = muDoubleScalarMax(c19_db_x, c19_mb_y);
  }

  exitg1 = false;
  while ((exitg1 == false) && (c19_m + 1 > 0)) {
    if (c19_iter >= 75.0) {
      c19_b_eml_error(chartInstance);
      exitg1 = true;
    } else {
      c19_b_q = c19_m;
      c19_i201 = c19_m;
      c19_f_a = c19_i201;
      c19_g_a = c19_f_a;
      if (c19_g_a < 0) {
        c19_b_overflow = false;
      } else {
        c19_eml_switch_helper(chartInstance);
        c19_eml_switch_helper(chartInstance);
        c19_b_overflow = false;
      }

      if (c19_b_overflow) {
        c19_check_forloop_overflow_error(chartInstance, c19_b_overflow);
      }

      c19_h_ii = c19_i201;
      guard3 = false;
      guard4 = false;
      exitg5 = false;
      while ((exitg5 == false) && (c19_h_ii > -1)) {
        c19_b_ii = c19_h_ii;
        c19_b_q = c19_b_ii;
        if (c19_b_ii == 0) {
          exitg5 = true;
        } else {
          c19_test0 = c19_abs(chartInstance, c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
            "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_ii), 1, 2, 1, 0) -
                              1]) + c19_abs(chartInstance,
            c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c19_b_ii + 1)), 1, 2, 1, 0) - 1]);
          c19_ztest0 = c19_abs(chartInstance, c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
            "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_ii), 1, 2, 1, 0) -
                               1]);
          c19_eps(chartInstance);
          if (c19_ztest0 <= 2.2204460492503131E-16 * c19_test0) {
            guard4 = true;
            exitg5 = true;
          } else if (c19_ztest0 <= 1.0020841800044864E-292) {
            guard4 = true;
            exitg5 = true;
          } else {
            guard11 = false;
            if (c19_iter > 20.0) {
              c19_eps(chartInstance);
              if (c19_ztest0 <= 2.2204460492503131E-16 * c19_snorm) {
                guard3 = true;
                exitg5 = true;
              } else {
                guard11 = true;
              }
            } else {
              guard11 = true;
            }

            if (guard11 == true) {
              c19_h_ii--;
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
        c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_b_ii), 1, 2, 1, 0) - 1] = 0.0;
      }

      if (c19_b_q == c19_m) {
        c19_kase = 4.0;
      } else {
        c19_qs = c19_m + 1;
        c19_b_m = c19_m + 1;
        c19_d_q = c19_b_q;
        c19_h_a = c19_b_m;
        c19_b = c19_d_q;
        c19_i_a = c19_h_a;
        c19_b_b = c19_b;
        if (c19_i_a < c19_b_b) {
          c19_c_overflow = false;
        } else {
          c19_eml_switch_helper(chartInstance);
          c19_eml_switch_helper(chartInstance);
          c19_c_overflow = (c19_b_b < -2147483647);
        }

        if (c19_c_overflow) {
          c19_check_forloop_overflow_error(chartInstance, c19_c_overflow);
        }

        c19_i_ii = c19_b_m;
        guard2 = false;
        exitg4 = false;
        while ((exitg4 == false) && (c19_i_ii >= c19_d_q)) {
          c19_b_ii = c19_i_ii;
          c19_qs = c19_b_ii;
          if (c19_b_ii == c19_b_q) {
            exitg4 = true;
          } else {
            c19_test = 0.0;
            if (c19_b_ii < c19_m + 1) {
              c19_test = c19_abs(chartInstance,
                                 c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c19_b_ii), 1, 2, 1, 0) - 1]);
            }

            if (c19_b_ii > c19_b_q + 1) {
              c19_test += c19_abs(chartInstance,
                                  c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)(c19_b_ii - 1)), 1, 2, 1, 0) - 1]);
            }

            c19_ztest = c19_abs(chartInstance, c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK
                                ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
              c19_b_ii), 1, 2, 1, 0) - 1]);
            c19_eps(chartInstance);
            if (c19_ztest <= 2.2204460492503131E-16 * c19_test) {
              guard2 = true;
              exitg4 = true;
            } else if (c19_ztest <= 1.0020841800044864E-292) {
              guard2 = true;
              exitg4 = true;
            } else {
              c19_i_ii--;
              guard2 = false;
            }
          }
        }

        if (guard2 == true) {
          c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_b_ii), 1, 2, 1, 0) - 1] = 0.0;
        }

        if (c19_qs == c19_b_q) {
          c19_kase = 3.0;
        } else if (c19_qs == c19_m + 1) {
          c19_kase = 1.0;
        } else {
          c19_kase = 2.0;
          c19_b_q = c19_qs;
        }
      }

      c19_b_q++;
      switch ((int32_T)_SFD_INTEGER_CHECK("", c19_kase)) {
       case 1:
        c19_f = c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_m), 1, 2, 1, 0) - 1];
        c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_m), 1, 2, 1, 0) - 1] = 0.0;
        c19_i202 = c19_m;
        c19_e_q = c19_b_q;
        c19_j_a = c19_i202;
        c19_c_b = c19_e_q;
        c19_k_a = c19_j_a;
        c19_d_b = c19_c_b;
        if (c19_k_a < c19_d_b) {
          c19_d_overflow = false;
        } else {
          c19_eml_switch_helper(chartInstance);
          c19_eml_switch_helper(chartInstance);
          c19_d_overflow = (c19_d_b < -2147483647);
        }

        if (c19_d_overflow) {
          c19_check_forloop_overflow_error(chartInstance, c19_d_overflow);
        }

        for (c19_e_k = c19_i202; c19_e_k >= c19_e_q; c19_e_k--) {
          c19_f_k = c19_e_k;
          c19_t1 = c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c19_f_k), 1, 2, 1, 0) - 1];
          c19_b_t1 = c19_t1;
          c19_b_f = c19_f;
          c19_b_eml_xrotg(chartInstance, &c19_b_t1, &c19_b_f, &c19_cs, &c19_sn);
          c19_t1 = c19_b_t1;
          c19_f = c19_b_f;
          c19_b_cs = c19_cs;
          c19_b_sn = c19_sn;
          c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_f_k), 1, 2, 1, 0) - 1] = c19_t1;
          if (c19_f_k > c19_b_q) {
            c19_km1 = c19_f_k - 1;
            c19_f = -c19_b_sn * c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c19_km1), 1, 2, 1, 0) - 1];
            c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c19_km1), 1, 2, 1, 0) - 1] =
              c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c19_km1), 1, 2, 1, 0) - 1] * c19_b_cs;
          }

          c19_colk = (c19_f_k - 1) << 1;
          c19_colm = c19_m << 1;
          c19_b_eml_xrot(chartInstance, c19_Vf, c19_colk + 1, c19_colm + 1,
                         c19_b_cs, c19_b_sn);
        }
        break;

       case 2:
        c19_qm1 = c19_b_q - 1;
        c19_f = c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_qm1), 1, 2, 1, 0) - 1];
        c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_qm1), 1, 2, 1, 0) - 1] = 0.0;
        c19_f_q = c19_b_q;
        c19_c_m = c19_m + 1;
        c19_l_a = c19_f_q;
        c19_e_b = c19_c_m;
        c19_m_a = c19_l_a;
        c19_f_b = c19_e_b;
        if (c19_m_a > c19_f_b) {
          c19_e_overflow = false;
        } else {
          c19_eml_switch_helper(chartInstance);
          c19_eml_switch_helper(chartInstance);
          c19_e_overflow = (c19_f_b > 2147483646);
        }

        if (c19_e_overflow) {
          c19_check_forloop_overflow_error(chartInstance, c19_e_overflow);
        }

        for (c19_g_k = c19_f_q; c19_g_k <= c19_c_m; c19_g_k++) {
          c19_f_k = c19_g_k;
          c19_t1 = c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c19_f_k), 1, 2, 1, 0) - 1];
          c19_c_t1 = c19_t1;
          c19_unusedU0 = c19_f;
          c19_b_eml_xrotg(chartInstance, &c19_c_t1, &c19_unusedU0, &c19_c_cs,
                          &c19_c_sn);
          c19_t1 = c19_c_t1;
          c19_b_cs = c19_c_cs;
          c19_b_sn = c19_c_sn;
          c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_f_k), 1, 2, 1, 0) - 1] = c19_t1;
          c19_f = -c19_b_sn * c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c19_f_k), 1, 2, 1, 0) - 1];
          c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_f_k), 1, 2, 1, 0) - 1] =
            c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_f_k), 1, 2, 1, 0) - 1] * c19_b_cs;
          c19_colk = (c19_f_k - 1) << 1;
          c19_colqm1 = (c19_qm1 - 1) << 1;
          c19_b_eml_xrot(chartInstance, c19_U, c19_colk + 1, c19_colqm1 + 1,
                         c19_b_cs, c19_b_sn);
        }
        break;

       case 3:
        c19_mm1 = c19_m;
        c19_d1 = c19_abs(chartInstance, c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)(c19_m + 1)), 1, 2, 1, 0) - 1]);
        c19_d2 = c19_abs(chartInstance, c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_mm1), 1, 2, 1, 0) - 1]);
        c19_d3 = c19_abs(chartInstance, c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_mm1), 1, 2, 1, 0) - 1]);
        c19_d4 = c19_abs(chartInstance, c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 1, 0) - 1]);
        c19_d5 = c19_abs(chartInstance, c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 1, 0) - 1]);
        c19_c_varargin_1[0] = c19_d1;
        c19_c_varargin_1[1] = c19_d2;
        c19_c_varargin_1[2] = c19_d3;
        c19_c_varargin_1[3] = c19_d4;
        c19_c_varargin_1[4] = c19_d5;
        c19_ixstart = 1;
        c19_mtmp = c19_c_varargin_1[0];
        c19_eb_x = c19_mtmp;
        c19_g_b = muDoubleScalarIsNaN(c19_eb_x);
        if (c19_g_b) {
          c19_eml_switch_helper(chartInstance);
          c19_eml_switch_helper(chartInstance);
          c19_ix = 2;
          exitg2 = false;
          while ((exitg2 == false) && (c19_ix < 6)) {
            c19_b_ix = c19_ix;
            c19_ixstart = c19_b_ix;
            c19_fb_x = c19_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c19_b_ix), 1, 5, 1, 0) - 1];
            c19_h_b = muDoubleScalarIsNaN(c19_fb_x);
            if (!c19_h_b) {
              c19_mtmp = c19_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_ix), 1, 5, 1, 0) -
                1];
              exitg2 = true;
            } else {
              c19_ix++;
            }
          }
        }

        if (c19_ixstart < 5) {
          c19_n_a = c19_ixstart;
          c19_o_a = c19_n_a + 1;
          c19_i203 = c19_o_a;
          c19_p_a = c19_i203;
          c19_q_a = c19_p_a;
          if (c19_q_a > 5) {
            c19_f_overflow = false;
          } else {
            c19_eml_switch_helper(chartInstance);
            c19_eml_switch_helper(chartInstance);
            c19_f_overflow = false;
          }

          if (c19_f_overflow) {
            c19_check_forloop_overflow_error(chartInstance, c19_f_overflow);
          }

          for (c19_c_ix = c19_i203; c19_c_ix < 6; c19_c_ix++) {
            c19_b_ix = c19_c_ix;
            c19_r_a = c19_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c19_b_ix), 1, 5, 1, 0) - 1];
            c19_i_b = c19_mtmp;
            c19_p = (c19_r_a > c19_i_b);
            if (c19_p) {
              c19_mtmp = c19_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_ix), 1, 5, 1, 0) -
                1];
            }
          }
        }

        c19_b_mtmp = c19_mtmp;
        c19_scale = c19_b_mtmp;
        c19_m_A = c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)(c19_m + 1)), 1, 2, 1, 0) - 1];
        c19_i_B = c19_scale;
        c19_gb_x = c19_m_A;
        c19_nb_y = c19_i_B;
        c19_hb_x = c19_gb_x;
        c19_ob_y = c19_nb_y;
        c19_ib_x = c19_hb_x;
        c19_pb_y = c19_ob_y;
        c19_sm = c19_ib_x / c19_pb_y;
        c19_n_A = c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_mm1), 1, 2, 1, 0) - 1];
        c19_j_B = c19_scale;
        c19_jb_x = c19_n_A;
        c19_qb_y = c19_j_B;
        c19_kb_x = c19_jb_x;
        c19_rb_y = c19_qb_y;
        c19_lb_x = c19_kb_x;
        c19_sb_y = c19_rb_y;
        c19_smm1 = c19_lb_x / c19_sb_y;
        c19_o_A = c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_mm1), 1, 2, 1, 0) - 1];
        c19_k_B = c19_scale;
        c19_mb_x = c19_o_A;
        c19_tb_y = c19_k_B;
        c19_nb_x = c19_mb_x;
        c19_ub_y = c19_tb_y;
        c19_ob_x = c19_nb_x;
        c19_vb_y = c19_ub_y;
        c19_emm1 = c19_ob_x / c19_vb_y;
        c19_p_A = c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 1, 0) - 1];
        c19_l_B = c19_scale;
        c19_pb_x = c19_p_A;
        c19_wb_y = c19_l_B;
        c19_qb_x = c19_pb_x;
        c19_xb_y = c19_wb_y;
        c19_rb_x = c19_qb_x;
        c19_yb_y = c19_xb_y;
        c19_sqds = c19_rb_x / c19_yb_y;
        c19_q_A = c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 1, 0) - 1];
        c19_m_B = c19_scale;
        c19_sb_x = c19_q_A;
        c19_ac_y = c19_m_B;
        c19_tb_x = c19_sb_x;
        c19_bc_y = c19_ac_y;
        c19_ub_x = c19_tb_x;
        c19_cc_y = c19_bc_y;
        c19_eqds = c19_ub_x / c19_cc_y;
        c19_r_A = (c19_smm1 + c19_sm) * (c19_smm1 - c19_sm) + c19_emm1 *
          c19_emm1;
        c19_vb_x = c19_r_A;
        c19_wb_x = c19_vb_x;
        c19_xb_x = c19_wb_x;
        c19_j_b = c19_xb_x / 2.0;
        c19_c = c19_sm * c19_emm1;
        c19_c *= c19_c;
        guard1 = false;
        if (c19_j_b != 0.0) {
          guard1 = true;
        } else if (c19_c != 0.0) {
          guard1 = true;
        } else {
          c19_shift = 0.0;
        }

        if (guard1 == true) {
          c19_shift = c19_j_b * c19_j_b + c19_c;
          c19_b_sqrt(chartInstance, &c19_shift);
          if (c19_j_b < 0.0) {
            c19_shift = -c19_shift;
          }

          c19_s_A = c19_c;
          c19_n_B = c19_j_b + c19_shift;
          c19_yb_x = c19_s_A;
          c19_dc_y = c19_n_B;
          c19_ac_x = c19_yb_x;
          c19_ec_y = c19_dc_y;
          c19_bc_x = c19_ac_x;
          c19_fc_y = c19_ec_y;
          c19_shift = c19_bc_x / c19_fc_y;
        }

        c19_f = (c19_sqds + c19_sm) * (c19_sqds - c19_sm) + c19_shift;
        c19_g = c19_sqds * c19_eqds;
        c19_g_q = c19_b_q;
        c19_b_mm1 = c19_mm1;
        c19_s_a = c19_g_q;
        c19_k_b = c19_b_mm1;
        c19_t_a = c19_s_a;
        c19_l_b = c19_k_b;
        if (c19_t_a > c19_l_b) {
          c19_g_overflow = false;
        } else {
          c19_eml_switch_helper(chartInstance);
          c19_eml_switch_helper(chartInstance);
          c19_g_overflow = (c19_l_b > 2147483646);
        }

        if (c19_g_overflow) {
          c19_check_forloop_overflow_error(chartInstance, c19_g_overflow);
        }

        for (c19_h_k = c19_g_q; c19_h_k <= c19_b_mm1; c19_h_k++) {
          c19_f_k = c19_h_k;
          c19_km1 = c19_f_k;
          c19_kp1 = c19_f_k + 1;
          c19_c_f = c19_f;
          c19_unusedU1 = c19_g;
          c19_b_eml_xrotg(chartInstance, &c19_c_f, &c19_unusedU1, &c19_d_cs,
                          &c19_d_sn);
          c19_f = c19_c_f;
          c19_b_cs = c19_d_cs;
          c19_b_sn = c19_d_sn;
          if (c19_f_k > c19_b_q) {
            c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)(c19_km1 - 1)), 1, 2, 1, 0) - 1] = c19_f;
          }

          c19_f = c19_b_cs * c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c19_f_k), 1, 2, 1, 0) - 1] + c19_b_sn
            * c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", (real_T)c19_f_k), 1, 2, 1, 0) - 1];
          c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_f_k), 1, 2, 1, 0) - 1] = c19_b_cs *
            c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_f_k), 1, 2, 1, 0) - 1] - c19_b_sn *
            c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_f_k), 1, 2, 1, 0) - 1];
          c19_g = c19_b_sn * c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c19_kp1), 1, 2, 1, 0) - 1];
          c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_kp1), 1, 2, 1, 0) - 1] =
            c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_kp1), 1, 2, 1, 0) - 1] * c19_b_cs;
          c19_colk = 1 + ((c19_f_k - 1) << 1);
          c19_colkp1 = 1 + (c19_f_k << 1);
          c19_b_eml_xrot(chartInstance, c19_Vf, c19_colk, c19_colkp1, c19_b_cs,
                         c19_b_sn);
          c19_d_f = c19_f;
          c19_unusedU2 = c19_g;
          c19_b_eml_xrotg(chartInstance, &c19_d_f, &c19_unusedU2, &c19_e_cs,
                          &c19_e_sn);
          c19_f = c19_d_f;
          c19_b_cs = c19_e_cs;
          c19_b_sn = c19_e_sn;
          c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_f_k), 1, 2, 1, 0) - 1] = c19_f;
          c19_f = c19_b_cs * c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c19_f_k), 1, 2, 1, 0) - 1] + c19_b_sn
            * c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", (real_T)c19_kp1), 1, 2, 1, 0) - 1];
          c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_kp1), 1, 2, 1, 0) - 1] = -c19_b_sn *
            c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_f_k), 1, 2, 1, 0) - 1] + c19_b_cs *
            c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_kp1), 1, 2, 1, 0) - 1];
          c19_g = c19_b_sn * c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c19_kp1), 1, 2, 1, 0) - 1];
          c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_kp1), 1, 2, 1, 0) - 1] =
            c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_kp1), 1, 2, 1, 0) - 1] * c19_b_cs;
          if (c19_f_k < 2) {
            c19_b_eml_xrot(chartInstance, c19_U, c19_colk, c19_colkp1, c19_b_cs,
                           c19_b_sn);
          }
        }

        c19_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_m), 1, 2, 1, 0) - 1] = c19_f;
        c19_iter++;
        break;

       default:
        if (c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c19_b_q), 1, 2, 1, 0) - 1] < 0.0) {
          c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_b_q), 1, 2, 1, 0) - 1] =
            -c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", (real_T)c19_b_q), 1, 2, 1, 0) - 1];
          c19_colq = (c19_b_q - 1) << 1;
          c19_f_eml_scalar_eg(chartInstance);
          c19_b_eml_xscal(chartInstance, -1.0, c19_Vf, c19_colq + 1);
        }

        c19_qp1 = c19_b_q + 1;
        exitg3 = false;
        while ((exitg3 == false) && (c19_b_q < 2)) {
          if (c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c19_b_q), 1, 2, 1, 0) - 1] <
              c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c19_qp1), 1, 2, 1, 0) - 1]) {
            c19_rt = c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c19_b_q), 1, 2, 1, 0) - 1];
            c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c19_b_q), 1, 2, 1, 0) - 1] =
              c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c19_qp1), 1, 2, 1, 0) - 1];
            c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c19_qp1), 1, 2, 1, 0) - 1] = c19_rt;
            if (c19_b_q < 2) {
              c19_colq = (c19_b_q - 1) << 1;
              c19_colqp1 = c19_b_q << 1;
              c19_b_eml_xswap(chartInstance, c19_Vf, c19_colq + 1, c19_colqp1 +
                              1);
            }

            if (c19_b_q < 2) {
              c19_colq = (c19_b_q - 1) << 1;
              c19_colqp1 = c19_b_q << 1;
              c19_b_eml_xswap(chartInstance, c19_U, c19_colq + 1, c19_colqp1 + 1);
            }

            c19_b_q = c19_qp1;
            c19_qp1 = c19_b_q + 1;
          } else {
            exitg3 = true;
          }
        }

        c19_iter = 0.0;
        c19_m--;
        break;
      }
    }
  }

  c19_eml_switch_helper(chartInstance);
  for (c19_i_k = 1; c19_i_k < 3; c19_i_k++) {
    c19_f_k = c19_i_k;
    c19_S[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_f_k), 1, 2, 1, 0) - 1] = c19_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_f_k), 1, 2, 1, 0) - 1];
  }

  c19_eml_switch_helper(chartInstance);
  for (c19_j = 1; c19_j < 3; c19_j++) {
    c19_b_j = c19_j;
    c19_eml_switch_helper(chartInstance);
    for (c19_i = 1; c19_i < 3; c19_i++) {
      c19_b_i = c19_i;
      c19_V[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c19_b_i), 1, 2, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
                (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_j), 1, 2, 2, 0) -
               1) << 1)) - 1] = c19_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c19_b_i), 1, 2, 1, 0) +
        ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c19_b_j), 1, 2, 2, 0) - 1) << 1)) - 1];
    }
  }
}

static real_T c19_eml_xnrm2(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4])
{
  real_T c19_y;
  real_T c19_scale;
  int32_T c19_k;
  int32_T c19_b_k;
  real_T c19_b_x;
  real_T c19_c_x;
  real_T c19_absxk;
  real_T c19_t;
  c19_below_threshold(chartInstance);
  c19_y = 0.0;
  c19_realmin(chartInstance);
  c19_scale = 2.2250738585072014E-308;
  c19_eml_switch_helper(chartInstance);
  for (c19_k = 1; c19_k < 3; c19_k++) {
    c19_b_k = c19_k;
    c19_b_x = c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c19_b_k), 1, 4, 1, 0) - 1];
    c19_c_x = c19_b_x;
    c19_absxk = muDoubleScalarAbs(c19_c_x);
    if (c19_absxk > c19_scale) {
      c19_t = c19_scale / c19_absxk;
      c19_y = 1.0 + c19_y * c19_t * c19_t;
      c19_scale = c19_absxk;
    } else {
      c19_t = c19_absxk / c19_scale;
      c19_y += c19_t * c19_t;
    }
  }

  return c19_scale * muDoubleScalarSqrt(c19_y);
}

static void c19_below_threshold(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_realmin(SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c19_abs(SFc19_SS6_Estimation2InstanceStruct *chartInstance, real_T
                      c19_x)
{
  real_T c19_b_x;
  (void)chartInstance;
  c19_b_x = c19_x;
  return muDoubleScalarAbs(c19_b_x);
}

static void c19_eps(SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c19_b_below_threshold(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  c19_b_eml_switch_helper(chartInstance);
}

static void c19_b_eml_switch_helper(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c19_eml_xdotc(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4], real_T c19_y[4])
{
  real_T c19_d;
  int32_T c19_ix;
  int32_T c19_iy;
  int32_T c19_k;
  int32_T c19_a;
  int32_T c19_b_a;
  c19_b_threshold(chartInstance);
  c19_d = 0.0;
  c19_ix = 1;
  c19_iy = 3;
  c19_eml_switch_helper(chartInstance);
  for (c19_k = 1; c19_k < 3; c19_k++) {
    c19_d += c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c19_ix), 1, 4, 1, 0) - 1] * c19_y[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_iy), 1, 4, 1, 0) - 1];
    c19_a = c19_ix + 1;
    c19_ix = c19_a;
    c19_b_a = c19_iy + 1;
    c19_iy = c19_b_a;
  }

  return c19_d;
}

static void c19_b_threshold(SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c19_eml_xaxpy(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_a, real_T c19_y[4], real_T c19_b_y[4])
{
  int32_T c19_i204;
  for (c19_i204 = 0; c19_i204 < 4; c19_i204++) {
    c19_b_y[c19_i204] = c19_y[c19_i204];
  }

  c19_c_eml_xaxpy(chartInstance, c19_a, c19_b_y);
}

static void c19_c_threshold(SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c19_check_forloop_overflow_error(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, boolean_T c19_overflow)
{
  int32_T c19_i205;
  static char_T c19_cv1[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c19_u[34];
  const mxArray *c19_y = NULL;
  int32_T c19_i206;
  static char_T c19_cv2[5] = { 'i', 'n', 't', '3', '2' };

  char_T c19_b_u[5];
  const mxArray *c19_b_y = NULL;
  (void)chartInstance;
  if (!c19_overflow) {
  } else {
    for (c19_i205 = 0; c19_i205 < 34; c19_i205++) {
      c19_u[c19_i205] = c19_cv1[c19_i205];
    }

    c19_y = NULL;
    sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  false);
    for (c19_i206 = 0; c19_i206 < 5; c19_i206++) {
      c19_b_u[c19_i206] = c19_cv2[c19_i206];
    }

    c19_b_y = NULL;
    sf_mex_assign(&c19_b_y, sf_mex_create("y", c19_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 2U, 14, c19_y, 14, c19_b_y));
  }
}

static real_T c19_b_eml_xdotc(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c19_n, real_T c19_x[4], int32_T c19_ix0, real_T c19_y[4], int32_T
  c19_iy0)
{
  real_T c19_d;
  int32_T c19_b_n;
  int32_T c19_b_ix0;
  int32_T c19_b_iy0;
  int32_T c19_c_n;
  int32_T c19_c_ix0;
  int32_T c19_c_iy0;
  int32_T c19_d_n;
  int32_T c19_d_ix0;
  int32_T c19_d_iy0;
  int32_T c19_e_n;
  int32_T c19_e_ix0;
  int32_T c19_e_iy0;
  int32_T c19_ix;
  int32_T c19_iy;
  int32_T c19_f_n;
  int32_T c19_b;
  int32_T c19_b_b;
  boolean_T c19_overflow;
  int32_T c19_k;
  int32_T c19_a;
  int32_T c19_b_a;
  c19_b_n = c19_n;
  c19_b_ix0 = c19_ix0;
  c19_b_iy0 = c19_iy0;
  c19_c_n = c19_b_n;
  c19_c_ix0 = c19_b_ix0;
  c19_c_iy0 = c19_b_iy0;
  c19_b_threshold(chartInstance);
  c19_d_n = c19_c_n;
  c19_d_ix0 = c19_c_ix0;
  c19_d_iy0 = c19_c_iy0;
  c19_e_n = c19_d_n;
  c19_e_ix0 = c19_d_ix0;
  c19_e_iy0 = c19_d_iy0;
  c19_d = 0.0;
  if (c19_e_n < 1) {
  } else {
    c19_ix = c19_e_ix0;
    c19_iy = c19_e_iy0;
    c19_f_n = c19_e_n;
    c19_b = c19_f_n;
    c19_b_b = c19_b;
    if (1 > c19_b_b) {
      c19_overflow = false;
    } else {
      c19_eml_switch_helper(chartInstance);
      c19_eml_switch_helper(chartInstance);
      c19_overflow = (c19_b_b > 2147483646);
    }

    if (c19_overflow) {
      c19_check_forloop_overflow_error(chartInstance, c19_overflow);
    }

    for (c19_k = 1; c19_k <= c19_f_n; c19_k++) {
      c19_d += c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c19_ix), 1, 4, 1, 0) - 1] *
        c19_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c19_iy), 1, 4, 1, 0) - 1];
      c19_a = c19_ix + 1;
      c19_ix = c19_a;
      c19_b_a = c19_iy + 1;
      c19_iy = c19_b_a;
    }
  }

  return c19_d;
}

static void c19_b_eml_xaxpy(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c19_n, real_T c19_a, int32_T c19_ix0, real_T c19_y[4], int32_T c19_iy0,
  real_T c19_b_y[4])
{
  int32_T c19_i207;
  for (c19_i207 = 0; c19_i207 < 4; c19_i207++) {
    c19_b_y[c19_i207] = c19_y[c19_i207];
  }

  c19_d_eml_xaxpy(chartInstance, c19_n, c19_a, c19_ix0, c19_b_y, c19_iy0);
}

static void c19_eml_xscal(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_a, real_T c19_x[4], int32_T c19_ix0, real_T c19_b_x[4])
{
  int32_T c19_i208;
  for (c19_i208 = 0; c19_i208 < 4; c19_i208++) {
    c19_b_x[c19_i208] = c19_x[c19_i208];
  }

  c19_b_eml_xscal(chartInstance, c19_a, c19_b_x, c19_ix0);
}

static void c19_g_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_b_eml_error(SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  int32_T c19_i209;
  static char_T c19_cv3[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 's', 'v', 'd', '_', 'N', 'o', 'C', 'o', 'n', 'v', 'e', 'r',
    'g', 'e', 'n', 'c', 'e' };

  char_T c19_u[30];
  const mxArray *c19_y = NULL;
  (void)chartInstance;
  for (c19_i209 = 0; c19_i209 < 30; c19_i209++) {
    c19_u[c19_i209] = c19_cv3[c19_i209];
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 10, 0U, 1U, 0U, 2, 1, 30),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c19_y));
}

static real_T c19_sqrt(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x)
{
  real_T c19_b_x;
  c19_b_x = c19_x;
  c19_b_sqrt(chartInstance, &c19_b_x);
  return c19_b_x;
}

static void c19_c_eml_error(SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  int32_T c19_i210;
  static char_T c19_cv4[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c19_u[30];
  const mxArray *c19_y = NULL;
  int32_T c19_i211;
  static char_T c19_cv5[4] = { 's', 'q', 'r', 't' };

  char_T c19_b_u[4];
  const mxArray *c19_b_y = NULL;
  (void)chartInstance;
  for (c19_i210 = 0; c19_i210 < 30; c19_i210++) {
    c19_u[c19_i210] = c19_cv4[c19_i210];
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 10, 0U, 1U, 0U, 2, 1, 30),
                false);
  for (c19_i211 = 0; c19_i211 < 4; c19_i211++) {
    c19_b_u[c19_i211] = c19_cv5[c19_i211];
  }

  c19_b_y = NULL;
  sf_mex_assign(&c19_b_y, sf_mex_create("y", c19_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c19_y, 14, c19_b_y));
}

static void c19_eml_xrotg(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_a, real_T c19_b, real_T *c19_b_a, real_T *c19_b_b, real_T *c19_c,
  real_T *c19_s)
{
  *c19_b_a = c19_a;
  *c19_b_b = c19_b;
  c19_b_eml_xrotg(chartInstance, c19_b_a, c19_b_b, c19_c, c19_s);
}

static void c19_eml_xrot(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4], int32_T c19_ix0, int32_T c19_iy0, real_T c19_c, real_T c19_s,
  real_T c19_b_x[4])
{
  int32_T c19_i212;
  for (c19_i212 = 0; c19_i212 < 4; c19_i212++) {
    c19_b_x[c19_i212] = c19_x[c19_i212];
  }

  c19_b_eml_xrot(chartInstance, c19_b_x, c19_ix0, c19_iy0, c19_c, c19_s);
}

static void c19_d_threshold(SFc19_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c19_eml_xswap(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4], int32_T c19_ix0, int32_T c19_iy0, real_T c19_b_x[4])
{
  int32_T c19_i213;
  for (c19_i213 = 0; c19_i213 < 4; c19_i213++) {
    c19_b_x[c19_i213] = c19_x[c19_i213];
  }

  c19_b_eml_xswap(chartInstance, c19_b_x, c19_ix0, c19_iy0);
}

static void c19_c_eml_switch_helper(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_eml_xgemm(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c19_k, real_T c19_A[4], real_T c19_B[4], real_T c19_C[4], real_T
  c19_b_C[4])
{
  int32_T c19_i214;
  int32_T c19_i215;
  real_T c19_b_A[4];
  int32_T c19_i216;
  real_T c19_b_B[4];
  for (c19_i214 = 0; c19_i214 < 4; c19_i214++) {
    c19_b_C[c19_i214] = c19_C[c19_i214];
  }

  for (c19_i215 = 0; c19_i215 < 4; c19_i215++) {
    c19_b_A[c19_i215] = c19_A[c19_i215];
  }

  for (c19_i216 = 0; c19_i216 < 4; c19_i216++) {
    c19_b_B[c19_i216] = c19_B[c19_i216];
  }

  c19_b_eml_xgemm(chartInstance, c19_k, c19_b_A, c19_b_B, c19_b_C);
}

static void c19_h_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_i_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_j_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c19_k_eml_scalar_eg(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c19_k_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_u;
  const mxArray *c19_y = NULL;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(int32_T *)c19_inData;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static int32_T c19_l_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  int32_T c19_y;
  int32_T c19_i217;
  (void)chartInstance;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_i217, 1, 6, 0U, 0, 0U, 0);
  c19_y = c19_i217;
  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_sfEvent;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  int32_T c19_y;
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c19_b_sfEvent = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_sfEvent),
    &c19_thisId);
  sf_mex_destroy(&c19_b_sfEvent);
  *(int32_T *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

static uint8_T c19_m_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_b_is_active_c19_SS6_Estimation2, const
  char_T *c19_identifier)
{
  uint8_T c19_y;
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_n_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c19_b_is_active_c19_SS6_Estimation2), &c19_thisId);
  sf_mex_destroy(&c19_b_is_active_c19_SS6_Estimation2);
  return c19_y;
}

static uint8_T c19_n_emlrt_marshallIn(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  uint8_T c19_y;
  uint8_T c19_u0;
  (void)chartInstance;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_u0, 1, 3, 0U, 0, 0U, 0);
  c19_y = c19_u0;
  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_c_eml_xaxpy(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_a, real_T c19_y[4])
{
  real_T c19_b_a;
  real_T c19_c_a;
  int32_T c19_ix;
  int32_T c19_iy;
  int32_T c19_k;
  int32_T c19_d_a;
  int32_T c19_c;
  int32_T c19_e_a;
  int32_T c19_b_c;
  int32_T c19_f_a;
  int32_T c19_c_c;
  int32_T c19_g_a;
  int32_T c19_h_a;
  c19_b_a = c19_a;
  c19_c_threshold(chartInstance);
  c19_c_a = c19_b_a;
  if (c19_c_a == 0.0) {
  } else {
    c19_ix = 0;
    c19_iy = 2;
    c19_eml_switch_helper(chartInstance);
    for (c19_k = 0; c19_k < 2; c19_k++) {
      c19_d_a = c19_iy;
      c19_c = c19_d_a;
      c19_e_a = c19_iy;
      c19_b_c = c19_e_a;
      c19_f_a = c19_ix;
      c19_c_c = c19_f_a;
      c19_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c19_c + 1)), 1, 4, 1, 0) - 1] =
        c19_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c19_b_c + 1)), 1, 4, 1, 0) - 1] + c19_c_a *
        c19_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c19_c_c + 1)), 1, 4, 1, 0) - 1];
      c19_g_a = c19_ix + 1;
      c19_ix = c19_g_a;
      c19_h_a = c19_iy + 1;
      c19_iy = c19_h_a;
    }
  }
}

static void c19_d_eml_xaxpy(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c19_n, real_T c19_a, int32_T c19_ix0, real_T c19_y[4], int32_T c19_iy0)
{
  int32_T c19_b_n;
  real_T c19_b_a;
  int32_T c19_b_ix0;
  int32_T c19_b_iy0;
  int32_T c19_c_n;
  real_T c19_c_a;
  int32_T c19_c_ix0;
  int32_T c19_c_iy0;
  int32_T c19_d_a;
  int32_T c19_ix;
  int32_T c19_e_a;
  int32_T c19_iy;
  int32_T c19_f_a;
  int32_T c19_i218;
  int32_T c19_b;
  int32_T c19_b_b;
  boolean_T c19_overflow;
  int32_T c19_k;
  int32_T c19_g_a;
  int32_T c19_c;
  int32_T c19_h_a;
  int32_T c19_b_c;
  int32_T c19_i_a;
  int32_T c19_c_c;
  int32_T c19_j_a;
  int32_T c19_k_a;
  c19_b_n = c19_n;
  c19_b_a = c19_a;
  c19_b_ix0 = c19_ix0;
  c19_b_iy0 = c19_iy0;
  c19_c_threshold(chartInstance);
  c19_c_n = c19_b_n;
  c19_c_a = c19_b_a;
  c19_c_ix0 = c19_b_ix0;
  c19_c_iy0 = c19_b_iy0;
  if (c19_c_n < 1) {
  } else if (c19_c_a == 0.0) {
  } else {
    c19_d_a = c19_c_ix0 - 1;
    c19_ix = c19_d_a;
    c19_e_a = c19_c_iy0 - 1;
    c19_iy = c19_e_a;
    c19_f_a = c19_c_n - 1;
    c19_i218 = c19_f_a;
    c19_b = c19_i218;
    c19_b_b = c19_b;
    if (0 > c19_b_b) {
      c19_overflow = false;
    } else {
      c19_eml_switch_helper(chartInstance);
      c19_eml_switch_helper(chartInstance);
      c19_overflow = (c19_b_b > 2147483646);
    }

    if (c19_overflow) {
      c19_check_forloop_overflow_error(chartInstance, c19_overflow);
    }

    for (c19_k = 0; c19_k <= c19_i218; c19_k++) {
      c19_g_a = c19_iy;
      c19_c = c19_g_a;
      c19_h_a = c19_iy;
      c19_b_c = c19_h_a;
      c19_i_a = c19_ix;
      c19_c_c = c19_i_a;
      c19_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c19_c + 1)), 1, 4, 1, 0) - 1] =
        c19_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c19_b_c + 1)), 1, 4, 1, 0) - 1] + c19_c_a *
        c19_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c19_c_c + 1)), 1, 4, 1, 0) - 1];
      c19_j_a = c19_ix + 1;
      c19_ix = c19_j_a;
      c19_k_a = c19_iy + 1;
      c19_iy = c19_k_a;
    }
  }
}

static void c19_b_eml_xscal(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_a, real_T c19_x[4], int32_T c19_ix0)
{
  real_T c19_b_a;
  int32_T c19_b_ix0;
  real_T c19_c_a;
  int32_T c19_c_ix0;
  int32_T c19_d_ix0;
  int32_T c19_d_a;
  int32_T c19_i219;
  int32_T c19_e_a;
  int32_T c19_b;
  int32_T c19_f_a;
  int32_T c19_b_b;
  boolean_T c19_overflow;
  int32_T c19_k;
  int32_T c19_b_k;
  c19_b_a = c19_a;
  c19_b_ix0 = c19_ix0;
  c19_b_below_threshold(chartInstance);
  c19_c_a = c19_b_a;
  c19_c_ix0 = c19_b_ix0;
  c19_d_ix0 = c19_c_ix0;
  c19_d_a = c19_c_ix0 + 1;
  c19_i219 = c19_d_a;
  c19_e_a = c19_d_ix0;
  c19_b = c19_i219;
  c19_f_a = c19_e_a;
  c19_b_b = c19_b;
  if (c19_f_a > c19_b_b) {
    c19_overflow = false;
  } else {
    c19_eml_switch_helper(chartInstance);
    c19_eml_switch_helper(chartInstance);
    c19_overflow = (c19_b_b > 2147483646);
  }

  if (c19_overflow) {
    c19_check_forloop_overflow_error(chartInstance, c19_overflow);
  }

  for (c19_k = c19_d_ix0; c19_k <= c19_i219; c19_k++) {
    c19_b_k = c19_k;
    c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_b_k), 1, 4, 1, 0) - 1] = c19_c_a *
      c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_b_k), 1, 4, 1, 0) - 1];
  }
}

static void c19_b_sqrt(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T *c19_x)
{
  if (*c19_x < 0.0) {
    c19_c_eml_error(chartInstance);
  }

  *c19_x = muDoubleScalarSqrt(*c19_x);
}

static void c19_b_eml_xrotg(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T *c19_a, real_T *c19_b, real_T *c19_c, real_T *c19_s)
{
  real_T c19_b_a;
  real_T c19_b_b;
  real_T c19_c_b;
  real_T c19_c_a;
  real_T c19_d_a;
  real_T c19_d_b;
  real_T c19_e_b;
  real_T c19_e_a;
  real_T c19_b_c;
  real_T c19_b_s;
  double * c19_a_t;
  double * c19_b_t;
  double * c19_c_t;
  double * c19_s_t;
  real_T c19_c_c;
  real_T c19_c_s;
  (void)chartInstance;
  c19_b_a = *c19_a;
  c19_b_b = *c19_b;
  c19_c_b = c19_b_b;
  c19_c_a = c19_b_a;
  c19_d_a = c19_c_a;
  c19_d_b = c19_c_b;
  c19_e_b = c19_d_b;
  c19_e_a = c19_d_a;
  c19_b_c = 0.0;
  c19_b_s = 0.0;
  c19_a_t = (double *)(&c19_e_a);
  c19_b_t = (double *)(&c19_e_b);
  c19_c_t = (double *)(&c19_b_c);
  c19_s_t = (double *)(&c19_b_s);
  drotg(c19_a_t, c19_b_t, c19_c_t, c19_s_t);
  c19_c_a = c19_e_a;
  c19_c_b = c19_e_b;
  c19_c_c = c19_b_c;
  c19_c_s = c19_b_s;
  *c19_a = c19_c_a;
  *c19_b = c19_c_b;
  *c19_c = c19_c_c;
  *c19_s = c19_c_s;
}

static void c19_b_eml_xrot(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4], int32_T c19_ix0, int32_T c19_iy0, real_T c19_c, real_T c19_s)
{
  int32_T c19_b_ix0;
  int32_T c19_b_iy0;
  real_T c19_b_c;
  real_T c19_b_s;
  int32_T c19_c_ix0;
  int32_T c19_c_iy0;
  real_T c19_c_c;
  real_T c19_c_s;
  int32_T c19_ix;
  int32_T c19_iy;
  int32_T c19_k;
  real_T c19_temp;
  int32_T c19_a;
  int32_T c19_b_a;
  c19_b_ix0 = c19_ix0;
  c19_b_iy0 = c19_iy0;
  c19_b_c = c19_c;
  c19_b_s = c19_s;
  c19_d_threshold(chartInstance);
  c19_c_ix0 = c19_b_ix0;
  c19_c_iy0 = c19_b_iy0;
  c19_c_c = c19_b_c;
  c19_c_s = c19_b_s;
  c19_ix = c19_c_ix0;
  c19_iy = c19_c_iy0;
  c19_eml_switch_helper(chartInstance);
  for (c19_k = 1; c19_k < 3; c19_k++) {
    c19_temp = c19_c_c * c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c19_ix), 1, 4, 1, 0) - 1] + c19_c_s *
      c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_iy), 1, 4, 1, 0) - 1];
    c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_iy), 1, 4, 1, 0) - 1] = c19_c_c *
      c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_iy), 1, 4, 1, 0) - 1] - c19_c_s *
      c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_ix), 1, 4, 1, 0) - 1];
    c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_ix), 1, 4, 1, 0) - 1] = c19_temp;
    c19_a = c19_iy + 1;
    c19_iy = c19_a;
    c19_b_a = c19_ix + 1;
    c19_ix = c19_b_a;
  }
}

static void c19_b_eml_xswap(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c19_x[4], int32_T c19_ix0, int32_T c19_iy0)
{
  int32_T c19_b_ix0;
  int32_T c19_b_iy0;
  int32_T c19_c_ix0;
  int32_T c19_c_iy0;
  int32_T c19_ix;
  int32_T c19_iy;
  int32_T c19_k;
  real_T c19_temp;
  int32_T c19_a;
  int32_T c19_b_a;
  c19_b_ix0 = c19_ix0;
  c19_b_iy0 = c19_iy0;
  c19_c_eml_switch_helper(chartInstance);
  c19_c_ix0 = c19_b_ix0;
  c19_c_iy0 = c19_b_iy0;
  c19_ix = c19_c_ix0;
  c19_iy = c19_c_iy0;
  c19_eml_switch_helper(chartInstance);
  for (c19_k = 1; c19_k < 3; c19_k++) {
    c19_temp = c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c19_ix), 1, 4, 1, 0) - 1];
    c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_ix), 1, 4, 1, 0) - 1] = c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_iy), 1, 4, 1, 0) - 1];
    c19_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c19_iy), 1, 4, 1, 0) - 1] = c19_temp;
    c19_a = c19_ix + 1;
    c19_ix = c19_a;
    c19_b_a = c19_iy + 1;
    c19_iy = c19_b_a;
  }
}

static void c19_b_eml_xgemm(SFc19_SS6_Estimation2InstanceStruct *chartInstance,
  int32_T c19_k, real_T c19_A[4], real_T c19_B[4], real_T c19_C[4])
{
  int32_T c19_b_k;
  int32_T c19_c_k;
  int32_T c19_a;
  int32_T c19_km1;
  int32_T c19_cr;
  int32_T c19_b_cr;
  int32_T c19_b_a;
  int32_T c19_i220;
  int32_T c19_c_a;
  int32_T c19_i221;
  int32_T c19_d_a;
  int32_T c19_b;
  int32_T c19_e_a;
  int32_T c19_b_b;
  boolean_T c19_overflow;
  int32_T c19_ic;
  int32_T c19_b_ic;
  int32_T c19_br;
  int32_T c19_c_cr;
  int32_T c19_ar;
  int32_T c19_f_a;
  int32_T c19_b_br;
  int32_T c19_c_b;
  int32_T c19_c;
  int32_T c19_g_a;
  int32_T c19_d_b;
  int32_T c19_i222;
  int32_T c19_h_a;
  int32_T c19_e_b;
  int32_T c19_i_a;
  int32_T c19_f_b;
  boolean_T c19_b_overflow;
  int32_T c19_ib;
  int32_T c19_b_ib;
  real_T c19_temp;
  int32_T c19_ia;
  int32_T c19_j_a;
  int32_T c19_i223;
  int32_T c19_k_a;
  int32_T c19_i224;
  int32_T c19_l_a;
  int32_T c19_g_b;
  int32_T c19_m_a;
  int32_T c19_h_b;
  boolean_T c19_c_overflow;
  int32_T c19_c_ic;
  int32_T c19_n_a;
  int32_T c19_o_a;
  c19_b_k = c19_k;
  if (c19_use_refblas(chartInstance)) {
  } else {
    c19_threshold(chartInstance);
  }

  c19_c_k = c19_b_k;
  c19_a = c19_c_k;
  c19_km1 = c19_a;
  c19_eml_switch_helper(chartInstance);
  for (c19_cr = 0; c19_cr < 3; c19_cr += 2) {
    c19_b_cr = c19_cr;
    c19_b_a = c19_b_cr + 1;
    c19_i220 = c19_b_a;
    c19_c_a = c19_b_cr + 2;
    c19_i221 = c19_c_a;
    c19_d_a = c19_i220;
    c19_b = c19_i221;
    c19_e_a = c19_d_a;
    c19_b_b = c19_b;
    if (c19_e_a > c19_b_b) {
      c19_overflow = false;
    } else {
      c19_eml_switch_helper(chartInstance);
      c19_eml_switch_helper(chartInstance);
      c19_overflow = (c19_b_b > 2147483646);
    }

    if (c19_overflow) {
      c19_check_forloop_overflow_error(chartInstance, c19_overflow);
    }

    for (c19_ic = c19_i220; c19_ic <= c19_i221; c19_ic++) {
      c19_b_ic = c19_ic;
      c19_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c19_b_ic), 1, 4, 1, 0) - 1] = 0.0;
    }
  }

  c19_br = 0;
  c19_eml_switch_helper(chartInstance);
  for (c19_c_cr = 0; c19_c_cr < 3; c19_c_cr += 2) {
    c19_b_cr = c19_c_cr;
    c19_ar = 0;
    c19_f_a = c19_br + 1;
    c19_br = c19_f_a;
    c19_b_br = c19_br;
    c19_c_b = c19_km1 - 1;
    c19_c = c19_c_b << 1;
    c19_g_a = c19_br;
    c19_d_b = c19_c;
    c19_i222 = c19_g_a + c19_d_b;
    c19_h_a = c19_b_br;
    c19_e_b = c19_i222;
    c19_i_a = c19_h_a;
    c19_f_b = c19_e_b;
    if (c19_i_a > c19_f_b) {
      c19_b_overflow = false;
    } else {
      c19_eml_switch_helper(chartInstance);
      c19_eml_switch_helper(chartInstance);
      c19_b_overflow = (c19_f_b > 2147483645);
    }

    if (c19_b_overflow) {
      c19_check_forloop_overflow_error(chartInstance, c19_b_overflow);
    }

    for (c19_ib = c19_b_br; c19_ib <= c19_i222; c19_ib += 2) {
      c19_b_ib = c19_ib;
      if (c19_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_b_ib), 1, 4, 1, 0) - 1] != 0.0) {
        c19_temp = c19_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_b_ib), 1, 4, 1, 0) - 1];
        c19_ia = c19_ar;
        c19_j_a = c19_b_cr + 1;
        c19_i223 = c19_j_a;
        c19_k_a = c19_b_cr + 2;
        c19_i224 = c19_k_a;
        c19_l_a = c19_i223;
        c19_g_b = c19_i224;
        c19_m_a = c19_l_a;
        c19_h_b = c19_g_b;
        if (c19_m_a > c19_h_b) {
          c19_c_overflow = false;
        } else {
          c19_eml_switch_helper(chartInstance);
          c19_eml_switch_helper(chartInstance);
          c19_c_overflow = (c19_h_b > 2147483646);
        }

        if (c19_c_overflow) {
          c19_check_forloop_overflow_error(chartInstance, c19_c_overflow);
        }

        for (c19_c_ic = c19_i223; c19_c_ic <= c19_i224; c19_c_ic++) {
          c19_b_ic = c19_c_ic;
          c19_n_a = c19_ia + 1;
          c19_ia = c19_n_a;
          c19_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_b_ic), 1, 4, 1, 0) - 1] =
            c19_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_b_ic), 1, 4, 1, 0) - 1] + c19_temp *
            c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_ia), 1, 4, 1, 0) - 1];
        }
      }

      c19_o_a = c19_ar + 2;
      c19_ar = c19_o_a;
    }
  }
}

static void init_dsm_address_info(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc19_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  chartInstance->c19_X_1 = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c19_Y_1 = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c19_X_GPS = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c19_Y_GPS = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c19_aX_IMU = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c19_aY_IMU = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c19_r_IMU = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c19_V_x = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c19_V_y = (real_T *)ssGetOutputPortSignal_wrapper
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

void sf_c19_SS6_Estimation2_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1121954543U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4036565571U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(170429564U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3070783999U);
}

mxArray* sf_c19_SS6_Estimation2_get_post_codegen_info(void);
mxArray *sf_c19_SS6_Estimation2_get_autoinheritance_info(void)
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
    mxArray* mxPostCodegenInfo = sf_c19_SS6_Estimation2_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c19_SS6_Estimation2_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c19_SS6_Estimation2_jit_fallback_info(void)
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

mxArray *sf_c19_SS6_Estimation2_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c19_SS6_Estimation2_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c19_SS6_Estimation2(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x7'type','srcId','name','auxInfo'{{M[1],M[14],T\"V_x\",},{M[1],M[18],T\"V_y\",},{M[1],M[19],T\"X_1\",},{M[1],M[20],T\"Y_1\",},{M[4],M[0],T\"State\",S'l','i','p'{{M1x2[87 92],M[0],}}},{M[4],M[0],T\"cov\",S'l','i','p'{{M1x2[83 86],M[0],}}},{M[8],M[0],T\"is_active_c19_SS6_Estimation2\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 7, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c19_SS6_Estimation2_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc19_SS6_Estimation2InstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SS6_Estimation2MachineNumber_,
           19,
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
          (MexFcnForType)c19_c_sf_marshallOut,(MexInFcnForType)
          c19_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_c_sf_marshallOut,(MexInFcnForType)
          c19_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_c_sf_marshallOut,(MexInFcnForType)
          c19_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_c_sf_marshallOut,(MexInFcnForType)
          c19_c_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c19_X_1);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c19_Y_1);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c19_X_GPS);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c19_Y_GPS);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c19_aX_IMU);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c19_aY_IMU);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c19_r_IMU);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c19_V_x);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c19_V_y);
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

static void sf_opaque_initialize_c19_SS6_Estimation2(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc19_SS6_Estimation2InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c19_SS6_Estimation2((SFc19_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
  initialize_c19_SS6_Estimation2((SFc19_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c19_SS6_Estimation2(void *chartInstanceVar)
{
  enable_c19_SS6_Estimation2((SFc19_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c19_SS6_Estimation2(void *chartInstanceVar)
{
  disable_c19_SS6_Estimation2((SFc19_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c19_SS6_Estimation2(void *chartInstanceVar)
{
  sf_gateway_c19_SS6_Estimation2((SFc19_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c19_SS6_Estimation2(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c19_SS6_Estimation2((SFc19_SS6_Estimation2InstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c19_SS6_Estimation2(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c19_SS6_Estimation2((SFc19_SS6_Estimation2InstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c19_SS6_Estimation2(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc19_SS6_Estimation2InstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SS6_Estimation2_optimization_info();
    }

    finalize_c19_SS6_Estimation2((SFc19_SS6_Estimation2InstanceStruct*)
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
  initSimStructsc19_SS6_Estimation2((SFc19_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c19_SS6_Estimation2(SimStruct *S)
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
    initialize_params_c19_SS6_Estimation2((SFc19_SS6_Estimation2InstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c19_SS6_Estimation2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SS6_Estimation2_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      19);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,19,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,19,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,19);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,19,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,19,4);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,19);
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

static void mdlRTW_c19_SS6_Estimation2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c19_SS6_Estimation2(SimStruct *S)
{
  SFc19_SS6_Estimation2InstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc19_SS6_Estimation2InstanceStruct *)utMalloc(sizeof
    (SFc19_SS6_Estimation2InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc19_SS6_Estimation2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c19_SS6_Estimation2;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c19_SS6_Estimation2;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c19_SS6_Estimation2;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c19_SS6_Estimation2;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c19_SS6_Estimation2;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c19_SS6_Estimation2;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c19_SS6_Estimation2;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c19_SS6_Estimation2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c19_SS6_Estimation2;
  chartInstance->chartInfo.mdlStart = mdlStart_c19_SS6_Estimation2;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c19_SS6_Estimation2;
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

void c19_SS6_Estimation2_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c19_SS6_Estimation2(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c19_SS6_Estimation2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c19_SS6_Estimation2(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c19_SS6_Estimation2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
