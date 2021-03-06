/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SS6_Estimation_sfun.h"
#include "c10_SS6_Estimation.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SS6_Estimation_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c10_debug_family_names[6] = { "nargin", "nargout", "w_L",
  "w_R", "count_L", "count_R" };

/* Function Declarations */
static void initialize_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance);
static void initialize_params_c10_SS6_Estimation
  (SFc10_SS6_EstimationInstanceStruct *chartInstance);
static void enable_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance);
static void disable_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance);
static void c10_update_debugger_state_c10_SS6_Estimation
  (SFc10_SS6_EstimationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c10_SS6_Estimation
  (SFc10_SS6_EstimationInstanceStruct *chartInstance);
static void set_sim_state_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct *
  chartInstance, const mxArray *c10_st);
static void finalize_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance);
static void sf_gateway_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance);
static void mdl_start_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance);
static void initSimStructsc10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c10_machineNumber, uint32_T
  c10_chartNumber, uint32_T c10_instanceNumber);
static const mxArray *c10_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static real_T c10_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_b_count_R, const char_T *c10_identifier);
static real_T c10_b_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void c10_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static void c10_info_helper(const mxArray **c10_info);
static const mxArray *c10_emlrt_marshallOut(const char * c10_u);
static const mxArray *c10_b_emlrt_marshallOut(const uint32_T c10_u);
static real_T c10_randn(SFc10_SS6_EstimationInstanceStruct *chartInstance);
static real_T c10_eml_randn(SFc10_SS6_EstimationInstanceStruct *chartInstance);
static uint32_T c10_eml_rand_str2id(SFc10_SS6_EstimationInstanceStruct
  *chartInstance);
static void c10_genrandu(SFc10_SS6_EstimationInstanceStruct *chartInstance,
  uint32_T c10_s, uint32_T *c10_e_state, real_T *c10_r);
static void c10_eml_rand_shr3cong(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_e_state[2], uint32_T c10_f_state[2], real_T
  *c10_r);
static real_T c10_abs(SFc10_SS6_EstimationInstanceStruct *chartInstance, real_T
                      c10_x);
static void c10_eml_rand_mt19937ar(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_e_state[625]);
static void c10_twister_state_vector(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_mt[625], real_T c10_seed, uint32_T c10_b_mt[625]);
static void c10_b_eml_rand_mt19937ar(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_e_state[625], uint32_T c10_f_state[625], real_T
  *c10_r);
static void c10_assert_valid_state(SFc10_SS6_EstimationInstanceStruct
  *chartInstance);
static void c10_genrand_uint32_vector(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_mt[625], uint32_T c10_b_mt[625], uint32_T c10_u[2]);
static void c10_b_genrandu(SFc10_SS6_EstimationInstanceStruct *chartInstance,
  uint32_T c10_mt[625], uint32_T c10_b_mt[625], real_T *c10_r);
static void c10_eml_error(SFc10_SS6_EstimationInstanceStruct *chartInstance);
static const mxArray *c10_b_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static int32_T c10_c_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void c10_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static uint32_T c10_d_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_c_method, const char_T *c10_identifier);
static uint32_T c10_e_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static uint32_T c10_f_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_c_method, const char_T *c10_identifier);
static uint32_T c10_g_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static uint32_T c10_h_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_e_state, const char_T *c10_identifier);
static uint32_T c10_i_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void c10_j_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_e_state, const char_T *c10_identifier,
  uint32_T c10_y[625]);
static void c10_k_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  uint32_T c10_y[625]);
static void c10_l_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_e_state, const char_T *c10_identifier,
  uint32_T c10_y[2]);
static void c10_m_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  uint32_T c10_y[2]);
static void c10_n_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_e_state, const char_T *c10_identifier,
  uint32_T c10_y[2]);
static void c10_o_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  uint32_T c10_y[2]);
static uint8_T c10_p_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_b_is_active_c10_SS6_Estimation, const
  char_T *c10_identifier);
static uint8_T c10_q_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static real_T c10_b_eml_rand_shr3cong(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_e_state[2]);
static void c10_b_twister_state_vector(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_mt[625], real_T c10_seed);
static real_T c10_c_eml_rand_mt19937ar(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_e_state[625]);
static void c10_b_genrand_uint32_vector(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_mt[625], uint32_T c10_u[2]);
static real_T c10_c_genrandu(SFc10_SS6_EstimationInstanceStruct *chartInstance,
  uint32_T c10_mt[625]);
static void init_dsm_address_info(SFc10_SS6_EstimationInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc10_SS6_EstimationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance)
{
  chartInstance->c10_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c10_method_not_empty = false;
  chartInstance->c10_state_not_empty = false;
  chartInstance->c10_b_method_not_empty = false;
  chartInstance->c10_b_state_not_empty = false;
  chartInstance->c10_c_state_not_empty = false;
  chartInstance->c10_d_state_not_empty = false;
  chartInstance->c10_is_active_c10_SS6_Estimation = 0U;
}

static void initialize_params_c10_SS6_Estimation
  (SFc10_SS6_EstimationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c10_update_debugger_state_c10_SS6_Estimation
  (SFc10_SS6_EstimationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c10_SS6_Estimation
  (SFc10_SS6_EstimationInstanceStruct *chartInstance)
{
  const mxArray *c10_st;
  const mxArray *c10_y = NULL;
  real_T c10_hoistedGlobal;
  real_T c10_u;
  const mxArray *c10_b_y = NULL;
  real_T c10_b_hoistedGlobal;
  real_T c10_b_u;
  const mxArray *c10_c_y = NULL;
  uint32_T c10_c_hoistedGlobal;
  uint32_T c10_c_u;
  const mxArray *c10_d_y = NULL;
  uint32_T c10_d_hoistedGlobal;
  uint32_T c10_d_u;
  const mxArray *c10_e_y = NULL;
  uint32_T c10_e_hoistedGlobal;
  uint32_T c10_e_u;
  const mxArray *c10_f_y = NULL;
  int32_T c10_i0;
  uint32_T c10_f_u[625];
  const mxArray *c10_g_y = NULL;
  int32_T c10_i1;
  uint32_T c10_g_u[2];
  const mxArray *c10_h_y = NULL;
  int32_T c10_i2;
  uint32_T c10_h_u[2];
  const mxArray *c10_i_y = NULL;
  uint8_T c10_f_hoistedGlobal;
  uint8_T c10_i_u;
  const mxArray *c10_j_y = NULL;
  c10_st = NULL;
  c10_st = NULL;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_createcellmatrix(9, 1), false);
  c10_hoistedGlobal = *chartInstance->c10_count_L;
  c10_u = c10_hoistedGlobal;
  c10_b_y = NULL;
  sf_mex_assign(&c10_b_y, sf_mex_create("y", &c10_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c10_y, 0, c10_b_y);
  c10_b_hoistedGlobal = *chartInstance->c10_count_R;
  c10_b_u = c10_b_hoistedGlobal;
  c10_c_y = NULL;
  sf_mex_assign(&c10_c_y, sf_mex_create("y", &c10_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c10_y, 1, c10_c_y);
  c10_c_hoistedGlobal = chartInstance->c10_b_method;
  c10_c_u = c10_c_hoistedGlobal;
  c10_d_y = NULL;
  if (!chartInstance->c10_b_method_not_empty) {
    sf_mex_assign(&c10_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c10_d_y, sf_mex_create("y", &c10_c_u, 7, 0U, 0U, 0U, 0),
                  false);
  }

  sf_mex_setcell(c10_y, 2, c10_d_y);
  c10_d_hoistedGlobal = chartInstance->c10_method;
  c10_d_u = c10_d_hoistedGlobal;
  c10_e_y = NULL;
  if (!chartInstance->c10_method_not_empty) {
    sf_mex_assign(&c10_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c10_e_y, sf_mex_create("y", &c10_d_u, 7, 0U, 0U, 0U, 0),
                  false);
  }

  sf_mex_setcell(c10_y, 3, c10_e_y);
  c10_e_hoistedGlobal = chartInstance->c10_b_state;
  c10_e_u = c10_e_hoistedGlobal;
  c10_f_y = NULL;
  if (!chartInstance->c10_b_state_not_empty) {
    sf_mex_assign(&c10_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c10_f_y, sf_mex_create("y", &c10_e_u, 7, 0U, 0U, 0U, 0),
                  false);
  }

  sf_mex_setcell(c10_y, 4, c10_f_y);
  for (c10_i0 = 0; c10_i0 < 625; c10_i0++) {
    c10_f_u[c10_i0] = chartInstance->c10_d_state[c10_i0];
  }

  c10_g_y = NULL;
  if (!chartInstance->c10_d_state_not_empty) {
    sf_mex_assign(&c10_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c10_g_y, sf_mex_create("y", c10_f_u, 7, 0U, 1U, 0U, 1, 625),
                  false);
  }

  sf_mex_setcell(c10_y, 5, c10_g_y);
  for (c10_i1 = 0; c10_i1 < 2; c10_i1++) {
    c10_g_u[c10_i1] = chartInstance->c10_c_state[c10_i1];
  }

  c10_h_y = NULL;
  if (!chartInstance->c10_c_state_not_empty) {
    sf_mex_assign(&c10_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c10_h_y, sf_mex_create("y", c10_g_u, 7, 0U, 1U, 0U, 1, 2),
                  false);
  }

  sf_mex_setcell(c10_y, 6, c10_h_y);
  for (c10_i2 = 0; c10_i2 < 2; c10_i2++) {
    c10_h_u[c10_i2] = chartInstance->c10_state[c10_i2];
  }

  c10_i_y = NULL;
  if (!chartInstance->c10_state_not_empty) {
    sf_mex_assign(&c10_i_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c10_i_y, sf_mex_create("y", c10_h_u, 7, 0U, 1U, 0U, 1, 2),
                  false);
  }

  sf_mex_setcell(c10_y, 7, c10_i_y);
  c10_f_hoistedGlobal = chartInstance->c10_is_active_c10_SS6_Estimation;
  c10_i_u = c10_f_hoistedGlobal;
  c10_j_y = NULL;
  sf_mex_assign(&c10_j_y, sf_mex_create("y", &c10_i_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c10_y, 8, c10_j_y);
  sf_mex_assign(&c10_st, c10_y, false);
  return c10_st;
}

static void set_sim_state_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct *
  chartInstance, const mxArray *c10_st)
{
  const mxArray *c10_u;
  uint32_T c10_uv0[625];
  int32_T c10_i3;
  uint32_T c10_uv1[2];
  int32_T c10_i4;
  uint32_T c10_uv2[2];
  int32_T c10_i5;
  chartInstance->c10_doneDoubleBufferReInit = true;
  c10_u = sf_mex_dup(c10_st);
  *chartInstance->c10_count_L = c10_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c10_u, 0)), "count_L");
  *chartInstance->c10_count_R = c10_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c10_u, 1)), "count_R");
  chartInstance->c10_b_method = c10_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c10_u, 2)), "method");
  chartInstance->c10_method = c10_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c10_u, 3)), "method");
  chartInstance->c10_b_state = c10_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c10_u, 4)), "state");
  c10_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c10_u, 5)),
    "state", c10_uv0);
  for (c10_i3 = 0; c10_i3 < 625; c10_i3++) {
    chartInstance->c10_d_state[c10_i3] = c10_uv0[c10_i3];
  }

  c10_l_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c10_u, 6)),
    "state", c10_uv1);
  for (c10_i4 = 0; c10_i4 < 2; c10_i4++) {
    chartInstance->c10_c_state[c10_i4] = c10_uv1[c10_i4];
  }

  c10_n_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c10_u, 7)),
    "state", c10_uv2);
  for (c10_i5 = 0; c10_i5 < 2; c10_i5++) {
    chartInstance->c10_state[c10_i5] = c10_uv2[c10_i5];
  }

  chartInstance->c10_is_active_c10_SS6_Estimation = c10_p_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c10_u, 8)),
     "is_active_c10_SS6_Estimation");
  sf_mex_destroy(&c10_u);
  c10_update_debugger_state_c10_SS6_Estimation(chartInstance);
  sf_mex_destroy(&c10_st);
}

static void finalize_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance)
{
  real_T c10_hoistedGlobal;
  real_T c10_b_hoistedGlobal;
  real_T c10_b_w_L;
  real_T c10_b_w_R;
  uint32_T c10_debug_family_var_map[6];
  real_T c10_nargin = 2.0;
  real_T c10_nargout = 2.0;
  real_T c10_b_count_L;
  real_T c10_b_count_R;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 9U, chartInstance->c10_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c10_w_L, 0U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c10_w_R, 1U);
  chartInstance->c10_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 9U, chartInstance->c10_sfEvent);
  c10_hoistedGlobal = *chartInstance->c10_w_L;
  c10_b_hoistedGlobal = *chartInstance->c10_w_R;
  c10_b_w_L = c10_hoistedGlobal;
  c10_b_w_R = c10_b_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c10_debug_family_names,
    c10_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_nargin, 0U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_nargout, 1U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c10_b_w_L, 2U, c10_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c10_b_w_R, 3U, c10_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_b_count_L, 4U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_b_count_R, 5U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 4);
<<<<<<< HEAD
  c10_b_count_L = c10_b_w_L + 0.0001 * c10_randn(chartInstance);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 5);
  c10_b_count_R = c10_b_w_R + 0.0001 * c10_randn(chartInstance);
=======
  c10_b_count_L = c10_b_w_L + 0.01 * c10_randn(chartInstance);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 5);
  c10_b_count_R = c10_b_w_R + 0.01 * c10_randn(chartInstance);
>>>>>>> 97c418b1f8209f4cdb2c89b2f22d3af95f3621c9
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, -5);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c10_count_L = c10_b_count_L;
  *chartInstance->c10_count_R = c10_b_count_R;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 9U, chartInstance->c10_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SS6_EstimationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c10_count_L, 2U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c10_count_R, 3U);
}

static void mdl_start_c10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc10_SS6_Estimation(SFc10_SS6_EstimationInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c10_machineNumber, uint32_T
  c10_chartNumber, uint32_T c10_instanceNumber)
{
  (void)c10_machineNumber;
  (void)c10_chartNumber;
  (void)c10_instanceNumber;
}

static const mxArray *c10_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  real_T c10_u;
  const mxArray *c10_y = NULL;
  SFc10_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc10_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_u = *(real_T *)c10_inData;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", &c10_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static real_T c10_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_b_count_R, const char_T *c10_identifier)
{
  real_T c10_y;
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_b_count_R),
    &c10_thisId);
  sf_mex_destroy(&c10_b_count_R);
  return c10_y;
}

static real_T c10_b_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  real_T c10_y;
  real_T c10_d0;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_d0, 1, 0, 0U, 0, 0U, 0);
  c10_y = c10_d0;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void c10_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_b_count_R;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y;
  SFc10_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc10_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c10_b_count_R = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_b_count_R),
    &c10_thisId);
  sf_mex_destroy(&c10_b_count_R);
  *(real_T *)c10_outData = c10_y;
  sf_mex_destroy(&c10_mxArrayInData);
}

const mxArray *sf_c10_SS6_Estimation_get_eml_resolved_functions_info(void)
{
  const mxArray *c10_nameCaptureInfo = NULL;
  c10_nameCaptureInfo = NULL;
  sf_mex_assign(&c10_nameCaptureInfo, sf_mex_createstruct("structure", 2, 47, 1),
                false);
  c10_info_helper(&c10_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c10_nameCaptureInfo);
  return c10_nameCaptureInfo;
}

static void c10_info_helper(const mxArray **c10_info)
{
  const mxArray *c10_rhs0 = NULL;
  const mxArray *c10_lhs0 = NULL;
  const mxArray *c10_rhs1 = NULL;
  const mxArray *c10_lhs1 = NULL;
  const mxArray *c10_rhs2 = NULL;
  const mxArray *c10_lhs2 = NULL;
  const mxArray *c10_rhs3 = NULL;
  const mxArray *c10_lhs3 = NULL;
  const mxArray *c10_rhs4 = NULL;
  const mxArray *c10_lhs4 = NULL;
  const mxArray *c10_rhs5 = NULL;
  const mxArray *c10_lhs5 = NULL;
  const mxArray *c10_rhs6 = NULL;
  const mxArray *c10_lhs6 = NULL;
  const mxArray *c10_rhs7 = NULL;
  const mxArray *c10_lhs7 = NULL;
  const mxArray *c10_rhs8 = NULL;
  const mxArray *c10_lhs8 = NULL;
  const mxArray *c10_rhs9 = NULL;
  const mxArray *c10_lhs9 = NULL;
  const mxArray *c10_rhs10 = NULL;
  const mxArray *c10_lhs10 = NULL;
  const mxArray *c10_rhs11 = NULL;
  const mxArray *c10_lhs11 = NULL;
  const mxArray *c10_rhs12 = NULL;
  const mxArray *c10_lhs12 = NULL;
  const mxArray *c10_rhs13 = NULL;
  const mxArray *c10_lhs13 = NULL;
  const mxArray *c10_rhs14 = NULL;
  const mxArray *c10_lhs14 = NULL;
  const mxArray *c10_rhs15 = NULL;
  const mxArray *c10_lhs15 = NULL;
  const mxArray *c10_rhs16 = NULL;
  const mxArray *c10_lhs16 = NULL;
  const mxArray *c10_rhs17 = NULL;
  const mxArray *c10_lhs17 = NULL;
  const mxArray *c10_rhs18 = NULL;
  const mxArray *c10_lhs18 = NULL;
  const mxArray *c10_rhs19 = NULL;
  const mxArray *c10_lhs19 = NULL;
  const mxArray *c10_rhs20 = NULL;
  const mxArray *c10_lhs20 = NULL;
  const mxArray *c10_rhs21 = NULL;
  const mxArray *c10_lhs21 = NULL;
  const mxArray *c10_rhs22 = NULL;
  const mxArray *c10_lhs22 = NULL;
  const mxArray *c10_rhs23 = NULL;
  const mxArray *c10_lhs23 = NULL;
  const mxArray *c10_rhs24 = NULL;
  const mxArray *c10_lhs24 = NULL;
  const mxArray *c10_rhs25 = NULL;
  const mxArray *c10_lhs25 = NULL;
  const mxArray *c10_rhs26 = NULL;
  const mxArray *c10_lhs26 = NULL;
  const mxArray *c10_rhs27 = NULL;
  const mxArray *c10_lhs27 = NULL;
  const mxArray *c10_rhs28 = NULL;
  const mxArray *c10_lhs28 = NULL;
  const mxArray *c10_rhs29 = NULL;
  const mxArray *c10_lhs29 = NULL;
  const mxArray *c10_rhs30 = NULL;
  const mxArray *c10_lhs30 = NULL;
  const mxArray *c10_rhs31 = NULL;
  const mxArray *c10_lhs31 = NULL;
  const mxArray *c10_rhs32 = NULL;
  const mxArray *c10_lhs32 = NULL;
  const mxArray *c10_rhs33 = NULL;
  const mxArray *c10_lhs33 = NULL;
  const mxArray *c10_rhs34 = NULL;
  const mxArray *c10_lhs34 = NULL;
  const mxArray *c10_rhs35 = NULL;
  const mxArray *c10_lhs35 = NULL;
  const mxArray *c10_rhs36 = NULL;
  const mxArray *c10_lhs36 = NULL;
  const mxArray *c10_rhs37 = NULL;
  const mxArray *c10_lhs37 = NULL;
  const mxArray *c10_rhs38 = NULL;
  const mxArray *c10_lhs38 = NULL;
  const mxArray *c10_rhs39 = NULL;
  const mxArray *c10_lhs39 = NULL;
  const mxArray *c10_rhs40 = NULL;
  const mxArray *c10_lhs40 = NULL;
  const mxArray *c10_rhs41 = NULL;
  const mxArray *c10_lhs41 = NULL;
  const mxArray *c10_rhs42 = NULL;
  const mxArray *c10_lhs42 = NULL;
  const mxArray *c10_rhs43 = NULL;
  const mxArray *c10_lhs43 = NULL;
  const mxArray *c10_rhs44 = NULL;
  const mxArray *c10_lhs44 = NULL;
  const mxArray *c10_rhs45 = NULL;
  const mxArray *c10_lhs45 = NULL;
  const mxArray *c10_rhs46 = NULL;
  const mxArray *c10_lhs46 = NULL;
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("randn"), "name", "name", 0);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/randn.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1383898890U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c10_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs0), "rhs", "rhs",
                  0);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs0), "lhs", "lhs",
                  0);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/randn.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_is_rand_extrinsic"),
                  "name", "name", 1);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_is_rand_extrinsic.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1368204632U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c10_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs1), "rhs", "rhs",
                  1);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs1), "lhs", "lhs",
                  1);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/randn.m"), "context",
                  "context", 2);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_randn"), "name", "name",
                  2);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_randn.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1313369422U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c10_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs2), "rhs", "rhs",
                  2);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs2), "lhs", "lhs",
                  2);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_randn.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_str2id"), "name",
                  "name", 3);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_str2id.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1313369422U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c10_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs3), "rhs", "rhs",
                  3);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs3), "lhs", "lhs",
                  3);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_str2id.m"),
                  "context", "context", 4);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 4);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1393352458U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c10_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs4), "rhs", "rhs",
                  4);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs4), "lhs", "lhs",
                  4);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_randn.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_shr3cong"), "name",
                  "name", 5);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1313369420U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c10_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs5), "rhs", "rhs",
                  5);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs5), "lhs", "lhs",
                  5);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_randn.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_tolower"), "name",
                  "name", 6);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_tolower.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363731870U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c10_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs6), "rhs", "rhs",
                  6);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs6), "lhs", "lhs",
                  6);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_randn.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 7);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1393352458U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c10_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs7), "rhs", "rhs",
                  7);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs7), "lhs", "lhs",
                  7);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_randn.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand"), "name", "name",
                  8);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand.m"), "resolved",
                  "resolved", 8);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1313369420U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c10_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs8), "rhs", "rhs",
                  8);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs8), "lhs", "lhs",
                  8);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_str2id"), "name",
                  "name", 9);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_str2id.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1313369422U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c10_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs9), "rhs", "rhs",
                  9);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs9), "lhs", "lhs",
                  9);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_mcg16807_stateful"),
                  "name", "name", 10);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mcg16807_stateful.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1366183844U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c10_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mcg16807_stateful.m"),
                  "context", "context", 11);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_mcg16807"), "name",
                  "name", 11);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mcg16807.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1313369420U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c10_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mcg16807_stateful.m"),
                  "context", "context", 12);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_mcg16807"), "name",
                  "name", 12);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("uint32"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mcg16807.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1313369420U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c10_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_shr3cong_stateful"),
                  "name", "name", 13);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong_stateful.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1366183844U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c10_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong_stateful.m"),
                  "context", "context", 14);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_shr3cong"), "name",
                  "name", 14);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1313369420U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c10_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong_stateful.m"),
                  "context", "context", 15);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_shr3cong"), "name",
                  "name", 15);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("uint32"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1313369420U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c10_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong.m!genrandn"),
                  "context", "context", 16);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("abs"), "name", "name", 16);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c10_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 17);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c10_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 18);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840312U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c10_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong.m!genrandn"),
                  "context", "context", 19);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("mrdivide"), "name", "name",
                  19);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c10_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 20);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 20);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389739374U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c10_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 21);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("rdivide"), "name", "name",
                  21);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c10_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 22);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c10_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 23);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c10_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_div"), "name", "name",
                  24);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 24);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c10_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 25);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 25);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 25);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c10_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong.m!genrandn"),
                  "context", "context", 26);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("exp"), "name", "name", 26);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/exp.m"), "resolved",
                  "resolved", 26);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1343851980U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c10_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/exp.m"), "context",
                  "context", 27);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_exp"), "name",
                  "name", 27);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_exp.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1395346500U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c10_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand.m"), "context",
                  "context", 28);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_mt19937ar_stateful"),
                  "name", "name", 28);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar_stateful.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1366183844U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c10_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar_stateful.m"),
                  "context", "context", 29);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_mt19937ar"), "name",
                  "name", 29);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1406834748U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c10_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar_stateful.m"),
                  "context", "context", 30);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_mt19937ar"), "name",
                  "name", 30);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("uint32"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1406834748U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c10_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!is_valid_state"),
                  "context", "context", 31);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("isequal"), "name", "name",
                  31);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 31);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840358U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c10_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "context",
                  "context", 32);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_isequal_core"), "name",
                  "name", 32);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840386U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c10_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m!isequal_scalar"),
                  "context", "context", 33);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("isnan"), "name", "name", 33);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 33);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c10_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 34);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 34);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c10_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!mtziggurat"),
                  "context", "context", 35);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("mrdivide"), "name", "name",
                  35);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 35);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c10_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!mtziggurat"),
                  "context", "context", 36);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("abs"), "name", "name", 36);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 36);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c10_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!genrandu"),
                  "context", "context", 37);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eps"), "name", "name", 37);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 37);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c10_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 38);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_eps"), "name", "name",
                  38);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c10_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 39);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 39);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c10_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!is_valid_state"),
                  "context", "context", 40);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 40);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c10_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!is_valid_state"),
                  "context", "context", 41);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 41);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c10_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 42);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 42);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c10_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!genrandu"),
                  "context", "context", 43);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_error"), "name", "name",
                  43);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 43);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c10_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!mtziggurat"),
                  "context", "context", 44);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("exp"), "name", "name", 44);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/exp.m"), "resolved",
                  "resolved", 44);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1343851980U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c10_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_randn.m"), "context",
                  "context", 45);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_mcg16807"), "name",
                  "name", 45);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("uint32"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mcg16807.m"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1313369420U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c10_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_randn.m"), "context",
                  "context", 46);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_rand_shr3cong"), "name",
                  "name", 46);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("uint32"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1313369420U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c10_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs46), "lhs", "lhs",
                  46);
  sf_mex_destroy(&c10_rhs0);
  sf_mex_destroy(&c10_lhs0);
  sf_mex_destroy(&c10_rhs1);
  sf_mex_destroy(&c10_lhs1);
  sf_mex_destroy(&c10_rhs2);
  sf_mex_destroy(&c10_lhs2);
  sf_mex_destroy(&c10_rhs3);
  sf_mex_destroy(&c10_lhs3);
  sf_mex_destroy(&c10_rhs4);
  sf_mex_destroy(&c10_lhs4);
  sf_mex_destroy(&c10_rhs5);
  sf_mex_destroy(&c10_lhs5);
  sf_mex_destroy(&c10_rhs6);
  sf_mex_destroy(&c10_lhs6);
  sf_mex_destroy(&c10_rhs7);
  sf_mex_destroy(&c10_lhs7);
  sf_mex_destroy(&c10_rhs8);
  sf_mex_destroy(&c10_lhs8);
  sf_mex_destroy(&c10_rhs9);
  sf_mex_destroy(&c10_lhs9);
  sf_mex_destroy(&c10_rhs10);
  sf_mex_destroy(&c10_lhs10);
  sf_mex_destroy(&c10_rhs11);
  sf_mex_destroy(&c10_lhs11);
  sf_mex_destroy(&c10_rhs12);
  sf_mex_destroy(&c10_lhs12);
  sf_mex_destroy(&c10_rhs13);
  sf_mex_destroy(&c10_lhs13);
  sf_mex_destroy(&c10_rhs14);
  sf_mex_destroy(&c10_lhs14);
  sf_mex_destroy(&c10_rhs15);
  sf_mex_destroy(&c10_lhs15);
  sf_mex_destroy(&c10_rhs16);
  sf_mex_destroy(&c10_lhs16);
  sf_mex_destroy(&c10_rhs17);
  sf_mex_destroy(&c10_lhs17);
  sf_mex_destroy(&c10_rhs18);
  sf_mex_destroy(&c10_lhs18);
  sf_mex_destroy(&c10_rhs19);
  sf_mex_destroy(&c10_lhs19);
  sf_mex_destroy(&c10_rhs20);
  sf_mex_destroy(&c10_lhs20);
  sf_mex_destroy(&c10_rhs21);
  sf_mex_destroy(&c10_lhs21);
  sf_mex_destroy(&c10_rhs22);
  sf_mex_destroy(&c10_lhs22);
  sf_mex_destroy(&c10_rhs23);
  sf_mex_destroy(&c10_lhs23);
  sf_mex_destroy(&c10_rhs24);
  sf_mex_destroy(&c10_lhs24);
  sf_mex_destroy(&c10_rhs25);
  sf_mex_destroy(&c10_lhs25);
  sf_mex_destroy(&c10_rhs26);
  sf_mex_destroy(&c10_lhs26);
  sf_mex_destroy(&c10_rhs27);
  sf_mex_destroy(&c10_lhs27);
  sf_mex_destroy(&c10_rhs28);
  sf_mex_destroy(&c10_lhs28);
  sf_mex_destroy(&c10_rhs29);
  sf_mex_destroy(&c10_lhs29);
  sf_mex_destroy(&c10_rhs30);
  sf_mex_destroy(&c10_lhs30);
  sf_mex_destroy(&c10_rhs31);
  sf_mex_destroy(&c10_lhs31);
  sf_mex_destroy(&c10_rhs32);
  sf_mex_destroy(&c10_lhs32);
  sf_mex_destroy(&c10_rhs33);
  sf_mex_destroy(&c10_lhs33);
  sf_mex_destroy(&c10_rhs34);
  sf_mex_destroy(&c10_lhs34);
  sf_mex_destroy(&c10_rhs35);
  sf_mex_destroy(&c10_lhs35);
  sf_mex_destroy(&c10_rhs36);
  sf_mex_destroy(&c10_lhs36);
  sf_mex_destroy(&c10_rhs37);
  sf_mex_destroy(&c10_lhs37);
  sf_mex_destroy(&c10_rhs38);
  sf_mex_destroy(&c10_lhs38);
  sf_mex_destroy(&c10_rhs39);
  sf_mex_destroy(&c10_lhs39);
  sf_mex_destroy(&c10_rhs40);
  sf_mex_destroy(&c10_lhs40);
  sf_mex_destroy(&c10_rhs41);
  sf_mex_destroy(&c10_lhs41);
  sf_mex_destroy(&c10_rhs42);
  sf_mex_destroy(&c10_lhs42);
  sf_mex_destroy(&c10_rhs43);
  sf_mex_destroy(&c10_lhs43);
  sf_mex_destroy(&c10_rhs44);
  sf_mex_destroy(&c10_lhs44);
  sf_mex_destroy(&c10_rhs45);
  sf_mex_destroy(&c10_lhs45);
  sf_mex_destroy(&c10_rhs46);
  sf_mex_destroy(&c10_lhs46);
}

static const mxArray *c10_emlrt_marshallOut(const char * c10_u)
{
  const mxArray *c10_y = NULL;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c10_u)), false);
  return c10_y;
}

static const mxArray *c10_b_emlrt_marshallOut(const uint32_T c10_u)
{
  const mxArray *c10_y = NULL;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", &c10_u, 7, 0U, 0U, 0U, 0), false);
  return c10_y;
}

static real_T c10_randn(SFc10_SS6_EstimationInstanceStruct *chartInstance)
{
  return c10_eml_randn(chartInstance);
}

static real_T c10_eml_randn(SFc10_SS6_EstimationInstanceStruct *chartInstance)
{
  real_T c10_r;
  int32_T c10_i6;
  uint32_T c10_V4;
  uint32_T c10_hoistedGlobal;
  uint32_T c10_e_state;
  uint32_T c10_f_state;
  uint32_T c10_g_state;
  uint32_T c10_h_state;
  real_T c10_b_r;
  uint32_T c10_i_state;
  real_T c10_c_r;
  real_T c10_t;
  uint32_T c10_j_state;
  real_T c10_b_t;
  real_T c10_d1;
  int32_T c10_i7;
  real_T c10_d2;
  uint32_T c10_uv3[625];
  int32_T c10_i8;
  real_T c10_d3;
  uint32_T c10_k_state;
  uint32_T c10_u0;
  uint32_T c10_l_state;
  uint32_T c10_m_state;
  real_T c10_d_r;
  uint32_T c10_n_state;
  real_T c10_e_r;
  real_T c10_c_t;
  uint32_T c10_o_state;
  real_T c10_d_t;
  real_T c10_d4;
  real_T c10_d5;
  if (!chartInstance->c10_method_not_empty) {
    chartInstance->c10_method = 0U;
    chartInstance->c10_method_not_empty = true;
    for (c10_i6 = 0; c10_i6 < 2; c10_i6++) {
      chartInstance->c10_state[c10_i6] = 362436069U + (uint32_T)(-362436069 *
        c10_i6);
    }

    if ((real_T)chartInstance->c10_state[1] == 0.0) {
      chartInstance->c10_state[1] = 521288629U;
    }

    chartInstance->c10_state_not_empty = true;
  }

  if (chartInstance->c10_method == 0U) {
    c10_V4 = c10_eml_rand_str2id(chartInstance);
    if (!chartInstance->c10_b_method_not_empty) {
      chartInstance->c10_b_method = 7U;
      chartInstance->c10_b_method_not_empty = true;
    }

    if (chartInstance->c10_b_method == c10_V4) {
      if (!chartInstance->c10_b_state_not_empty) {
        chartInstance->c10_b_state = 1144108930U;
        chartInstance->c10_b_state_not_empty = true;
      }

      c10_hoistedGlobal = chartInstance->c10_b_state;
      c10_e_state = c10_hoistedGlobal;
      c10_f_state = c10_e_state;
      c10_g_state = c10_f_state;
      c10_h_state = c10_g_state;
      do {
        c10_genrandu(chartInstance, c10_h_state, &c10_i_state, &c10_b_r);
        c10_h_state = c10_i_state;
        c10_c_r = c10_b_r;
        c10_genrandu(chartInstance, c10_h_state, &c10_j_state, &c10_t);
        c10_h_state = c10_j_state;
        c10_b_t = c10_t;
        c10_c_r = 2.0 * c10_c_r - 1.0;
        c10_b_t = 2.0 * c10_b_t - 1.0;
        c10_b_t = c10_b_t * c10_b_t + c10_c_r * c10_c_r;
      } while (!(c10_b_t <= 1.0));

      c10_c_r *= muDoubleScalarSqrt(-2.0 * muDoubleScalarLog(c10_b_t) / c10_b_t);
      c10_f_state = c10_h_state;
      c10_d1 = c10_c_r;
      chartInstance->c10_b_state = c10_f_state;
      c10_r = c10_d1;
    } else if (chartInstance->c10_b_method == 5U) {
      if (!chartInstance->c10_c_state_not_empty) {
        for (c10_i7 = 0; c10_i7 < 2; c10_i7++) {
          chartInstance->c10_c_state[c10_i7] = 362436069U + 158852560U *
            (uint32_T)c10_i7;
        }

        chartInstance->c10_c_state_not_empty = true;
      }

      c10_d2 = c10_b_eml_rand_shr3cong(chartInstance, chartInstance->c10_c_state);
      c10_r = c10_d2;
    } else {
      if (!chartInstance->c10_d_state_not_empty) {
        c10_eml_rand_mt19937ar(chartInstance, c10_uv3);
        for (c10_i8 = 0; c10_i8 < 625; c10_i8++) {
          chartInstance->c10_d_state[c10_i8] = c10_uv3[c10_i8];
        }

        chartInstance->c10_d_state_not_empty = true;
      }

      c10_d3 = c10_c_eml_rand_mt19937ar(chartInstance,
        chartInstance->c10_d_state);
      c10_r = c10_d3;
    }
  } else if (chartInstance->c10_method == c10_eml_rand_str2id(chartInstance)) {
    c10_k_state = chartInstance->c10_state[0];
    c10_u0 = c10_k_state;
    c10_l_state = c10_u0;
    c10_m_state = c10_l_state;
    do {
      c10_genrandu(chartInstance, c10_m_state, &c10_n_state, &c10_d_r);
      c10_m_state = c10_n_state;
      c10_e_r = c10_d_r;
      c10_genrandu(chartInstance, c10_m_state, &c10_o_state, &c10_c_t);
      c10_m_state = c10_o_state;
      c10_d_t = c10_c_t;
      c10_e_r = 2.0 * c10_e_r - 1.0;
      c10_d_t = 2.0 * c10_d_t - 1.0;
      c10_d_t = c10_d_t * c10_d_t + c10_e_r * c10_e_r;
    } while (!(c10_d_t <= 1.0));

    c10_e_r *= muDoubleScalarSqrt(-2.0 * muDoubleScalarLog(c10_d_t) / c10_d_t);
    c10_u0 = c10_m_state;
    c10_d4 = c10_e_r;
    chartInstance->c10_state[0] = c10_u0;
    c10_r = c10_d4;
  } else {
    c10_d5 = c10_b_eml_rand_shr3cong(chartInstance, chartInstance->c10_state);
    c10_r = c10_d5;
  }

  return c10_r;
}

static uint32_T c10_eml_rand_str2id(SFc10_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
  return 4U;
}

static void c10_genrandu(SFc10_SS6_EstimationInstanceStruct *chartInstance,
  uint32_T c10_s, uint32_T *c10_e_state, real_T *c10_r)
{
  uint32_T c10_u1;
  uint32_T c10_hi;
  uint32_T c10_lo;
  uint32_T c10_test1;
  uint32_T c10_test2;
  (void)chartInstance;
  c10_u1 = 127773U;
  if (c10_u1 == 0U) {
    c10_hi = MAX_uint32_T;
  } else {
    c10_hi = c10_s / c10_u1;
  }

  c10_lo = c10_s - c10_hi * 127773U;
  c10_test1 = 16807U * c10_lo;
  c10_test2 = 2836U * c10_hi;
  if (c10_test1 < c10_test2) {
    *c10_e_state = (c10_test1 - c10_test2) + 2147483647U;
  } else {
    *c10_e_state = c10_test1 - c10_test2;
  }

  *c10_r = (real_T)*c10_e_state * 4.6566128752457969E-10;
}

static void c10_eml_rand_shr3cong(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_e_state[2], uint32_T c10_f_state[2], real_T
  *c10_r)
{
  int32_T c10_i9;
  for (c10_i9 = 0; c10_i9 < 2; c10_i9++) {
    c10_f_state[c10_i9] = c10_e_state[c10_i9];
  }

  *c10_r = c10_b_eml_rand_shr3cong(chartInstance, c10_f_state);
}

static real_T c10_abs(SFc10_SS6_EstimationInstanceStruct *chartInstance, real_T
                      c10_x)
{
  real_T c10_b_x;
  (void)chartInstance;
  c10_b_x = c10_x;
  return muDoubleScalarAbs(c10_b_x);
}

static void c10_eml_rand_mt19937ar(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_e_state[625])
{
  int32_T c10_i10;
  for (c10_i10 = 0; c10_i10 < 625; c10_i10++) {
    c10_e_state[c10_i10] = 0U;
  }

  c10_b_twister_state_vector(chartInstance, c10_e_state, 5489.0);
}

static void c10_twister_state_vector(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_mt[625], real_T c10_seed, uint32_T c10_b_mt[625])
{
  int32_T c10_i11;
  for (c10_i11 = 0; c10_i11 < 625; c10_i11++) {
    c10_b_mt[c10_i11] = c10_mt[c10_i11];
  }

  c10_b_twister_state_vector(chartInstance, c10_b_mt, c10_seed);
}

static void c10_b_eml_rand_mt19937ar(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_e_state[625], uint32_T c10_f_state[625], real_T
  *c10_r)
{
  int32_T c10_i12;
  for (c10_i12 = 0; c10_i12 < 625; c10_i12++) {
    c10_f_state[c10_i12] = c10_e_state[c10_i12];
  }

  *c10_r = c10_c_eml_rand_mt19937ar(chartInstance, c10_f_state);
}

static void c10_assert_valid_state(SFc10_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c10_genrand_uint32_vector(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_mt[625], uint32_T c10_b_mt[625], uint32_T c10_u[2])
{
  int32_T c10_i13;
  for (c10_i13 = 0; c10_i13 < 625; c10_i13++) {
    c10_b_mt[c10_i13] = c10_mt[c10_i13];
  }

  c10_b_genrand_uint32_vector(chartInstance, c10_b_mt, c10_u);
}

static void c10_b_genrandu(SFc10_SS6_EstimationInstanceStruct *chartInstance,
  uint32_T c10_mt[625], uint32_T c10_b_mt[625], real_T *c10_r)
{
  int32_T c10_i14;
  for (c10_i14 = 0; c10_i14 < 625; c10_i14++) {
    c10_b_mt[c10_i14] = c10_mt[c10_i14];
  }

  *c10_r = c10_c_genrandu(chartInstance, c10_b_mt);
}

static void c10_eml_error(SFc10_SS6_EstimationInstanceStruct *chartInstance)
{
  int32_T c10_i15;
  static char_T c10_cv0[37] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 'r', 'a', 'n', 'd', '_', 'i', 'n', 'v', 'a', 'l', 'i', 'd',
    'T', 'w', 'i', 's', 't', 'e', 'r', 'S', 't', 'a', 't', 'e' };

  char_T c10_u[37];
  const mxArray *c10_y = NULL;
  (void)chartInstance;
  for (c10_i15 = 0; c10_i15 < 37; c10_i15++) {
    c10_u[c10_i15] = c10_cv0[c10_i15];
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 10, 0U, 1U, 0U, 2, 1, 37),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c10_y));
}

static const mxArray *c10_b_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_u;
  const mxArray *c10_y = NULL;
  SFc10_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc10_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_u = *(int32_T *)c10_inData;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", &c10_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static int32_T c10_c_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  int32_T c10_y;
  int32_T c10_i16;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_i16, 1, 6, 0U, 0, 0U, 0);
  c10_y = c10_i16;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void c10_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_b_sfEvent;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  int32_T c10_y;
  SFc10_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc10_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c10_b_sfEvent = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_b_sfEvent),
    &c10_thisId);
  sf_mex_destroy(&c10_b_sfEvent);
  *(int32_T *)c10_outData = c10_y;
  sf_mex_destroy(&c10_mxArrayInData);
}

static uint32_T c10_d_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_c_method, const char_T *c10_identifier)
{
  uint32_T c10_y;
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_c_method),
    &c10_thisId);
  sf_mex_destroy(&c10_c_method);
  return c10_y;
}

static uint32_T c10_e_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  uint32_T c10_y;
  uint32_T c10_u2;
  if (mxIsEmpty(c10_u)) {
    chartInstance->c10_b_method_not_empty = false;
  } else {
    chartInstance->c10_b_method_not_empty = true;
    sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_u2, 1, 7, 0U, 0, 0U, 0);
    c10_y = c10_u2;
  }

  sf_mex_destroy(&c10_u);
  return c10_y;
}

static uint32_T c10_f_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_c_method, const char_T *c10_identifier)
{
  uint32_T c10_y;
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_c_method),
    &c10_thisId);
  sf_mex_destroy(&c10_c_method);
  return c10_y;
}

static uint32_T c10_g_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  uint32_T c10_y;
  uint32_T c10_u3;
  if (mxIsEmpty(c10_u)) {
    chartInstance->c10_method_not_empty = false;
  } else {
    chartInstance->c10_method_not_empty = true;
    sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_u3, 1, 7, 0U, 0, 0U, 0);
    c10_y = c10_u3;
  }

  sf_mex_destroy(&c10_u);
  return c10_y;
}

static uint32_T c10_h_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_e_state, const char_T *c10_identifier)
{
  uint32_T c10_y;
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_e_state),
    &c10_thisId);
  sf_mex_destroy(&c10_e_state);
  return c10_y;
}

static uint32_T c10_i_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  uint32_T c10_y;
  uint32_T c10_u4;
  if (mxIsEmpty(c10_u)) {
    chartInstance->c10_b_state_not_empty = false;
  } else {
    chartInstance->c10_b_state_not_empty = true;
    sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_u4, 1, 7, 0U, 0, 0U, 0);
    c10_y = c10_u4;
  }

  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void c10_j_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_e_state, const char_T *c10_identifier,
  uint32_T c10_y[625])
{
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_e_state), &c10_thisId,
    c10_y);
  sf_mex_destroy(&c10_e_state);
}

static void c10_k_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  uint32_T c10_y[625])
{
  uint32_T c10_uv4[625];
  int32_T c10_i17;
  if (mxIsEmpty(c10_u)) {
    chartInstance->c10_d_state_not_empty = false;
  } else {
    chartInstance->c10_d_state_not_empty = true;
    sf_mex_import(c10_parentId, sf_mex_dup(c10_u), c10_uv4, 1, 7, 0U, 1, 0U, 1,
                  625);
    for (c10_i17 = 0; c10_i17 < 625; c10_i17++) {
      c10_y[c10_i17] = c10_uv4[c10_i17];
    }
  }

  sf_mex_destroy(&c10_u);
}

static void c10_l_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_e_state, const char_T *c10_identifier,
  uint32_T c10_y[2])
{
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_e_state), &c10_thisId,
    c10_y);
  sf_mex_destroy(&c10_e_state);
}

static void c10_m_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  uint32_T c10_y[2])
{
  uint32_T c10_uv5[2];
  int32_T c10_i18;
  if (mxIsEmpty(c10_u)) {
    chartInstance->c10_c_state_not_empty = false;
  } else {
    chartInstance->c10_c_state_not_empty = true;
    sf_mex_import(c10_parentId, sf_mex_dup(c10_u), c10_uv5, 1, 7, 0U, 1, 0U, 1,
                  2);
    for (c10_i18 = 0; c10_i18 < 2; c10_i18++) {
      c10_y[c10_i18] = c10_uv5[c10_i18];
    }
  }

  sf_mex_destroy(&c10_u);
}

static void c10_n_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_e_state, const char_T *c10_identifier,
  uint32_T c10_y[2])
{
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_e_state), &c10_thisId,
    c10_y);
  sf_mex_destroy(&c10_e_state);
}

static void c10_o_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  uint32_T c10_y[2])
{
  uint32_T c10_uv6[2];
  int32_T c10_i19;
  if (mxIsEmpty(c10_u)) {
    chartInstance->c10_state_not_empty = false;
  } else {
    chartInstance->c10_state_not_empty = true;
    sf_mex_import(c10_parentId, sf_mex_dup(c10_u), c10_uv6, 1, 7, 0U, 1, 0U, 1,
                  2);
    for (c10_i19 = 0; c10_i19 < 2; c10_i19++) {
      c10_y[c10_i19] = c10_uv6[c10_i19];
    }
  }

  sf_mex_destroy(&c10_u);
}

static uint8_T c10_p_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_b_is_active_c10_SS6_Estimation, const
  char_T *c10_identifier)
{
  uint8_T c10_y;
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_q_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c10_b_is_active_c10_SS6_Estimation), &c10_thisId);
  sf_mex_destroy(&c10_b_is_active_c10_SS6_Estimation);
  return c10_y;
}

static uint8_T c10_q_emlrt_marshallIn(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  uint8_T c10_y;
  uint8_T c10_u5;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_u5, 1, 3, 0U, 0, 0U, 0);
  c10_y = c10_u5;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static real_T c10_b_eml_rand_shr3cong(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_e_state[2])
{
  uint32_T c10_icng;
  uint32_T c10_jsr;
  uint32_T c10_b_icng;
  uint32_T c10_b_jsr;
  uint32_T c10_c_jsr;
  uint32_T c10_c_icng;
  uint32_T c10_ui;
  uint32_T c10_b_ui;
  uint32_T c10_j;
  uint32_T c10_jp1;
  int32_T c10_i;
  static real_T c10_dv0[65] = { 0.340945, 0.4573146, 0.5397793, 0.6062427,
    0.6631691, 0.7136975, 0.7596125, 0.8020356, 0.8417227, 0.8792102, 0.9148948,
    0.9490791, 0.9820005, 1.0138492, 1.044781, 1.0749254, 1.1043917, 1.1332738,
    1.161653, 1.189601, 1.2171815, 1.2444516, 1.2714635, 1.298265, 1.3249008,
    1.3514125, 1.3778399, 1.4042211, 1.4305929, 1.4569915, 1.4834527, 1.5100122,
    1.5367061, 1.5635712, 1.5906454, 1.617968, 1.6455802, 1.6735255, 1.7018503,
    1.7306045, 1.7598422, 1.7896223, 1.8200099, 1.851077, 1.8829044, 1.9155831,
    1.9492166, 1.9839239, 2.0198431, 2.0571356, 2.095993, 2.136645, 2.1793713,
    2.2245175, 2.2725186, 2.3239338, 2.3795008, 2.4402218, 2.5075117, 2.5834658,
    2.6713916, 2.7769942, 2.7769942, 2.7769942, 2.7769942 };

  real_T c10_b_r;
  real_T c10_x;
  real_T c10_b_x;
  real_T c10_y;
  real_T c10_c_r;
  real_T c10_c_x;
  real_T c10_d_x;
  real_T c10_b_y;
  real_T c10_A;
  real_T c10_B;
  real_T c10_e_x;
  real_T c10_c_y;
  real_T c10_f_x;
  real_T c10_d_y;
  real_T c10_g_x;
  real_T c10_e_y;
  real_T c10_h_x;
  uint32_T c10_d_icng;
  uint32_T c10_d_jsr;
  uint32_T c10_e_jsr;
  uint32_T c10_e_icng;
  uint32_T c10_c_ui;
  real_T c10_f_y;
  real_T c10_s;
  real_T c10_i_x;
  real_T c10_j_x;
  real_T c10_b_A;
  real_T c10_b_B;
  real_T c10_k_x;
  real_T c10_g_y;
  real_T c10_l_x;
  real_T c10_h_y;
  real_T c10_m_x;
  real_T c10_i_y;
  real_T c10_j_y;
  uint32_T c10_f_icng;
  uint32_T c10_f_jsr;
  uint32_T c10_g_jsr;
  uint32_T c10_g_icng;
  uint32_T c10_d_ui;
  real_T c10_c_A;
  real_T c10_n_x;
  real_T c10_o_x;
  real_T c10_p_x;
  uint32_T c10_h_icng;
  uint32_T c10_h_jsr;
  uint32_T c10_i_jsr;
  uint32_T c10_i_icng;
  uint32_T c10_e_ui;
  (void)chartInstance;
  c10_icng = c10_e_state[0];
  c10_jsr = c10_e_state[1];
  c10_b_icng = c10_icng;
  c10_b_jsr = c10_jsr;
  c10_c_jsr = c10_b_jsr;
  c10_c_icng = c10_b_icng;
  c10_c_icng = 69069U * c10_c_icng + 1234567U;
  c10_c_jsr ^= c10_c_jsr << 13;
  c10_c_jsr ^= c10_c_jsr >> 17;
  c10_c_jsr ^= c10_c_jsr << 5;
  c10_ui = c10_c_icng + c10_c_jsr;
  c10_icng = c10_c_icng;
  c10_jsr = c10_c_jsr;
  c10_b_ui = c10_ui;
  c10_j = (c10_b_ui & 63U) + 1U;
  c10_j;
  c10_jp1 = c10_j + 1U;
  c10_i = (int32_T)c10_b_ui;
  c10_b_r = (real_T)c10_i * 4.6566128730773926E-10 *
    c10_dv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
    _SFD_INTEGER_CHECK("", (real_T)c10_jp1), 1, 65, 1, 0) - 1];
  c10_x = c10_b_r;
  c10_b_x = c10_x;
  c10_y = muDoubleScalarAbs(c10_b_x);
  if (c10_y <= c10_dv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
       _SFD_INTEGER_CHECK("", (real_T)c10_j), 1, 65, 1, 0) - 1]) {
    c10_c_r = c10_b_r;
  } else {
    c10_c_x = c10_b_r;
    c10_d_x = c10_c_x;
    c10_b_y = muDoubleScalarAbs(c10_d_x);
    c10_A = c10_b_y - c10_dv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
      _SFD_INTEGER_CHECK("", (real_T)c10_j), 1, 65, 1, 0) - 1];
    c10_B = c10_dv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
      _SFD_INTEGER_CHECK("", (real_T)c10_jp1), 1, 65, 1, 0) - 1] -
      c10_dv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
      _SFD_INTEGER_CHECK("", (real_T)c10_j), 1, 65, 1, 0) - 1];
    c10_e_x = c10_A;
    c10_c_y = c10_B;
    c10_f_x = c10_e_x;
    c10_d_y = c10_c_y;
    c10_g_x = c10_f_x;
    c10_e_y = c10_d_y;
    c10_h_x = c10_g_x / c10_e_y;
    c10_d_icng = c10_icng;
    c10_d_jsr = c10_jsr;
    c10_e_jsr = c10_d_jsr;
    c10_e_icng = c10_d_icng;
    c10_e_icng = 69069U * c10_e_icng + 1234567U;
    c10_e_jsr ^= c10_e_jsr << 13;
    c10_e_jsr ^= c10_e_jsr >> 17;
    c10_e_jsr ^= c10_e_jsr << 5;
    c10_c_ui = c10_e_icng + c10_e_jsr;
    c10_icng = c10_e_icng;
    c10_jsr = c10_e_jsr;
    c10_b_ui = c10_c_ui;
    c10_i = (int32_T)c10_b_ui;
    c10_f_y = 0.5 + (real_T)c10_i * 2.328306436538696E-10;
    c10_s = c10_h_x + c10_f_y;
    if (c10_s > 1.301198) {
      if (c10_b_r < 0.0) {
        c10_c_r = 0.4878992 * c10_h_x - 0.4878992;
      } else {
        c10_c_r = 0.4878992 - 0.4878992 * c10_h_x;
      }
    } else if (c10_s <= 0.9689279) {
      c10_c_r = c10_b_r;
    } else {
      c10_h_x = 0.4878992 - 0.4878992 * c10_h_x;
      if (c10_f_y > 12.67706 - 12.37586 * muDoubleScalarExp(-0.5 * c10_h_x *
           c10_h_x)) {
        if (c10_b_r < 0.0) {
          c10_c_r = -c10_h_x;
        } else {
          c10_c_r = c10_h_x;
        }
      } else {
        c10_i_x = -0.5 * c10_dv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          (uint32_T)_SFD_INTEGER_CHECK("", (real_T)c10_jp1), 1, 65, 1, 0) - 1] *
          c10_dv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
          _SFD_INTEGER_CHECK("", (real_T)c10_jp1), 1, 65, 1, 0) - 1];
        c10_j_x = c10_i_x;
        c10_j_x = muDoubleScalarExp(c10_j_x);
        c10_b_A = c10_f_y * 0.01958303;
        c10_b_B = c10_dv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
          _SFD_INTEGER_CHECK("", (real_T)c10_jp1), 1, 65, 1, 0) - 1];
        c10_k_x = c10_b_A;
        c10_g_y = c10_b_B;
        c10_l_x = c10_k_x;
        c10_h_y = c10_g_y;
        c10_m_x = c10_l_x;
        c10_i_y = c10_h_y;
        c10_j_y = c10_m_x / c10_i_y;
        if (c10_j_x + c10_j_y <= muDoubleScalarExp(-0.5 * c10_b_r * c10_b_r)) {
          c10_c_r = c10_b_r;
        } else {
          do {
            c10_f_icng = c10_icng;
            c10_f_jsr = c10_jsr;
            c10_g_jsr = c10_f_jsr;
            c10_g_icng = c10_f_icng;
            c10_g_icng = 69069U * c10_g_icng + 1234567U;
            c10_g_jsr ^= c10_g_jsr << 13;
            c10_g_jsr ^= c10_g_jsr >> 17;
            c10_g_jsr ^= c10_g_jsr << 5;
            c10_d_ui = c10_g_icng + c10_g_jsr;
            c10_icng = c10_g_icng;
            c10_jsr = c10_g_jsr;
            c10_b_ui = c10_d_ui;
            c10_i = (int32_T)c10_b_ui;
            c10_c_A = muDoubleScalarLog(0.5 + (real_T)c10_i *
              2.328306436538696E-10);
            c10_n_x = c10_c_A;
            c10_o_x = c10_n_x;
            c10_p_x = c10_o_x;
            c10_h_x = c10_p_x / 2.776994;
            c10_h_icng = c10_icng;
            c10_h_jsr = c10_jsr;
            c10_i_jsr = c10_h_jsr;
            c10_i_icng = c10_h_icng;
            c10_i_icng = 69069U * c10_i_icng + 1234567U;
            c10_i_jsr ^= c10_i_jsr << 13;
            c10_i_jsr ^= c10_i_jsr >> 17;
            c10_i_jsr ^= c10_i_jsr << 5;
            c10_e_ui = c10_i_icng + c10_i_jsr;
            c10_icng = c10_i_icng;
            c10_jsr = c10_i_jsr;
            c10_b_ui = c10_e_ui;
            c10_i = (int32_T)c10_b_ui;
          } while (!(-2.0 * muDoubleScalarLog(0.5 + (real_T)c10_i *
                     2.328306436538696E-10) > c10_h_x * c10_h_x));

          if (c10_b_r < 0.0) {
            c10_c_r = c10_h_x - 2.776994;
          } else {
            c10_c_r = 2.776994 - c10_h_x;
          }
        }
      }
    }
  }

  c10_e_state[0] = c10_icng;
  c10_e_state[1] = c10_jsr;
  return c10_c_r;
}

static void c10_b_twister_state_vector(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_mt[625], real_T c10_seed)
{
  real_T c10_d6;
  uint32_T c10_u6;
  uint32_T c10_r;
  int32_T c10_mti;
  real_T c10_b_mti;
  real_T c10_d7;
  uint32_T c10_u7;
  (void)chartInstance;
  c10_d6 = c10_seed;
  if (c10_d6 < 4.294967296E+9) {
    if (c10_d6 >= 0.0) {
      c10_u6 = (uint32_T)c10_d6;
    } else {
      c10_u6 = 0U;
    }
  } else if (c10_d6 >= 4.294967296E+9) {
    c10_u6 = MAX_uint32_T;
  } else {
    c10_u6 = 0U;
  }

  c10_r = c10_u6;
  c10_mt[0] = c10_r;
  for (c10_mti = 0; c10_mti < 623; c10_mti++) {
    c10_b_mti = 2.0 + (real_T)c10_mti;
    c10_d7 = muDoubleScalarRound(c10_b_mti - 1.0);
    if (c10_d7 < 4.294967296E+9) {
      if (c10_d7 >= 0.0) {
        c10_u7 = (uint32_T)c10_d7;
      } else {
        c10_u7 = 0U;
      }
    } else if (c10_d7 >= 4.294967296E+9) {
      c10_u7 = MAX_uint32_T;
    } else {
      c10_u7 = 0U;
    }

    c10_r = (c10_r ^ c10_r >> 30U) * 1812433253U + c10_u7;
    c10_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c10_b_mti), 1, 625, 1, 0) - 1] = c10_r;
  }

  c10_mt[624] = 624U;
}

static real_T c10_c_eml_rand_mt19937ar(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_e_state[625])
{
  uint32_T c10_u32[2];
  uint32_T c10_i;
  uint32_T c10_ip1;
  real_T c10_u;
  static real_T c10_dv1[257] = { 0.0, 0.215241895984875, 0.286174591792068,
    0.335737519214422, 0.375121332878378, 0.408389134611989, 0.43751840220787,
    0.46363433679088, 0.487443966139235, 0.50942332960209, 0.529909720661557,
    0.549151702327164, 0.567338257053817, 0.584616766106378, 0.601104617755991,
    0.61689699000775, 0.63207223638606, 0.646695714894993, 0.660822574244419,
    0.674499822837293, 0.687767892795788, 0.700661841106814, 0.713212285190975,
    0.725446140909999, 0.737387211434295, 0.749056662017815, 0.760473406430107,
    0.771654424224568, 0.782615023307232, 0.793369058840623, 0.80392911698997,
    0.814306670135215, 0.824512208752291, 0.834555354086381, 0.844444954909153,
    0.854189171008163, 0.863795545553308, 0.87327106808886, 0.882622229585165,
    0.891855070732941, 0.900975224461221, 0.909987953496718, 0.91889818364959,
    0.927710533401999, 0.936429340286575, 0.945058684468165, 0.953602409881086,
    0.96206414322304, 0.970447311064224, 0.978755155294224, 0.986990747099062,
    0.99515699963509, 1.00325667954467, 1.01129241744, 1.01926671746548,
    1.02718196603564, 1.03504043983344, 1.04284431314415, 1.05059566459093,
    1.05829648333067, 1.06594867476212, 1.07355406579244, 1.0811144097034,
    1.08863139065398, 1.09610662785202, 1.10354167942464, 1.11093804601357,
    1.11829717411934, 1.12562045921553, 1.13290924865253, 1.14016484436815,
    1.14738850542085, 1.15458145035993, 1.16174485944561, 1.16887987673083,
    1.17598761201545, 1.18306914268269, 1.19012551542669, 1.19715774787944,
    1.20416683014438, 1.2111537262437, 1.21811937548548, 1.22506469375653,
    1.23199057474614, 1.23889789110569, 1.24578749554863, 1.2526602218949,
    1.25951688606371, 1.26635828701823, 1.27318520766536, 1.27999841571382,
    1.28679866449324, 1.29358669373695, 1.30036323033084, 1.30712898903073,
    1.31388467315022, 1.32063097522106, 1.32736857762793, 1.33409815321936,
    1.3408203658964, 1.34753587118059, 1.35424531676263, 1.36094934303328,
    1.36764858359748, 1.37434366577317, 1.38103521107586, 1.38772383568998,
    1.39441015092814, 1.40109476367925, 1.4077782768464, 1.41446128977547,
    1.42114439867531, 1.42782819703026, 1.43451327600589, 1.44120022484872,
    1.44788963128058, 1.45458208188841, 1.46127816251028, 1.46797845861808,
    1.47468355569786, 1.48139403962819, 1.48811049705745, 1.49483351578049,
    1.50156368511546, 1.50830159628131, 1.51504784277671, 1.521803020761,
    1.52856772943771, 1.53534257144151, 1.542128153229, 1.54892508547417,
    1.55573398346918, 1.56255546753104, 1.56939016341512, 1.57623870273591,
    1.58310172339603, 1.58997987002419, 1.59687379442279, 1.60378415602609,
    1.61071162236983, 1.61765686957301, 1.62462058283303, 1.63160345693487,
    1.63860619677555, 1.64562951790478, 1.65267414708306, 1.65974082285818,
    1.66683029616166, 1.67394333092612, 1.68108070472517, 1.68824320943719,
    1.69543165193456, 1.70264685479992, 1.7098896570713, 1.71716091501782,
    1.72446150294804, 1.73179231405296, 1.73915426128591, 1.74654827828172,
    1.75397532031767, 1.76143636531891, 1.76893241491127, 1.77646449552452,
    1.78403365954944, 1.79164098655216, 1.79928758454972, 1.80697459135082,
    1.81470317596628, 1.82247454009388, 1.83028991968276, 1.83815058658281,
    1.84605785028518, 1.8540130597602, 1.86201760539967, 1.87007292107127,
    1.878180486293, 1.88634182853678, 1.8945585256707, 1.90283220855043,
    1.91116456377125, 1.91955733659319, 1.92801233405266, 1.93653142827569,
    1.94511656000868, 1.95376974238465, 1.96249306494436, 1.97128869793366,
    1.98015889690048, 1.98910600761744, 1.99813247135842, 2.00724083056053,
    2.0164337349062, 2.02571394786385, 2.03508435372962, 2.04454796521753,
    2.05410793165065, 2.06376754781173, 2.07353026351874, 2.0833996939983,
    2.09337963113879, 2.10347405571488, 2.11368715068665, 2.12402331568952,
    2.13448718284602, 2.14508363404789, 2.15581781987674, 2.16669518035431,
    2.17772146774029, 2.18890277162636, 2.20024554661128, 2.21175664288416,
    2.22344334009251, 2.23531338492992, 2.24737503294739, 2.25963709517379,
    2.27210899022838, 2.28480080272449, 2.29772334890286, 2.31088825060137,
    2.32430801887113, 2.33799614879653, 2.35196722737914, 2.36623705671729,
    2.38082279517208, 2.39574311978193, 2.41101841390112, 2.42667098493715,
    2.44272531820036, 2.4592083743347, 2.47614993967052, 2.49358304127105,
    2.51154444162669, 2.53007523215985, 2.54922155032478, 2.56903545268184,
    2.58957598670829, 2.61091051848882, 2.63311639363158, 2.65628303757674,
    2.68051464328574, 2.70593365612306, 2.73268535904401, 2.76094400527999,
    2.79092117400193, 2.82287739682644, 2.85713873087322, 2.89412105361341,
    2.93436686720889, 2.97860327988184, 3.02783779176959, 3.08352613200214,
    3.147889289518, 3.2245750520478, 3.32024473383983, 3.44927829856143,
    3.65415288536101, 3.91075795952492 };

  real_T c10_b_r;
  real_T c10_b_u;
  real_T c10_x;
  real_T c10_b_x;
  static real_T c10_dv2[257] = { 1.0, 0.977101701267673, 0.959879091800108,
    0.9451989534423, 0.932060075959231, 0.919991505039348, 0.908726440052131,
    0.898095921898344, 0.887984660755834, 0.878309655808918, 0.869008688036857,
    0.860033621196332, 0.851346258458678, 0.842915653112205, 0.834716292986884,
    0.826726833946222, 0.818929191603703, 0.811307874312656, 0.803849483170964,
    0.796542330422959, 0.789376143566025, 0.782341832654803, 0.775431304981187,
    0.768637315798486, 0.761953346836795, 0.755373506507096, 0.748892447219157,
    0.742505296340151, 0.736207598126863, 0.729995264561476, 0.72386453346863,
    0.717811932630722, 0.711834248878248, 0.705928501332754, 0.700091918136512,
    0.694321916126117, 0.688616083004672, 0.682972161644995, 0.677388036218774,
    0.671861719897082, 0.66639134390875, 0.660975147776663, 0.655611470579697,
    0.650298743110817, 0.645035480820822, 0.639820277453057, 0.634651799287624,
    0.629528779924837, 0.624450015547027, 0.619414360605834, 0.614420723888914,
    0.609468064925773, 0.604555390697468, 0.599681752619125, 0.594846243767987,
    0.590047996332826, 0.585286179263371, 0.580559996100791, 0.575868682972354,
    0.571211506735253, 0.566587763256165, 0.561996775814525, 0.557437893618766,
    0.552910490425833, 0.548413963255266, 0.543947731190026, 0.539511234256952,
    0.535103932380458, 0.530725304403662, 0.526374847171684, 0.522052074672322,
    0.517756517229756, 0.513487720747327, 0.509245245995748, 0.505028667943468,
    0.500837575126149, 0.49667156905249, 0.492530263643869, 0.488413284705458,
    0.484320269426683, 0.480250865909047, 0.476204732719506, 0.47218153846773,
    0.468180961405694, 0.464202689048174, 0.460246417812843, 0.456311852678716,
    0.452398706861849, 0.448506701507203, 0.444635565395739, 0.440785034665804,
    0.436954852547985, 0.433144769112652, 0.429354541029442, 0.425583931338022,
    0.421832709229496, 0.418100649837848, 0.414387534040891, 0.410693148270188,
    0.407017284329473, 0.403359739221114, 0.399720314980197, 0.396098818515832,
    0.392495061459315, 0.388908860018789, 0.385340034840077, 0.381788410873393,
    0.378253817245619, 0.374736087137891, 0.371235057668239, 0.367750569779032,
    0.364282468129004, 0.360830600989648, 0.357394820145781, 0.353974980800077,
    0.350570941481406, 0.347182563956794, 0.343809713146851, 0.340452257044522,
    0.337110066637006, 0.333783015830718, 0.330470981379163, 0.327173842813601,
    0.323891482376391, 0.320623784956905, 0.317370638029914, 0.314131931596337,
    0.310907558126286, 0.307697412504292, 0.30450139197665, 0.301319396100803,
    0.298151326696685, 0.294997087799962, 0.291856585617095, 0.288729728482183,
    0.285616426815502, 0.282516593083708, 0.279430141761638, 0.276356989295668,
    0.273297054068577, 0.270250256365875, 0.267216518343561, 0.264195763997261,
    0.261187919132721, 0.258192911337619, 0.255210669954662, 0.252241126055942,
    0.249284212418529, 0.246339863501264, 0.24340801542275, 0.240488605940501,
    0.237581574431238, 0.23468686187233, 0.231804410824339, 0.228934165414681,
    0.226076071322381, 0.223230075763918, 0.220396127480152, 0.217574176724331,
    0.214764175251174, 0.211966076307031, 0.209179834621125, 0.206405406397881,
    0.203642749310335, 0.200891822494657, 0.198152586545776, 0.195425003514135,
    0.192709036903589, 0.190004651670465, 0.187311814223801, 0.1846304924268,
    0.181960655599523, 0.179302274522848, 0.176655321443735, 0.174019770081839,
    0.171395595637506, 0.168782774801212, 0.166181285764482, 0.163591108232366,
    0.161012223437511, 0.158444614155925, 0.15588826472448, 0.153343161060263,
    0.150809290681846, 0.148286642732575, 0.145775208005994, 0.143274978973514,
    0.140785949814445, 0.138308116448551, 0.135841476571254, 0.133386029691669,
    0.130941777173644, 0.12850872228, 0.126086870220186, 0.123676228201597,
    0.12127680548479, 0.11888861344291, 0.116511665625611, 0.114145977827839,
    0.111791568163838, 0.109448457146812, 0.107116667774684, 0.104796225622487,
    0.102487158941935, 0.10018949876881, 0.0979032790388625, 0.095628536713009,
    0.093365311912691, 0.0911136480663738, 0.0888735920682759,
    0.0866451944505581, 0.0844285095703535, 0.082223595813203,
    0.0800305158146631, 0.0778493367020961, 0.0756801303589272,
    0.0735229737139814, 0.0713779490588905, 0.0692451443970068,
    0.0671246538277886, 0.065016577971243, 0.0629210244377582, 0.06083810834954,
    0.0587679529209339, 0.0567106901062031, 0.0546664613248891,
    0.0526354182767924, 0.0506177238609479, 0.0486135532158687,
    0.0466230949019305, 0.0446465522512946, 0.0426841449164746,
    0.0407361106559411, 0.0388027074045262, 0.0368842156885674,
    0.0349809414617162, 0.0330932194585786, 0.0312214171919203,
    0.0293659397581334, 0.0275272356696031, 0.0257058040085489,
    0.0239022033057959, 0.0221170627073089, 0.0203510962300445,
    0.0186051212757247, 0.0168800831525432, 0.0151770883079353,
    0.0134974506017399, 0.0118427578579079, 0.0102149714397015,
    0.00861658276939875, 0.00705087547137324, 0.00552240329925101,
    0.00403797259336304, 0.00260907274610216, 0.0012602859304986,
    0.000477467764609386 };

  real_T c10_c_u;
  real_T c10_c_x;
  real_T c10_d_u;
  int32_T exitg1;
  c10_assert_valid_state(chartInstance);
  do {
    exitg1 = 0;
    c10_b_genrand_uint32_vector(chartInstance, c10_e_state, c10_u32);
    c10_i = (c10_u32[1] >> 24U) + 1U;
    c10_ip1 = c10_i + 1U;
    c10_u = ((real_T)(c10_u32[0] >> 3U) * 1.6777216E+7 + (real_T)(c10_u32[1] &
              16777215U)) * 2.2204460492503131E-16 - 1.0;
    c10_b_r = c10_u * c10_dv1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
      _SFD_INTEGER_CHECK("", (real_T)c10_ip1), 1, 257, 1, 0) - 1];
    if (c10_abs(chartInstance, c10_b_r) <= c10_dv1[_SFD_EML_ARRAY_BOUNDS_CHECK(
         "", (int32_T)(uint32_T)_SFD_INTEGER_CHECK("", (real_T)c10_i), 1, 257, 1,
         0) - 1]) {
      exitg1 = 1;
    } else if ((real_T)c10_i < 256.0) {
      c10_b_u = c10_c_genrandu(chartInstance, c10_e_state);
      c10_u = c10_b_u;
      c10_x = -0.5 * c10_b_r * c10_b_r;
      c10_b_x = c10_x;
      c10_b_x = muDoubleScalarExp(c10_b_x);
      if (c10_dv2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
           _SFD_INTEGER_CHECK("", (real_T)c10_ip1), 1, 257, 1, 0) - 1] + c10_u *
          (c10_dv2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
            _SFD_INTEGER_CHECK("", (real_T)c10_i), 1, 257, 1, 0) - 1] -
           c10_dv2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
            _SFD_INTEGER_CHECK("", (real_T)c10_ip1), 1, 257, 1, 0) - 1]) <
          c10_b_x) {
        exitg1 = 1;
      }
    } else {
      do {
        c10_c_u = c10_c_genrandu(chartInstance, c10_e_state);
        c10_u = c10_c_u;
        c10_c_x = muDoubleScalarLog(c10_u) * 0.273661237329758;
        c10_d_u = c10_c_genrandu(chartInstance, c10_e_state);
        c10_u = c10_d_u;
      } while (!(-2.0 * muDoubleScalarLog(c10_u) > c10_c_x * c10_c_x));

      if (c10_b_r < 0.0) {
        c10_b_r = c10_c_x - 3.65415288536101;
      } else {
        c10_b_r = 3.65415288536101 - c10_c_x;
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c10_b_r;
}

static void c10_b_genrand_uint32_vector(SFc10_SS6_EstimationInstanceStruct
  *chartInstance, uint32_T c10_mt[625], uint32_T c10_u[2])
{
  int32_T c10_i20;
  int32_T c10_j;
  real_T c10_b_j;
  uint32_T c10_mti;
  int32_T c10_kk;
  real_T c10_b_kk;
  uint32_T c10_y;
  uint32_T c10_b_y;
  uint32_T c10_c_y;
  int32_T c10_c_kk;
  uint32_T c10_d_y;
  uint32_T c10_e_y;
  uint32_T c10_f_y;
  uint32_T c10_g_y;
  (void)chartInstance;
  for (c10_i20 = 0; c10_i20 < 2; c10_i20++) {
    c10_u[c10_i20] = 0U;
  }

  for (c10_j = 0; c10_j < 2; c10_j++) {
    c10_b_j = 1.0 + (real_T)c10_j;
    c10_mti = c10_mt[624] + 1U;
    if ((real_T)c10_mti >= 625.0) {
      for (c10_kk = 0; c10_kk < 227; c10_kk++) {
        c10_b_kk = 1.0 + (real_T)c10_kk;
        c10_y = (c10_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                  _SFD_INTEGER_CHECK("", c10_b_kk), 1, 625, 1, 0) - 1] &
                 2147483648U) | (c10_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", c10_b_kk + 1.0), 1, 625, 1, 0) - 1] &
          2147483647U);
        c10_b_y = c10_y;
        c10_c_y = c10_b_y;
        if ((real_T)(c10_c_y & 1U) == 0.0) {
          c10_c_y >>= 1U;
        } else {
          c10_c_y = c10_c_y >> 1U ^ 2567483615U;
        }

        c10_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          c10_b_kk), 1, 625, 1, 0) - 1] = c10_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", c10_b_kk + 397.0), 1, 625, 1, 0) - 1] ^
          c10_c_y;
      }

      for (c10_c_kk = 0; c10_c_kk < 396; c10_c_kk++) {
        c10_b_kk = 228.0 + (real_T)c10_c_kk;
        c10_y = (c10_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                  _SFD_INTEGER_CHECK("", c10_b_kk), 1, 625, 1, 0) - 1] &
                 2147483648U) | (c10_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", c10_b_kk + 1.0), 1, 625, 1, 0) - 1] &
          2147483647U);
        c10_d_y = c10_y;
        c10_e_y = c10_d_y;
        if ((real_T)(c10_e_y & 1U) == 0.0) {
          c10_e_y >>= 1U;
        } else {
          c10_e_y = c10_e_y >> 1U ^ 2567483615U;
        }

        c10_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          c10_b_kk), 1, 625, 1, 0) - 1] = c10_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (c10_b_kk + 1.0) - 228.0), 1, 625, 1,
          0) - 1] ^ c10_e_y;
      }

      c10_y = (c10_mt[623] & 2147483648U) | (c10_mt[0] & 2147483647U);
      c10_f_y = c10_y;
      c10_g_y = c10_f_y;
      if ((real_T)(c10_g_y & 1U) == 0.0) {
        c10_g_y >>= 1U;
      } else {
        c10_g_y = c10_g_y >> 1U ^ 2567483615U;
      }

      c10_mt[623] = c10_mt[396] ^ c10_g_y;
      c10_mti = 1U;
    }

    c10_y = c10_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
      _SFD_INTEGER_CHECK("", (real_T)c10_mti), 1, 625, 1, 0) - 1];
    c10_mt[624] = c10_mti;
    c10_y ^= c10_y >> 11U;
    c10_y ^= c10_y << 7U & 2636928640U;
    c10_y ^= c10_y << 15U & 4022730752U;
    c10_y ^= c10_y >> 18U;
    c10_u[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c10_b_j), 1, 2, 1, 0) - 1] = c10_y;
  }
}

static real_T c10_c_genrandu(SFc10_SS6_EstimationInstanceStruct *chartInstance,
  uint32_T c10_mt[625])
{
  real_T c10_r;
  uint32_T c10_u[2];
  boolean_T c10_b0;
  boolean_T c10_isvalid;
  int32_T c10_k;
  int32_T c10_a;
  int32_T c10_b_a;
  boolean_T guard1 = false;
  int32_T exitg1;
  boolean_T exitg2;

  /* ========================= COPYRIGHT NOTICE ============================ */
  /*  This is a uniform (0,1) pseudorandom number generator based on:        */
  /*                                                                         */
  /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
  /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
  /*                                                                         */
  /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
  /*  All rights reserved.                                                   */
  /*                                                                         */
  /*  Redistribution and use in source and binary forms, with or without     */
  /*  modification, are permitted provided that the following conditions     */
  /*  are met:                                                               */
  /*                                                                         */
  /*    1. Redistributions of source code must retain the above copyright    */
  /*       notice, this list of conditions and the following disclaimer.     */
  /*                                                                         */
  /*    2. Redistributions in binary form must reproduce the above copyright */
  /*       notice, this list of conditions and the following disclaimer      */
  /*       in the documentation and/or other materials provided with the     */
  /*       distribution.                                                     */
  /*                                                                         */
  /*    3. The names of its contributors may not be used to endorse or       */
  /*       promote products derived from this software without specific      */
  /*       prior written permission.                                         */
  /*                                                                         */
  /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
  /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
  /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
  /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
  /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
  /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
  /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
  /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
  /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
  /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
  /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
  /*                                                                         */
  /* =============================   END   ================================= */
  do {
    exitg1 = 0;
    c10_b_genrand_uint32_vector(chartInstance, c10_mt, c10_u);
    c10_u[0] >>= 5U;
    c10_u[1] >>= 6U;
    c10_r = 1.1102230246251565E-16 * ((real_T)c10_u[0] * 6.7108864E+7 + (real_T)
      c10_u[1]);
    if (c10_r == 0.0) {
      guard1 = false;
      if ((real_T)c10_mt[624] >= 1.0) {
        if ((real_T)c10_mt[624] < 625.0) {
          c10_b0 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1 == true) {
        c10_b0 = false;
      }

      c10_isvalid = c10_b0;
      if (c10_isvalid) {
        c10_isvalid = false;
        c10_k = 1;
        exitg2 = false;
        while ((exitg2 == false) && (c10_k < 625)) {
          if ((real_T)c10_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
               _SFD_INTEGER_CHECK("", (real_T)c10_k), 1, 625, 1, 0) - 1] == 0.0)
          {
            c10_a = c10_k;
            c10_b_a = c10_a + 1;
            c10_k = c10_b_a;
          } else {
            c10_isvalid = true;
            exitg2 = true;
          }
        }
      }

      if (!c10_isvalid) {
        c10_eml_error(chartInstance);
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c10_r;
}

static void init_dsm_address_info(SFc10_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc10_SS6_EstimationInstanceStruct
  *chartInstance)
{
  chartInstance->c10_w_L = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c10_w_R = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c10_count_L = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c10_count_R = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
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

void sf_c10_SS6_Estimation_get_check_sum(mxArray *plhs[])
{
<<<<<<< HEAD
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(392965637U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(285888733U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2132687234U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3883696371U);
=======
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(839902063U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(989001978U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2988198943U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2418329351U);
>>>>>>> 97c418b1f8209f4cdb2c89b2f22d3af95f3621c9
}

mxArray* sf_c10_SS6_Estimation_get_post_codegen_info(void);
mxArray *sf_c10_SS6_Estimation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
<<<<<<< HEAD
    mxArray *mxChecksum = mxCreateString("vRU7xjquqLOo3QxzmQF1fH");
=======
    mxArray *mxChecksum = mxCreateString("IbmeFroW1325KPZ8OZw1rH");
>>>>>>> 97c418b1f8209f4cdb2c89b2f22d3af95f3621c9
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c10_SS6_Estimation_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c10_SS6_Estimation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c10_SS6_Estimation_jit_fallback_info(void)
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

mxArray *sf_c10_SS6_Estimation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c10_SS6_Estimation_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c10_SS6_Estimation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x9'type','srcId','name','auxInfo'{{M[1],M[6],T\"count_L\",},{M[1],M[7],T\"count_R\",},{M[4],M[0],T\"method\",S'l','i','p'{{M1x2[512 518],M[1],T\"C:\\Program Files\\MATLAB\\R2014b\\toolbox\\eml\\lib\\matlab\\randfun\\eml_rand.m\"}}},{M[4],M[0],T\"method\",S'l','i','p'{{M1x2[638 644],M[1],T\"C:\\Program Files\\MATLAB\\R2014b\\toolbox\\eml\\lib\\matlab\\randfun\\eml_randn.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[165 170],M[1],T\"C:\\Program Files\\MATLAB\\R2014b\\toolbox\\eml\\lib\\matlab\\randfun\\eml_rand_mcg16807_stateful.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[166 171],M[1],T\"C:\\Program Files\\MATLAB\\R2014b\\toolbox\\eml\\lib\\matlab\\randfun\\eml_rand_mt19937ar_stateful.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[165 170],M[1],T\"C:\\Program Files\\MATLAB\\R2014b\\toolbox\\eml\\lib\\matlab\\randfun\\eml_rand_shr3cong_stateful.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[645 650],M[1],T\"C:\\Program Files\\MATLAB\\R2014b\\toolbox\\eml\\lib\\matlab\\randfun\\eml_randn.m\"}}},{M[8],M[0],T\"is_active_c10_SS6_Estimation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 9, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c10_SS6_Estimation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc10_SS6_EstimationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc10_SS6_EstimationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SS6_EstimationMachineNumber_,
           10,
           1,
           1,
           0,
           4,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_SS6_EstimationMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_SS6_EstimationMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _SS6_EstimationMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"w_L");
          _SFD_SET_DATA_PROPS(1,1,1,0,"w_R");
          _SFD_SET_DATA_PROPS(2,2,0,1,"count_L");
          _SFD_SET_DATA_PROPS(3,2,0,1,"count_R");
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
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
<<<<<<< HEAD
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,119);
=======
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,115);
>>>>>>> 97c418b1f8209f4cdb2c89b2f22d3af95f3621c9
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_sf_marshallOut,(MexInFcnForType)c10_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_sf_marshallOut,(MexInFcnForType)c10_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c10_w_L);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c10_w_R);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c10_count_L);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c10_count_R);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _SS6_EstimationMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
<<<<<<< HEAD
  return "F7ISXg0e6e4OH33X2IsQjD";
=======
  return "fCLYWgHf1MQ0lo0Biz9uCC";
>>>>>>> 97c418b1f8209f4cdb2c89b2f22d3af95f3621c9
}

static void sf_opaque_initialize_c10_SS6_Estimation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc10_SS6_EstimationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c10_SS6_Estimation((SFc10_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
  initialize_c10_SS6_Estimation((SFc10_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c10_SS6_Estimation(void *chartInstanceVar)
{
  enable_c10_SS6_Estimation((SFc10_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c10_SS6_Estimation(void *chartInstanceVar)
{
  disable_c10_SS6_Estimation((SFc10_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c10_SS6_Estimation(void *chartInstanceVar)
{
  sf_gateway_c10_SS6_Estimation((SFc10_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c10_SS6_Estimation(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c10_SS6_Estimation((SFc10_SS6_EstimationInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c10_SS6_Estimation(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c10_SS6_Estimation((SFc10_SS6_EstimationInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c10_SS6_Estimation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc10_SS6_EstimationInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SS6_Estimation_optimization_info();
    }

    finalize_c10_SS6_Estimation((SFc10_SS6_EstimationInstanceStruct*)
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
  initSimStructsc10_SS6_Estimation((SFc10_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c10_SS6_Estimation(SimStruct *S)
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
    initialize_params_c10_SS6_Estimation((SFc10_SS6_EstimationInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c10_SS6_Estimation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SS6_Estimation_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      10);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,10,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,10,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,10);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,10,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,10,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,10);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
<<<<<<< HEAD
  ssSetChecksum0(S,(1078771331U));
  ssSetChecksum1(S,(3881980995U));
  ssSetChecksum2(S,(1990217784U));
  ssSetChecksum3(S,(86088184U));
=======
  ssSetChecksum0(S,(763183646U));
  ssSetChecksum1(S,(574690450U));
  ssSetChecksum2(S,(400044273U));
  ssSetChecksum3(S,(2329621277U));
>>>>>>> 97c418b1f8209f4cdb2c89b2f22d3af95f3621c9
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c10_SS6_Estimation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c10_SS6_Estimation(SimStruct *S)
{
  SFc10_SS6_EstimationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc10_SS6_EstimationInstanceStruct *)utMalloc(sizeof
    (SFc10_SS6_EstimationInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc10_SS6_EstimationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c10_SS6_Estimation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c10_SS6_Estimation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c10_SS6_Estimation;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c10_SS6_Estimation;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c10_SS6_Estimation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c10_SS6_Estimation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c10_SS6_Estimation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c10_SS6_Estimation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c10_SS6_Estimation;
  chartInstance->chartInfo.mdlStart = mdlStart_c10_SS6_Estimation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c10_SS6_Estimation;
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

void c10_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c10_SS6_Estimation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c10_SS6_Estimation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c10_SS6_Estimation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c10_SS6_Estimation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
