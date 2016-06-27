/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SS6_Estimation_sfun.h"
#include "c19_SS6_Estimation.h"
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
static const char * c19_debug_family_names[13] = { "Ts", "nargin", "nargout",
  "aX", "aY", "r", "X_pos", "Y_pos", "Vx", "Vy", "yaw", "X", "Y" };

/* Function Declarations */
static void initialize_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance);
static void initialize_params_c19_SS6_Estimation
  (SFc19_SS6_EstimationInstanceStruct *chartInstance);
static void enable_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance);
static void disable_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance);
static void c19_update_debugger_state_c19_SS6_Estimation
  (SFc19_SS6_EstimationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c19_SS6_Estimation
  (SFc19_SS6_EstimationInstanceStruct *chartInstance);
static void set_sim_state_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct *
  chartInstance, const mxArray *c19_st);
static void finalize_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance);
static void sf_gateway_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance);
static void mdl_start_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance);
static void initSimStructsc19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c19_machineNumber, uint32_T
  c19_chartNumber, uint32_T c19_instanceNumber);
static const mxArray *c19_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static real_T c19_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_Y, const char_T *c19_identifier);
static real_T c19_b_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_b_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static real_T c19_c_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_X, const char_T *c19_identifier);
static real_T c19_d_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_c_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static real_T c19_e_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_yaw, const char_T *c19_identifier);
static real_T c19_f_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_d_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static real_T c19_g_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_Vy, const char_T *c19_identifier);
static real_T c19_h_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_e_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static real_T c19_i_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_Vx, const char_T *c19_identifier);
static real_T c19_j_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_f_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static real_T c19_k_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_Y_pos, const char_T *c19_identifier);
static real_T c19_l_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_g_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static int32_T c19_m_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static uint8_T c19_n_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_is_active_c19_SS6_Estimation, const
  char_T *c19_identifier);
static uint8_T c19_o_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void init_dsm_address_info(SFc19_SS6_EstimationInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc19_SS6_EstimationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance)
{
  chartInstance->c19_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c19_Vx_not_empty = false;
  chartInstance->c19_Vy_not_empty = false;
  chartInstance->c19_yaw_not_empty = false;
  chartInstance->c19_X_not_empty = false;
  chartInstance->c19_Y_not_empty = false;
  chartInstance->c19_is_active_c19_SS6_Estimation = 0U;
}

static void initialize_params_c19_SS6_Estimation
  (SFc19_SS6_EstimationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c19_update_debugger_state_c19_SS6_Estimation
  (SFc19_SS6_EstimationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c19_SS6_Estimation
  (SFc19_SS6_EstimationInstanceStruct *chartInstance)
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
  real_T c19_e_hoistedGlobal;
  real_T c19_e_u;
  const mxArray *c19_f_y = NULL;
  real_T c19_f_hoistedGlobal;
  real_T c19_f_u;
  const mxArray *c19_g_y = NULL;
  real_T c19_g_hoistedGlobal;
  real_T c19_g_u;
  const mxArray *c19_h_y = NULL;
  uint8_T c19_h_hoistedGlobal;
  uint8_T c19_h_u;
  const mxArray *c19_i_y = NULL;
  c19_st = NULL;
  c19_st = NULL;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_createcellmatrix(8, 1), false);
  c19_hoistedGlobal = *chartInstance->c19_X_pos;
  c19_u = c19_hoistedGlobal;
  c19_b_y = NULL;
  sf_mex_assign(&c19_b_y, sf_mex_create("y", &c19_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c19_y, 0, c19_b_y);
  c19_b_hoistedGlobal = *chartInstance->c19_Y_pos;
  c19_b_u = c19_b_hoistedGlobal;
  c19_c_y = NULL;
  sf_mex_assign(&c19_c_y, sf_mex_create("y", &c19_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c19_y, 1, c19_c_y);
  c19_c_hoistedGlobal = chartInstance->c19_Vx;
  c19_c_u = c19_c_hoistedGlobal;
  c19_d_y = NULL;
  if (!chartInstance->c19_Vx_not_empty) {
    sf_mex_assign(&c19_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_d_y, sf_mex_create("y", &c19_c_u, 0, 0U, 0U, 0U, 0),
                  false);
  }

  sf_mex_setcell(c19_y, 2, c19_d_y);
  c19_d_hoistedGlobal = chartInstance->c19_Vy;
  c19_d_u = c19_d_hoistedGlobal;
  c19_e_y = NULL;
  if (!chartInstance->c19_Vy_not_empty) {
    sf_mex_assign(&c19_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_e_y, sf_mex_create("y", &c19_d_u, 0, 0U, 0U, 0U, 0),
                  false);
  }

  sf_mex_setcell(c19_y, 3, c19_e_y);
  c19_e_hoistedGlobal = chartInstance->c19_X;
  c19_e_u = c19_e_hoistedGlobal;
  c19_f_y = NULL;
  if (!chartInstance->c19_X_not_empty) {
    sf_mex_assign(&c19_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_f_y, sf_mex_create("y", &c19_e_u, 0, 0U, 0U, 0U, 0),
                  false);
  }

  sf_mex_setcell(c19_y, 4, c19_f_y);
  c19_f_hoistedGlobal = chartInstance->c19_Y;
  c19_f_u = c19_f_hoistedGlobal;
  c19_g_y = NULL;
  if (!chartInstance->c19_Y_not_empty) {
    sf_mex_assign(&c19_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_g_y, sf_mex_create("y", &c19_f_u, 0, 0U, 0U, 0U, 0),
                  false);
  }

  sf_mex_setcell(c19_y, 5, c19_g_y);
  c19_g_hoistedGlobal = chartInstance->c19_yaw;
  c19_g_u = c19_g_hoistedGlobal;
  c19_h_y = NULL;
  if (!chartInstance->c19_yaw_not_empty) {
    sf_mex_assign(&c19_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_h_y, sf_mex_create("y", &c19_g_u, 0, 0U, 0U, 0U, 0),
                  false);
  }

  sf_mex_setcell(c19_y, 6, c19_h_y);
  c19_h_hoistedGlobal = chartInstance->c19_is_active_c19_SS6_Estimation;
  c19_h_u = c19_h_hoistedGlobal;
  c19_i_y = NULL;
  sf_mex_assign(&c19_i_y, sf_mex_create("y", &c19_h_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c19_y, 7, c19_i_y);
  sf_mex_assign(&c19_st, c19_y, false);
  return c19_st;
}

static void set_sim_state_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct *
  chartInstance, const mxArray *c19_st)
{
  const mxArray *c19_u;
  chartInstance->c19_doneDoubleBufferReInit = true;
  c19_u = sf_mex_dup(c19_st);
  *chartInstance->c19_X_pos = c19_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c19_u, 0)), "X_pos");
  *chartInstance->c19_Y_pos = c19_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c19_u, 1)), "Y_pos");
  chartInstance->c19_Vx = c19_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c19_u, 2)), "Vx");
  chartInstance->c19_Vy = c19_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c19_u, 3)), "Vy");
  chartInstance->c19_X = c19_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c19_u, 4)), "X");
  chartInstance->c19_Y = c19_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c19_u, 5)), "Y");
  chartInstance->c19_yaw = c19_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c19_u, 6)), "yaw");
  chartInstance->c19_is_active_c19_SS6_Estimation = c19_n_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c19_u, 7)),
     "is_active_c19_SS6_Estimation");
  sf_mex_destroy(&c19_u);
  c19_update_debugger_state_c19_SS6_Estimation(chartInstance);
  sf_mex_destroy(&c19_st);
}

static void finalize_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance)
{
  real_T c19_hoistedGlobal;
  real_T c19_b_hoistedGlobal;
  real_T c19_c_hoistedGlobal;
  real_T c19_b_aX;
  real_T c19_b_aY;
  real_T c19_b_r;
  uint32_T c19_debug_family_var_map[13];
  real_T c19_Ts;
  real_T c19_nargin = 3.0;
  real_T c19_nargout = 2.0;
  real_T c19_b_X_pos;
  real_T c19_b_Y_pos;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 18U, chartInstance->c19_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_aX, 0U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_aY, 1U);
  chartInstance->c19_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 18U, chartInstance->c19_sfEvent);
  c19_hoistedGlobal = *chartInstance->c19_aX;
  c19_b_hoistedGlobal = *chartInstance->c19_aY;
  c19_c_hoistedGlobal = *chartInstance->c19_r;
  c19_b_aX = c19_hoistedGlobal;
  c19_b_aY = c19_b_hoistedGlobal;
  c19_b_r = c19_c_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 13U, 13U, c19_debug_family_names,
    c19_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c19_Ts, 0U, c19_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_nargin, 1U, c19_f_sf_marshallOut,
    c19_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_nargout, 2U, c19_f_sf_marshallOut,
    c19_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c19_b_aX, 3U, c19_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c19_b_aY, 4U, c19_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c19_b_r, 5U, c19_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_b_X_pos, 6U, c19_f_sf_marshallOut,
    c19_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_b_Y_pos, 7U, c19_f_sf_marshallOut,
    c19_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c19_Vx, 8U,
    c19_e_sf_marshallOut, c19_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c19_Vy, 9U,
    c19_d_sf_marshallOut, c19_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c19_yaw, 10U,
    c19_c_sf_marshallOut, c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c19_X, 11U,
    c19_b_sf_marshallOut, c19_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c19_Y, 12U,
    c19_sf_marshallOut, c19_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 3);
  c19_Ts = 0.01;
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 6);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c19_X_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 7);
    chartInstance->c19_Vx = 0.0;
    chartInstance->c19_Vx_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 8);
    chartInstance->c19_Vy = 0.0;
    chartInstance->c19_Vy_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 9);
    chartInstance->c19_yaw = 0.0;
    chartInstance->c19_yaw_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 10);
    chartInstance->c19_X = 0.0;
    chartInstance->c19_X_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 11);
    chartInstance->c19_Y = 0.0;
    chartInstance->c19_Y_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 14);
  chartInstance->c19_Vx += 0.5 * c19_b_aX * 0.01 * 0.01;
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 15);
  chartInstance->c19_Vy += 0.5 * c19_b_aY * 0.01 * 0.01;
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 16);
  chartInstance->c19_yaw += 0.5 * c19_b_r * 0.01 * 0.01;
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 18);
  chartInstance->c19_X += chartInstance->c19_Vx * 0.01;
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 19);
  chartInstance->c19_Y += chartInstance->c19_Vy * 0.01;
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 21);
  c19_b_X_pos = chartInstance->c19_Vx;
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 21);
  c19_b_Y_pos = chartInstance->c19_Vy;
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, -21);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c19_X_pos = c19_b_X_pos;
  *chartInstance->c19_Y_pos = c19_b_Y_pos;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 18U, chartInstance->c19_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SS6_EstimationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_X_pos, 2U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_Y_pos, 3U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c19_r, 4U);
}

static void mdl_start_c19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc19_SS6_Estimation(SFc19_SS6_EstimationInstanceStruct *
  chartInstance)
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
  real_T c19_u;
  const mxArray *c19_y = NULL;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(real_T *)c19_inData;
  c19_y = NULL;
  if (!chartInstance->c19_Y_not_empty) {
    sf_mex_assign(&c19_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static real_T c19_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_Y, const char_T *c19_identifier)
{
  real_T c19_y;
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_Y), &c19_thisId);
  sf_mex_destroy(&c19_b_Y);
  return c19_y;
}

static real_T c19_b_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  real_T c19_y;
  real_T c19_d0;
  if (mxIsEmpty(c19_u)) {
    chartInstance->c19_Y_not_empty = false;
  } else {
    chartInstance->c19_Y_not_empty = true;
    sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_d0, 1, 0, 0U, 0, 0U, 0);
    c19_y = c19_d0;
  }

  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_Y;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_b_Y = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_Y), &c19_thisId);
  sf_mex_destroy(&c19_b_Y);
  *(real_T *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_b_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  real_T c19_u;
  const mxArray *c19_y = NULL;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(real_T *)c19_inData;
  c19_y = NULL;
  if (!chartInstance->c19_X_not_empty) {
    sf_mex_assign(&c19_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static real_T c19_c_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_X, const char_T *c19_identifier)
{
  real_T c19_y;
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_X), &c19_thisId);
  sf_mex_destroy(&c19_b_X);
  return c19_y;
}

static real_T c19_d_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  real_T c19_y;
  real_T c19_d1;
  if (mxIsEmpty(c19_u)) {
    chartInstance->c19_X_not_empty = false;
  } else {
    chartInstance->c19_X_not_empty = true;
    sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_d1, 1, 0, 0U, 0, 0U, 0);
    c19_y = c19_d1;
  }

  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_X;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_b_X = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_X), &c19_thisId);
  sf_mex_destroy(&c19_b_X);
  *(real_T *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_c_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  real_T c19_u;
  const mxArray *c19_y = NULL;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(real_T *)c19_inData;
  c19_y = NULL;
  if (!chartInstance->c19_yaw_not_empty) {
    sf_mex_assign(&c19_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static real_T c19_e_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_yaw, const char_T *c19_identifier)
{
  real_T c19_y;
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_yaw),
    &c19_thisId);
  sf_mex_destroy(&c19_b_yaw);
  return c19_y;
}

static real_T c19_f_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  real_T c19_y;
  real_T c19_d2;
  if (mxIsEmpty(c19_u)) {
    chartInstance->c19_yaw_not_empty = false;
  } else {
    chartInstance->c19_yaw_not_empty = true;
    sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_d2, 1, 0, 0U, 0, 0U, 0);
    c19_y = c19_d2;
  }

  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_yaw;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_b_yaw = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_yaw),
    &c19_thisId);
  sf_mex_destroy(&c19_b_yaw);
  *(real_T *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_d_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  real_T c19_u;
  const mxArray *c19_y = NULL;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(real_T *)c19_inData;
  c19_y = NULL;
  if (!chartInstance->c19_Vy_not_empty) {
    sf_mex_assign(&c19_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static real_T c19_g_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_Vy, const char_T *c19_identifier)
{
  real_T c19_y;
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_Vy),
    &c19_thisId);
  sf_mex_destroy(&c19_b_Vy);
  return c19_y;
}

static real_T c19_h_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  real_T c19_y;
  real_T c19_d3;
  if (mxIsEmpty(c19_u)) {
    chartInstance->c19_Vy_not_empty = false;
  } else {
    chartInstance->c19_Vy_not_empty = true;
    sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_d3, 1, 0, 0U, 0, 0U, 0);
    c19_y = c19_d3;
  }

  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_Vy;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_b_Vy = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_Vy),
    &c19_thisId);
  sf_mex_destroy(&c19_b_Vy);
  *(real_T *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_e_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  real_T c19_u;
  const mxArray *c19_y = NULL;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(real_T *)c19_inData;
  c19_y = NULL;
  if (!chartInstance->c19_Vx_not_empty) {
    sf_mex_assign(&c19_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static real_T c19_i_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_Vx, const char_T *c19_identifier)
{
  real_T c19_y;
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_Vx),
    &c19_thisId);
  sf_mex_destroy(&c19_b_Vx);
  return c19_y;
}

static real_T c19_j_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  real_T c19_y;
  real_T c19_d4;
  if (mxIsEmpty(c19_u)) {
    chartInstance->c19_Vx_not_empty = false;
  } else {
    chartInstance->c19_Vx_not_empty = true;
    sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_d4, 1, 0, 0U, 0, 0U, 0);
    c19_y = c19_d4;
  }

  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_Vx;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_b_Vx = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_Vx),
    &c19_thisId);
  sf_mex_destroy(&c19_b_Vx);
  *(real_T *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_f_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  real_T c19_u;
  const mxArray *c19_y = NULL;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(real_T *)c19_inData;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static real_T c19_k_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_Y_pos, const char_T *c19_identifier)
{
  real_T c19_y;
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_Y_pos),
    &c19_thisId);
  sf_mex_destroy(&c19_b_Y_pos);
  return c19_y;
}

static real_T c19_l_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  real_T c19_y;
  real_T c19_d5;
  (void)chartInstance;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_d5, 1, 0, 0U, 0, 0U, 0);
  c19_y = c19_d5;
  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_Y_pos;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_b_Y_pos = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_Y_pos),
    &c19_thisId);
  sf_mex_destroy(&c19_b_Y_pos);
  *(real_T *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

const mxArray *sf_c19_SS6_Estimation_get_eml_resolved_functions_info(void)
{
  const mxArray *c19_nameCaptureInfo = NULL;
  c19_nameCaptureInfo = NULL;
  sf_mex_assign(&c19_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c19_nameCaptureInfo;
}

static const mxArray *c19_g_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_u;
  const mxArray *c19_y = NULL;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(int32_T *)c19_inData;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, false);
  return c19_mxArrayOutData;
}

static int32_T c19_m_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  int32_T c19_y;
  int32_T c19_i0;
  (void)chartInstance;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_i0, 1, 6, 0U, 0, 0U, 0);
  c19_y = c19_i0;
  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_sfEvent;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  int32_T c19_y;
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c19_b_sfEvent = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_sfEvent),
    &c19_thisId);
  sf_mex_destroy(&c19_b_sfEvent);
  *(int32_T *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

static uint8_T c19_n_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c19_b_is_active_c19_SS6_Estimation, const
  char_T *c19_identifier)
{
  uint8_T c19_y;
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_o_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c19_b_is_active_c19_SS6_Estimation), &c19_thisId);
  sf_mex_destroy(&c19_b_is_active_c19_SS6_Estimation);
  return c19_y;
}

static uint8_T c19_o_emlrt_marshallIn(SFc19_SS6_EstimationInstanceStruct
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

static void init_dsm_address_info(SFc19_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc19_SS6_EstimationInstanceStruct
  *chartInstance)
{
  chartInstance->c19_aX = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c19_aY = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c19_X_pos = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c19_Y_pos = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c19_r = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    2);
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

void sf_c19_SS6_Estimation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1260886196U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1028144018U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2397586006U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4022516713U);
}

mxArray* sf_c19_SS6_Estimation_get_post_codegen_info(void);
mxArray *sf_c19_SS6_Estimation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("XzykCtTyf6HLYWnnK8qdfB");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

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
    mxArray* mxPostCodegenInfo = sf_c19_SS6_Estimation_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c19_SS6_Estimation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c19_SS6_Estimation_jit_fallback_info(void)
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

mxArray *sf_c19_SS6_Estimation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c19_SS6_Estimation_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c19_SS6_Estimation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x8'type','srcId','name','auxInfo'{{M[1],M[5],T\"X_pos\",},{M[1],M[15],T\"Y_pos\",},{M[4],M[0],T\"Vx\",S'l','i','p'{{M1x2[69 71],M[0],}}},{M[4],M[0],T\"Vy\",S'l','i','p'{{M1x2[72 74],M[0],}}},{M[4],M[0],T\"X\",S'l','i','p'{{M1x2[79 80],M[0],}}},{M[4],M[0],T\"Y\",S'l','i','p'{{M1x2[81 82],M[0],}}},{M[4],M[0],T\"yaw\",S'l','i','p'{{M1x2[75 78],M[0],}}},{M[8],M[0],T\"is_active_c19_SS6_Estimation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 8, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c19_SS6_Estimation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc19_SS6_EstimationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc19_SS6_EstimationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SS6_EstimationMachineNumber_,
           19,
           1,
           1,
           0,
           5,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"aX");
          _SFD_SET_DATA_PROPS(1,1,1,0,"aY");
          _SFD_SET_DATA_PROPS(2,2,0,1,"X_pos");
          _SFD_SET_DATA_PROPS(3,2,0,1,"Y_pos");
          _SFD_SET_DATA_PROPS(4,1,1,0,"r");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,259);
        _SFD_CV_INIT_EML_IF(0,1,0,84,97,-1,150);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_f_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_f_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_f_sf_marshallOut,(MexInFcnForType)
          c19_f_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_f_sf_marshallOut,(MexInFcnForType)
          c19_f_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_f_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c19_aX);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c19_aY);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c19_X_pos);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c19_Y_pos);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c19_r);
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
  return "Dn3ar7WDiyfScTMYDfAQxE";
}

static void sf_opaque_initialize_c19_SS6_Estimation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc19_SS6_EstimationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c19_SS6_Estimation((SFc19_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
  initialize_c19_SS6_Estimation((SFc19_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c19_SS6_Estimation(void *chartInstanceVar)
{
  enable_c19_SS6_Estimation((SFc19_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c19_SS6_Estimation(void *chartInstanceVar)
{
  disable_c19_SS6_Estimation((SFc19_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c19_SS6_Estimation(void *chartInstanceVar)
{
  sf_gateway_c19_SS6_Estimation((SFc19_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c19_SS6_Estimation(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c19_SS6_Estimation((SFc19_SS6_EstimationInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c19_SS6_Estimation(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c19_SS6_Estimation((SFc19_SS6_EstimationInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c19_SS6_Estimation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc19_SS6_EstimationInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SS6_Estimation_optimization_info();
    }

    finalize_c19_SS6_Estimation((SFc19_SS6_EstimationInstanceStruct*)
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
  initSimStructsc19_SS6_Estimation((SFc19_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c19_SS6_Estimation(SimStruct *S)
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
    initialize_params_c19_SS6_Estimation((SFc19_SS6_EstimationInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c19_SS6_Estimation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SS6_Estimation_optimization_info();
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
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,19,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,19,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,19);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2890058429U));
  ssSetChecksum1(S,(3096158517U));
  ssSetChecksum2(S,(692293675U));
  ssSetChecksum3(S,(1577987477U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c19_SS6_Estimation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c19_SS6_Estimation(SimStruct *S)
{
  SFc19_SS6_EstimationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc19_SS6_EstimationInstanceStruct *)utMalloc(sizeof
    (SFc19_SS6_EstimationInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc19_SS6_EstimationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c19_SS6_Estimation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c19_SS6_Estimation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c19_SS6_Estimation;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c19_SS6_Estimation;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c19_SS6_Estimation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c19_SS6_Estimation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c19_SS6_Estimation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c19_SS6_Estimation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c19_SS6_Estimation;
  chartInstance->chartInfo.mdlStart = mdlStart_c19_SS6_Estimation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c19_SS6_Estimation;
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

void c19_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c19_SS6_Estimation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c19_SS6_Estimation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c19_SS6_Estimation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c19_SS6_Estimation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
