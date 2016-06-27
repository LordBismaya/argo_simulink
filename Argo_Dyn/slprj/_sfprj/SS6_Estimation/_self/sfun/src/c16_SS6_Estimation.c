/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SS6_Estimation_sfun.h"
#include "c16_SS6_Estimation.h"
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
static const char * c16_debug_family_names[44] = { "M", "Ms", "Izz", "Ixx",
  "Ixz", "a", "b", "hcg", "e", "Tw", "R", "I_t", "I_e", "A_f", "K_bf", "zi1",
  "z2", "zi3", "zi4", "zi5", "Lbpsi", "Lbp", "C_a", "C_i", "mu", "e_r", "K_rsf",
  "K_rsr", "C1", "C2", "C3", "C_s1", "C_d", "d", "n", "rho", "g", "nargin",
  "nargout", "U_dot", "V", "r", "Fz_L", "Fz_R" };

/* Function Declarations */
static void initialize_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance);
static void initialize_params_c16_SS6_Estimation
  (SFc16_SS6_EstimationInstanceStruct *chartInstance);
static void enable_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance);
static void disable_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance);
static void c16_update_debugger_state_c16_SS6_Estimation
  (SFc16_SS6_EstimationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c16_SS6_Estimation
  (SFc16_SS6_EstimationInstanceStruct *chartInstance);
static void set_sim_state_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct *
  chartInstance, const mxArray *c16_st);
static void finalize_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance);
static void sf_gateway_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance);
static void mdl_start_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance);
static void c16_chartstep_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct *
  chartInstance);
static void initSimStructsc16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c16_machineNumber, uint32_T
  c16_chartNumber, uint32_T c16_instanceNumber);
static const mxArray *c16_sf_marshallOut(void *chartInstanceVoid, void
  *c16_inData);
static real_T c16_emlrt_marshallIn(SFc16_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c16_b_Fz_R, const char_T *c16_identifier);
static real_T c16_b_emlrt_marshallIn(SFc16_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c16_u, const emlrtMsgIdentifier *c16_parentId);
static void c16_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c16_mxArrayInData, const char_T *c16_varName, void *c16_outData);
static void c16_info_helper(const mxArray **c16_info);
static const mxArray *c16_emlrt_marshallOut(const char * c16_u);
static const mxArray *c16_b_emlrt_marshallOut(const uint32_T c16_u);
static const mxArray *c16_b_sf_marshallOut(void *chartInstanceVoid, void
  *c16_inData);
static int32_T c16_c_emlrt_marshallIn(SFc16_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c16_u, const emlrtMsgIdentifier *c16_parentId);
static void c16_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c16_mxArrayInData, const char_T *c16_varName, void *c16_outData);
static uint8_T c16_d_emlrt_marshallIn(SFc16_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c16_b_is_active_c16_SS6_Estimation, const
  char_T *c16_identifier);
static uint8_T c16_e_emlrt_marshallIn(SFc16_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c16_u, const emlrtMsgIdentifier *c16_parentId);
static void init_dsm_address_info(SFc16_SS6_EstimationInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc16_SS6_EstimationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance)
{
  chartInstance->c16_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c16_is_active_c16_SS6_Estimation = 0U;
}

static void initialize_params_c16_SS6_Estimation
  (SFc16_SS6_EstimationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c16_update_debugger_state_c16_SS6_Estimation
  (SFc16_SS6_EstimationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c16_SS6_Estimation
  (SFc16_SS6_EstimationInstanceStruct *chartInstance)
{
  const mxArray *c16_st;
  const mxArray *c16_y = NULL;
  real_T c16_hoistedGlobal;
  real_T c16_u;
  const mxArray *c16_b_y = NULL;
  real_T c16_b_hoistedGlobal;
  real_T c16_b_u;
  const mxArray *c16_c_y = NULL;
  uint8_T c16_c_hoistedGlobal;
  uint8_T c16_c_u;
  const mxArray *c16_d_y = NULL;
  c16_st = NULL;
  c16_st = NULL;
  c16_y = NULL;
  sf_mex_assign(&c16_y, sf_mex_createcellmatrix(3, 1), false);
  c16_hoistedGlobal = *chartInstance->c16_Fz_L;
  c16_u = c16_hoistedGlobal;
  c16_b_y = NULL;
  sf_mex_assign(&c16_b_y, sf_mex_create("y", &c16_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c16_y, 0, c16_b_y);
  c16_b_hoistedGlobal = *chartInstance->c16_Fz_R;
  c16_b_u = c16_b_hoistedGlobal;
  c16_c_y = NULL;
  sf_mex_assign(&c16_c_y, sf_mex_create("y", &c16_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c16_y, 1, c16_c_y);
  c16_c_hoistedGlobal = chartInstance->c16_is_active_c16_SS6_Estimation;
  c16_c_u = c16_c_hoistedGlobal;
  c16_d_y = NULL;
  sf_mex_assign(&c16_d_y, sf_mex_create("y", &c16_c_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c16_y, 2, c16_d_y);
  sf_mex_assign(&c16_st, c16_y, false);
  return c16_st;
}

static void set_sim_state_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct *
  chartInstance, const mxArray *c16_st)
{
  const mxArray *c16_u;
  chartInstance->c16_doneDoubleBufferReInit = true;
  c16_u = sf_mex_dup(c16_st);
  *chartInstance->c16_Fz_L = c16_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c16_u, 0)), "Fz_L");
  *chartInstance->c16_Fz_R = c16_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c16_u, 1)), "Fz_R");
  chartInstance->c16_is_active_c16_SS6_Estimation = c16_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c16_u, 2)),
     "is_active_c16_SS6_Estimation");
  sf_mex_destroy(&c16_u);
  c16_update_debugger_state_c16_SS6_Estimation(chartInstance);
  sf_mex_destroy(&c16_st);
}

static void finalize_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 15U, chartInstance->c16_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c16_U_dot, 0U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c16_V, 1U);
  chartInstance->c16_sfEvent = CALL_EVENT;
  c16_chartstep_c16_SS6_Estimation(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SS6_EstimationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c16_Fz_L, 2U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c16_Fz_R, 3U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c16_r, 4U);
}

static void mdl_start_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c16_chartstep_c16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct *
  chartInstance)
{
  real_T c16_hoistedGlobal;
  real_T c16_b_hoistedGlobal;
  real_T c16_c_hoistedGlobal;
  real_T c16_b_U_dot;
  real_T c16_b_V;
  real_T c16_b_r;
  uint32_T c16_debug_family_var_map[44];
  real_T c16_M;
  real_T c16_Ms;
  real_T c16_Izz;
  real_T c16_Ixx;
  real_T c16_Ixz;
  real_T c16_a;
  real_T c16_b;
  real_T c16_hcg;
  real_T c16_e;
  real_T c16_Tw;
  real_T c16_R;
  real_T c16_I_t;
  real_T c16_I_e;
  real_T c16_A_f;
  real_T c16_K_bf;
  real_T c16_zi1;
  real_T c16_z2;
  real_T c16_zi3;
  real_T c16_zi4;
  real_T c16_zi5;
  real_T c16_Lbpsi;
  real_T c16_Lbp;
  real_T c16_C_a;
  real_T c16_C_i;
  real_T c16_mu;
  real_T c16_e_r;
  real_T c16_K_rsf;
  real_T c16_K_rsr;
  real_T c16_C1;
  real_T c16_C2;
  real_T c16_C3;
  real_T c16_C_s1;
  real_T c16_C_d;
  real_T c16_d;
  real_T c16_n;
  real_T c16_rho;
  real_T c16_g;
  real_T c16_nargin = 3.0;
  real_T c16_nargout = 2.0;
  real_T c16_b_Fz_L;
  real_T c16_b_Fz_R;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 15U, chartInstance->c16_sfEvent);
  c16_hoistedGlobal = *chartInstance->c16_U_dot;
  c16_b_hoistedGlobal = *chartInstance->c16_V;
  c16_c_hoistedGlobal = *chartInstance->c16_r;
  c16_b_U_dot = c16_hoistedGlobal;
  c16_b_V = c16_b_hoistedGlobal;
  c16_b_r = c16_c_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 44U, 44U, c16_debug_family_names,
    c16_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c16_M, 0U, c16_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_Ms, 1U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_Izz, 2U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_Ixx, 3U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_Ixz, 4U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_a, 5U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_b, 6U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_hcg, 7U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_e, 8U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_Tw, 9U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_R, 10U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_I_t, 11U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_I_e, 12U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_A_f, 13U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_K_bf, 14U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_zi1, 15U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_z2, 16U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_zi3, 17U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_zi4, 18U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_zi5, 19U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_Lbpsi, 20U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_Lbp, 21U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_C_a, 22U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_C_i, 23U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_mu, 24U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_e_r, 25U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_K_rsf, 26U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_K_rsr, 27U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_C1, 28U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_C2, 29U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_C3, 30U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_C_s1, 31U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_C_d, 32U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_d, 33U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_n, 34U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_rho, 35U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c16_g, 36U, c16_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_nargin, 37U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_nargout, 38U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c16_b_U_dot, 39U, c16_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c16_b_V, 40U, c16_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c16_b_r, 41U, c16_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_b_Fz_L, 42U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c16_b_Fz_R, 43U, c16_sf_marshallOut,
    c16_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_M = 1280.0;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_Ms = 1160.0;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_Izz = 2500.0;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_Ixx = 750.0;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_Ixz = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_a = 1.203;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_b = 1.217;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_hcg = 0.5;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_e = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_Tw = 1.33;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_R = 0.3;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 2);
  c16_I_t = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 3);
  c16_I_e = 0.136;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 3);
  c16_A_f = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 3);
  c16_K_bf = 0.55;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 3);
  c16_zi1 = 13.56;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 3);
  c16_z2 = 7.5;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 3);
  c16_zi3 = 5.37;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 3);
  c16_zi4 = 4.22;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 3);
  c16_zi5 = 3.28;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 4);
  c16_Lbpsi = 45000.0;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 4);
  c16_Lbp = 2600.0;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 4);
  c16_C_a = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 4);
  c16_C_i = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 4);
  c16_mu = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 4);
  c16_e_r = 0.015;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 4);
  c16_K_rsf = -0.05;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 4);
  c16_K_rsr = 0.1;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 5);
  c16_C1 = -6.0;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 5);
  c16_C2 = 59.16;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 5);
  c16_C3 = 25.0;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 5);
  c16_C_s1 = 1.38;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 5);
  c16_K_rsf = 0.444;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 5);
  c16_C_d = 0.32;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 5);
  c16_d = 0.014;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 5);
  c16_n = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 6);
  c16_rho = 1.204;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 6);
  c16_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 12);
  c16_b_Fz_L = 6278.4000000000005;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, 13);
  c16_b_Fz_R = 6278.4000000000005;
  _SFD_EML_CALL(0U, chartInstance->c16_sfEvent, -13);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c16_Fz_L = c16_b_Fz_L;
  *chartInstance->c16_Fz_R = c16_b_Fz_R;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 15U, chartInstance->c16_sfEvent);
}

static void initSimStructsc16_SS6_Estimation(SFc16_SS6_EstimationInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c16_machineNumber, uint32_T
  c16_chartNumber, uint32_T c16_instanceNumber)
{
  (void)c16_machineNumber;
  (void)c16_chartNumber;
  (void)c16_instanceNumber;
}

static const mxArray *c16_sf_marshallOut(void *chartInstanceVoid, void
  *c16_inData)
{
  const mxArray *c16_mxArrayOutData = NULL;
  real_T c16_u;
  const mxArray *c16_y = NULL;
  SFc16_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc16_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c16_mxArrayOutData = NULL;
  c16_u = *(real_T *)c16_inData;
  c16_y = NULL;
  sf_mex_assign(&c16_y, sf_mex_create("y", &c16_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c16_mxArrayOutData, c16_y, false);
  return c16_mxArrayOutData;
}

static real_T c16_emlrt_marshallIn(SFc16_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c16_b_Fz_R, const char_T *c16_identifier)
{
  real_T c16_y;
  emlrtMsgIdentifier c16_thisId;
  c16_thisId.fIdentifier = c16_identifier;
  c16_thisId.fParent = NULL;
  c16_y = c16_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c16_b_Fz_R),
    &c16_thisId);
  sf_mex_destroy(&c16_b_Fz_R);
  return c16_y;
}

static real_T c16_b_emlrt_marshallIn(SFc16_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c16_u, const emlrtMsgIdentifier *c16_parentId)
{
  real_T c16_y;
  real_T c16_d0;
  (void)chartInstance;
  sf_mex_import(c16_parentId, sf_mex_dup(c16_u), &c16_d0, 1, 0, 0U, 0, 0U, 0);
  c16_y = c16_d0;
  sf_mex_destroy(&c16_u);
  return c16_y;
}

static void c16_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c16_mxArrayInData, const char_T *c16_varName, void *c16_outData)
{
  const mxArray *c16_b_Fz_R;
  const char_T *c16_identifier;
  emlrtMsgIdentifier c16_thisId;
  real_T c16_y;
  SFc16_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc16_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c16_b_Fz_R = sf_mex_dup(c16_mxArrayInData);
  c16_identifier = c16_varName;
  c16_thisId.fIdentifier = c16_identifier;
  c16_thisId.fParent = NULL;
  c16_y = c16_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c16_b_Fz_R),
    &c16_thisId);
  sf_mex_destroy(&c16_b_Fz_R);
  *(real_T *)c16_outData = c16_y;
  sf_mex_destroy(&c16_mxArrayInData);
}

const mxArray *sf_c16_SS6_Estimation_get_eml_resolved_functions_info(void)
{
  const mxArray *c16_nameCaptureInfo = NULL;
  c16_nameCaptureInfo = NULL;
  sf_mex_assign(&c16_nameCaptureInfo, sf_mex_createstruct("structure", 2, 7, 1),
                false);
  c16_info_helper(&c16_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c16_nameCaptureInfo);
  return c16_nameCaptureInfo;
}

static void c16_info_helper(const mxArray **c16_info)
{
  const mxArray *c16_rhs0 = NULL;
  const mxArray *c16_lhs0 = NULL;
  const mxArray *c16_rhs1 = NULL;
  const mxArray *c16_lhs1 = NULL;
  const mxArray *c16_rhs2 = NULL;
  const mxArray *c16_lhs2 = NULL;
  const mxArray *c16_rhs3 = NULL;
  const mxArray *c16_lhs3 = NULL;
  const mxArray *c16_rhs4 = NULL;
  const mxArray *c16_lhs4 = NULL;
  const mxArray *c16_rhs5 = NULL;
  const mxArray *c16_lhs5 = NULL;
  const mxArray *c16_rhs6 = NULL;
  const mxArray *c16_lhs6 = NULL;
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("mrdivide"), "name", "name",
                  0);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c16_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c16_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_rhs0), "rhs", "rhs",
                  0);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_lhs0), "lhs", "lhs",
                  0);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 1);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(1389739374U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c16_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c16_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_rhs1), "rhs", "rhs",
                  1);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_lhs1), "lhs", "lhs",
                  1);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c16_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c16_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_rhs2), "rhs", "rhs",
                  2);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_lhs2), "lhs", "lhs",
                  2);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c16_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c16_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_rhs3), "rhs", "rhs",
                  3);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_lhs3), "lhs", "lhs",
                  3);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c16_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c16_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_rhs4), "rhs", "rhs",
                  4);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_lhs4), "lhs", "lhs",
                  4);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c16_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c16_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_rhs5), "rhs", "rhs",
                  5);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_lhs5), "lhs", "lhs",
                  5);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c16_info, c16_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c16_info, c16_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c16_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c16_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_rhs6), "rhs", "rhs",
                  6);
  sf_mex_addfield(*c16_info, sf_mex_duplicatearraysafe(&c16_lhs6), "lhs", "lhs",
                  6);
  sf_mex_destroy(&c16_rhs0);
  sf_mex_destroy(&c16_lhs0);
  sf_mex_destroy(&c16_rhs1);
  sf_mex_destroy(&c16_lhs1);
  sf_mex_destroy(&c16_rhs2);
  sf_mex_destroy(&c16_lhs2);
  sf_mex_destroy(&c16_rhs3);
  sf_mex_destroy(&c16_lhs3);
  sf_mex_destroy(&c16_rhs4);
  sf_mex_destroy(&c16_lhs4);
  sf_mex_destroy(&c16_rhs5);
  sf_mex_destroy(&c16_lhs5);
  sf_mex_destroy(&c16_rhs6);
  sf_mex_destroy(&c16_lhs6);
}

static const mxArray *c16_emlrt_marshallOut(const char * c16_u)
{
  const mxArray *c16_y = NULL;
  c16_y = NULL;
  sf_mex_assign(&c16_y, sf_mex_create("y", c16_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c16_u)), false);
  return c16_y;
}

static const mxArray *c16_b_emlrt_marshallOut(const uint32_T c16_u)
{
  const mxArray *c16_y = NULL;
  c16_y = NULL;
  sf_mex_assign(&c16_y, sf_mex_create("y", &c16_u, 7, 0U, 0U, 0U, 0), false);
  return c16_y;
}

static const mxArray *c16_b_sf_marshallOut(void *chartInstanceVoid, void
  *c16_inData)
{
  const mxArray *c16_mxArrayOutData = NULL;
  int32_T c16_u;
  const mxArray *c16_y = NULL;
  SFc16_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc16_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c16_mxArrayOutData = NULL;
  c16_u = *(int32_T *)c16_inData;
  c16_y = NULL;
  sf_mex_assign(&c16_y, sf_mex_create("y", &c16_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c16_mxArrayOutData, c16_y, false);
  return c16_mxArrayOutData;
}

static int32_T c16_c_emlrt_marshallIn(SFc16_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c16_u, const emlrtMsgIdentifier *c16_parentId)
{
  int32_T c16_y;
  int32_T c16_i0;
  (void)chartInstance;
  sf_mex_import(c16_parentId, sf_mex_dup(c16_u), &c16_i0, 1, 6, 0U, 0, 0U, 0);
  c16_y = c16_i0;
  sf_mex_destroy(&c16_u);
  return c16_y;
}

static void c16_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c16_mxArrayInData, const char_T *c16_varName, void *c16_outData)
{
  const mxArray *c16_b_sfEvent;
  const char_T *c16_identifier;
  emlrtMsgIdentifier c16_thisId;
  int32_T c16_y;
  SFc16_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc16_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c16_b_sfEvent = sf_mex_dup(c16_mxArrayInData);
  c16_identifier = c16_varName;
  c16_thisId.fIdentifier = c16_identifier;
  c16_thisId.fParent = NULL;
  c16_y = c16_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c16_b_sfEvent),
    &c16_thisId);
  sf_mex_destroy(&c16_b_sfEvent);
  *(int32_T *)c16_outData = c16_y;
  sf_mex_destroy(&c16_mxArrayInData);
}

static uint8_T c16_d_emlrt_marshallIn(SFc16_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c16_b_is_active_c16_SS6_Estimation, const
  char_T *c16_identifier)
{
  uint8_T c16_y;
  emlrtMsgIdentifier c16_thisId;
  c16_thisId.fIdentifier = c16_identifier;
  c16_thisId.fParent = NULL;
  c16_y = c16_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c16_b_is_active_c16_SS6_Estimation), &c16_thisId);
  sf_mex_destroy(&c16_b_is_active_c16_SS6_Estimation);
  return c16_y;
}

static uint8_T c16_e_emlrt_marshallIn(SFc16_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c16_u, const emlrtMsgIdentifier *c16_parentId)
{
  uint8_T c16_y;
  uint8_T c16_u0;
  (void)chartInstance;
  sf_mex_import(c16_parentId, sf_mex_dup(c16_u), &c16_u0, 1, 3, 0U, 0, 0U, 0);
  c16_y = c16_u0;
  sf_mex_destroy(&c16_u);
  return c16_y;
}

static void init_dsm_address_info(SFc16_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc16_SS6_EstimationInstanceStruct
  *chartInstance)
{
  chartInstance->c16_U_dot = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c16_V = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c16_Fz_L = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c16_Fz_R = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c16_r = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
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

void sf_c16_SS6_Estimation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1889486633U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1525034767U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3515802662U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2429970916U);
}

mxArray* sf_c16_SS6_Estimation_get_post_codegen_info(void);
mxArray *sf_c16_SS6_Estimation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("o2G06cpO3M4BRVz1MzVKGD");
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
    mxArray* mxPostCodegenInfo = sf_c16_SS6_Estimation_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c16_SS6_Estimation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c16_SS6_Estimation_jit_fallback_info(void)
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

mxArray *sf_c16_SS6_Estimation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c16_SS6_Estimation_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c16_SS6_Estimation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[5],T\"Fz_L\",},{M[1],M[15],T\"Fz_R\",},{M[8],M[0],T\"is_active_c16_SS6_Estimation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c16_SS6_Estimation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc16_SS6_EstimationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc16_SS6_EstimationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SS6_EstimationMachineNumber_,
           16,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"U_dot");
          _SFD_SET_DATA_PROPS(1,1,1,0,"V");
          _SFD_SET_DATA_PROPS(2,2,0,1,"Fz_L");
          _SFD_SET_DATA_PROPS(3,2,0,1,"Fz_R");
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
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,535);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c16_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c16_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c16_sf_marshallOut,(MexInFcnForType)c16_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c16_sf_marshallOut,(MexInFcnForType)c16_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c16_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c16_U_dot);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c16_V);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c16_Fz_L);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c16_Fz_R);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c16_r);
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
  return "nrpenqkF8ejDanyXb4eC";
}

static void sf_opaque_initialize_c16_SS6_Estimation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc16_SS6_EstimationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c16_SS6_Estimation((SFc16_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
  initialize_c16_SS6_Estimation((SFc16_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c16_SS6_Estimation(void *chartInstanceVar)
{
  enable_c16_SS6_Estimation((SFc16_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c16_SS6_Estimation(void *chartInstanceVar)
{
  disable_c16_SS6_Estimation((SFc16_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c16_SS6_Estimation(void *chartInstanceVar)
{
  sf_gateway_c16_SS6_Estimation((SFc16_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c16_SS6_Estimation(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c16_SS6_Estimation((SFc16_SS6_EstimationInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c16_SS6_Estimation(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c16_SS6_Estimation((SFc16_SS6_EstimationInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c16_SS6_Estimation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc16_SS6_EstimationInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SS6_Estimation_optimization_info();
    }

    finalize_c16_SS6_Estimation((SFc16_SS6_EstimationInstanceStruct*)
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
  initSimStructsc16_SS6_Estimation((SFc16_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c16_SS6_Estimation(SimStruct *S)
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
    initialize_params_c16_SS6_Estimation((SFc16_SS6_EstimationInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c16_SS6_Estimation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SS6_Estimation_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      16);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,16,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,16,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,16);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,16,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,16,2);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,16);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1349082448U));
  ssSetChecksum1(S,(3185962350U));
  ssSetChecksum2(S,(1553039241U));
  ssSetChecksum3(S,(21550388U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c16_SS6_Estimation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c16_SS6_Estimation(SimStruct *S)
{
  SFc16_SS6_EstimationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc16_SS6_EstimationInstanceStruct *)utMalloc(sizeof
    (SFc16_SS6_EstimationInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc16_SS6_EstimationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c16_SS6_Estimation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c16_SS6_Estimation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c16_SS6_Estimation;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c16_SS6_Estimation;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c16_SS6_Estimation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c16_SS6_Estimation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c16_SS6_Estimation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c16_SS6_Estimation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c16_SS6_Estimation;
  chartInstance->chartInfo.mdlStart = mdlStart_c16_SS6_Estimation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c16_SS6_Estimation;
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

void c16_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c16_SS6_Estimation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c16_SS6_Estimation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c16_SS6_Estimation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c16_SS6_Estimation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
