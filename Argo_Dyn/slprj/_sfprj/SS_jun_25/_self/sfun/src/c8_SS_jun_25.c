/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SS_jun_25_sfun.h"
#include "c8_SS_jun_25.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SS_jun_25_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c8_debug_family_names[46] = { "M", "Ms", "Izz", "Ixx", "Ixz",
  "a", "b", "hcg", "e", "Tw", "R", "I_t", "I_e", "A_f", "K_bf", "zi1", "z2",
  "zi3", "zi4", "zi5", "Lbpsi", "Lbp", "C_a", "C_i", "mu", "e_r", "K_rsf",
  "K_rsr", "C1", "C2", "C3", "C_s1", "C_d", "d", "n", "rho", "g", "c", "nargin",
  "nargout", "U", "V", "theta", "X_dot", "Y_dot", "V_SS" };

/* Function Declarations */
static void initialize_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance);
static void initialize_params_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance);
static void enable_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance);
static void disable_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance);
static void c8_update_debugger_state_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance);
static void set_sim_state_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance, const mxArray *c8_st);
static void finalize_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance);
static void sf_gateway_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance);
static void mdl_start_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance);
static void c8_chartstep_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance);
static void initSimStructsc8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber, uint32_T c8_instanceNumber);
static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData);
static real_T c8_emlrt_marshallIn(SFc8_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c8_b_V_SS, const char_T *c8_identifier);
static real_T c8_b_emlrt_marshallIn(SFc8_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static void c8_info_helper(const mxArray **c8_info);
static const mxArray *c8_emlrt_marshallOut(const char * c8_u);
static const mxArray *c8_b_emlrt_marshallOut(const uint32_T c8_u);
static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static int32_T c8_c_emlrt_marshallIn(SFc8_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static uint8_T c8_d_emlrt_marshallIn(SFc8_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c8_b_is_active_c8_SS_jun_25, const char_T *c8_identifier);
static uint8_T c8_e_emlrt_marshallIn(SFc8_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void init_dsm_address_info(SFc8_SS_jun_25InstanceStruct *chartInstance);
static void init_simulink_io_address(SFc8_SS_jun_25InstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance)
{
  chartInstance->c8_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c8_is_active_c8_SS_jun_25 = 0U;
}

static void initialize_params_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c8_update_debugger_state_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance)
{
  const mxArray *c8_st;
  const mxArray *c8_y = NULL;
  real_T c8_hoistedGlobal;
  real_T c8_u;
  const mxArray *c8_b_y = NULL;
  real_T c8_b_hoistedGlobal;
  real_T c8_b_u;
  const mxArray *c8_c_y = NULL;
  real_T c8_c_hoistedGlobal;
  real_T c8_c_u;
  const mxArray *c8_d_y = NULL;
  uint8_T c8_d_hoistedGlobal;
  uint8_T c8_d_u;
  const mxArray *c8_e_y = NULL;
  c8_st = NULL;
  c8_st = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_createcellmatrix(4, 1), false);
  c8_hoistedGlobal = *chartInstance->c8_V_SS;
  c8_u = c8_hoistedGlobal;
  c8_b_y = NULL;
  sf_mex_assign(&c8_b_y, sf_mex_create("y", &c8_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c8_y, 0, c8_b_y);
  c8_b_hoistedGlobal = *chartInstance->c8_X_dot;
  c8_b_u = c8_b_hoistedGlobal;
  c8_c_y = NULL;
  sf_mex_assign(&c8_c_y, sf_mex_create("y", &c8_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c8_y, 1, c8_c_y);
  c8_c_hoistedGlobal = *chartInstance->c8_Y_dot;
  c8_c_u = c8_c_hoistedGlobal;
  c8_d_y = NULL;
  sf_mex_assign(&c8_d_y, sf_mex_create("y", &c8_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c8_y, 2, c8_d_y);
  c8_d_hoistedGlobal = chartInstance->c8_is_active_c8_SS_jun_25;
  c8_d_u = c8_d_hoistedGlobal;
  c8_e_y = NULL;
  sf_mex_assign(&c8_e_y, sf_mex_create("y", &c8_d_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c8_y, 3, c8_e_y);
  sf_mex_assign(&c8_st, c8_y, false);
  return c8_st;
}

static void set_sim_state_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance, const mxArray *c8_st)
{
  const mxArray *c8_u;
  chartInstance->c8_doneDoubleBufferReInit = true;
  c8_u = sf_mex_dup(c8_st);
  *chartInstance->c8_V_SS = c8_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c8_u, 0)), "V_SS");
  *chartInstance->c8_X_dot = c8_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c8_u, 1)), "X_dot");
  *chartInstance->c8_Y_dot = c8_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c8_u, 2)), "Y_dot");
  chartInstance->c8_is_active_c8_SS_jun_25 = c8_d_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c8_u, 3)), "is_active_c8_SS_jun_25");
  sf_mex_destroy(&c8_u);
  c8_update_debugger_state_c8_SS_jun_25(chartInstance);
  sf_mex_destroy(&c8_st);
}

static void finalize_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance)
{
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 6U, chartInstance->c8_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c8_U, 0U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c8_V, 1U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c8_theta, 2U);
  chartInstance->c8_sfEvent = CALL_EVENT;
  c8_chartstep_c8_SS_jun_25(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SS_jun_25MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c8_X_dot, 3U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c8_Y_dot, 4U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c8_V_SS, 5U);
}

static void mdl_start_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c8_chartstep_c8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance)
{
  real_T c8_hoistedGlobal;
  real_T c8_b_hoistedGlobal;
  real_T c8_c_hoistedGlobal;
  real_T c8_b_U;
  real_T c8_b_V;
  real_T c8_b_theta;
  uint32_T c8_debug_family_var_map[46];
  real_T c8_M;
  real_T c8_Ms;
  real_T c8_Izz;
  real_T c8_Ixx;
  real_T c8_Ixz;
  real_T c8_a;
  real_T c8_b;
  real_T c8_hcg;
  real_T c8_e;
  real_T c8_Tw;
  real_T c8_R;
  real_T c8_I_t;
  real_T c8_I_e;
  real_T c8_A_f;
  real_T c8_K_bf;
  real_T c8_zi1;
  real_T c8_z2;
  real_T c8_zi3;
  real_T c8_zi4;
  real_T c8_zi5;
  real_T c8_Lbpsi;
  real_T c8_Lbp;
  real_T c8_C_a;
  real_T c8_C_i;
  real_T c8_mu;
  real_T c8_e_r;
  real_T c8_K_rsf;
  real_T c8_K_rsr;
  real_T c8_C1;
  real_T c8_C2;
  real_T c8_C3;
  real_T c8_C_s1;
  real_T c8_C_d;
  real_T c8_d;
  real_T c8_n;
  real_T c8_rho;
  real_T c8_g;
  real_T c8_c;
  real_T c8_nargin = 3.0;
  real_T c8_nargout = 3.0;
  real_T c8_b_X_dot;
  real_T c8_b_Y_dot;
  real_T c8_b_V_SS;
  real_T c8_x;
  real_T c8_b_x;
  real_T c8_c_x;
  real_T c8_d_x;
  real_T c8_e_x;
  real_T c8_f_x;
  real_T c8_g_x;
  real_T c8_h_x;
  real_T c8_A;
  real_T c8_B;
  real_T c8_i_x;
  real_T c8_y;
  real_T c8_j_x;
  real_T c8_b_y;
  real_T c8_k_x;
  real_T c8_c_y;
  real_T c8_d_y;
  real_T c8_l_x;
  real_T c8_m_x;
  real_T c8_n_x;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 6U, chartInstance->c8_sfEvent);
  c8_hoistedGlobal = *chartInstance->c8_U;
  c8_b_hoistedGlobal = *chartInstance->c8_V;
  c8_c_hoistedGlobal = *chartInstance->c8_theta;
  c8_b_U = c8_hoistedGlobal;
  c8_b_V = c8_b_hoistedGlobal;
  c8_b_theta = c8_c_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 46U, 46U, c8_debug_family_names,
    c8_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_M, 0U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_Ms, 1U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_Izz, 2U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_Ixx, 3U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_Ixz, 4U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_a, 5U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_b, 6U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_hcg, 7U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_e, 8U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_Tw, 9U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_R, 10U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_I_t, 11U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_I_e, 12U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_A_f, 13U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_K_bf, 14U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_zi1, 15U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_z2, 16U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_zi3, 17U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_zi4, 18U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_zi5, 19U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_Lbpsi, 20U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_Lbp, 21U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_C_a, 22U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_C_i, 23U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_mu, 24U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_e_r, 25U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_K_rsf, 26U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_K_rsr, 27U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_C1, 28U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_C2, 29U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_C3, 30U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_C_s1, 31U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_C_d, 32U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_d, 33U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_n, 34U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_rho, 35U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_g, 36U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_c, 37U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_nargin, 38U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_nargout, 39U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c8_b_U, 40U, c8_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c8_b_V, 41U, c8_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c8_b_theta, 42U, c8_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_b_X_dot, 43U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_b_Y_dot, 44U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_b_V_SS, 45U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_M = 1280.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_Ms = 1160.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_Izz = 2500.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_Ixx = 750.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_Ixz = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_a = 1.203;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_b = 1.217;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_hcg = 0.5;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_e = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_Tw = 1.33;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_R = 0.3;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 2);
  c8_I_t = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 3);
  c8_I_e = 0.136;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 3);
  c8_A_f = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 3);
  c8_K_bf = 0.55;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 3);
  c8_zi1 = 13.56;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 3);
  c8_z2 = 7.5;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 3);
  c8_zi3 = 5.37;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 3);
  c8_zi4 = 4.22;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 3);
  c8_zi5 = 3.28;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
  c8_Lbpsi = 45000.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
  c8_Lbp = 2600.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
  c8_C_a = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
  c8_C_i = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
  c8_mu = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
  c8_e_r = 0.015;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
  c8_K_rsf = -0.05;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
  c8_K_rsr = 0.1;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 5);
  c8_C1 = -6.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 5);
  c8_C2 = 59.16;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 5);
  c8_C3 = 25.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 5);
  c8_C_s1 = 1.38;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 5);
  c8_K_rsf = 0.444;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 5);
  c8_C_d = 0.32;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 5);
  c8_d = 0.014;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 5);
  c8_n = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 6);
  c8_rho = 1.204;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 6);
  c8_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 6);
  c8_c = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 10);
  c8_x = c8_b_theta;
  c8_b_x = c8_x;
  c8_b_x = muDoubleScalarSin(c8_b_x);
  c8_c_x = c8_b_theta;
  c8_d_x = c8_c_x;
  c8_d_x = muDoubleScalarCos(c8_d_x);
  c8_b_X_dot = c8_b_U * c8_b_x + c8_b_V * c8_d_x;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 11);
  c8_e_x = c8_b_theta;
  c8_f_x = c8_e_x;
  c8_f_x = muDoubleScalarCos(c8_f_x);
  c8_g_x = c8_b_theta;
  c8_h_x = c8_g_x;
  c8_h_x = muDoubleScalarSin(c8_h_x);
  c8_b_Y_dot = c8_b_U * c8_f_x - c8_b_V * c8_h_x;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 12);
  c8_A = c8_b_V;
  c8_B = c8_b_U;
  c8_i_x = c8_A;
  c8_y = c8_B;
  c8_j_x = c8_i_x;
  c8_b_y = c8_y;
  c8_k_x = c8_j_x;
  c8_c_y = c8_b_y;
  c8_d_y = c8_k_x / c8_c_y;
  c8_l_x = c8_d_y;
  c8_b_V_SS = c8_l_x;
  c8_m_x = c8_b_V_SS;
  c8_n_x = c8_m_x;
  c8_n_x = muDoubleScalarAtan(c8_n_x);
  c8_b_V_SS = 57.295779513082323 * c8_n_x;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, -12);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c8_X_dot = c8_b_X_dot;
  *chartInstance->c8_Y_dot = c8_b_Y_dot;
  *chartInstance->c8_V_SS = c8_b_V_SS;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 6U, chartInstance->c8_sfEvent);
}

static void initSimStructsc8_SS_jun_25(SFc8_SS_jun_25InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber, uint32_T c8_instanceNumber)
{
  (void)c8_machineNumber;
  (void)c8_chartNumber;
  (void)c8_instanceNumber;
}

static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  real_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc8_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(real_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static real_T c8_emlrt_marshallIn(SFc8_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c8_b_V_SS, const char_T *c8_identifier)
{
  real_T c8_y;
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_V_SS), &c8_thisId);
  sf_mex_destroy(&c8_b_V_SS);
  return c8_y;
}

static real_T c8_b_emlrt_marshallIn(SFc8_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  real_T c8_y;
  real_T c8_d0;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_d0, 1, 0, 0U, 0, 0U, 0);
  c8_y = c8_d0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_V_SS;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y;
  SFc8_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc8_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c8_b_V_SS = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_V_SS), &c8_thisId);
  sf_mex_destroy(&c8_b_V_SS);
  *(real_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

const mxArray *sf_c8_SS_jun_25_get_eml_resolved_functions_info(void)
{
  const mxArray *c8_nameCaptureInfo = NULL;
  c8_nameCaptureInfo = NULL;
  sf_mex_assign(&c8_nameCaptureInfo, sf_mex_createstruct("structure", 2, 14, 1),
                false);
  c8_info_helper(&c8_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c8_nameCaptureInfo);
  return c8_nameCaptureInfo;
}

static void c8_info_helper(const mxArray **c8_info)
{
  const mxArray *c8_rhs0 = NULL;
  const mxArray *c8_lhs0 = NULL;
  const mxArray *c8_rhs1 = NULL;
  const mxArray *c8_lhs1 = NULL;
  const mxArray *c8_rhs2 = NULL;
  const mxArray *c8_lhs2 = NULL;
  const mxArray *c8_rhs3 = NULL;
  const mxArray *c8_lhs3 = NULL;
  const mxArray *c8_rhs4 = NULL;
  const mxArray *c8_lhs4 = NULL;
  const mxArray *c8_rhs5 = NULL;
  const mxArray *c8_lhs5 = NULL;
  const mxArray *c8_rhs6 = NULL;
  const mxArray *c8_lhs6 = NULL;
  const mxArray *c8_rhs7 = NULL;
  const mxArray *c8_lhs7 = NULL;
  const mxArray *c8_rhs8 = NULL;
  const mxArray *c8_lhs8 = NULL;
  const mxArray *c8_rhs9 = NULL;
  const mxArray *c8_lhs9 = NULL;
  const mxArray *c8_rhs10 = NULL;
  const mxArray *c8_lhs10 = NULL;
  const mxArray *c8_rhs11 = NULL;
  const mxArray *c8_lhs11 = NULL;
  const mxArray *c8_rhs12 = NULL;
  const mxArray *c8_lhs12 = NULL;
  const mxArray *c8_rhs13 = NULL;
  const mxArray *c8_lhs13 = NULL;
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("sin"), "name", "name", 0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1395346504U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c8_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286840336U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c8_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "context", "context", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("cos"), "name", "name", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1395346496U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c8_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286840322U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c8_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "context", "context", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("mrdivide"), "name", "name", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c8_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1389739374U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c8_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("rdivide"), "name", "name", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c8_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c8_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c8_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_div"), "name", "name", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c8_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c8_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "context", "context", 11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("atand"), "name", "name", 11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atand.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1395346496U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c8_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atand.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("mrdivide"), "name", "name", 12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c8_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atand.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_atan"), "name",
                  "name", 13);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286840318U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c8_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs13), "lhs", "lhs",
                  13);
  sf_mex_destroy(&c8_rhs0);
  sf_mex_destroy(&c8_lhs0);
  sf_mex_destroy(&c8_rhs1);
  sf_mex_destroy(&c8_lhs1);
  sf_mex_destroy(&c8_rhs2);
  sf_mex_destroy(&c8_lhs2);
  sf_mex_destroy(&c8_rhs3);
  sf_mex_destroy(&c8_lhs3);
  sf_mex_destroy(&c8_rhs4);
  sf_mex_destroy(&c8_lhs4);
  sf_mex_destroy(&c8_rhs5);
  sf_mex_destroy(&c8_lhs5);
  sf_mex_destroy(&c8_rhs6);
  sf_mex_destroy(&c8_lhs6);
  sf_mex_destroy(&c8_rhs7);
  sf_mex_destroy(&c8_lhs7);
  sf_mex_destroy(&c8_rhs8);
  sf_mex_destroy(&c8_lhs8);
  sf_mex_destroy(&c8_rhs9);
  sf_mex_destroy(&c8_lhs9);
  sf_mex_destroy(&c8_rhs10);
  sf_mex_destroy(&c8_lhs10);
  sf_mex_destroy(&c8_rhs11);
  sf_mex_destroy(&c8_lhs11);
  sf_mex_destroy(&c8_rhs12);
  sf_mex_destroy(&c8_lhs12);
  sf_mex_destroy(&c8_rhs13);
  sf_mex_destroy(&c8_lhs13);
}

static const mxArray *c8_emlrt_marshallOut(const char * c8_u)
{
  const mxArray *c8_y = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c8_u)), false);
  return c8_y;
}

static const mxArray *c8_b_emlrt_marshallOut(const uint32_T c8_u)
{
  const mxArray *c8_y = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 7, 0U, 0U, 0U, 0), false);
  return c8_y;
}

static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc8_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(int32_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static int32_T c8_c_emlrt_marshallIn(SFc8_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  int32_T c8_y;
  int32_T c8_i0;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_i0, 1, 6, 0U, 0, 0U, 0);
  c8_y = c8_i0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_sfEvent;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  int32_T c8_y;
  SFc8_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc8_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c8_b_sfEvent = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_sfEvent),
    &c8_thisId);
  sf_mex_destroy(&c8_b_sfEvent);
  *(int32_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static uint8_T c8_d_emlrt_marshallIn(SFc8_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c8_b_is_active_c8_SS_jun_25, const char_T *c8_identifier)
{
  uint8_T c8_y;
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_b_is_active_c8_SS_jun_25), &c8_thisId);
  sf_mex_destroy(&c8_b_is_active_c8_SS_jun_25);
  return c8_y;
}

static uint8_T c8_e_emlrt_marshallIn(SFc8_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  uint8_T c8_y;
  uint8_T c8_u0;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_u0, 1, 3, 0U, 0, 0U, 0);
  c8_y = c8_u0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void init_dsm_address_info(SFc8_SS_jun_25InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc8_SS_jun_25InstanceStruct *chartInstance)
{
  chartInstance->c8_U = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    0);
  chartInstance->c8_V = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c8_theta = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c8_X_dot = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c8_Y_dot = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c8_V_SS = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
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

void sf_c8_SS_jun_25_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2008879070U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1201820321U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3823485881U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2540865568U);
}

mxArray* sf_c8_SS_jun_25_get_post_codegen_info(void);
mxArray *sf_c8_SS_jun_25_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("OofZL9Xt0zCWHZsG5xOjlG");
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c8_SS_jun_25_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c8_SS_jun_25_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c8_SS_jun_25_jit_fallback_info(void)
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

mxArray *sf_c8_SS_jun_25_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c8_SS_jun_25_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c8_SS_jun_25(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[18],T\"V_SS\",},{M[1],M[5],T\"X_dot\",},{M[1],M[15],T\"Y_dot\",},{M[8],M[0],T\"is_active_c8_SS_jun_25\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c8_SS_jun_25_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc8_SS_jun_25InstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc8_SS_jun_25InstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SS_jun_25MachineNumber_,
           8,
           1,
           1,
           0,
           6,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_SS_jun_25MachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_SS_jun_25MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _SS_jun_25MachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"U");
          _SFD_SET_DATA_PROPS(1,1,1,0,"V");
          _SFD_SET_DATA_PROPS(2,1,1,0,"theta");
          _SFD_SET_DATA_PROPS(3,2,0,1,"X_dot");
          _SFD_SET_DATA_PROPS(4,2,0,1,"Y_dot");
          _SFD_SET_DATA_PROPS(5,2,0,1,"V_SS");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,540);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)c8_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)c8_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)c8_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c8_U);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c8_V);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c8_theta);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c8_X_dot);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c8_Y_dot);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c8_V_SS);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _SS_jun_25MachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "jm9u2ghEeewM2QOxSHRW3C";
}

static void sf_opaque_initialize_c8_SS_jun_25(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc8_SS_jun_25InstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c8_SS_jun_25((SFc8_SS_jun_25InstanceStruct*)
    chartInstanceVar);
  initialize_c8_SS_jun_25((SFc8_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c8_SS_jun_25(void *chartInstanceVar)
{
  enable_c8_SS_jun_25((SFc8_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c8_SS_jun_25(void *chartInstanceVar)
{
  disable_c8_SS_jun_25((SFc8_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c8_SS_jun_25(void *chartInstanceVar)
{
  sf_gateway_c8_SS_jun_25((SFc8_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c8_SS_jun_25(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c8_SS_jun_25((SFc8_SS_jun_25InstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c8_SS_jun_25(SimStruct* S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c8_SS_jun_25((SFc8_SS_jun_25InstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c8_SS_jun_25(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc8_SS_jun_25InstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SS_jun_25_optimization_info();
    }

    finalize_c8_SS_jun_25((SFc8_SS_jun_25InstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc8_SS_jun_25((SFc8_SS_jun_25InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c8_SS_jun_25(SimStruct *S)
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
    initialize_params_c8_SS_jun_25((SFc8_SS_jun_25InstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c8_SS_jun_25(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SS_jun_25_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,8);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,8,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,8,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,8);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,8,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,8,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,8);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3021289379U));
  ssSetChecksum1(S,(4007944888U));
  ssSetChecksum2(S,(2632985017U));
  ssSetChecksum3(S,(2059042772U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c8_SS_jun_25(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c8_SS_jun_25(SimStruct *S)
{
  SFc8_SS_jun_25InstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc8_SS_jun_25InstanceStruct *)utMalloc(sizeof
    (SFc8_SS_jun_25InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc8_SS_jun_25InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c8_SS_jun_25;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c8_SS_jun_25;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c8_SS_jun_25;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c8_SS_jun_25;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c8_SS_jun_25;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c8_SS_jun_25;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c8_SS_jun_25;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c8_SS_jun_25;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c8_SS_jun_25;
  chartInstance->chartInfo.mdlStart = mdlStart_c8_SS_jun_25;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c8_SS_jun_25;
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

void c8_SS_jun_25_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c8_SS_jun_25(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c8_SS_jun_25(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c8_SS_jun_25(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c8_SS_jun_25_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
