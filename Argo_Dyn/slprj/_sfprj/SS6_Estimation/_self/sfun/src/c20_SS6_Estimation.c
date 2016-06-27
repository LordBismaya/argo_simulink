/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SS6_Estimation_sfun.h"
#include "c20_SS6_Estimation.h"
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
static const char * c20_debug_family_names[46] = { "M", "Ms", "Izz", "Ixx",
  "Ixz", "a", "b", "hcg", "e", "Tw", "R", "I_t", "I_e", "A_f", "K_bf", "zi1",
  "z2", "zi3", "zi4", "zi5", "Lbpsi", "Lbp", "C_a", "C_i", "mu", "e_r", "K_rsf",
  "K_rsr", "C1", "C2", "C3", "C_s1", "C_d", "d", "n", "rho", "g", "c", "nargin",
  "nargout", "U", "V", "theta", "X_dot", "Y_dot", "V_SS" };

/* Function Declarations */
static void initialize_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance);
static void initialize_params_c20_SS6_Estimation
  (SFc20_SS6_EstimationInstanceStruct *chartInstance);
static void enable_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance);
static void disable_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance);
static void c20_update_debugger_state_c20_SS6_Estimation
  (SFc20_SS6_EstimationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c20_SS6_Estimation
  (SFc20_SS6_EstimationInstanceStruct *chartInstance);
static void set_sim_state_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct *
  chartInstance, const mxArray *c20_st);
static void finalize_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance);
static void sf_gateway_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance);
static void mdl_start_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance);
static void c20_chartstep_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct *
  chartInstance);
static void initSimStructsc20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c20_machineNumber, uint32_T
  c20_chartNumber, uint32_T c20_instanceNumber);
static const mxArray *c20_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static real_T c20_emlrt_marshallIn(SFc20_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c20_b_V_SS, const char_T *c20_identifier);
static real_T c20_b_emlrt_marshallIn(SFc20_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId);
static void c20_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static void c20_info_helper(const mxArray **c20_info);
static const mxArray *c20_emlrt_marshallOut(const char * c20_u);
static const mxArray *c20_b_emlrt_marshallOut(const uint32_T c20_u);
static const mxArray *c20_b_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static int32_T c20_c_emlrt_marshallIn(SFc20_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId);
static void c20_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static uint8_T c20_d_emlrt_marshallIn(SFc20_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c20_b_is_active_c20_SS6_Estimation, const
  char_T *c20_identifier);
static uint8_T c20_e_emlrt_marshallIn(SFc20_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId);
static void init_dsm_address_info(SFc20_SS6_EstimationInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc20_SS6_EstimationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance)
{
  chartInstance->c20_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c20_is_active_c20_SS6_Estimation = 0U;
}

static void initialize_params_c20_SS6_Estimation
  (SFc20_SS6_EstimationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c20_update_debugger_state_c20_SS6_Estimation
  (SFc20_SS6_EstimationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c20_SS6_Estimation
  (SFc20_SS6_EstimationInstanceStruct *chartInstance)
{
  const mxArray *c20_st;
  const mxArray *c20_y = NULL;
  real_T c20_hoistedGlobal;
  real_T c20_u;
  const mxArray *c20_b_y = NULL;
  real_T c20_b_hoistedGlobal;
  real_T c20_b_u;
  const mxArray *c20_c_y = NULL;
  real_T c20_c_hoistedGlobal;
  real_T c20_c_u;
  const mxArray *c20_d_y = NULL;
  uint8_T c20_d_hoistedGlobal;
  uint8_T c20_d_u;
  const mxArray *c20_e_y = NULL;
  c20_st = NULL;
  c20_st = NULL;
  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_createcellmatrix(4, 1), false);
  c20_hoistedGlobal = *chartInstance->c20_V_SS;
  c20_u = c20_hoistedGlobal;
  c20_b_y = NULL;
  sf_mex_assign(&c20_b_y, sf_mex_create("y", &c20_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c20_y, 0, c20_b_y);
  c20_b_hoistedGlobal = *chartInstance->c20_X_dot;
  c20_b_u = c20_b_hoistedGlobal;
  c20_c_y = NULL;
  sf_mex_assign(&c20_c_y, sf_mex_create("y", &c20_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c20_y, 1, c20_c_y);
  c20_c_hoistedGlobal = *chartInstance->c20_Y_dot;
  c20_c_u = c20_c_hoistedGlobal;
  c20_d_y = NULL;
  sf_mex_assign(&c20_d_y, sf_mex_create("y", &c20_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c20_y, 2, c20_d_y);
  c20_d_hoistedGlobal = chartInstance->c20_is_active_c20_SS6_Estimation;
  c20_d_u = c20_d_hoistedGlobal;
  c20_e_y = NULL;
  sf_mex_assign(&c20_e_y, sf_mex_create("y", &c20_d_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c20_y, 3, c20_e_y);
  sf_mex_assign(&c20_st, c20_y, false);
  return c20_st;
}

static void set_sim_state_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct *
  chartInstance, const mxArray *c20_st)
{
  const mxArray *c20_u;
  chartInstance->c20_doneDoubleBufferReInit = true;
  c20_u = sf_mex_dup(c20_st);
  *chartInstance->c20_V_SS = c20_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c20_u, 0)), "V_SS");
  *chartInstance->c20_X_dot = c20_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c20_u, 1)), "X_dot");
  *chartInstance->c20_Y_dot = c20_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c20_u, 2)), "Y_dot");
  chartInstance->c20_is_active_c20_SS6_Estimation = c20_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c20_u, 3)),
     "is_active_c20_SS6_Estimation");
  sf_mex_destroy(&c20_u);
  c20_update_debugger_state_c20_SS6_Estimation(chartInstance);
  sf_mex_destroy(&c20_st);
}

static void finalize_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 19U, chartInstance->c20_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c20_U, 0U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c20_V, 1U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c20_theta, 2U);
  chartInstance->c20_sfEvent = CALL_EVENT;
  c20_chartstep_c20_SS6_Estimation(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SS6_EstimationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c20_X_dot, 3U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c20_Y_dot, 4U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c20_V_SS, 5U);
}

static void mdl_start_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c20_chartstep_c20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct *
  chartInstance)
{
  real_T c20_hoistedGlobal;
  real_T c20_b_hoistedGlobal;
  real_T c20_c_hoistedGlobal;
  real_T c20_b_U;
  real_T c20_b_V;
  real_T c20_b_theta;
  uint32_T c20_debug_family_var_map[46];
  real_T c20_M;
  real_T c20_Ms;
  real_T c20_Izz;
  real_T c20_Ixx;
  real_T c20_Ixz;
  real_T c20_a;
  real_T c20_b;
  real_T c20_hcg;
  real_T c20_e;
  real_T c20_Tw;
  real_T c20_R;
  real_T c20_I_t;
  real_T c20_I_e;
  real_T c20_A_f;
  real_T c20_K_bf;
  real_T c20_zi1;
  real_T c20_z2;
  real_T c20_zi3;
  real_T c20_zi4;
  real_T c20_zi5;
  real_T c20_Lbpsi;
  real_T c20_Lbp;
  real_T c20_C_a;
  real_T c20_C_i;
  real_T c20_mu;
  real_T c20_e_r;
  real_T c20_K_rsf;
  real_T c20_K_rsr;
  real_T c20_C1;
  real_T c20_C2;
  real_T c20_C3;
  real_T c20_C_s1;
  real_T c20_C_d;
  real_T c20_d;
  real_T c20_n;
  real_T c20_rho;
  real_T c20_g;
  real_T c20_c;
  real_T c20_nargin = 3.0;
  real_T c20_nargout = 3.0;
  real_T c20_b_X_dot;
  real_T c20_b_Y_dot;
  real_T c20_b_V_SS;
  real_T c20_x;
  real_T c20_b_x;
  real_T c20_c_x;
  real_T c20_d_x;
  real_T c20_e_x;
  real_T c20_f_x;
  real_T c20_g_x;
  real_T c20_h_x;
  real_T c20_A;
  real_T c20_B;
  real_T c20_i_x;
  real_T c20_y;
  real_T c20_j_x;
  real_T c20_b_y;
  real_T c20_k_x;
  real_T c20_c_y;
  real_T c20_d_y;
  real_T c20_l_x;
  real_T c20_m_x;
  real_T c20_n_x;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 19U, chartInstance->c20_sfEvent);
  c20_hoistedGlobal = *chartInstance->c20_U;
  c20_b_hoistedGlobal = *chartInstance->c20_V;
  c20_c_hoistedGlobal = *chartInstance->c20_theta;
  c20_b_U = c20_hoistedGlobal;
  c20_b_V = c20_b_hoistedGlobal;
  c20_b_theta = c20_c_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 46U, 46U, c20_debug_family_names,
    c20_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_M, 0U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_Ms, 1U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_Izz, 2U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_Ixx, 3U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_Ixz, 4U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_a, 5U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_b, 6U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_hcg, 7U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_e, 8U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_Tw, 9U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_R, 10U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_I_t, 11U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_I_e, 12U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_A_f, 13U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_K_bf, 14U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_zi1, 15U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_z2, 16U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_zi3, 17U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_zi4, 18U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_zi5, 19U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_Lbpsi, 20U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_Lbp, 21U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_C_a, 22U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_C_i, 23U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_mu, 24U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_e_r, 25U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_K_rsf, 26U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_K_rsr, 27U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_C1, 28U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_C2, 29U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_C3, 30U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_C_s1, 31U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_C_d, 32U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_d, 33U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_n, 34U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_rho, 35U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_g, 36U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_c, 37U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_nargin, 38U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_nargout, 39U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c20_b_U, 40U, c20_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c20_b_V, 41U, c20_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c20_b_theta, 42U, c20_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_b_X_dot, 43U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_b_Y_dot, 44U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_b_V_SS, 45U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_M = 1280.0;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_Ms = 1160.0;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_Izz = 2500.0;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_Ixx = 750.0;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_Ixz = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_a = 1.203;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_b = 1.217;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_hcg = 0.5;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_e = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_Tw = 1.33;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_R = 0.3;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 2);
  c20_I_t = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 3);
  c20_I_e = 0.136;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 3);
  c20_A_f = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 3);
  c20_K_bf = 0.55;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 3);
  c20_zi1 = 13.56;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 3);
  c20_z2 = 7.5;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 3);
  c20_zi3 = 5.37;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 3);
  c20_zi4 = 4.22;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 3);
  c20_zi5 = 3.28;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 4);
  c20_Lbpsi = 45000.0;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 4);
  c20_Lbp = 2600.0;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 4);
  c20_C_a = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 4);
  c20_C_i = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 4);
  c20_mu = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 4);
  c20_e_r = 0.015;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 4);
  c20_K_rsf = -0.05;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 4);
  c20_K_rsr = 0.1;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 5);
  c20_C1 = -6.0;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 5);
  c20_C2 = 59.16;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 5);
  c20_C3 = 25.0;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 5);
  c20_C_s1 = 1.38;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 5);
  c20_K_rsf = 0.444;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 5);
  c20_C_d = 0.32;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 5);
  c20_d = 0.014;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 5);
  c20_n = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 6);
  c20_rho = 1.204;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 6);
  c20_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 6);
  c20_c = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 8);
  c20_x = c20_b_theta;
  c20_b_x = c20_x;
  c20_b_x = muDoubleScalarCos(c20_b_x);
  c20_c_x = c20_b_theta;
  c20_d_x = c20_c_x;
  c20_d_x = muDoubleScalarSin(c20_d_x);
  c20_b_X_dot = c20_b_U * c20_b_x - c20_b_V * c20_d_x;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 9);
  c20_e_x = c20_b_theta;
  c20_f_x = c20_e_x;
  c20_f_x = muDoubleScalarSin(c20_f_x);
  c20_g_x = c20_b_theta;
  c20_h_x = c20_g_x;
  c20_h_x = muDoubleScalarCos(c20_h_x);
  c20_b_Y_dot = c20_b_U * c20_f_x + c20_b_V * c20_h_x;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 12);
  c20_A = c20_b_V;
  c20_B = c20_b_U;
  c20_i_x = c20_A;
  c20_y = c20_B;
  c20_j_x = c20_i_x;
  c20_b_y = c20_y;
  c20_k_x = c20_j_x;
  c20_c_y = c20_b_y;
  c20_d_y = c20_k_x / c20_c_y;
  c20_l_x = c20_d_y;
  c20_b_V_SS = c20_l_x;
  c20_m_x = c20_b_V_SS;
  c20_n_x = c20_m_x;
  c20_n_x = muDoubleScalarAtan(c20_n_x);
  c20_b_V_SS = 57.295779513082323 * c20_n_x;
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, -12);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c20_X_dot = c20_b_X_dot;
  *chartInstance->c20_Y_dot = c20_b_Y_dot;
  *chartInstance->c20_V_SS = c20_b_V_SS;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 19U, chartInstance->c20_sfEvent);
}

static void initSimStructsc20_SS6_Estimation(SFc20_SS6_EstimationInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c20_machineNumber, uint32_T
  c20_chartNumber, uint32_T c20_instanceNumber)
{
  (void)c20_machineNumber;
  (void)c20_chartNumber;
  (void)c20_instanceNumber;
}

static const mxArray *c20_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  real_T c20_u;
  const mxArray *c20_y = NULL;
  SFc20_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc20_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  c20_u = *(real_T *)c20_inData;
  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", &c20_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, false);
  return c20_mxArrayOutData;
}

static real_T c20_emlrt_marshallIn(SFc20_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c20_b_V_SS, const char_T *c20_identifier)
{
  real_T c20_y;
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_V_SS),
    &c20_thisId);
  sf_mex_destroy(&c20_b_V_SS);
  return c20_y;
}

static real_T c20_b_emlrt_marshallIn(SFc20_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId)
{
  real_T c20_y;
  real_T c20_d0;
  (void)chartInstance;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), &c20_d0, 1, 0, 0U, 0, 0U, 0);
  c20_y = c20_d0;
  sf_mex_destroy(&c20_u);
  return c20_y;
}

static void c20_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_b_V_SS;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y;
  SFc20_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc20_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c20_b_V_SS = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_V_SS),
    &c20_thisId);
  sf_mex_destroy(&c20_b_V_SS);
  *(real_T *)c20_outData = c20_y;
  sf_mex_destroy(&c20_mxArrayInData);
}

const mxArray *sf_c20_SS6_Estimation_get_eml_resolved_functions_info(void)
{
  const mxArray *c20_nameCaptureInfo = NULL;
  c20_nameCaptureInfo = NULL;
  sf_mex_assign(&c20_nameCaptureInfo, sf_mex_createstruct("structure", 2, 14, 1),
                false);
  c20_info_helper(&c20_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c20_nameCaptureInfo);
  return c20_nameCaptureInfo;
}

static void c20_info_helper(const mxArray **c20_info)
{
  const mxArray *c20_rhs0 = NULL;
  const mxArray *c20_lhs0 = NULL;
  const mxArray *c20_rhs1 = NULL;
  const mxArray *c20_lhs1 = NULL;
  const mxArray *c20_rhs2 = NULL;
  const mxArray *c20_lhs2 = NULL;
  const mxArray *c20_rhs3 = NULL;
  const mxArray *c20_lhs3 = NULL;
  const mxArray *c20_rhs4 = NULL;
  const mxArray *c20_lhs4 = NULL;
  const mxArray *c20_rhs5 = NULL;
  const mxArray *c20_lhs5 = NULL;
  const mxArray *c20_rhs6 = NULL;
  const mxArray *c20_lhs6 = NULL;
  const mxArray *c20_rhs7 = NULL;
  const mxArray *c20_lhs7 = NULL;
  const mxArray *c20_rhs8 = NULL;
  const mxArray *c20_lhs8 = NULL;
  const mxArray *c20_rhs9 = NULL;
  const mxArray *c20_lhs9 = NULL;
  const mxArray *c20_rhs10 = NULL;
  const mxArray *c20_lhs10 = NULL;
  const mxArray *c20_rhs11 = NULL;
  const mxArray *c20_lhs11 = NULL;
  const mxArray *c20_rhs12 = NULL;
  const mxArray *c20_lhs12 = NULL;
  const mxArray *c20_rhs13 = NULL;
  const mxArray *c20_lhs13 = NULL;
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("cos"), "name", "name", 0);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1395346496U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c20_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs0), "rhs", "rhs",
                  0);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs0), "lhs", "lhs",
                  0);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 1);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1286840322U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c20_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs1), "rhs", "rhs",
                  1);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs1), "lhs", "lhs",
                  1);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(""), "context", "context", 2);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("sin"), "name", "name", 2);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1395346504U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c20_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs2), "rhs", "rhs",
                  2);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs2), "lhs", "lhs",
                  2);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 3);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1286840336U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c20_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs3), "rhs", "rhs",
                  3);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs3), "lhs", "lhs",
                  3);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(""), "context", "context", 4);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("mrdivide"), "name", "name",
                  4);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c20_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs4), "rhs", "rhs",
                  4);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs4), "lhs", "lhs",
                  4);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 5);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 5);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1389739374U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c20_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs5), "rhs", "rhs",
                  5);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs5), "lhs", "lhs",
                  5);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 6);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("rdivide"), "name", "name", 6);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c20_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs6), "rhs", "rhs",
                  6);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs6), "lhs", "lhs",
                  6);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 7);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c20_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs7), "rhs", "rhs",
                  7);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs7), "lhs", "lhs",
                  7);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 8);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c20_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs8), "rhs", "rhs",
                  8);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs8), "lhs", "lhs",
                  8);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("eml_div"), "name", "name", 9);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c20_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs9), "rhs", "rhs",
                  9);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs9), "lhs", "lhs",
                  9);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 10);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c20_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(""), "context", "context", 11);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("atand"), "name", "name", 11);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atand.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1395346496U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c20_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atand.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("mrdivide"), "name", "name",
                  12);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c20_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atand.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("eml_scalar_atan"), "name",
                  "name", 13);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c20_info, c20_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(1286840318U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c20_info, c20_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c20_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c20_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c20_info, sf_mex_duplicatearraysafe(&c20_lhs13), "lhs", "lhs",
                  13);
  sf_mex_destroy(&c20_rhs0);
  sf_mex_destroy(&c20_lhs0);
  sf_mex_destroy(&c20_rhs1);
  sf_mex_destroy(&c20_lhs1);
  sf_mex_destroy(&c20_rhs2);
  sf_mex_destroy(&c20_lhs2);
  sf_mex_destroy(&c20_rhs3);
  sf_mex_destroy(&c20_lhs3);
  sf_mex_destroy(&c20_rhs4);
  sf_mex_destroy(&c20_lhs4);
  sf_mex_destroy(&c20_rhs5);
  sf_mex_destroy(&c20_lhs5);
  sf_mex_destroy(&c20_rhs6);
  sf_mex_destroy(&c20_lhs6);
  sf_mex_destroy(&c20_rhs7);
  sf_mex_destroy(&c20_lhs7);
  sf_mex_destroy(&c20_rhs8);
  sf_mex_destroy(&c20_lhs8);
  sf_mex_destroy(&c20_rhs9);
  sf_mex_destroy(&c20_lhs9);
  sf_mex_destroy(&c20_rhs10);
  sf_mex_destroy(&c20_lhs10);
  sf_mex_destroy(&c20_rhs11);
  sf_mex_destroy(&c20_lhs11);
  sf_mex_destroy(&c20_rhs12);
  sf_mex_destroy(&c20_lhs12);
  sf_mex_destroy(&c20_rhs13);
  sf_mex_destroy(&c20_lhs13);
}

static const mxArray *c20_emlrt_marshallOut(const char * c20_u)
{
  const mxArray *c20_y = NULL;
  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c20_u)), false);
  return c20_y;
}

static const mxArray *c20_b_emlrt_marshallOut(const uint32_T c20_u)
{
  const mxArray *c20_y = NULL;
  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", &c20_u, 7, 0U, 0U, 0U, 0), false);
  return c20_y;
}

static const mxArray *c20_b_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_u;
  const mxArray *c20_y = NULL;
  SFc20_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc20_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  c20_u = *(int32_T *)c20_inData;
  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", &c20_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, false);
  return c20_mxArrayOutData;
}

static int32_T c20_c_emlrt_marshallIn(SFc20_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId)
{
  int32_T c20_y;
  int32_T c20_i0;
  (void)chartInstance;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), &c20_i0, 1, 6, 0U, 0, 0U, 0);
  c20_y = c20_i0;
  sf_mex_destroy(&c20_u);
  return c20_y;
}

static void c20_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_b_sfEvent;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  int32_T c20_y;
  SFc20_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc20_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c20_b_sfEvent = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_sfEvent),
    &c20_thisId);
  sf_mex_destroy(&c20_b_sfEvent);
  *(int32_T *)c20_outData = c20_y;
  sf_mex_destroy(&c20_mxArrayInData);
}

static uint8_T c20_d_emlrt_marshallIn(SFc20_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c20_b_is_active_c20_SS6_Estimation, const
  char_T *c20_identifier)
{
  uint8_T c20_y;
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c20_b_is_active_c20_SS6_Estimation), &c20_thisId);
  sf_mex_destroy(&c20_b_is_active_c20_SS6_Estimation);
  return c20_y;
}

static uint8_T c20_e_emlrt_marshallIn(SFc20_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId)
{
  uint8_T c20_y;
  uint8_T c20_u0;
  (void)chartInstance;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), &c20_u0, 1, 3, 0U, 0, 0U, 0);
  c20_y = c20_u0;
  sf_mex_destroy(&c20_u);
  return c20_y;
}

static void init_dsm_address_info(SFc20_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc20_SS6_EstimationInstanceStruct
  *chartInstance)
{
  chartInstance->c20_U = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    0);
  chartInstance->c20_V = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c20_theta = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c20_X_dot = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c20_Y_dot = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c20_V_SS = (real_T *)ssGetOutputPortSignal_wrapper
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

void sf_c20_SS6_Estimation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(161937356U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2014606891U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3571188621U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2416102041U);
}

mxArray* sf_c20_SS6_Estimation_get_post_codegen_info(void);
mxArray *sf_c20_SS6_Estimation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("y1cG55g0wEVBwWeaZX6MVB");
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
    mxArray* mxPostCodegenInfo = sf_c20_SS6_Estimation_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c20_SS6_Estimation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c20_SS6_Estimation_jit_fallback_info(void)
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

mxArray *sf_c20_SS6_Estimation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c20_SS6_Estimation_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c20_SS6_Estimation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[18],T\"V_SS\",},{M[1],M[5],T\"X_dot\",},{M[1],M[15],T\"Y_dot\",},{M[8],M[0],T\"is_active_c20_SS6_Estimation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c20_SS6_Estimation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc20_SS6_EstimationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc20_SS6_EstimationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SS6_EstimationMachineNumber_,
           20,
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",4,-1,544);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_sf_marshallOut,(MexInFcnForType)c20_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_sf_marshallOut,(MexInFcnForType)c20_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_sf_marshallOut,(MexInFcnForType)c20_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c20_U);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c20_V);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c20_theta);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c20_X_dot);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c20_Y_dot);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c20_V_SS);
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
  return "45vR29wOiaO707YVLpykwF";
}

static void sf_opaque_initialize_c20_SS6_Estimation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc20_SS6_EstimationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c20_SS6_Estimation((SFc20_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
  initialize_c20_SS6_Estimation((SFc20_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c20_SS6_Estimation(void *chartInstanceVar)
{
  enable_c20_SS6_Estimation((SFc20_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c20_SS6_Estimation(void *chartInstanceVar)
{
  disable_c20_SS6_Estimation((SFc20_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c20_SS6_Estimation(void *chartInstanceVar)
{
  sf_gateway_c20_SS6_Estimation((SFc20_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c20_SS6_Estimation(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c20_SS6_Estimation((SFc20_SS6_EstimationInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c20_SS6_Estimation(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c20_SS6_Estimation((SFc20_SS6_EstimationInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c20_SS6_Estimation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc20_SS6_EstimationInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SS6_Estimation_optimization_info();
    }

    finalize_c20_SS6_Estimation((SFc20_SS6_EstimationInstanceStruct*)
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
  initSimStructsc20_SS6_Estimation((SFc20_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c20_SS6_Estimation(SimStruct *S)
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
    initialize_params_c20_SS6_Estimation((SFc20_SS6_EstimationInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c20_SS6_Estimation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SS6_Estimation_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      20);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,20,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,20,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,20);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,20,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,20,3);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,20);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4284969431U));
  ssSetChecksum1(S,(3507547071U));
  ssSetChecksum2(S,(2237598159U));
  ssSetChecksum3(S,(3062236694U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c20_SS6_Estimation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c20_SS6_Estimation(SimStruct *S)
{
  SFc20_SS6_EstimationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc20_SS6_EstimationInstanceStruct *)utMalloc(sizeof
    (SFc20_SS6_EstimationInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc20_SS6_EstimationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c20_SS6_Estimation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c20_SS6_Estimation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c20_SS6_Estimation;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c20_SS6_Estimation;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c20_SS6_Estimation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c20_SS6_Estimation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c20_SS6_Estimation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c20_SS6_Estimation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c20_SS6_Estimation;
  chartInstance->chartInfo.mdlStart = mdlStart_c20_SS6_Estimation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c20_SS6_Estimation;
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

void c20_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c20_SS6_Estimation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c20_SS6_Estimation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c20_SS6_Estimation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c20_SS6_Estimation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
