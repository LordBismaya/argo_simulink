/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SS_jun_25_sfun.h"
#include "c7_SS_jun_25.h"
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
static const char * c7_debug_family_names[66] = { "M", "Ms", "Izz", "Ixx", "Ixz",
  "a", "b", "hcg", "e", "Tw", "R", "I_t", "I_e", "A_f", "K_bf", "zi1", "z2",
  "zi3", "zi4", "zi5", "Lbpsi", "Lbp", "C_a", "C_i", "mu", "e_r", "K_rsf",
  "K_rsr", "C1", "C2", "C3", "C_s1", "C_d", "d", "n", "rho", "g", "c", "Fz",
  "U_tl", "U_tr", "alpha", "S_f", "f_S", "F_tL_f", "S_r", "F_tL_r", "S_m",
  "F_tL_m", "F_L", "nargin", "nargout", "FzL", "U", "V", "Slip", "delta", "r",
  "F_SL", "F_tL", "a_fL", "a_rL", "a_mL", "F_SL_f", "F_SL_r", "F_SL_m" };

/* Function Declarations */
static void initialize_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance);
static void initialize_params_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance);
static void enable_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance);
static void disable_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance);
static void c7_update_debugger_state_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance);
static void set_sim_state_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance, const mxArray *c7_st);
static void finalize_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance);
static void sf_gateway_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance);
static void mdl_start_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance);
static void c7_chartstep_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance);
static void initSimStructsc7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber, uint32_T c7_instanceNumber);
static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData);
static real_T c7_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_b_F_SL_m, const char_T *c7_identifier);
static real_T c7_b_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_c_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[6]);
static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static void c7_info_helper(const mxArray **c7_info);
static const mxArray *c7_emlrt_marshallOut(const char * c7_u);
static const mxArray *c7_b_emlrt_marshallOut(const uint32_T c7_u);
static real_T c7_sqrt(SFc7_SS_jun_25InstanceStruct *chartInstance, real_T c7_x);
static void c7_eml_error(SFc7_SS_jun_25InstanceStruct *chartInstance);
static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static int32_T c7_d_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static uint8_T c7_e_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_b_is_active_c7_SS_jun_25, const char_T *c7_identifier);
static uint8_T c7_f_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_b_sqrt(SFc7_SS_jun_25InstanceStruct *chartInstance, real_T *c7_x);
static void init_dsm_address_info(SFc7_SS_jun_25InstanceStruct *chartInstance);
static void init_simulink_io_address(SFc7_SS_jun_25InstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance)
{
  chartInstance->c7_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c7_is_active_c7_SS_jun_25 = 0U;
}

static void initialize_params_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c7_update_debugger_state_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance)
{
  const mxArray *c7_st;
  const mxArray *c7_y = NULL;
  real_T c7_hoistedGlobal;
  real_T c7_u;
  const mxArray *c7_b_y = NULL;
  real_T c7_b_hoistedGlobal;
  real_T c7_b_u;
  const mxArray *c7_c_y = NULL;
  real_T c7_c_hoistedGlobal;
  real_T c7_c_u;
  const mxArray *c7_d_y = NULL;
  real_T c7_d_hoistedGlobal;
  real_T c7_d_u;
  const mxArray *c7_e_y = NULL;
  real_T c7_e_hoistedGlobal;
  real_T c7_e_u;
  const mxArray *c7_f_y = NULL;
  real_T c7_f_hoistedGlobal;
  real_T c7_f_u;
  const mxArray *c7_g_y = NULL;
  real_T c7_g_hoistedGlobal;
  real_T c7_g_u;
  const mxArray *c7_h_y = NULL;
  real_T c7_h_hoistedGlobal;
  real_T c7_h_u;
  const mxArray *c7_i_y = NULL;
  uint8_T c7_i_hoistedGlobal;
  uint8_T c7_i_u;
  const mxArray *c7_j_y = NULL;
  c7_st = NULL;
  c7_st = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_createcellmatrix(9, 1), false);
  c7_hoistedGlobal = *chartInstance->c7_F_SL;
  c7_u = c7_hoistedGlobal;
  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", &c7_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c7_y, 0, c7_b_y);
  c7_b_hoistedGlobal = *chartInstance->c7_F_SL_f;
  c7_b_u = c7_b_hoistedGlobal;
  c7_c_y = NULL;
  sf_mex_assign(&c7_c_y, sf_mex_create("y", &c7_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c7_y, 1, c7_c_y);
  c7_c_hoistedGlobal = *chartInstance->c7_F_SL_m;
  c7_c_u = c7_c_hoistedGlobal;
  c7_d_y = NULL;
  sf_mex_assign(&c7_d_y, sf_mex_create("y", &c7_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c7_y, 2, c7_d_y);
  c7_d_hoistedGlobal = *chartInstance->c7_F_SL_r;
  c7_d_u = c7_d_hoistedGlobal;
  c7_e_y = NULL;
  sf_mex_assign(&c7_e_y, sf_mex_create("y", &c7_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c7_y, 3, c7_e_y);
  c7_e_hoistedGlobal = *chartInstance->c7_F_tL;
  c7_e_u = c7_e_hoistedGlobal;
  c7_f_y = NULL;
  sf_mex_assign(&c7_f_y, sf_mex_create("y", &c7_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c7_y, 4, c7_f_y);
  c7_f_hoistedGlobal = *chartInstance->c7_a_fL;
  c7_f_u = c7_f_hoistedGlobal;
  c7_g_y = NULL;
  sf_mex_assign(&c7_g_y, sf_mex_create("y", &c7_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c7_y, 5, c7_g_y);
  c7_g_hoistedGlobal = *chartInstance->c7_a_mL;
  c7_g_u = c7_g_hoistedGlobal;
  c7_h_y = NULL;
  sf_mex_assign(&c7_h_y, sf_mex_create("y", &c7_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c7_y, 6, c7_h_y);
  c7_h_hoistedGlobal = *chartInstance->c7_a_rL;
  c7_h_u = c7_h_hoistedGlobal;
  c7_i_y = NULL;
  sf_mex_assign(&c7_i_y, sf_mex_create("y", &c7_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c7_y, 7, c7_i_y);
  c7_i_hoistedGlobal = chartInstance->c7_is_active_c7_SS_jun_25;
  c7_i_u = c7_i_hoistedGlobal;
  c7_j_y = NULL;
  sf_mex_assign(&c7_j_y, sf_mex_create("y", &c7_i_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c7_y, 8, c7_j_y);
  sf_mex_assign(&c7_st, c7_y, false);
  return c7_st;
}

static void set_sim_state_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance, const mxArray *c7_st)
{
  const mxArray *c7_u;
  chartInstance->c7_doneDoubleBufferReInit = true;
  c7_u = sf_mex_dup(c7_st);
  *chartInstance->c7_F_SL = c7_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c7_u, 0)), "F_SL");
  *chartInstance->c7_F_SL_f = c7_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c7_u, 1)), "F_SL_f");
  *chartInstance->c7_F_SL_m = c7_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c7_u, 2)), "F_SL_m");
  *chartInstance->c7_F_SL_r = c7_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c7_u, 3)), "F_SL_r");
  *chartInstance->c7_F_tL = c7_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c7_u, 4)), "F_tL");
  *chartInstance->c7_a_fL = c7_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c7_u, 5)), "a_fL");
  *chartInstance->c7_a_mL = c7_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c7_u, 6)), "a_mL");
  *chartInstance->c7_a_rL = c7_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c7_u, 7)), "a_rL");
  chartInstance->c7_is_active_c7_SS_jun_25 = c7_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c7_u, 8)), "is_active_c7_SS_jun_25");
  sf_mex_destroy(&c7_u);
  c7_update_debugger_state_c7_SS_jun_25(chartInstance);
  sf_mex_destroy(&c7_st);
}

static void finalize_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance)
{
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 5U, chartInstance->c7_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_FzL, 0U);
  chartInstance->c7_sfEvent = CALL_EVENT;
  c7_chartstep_c7_SS_jun_25(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SS_jun_25MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_F_SL, 1U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_F_tL, 2U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_U, 3U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_V, 4U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_Slip, 5U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_delta, 6U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_r, 7U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_a_fL, 8U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_a_rL, 9U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_a_mL, 10U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_F_SL_f, 11U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_F_SL_r, 12U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c7_F_SL_m, 13U);
}

static void mdl_start_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c7_chartstep_c7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance)
{
  real_T c7_hoistedGlobal;
  real_T c7_b_hoistedGlobal;
  real_T c7_c_hoistedGlobal;
  real_T c7_d_hoistedGlobal;
  real_T c7_e_hoistedGlobal;
  real_T c7_f_hoistedGlobal;
  real_T c7_b_FzL;
  real_T c7_b_U;
  real_T c7_b_V;
  real_T c7_b_Slip;
  real_T c7_b_delta;
  real_T c7_b_r;
  uint32_T c7_debug_family_var_map[66];
  real_T c7_M;
  real_T c7_Ms;
  real_T c7_Izz;
  real_T c7_Ixx;
  real_T c7_Ixz;
  real_T c7_a;
  real_T c7_b;
  real_T c7_hcg;
  real_T c7_e;
  real_T c7_Tw;
  real_T c7_R;
  real_T c7_I_t;
  real_T c7_I_e;
  real_T c7_A_f;
  real_T c7_K_bf;
  real_T c7_zi1;
  real_T c7_z2;
  real_T c7_zi3;
  real_T c7_zi4;
  real_T c7_zi5;
  real_T c7_Lbpsi;
  real_T c7_Lbp;
  real_T c7_C_a;
  real_T c7_C_i;
  real_T c7_mu;
  real_T c7_e_r;
  real_T c7_K_rsf;
  real_T c7_K_rsr;
  real_T c7_C1;
  real_T c7_C2;
  real_T c7_C3;
  real_T c7_C_s1;
  real_T c7_C_d;
  real_T c7_d;
  real_T c7_n;
  real_T c7_rho;
  real_T c7_g;
  real_T c7_c;
  real_T c7_Fz;
  real_T c7_U_tl;
  real_T c7_U_tr;
  real_T c7_alpha;
  real_T c7_S_f;
  real_T c7_f_S;
  real_T c7_F_tL_f;
  real_T c7_S_r;
  real_T c7_F_tL_r;
  real_T c7_S_m;
  real_T c7_F_tL_m;
  real_T c7_F_L[6];
  real_T c7_nargin = 6.0;
  real_T c7_nargout = 8.0;
  real_T c7_b_F_SL;
  real_T c7_b_F_tL;
  real_T c7_b_a_fL;
  real_T c7_b_a_rL;
  real_T c7_b_a_mL;
  real_T c7_b_F_SL_f;
  real_T c7_b_F_SL_r;
  real_T c7_b_F_SL_m;
  real_T c7_A;
  real_T c7_B;
  real_T c7_x;
  real_T c7_y;
  real_T c7_b_x;
  real_T c7_b_y;
  real_T c7_c_x;
  real_T c7_c_y;
  real_T c7_d_y;
  real_T c7_d_x;
  real_T c7_e_x;
  real_T c7_b_A;
  real_T c7_b_B;
  real_T c7_f_x;
  real_T c7_e_y;
  real_T c7_g_x;
  real_T c7_f_y;
  real_T c7_h_x;
  real_T c7_g_y;
  real_T c7_h_y;
  real_T c7_i_x;
  real_T c7_j_x;
  real_T c7_c_A;
  real_T c7_c_B;
  real_T c7_k_x;
  real_T c7_i_y;
  real_T c7_l_x;
  real_T c7_j_y;
  real_T c7_m_x;
  real_T c7_k_y;
  real_T c7_l_y;
  real_T c7_n_x;
  real_T c7_o_x;
  real_T c7_d_A;
  real_T c7_p_x;
  real_T c7_q_x;
  real_T c7_r_x;
  real_T c7_m_y;
  real_T c7_s_x;
  real_T c7_t_x;
  real_T c7_u_x;
  real_T c7_v_x;
  real_T c7_w_x;
  real_T c7_x_x;
  real_T c7_y_x;
  real_T c7_ab_x;
  real_T c7_d0;
  real_T c7_e_A;
  real_T c7_d1;
  real_T c7_d_B;
  real_T c7_bb_x;
  real_T c7_n_y;
  real_T c7_cb_x;
  real_T c7_o_y;
  real_T c7_db_x;
  real_T c7_p_y;
  real_T c7_eb_x;
  real_T c7_fb_x;
  real_T c7_f_A;
  real_T c7_e_B;
  real_T c7_gb_x;
  real_T c7_q_y;
  real_T c7_hb_x;
  real_T c7_r_y;
  real_T c7_ib_x;
  real_T c7_s_y;
  real_T c7_g_A;
  real_T c7_f_B;
  real_T c7_jb_x;
  real_T c7_t_y;
  real_T c7_kb_x;
  real_T c7_u_y;
  real_T c7_lb_x;
  real_T c7_v_y;
  real_T c7_h_A;
  real_T c7_mb_x;
  real_T c7_nb_x;
  real_T c7_ob_x;
  real_T c7_w_y;
  real_T c7_pb_x;
  real_T c7_qb_x;
  real_T c7_rb_x;
  real_T c7_sb_x;
  real_T c7_tb_x;
  real_T c7_ub_x;
  real_T c7_vb_x;
  real_T c7_wb_x;
  real_T c7_d2;
  real_T c7_i_A;
  real_T c7_d3;
  real_T c7_g_B;
  real_T c7_xb_x;
  real_T c7_x_y;
  real_T c7_yb_x;
  real_T c7_y_y;
  real_T c7_ac_x;
  real_T c7_ab_y;
  real_T c7_bc_x;
  real_T c7_cc_x;
  real_T c7_j_A;
  real_T c7_h_B;
  real_T c7_dc_x;
  real_T c7_bb_y;
  real_T c7_ec_x;
  real_T c7_cb_y;
  real_T c7_fc_x;
  real_T c7_db_y;
  real_T c7_k_A;
  real_T c7_i_B;
  real_T c7_gc_x;
  real_T c7_eb_y;
  real_T c7_hc_x;
  real_T c7_fb_y;
  real_T c7_ic_x;
  real_T c7_gb_y;
  real_T c7_l_A;
  real_T c7_jc_x;
  real_T c7_kc_x;
  real_T c7_lc_x;
  real_T c7_hb_y;
  real_T c7_mc_x;
  real_T c7_nc_x;
  real_T c7_oc_x;
  real_T c7_pc_x;
  real_T c7_qc_x;
  real_T c7_rc_x;
  real_T c7_sc_x;
  real_T c7_tc_x;
  real_T c7_uc_x;
  real_T c7_vc_x;
  real_T c7_wc_x;
  real_T c7_xc_x;
  real_T c7_m_A;
  real_T c7_j_B;
  real_T c7_yc_x;
  real_T c7_ib_y;
  real_T c7_ad_x;
  real_T c7_jb_y;
  real_T c7_bd_x;
  real_T c7_kb_y;
  real_T c7_cd_x;
  real_T c7_dd_x;
  real_T c7_n_A;
  real_T c7_k_B;
  real_T c7_ed_x;
  real_T c7_lb_y;
  real_T c7_fd_x;
  real_T c7_mb_y;
  real_T c7_gd_x;
  real_T c7_nb_y;
  real_T c7_o_A;
  real_T c7_l_B;
  real_T c7_hd_x;
  real_T c7_ob_y;
  real_T c7_id_x;
  real_T c7_pb_y;
  real_T c7_jd_x;
  real_T c7_qb_y;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 5U, chartInstance->c7_sfEvent);
  c7_hoistedGlobal = *chartInstance->c7_FzL;
  c7_b_hoistedGlobal = *chartInstance->c7_U;
  c7_c_hoistedGlobal = *chartInstance->c7_V;
  c7_d_hoistedGlobal = *chartInstance->c7_Slip;
  c7_e_hoistedGlobal = *chartInstance->c7_delta;
  c7_f_hoistedGlobal = *chartInstance->c7_r;
  c7_b_FzL = c7_hoistedGlobal;
  c7_b_U = c7_b_hoistedGlobal;
  c7_b_V = c7_c_hoistedGlobal;
  c7_b_Slip = c7_d_hoistedGlobal;
  c7_b_delta = c7_e_hoistedGlobal;
  c7_b_r = c7_f_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 66U, 66U, c7_debug_family_names,
    c7_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_M, 0U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_Ms, 1U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_Izz, 2U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_Ixx, 3U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_Ixz, 4U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_a, 5U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_b, 6U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_hcg, 7U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_e, 8U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_Tw, 9U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_R, 10U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_I_t, 11U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_I_e, 12U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_A_f, 13U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_K_bf, 14U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_zi1, 15U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_z2, 16U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_zi3, 17U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_zi4, 18U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_zi5, 19U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_Lbpsi, 20U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_Lbp, 21U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_C_a, 22U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_C_i, 23U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_mu, 24U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_e_r, 25U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_K_rsf, 26U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_K_rsr, 27U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_C1, 28U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_C2, 29U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_C3, 30U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_C_s1, 31U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_C_d, 32U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_d, 33U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_n, 34U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_rho, 35U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_g, 36U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_c, 37U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_Fz, 38U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_U_tl, 39U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_U_tr, 40U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_alpha, 41U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_S_f, 42U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_f_S, 43U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_F_tL_f, 44U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_S_r, 45U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_F_tL_r, 46U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_S_m, 47U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_F_tL_m, 48U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_F_L, 49U, c7_b_sf_marshallOut,
    c7_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargin, 50U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargout, 51U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_b_FzL, 52U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_b_U, 53U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_b_V, 54U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_b_Slip, 55U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_b_delta, 56U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_b_r, 57U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_b_F_SL, 58U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_b_F_tL, 59U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_b_a_fL, 60U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_b_a_rL, 61U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_b_a_mL, 62U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_b_F_SL_f, 63U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_b_F_SL_r, 64U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_b_F_SL_m, 65U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_M = 1280.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_Ms = 1160.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_Izz = 2500.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_Ixx = 750.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_Ixz = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_a = 1.203;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_b = 1.217;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_hcg = 0.5;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_e = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_Tw = 1.33;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_R = 0.3;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 2);
  c7_I_t = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 3);
  c7_I_e = 0.136;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 3);
  c7_A_f = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 3);
  c7_K_bf = 0.55;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 3);
  c7_zi1 = 13.56;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 3);
  c7_z2 = 7.5;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 3);
  c7_zi3 = 5.37;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 3);
  c7_zi4 = 4.22;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 3);
  c7_zi5 = 3.28;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 4);
  c7_Lbpsi = 45000.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 4);
  c7_Lbp = 2600.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 4);
  c7_C_a = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 4);
  c7_C_i = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 4);
  c7_mu = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 4);
  c7_e_r = 0.015;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 4);
  c7_K_rsf = -0.05;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 4);
  c7_K_rsr = 0.1;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 5);
  c7_C1 = -6.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 5);
  c7_C2 = 59.16;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 5);
  c7_C3 = 25.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 5);
  c7_C_s1 = 1.38;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 5);
  c7_K_rsf = 0.444;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 5);
  c7_C_d = 0.32;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 5);
  c7_d = 0.014;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 5);
  c7_n = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 6);
  c7_rho = 1.204;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 6);
  c7_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 6);
  c7_c = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 11);
  c7_Fz = c7_b_FzL;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 16);
  c7_U_tl = c7_b_U + 0.665 * c7_b_r;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 17);
  c7_U_tr = c7_b_U - 0.665 * c7_b_r;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 20);
  c7_A = c7_b_V + 1.203 * c7_b_r;
  c7_B = c7_b_U + 0.665 * c7_b_r;
  c7_x = c7_A;
  c7_y = c7_B;
  c7_b_x = c7_x;
  c7_b_y = c7_y;
  c7_c_x = c7_b_x;
  c7_c_y = c7_b_y;
  c7_d_y = c7_c_x / c7_c_y;
  c7_d_x = c7_d_y;
  c7_e_x = c7_d_x;
  c7_e_x = muDoubleScalarAtan(c7_e_x);
  c7_b_a_fL = -c7_e_x;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 21);
  c7_b_A = 1.217 * c7_b_r - c7_b_V;
  c7_b_B = c7_b_U + 0.665 * c7_b_r;
  c7_f_x = c7_b_A;
  c7_e_y = c7_b_B;
  c7_g_x = c7_f_x;
  c7_f_y = c7_e_y;
  c7_h_x = c7_g_x;
  c7_g_y = c7_f_y;
  c7_h_y = c7_h_x / c7_g_y;
  c7_i_x = c7_h_y;
  c7_b_a_rL = c7_i_x;
  c7_j_x = c7_b_a_rL;
  c7_b_a_rL = c7_j_x;
  c7_b_a_rL = muDoubleScalarAtan(c7_b_a_rL);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 22);
  c7_c_A = 0.2 * c7_b_r - c7_b_V;
  c7_c_B = c7_b_U + 0.665 * c7_b_r;
  c7_k_x = c7_c_A;
  c7_i_y = c7_c_B;
  c7_l_x = c7_k_x;
  c7_j_y = c7_i_y;
  c7_m_x = c7_l_x;
  c7_k_y = c7_j_y;
  c7_l_y = c7_m_x / c7_k_y;
  c7_n_x = c7_l_y;
  c7_b_a_mL = c7_n_x;
  c7_o_x = c7_b_a_mL;
  c7_b_a_mL = c7_o_x;
  c7_b_a_mL = muDoubleScalarAtan(c7_b_a_mL);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 24);
  c7_alpha = c7_b_a_fL;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 26);
  c7_d_A = c7_Fz;
  c7_p_x = c7_d_A;
  c7_q_x = c7_p_x;
  c7_r_x = c7_q_x;
  c7_m_y = c7_r_x / 3.0;
  c7_s_x = c7_alpha;
  c7_t_x = c7_s_x;
  c7_t_x = muDoubleScalarTan(c7_t_x);
  c7_u_x = c7_alpha;
  c7_v_x = c7_u_x;
  c7_v_x = muDoubleScalarTan(c7_v_x);
  c7_w_x = c7_alpha;
  c7_x_x = c7_w_x;
  c7_x_x = muDoubleScalarTan(c7_x_x);
  c7_y_x = c7_alpha;
  c7_ab_x = c7_y_x;
  c7_ab_x = muDoubleScalarTan(c7_ab_x);
  c7_d0 = c7_b_Slip * c7_b_Slip + c7_t_x * c7_v_x;
  c7_b_sqrt(chartInstance, &c7_d0);
  c7_e_A = 0.85 * c7_m_y * (1.0 - 0.015 * c7_U_tl * c7_d0) * (1.0 - c7_b_Slip);
  c7_d1 = 4.0E+8 * c7_b_Slip * c7_b_Slip + 4.0E+8 * c7_x_x * c7_ab_x;
  c7_b_sqrt(chartInstance, &c7_d1);
  c7_d_B = 2.0 * c7_d1;
  c7_bb_x = c7_e_A;
  c7_n_y = c7_d_B;
  c7_cb_x = c7_bb_x;
  c7_o_y = c7_n_y;
  c7_db_x = c7_cb_x;
  c7_p_y = c7_o_y;
  c7_S_f = c7_db_x / c7_p_y;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 28);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c7_S_f, 1.0, -1, 2U,
        c7_S_f < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 29);
    c7_f_S = c7_S_f * (2.0 - c7_S_f);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 31);
    c7_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 33);
  c7_eb_x = c7_alpha;
  c7_fb_x = c7_eb_x;
  c7_fb_x = muDoubleScalarTan(c7_fb_x);
  c7_f_A = 20000.0 * c7_fb_x * c7_f_S;
  c7_e_B = 1.0 - c7_b_Slip;
  c7_gb_x = c7_f_A;
  c7_q_y = c7_e_B;
  c7_hb_x = c7_gb_x;
  c7_r_y = c7_q_y;
  c7_ib_x = c7_hb_x;
  c7_s_y = c7_r_y;
  c7_b_F_SL_f = c7_ib_x / c7_s_y;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 34);
  c7_g_A = 20000.0 * c7_b_Slip * c7_f_S;
  c7_f_B = 1.0 - c7_b_Slip;
  c7_jb_x = c7_g_A;
  c7_t_y = c7_f_B;
  c7_kb_x = c7_jb_x;
  c7_u_y = c7_t_y;
  c7_lb_x = c7_kb_x;
  c7_v_y = c7_u_y;
  c7_F_tL_f = c7_lb_x / c7_v_y;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 37);
  c7_alpha = c7_b_a_rL;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 39);
  c7_h_A = c7_Fz;
  c7_mb_x = c7_h_A;
  c7_nb_x = c7_mb_x;
  c7_ob_x = c7_nb_x;
  c7_w_y = c7_ob_x / 3.0;
  c7_pb_x = c7_alpha;
  c7_qb_x = c7_pb_x;
  c7_qb_x = muDoubleScalarTan(c7_qb_x);
  c7_rb_x = c7_alpha;
  c7_sb_x = c7_rb_x;
  c7_sb_x = muDoubleScalarTan(c7_sb_x);
  c7_tb_x = c7_alpha;
  c7_ub_x = c7_tb_x;
  c7_ub_x = muDoubleScalarTan(c7_ub_x);
  c7_vb_x = c7_alpha;
  c7_wb_x = c7_vb_x;
  c7_wb_x = muDoubleScalarTan(c7_wb_x);
  c7_d2 = c7_b_Slip * c7_b_Slip + c7_qb_x * c7_sb_x;
  c7_b_sqrt(chartInstance, &c7_d2);
  c7_i_A = 0.85 * c7_w_y * (1.0 - 0.015 * c7_U_tl * c7_d2) * (1.0 - c7_b_Slip);
  c7_d3 = 4.0E+8 * c7_b_Slip * c7_b_Slip + 4.0E+8 * c7_ub_x * c7_wb_x;
  c7_b_sqrt(chartInstance, &c7_d3);
  c7_g_B = 2.0 * c7_d3;
  c7_xb_x = c7_i_A;
  c7_x_y = c7_g_B;
  c7_yb_x = c7_xb_x;
  c7_y_y = c7_x_y;
  c7_ac_x = c7_yb_x;
  c7_ab_y = c7_y_y;
  c7_S_r = c7_ac_x / c7_ab_y;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 41);
  if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c7_S_r, 1.0, -1, 2U,
        c7_S_r < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 42);
    c7_f_S = c7_S_r * (2.0 - c7_S_r);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 44);
    c7_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 46);
  c7_bc_x = c7_alpha;
  c7_cc_x = c7_bc_x;
  c7_cc_x = muDoubleScalarTan(c7_cc_x);
  c7_j_A = 20000.0 * c7_cc_x * c7_f_S;
  c7_h_B = 1.0 - c7_b_Slip;
  c7_dc_x = c7_j_A;
  c7_bb_y = c7_h_B;
  c7_ec_x = c7_dc_x;
  c7_cb_y = c7_bb_y;
  c7_fc_x = c7_ec_x;
  c7_db_y = c7_cb_y;
  c7_b_F_SL_r = c7_fc_x / c7_db_y;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 47);
  c7_k_A = 20000.0 * c7_b_Slip * c7_f_S;
  c7_i_B = 1.0 - c7_b_Slip;
  c7_gc_x = c7_k_A;
  c7_eb_y = c7_i_B;
  c7_hc_x = c7_gc_x;
  c7_fb_y = c7_eb_y;
  c7_ic_x = c7_hc_x;
  c7_gb_y = c7_fb_y;
  c7_F_tL_r = c7_ic_x / c7_gb_y;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 50);
  c7_alpha = c7_b_a_mL;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 52);
  c7_l_A = c7_Fz;
  c7_jc_x = c7_l_A;
  c7_kc_x = c7_jc_x;
  c7_lc_x = c7_kc_x;
  c7_hb_y = c7_lc_x / 3.0;
  c7_mc_x = c7_alpha;
  c7_nc_x = c7_mc_x;
  c7_nc_x = muDoubleScalarTan(c7_nc_x);
  c7_oc_x = c7_alpha;
  c7_pc_x = c7_oc_x;
  c7_pc_x = muDoubleScalarTan(c7_pc_x);
  c7_qc_x = c7_b_Slip * c7_b_Slip + c7_nc_x * c7_pc_x;
  c7_rc_x = c7_qc_x;
  if (c7_rc_x < 0.0) {
    c7_eml_error(chartInstance);
  }

  c7_rc_x = muDoubleScalarSqrt(c7_rc_x);
  c7_sc_x = c7_alpha;
  c7_tc_x = c7_sc_x;
  c7_tc_x = muDoubleScalarTan(c7_tc_x);
  c7_uc_x = c7_alpha;
  c7_vc_x = c7_uc_x;
  c7_vc_x = muDoubleScalarTan(c7_vc_x);
  c7_wc_x = 4.0E+8 * c7_b_Slip * c7_b_Slip + 4.0E+8 * c7_tc_x * c7_vc_x;
  c7_xc_x = c7_wc_x;
  if (c7_xc_x < 0.0) {
    c7_eml_error(chartInstance);
  }

  c7_xc_x = muDoubleScalarSqrt(c7_xc_x);
  c7_m_A = 0.85 * c7_hb_y * (1.0 - 0.015 * c7_U_tl * c7_rc_x) * (1.0 - c7_b_Slip);
  c7_j_B = 2.0 * c7_xc_x;
  c7_yc_x = c7_m_A;
  c7_ib_y = c7_j_B;
  c7_ad_x = c7_yc_x;
  c7_jb_y = c7_ib_y;
  c7_bd_x = c7_ad_x;
  c7_kb_y = c7_jb_y;
  c7_S_m = c7_bd_x / c7_kb_y;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 54);
  if (CV_EML_IF(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 2, c7_S_m, 1.0, -1, 2U,
        c7_S_m < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 55);
    c7_f_S = c7_S_m * (2.0 - c7_S_m);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 57);
    c7_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 59);
  c7_cd_x = c7_alpha;
  c7_dd_x = c7_cd_x;
  c7_dd_x = muDoubleScalarTan(c7_dd_x);
  c7_n_A = 20000.0 * c7_dd_x * c7_f_S;
  c7_k_B = 1.0 - c7_b_Slip;
  c7_ed_x = c7_n_A;
  c7_lb_y = c7_k_B;
  c7_fd_x = c7_ed_x;
  c7_mb_y = c7_lb_y;
  c7_gd_x = c7_fd_x;
  c7_nb_y = c7_mb_y;
  c7_b_F_SL_m = c7_gd_x / c7_nb_y;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 60);
  c7_o_A = 20000.0 * c7_b_Slip * c7_f_S;
  c7_l_B = 1.0 - c7_b_Slip;
  c7_hd_x = c7_o_A;
  c7_ob_y = c7_l_B;
  c7_id_x = c7_hd_x;
  c7_pb_y = c7_ob_y;
  c7_jd_x = c7_id_x;
  c7_qb_y = c7_pb_y;
  c7_F_tL_m = c7_jd_x / c7_qb_y;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 63);
  c7_b_F_SL = (c7_b_F_SL_f + c7_b_F_SL_r) + c7_b_F_SL_m;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 64);
  c7_b_F_tL = (c7_F_tL_f + c7_F_tL_r) + c7_F_tL_m;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 65);
  c7_F_L[0] = c7_F_tL_f;
  c7_F_L[1] = c7_F_tL_r;
  c7_F_L[2] = c7_F_tL_m;
  c7_F_L[3] = c7_b_F_SL_f;
  c7_F_L[4] = c7_b_F_SL_r;
  c7_F_L[5] = c7_b_F_SL_m;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -65);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c7_F_SL = c7_b_F_SL;
  *chartInstance->c7_F_tL = c7_b_F_tL;
  *chartInstance->c7_a_fL = c7_b_a_fL;
  *chartInstance->c7_a_rL = c7_b_a_rL;
  *chartInstance->c7_a_mL = c7_b_a_mL;
  *chartInstance->c7_F_SL_f = c7_b_F_SL_f;
  *chartInstance->c7_F_SL_r = c7_b_F_SL_r;
  *chartInstance->c7_F_SL_m = c7_b_F_SL_m;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 5U, chartInstance->c7_sfEvent);
}

static void initSimStructsc7_SS_jun_25(SFc7_SS_jun_25InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber, uint32_T c7_instanceNumber)
{
  (void)c7_machineNumber;
  (void)c7_chartNumber;
  (void)c7_instanceNumber;
}

static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  real_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc7_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(real_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static real_T c7_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_b_F_SL_m, const char_T *c7_identifier)
{
  real_T c7_y;
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_F_SL_m),
    &c7_thisId);
  sf_mex_destroy(&c7_b_F_SL_m);
  return c7_y;
}

static real_T c7_b_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  real_T c7_y;
  real_T c7_d4;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_d4, 1, 0, 0U, 0, 0U, 0);
  c7_y = c7_d4;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_b_F_SL_m;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y;
  SFc7_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc7_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c7_b_F_SL_m = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_F_SL_m),
    &c7_thisId);
  sf_mex_destroy(&c7_b_F_SL_m);
  *(real_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i0;
  real_T c7_b_inData[6];
  int32_T c7_i1;
  real_T c7_u[6];
  const mxArray *c7_y = NULL;
  SFc7_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc7_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i0 = 0; c7_i0 < 6; c7_i0++) {
    c7_b_inData[c7_i0] = (*(real_T (*)[6])c7_inData)[c7_i0];
  }

  for (c7_i1 = 0; c7_i1 < 6; c7_i1++) {
    c7_u[c7_i1] = c7_b_inData[c7_i1];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_c_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[6])
{
  real_T c7_dv0[6];
  int32_T c7_i2;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv0, 1, 0, 0U, 1, 0U, 1, 6);
  for (c7_i2 = 0; c7_i2 < 6; c7_i2++) {
    c7_y[c7_i2] = c7_dv0[c7_i2];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_F_L;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[6];
  int32_T c7_i3;
  SFc7_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc7_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c7_F_L = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_F_L), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_F_L);
  for (c7_i3 = 0; c7_i3 < 6; c7_i3++) {
    (*(real_T (*)[6])c7_outData)[c7_i3] = c7_y[c7_i3];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

const mxArray *sf_c7_SS_jun_25_get_eml_resolved_functions_info(void)
{
  const mxArray *c7_nameCaptureInfo = NULL;
  c7_nameCaptureInfo = NULL;
  sf_mex_assign(&c7_nameCaptureInfo, sf_mex_createstruct("structure", 2, 14, 1),
                false);
  c7_info_helper(&c7_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c7_nameCaptureInfo);
  return c7_nameCaptureInfo;
}

static void c7_info_helper(const mxArray **c7_info)
{
  const mxArray *c7_rhs0 = NULL;
  const mxArray *c7_lhs0 = NULL;
  const mxArray *c7_rhs1 = NULL;
  const mxArray *c7_lhs1 = NULL;
  const mxArray *c7_rhs2 = NULL;
  const mxArray *c7_lhs2 = NULL;
  const mxArray *c7_rhs3 = NULL;
  const mxArray *c7_lhs3 = NULL;
  const mxArray *c7_rhs4 = NULL;
  const mxArray *c7_lhs4 = NULL;
  const mxArray *c7_rhs5 = NULL;
  const mxArray *c7_lhs5 = NULL;
  const mxArray *c7_rhs6 = NULL;
  const mxArray *c7_lhs6 = NULL;
  const mxArray *c7_rhs7 = NULL;
  const mxArray *c7_lhs7 = NULL;
  const mxArray *c7_rhs8 = NULL;
  const mxArray *c7_lhs8 = NULL;
  const mxArray *c7_rhs9 = NULL;
  const mxArray *c7_lhs9 = NULL;
  const mxArray *c7_rhs10 = NULL;
  const mxArray *c7_lhs10 = NULL;
  const mxArray *c7_rhs11 = NULL;
  const mxArray *c7_lhs11 = NULL;
  const mxArray *c7_rhs12 = NULL;
  const mxArray *c7_lhs12 = NULL;
  const mxArray *c7_rhs13 = NULL;
  const mxArray *c7_lhs13 = NULL;
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c7_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389739374U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c7_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c7_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c7_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c7_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c7_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c7_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("atan"), "name", "name", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1395346496U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c7_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_atan"), "name",
                  "name", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840318U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c7_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("tan"), "name", "name", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/tan.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1395346504U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c7_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/tan.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_tan"), "name",
                  "name", 10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_tan.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c7_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("sqrt"), "name", "name", 11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c7_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_error"), "name", "name",
                  12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c7_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 13);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c7_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs13), "lhs", "lhs",
                  13);
  sf_mex_destroy(&c7_rhs0);
  sf_mex_destroy(&c7_lhs0);
  sf_mex_destroy(&c7_rhs1);
  sf_mex_destroy(&c7_lhs1);
  sf_mex_destroy(&c7_rhs2);
  sf_mex_destroy(&c7_lhs2);
  sf_mex_destroy(&c7_rhs3);
  sf_mex_destroy(&c7_lhs3);
  sf_mex_destroy(&c7_rhs4);
  sf_mex_destroy(&c7_lhs4);
  sf_mex_destroy(&c7_rhs5);
  sf_mex_destroy(&c7_lhs5);
  sf_mex_destroy(&c7_rhs6);
  sf_mex_destroy(&c7_lhs6);
  sf_mex_destroy(&c7_rhs7);
  sf_mex_destroy(&c7_lhs7);
  sf_mex_destroy(&c7_rhs8);
  sf_mex_destroy(&c7_lhs8);
  sf_mex_destroy(&c7_rhs9);
  sf_mex_destroy(&c7_lhs9);
  sf_mex_destroy(&c7_rhs10);
  sf_mex_destroy(&c7_lhs10);
  sf_mex_destroy(&c7_rhs11);
  sf_mex_destroy(&c7_lhs11);
  sf_mex_destroy(&c7_rhs12);
  sf_mex_destroy(&c7_lhs12);
  sf_mex_destroy(&c7_rhs13);
  sf_mex_destroy(&c7_lhs13);
}

static const mxArray *c7_emlrt_marshallOut(const char * c7_u)
{
  const mxArray *c7_y = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c7_u)), false);
  return c7_y;
}

static const mxArray *c7_b_emlrt_marshallOut(const uint32_T c7_u)
{
  const mxArray *c7_y = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 7, 0U, 0U, 0U, 0), false);
  return c7_y;
}

static real_T c7_sqrt(SFc7_SS_jun_25InstanceStruct *chartInstance, real_T c7_x)
{
  real_T c7_b_x;
  c7_b_x = c7_x;
  c7_b_sqrt(chartInstance, &c7_b_x);
  return c7_b_x;
}

static void c7_eml_error(SFc7_SS_jun_25InstanceStruct *chartInstance)
{
  int32_T c7_i4;
  static char_T c7_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c7_u[30];
  const mxArray *c7_y = NULL;
  int32_T c7_i5;
  static char_T c7_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c7_b_u[4];
  const mxArray *c7_b_y = NULL;
  (void)chartInstance;
  for (c7_i4 = 0; c7_i4 < 30; c7_i4++) {
    c7_u[c7_i4] = c7_cv0[c7_i4];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c7_i5 = 0; c7_i5 < 4; c7_i5++) {
    c7_b_u[c7_i5] = c7_cv1[c7_i5];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c7_y, 14, c7_b_y));
}

static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc7_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(int32_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static int32_T c7_d_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  int32_T c7_y;
  int32_T c7_i6;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_i6, 1, 6, 0U, 0, 0U, 0);
  c7_y = c7_i6;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_b_sfEvent;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  int32_T c7_y;
  SFc7_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc7_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c7_b_sfEvent = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_sfEvent),
    &c7_thisId);
  sf_mex_destroy(&c7_b_sfEvent);
  *(int32_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static uint8_T c7_e_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_b_is_active_c7_SS_jun_25, const char_T *c7_identifier)
{
  uint8_T c7_y;
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c7_b_is_active_c7_SS_jun_25), &c7_thisId);
  sf_mex_destroy(&c7_b_is_active_c7_SS_jun_25);
  return c7_y;
}

static uint8_T c7_f_emlrt_marshallIn(SFc7_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  uint8_T c7_y;
  uint8_T c7_u0;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_u0, 1, 3, 0U, 0, 0U, 0);
  c7_y = c7_u0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_b_sqrt(SFc7_SS_jun_25InstanceStruct *chartInstance, real_T *c7_x)
{
  if (*c7_x < 0.0) {
    c7_eml_error(chartInstance);
  }

  *c7_x = muDoubleScalarSqrt(*c7_x);
}

static void init_dsm_address_info(SFc7_SS_jun_25InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc7_SS_jun_25InstanceStruct *chartInstance)
{
  chartInstance->c7_FzL = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c7_F_SL = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c7_F_tL = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c7_U = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c7_V = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c7_Slip = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c7_delta = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c7_r = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    5);
  chartInstance->c7_a_fL = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c7_a_rL = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c7_a_mL = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c7_F_SL_f = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c7_F_SL_r = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 7);
  chartInstance->c7_F_SL_m = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 8);
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

void sf_c7_SS_jun_25_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1364293669U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2340020182U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2264142044U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2842152599U);
}

mxArray* sf_c7_SS_jun_25_get_post_codegen_info(void);
mxArray *sf_c7_SS_jun_25_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("kK5Z8dBsWnV3UyXYMwE9NC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,8,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c7_SS_jun_25_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c7_SS_jun_25_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c7_SS_jun_25_jit_fallback_info(void)
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

mxArray *sf_c7_SS_jun_25_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c7_SS_jun_25_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c7_SS_jun_25(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x9'type','srcId','name','auxInfo'{{M[1],M[5],T\"F_SL\",},{M[1],M[24],T\"F_SL_f\",},{M[1],M[26],T\"F_SL_m\",},{M[1],M[25],T\"F_SL_r\",},{M[1],M[15],T\"F_tL\",},{M[1],M[18],T\"a_fL\",},{M[1],M[21],T\"a_mL\",},{M[1],M[19],T\"a_rL\",},{M[8],M[0],T\"is_active_c7_SS_jun_25\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 9, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c7_SS_jun_25_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc7_SS_jun_25InstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc7_SS_jun_25InstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SS_jun_25MachineNumber_,
           7,
           1,
           1,
           0,
           14,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"FzL");
          _SFD_SET_DATA_PROPS(1,2,0,1,"F_SL");
          _SFD_SET_DATA_PROPS(2,2,0,1,"F_tL");
          _SFD_SET_DATA_PROPS(3,1,1,0,"U");
          _SFD_SET_DATA_PROPS(4,1,1,0,"V");
          _SFD_SET_DATA_PROPS(5,1,1,0,"Slip");
          _SFD_SET_DATA_PROPS(6,1,1,0,"delta");
          _SFD_SET_DATA_PROPS(7,1,1,0,"r");
          _SFD_SET_DATA_PROPS(8,2,0,1,"a_fL");
          _SFD_SET_DATA_PROPS(9,2,0,1,"a_rL");
          _SFD_SET_DATA_PROPS(10,2,0,1,"a_mL");
          _SFD_SET_DATA_PROPS(11,2,0,1,"F_SL_f");
          _SFD_SET_DATA_PROPS(12,2,0,1,"F_SL_r");
          _SFD_SET_DATA_PROPS(13,2,0,1,"F_SL_m");
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
        _SFD_CV_INIT_EML(0,1,1,3,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1714);
        _SFD_CV_INIT_EML_IF(0,1,0,894,903,925,944);
        _SFD_CV_INIT_EML_IF(0,1,1,1190,1199,1221,1240);
        _SFD_CV_INIT_EML_IF(0,1,2,1488,1497,1519,1538);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,897,902,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,1193,1198,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,1491,1496,-1,2);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)c7_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)c7_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)c7_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)c7_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)c7_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)c7_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)c7_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)c7_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c7_FzL);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c7_F_SL);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c7_F_tL);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c7_U);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c7_V);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c7_Slip);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c7_delta);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c7_r);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c7_a_fL);
        _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c7_a_rL);
        _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c7_a_mL);
        _SFD_SET_DATA_VALUE_PTR(11U, chartInstance->c7_F_SL_f);
        _SFD_SET_DATA_VALUE_PTR(12U, chartInstance->c7_F_SL_r);
        _SFD_SET_DATA_VALUE_PTR(13U, chartInstance->c7_F_SL_m);
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
  return "Ka1sGGdG7ZSC8JaFcFAPHH";
}

static void sf_opaque_initialize_c7_SS_jun_25(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc7_SS_jun_25InstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c7_SS_jun_25((SFc7_SS_jun_25InstanceStruct*)
    chartInstanceVar);
  initialize_c7_SS_jun_25((SFc7_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c7_SS_jun_25(void *chartInstanceVar)
{
  enable_c7_SS_jun_25((SFc7_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c7_SS_jun_25(void *chartInstanceVar)
{
  disable_c7_SS_jun_25((SFc7_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c7_SS_jun_25(void *chartInstanceVar)
{
  sf_gateway_c7_SS_jun_25((SFc7_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c7_SS_jun_25(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c7_SS_jun_25((SFc7_SS_jun_25InstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c7_SS_jun_25(SimStruct* S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c7_SS_jun_25((SFc7_SS_jun_25InstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c7_SS_jun_25(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc7_SS_jun_25InstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SS_jun_25_optimization_info();
    }

    finalize_c7_SS_jun_25((SFc7_SS_jun_25InstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc7_SS_jun_25((SFc7_SS_jun_25InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c7_SS_jun_25(SimStruct *S)
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
    initialize_params_c7_SS_jun_25((SFc7_SS_jun_25InstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c7_SS_jun_25(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SS_jun_25_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,7);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,7,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,7,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,7);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,7,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,7,8);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=8; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,7);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3842713562U));
  ssSetChecksum1(S,(1698170374U));
  ssSetChecksum2(S,(1519212192U));
  ssSetChecksum3(S,(4160879712U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c7_SS_jun_25(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c7_SS_jun_25(SimStruct *S)
{
  SFc7_SS_jun_25InstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc7_SS_jun_25InstanceStruct *)utMalloc(sizeof
    (SFc7_SS_jun_25InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc7_SS_jun_25InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c7_SS_jun_25;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c7_SS_jun_25;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c7_SS_jun_25;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c7_SS_jun_25;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c7_SS_jun_25;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c7_SS_jun_25;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c7_SS_jun_25;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c7_SS_jun_25;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c7_SS_jun_25;
  chartInstance->chartInfo.mdlStart = mdlStart_c7_SS_jun_25;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c7_SS_jun_25;
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

void c7_SS_jun_25_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c7_SS_jun_25(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c7_SS_jun_25(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c7_SS_jun_25(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c7_SS_jun_25_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
