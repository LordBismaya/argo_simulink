/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SS_jun_25_sfun.h"
#include "c3_SS_jun_25.h"
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
static const char * c3_debug_family_names[66] = { "M", "Ms", "Izz", "Ixx", "Ixz",
  "a", "b", "hcg", "e", "Tw", "R", "I_t", "I_e", "A_f", "K_bf", "zi1", "z2",
  "zi3", "zi4", "zi5", "Lbpsi", "Lbp", "C_a", "C_i", "mu", "e_r", "K_rsf",
  "K_rsr", "C1", "C2", "C3", "C_s1", "C_d", "d", "n", "rho", "g", "c", "Fz",
  "alpha", "U_tl", "U_tr", "S_f", "f_S", "F_tR_f", "S_r", "F_tR_r", "S_m",
  "F_tR_m", "F_R", "nargin", "nargout", "FzR", "U", "V", "Slip", "delta", "r",
  "F_SR", "F_tR", "a_fR", "a_rR", "a_mR", "F_SR_f", "F_SR_r", "F_SR_m" };

/* Function Declarations */
static void initialize_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance);
static void initialize_params_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance);
static void enable_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance);
static void disable_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance);
static void c3_update_debugger_state_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance);
static void set_sim_state_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance, const mxArray *c3_st);
static void finalize_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance);
static void sf_gateway_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance);
static void mdl_start_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance);
static void c3_chartstep_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance);
static void initSimStructsc3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static real_T c3_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_b_F_SR_m, const char_T *c3_identifier);
static real_T c3_b_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_c_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId, real_T c3_y[6]);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static void c3_info_helper(const mxArray **c3_info);
static const mxArray *c3_emlrt_marshallOut(const char * c3_u);
static const mxArray *c3_b_emlrt_marshallOut(const uint32_T c3_u);
static real_T c3_sqrt(SFc3_SS_jun_25InstanceStruct *chartInstance, real_T c3_x);
static void c3_eml_error(SFc3_SS_jun_25InstanceStruct *chartInstance);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_d_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint8_T c3_e_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_b_is_active_c3_SS_jun_25, const char_T *c3_identifier);
static uint8_T c3_f_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_sqrt(SFc3_SS_jun_25InstanceStruct *chartInstance, real_T *c3_x);
static void init_dsm_address_info(SFc3_SS_jun_25InstanceStruct *chartInstance);
static void init_simulink_io_address(SFc3_SS_jun_25InstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c3_is_active_c3_SS_jun_25 = 0U;
}

static void initialize_params_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c3_update_debugger_state_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  real_T c3_hoistedGlobal;
  real_T c3_u;
  const mxArray *c3_b_y = NULL;
  real_T c3_b_hoistedGlobal;
  real_T c3_b_u;
  const mxArray *c3_c_y = NULL;
  real_T c3_c_hoistedGlobal;
  real_T c3_c_u;
  const mxArray *c3_d_y = NULL;
  real_T c3_d_hoistedGlobal;
  real_T c3_d_u;
  const mxArray *c3_e_y = NULL;
  real_T c3_e_hoistedGlobal;
  real_T c3_e_u;
  const mxArray *c3_f_y = NULL;
  real_T c3_f_hoistedGlobal;
  real_T c3_f_u;
  const mxArray *c3_g_y = NULL;
  real_T c3_g_hoistedGlobal;
  real_T c3_g_u;
  const mxArray *c3_h_y = NULL;
  real_T c3_h_hoistedGlobal;
  real_T c3_h_u;
  const mxArray *c3_i_y = NULL;
  uint8_T c3_i_hoistedGlobal;
  uint8_T c3_i_u;
  const mxArray *c3_j_y = NULL;
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellmatrix(9, 1), false);
  c3_hoistedGlobal = *chartInstance->c3_F_SR;
  c3_u = c3_hoistedGlobal;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_b_hoistedGlobal = *chartInstance->c3_F_SR_f;
  c3_b_u = c3_b_hoistedGlobal;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 1, c3_c_y);
  c3_c_hoistedGlobal = *chartInstance->c3_F_SR_m;
  c3_c_u = c3_c_hoistedGlobal;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 2, c3_d_y);
  c3_d_hoistedGlobal = *chartInstance->c3_F_SR_r;
  c3_d_u = c3_d_hoistedGlobal;
  c3_e_y = NULL;
  sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 3, c3_e_y);
  c3_e_hoistedGlobal = *chartInstance->c3_F_tR;
  c3_e_u = c3_e_hoistedGlobal;
  c3_f_y = NULL;
  sf_mex_assign(&c3_f_y, sf_mex_create("y", &c3_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 4, c3_f_y);
  c3_f_hoistedGlobal = *chartInstance->c3_a_fR;
  c3_f_u = c3_f_hoistedGlobal;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 5, c3_g_y);
  c3_g_hoistedGlobal = *chartInstance->c3_a_mR;
  c3_g_u = c3_g_hoistedGlobal;
  c3_h_y = NULL;
  sf_mex_assign(&c3_h_y, sf_mex_create("y", &c3_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 6, c3_h_y);
  c3_h_hoistedGlobal = *chartInstance->c3_a_rR;
  c3_h_u = c3_h_hoistedGlobal;
  c3_i_y = NULL;
  sf_mex_assign(&c3_i_y, sf_mex_create("y", &c3_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 7, c3_i_y);
  c3_i_hoistedGlobal = chartInstance->c3_is_active_c3_SS_jun_25;
  c3_i_u = c3_i_hoistedGlobal;
  c3_j_y = NULL;
  sf_mex_assign(&c3_j_y, sf_mex_create("y", &c3_i_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 8, c3_j_y);
  sf_mex_assign(&c3_st, c3_y, false);
  return c3_st;
}

static void set_sim_state_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  chartInstance->c3_doneDoubleBufferReInit = true;
  c3_u = sf_mex_dup(c3_st);
  *chartInstance->c3_F_SR = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 0)), "F_SR");
  *chartInstance->c3_F_SR_f = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 1)), "F_SR_f");
  *chartInstance->c3_F_SR_m = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 2)), "F_SR_m");
  *chartInstance->c3_F_SR_r = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 3)), "F_SR_r");
  *chartInstance->c3_F_tR = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 4)), "F_tR");
  *chartInstance->c3_a_fR = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 5)), "a_fR");
  *chartInstance->c3_a_mR = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 6)), "a_mR");
  *chartInstance->c3_a_rR = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 7)), "a_rR");
  chartInstance->c3_is_active_c3_SS_jun_25 = c3_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c3_u, 8)), "is_active_c3_SS_jun_25");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_SS_jun_25(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance)
{
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c3_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_FzR, 0U);
  chartInstance->c3_sfEvent = CALL_EVENT;
  c3_chartstep_c3_SS_jun_25(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SS_jun_25MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_F_SR, 1U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_F_tR, 2U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_U, 3U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_V, 4U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_Slip, 5U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_delta, 6U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_r, 7U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_a_fR, 8U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_a_rR, 9U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_a_mR, 10U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_F_SR_f, 11U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_F_SR_r, 12U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_F_SR_m, 13U);
}

static void mdl_start_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_chartstep_c3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance)
{
  real_T c3_hoistedGlobal;
  real_T c3_b_hoistedGlobal;
  real_T c3_c_hoistedGlobal;
  real_T c3_d_hoistedGlobal;
  real_T c3_e_hoistedGlobal;
  real_T c3_f_hoistedGlobal;
  real_T c3_b_FzR;
  real_T c3_b_U;
  real_T c3_b_V;
  real_T c3_b_Slip;
  real_T c3_b_delta;
  real_T c3_b_r;
  uint32_T c3_debug_family_var_map[66];
  real_T c3_M;
  real_T c3_Ms;
  real_T c3_Izz;
  real_T c3_Ixx;
  real_T c3_Ixz;
  real_T c3_a;
  real_T c3_b;
  real_T c3_hcg;
  real_T c3_e;
  real_T c3_Tw;
  real_T c3_R;
  real_T c3_I_t;
  real_T c3_I_e;
  real_T c3_A_f;
  real_T c3_K_bf;
  real_T c3_zi1;
  real_T c3_z2;
  real_T c3_zi3;
  real_T c3_zi4;
  real_T c3_zi5;
  real_T c3_Lbpsi;
  real_T c3_Lbp;
  real_T c3_C_a;
  real_T c3_C_i;
  real_T c3_mu;
  real_T c3_e_r;
  real_T c3_K_rsf;
  real_T c3_K_rsr;
  real_T c3_C1;
  real_T c3_C2;
  real_T c3_C3;
  real_T c3_C_s1;
  real_T c3_C_d;
  real_T c3_d;
  real_T c3_n;
  real_T c3_rho;
  real_T c3_g;
  real_T c3_c;
  real_T c3_Fz;
  real_T c3_alpha;
  real_T c3_U_tl;
  real_T c3_U_tr;
  real_T c3_S_f;
  real_T c3_f_S;
  real_T c3_F_tR_f;
  real_T c3_S_r;
  real_T c3_F_tR_r;
  real_T c3_S_m;
  real_T c3_F_tR_m;
  real_T c3_F_R[6];
  real_T c3_nargin = 6.0;
  real_T c3_nargout = 8.0;
  real_T c3_b_F_SR;
  real_T c3_b_F_tR;
  real_T c3_b_a_fR;
  real_T c3_b_a_rR;
  real_T c3_b_a_mR;
  real_T c3_b_F_SR_f;
  real_T c3_b_F_SR_r;
  real_T c3_b_F_SR_m;
  real_T c3_A;
  real_T c3_B;
  real_T c3_x;
  real_T c3_y;
  real_T c3_b_x;
  real_T c3_b_y;
  real_T c3_c_x;
  real_T c3_c_y;
  real_T c3_d_y;
  real_T c3_d_x;
  real_T c3_e_x;
  real_T c3_b_A;
  real_T c3_b_B;
  real_T c3_f_x;
  real_T c3_e_y;
  real_T c3_g_x;
  real_T c3_f_y;
  real_T c3_h_x;
  real_T c3_g_y;
  real_T c3_h_y;
  real_T c3_i_x;
  real_T c3_j_x;
  real_T c3_c_A;
  real_T c3_c_B;
  real_T c3_k_x;
  real_T c3_i_y;
  real_T c3_l_x;
  real_T c3_j_y;
  real_T c3_m_x;
  real_T c3_k_y;
  real_T c3_l_y;
  real_T c3_n_x;
  real_T c3_o_x;
  real_T c3_d_A;
  real_T c3_p_x;
  real_T c3_q_x;
  real_T c3_r_x;
  real_T c3_m_y;
  real_T c3_s_x;
  real_T c3_t_x;
  real_T c3_u_x;
  real_T c3_v_x;
  real_T c3_w_x;
  real_T c3_x_x;
  real_T c3_y_x;
  real_T c3_ab_x;
  real_T c3_d0;
  real_T c3_e_A;
  real_T c3_d1;
  real_T c3_d_B;
  real_T c3_bb_x;
  real_T c3_n_y;
  real_T c3_cb_x;
  real_T c3_o_y;
  real_T c3_db_x;
  real_T c3_p_y;
  real_T c3_eb_x;
  real_T c3_fb_x;
  real_T c3_f_A;
  real_T c3_e_B;
  real_T c3_gb_x;
  real_T c3_q_y;
  real_T c3_hb_x;
  real_T c3_r_y;
  real_T c3_ib_x;
  real_T c3_s_y;
  real_T c3_g_A;
  real_T c3_f_B;
  real_T c3_jb_x;
  real_T c3_t_y;
  real_T c3_kb_x;
  real_T c3_u_y;
  real_T c3_lb_x;
  real_T c3_v_y;
  real_T c3_h_A;
  real_T c3_mb_x;
  real_T c3_nb_x;
  real_T c3_ob_x;
  real_T c3_w_y;
  real_T c3_pb_x;
  real_T c3_qb_x;
  real_T c3_rb_x;
  real_T c3_sb_x;
  real_T c3_tb_x;
  real_T c3_ub_x;
  real_T c3_vb_x;
  real_T c3_wb_x;
  real_T c3_d2;
  real_T c3_i_A;
  real_T c3_d3;
  real_T c3_g_B;
  real_T c3_xb_x;
  real_T c3_x_y;
  real_T c3_yb_x;
  real_T c3_y_y;
  real_T c3_ac_x;
  real_T c3_ab_y;
  real_T c3_bc_x;
  real_T c3_cc_x;
  real_T c3_j_A;
  real_T c3_h_B;
  real_T c3_dc_x;
  real_T c3_bb_y;
  real_T c3_ec_x;
  real_T c3_cb_y;
  real_T c3_fc_x;
  real_T c3_db_y;
  real_T c3_k_A;
  real_T c3_i_B;
  real_T c3_gc_x;
  real_T c3_eb_y;
  real_T c3_hc_x;
  real_T c3_fb_y;
  real_T c3_ic_x;
  real_T c3_gb_y;
  real_T c3_l_A;
  real_T c3_jc_x;
  real_T c3_kc_x;
  real_T c3_lc_x;
  real_T c3_hb_y;
  real_T c3_mc_x;
  real_T c3_nc_x;
  real_T c3_oc_x;
  real_T c3_pc_x;
  real_T c3_qc_x;
  real_T c3_rc_x;
  real_T c3_sc_x;
  real_T c3_tc_x;
  real_T c3_uc_x;
  real_T c3_vc_x;
  real_T c3_wc_x;
  real_T c3_xc_x;
  real_T c3_m_A;
  real_T c3_j_B;
  real_T c3_yc_x;
  real_T c3_ib_y;
  real_T c3_ad_x;
  real_T c3_jb_y;
  real_T c3_bd_x;
  real_T c3_kb_y;
  real_T c3_cd_x;
  real_T c3_dd_x;
  real_T c3_n_A;
  real_T c3_k_B;
  real_T c3_ed_x;
  real_T c3_lb_y;
  real_T c3_fd_x;
  real_T c3_mb_y;
  real_T c3_gd_x;
  real_T c3_nb_y;
  real_T c3_o_A;
  real_T c3_l_B;
  real_T c3_hd_x;
  real_T c3_ob_y;
  real_T c3_id_x;
  real_T c3_pb_y;
  real_T c3_jd_x;
  real_T c3_qb_y;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c3_sfEvent);
  c3_hoistedGlobal = *chartInstance->c3_FzR;
  c3_b_hoistedGlobal = *chartInstance->c3_U;
  c3_c_hoistedGlobal = *chartInstance->c3_V;
  c3_d_hoistedGlobal = *chartInstance->c3_Slip;
  c3_e_hoistedGlobal = *chartInstance->c3_delta;
  c3_f_hoistedGlobal = *chartInstance->c3_r;
  c3_b_FzR = c3_hoistedGlobal;
  c3_b_U = c3_b_hoistedGlobal;
  c3_b_V = c3_c_hoistedGlobal;
  c3_b_Slip = c3_d_hoistedGlobal;
  c3_b_delta = c3_e_hoistedGlobal;
  c3_b_r = c3_f_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 66U, 66U, c3_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_M, 0U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Ms, 1U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Izz, 2U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Ixx, 3U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Ixz, 4U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_a, 5U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b, 6U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_hcg, 7U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_e, 8U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_Tw, 9U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_R, 10U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_I_t, 11U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_I_e, 12U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_A_f, 13U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_K_bf, 14U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_zi1, 15U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_z2, 16U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_zi3, 17U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_zi4, 18U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_zi5, 19U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Lbpsi, 20U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Lbp, 21U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_C_a, 22U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_C_i, 23U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_mu, 24U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_e_r, 25U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_K_rsf, 26U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_K_rsr, 27U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_C1, 28U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_C2, 29U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_C3, 30U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_C_s1, 31U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_C_d, 32U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_d, 33U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_n, 34U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_rho, 35U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_g, 36U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_c, 37U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Fz, 38U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_alpha, 39U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_U_tl, 40U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_U_tr, 41U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_S_f, 42U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_f_S, 43U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_F_tR_f, 44U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_S_r, 45U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_F_tR_r, 46U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_S_m, 47U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_F_tR_m, 48U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_F_R, 49U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 50U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 51U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b_FzR, 52U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b_U, 53U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b_V, 54U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b_Slip, 55U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b_delta, 56U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b_r, 57U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_F_SR, 58U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_F_tR, 59U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_a_fR, 60U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_a_rR, 61U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_a_mR, 62U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_F_SR_f, 63U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_F_SR_r, 64U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_F_SR_m, 65U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_M = 1280.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_Ms = 1160.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_Izz = 2500.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_Ixx = 750.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_Ixz = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_a = 1.203;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_b = 1.217;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_hcg = 0.5;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_e = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_Tw = 1.33;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_R = 0.3;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  c3_I_t = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 3);
  c3_I_e = 0.136;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 3);
  c3_A_f = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 3);
  c3_K_bf = 0.55;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 3);
  c3_zi1 = 13.56;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 3);
  c3_z2 = 7.5;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 3);
  c3_zi3 = 5.37;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 3);
  c3_zi4 = 4.22;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 3);
  c3_zi5 = 3.28;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_Lbpsi = 45000.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_Lbp = 2600.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_C_a = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_C_i = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_mu = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_e_r = 0.015;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_K_rsf = -0.05;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_K_rsr = 0.1;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  c3_C1 = -6.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  c3_C2 = 59.16;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  c3_C3 = 25.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  c3_C_s1 = 1.38;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  c3_K_rsf = 0.444;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  c3_C_d = 0.32;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  c3_d = 0.014;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  c3_n = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 6);
  c3_rho = 1.204;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 6);
  c3_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 6);
  c3_c = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 10);
  c3_Fz = c3_b_FzR;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 17);
  c3_A = c3_b_V + 1.203 * c3_b_r;
  c3_B = c3_b_U - 0.665 * c3_b_r;
  c3_x = c3_A;
  c3_y = c3_B;
  c3_b_x = c3_x;
  c3_b_y = c3_y;
  c3_c_x = c3_b_x;
  c3_c_y = c3_b_y;
  c3_d_y = c3_c_x / c3_c_y;
  c3_d_x = c3_d_y;
  c3_e_x = c3_d_x;
  c3_e_x = muDoubleScalarAtan(c3_e_x);
  c3_b_a_fR = -c3_e_x;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 18);
  c3_b_A = 1.217 * c3_b_r - c3_b_V;
  c3_b_B = c3_b_U - 0.665 * c3_b_r;
  c3_f_x = c3_b_A;
  c3_e_y = c3_b_B;
  c3_g_x = c3_f_x;
  c3_f_y = c3_e_y;
  c3_h_x = c3_g_x;
  c3_g_y = c3_f_y;
  c3_h_y = c3_h_x / c3_g_y;
  c3_i_x = c3_h_y;
  c3_j_x = c3_i_x;
  c3_j_x = muDoubleScalarAtan(c3_j_x);
  c3_b_a_rR = -c3_j_x;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 19);
  c3_c_A = 0.2 * c3_b_r - c3_b_V;
  c3_c_B = c3_b_U - 0.665 * c3_b_r;
  c3_k_x = c3_c_A;
  c3_i_y = c3_c_B;
  c3_l_x = c3_k_x;
  c3_j_y = c3_i_y;
  c3_m_x = c3_l_x;
  c3_k_y = c3_j_y;
  c3_l_y = c3_m_x / c3_k_y;
  c3_n_x = c3_l_y;
  c3_o_x = c3_n_x;
  c3_o_x = muDoubleScalarAtan(c3_o_x);
  c3_b_a_mR = -c3_o_x;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 21);
  c3_alpha = c3_b_a_fR;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 23);
  c3_U_tl = c3_b_U + 0.665 * c3_b_r;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 24);
  c3_U_tr = c3_b_U - 0.665 * c3_b_r;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 27);
  c3_d_A = c3_Fz;
  c3_p_x = c3_d_A;
  c3_q_x = c3_p_x;
  c3_r_x = c3_q_x;
  c3_m_y = c3_r_x / 3.0;
  c3_s_x = c3_alpha;
  c3_t_x = c3_s_x;
  c3_t_x = muDoubleScalarTan(c3_t_x);
  c3_u_x = c3_alpha;
  c3_v_x = c3_u_x;
  c3_v_x = muDoubleScalarTan(c3_v_x);
  c3_w_x = c3_alpha;
  c3_x_x = c3_w_x;
  c3_x_x = muDoubleScalarTan(c3_x_x);
  c3_y_x = c3_alpha;
  c3_ab_x = c3_y_x;
  c3_ab_x = muDoubleScalarTan(c3_ab_x);
  c3_d0 = c3_b_Slip * c3_b_Slip + c3_t_x * c3_v_x;
  c3_b_sqrt(chartInstance, &c3_d0);
  c3_e_A = 0.85 * c3_m_y * (1.0 - 0.015 * c3_U_tr * c3_d0) * (1.0 - c3_b_Slip);
  c3_d1 = 4.0E+8 * c3_b_Slip * c3_b_Slip + 4.0E+8 * c3_x_x * c3_ab_x;
  c3_b_sqrt(chartInstance, &c3_d1);
  c3_d_B = 2.0 * c3_d1;
  c3_bb_x = c3_e_A;
  c3_n_y = c3_d_B;
  c3_cb_x = c3_bb_x;
  c3_o_y = c3_n_y;
  c3_db_x = c3_cb_x;
  c3_p_y = c3_o_y;
  c3_S_f = c3_db_x / c3_p_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 29);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c3_S_f, 1.0, -1, 2U,
        c3_S_f < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 30);
    c3_f_S = c3_S_f * (2.0 - c3_S_f);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 32);
    c3_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 34);
  c3_eb_x = c3_alpha;
  c3_fb_x = c3_eb_x;
  c3_fb_x = muDoubleScalarTan(c3_fb_x);
  c3_f_A = 20000.0 * c3_fb_x * c3_f_S;
  c3_e_B = 1.0 - c3_b_Slip;
  c3_gb_x = c3_f_A;
  c3_q_y = c3_e_B;
  c3_hb_x = c3_gb_x;
  c3_r_y = c3_q_y;
  c3_ib_x = c3_hb_x;
  c3_s_y = c3_r_y;
  c3_b_F_SR_f = c3_ib_x / c3_s_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 35);
  c3_g_A = 20000.0 * c3_b_Slip * c3_f_S;
  c3_f_B = 1.0 - c3_b_Slip;
  c3_jb_x = c3_g_A;
  c3_t_y = c3_f_B;
  c3_kb_x = c3_jb_x;
  c3_u_y = c3_t_y;
  c3_lb_x = c3_kb_x;
  c3_v_y = c3_u_y;
  c3_F_tR_f = c3_lb_x / c3_v_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 38);
  c3_alpha = c3_b_a_rR;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 40);
  c3_h_A = c3_Fz;
  c3_mb_x = c3_h_A;
  c3_nb_x = c3_mb_x;
  c3_ob_x = c3_nb_x;
  c3_w_y = c3_ob_x / 3.0;
  c3_pb_x = c3_alpha;
  c3_qb_x = c3_pb_x;
  c3_qb_x = muDoubleScalarTan(c3_qb_x);
  c3_rb_x = c3_alpha;
  c3_sb_x = c3_rb_x;
  c3_sb_x = muDoubleScalarTan(c3_sb_x);
  c3_tb_x = c3_alpha;
  c3_ub_x = c3_tb_x;
  c3_ub_x = muDoubleScalarTan(c3_ub_x);
  c3_vb_x = c3_alpha;
  c3_wb_x = c3_vb_x;
  c3_wb_x = muDoubleScalarTan(c3_wb_x);
  c3_d2 = c3_b_Slip * c3_b_Slip + c3_qb_x * c3_sb_x;
  c3_b_sqrt(chartInstance, &c3_d2);
  c3_i_A = 0.85 * c3_w_y * (1.0 - 0.015 * c3_U_tr * c3_d2) * (1.0 - c3_b_Slip);
  c3_d3 = 4.0E+8 * c3_b_Slip * c3_b_Slip + 4.0E+8 * c3_ub_x * c3_wb_x;
  c3_b_sqrt(chartInstance, &c3_d3);
  c3_g_B = 2.0 * c3_d3;
  c3_xb_x = c3_i_A;
  c3_x_y = c3_g_B;
  c3_yb_x = c3_xb_x;
  c3_y_y = c3_x_y;
  c3_ac_x = c3_yb_x;
  c3_ab_y = c3_y_y;
  c3_S_r = c3_ac_x / c3_ab_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 42);
  if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c3_S_r, 1.0, -1, 2U,
        c3_S_r < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 43);
    c3_f_S = c3_S_r * (2.0 - c3_S_r);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 45);
    c3_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 47);
  c3_bc_x = c3_alpha;
  c3_cc_x = c3_bc_x;
  c3_cc_x = muDoubleScalarTan(c3_cc_x);
  c3_j_A = 20000.0 * c3_cc_x * c3_f_S;
  c3_h_B = 1.0 - c3_b_Slip;
  c3_dc_x = c3_j_A;
  c3_bb_y = c3_h_B;
  c3_ec_x = c3_dc_x;
  c3_cb_y = c3_bb_y;
  c3_fc_x = c3_ec_x;
  c3_db_y = c3_cb_y;
  c3_b_F_SR_r = c3_fc_x / c3_db_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 48);
  c3_k_A = 20000.0 * c3_b_Slip * c3_f_S;
  c3_i_B = 1.0 - c3_b_Slip;
  c3_gc_x = c3_k_A;
  c3_eb_y = c3_i_B;
  c3_hc_x = c3_gc_x;
  c3_fb_y = c3_eb_y;
  c3_ic_x = c3_hc_x;
  c3_gb_y = c3_fb_y;
  c3_F_tR_r = c3_ic_x / c3_gb_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 51);
  c3_alpha = c3_b_a_mR;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 53);
  c3_l_A = c3_Fz;
  c3_jc_x = c3_l_A;
  c3_kc_x = c3_jc_x;
  c3_lc_x = c3_kc_x;
  c3_hb_y = c3_lc_x / 3.0;
  c3_mc_x = c3_alpha;
  c3_nc_x = c3_mc_x;
  c3_nc_x = muDoubleScalarTan(c3_nc_x);
  c3_oc_x = c3_alpha;
  c3_pc_x = c3_oc_x;
  c3_pc_x = muDoubleScalarTan(c3_pc_x);
  c3_qc_x = c3_b_Slip * c3_b_Slip + c3_nc_x * c3_pc_x;
  c3_rc_x = c3_qc_x;
  if (c3_rc_x < 0.0) {
    c3_eml_error(chartInstance);
  }

  c3_rc_x = muDoubleScalarSqrt(c3_rc_x);
  c3_sc_x = c3_alpha;
  c3_tc_x = c3_sc_x;
  c3_tc_x = muDoubleScalarTan(c3_tc_x);
  c3_uc_x = c3_alpha;
  c3_vc_x = c3_uc_x;
  c3_vc_x = muDoubleScalarTan(c3_vc_x);
  c3_wc_x = 4.0E+8 * c3_b_Slip * c3_b_Slip + 4.0E+8 * c3_tc_x * c3_vc_x;
  c3_xc_x = c3_wc_x;
  if (c3_xc_x < 0.0) {
    c3_eml_error(chartInstance);
  }

  c3_xc_x = muDoubleScalarSqrt(c3_xc_x);
  c3_m_A = 0.85 * c3_hb_y * (1.0 - 0.015 * c3_U_tr * c3_rc_x) * (1.0 - c3_b_Slip);
  c3_j_B = 2.0 * c3_xc_x;
  c3_yc_x = c3_m_A;
  c3_ib_y = c3_j_B;
  c3_ad_x = c3_yc_x;
  c3_jb_y = c3_ib_y;
  c3_bd_x = c3_ad_x;
  c3_kb_y = c3_jb_y;
  c3_S_m = c3_bd_x / c3_kb_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 55);
  if (CV_EML_IF(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 2, c3_S_m, 1.0, -1, 2U,
        c3_S_m < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 56);
    c3_f_S = c3_S_m * (2.0 - c3_S_m);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 58);
    c3_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 60);
  c3_cd_x = c3_alpha;
  c3_dd_x = c3_cd_x;
  c3_dd_x = muDoubleScalarTan(c3_dd_x);
  c3_n_A = 20000.0 * c3_dd_x * c3_f_S;
  c3_k_B = 1.0 - c3_b_Slip;
  c3_ed_x = c3_n_A;
  c3_lb_y = c3_k_B;
  c3_fd_x = c3_ed_x;
  c3_mb_y = c3_lb_y;
  c3_gd_x = c3_fd_x;
  c3_nb_y = c3_mb_y;
  c3_b_F_SR_m = c3_gd_x / c3_nb_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 61);
  c3_o_A = 20000.0 * c3_b_Slip * c3_f_S;
  c3_l_B = 1.0 - c3_b_Slip;
  c3_hd_x = c3_o_A;
  c3_ob_y = c3_l_B;
  c3_id_x = c3_hd_x;
  c3_pb_y = c3_ob_y;
  c3_jd_x = c3_id_x;
  c3_qb_y = c3_pb_y;
  c3_F_tR_m = c3_jd_x / c3_qb_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 64);
  c3_b_F_SR = (c3_b_F_SR_f + c3_b_F_SR_r) + c3_b_F_SR_m;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 65);
  c3_b_F_tR = (c3_F_tR_f + c3_F_tR_r) + c3_F_tR_m;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 66);
  c3_F_R[0] = c3_F_tR_f;
  c3_F_R[1] = c3_F_tR_r;
  c3_F_R[2] = c3_F_tR_m;
  c3_F_R[3] = c3_b_F_SR_f;
  c3_F_R[4] = c3_b_F_SR_r;
  c3_F_R[5] = c3_b_F_SR_m;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -66);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c3_F_SR = c3_b_F_SR;
  *chartInstance->c3_F_tR = c3_b_F_tR;
  *chartInstance->c3_a_fR = c3_b_a_fR;
  *chartInstance->c3_a_rR = c3_b_a_rR;
  *chartInstance->c3_a_mR = c3_b_a_mR;
  *chartInstance->c3_F_SR_f = c3_b_F_SR_f;
  *chartInstance->c3_F_SR_r = c3_b_F_SR_r;
  *chartInstance->c3_F_SR_m = c3_b_F_SR_m;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c3_sfEvent);
}

static void initSimStructsc3_SS_jun_25(SFc3_SS_jun_25InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber)
{
  (void)c3_machineNumber;
  (void)c3_chartNumber;
  (void)c3_instanceNumber;
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc3_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static real_T c3_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_b_F_SR_m, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_F_SR_m),
    &c3_thisId);
  sf_mex_destroy(&c3_b_F_SR_m);
  return c3_y;
}

static real_T c3_b_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d4;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d4, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d4;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_F_SR_m;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc3_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c3_b_F_SR_m = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_F_SR_m),
    &c3_thisId);
  sf_mex_destroy(&c3_b_F_SR_m);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i0;
  real_T c3_b_inData[6];
  int32_T c3_i1;
  real_T c3_u[6];
  const mxArray *c3_y = NULL;
  SFc3_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc3_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i0 = 0; c3_i0 < 6; c3_i0++) {
    c3_b_inData[c3_i0] = (*(real_T (*)[6])c3_inData)[c3_i0];
  }

  for (c3_i1 = 0; c3_i1 < 6; c3_i1++) {
    c3_u[c3_i1] = c3_b_inData[c3_i1];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_c_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId, real_T c3_y[6])
{
  real_T c3_dv0[6];
  int32_T c3_i2;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv0, 1, 0, 0U, 1, 0U, 1, 6);
  for (c3_i2 = 0; c3_i2 < 6; c3_i2++) {
    c3_y[c3_i2] = c3_dv0[c3_i2];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_F_R;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[6];
  int32_T c3_i3;
  SFc3_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc3_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c3_F_R = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_F_R), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_F_R);
  for (c3_i3 = 0; c3_i3 < 6; c3_i3++) {
    (*(real_T (*)[6])c3_outData)[c3_i3] = c3_y[c3_i3];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

const mxArray *sf_c3_SS_jun_25_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  sf_mex_assign(&c3_nameCaptureInfo, sf_mex_createstruct("structure", 2, 14, 1),
                false);
  c3_info_helper(&c3_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static void c3_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs0 = NULL;
  const mxArray *c3_lhs0 = NULL;
  const mxArray *c3_rhs1 = NULL;
  const mxArray *c3_lhs1 = NULL;
  const mxArray *c3_rhs2 = NULL;
  const mxArray *c3_lhs2 = NULL;
  const mxArray *c3_rhs3 = NULL;
  const mxArray *c3_lhs3 = NULL;
  const mxArray *c3_rhs4 = NULL;
  const mxArray *c3_lhs4 = NULL;
  const mxArray *c3_rhs5 = NULL;
  const mxArray *c3_lhs5 = NULL;
  const mxArray *c3_rhs6 = NULL;
  const mxArray *c3_lhs6 = NULL;
  const mxArray *c3_rhs7 = NULL;
  const mxArray *c3_lhs7 = NULL;
  const mxArray *c3_rhs8 = NULL;
  const mxArray *c3_lhs8 = NULL;
  const mxArray *c3_rhs9 = NULL;
  const mxArray *c3_lhs9 = NULL;
  const mxArray *c3_rhs10 = NULL;
  const mxArray *c3_lhs10 = NULL;
  const mxArray *c3_rhs11 = NULL;
  const mxArray *c3_lhs11 = NULL;
  const mxArray *c3_rhs12 = NULL;
  const mxArray *c3_lhs12 = NULL;
  const mxArray *c3_rhs13 = NULL;
  const mxArray *c3_lhs13 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c3_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389739374U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c3_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c3_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c3_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c3_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c3_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c3_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("atan"), "name", "name", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395346496U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c3_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_atan"), "name",
                  "name", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286840318U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c3_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("tan"), "name", "name", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/tan.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395346504U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c3_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/tan.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_tan"), "name",
                  "name", 10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_tan.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c3_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("sqrt"), "name", "name", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c3_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_error"), "name", "name",
                  12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c3_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c3_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs13), "lhs", "lhs",
                  13);
  sf_mex_destroy(&c3_rhs0);
  sf_mex_destroy(&c3_lhs0);
  sf_mex_destroy(&c3_rhs1);
  sf_mex_destroy(&c3_lhs1);
  sf_mex_destroy(&c3_rhs2);
  sf_mex_destroy(&c3_lhs2);
  sf_mex_destroy(&c3_rhs3);
  sf_mex_destroy(&c3_lhs3);
  sf_mex_destroy(&c3_rhs4);
  sf_mex_destroy(&c3_lhs4);
  sf_mex_destroy(&c3_rhs5);
  sf_mex_destroy(&c3_lhs5);
  sf_mex_destroy(&c3_rhs6);
  sf_mex_destroy(&c3_lhs6);
  sf_mex_destroy(&c3_rhs7);
  sf_mex_destroy(&c3_lhs7);
  sf_mex_destroy(&c3_rhs8);
  sf_mex_destroy(&c3_lhs8);
  sf_mex_destroy(&c3_rhs9);
  sf_mex_destroy(&c3_lhs9);
  sf_mex_destroy(&c3_rhs10);
  sf_mex_destroy(&c3_lhs10);
  sf_mex_destroy(&c3_rhs11);
  sf_mex_destroy(&c3_lhs11);
  sf_mex_destroy(&c3_rhs12);
  sf_mex_destroy(&c3_lhs12);
  sf_mex_destroy(&c3_rhs13);
  sf_mex_destroy(&c3_lhs13);
}

static const mxArray *c3_emlrt_marshallOut(const char * c3_u)
{
  const mxArray *c3_y = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c3_u)), false);
  return c3_y;
}

static const mxArray *c3_b_emlrt_marshallOut(const uint32_T c3_u)
{
  const mxArray *c3_y = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 7, 0U, 0U, 0U, 0), false);
  return c3_y;
}

static real_T c3_sqrt(SFc3_SS_jun_25InstanceStruct *chartInstance, real_T c3_x)
{
  real_T c3_b_x;
  c3_b_x = c3_x;
  c3_b_sqrt(chartInstance, &c3_b_x);
  return c3_b_x;
}

static void c3_eml_error(SFc3_SS_jun_25InstanceStruct *chartInstance)
{
  int32_T c3_i4;
  static char_T c3_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c3_u[30];
  const mxArray *c3_y = NULL;
  int32_T c3_i5;
  static char_T c3_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c3_b_u[4];
  const mxArray *c3_b_y = NULL;
  (void)chartInstance;
  for (c3_i4 = 0; c3_i4 < 30; c3_i4++) {
    c3_u[c3_i4] = c3_cv0[c3_i4];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c3_i5 = 0; c3_i5 < 4; c3_i5++) {
    c3_b_u[c3_i5] = c3_cv1[c3_i5];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c3_y, 14, c3_b_y));
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc3_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static int32_T c3_d_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i6;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i6, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i6;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y;
  SFc3_SS_jun_25InstanceStruct *chartInstance;
  chartInstance = (SFc3_SS_jun_25InstanceStruct *)chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint8_T c3_e_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_b_is_active_c3_SS_jun_25, const char_T *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_SS_jun_25), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_SS_jun_25);
  return c3_y;
}

static uint8_T c3_f_emlrt_marshallIn(SFc3_SS_jun_25InstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_y;
  uint8_T c3_u0;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_u0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_sqrt(SFc3_SS_jun_25InstanceStruct *chartInstance, real_T *c3_x)
{
  if (*c3_x < 0.0) {
    c3_eml_error(chartInstance);
  }

  *c3_x = muDoubleScalarSqrt(*c3_x);
}

static void init_dsm_address_info(SFc3_SS_jun_25InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc3_SS_jun_25InstanceStruct *chartInstance)
{
  chartInstance->c3_FzR = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c3_F_SR = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c3_F_tR = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c3_U = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c3_V = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c3_Slip = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c3_delta = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c3_r = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    5);
  chartInstance->c3_a_fR = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c3_a_rR = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c3_a_mR = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c3_F_SR_f = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c3_F_SR_r = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 7);
  chartInstance->c3_F_SR_m = (real_T *)ssGetOutputPortSignal_wrapper
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

void sf_c3_SS_jun_25_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3763031064U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(544951293U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(281592576U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4272213884U);
}

mxArray* sf_c3_SS_jun_25_get_post_codegen_info(void);
mxArray *sf_c3_SS_jun_25_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("OHcGWVeHnzV2ebcXX45oGH");
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
    mxArray* mxPostCodegenInfo = sf_c3_SS_jun_25_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c3_SS_jun_25_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c3_SS_jun_25_jit_fallback_info(void)
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

mxArray *sf_c3_SS_jun_25_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c3_SS_jun_25_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c3_SS_jun_25(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x9'type','srcId','name','auxInfo'{{M[1],M[5],T\"F_SR\",},{M[1],M[25],T\"F_SR_f\",},{M[1],M[27],T\"F_SR_m\",},{M[1],M[26],T\"F_SR_r\",},{M[1],M[15],T\"F_tR\",},{M[1],M[18],T\"a_fR\",},{M[1],M[21],T\"a_mR\",},{M[1],M[19],T\"a_rR\",},{M[8],M[0],T\"is_active_c3_SS_jun_25\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 9, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_SS_jun_25_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_SS_jun_25InstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc3_SS_jun_25InstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SS_jun_25MachineNumber_,
           3,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"FzR");
          _SFD_SET_DATA_PROPS(1,2,0,1,"F_SR");
          _SFD_SET_DATA_PROPS(2,2,0,1,"F_tR");
          _SFD_SET_DATA_PROPS(3,1,1,0,"U");
          _SFD_SET_DATA_PROPS(4,1,1,0,"V");
          _SFD_SET_DATA_PROPS(5,1,1,0,"Slip");
          _SFD_SET_DATA_PROPS(6,1,1,0,"delta");
          _SFD_SET_DATA_PROPS(7,1,1,0,"r");
          _SFD_SET_DATA_PROPS(8,2,0,1,"a_fR");
          _SFD_SET_DATA_PROPS(9,2,0,1,"a_rR");
          _SFD_SET_DATA_PROPS(10,2,0,1,"a_mR");
          _SFD_SET_DATA_PROPS(11,2,0,1,"F_SR_f");
          _SFD_SET_DATA_PROPS(12,2,0,1,"F_SR_r");
          _SFD_SET_DATA_PROPS(13,2,0,1,"F_SR_m");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1719);
        _SFD_CV_INIT_EML_IF(0,1,0,898,907,929,948);
        _SFD_CV_INIT_EML_IF(0,1,1,1195,1204,1226,1245);
        _SFD_CV_INIT_EML_IF(0,1,2,1494,1503,1525,1544);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,901,906,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,1198,1203,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,1497,1502,-1,2);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c3_FzR);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c3_F_SR);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c3_F_tR);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c3_U);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c3_V);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c3_Slip);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c3_delta);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c3_r);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c3_a_fR);
        _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c3_a_rR);
        _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c3_a_mR);
        _SFD_SET_DATA_VALUE_PTR(11U, chartInstance->c3_F_SR_f);
        _SFD_SET_DATA_VALUE_PTR(12U, chartInstance->c3_F_SR_r);
        _SFD_SET_DATA_VALUE_PTR(13U, chartInstance->c3_F_SR_m);
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
  return "DFLoX7d11F0LmxKcahg6OD";
}

static void sf_opaque_initialize_c3_SS_jun_25(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_SS_jun_25InstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c3_SS_jun_25((SFc3_SS_jun_25InstanceStruct*)
    chartInstanceVar);
  initialize_c3_SS_jun_25((SFc3_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c3_SS_jun_25(void *chartInstanceVar)
{
  enable_c3_SS_jun_25((SFc3_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c3_SS_jun_25(void *chartInstanceVar)
{
  disable_c3_SS_jun_25((SFc3_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c3_SS_jun_25(void *chartInstanceVar)
{
  sf_gateway_c3_SS_jun_25((SFc3_SS_jun_25InstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c3_SS_jun_25(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c3_SS_jun_25((SFc3_SS_jun_25InstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c3_SS_jun_25(SimStruct* S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c3_SS_jun_25((SFc3_SS_jun_25InstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c3_SS_jun_25(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_SS_jun_25InstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SS_jun_25_optimization_info();
    }

    finalize_c3_SS_jun_25((SFc3_SS_jun_25InstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_SS_jun_25((SFc3_SS_jun_25InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_SS_jun_25(SimStruct *S)
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
    initialize_params_c3_SS_jun_25((SFc3_SS_jun_25InstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_SS_jun_25(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SS_jun_25_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,3,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,3);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,8);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(199569545U));
  ssSetChecksum1(S,(638370156U));
  ssSetChecksum2(S,(1679271455U));
  ssSetChecksum3(S,(1453112598U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_SS_jun_25(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_SS_jun_25(SimStruct *S)
{
  SFc3_SS_jun_25InstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc3_SS_jun_25InstanceStruct *)utMalloc(sizeof
    (SFc3_SS_jun_25InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_SS_jun_25InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c3_SS_jun_25;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c3_SS_jun_25;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c3_SS_jun_25;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c3_SS_jun_25;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c3_SS_jun_25;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c3_SS_jun_25;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c3_SS_jun_25;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c3_SS_jun_25;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_SS_jun_25;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_SS_jun_25;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c3_SS_jun_25;
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

void c3_SS_jun_25_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_SS_jun_25(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_SS_jun_25(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_SS_jun_25(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_SS_jun_25_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
