/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SS6_Estimation_sfun.h"
#include "c15_SS6_Estimation.h"
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
static const char * c15_debug_family_names[65] = { "M", "Ms", "Izz", "Ixx",
  "Ixz", "a", "b", "hcg", "e", "Tw", "R", "I_t", "I_e", "A_f", "K_bf", "zi1",
  "z2", "zi3", "zi4", "zi5", "Lbpsi", "Lbp", "C_a", "C_i", "mu", "e_r", "K_rsf",
  "K_rsr", "C1", "C2", "C3", "C_s1", "C_d", "d", "n", "rho", "g", "c", "Fz",
  "alpha", "U_tl", "U_tr", "S_f", "f_S", "F_tR_f", "S_r", "F_tR_r", "S_m",
  "F_tR_m", "F_R", "nargin", "nargout", "FzR", "U", "V", "Slip", "r", "F_SR",
  "F_tR", "a_fR", "a_rR", "a_mR", "F_SR_f", "F_SR_r", "F_SR_m" };

/* Function Declarations */
static void initialize_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance);
static void initialize_params_c15_SS6_Estimation
  (SFc15_SS6_EstimationInstanceStruct *chartInstance);
static void enable_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance);
static void disable_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance);
static void c15_update_debugger_state_c15_SS6_Estimation
  (SFc15_SS6_EstimationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c15_SS6_Estimation
  (SFc15_SS6_EstimationInstanceStruct *chartInstance);
static void set_sim_state_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct *
  chartInstance, const mxArray *c15_st);
static void finalize_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance);
static void sf_gateway_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance);
static void mdl_start_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance);
static void c15_chartstep_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct *
  chartInstance);
static void initSimStructsc15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c15_machineNumber, uint32_T
  c15_chartNumber, uint32_T c15_instanceNumber);
static const mxArray *c15_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData);
static real_T c15_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_b_F_SR_m, const char_T *c15_identifier);
static real_T c15_b_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId);
static void c15_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c15_mxArrayInData, const char_T *c15_varName, void *c15_outData);
static const mxArray *c15_b_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData);
static void c15_c_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId,
  real_T c15_y[6]);
static void c15_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c15_mxArrayInData, const char_T *c15_varName, void *c15_outData);
static void c15_info_helper(const mxArray **c15_info);
static const mxArray *c15_emlrt_marshallOut(const char * c15_u);
static const mxArray *c15_b_emlrt_marshallOut(const uint32_T c15_u);
static real_T c15_sqrt(SFc15_SS6_EstimationInstanceStruct *chartInstance, real_T
  c15_x);
static void c15_eml_error(SFc15_SS6_EstimationInstanceStruct *chartInstance);
static const mxArray *c15_c_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData);
static int32_T c15_d_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId);
static void c15_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c15_mxArrayInData, const char_T *c15_varName, void *c15_outData);
static uint8_T c15_e_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_b_is_active_c15_SS6_Estimation, const
  char_T *c15_identifier);
static uint8_T c15_f_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId);
static void c15_b_sqrt(SFc15_SS6_EstimationInstanceStruct *chartInstance, real_T
  *c15_x);
static void init_dsm_address_info(SFc15_SS6_EstimationInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc15_SS6_EstimationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance)
{
  chartInstance->c15_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c15_is_active_c15_SS6_Estimation = 0U;
}

static void initialize_params_c15_SS6_Estimation
  (SFc15_SS6_EstimationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c15_update_debugger_state_c15_SS6_Estimation
  (SFc15_SS6_EstimationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c15_SS6_Estimation
  (SFc15_SS6_EstimationInstanceStruct *chartInstance)
{
  const mxArray *c15_st;
  const mxArray *c15_y = NULL;
  real_T c15_hoistedGlobal;
  real_T c15_u;
  const mxArray *c15_b_y = NULL;
  real_T c15_b_hoistedGlobal;
  real_T c15_b_u;
  const mxArray *c15_c_y = NULL;
  real_T c15_c_hoistedGlobal;
  real_T c15_c_u;
  const mxArray *c15_d_y = NULL;
  real_T c15_d_hoistedGlobal;
  real_T c15_d_u;
  const mxArray *c15_e_y = NULL;
  real_T c15_e_hoistedGlobal;
  real_T c15_e_u;
  const mxArray *c15_f_y = NULL;
  real_T c15_f_hoistedGlobal;
  real_T c15_f_u;
  const mxArray *c15_g_y = NULL;
  real_T c15_g_hoistedGlobal;
  real_T c15_g_u;
  const mxArray *c15_h_y = NULL;
  real_T c15_h_hoistedGlobal;
  real_T c15_h_u;
  const mxArray *c15_i_y = NULL;
  uint8_T c15_i_hoistedGlobal;
  uint8_T c15_i_u;
  const mxArray *c15_j_y = NULL;
  c15_st = NULL;
  c15_st = NULL;
  c15_y = NULL;
  sf_mex_assign(&c15_y, sf_mex_createcellmatrix(9, 1), false);
  c15_hoistedGlobal = *chartInstance->c15_F_SR;
  c15_u = c15_hoistedGlobal;
  c15_b_y = NULL;
  sf_mex_assign(&c15_b_y, sf_mex_create("y", &c15_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c15_y, 0, c15_b_y);
  c15_b_hoistedGlobal = *chartInstance->c15_F_SR_f;
  c15_b_u = c15_b_hoistedGlobal;
  c15_c_y = NULL;
  sf_mex_assign(&c15_c_y, sf_mex_create("y", &c15_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c15_y, 1, c15_c_y);
  c15_c_hoistedGlobal = *chartInstance->c15_F_SR_m;
  c15_c_u = c15_c_hoistedGlobal;
  c15_d_y = NULL;
  sf_mex_assign(&c15_d_y, sf_mex_create("y", &c15_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c15_y, 2, c15_d_y);
  c15_d_hoistedGlobal = *chartInstance->c15_F_SR_r;
  c15_d_u = c15_d_hoistedGlobal;
  c15_e_y = NULL;
  sf_mex_assign(&c15_e_y, sf_mex_create("y", &c15_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c15_y, 3, c15_e_y);
  c15_e_hoistedGlobal = *chartInstance->c15_F_tR;
  c15_e_u = c15_e_hoistedGlobal;
  c15_f_y = NULL;
  sf_mex_assign(&c15_f_y, sf_mex_create("y", &c15_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c15_y, 4, c15_f_y);
  c15_f_hoistedGlobal = *chartInstance->c15_a_fR;
  c15_f_u = c15_f_hoistedGlobal;
  c15_g_y = NULL;
  sf_mex_assign(&c15_g_y, sf_mex_create("y", &c15_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c15_y, 5, c15_g_y);
  c15_g_hoistedGlobal = *chartInstance->c15_a_mR;
  c15_g_u = c15_g_hoistedGlobal;
  c15_h_y = NULL;
  sf_mex_assign(&c15_h_y, sf_mex_create("y", &c15_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c15_y, 6, c15_h_y);
  c15_h_hoistedGlobal = *chartInstance->c15_a_rR;
  c15_h_u = c15_h_hoistedGlobal;
  c15_i_y = NULL;
  sf_mex_assign(&c15_i_y, sf_mex_create("y", &c15_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c15_y, 7, c15_i_y);
  c15_i_hoistedGlobal = chartInstance->c15_is_active_c15_SS6_Estimation;
  c15_i_u = c15_i_hoistedGlobal;
  c15_j_y = NULL;
  sf_mex_assign(&c15_j_y, sf_mex_create("y", &c15_i_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c15_y, 8, c15_j_y);
  sf_mex_assign(&c15_st, c15_y, false);
  return c15_st;
}

static void set_sim_state_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct *
  chartInstance, const mxArray *c15_st)
{
  const mxArray *c15_u;
  chartInstance->c15_doneDoubleBufferReInit = true;
  c15_u = sf_mex_dup(c15_st);
  *chartInstance->c15_F_SR = c15_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c15_u, 0)), "F_SR");
  *chartInstance->c15_F_SR_f = c15_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c15_u, 1)), "F_SR_f");
  *chartInstance->c15_F_SR_m = c15_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c15_u, 2)), "F_SR_m");
  *chartInstance->c15_F_SR_r = c15_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c15_u, 3)), "F_SR_r");
  *chartInstance->c15_F_tR = c15_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c15_u, 4)), "F_tR");
  *chartInstance->c15_a_fR = c15_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c15_u, 5)), "a_fR");
  *chartInstance->c15_a_mR = c15_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c15_u, 6)), "a_mR");
  *chartInstance->c15_a_rR = c15_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c15_u, 7)), "a_rR");
  chartInstance->c15_is_active_c15_SS6_Estimation = c15_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c15_u, 8)),
     "is_active_c15_SS6_Estimation");
  sf_mex_destroy(&c15_u);
  c15_update_debugger_state_c15_SS6_Estimation(chartInstance);
  sf_mex_destroy(&c15_st);
}

static void finalize_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance)
{
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 14U, chartInstance->c15_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_FzR, 0U);
  chartInstance->c15_sfEvent = CALL_EVENT;
  c15_chartstep_c15_SS6_Estimation(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SS6_EstimationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_F_SR, 1U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_F_tR, 2U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_U, 3U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_V, 4U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_Slip, 5U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_r, 6U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_a_fR, 7U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_a_rR, 8U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_a_mR, 9U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_F_SR_f, 10U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_F_SR_r, 11U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c15_F_SR_m, 12U);
}

static void mdl_start_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c15_chartstep_c15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct *
  chartInstance)
{
  real_T c15_hoistedGlobal;
  real_T c15_b_hoistedGlobal;
  real_T c15_c_hoistedGlobal;
  real_T c15_d_hoistedGlobal;
  real_T c15_e_hoistedGlobal;
  real_T c15_b_FzR;
  real_T c15_b_U;
  real_T c15_b_V;
  real_T c15_b_Slip;
  real_T c15_b_r;
  uint32_T c15_debug_family_var_map[65];
  real_T c15_M;
  real_T c15_Ms;
  real_T c15_Izz;
  real_T c15_Ixx;
  real_T c15_Ixz;
  real_T c15_a;
  real_T c15_b;
  real_T c15_hcg;
  real_T c15_e;
  real_T c15_Tw;
  real_T c15_R;
  real_T c15_I_t;
  real_T c15_I_e;
  real_T c15_A_f;
  real_T c15_K_bf;
  real_T c15_zi1;
  real_T c15_z2;
  real_T c15_zi3;
  real_T c15_zi4;
  real_T c15_zi5;
  real_T c15_Lbpsi;
  real_T c15_Lbp;
  real_T c15_C_a;
  real_T c15_C_i;
  real_T c15_mu;
  real_T c15_e_r;
  real_T c15_K_rsf;
  real_T c15_K_rsr;
  real_T c15_C1;
  real_T c15_C2;
  real_T c15_C3;
  real_T c15_C_s1;
  real_T c15_C_d;
  real_T c15_d;
  real_T c15_n;
  real_T c15_rho;
  real_T c15_g;
  real_T c15_c;
  real_T c15_Fz;
  real_T c15_alpha;
  real_T c15_U_tl;
  real_T c15_U_tr;
  real_T c15_S_f;
  real_T c15_f_S;
  real_T c15_F_tR_f;
  real_T c15_S_r;
  real_T c15_F_tR_r;
  real_T c15_S_m;
  real_T c15_F_tR_m;
  real_T c15_F_R[6];
  real_T c15_nargin = 5.0;
  real_T c15_nargout = 8.0;
  real_T c15_b_F_SR;
  real_T c15_b_F_tR;
  real_T c15_b_a_fR;
  real_T c15_b_a_rR;
  real_T c15_b_a_mR;
  real_T c15_b_F_SR_f;
  real_T c15_b_F_SR_r;
  real_T c15_b_F_SR_m;
  real_T c15_A;
  real_T c15_B;
  real_T c15_x;
  real_T c15_y;
  real_T c15_b_x;
  real_T c15_b_y;
  real_T c15_c_x;
  real_T c15_c_y;
  real_T c15_d_y;
  real_T c15_d_x;
  real_T c15_e_x;
  real_T c15_b_A;
  real_T c15_b_B;
  real_T c15_f_x;
  real_T c15_e_y;
  real_T c15_g_x;
  real_T c15_f_y;
  real_T c15_h_x;
  real_T c15_g_y;
  real_T c15_h_y;
  real_T c15_i_x;
  real_T c15_j_x;
  real_T c15_c_A;
  real_T c15_c_B;
  real_T c15_k_x;
  real_T c15_i_y;
  real_T c15_l_x;
  real_T c15_j_y;
  real_T c15_m_x;
  real_T c15_k_y;
  real_T c15_l_y;
  real_T c15_n_x;
  real_T c15_o_x;
  real_T c15_d_A;
  real_T c15_p_x;
  real_T c15_q_x;
  real_T c15_r_x;
  real_T c15_m_y;
  real_T c15_s_x;
  real_T c15_t_x;
  real_T c15_u_x;
  real_T c15_v_x;
  real_T c15_w_x;
  real_T c15_x_x;
  real_T c15_y_x;
  real_T c15_ab_x;
  real_T c15_d0;
  real_T c15_e_A;
  real_T c15_d1;
  real_T c15_d_B;
  real_T c15_bb_x;
  real_T c15_n_y;
  real_T c15_cb_x;
  real_T c15_o_y;
  real_T c15_db_x;
  real_T c15_p_y;
  real_T c15_eb_x;
  real_T c15_fb_x;
  real_T c15_f_A;
  real_T c15_e_B;
  real_T c15_gb_x;
  real_T c15_q_y;
  real_T c15_hb_x;
  real_T c15_r_y;
  real_T c15_ib_x;
  real_T c15_s_y;
  real_T c15_g_A;
  real_T c15_f_B;
  real_T c15_jb_x;
  real_T c15_t_y;
  real_T c15_kb_x;
  real_T c15_u_y;
  real_T c15_lb_x;
  real_T c15_v_y;
  real_T c15_h_A;
  real_T c15_mb_x;
  real_T c15_nb_x;
  real_T c15_ob_x;
  real_T c15_w_y;
  real_T c15_pb_x;
  real_T c15_qb_x;
  real_T c15_rb_x;
  real_T c15_sb_x;
  real_T c15_tb_x;
  real_T c15_ub_x;
  real_T c15_vb_x;
  real_T c15_wb_x;
  real_T c15_d2;
  real_T c15_i_A;
  real_T c15_d3;
  real_T c15_g_B;
  real_T c15_xb_x;
  real_T c15_x_y;
  real_T c15_yb_x;
  real_T c15_y_y;
  real_T c15_ac_x;
  real_T c15_ab_y;
  real_T c15_bc_x;
  real_T c15_cc_x;
  real_T c15_j_A;
  real_T c15_h_B;
  real_T c15_dc_x;
  real_T c15_bb_y;
  real_T c15_ec_x;
  real_T c15_cb_y;
  real_T c15_fc_x;
  real_T c15_db_y;
  real_T c15_k_A;
  real_T c15_i_B;
  real_T c15_gc_x;
  real_T c15_eb_y;
  real_T c15_hc_x;
  real_T c15_fb_y;
  real_T c15_ic_x;
  real_T c15_gb_y;
  real_T c15_l_A;
  real_T c15_jc_x;
  real_T c15_kc_x;
  real_T c15_lc_x;
  real_T c15_hb_y;
  real_T c15_mc_x;
  real_T c15_nc_x;
  real_T c15_oc_x;
  real_T c15_pc_x;
  real_T c15_qc_x;
  real_T c15_rc_x;
  real_T c15_sc_x;
  real_T c15_tc_x;
  real_T c15_uc_x;
  real_T c15_vc_x;
  real_T c15_wc_x;
  real_T c15_xc_x;
  real_T c15_m_A;
  real_T c15_j_B;
  real_T c15_yc_x;
  real_T c15_ib_y;
  real_T c15_ad_x;
  real_T c15_jb_y;
  real_T c15_bd_x;
  real_T c15_kb_y;
  real_T c15_cd_x;
  real_T c15_dd_x;
  real_T c15_n_A;
  real_T c15_k_B;
  real_T c15_ed_x;
  real_T c15_lb_y;
  real_T c15_fd_x;
  real_T c15_mb_y;
  real_T c15_gd_x;
  real_T c15_nb_y;
  real_T c15_o_A;
  real_T c15_l_B;
  real_T c15_hd_x;
  real_T c15_ob_y;
  real_T c15_id_x;
  real_T c15_pb_y;
  real_T c15_jd_x;
  real_T c15_qb_y;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 14U, chartInstance->c15_sfEvent);
  c15_hoistedGlobal = *chartInstance->c15_FzR;
  c15_b_hoistedGlobal = *chartInstance->c15_U;
  c15_c_hoistedGlobal = *chartInstance->c15_V;
  c15_d_hoistedGlobal = *chartInstance->c15_Slip;
  c15_e_hoistedGlobal = *chartInstance->c15_r;
  c15_b_FzR = c15_hoistedGlobal;
  c15_b_U = c15_b_hoistedGlobal;
  c15_b_V = c15_c_hoistedGlobal;
  c15_b_Slip = c15_d_hoistedGlobal;
  c15_b_r = c15_e_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 65U, 65U, c15_debug_family_names,
    c15_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_M, 0U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_Ms, 1U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_Izz, 2U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_Ixx, 3U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_Ixz, 4U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_a, 5U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_b, 6U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_hcg, 7U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_e, 8U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_Tw, 9U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_R, 10U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_I_t, 11U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_I_e, 12U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_A_f, 13U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_K_bf, 14U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_zi1, 15U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_z2, 16U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_zi3, 17U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_zi4, 18U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_zi5, 19U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_Lbpsi, 20U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_Lbp, 21U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_C_a, 22U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_C_i, 23U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_mu, 24U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_e_r, 25U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_K_rsf, 26U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_K_rsr, 27U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_C1, 28U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_C2, 29U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_C3, 30U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_C_s1, 31U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_C_d, 32U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_d, 33U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_n, 34U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_rho, 35U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_g, 36U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_c, 37U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_Fz, 38U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_alpha, 39U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_U_tl, 40U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_U_tr, 41U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_S_f, 42U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_f_S, 43U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_F_tR_f, 44U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_S_r, 45U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_F_tR_r, 46U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_S_m, 47U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_F_tR_m, 48U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c15_F_R, 49U, c15_b_sf_marshallOut,
    c15_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_nargin, 50U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_nargout, 51U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_b_FzR, 52U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_b_U, 53U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_b_V, 54U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_b_Slip, 55U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c15_b_r, 56U, c15_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_b_F_SR, 57U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_b_F_tR, 58U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_b_a_fR, 59U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_b_a_rR, 60U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_b_a_mR, 61U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_b_F_SR_f, 62U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_b_F_SR_r, 63U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_b_F_SR_m, 64U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_M = 1280.0;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_Ms = 1160.0;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_Izz = 2500.0;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_Ixx = 750.0;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_Ixz = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_a = 1.203;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_b = 1.217;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_hcg = 0.5;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_e = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_Tw = 1.33;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_R = 0.3;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 2);
  c15_I_t = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 3);
  c15_I_e = 0.136;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 3);
  c15_A_f = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 3);
  c15_K_bf = 0.55;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 3);
  c15_zi1 = 13.56;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 3);
  c15_z2 = 7.5;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 3);
  c15_zi3 = 5.37;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 3);
  c15_zi4 = 4.22;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 3);
  c15_zi5 = 3.28;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 4);
  c15_Lbpsi = 45000.0;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 4);
  c15_Lbp = 2600.0;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 4);
  c15_C_a = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 4);
  c15_C_i = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 4);
  c15_mu = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 4);
  c15_e_r = 0.015;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 4);
  c15_K_rsf = -0.05;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 4);
  c15_K_rsr = 0.1;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 5);
  c15_C1 = -6.0;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 5);
  c15_C2 = 59.16;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 5);
  c15_C3 = 25.0;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 5);
  c15_C_s1 = 1.38;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 5);
  c15_K_rsf = 0.444;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 5);
  c15_C_d = 0.32;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 5);
  c15_d = 0.014;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 5);
  c15_n = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 6);
  c15_rho = 1.204;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 6);
  c15_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 6);
  c15_c = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 10);
  c15_Fz = c15_b_FzR;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 17);
  c15_A = c15_b_V + 1.203 * c15_b_r;
  c15_B = c15_b_U - 0.665 * c15_b_r;
  c15_x = c15_A;
  c15_y = c15_B;
  c15_b_x = c15_x;
  c15_b_y = c15_y;
  c15_c_x = c15_b_x;
  c15_c_y = c15_b_y;
  c15_d_y = c15_c_x / c15_c_y;
  c15_d_x = c15_d_y;
  c15_e_x = c15_d_x;
  c15_e_x = muDoubleScalarAtan(c15_e_x);
  c15_b_a_fR = -c15_e_x;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 18);
  c15_b_A = 1.217 * c15_b_r - c15_b_V;
  c15_b_B = c15_b_U - 0.665 * c15_b_r;
  c15_f_x = c15_b_A;
  c15_e_y = c15_b_B;
  c15_g_x = c15_f_x;
  c15_f_y = c15_e_y;
  c15_h_x = c15_g_x;
  c15_g_y = c15_f_y;
  c15_h_y = c15_h_x / c15_g_y;
  c15_i_x = c15_h_y;
  c15_b_a_rR = c15_i_x;
  c15_j_x = c15_b_a_rR;
  c15_b_a_rR = c15_j_x;
  c15_b_a_rR = muDoubleScalarAtan(c15_b_a_rR);
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 19);
  c15_c_A = 0.2 * c15_b_r - c15_b_V;
  c15_c_B = c15_b_U - 0.665 * c15_b_r;
  c15_k_x = c15_c_A;
  c15_i_y = c15_c_B;
  c15_l_x = c15_k_x;
  c15_j_y = c15_i_y;
  c15_m_x = c15_l_x;
  c15_k_y = c15_j_y;
  c15_l_y = c15_m_x / c15_k_y;
  c15_n_x = c15_l_y;
  c15_b_a_mR = c15_n_x;
  c15_o_x = c15_b_a_mR;
  c15_b_a_mR = c15_o_x;
  c15_b_a_mR = muDoubleScalarAtan(c15_b_a_mR);
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 21);
  c15_alpha = c15_b_a_fR;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 23);
  c15_U_tl = c15_b_U + 0.665 * c15_b_r;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 24);
  c15_U_tr = c15_b_U - 0.665 * c15_b_r;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 27);
  c15_d_A = c15_Fz;
  c15_p_x = c15_d_A;
  c15_q_x = c15_p_x;
  c15_r_x = c15_q_x;
  c15_m_y = c15_r_x / 3.0;
  c15_s_x = c15_alpha;
  c15_t_x = c15_s_x;
  c15_t_x = muDoubleScalarTan(c15_t_x);
  c15_u_x = c15_alpha;
  c15_v_x = c15_u_x;
  c15_v_x = muDoubleScalarTan(c15_v_x);
  c15_w_x = c15_alpha;
  c15_x_x = c15_w_x;
  c15_x_x = muDoubleScalarTan(c15_x_x);
  c15_y_x = c15_alpha;
  c15_ab_x = c15_y_x;
  c15_ab_x = muDoubleScalarTan(c15_ab_x);
  c15_d0 = c15_b_Slip * c15_b_Slip + c15_t_x * c15_v_x;
  c15_b_sqrt(chartInstance, &c15_d0);
  c15_e_A = 0.85 * c15_m_y * (1.0 - 0.015 * c15_U_tr * c15_d0) * (1.0 -
    c15_b_Slip);
  c15_d1 = 4.0E+8 * c15_b_Slip * c15_b_Slip + 4.0E+8 * c15_x_x * c15_ab_x;
  c15_b_sqrt(chartInstance, &c15_d1);
  c15_d_B = 2.0 * c15_d1;
  c15_bb_x = c15_e_A;
  c15_n_y = c15_d_B;
  c15_cb_x = c15_bb_x;
  c15_o_y = c15_n_y;
  c15_db_x = c15_cb_x;
  c15_p_y = c15_o_y;
  c15_S_f = c15_db_x / c15_p_y;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 29);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c15_S_f, 1.0, -1, 2U,
        c15_S_f < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 30);
    c15_f_S = c15_S_f * (2.0 - c15_S_f);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 32);
    c15_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 34);
  c15_eb_x = c15_alpha;
  c15_fb_x = c15_eb_x;
  c15_fb_x = muDoubleScalarTan(c15_fb_x);
  c15_f_A = 20000.0 * c15_fb_x * c15_f_S;
  c15_e_B = 1.0 - c15_b_Slip;
  c15_gb_x = c15_f_A;
  c15_q_y = c15_e_B;
  c15_hb_x = c15_gb_x;
  c15_r_y = c15_q_y;
  c15_ib_x = c15_hb_x;
  c15_s_y = c15_r_y;
  c15_b_F_SR_f = c15_ib_x / c15_s_y;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 35);
  c15_g_A = 20000.0 * c15_b_Slip * c15_f_S;
  c15_f_B = 1.0 - c15_b_Slip;
  c15_jb_x = c15_g_A;
  c15_t_y = c15_f_B;
  c15_kb_x = c15_jb_x;
  c15_u_y = c15_t_y;
  c15_lb_x = c15_kb_x;
  c15_v_y = c15_u_y;
  c15_F_tR_f = c15_lb_x / c15_v_y;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 38);
  c15_alpha = c15_b_a_rR;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 40);
  c15_h_A = c15_Fz;
  c15_mb_x = c15_h_A;
  c15_nb_x = c15_mb_x;
  c15_ob_x = c15_nb_x;
  c15_w_y = c15_ob_x / 3.0;
  c15_pb_x = c15_alpha;
  c15_qb_x = c15_pb_x;
  c15_qb_x = muDoubleScalarTan(c15_qb_x);
  c15_rb_x = c15_alpha;
  c15_sb_x = c15_rb_x;
  c15_sb_x = muDoubleScalarTan(c15_sb_x);
  c15_tb_x = c15_alpha;
  c15_ub_x = c15_tb_x;
  c15_ub_x = muDoubleScalarTan(c15_ub_x);
  c15_vb_x = c15_alpha;
  c15_wb_x = c15_vb_x;
  c15_wb_x = muDoubleScalarTan(c15_wb_x);
  c15_d2 = c15_b_Slip * c15_b_Slip + c15_qb_x * c15_sb_x;
  c15_b_sqrt(chartInstance, &c15_d2);
  c15_i_A = 0.85 * c15_w_y * (1.0 - 0.015 * c15_U_tr * c15_d2) * (1.0 -
    c15_b_Slip);
  c15_d3 = 4.0E+8 * c15_b_Slip * c15_b_Slip + 4.0E+8 * c15_ub_x * c15_wb_x;
  c15_b_sqrt(chartInstance, &c15_d3);
  c15_g_B = 2.0 * c15_d3;
  c15_xb_x = c15_i_A;
  c15_x_y = c15_g_B;
  c15_yb_x = c15_xb_x;
  c15_y_y = c15_x_y;
  c15_ac_x = c15_yb_x;
  c15_ab_y = c15_y_y;
  c15_S_r = c15_ac_x / c15_ab_y;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 42);
  if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c15_S_r, 1.0, -1, 2U,
        c15_S_r < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 43);
    c15_f_S = c15_S_r * (2.0 - c15_S_r);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 45);
    c15_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 47);
  c15_bc_x = c15_alpha;
  c15_cc_x = c15_bc_x;
  c15_cc_x = muDoubleScalarTan(c15_cc_x);
  c15_j_A = 20000.0 * c15_cc_x * c15_f_S;
  c15_h_B = 1.0 - c15_b_Slip;
  c15_dc_x = c15_j_A;
  c15_bb_y = c15_h_B;
  c15_ec_x = c15_dc_x;
  c15_cb_y = c15_bb_y;
  c15_fc_x = c15_ec_x;
  c15_db_y = c15_cb_y;
  c15_b_F_SR_r = c15_fc_x / c15_db_y;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 48);
  c15_k_A = 20000.0 * c15_b_Slip * c15_f_S;
  c15_i_B = 1.0 - c15_b_Slip;
  c15_gc_x = c15_k_A;
  c15_eb_y = c15_i_B;
  c15_hc_x = c15_gc_x;
  c15_fb_y = c15_eb_y;
  c15_ic_x = c15_hc_x;
  c15_gb_y = c15_fb_y;
  c15_F_tR_r = c15_ic_x / c15_gb_y;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 51);
  c15_alpha = c15_b_a_mR;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 53);
  c15_l_A = c15_Fz;
  c15_jc_x = c15_l_A;
  c15_kc_x = c15_jc_x;
  c15_lc_x = c15_kc_x;
  c15_hb_y = c15_lc_x / 3.0;
  c15_mc_x = c15_alpha;
  c15_nc_x = c15_mc_x;
  c15_nc_x = muDoubleScalarTan(c15_nc_x);
  c15_oc_x = c15_alpha;
  c15_pc_x = c15_oc_x;
  c15_pc_x = muDoubleScalarTan(c15_pc_x);
  c15_qc_x = c15_b_Slip * c15_b_Slip + c15_nc_x * c15_pc_x;
  c15_rc_x = c15_qc_x;
  if (c15_rc_x < 0.0) {
    c15_eml_error(chartInstance);
  }

  c15_rc_x = muDoubleScalarSqrt(c15_rc_x);
  c15_sc_x = c15_alpha;
  c15_tc_x = c15_sc_x;
  c15_tc_x = muDoubleScalarTan(c15_tc_x);
  c15_uc_x = c15_alpha;
  c15_vc_x = c15_uc_x;
  c15_vc_x = muDoubleScalarTan(c15_vc_x);
  c15_wc_x = 4.0E+8 * c15_b_Slip * c15_b_Slip + 4.0E+8 * c15_tc_x * c15_vc_x;
  c15_xc_x = c15_wc_x;
  if (c15_xc_x < 0.0) {
    c15_eml_error(chartInstance);
  }

  c15_xc_x = muDoubleScalarSqrt(c15_xc_x);
  c15_m_A = 0.85 * c15_hb_y * (1.0 - 0.015 * c15_U_tr * c15_rc_x) * (1.0 -
    c15_b_Slip);
  c15_j_B = 2.0 * c15_xc_x;
  c15_yc_x = c15_m_A;
  c15_ib_y = c15_j_B;
  c15_ad_x = c15_yc_x;
  c15_jb_y = c15_ib_y;
  c15_bd_x = c15_ad_x;
  c15_kb_y = c15_jb_y;
  c15_S_m = c15_bd_x / c15_kb_y;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 55);
  if (CV_EML_IF(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 2, c15_S_m, 1.0, -1, 2U,
        c15_S_m < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 56);
    c15_f_S = c15_S_m * (2.0 - c15_S_m);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 58);
    c15_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 60);
  c15_cd_x = c15_alpha;
  c15_dd_x = c15_cd_x;
  c15_dd_x = muDoubleScalarTan(c15_dd_x);
  c15_n_A = 20000.0 * c15_dd_x * c15_f_S;
  c15_k_B = 1.0 - c15_b_Slip;
  c15_ed_x = c15_n_A;
  c15_lb_y = c15_k_B;
  c15_fd_x = c15_ed_x;
  c15_mb_y = c15_lb_y;
  c15_gd_x = c15_fd_x;
  c15_nb_y = c15_mb_y;
  c15_b_F_SR_m = c15_gd_x / c15_nb_y;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 61);
  c15_o_A = 20000.0 * c15_b_Slip * c15_f_S;
  c15_l_B = 1.0 - c15_b_Slip;
  c15_hd_x = c15_o_A;
  c15_ob_y = c15_l_B;
  c15_id_x = c15_hd_x;
  c15_pb_y = c15_ob_y;
  c15_jd_x = c15_id_x;
  c15_qb_y = c15_pb_y;
  c15_F_tR_m = c15_jd_x / c15_qb_y;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 64);
  c15_b_F_SR = (c15_b_F_SR_f + c15_b_F_SR_r) + c15_b_F_SR_m;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 65);
  c15_b_F_tR = (c15_F_tR_f + c15_F_tR_r) + c15_F_tR_m;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 66);
  c15_F_R[0] = c15_F_tR_f;
  c15_F_R[1] = c15_F_tR_r;
  c15_F_R[2] = c15_F_tR_m;
  c15_F_R[3] = c15_b_F_SR_f;
  c15_F_R[4] = c15_b_F_SR_r;
  c15_F_R[5] = c15_b_F_SR_m;
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, -66);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c15_F_SR = c15_b_F_SR;
  *chartInstance->c15_F_tR = c15_b_F_tR;
  *chartInstance->c15_a_fR = c15_b_a_fR;
  *chartInstance->c15_a_rR = c15_b_a_rR;
  *chartInstance->c15_a_mR = c15_b_a_mR;
  *chartInstance->c15_F_SR_f = c15_b_F_SR_f;
  *chartInstance->c15_F_SR_r = c15_b_F_SR_r;
  *chartInstance->c15_F_SR_m = c15_b_F_SR_m;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 14U, chartInstance->c15_sfEvent);
}

static void initSimStructsc15_SS6_Estimation(SFc15_SS6_EstimationInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c15_machineNumber, uint32_T
  c15_chartNumber, uint32_T c15_instanceNumber)
{
  (void)c15_machineNumber;
  (void)c15_chartNumber;
  (void)c15_instanceNumber;
}

static const mxArray *c15_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData)
{
  const mxArray *c15_mxArrayOutData = NULL;
  real_T c15_u;
  const mxArray *c15_y = NULL;
  SFc15_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc15_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c15_mxArrayOutData = NULL;
  c15_u = *(real_T *)c15_inData;
  c15_y = NULL;
  sf_mex_assign(&c15_y, sf_mex_create("y", &c15_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c15_mxArrayOutData, c15_y, false);
  return c15_mxArrayOutData;
}

static real_T c15_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_b_F_SR_m, const char_T *c15_identifier)
{
  real_T c15_y;
  emlrtMsgIdentifier c15_thisId;
  c15_thisId.fIdentifier = c15_identifier;
  c15_thisId.fParent = NULL;
  c15_y = c15_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c15_b_F_SR_m),
    &c15_thisId);
  sf_mex_destroy(&c15_b_F_SR_m);
  return c15_y;
}

static real_T c15_b_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId)
{
  real_T c15_y;
  real_T c15_d4;
  (void)chartInstance;
  sf_mex_import(c15_parentId, sf_mex_dup(c15_u), &c15_d4, 1, 0, 0U, 0, 0U, 0);
  c15_y = c15_d4;
  sf_mex_destroy(&c15_u);
  return c15_y;
}

static void c15_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c15_mxArrayInData, const char_T *c15_varName, void *c15_outData)
{
  const mxArray *c15_b_F_SR_m;
  const char_T *c15_identifier;
  emlrtMsgIdentifier c15_thisId;
  real_T c15_y;
  SFc15_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc15_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c15_b_F_SR_m = sf_mex_dup(c15_mxArrayInData);
  c15_identifier = c15_varName;
  c15_thisId.fIdentifier = c15_identifier;
  c15_thisId.fParent = NULL;
  c15_y = c15_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c15_b_F_SR_m),
    &c15_thisId);
  sf_mex_destroy(&c15_b_F_SR_m);
  *(real_T *)c15_outData = c15_y;
  sf_mex_destroy(&c15_mxArrayInData);
}

static const mxArray *c15_b_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData)
{
  const mxArray *c15_mxArrayOutData = NULL;
  int32_T c15_i0;
  real_T c15_b_inData[6];
  int32_T c15_i1;
  real_T c15_u[6];
  const mxArray *c15_y = NULL;
  SFc15_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc15_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c15_mxArrayOutData = NULL;
  for (c15_i0 = 0; c15_i0 < 6; c15_i0++) {
    c15_b_inData[c15_i0] = (*(real_T (*)[6])c15_inData)[c15_i0];
  }

  for (c15_i1 = 0; c15_i1 < 6; c15_i1++) {
    c15_u[c15_i1] = c15_b_inData[c15_i1];
  }

  c15_y = NULL;
  sf_mex_assign(&c15_y, sf_mex_create("y", c15_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c15_mxArrayOutData, c15_y, false);
  return c15_mxArrayOutData;
}

static void c15_c_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId,
  real_T c15_y[6])
{
  real_T c15_dv0[6];
  int32_T c15_i2;
  (void)chartInstance;
  sf_mex_import(c15_parentId, sf_mex_dup(c15_u), c15_dv0, 1, 0, 0U, 1, 0U, 1, 6);
  for (c15_i2 = 0; c15_i2 < 6; c15_i2++) {
    c15_y[c15_i2] = c15_dv0[c15_i2];
  }

  sf_mex_destroy(&c15_u);
}

static void c15_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c15_mxArrayInData, const char_T *c15_varName, void *c15_outData)
{
  const mxArray *c15_F_R;
  const char_T *c15_identifier;
  emlrtMsgIdentifier c15_thisId;
  real_T c15_y[6];
  int32_T c15_i3;
  SFc15_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc15_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c15_F_R = sf_mex_dup(c15_mxArrayInData);
  c15_identifier = c15_varName;
  c15_thisId.fIdentifier = c15_identifier;
  c15_thisId.fParent = NULL;
  c15_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c15_F_R), &c15_thisId, c15_y);
  sf_mex_destroy(&c15_F_R);
  for (c15_i3 = 0; c15_i3 < 6; c15_i3++) {
    (*(real_T (*)[6])c15_outData)[c15_i3] = c15_y[c15_i3];
  }

  sf_mex_destroy(&c15_mxArrayInData);
}

const mxArray *sf_c15_SS6_Estimation_get_eml_resolved_functions_info(void)
{
  const mxArray *c15_nameCaptureInfo = NULL;
  c15_nameCaptureInfo = NULL;
  sf_mex_assign(&c15_nameCaptureInfo, sf_mex_createstruct("structure", 2, 14, 1),
                false);
  c15_info_helper(&c15_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c15_nameCaptureInfo);
  return c15_nameCaptureInfo;
}

static void c15_info_helper(const mxArray **c15_info)
{
  const mxArray *c15_rhs0 = NULL;
  const mxArray *c15_lhs0 = NULL;
  const mxArray *c15_rhs1 = NULL;
  const mxArray *c15_lhs1 = NULL;
  const mxArray *c15_rhs2 = NULL;
  const mxArray *c15_lhs2 = NULL;
  const mxArray *c15_rhs3 = NULL;
  const mxArray *c15_lhs3 = NULL;
  const mxArray *c15_rhs4 = NULL;
  const mxArray *c15_lhs4 = NULL;
  const mxArray *c15_rhs5 = NULL;
  const mxArray *c15_lhs5 = NULL;
  const mxArray *c15_rhs6 = NULL;
  const mxArray *c15_lhs6 = NULL;
  const mxArray *c15_rhs7 = NULL;
  const mxArray *c15_lhs7 = NULL;
  const mxArray *c15_rhs8 = NULL;
  const mxArray *c15_lhs8 = NULL;
  const mxArray *c15_rhs9 = NULL;
  const mxArray *c15_lhs9 = NULL;
  const mxArray *c15_rhs10 = NULL;
  const mxArray *c15_lhs10 = NULL;
  const mxArray *c15_rhs11 = NULL;
  const mxArray *c15_lhs11 = NULL;
  const mxArray *c15_rhs12 = NULL;
  const mxArray *c15_lhs12 = NULL;
  const mxArray *c15_rhs13 = NULL;
  const mxArray *c15_lhs13 = NULL;
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("mrdivide"), "name", "name",
                  0);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c15_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs0), "rhs", "rhs",
                  0);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs0), "lhs", "lhs",
                  0);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 1);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1389739374U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c15_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs1), "rhs", "rhs",
                  1);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs1), "lhs", "lhs",
                  1);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c15_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs2), "rhs", "rhs",
                  2);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs2), "lhs", "lhs",
                  2);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c15_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs3), "rhs", "rhs",
                  3);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs3), "lhs", "lhs",
                  3);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c15_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs4), "rhs", "rhs",
                  4);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs4), "lhs", "lhs",
                  4);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c15_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs5), "rhs", "rhs",
                  5);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs5), "lhs", "lhs",
                  5);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c15_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs6), "rhs", "rhs",
                  6);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs6), "lhs", "lhs",
                  6);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("atan"), "name", "name", 7);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1395346496U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c15_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs7), "rhs", "rhs",
                  7);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs7), "lhs", "lhs",
                  7);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("eml_scalar_atan"), "name",
                  "name", 8);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1286840318U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c15_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs8), "rhs", "rhs",
                  8);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs8), "lhs", "lhs",
                  8);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(""), "context", "context", 9);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("tan"), "name", "name", 9);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/tan.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1395346504U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c15_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs9), "rhs", "rhs",
                  9);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs9), "lhs", "lhs",
                  9);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/tan.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("eml_scalar_tan"), "name",
                  "name", 10);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_tan.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c15_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(""), "context", "context", 11);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("sqrt"), "name", "name", 11);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c15_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("eml_error"), "name", "name",
                  12);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c15_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 13);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c15_info, c15_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c15_info, c15_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c15_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c15_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c15_info, sf_mex_duplicatearraysafe(&c15_lhs13), "lhs", "lhs",
                  13);
  sf_mex_destroy(&c15_rhs0);
  sf_mex_destroy(&c15_lhs0);
  sf_mex_destroy(&c15_rhs1);
  sf_mex_destroy(&c15_lhs1);
  sf_mex_destroy(&c15_rhs2);
  sf_mex_destroy(&c15_lhs2);
  sf_mex_destroy(&c15_rhs3);
  sf_mex_destroy(&c15_lhs3);
  sf_mex_destroy(&c15_rhs4);
  sf_mex_destroy(&c15_lhs4);
  sf_mex_destroy(&c15_rhs5);
  sf_mex_destroy(&c15_lhs5);
  sf_mex_destroy(&c15_rhs6);
  sf_mex_destroy(&c15_lhs6);
  sf_mex_destroy(&c15_rhs7);
  sf_mex_destroy(&c15_lhs7);
  sf_mex_destroy(&c15_rhs8);
  sf_mex_destroy(&c15_lhs8);
  sf_mex_destroy(&c15_rhs9);
  sf_mex_destroy(&c15_lhs9);
  sf_mex_destroy(&c15_rhs10);
  sf_mex_destroy(&c15_lhs10);
  sf_mex_destroy(&c15_rhs11);
  sf_mex_destroy(&c15_lhs11);
  sf_mex_destroy(&c15_rhs12);
  sf_mex_destroy(&c15_lhs12);
  sf_mex_destroy(&c15_rhs13);
  sf_mex_destroy(&c15_lhs13);
}

static const mxArray *c15_emlrt_marshallOut(const char * c15_u)
{
  const mxArray *c15_y = NULL;
  c15_y = NULL;
  sf_mex_assign(&c15_y, sf_mex_create("y", c15_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c15_u)), false);
  return c15_y;
}

static const mxArray *c15_b_emlrt_marshallOut(const uint32_T c15_u)
{
  const mxArray *c15_y = NULL;
  c15_y = NULL;
  sf_mex_assign(&c15_y, sf_mex_create("y", &c15_u, 7, 0U, 0U, 0U, 0), false);
  return c15_y;
}

static real_T c15_sqrt(SFc15_SS6_EstimationInstanceStruct *chartInstance, real_T
  c15_x)
{
  real_T c15_b_x;
  c15_b_x = c15_x;
  c15_b_sqrt(chartInstance, &c15_b_x);
  return c15_b_x;
}

static void c15_eml_error(SFc15_SS6_EstimationInstanceStruct *chartInstance)
{
  int32_T c15_i4;
  static char_T c15_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c15_u[30];
  const mxArray *c15_y = NULL;
  int32_T c15_i5;
  static char_T c15_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c15_b_u[4];
  const mxArray *c15_b_y = NULL;
  (void)chartInstance;
  for (c15_i4 = 0; c15_i4 < 30; c15_i4++) {
    c15_u[c15_i4] = c15_cv0[c15_i4];
  }

  c15_y = NULL;
  sf_mex_assign(&c15_y, sf_mex_create("y", c15_u, 10, 0U, 1U, 0U, 2, 1, 30),
                false);
  for (c15_i5 = 0; c15_i5 < 4; c15_i5++) {
    c15_b_u[c15_i5] = c15_cv1[c15_i5];
  }

  c15_b_y = NULL;
  sf_mex_assign(&c15_b_y, sf_mex_create("y", c15_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c15_y, 14, c15_b_y));
}

static const mxArray *c15_c_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData)
{
  const mxArray *c15_mxArrayOutData = NULL;
  int32_T c15_u;
  const mxArray *c15_y = NULL;
  SFc15_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc15_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c15_mxArrayOutData = NULL;
  c15_u = *(int32_T *)c15_inData;
  c15_y = NULL;
  sf_mex_assign(&c15_y, sf_mex_create("y", &c15_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c15_mxArrayOutData, c15_y, false);
  return c15_mxArrayOutData;
}

static int32_T c15_d_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId)
{
  int32_T c15_y;
  int32_T c15_i6;
  (void)chartInstance;
  sf_mex_import(c15_parentId, sf_mex_dup(c15_u), &c15_i6, 1, 6, 0U, 0, 0U, 0);
  c15_y = c15_i6;
  sf_mex_destroy(&c15_u);
  return c15_y;
}

static void c15_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c15_mxArrayInData, const char_T *c15_varName, void *c15_outData)
{
  const mxArray *c15_b_sfEvent;
  const char_T *c15_identifier;
  emlrtMsgIdentifier c15_thisId;
  int32_T c15_y;
  SFc15_SS6_EstimationInstanceStruct *chartInstance;
  chartInstance = (SFc15_SS6_EstimationInstanceStruct *)chartInstanceVoid;
  c15_b_sfEvent = sf_mex_dup(c15_mxArrayInData);
  c15_identifier = c15_varName;
  c15_thisId.fIdentifier = c15_identifier;
  c15_thisId.fParent = NULL;
  c15_y = c15_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c15_b_sfEvent),
    &c15_thisId);
  sf_mex_destroy(&c15_b_sfEvent);
  *(int32_T *)c15_outData = c15_y;
  sf_mex_destroy(&c15_mxArrayInData);
}

static uint8_T c15_e_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_b_is_active_c15_SS6_Estimation, const
  char_T *c15_identifier)
{
  uint8_T c15_y;
  emlrtMsgIdentifier c15_thisId;
  c15_thisId.fIdentifier = c15_identifier;
  c15_thisId.fParent = NULL;
  c15_y = c15_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c15_b_is_active_c15_SS6_Estimation), &c15_thisId);
  sf_mex_destroy(&c15_b_is_active_c15_SS6_Estimation);
  return c15_y;
}

static uint8_T c15_f_emlrt_marshallIn(SFc15_SS6_EstimationInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId)
{
  uint8_T c15_y;
  uint8_T c15_u0;
  (void)chartInstance;
  sf_mex_import(c15_parentId, sf_mex_dup(c15_u), &c15_u0, 1, 3, 0U, 0, 0U, 0);
  c15_y = c15_u0;
  sf_mex_destroy(&c15_u);
  return c15_y;
}

static void c15_b_sqrt(SFc15_SS6_EstimationInstanceStruct *chartInstance, real_T
  *c15_x)
{
  if (*c15_x < 0.0) {
    c15_eml_error(chartInstance);
  }

  *c15_x = muDoubleScalarSqrt(*c15_x);
}

static void init_dsm_address_info(SFc15_SS6_EstimationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc15_SS6_EstimationInstanceStruct
  *chartInstance)
{
  chartInstance->c15_FzR = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c15_F_SR = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c15_F_tR = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c15_U = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c15_V = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c15_Slip = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c15_r = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    4);
  chartInstance->c15_a_fR = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c15_a_rR = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c15_a_mR = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c15_F_SR_f = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c15_F_SR_r = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 7);
  chartInstance->c15_F_SR_m = (real_T *)ssGetOutputPortSignal_wrapper
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

void sf_c15_SS6_Estimation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(83905453U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2027019346U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(412307648U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2634295697U);
}

mxArray* sf_c15_SS6_Estimation_get_post_codegen_info(void);
mxArray *sf_c15_SS6_Estimation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("Jb33eHdHaBoeR9W53vjoFH");
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
    mxArray* mxPostCodegenInfo = sf_c15_SS6_Estimation_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c15_SS6_Estimation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c15_SS6_Estimation_jit_fallback_info(void)
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

mxArray *sf_c15_SS6_Estimation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c15_SS6_Estimation_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c15_SS6_Estimation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x9'type','srcId','name','auxInfo'{{M[1],M[5],T\"F_SR\",},{M[1],M[25],T\"F_SR_f\",},{M[1],M[27],T\"F_SR_m\",},{M[1],M[26],T\"F_SR_r\",},{M[1],M[15],T\"F_tR\",},{M[1],M[18],T\"a_fR\",},{M[1],M[21],T\"a_mR\",},{M[1],M[19],T\"a_rR\",},{M[8],M[0],T\"is_active_c15_SS6_Estimation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 9, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c15_SS6_Estimation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc15_SS6_EstimationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc15_SS6_EstimationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SS6_EstimationMachineNumber_,
           15,
           1,
           1,
           0,
           13,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"FzR");
          _SFD_SET_DATA_PROPS(1,2,0,1,"F_SR");
          _SFD_SET_DATA_PROPS(2,2,0,1,"F_tR");
          _SFD_SET_DATA_PROPS(3,1,1,0,"U");
          _SFD_SET_DATA_PROPS(4,1,1,0,"V");
          _SFD_SET_DATA_PROPS(5,1,1,0,"Slip");
          _SFD_SET_DATA_PROPS(6,1,1,0,"r");
          _SFD_SET_DATA_PROPS(7,2,0,1,"a_fR");
          _SFD_SET_DATA_PROPS(8,2,0,1,"a_rR");
          _SFD_SET_DATA_PROPS(9,2,0,1,"a_mR");
          _SFD_SET_DATA_PROPS(10,2,0,1,"F_SR_f");
          _SFD_SET_DATA_PROPS(11,2,0,1,"F_SR_r");
          _SFD_SET_DATA_PROPS(12,2,0,1,"F_SR_m");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1711);
        _SFD_CV_INIT_EML_IF(0,1,0,890,899,921,940);
        _SFD_CV_INIT_EML_IF(0,1,1,1187,1196,1218,1237);
        _SFD_CV_INIT_EML_IF(0,1,2,1486,1495,1517,1536);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,893,898,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,1190,1195,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,1489,1494,-1,2);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)c15_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)c15_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)c15_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)c15_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)c15_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)c15_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)c15_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)c15_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c15_FzR);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c15_F_SR);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c15_F_tR);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c15_U);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c15_V);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c15_Slip);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c15_r);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c15_a_fR);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c15_a_rR);
        _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c15_a_mR);
        _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c15_F_SR_f);
        _SFD_SET_DATA_VALUE_PTR(11U, chartInstance->c15_F_SR_r);
        _SFD_SET_DATA_VALUE_PTR(12U, chartInstance->c15_F_SR_m);
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
  return "VDOXVMN2OffVJVx1mVOexG";
}

static void sf_opaque_initialize_c15_SS6_Estimation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc15_SS6_EstimationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c15_SS6_Estimation((SFc15_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
  initialize_c15_SS6_Estimation((SFc15_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c15_SS6_Estimation(void *chartInstanceVar)
{
  enable_c15_SS6_Estimation((SFc15_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c15_SS6_Estimation(void *chartInstanceVar)
{
  disable_c15_SS6_Estimation((SFc15_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c15_SS6_Estimation(void *chartInstanceVar)
{
  sf_gateway_c15_SS6_Estimation((SFc15_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c15_SS6_Estimation(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c15_SS6_Estimation((SFc15_SS6_EstimationInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c15_SS6_Estimation(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c15_SS6_Estimation((SFc15_SS6_EstimationInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c15_SS6_Estimation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc15_SS6_EstimationInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SS6_Estimation_optimization_info();
    }

    finalize_c15_SS6_Estimation((SFc15_SS6_EstimationInstanceStruct*)
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
  initSimStructsc15_SS6_Estimation((SFc15_SS6_EstimationInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c15_SS6_Estimation(SimStruct *S)
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
    initialize_params_c15_SS6_Estimation((SFc15_SS6_EstimationInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c15_SS6_Estimation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SS6_Estimation_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      15);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,15,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,15,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,15);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,15,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,15,8);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=8; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,15);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3821430295U));
  ssSetChecksum1(S,(3484719826U));
  ssSetChecksum2(S,(964849085U));
  ssSetChecksum3(S,(3985659557U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c15_SS6_Estimation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c15_SS6_Estimation(SimStruct *S)
{
  SFc15_SS6_EstimationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc15_SS6_EstimationInstanceStruct *)utMalloc(sizeof
    (SFc15_SS6_EstimationInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc15_SS6_EstimationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c15_SS6_Estimation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c15_SS6_Estimation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c15_SS6_Estimation;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c15_SS6_Estimation;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c15_SS6_Estimation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c15_SS6_Estimation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c15_SS6_Estimation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c15_SS6_Estimation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c15_SS6_Estimation;
  chartInstance->chartInfo.mdlStart = mdlStart_c15_SS6_Estimation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c15_SS6_Estimation;
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

void c15_SS6_Estimation_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c15_SS6_Estimation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c15_SS6_Estimation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c15_SS6_Estimation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c15_SS6_Estimation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
