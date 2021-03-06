/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SS6_Estimation2_sfun.h"
#include "c14_SS6_Estimation2.h"
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
static const char * c14_debug_family_names[65] = { "M", "Ms", "Izz", "Ixx",
  "Ixz", "a", "b", "hcg", "e", "Tw", "R", "I_t", "I_e", "A_f", "K_bf", "zi1",
  "z2", "zi3", "zi4", "zi5", "Lbpsi", "Lbp", "C_a", "C_i", "mu", "e_r", "K_rsf",
  "K_rsr", "C1", "C2", "C3", "C_s1", "C_d", "d", "n", "rho", "g", "c", "Fz",
  "U_tl", "U_tr", "alpha", "S_f", "f_S", "F_tL_f", "S_r", "F_tL_r", "S_m",
  "F_tL_m", "F_L", "nargin", "nargout", "FzL", "U", "V", "Slip", "r", "F_SL",
  "F_tL", "a_fL", "a_rL", "a_mL", "F_SL_f", "F_SL_r", "F_SL_m" };

/* Function Declarations */
static void initialize_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance);
static void initialize_params_c14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance);
static void enable_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance);
static void disable_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c14_update_debugger_state_c14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance);
static void set_sim_state_c14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance, const mxArray *c14_st);
static void finalize_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance);
static void sf_gateway_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance);
static void mdl_start_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance);
static void c14_chartstep_c14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance);
static void initSimStructsc14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c14_machineNumber, uint32_T
  c14_chartNumber, uint32_T c14_instanceNumber);
static const mxArray *c14_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static real_T c14_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_b_F_SL_m, const char_T *c14_identifier);
static real_T c14_b_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId);
static void c14_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static const mxArray *c14_b_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static void c14_c_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[6]);
static void c14_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static void c14_info_helper(const mxArray **c14_info);
static const mxArray *c14_emlrt_marshallOut(const char * c14_u);
static const mxArray *c14_b_emlrt_marshallOut(const uint32_T c14_u);
static real_T c14_sqrt(SFc14_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c14_x);
static void c14_eml_error(SFc14_SS6_Estimation2InstanceStruct *chartInstance);
static const mxArray *c14_c_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static int32_T c14_d_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId);
static void c14_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static uint8_T c14_e_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_b_is_active_c14_SS6_Estimation2, const
  char_T *c14_identifier);
static uint8_T c14_f_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId);
static void c14_b_sqrt(SFc14_SS6_Estimation2InstanceStruct *chartInstance,
  real_T *c14_x);
static void init_dsm_address_info(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  chartInstance->c14_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c14_is_active_c14_SS6_Estimation2 = 0U;
}

static void initialize_params_c14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c14_update_debugger_state_c14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance)
{
  const mxArray *c14_st;
  const mxArray *c14_y = NULL;
  real_T c14_hoistedGlobal;
  real_T c14_u;
  const mxArray *c14_b_y = NULL;
  real_T c14_b_hoistedGlobal;
  real_T c14_b_u;
  const mxArray *c14_c_y = NULL;
  real_T c14_c_hoistedGlobal;
  real_T c14_c_u;
  const mxArray *c14_d_y = NULL;
  real_T c14_d_hoistedGlobal;
  real_T c14_d_u;
  const mxArray *c14_e_y = NULL;
  real_T c14_e_hoistedGlobal;
  real_T c14_e_u;
  const mxArray *c14_f_y = NULL;
  real_T c14_f_hoistedGlobal;
  real_T c14_f_u;
  const mxArray *c14_g_y = NULL;
  real_T c14_g_hoistedGlobal;
  real_T c14_g_u;
  const mxArray *c14_h_y = NULL;
  real_T c14_h_hoistedGlobal;
  real_T c14_h_u;
  const mxArray *c14_i_y = NULL;
  uint8_T c14_i_hoistedGlobal;
  uint8_T c14_i_u;
  const mxArray *c14_j_y = NULL;
  c14_st = NULL;
  c14_st = NULL;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_createcellmatrix(9, 1), false);
  c14_hoistedGlobal = *chartInstance->c14_F_SL;
  c14_u = c14_hoistedGlobal;
  c14_b_y = NULL;
  sf_mex_assign(&c14_b_y, sf_mex_create("y", &c14_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c14_y, 0, c14_b_y);
  c14_b_hoistedGlobal = *chartInstance->c14_F_SL_f;
  c14_b_u = c14_b_hoistedGlobal;
  c14_c_y = NULL;
  sf_mex_assign(&c14_c_y, sf_mex_create("y", &c14_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c14_y, 1, c14_c_y);
  c14_c_hoistedGlobal = *chartInstance->c14_F_SL_m;
  c14_c_u = c14_c_hoistedGlobal;
  c14_d_y = NULL;
  sf_mex_assign(&c14_d_y, sf_mex_create("y", &c14_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c14_y, 2, c14_d_y);
  c14_d_hoistedGlobal = *chartInstance->c14_F_SL_r;
  c14_d_u = c14_d_hoistedGlobal;
  c14_e_y = NULL;
  sf_mex_assign(&c14_e_y, sf_mex_create("y", &c14_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c14_y, 3, c14_e_y);
  c14_e_hoistedGlobal = *chartInstance->c14_F_tL;
  c14_e_u = c14_e_hoistedGlobal;
  c14_f_y = NULL;
  sf_mex_assign(&c14_f_y, sf_mex_create("y", &c14_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c14_y, 4, c14_f_y);
  c14_f_hoistedGlobal = *chartInstance->c14_a_fL;
  c14_f_u = c14_f_hoistedGlobal;
  c14_g_y = NULL;
  sf_mex_assign(&c14_g_y, sf_mex_create("y", &c14_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c14_y, 5, c14_g_y);
  c14_g_hoistedGlobal = *chartInstance->c14_a_mL;
  c14_g_u = c14_g_hoistedGlobal;
  c14_h_y = NULL;
  sf_mex_assign(&c14_h_y, sf_mex_create("y", &c14_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c14_y, 6, c14_h_y);
  c14_h_hoistedGlobal = *chartInstance->c14_a_rL;
  c14_h_u = c14_h_hoistedGlobal;
  c14_i_y = NULL;
  sf_mex_assign(&c14_i_y, sf_mex_create("y", &c14_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c14_y, 7, c14_i_y);
  c14_i_hoistedGlobal = chartInstance->c14_is_active_c14_SS6_Estimation2;
  c14_i_u = c14_i_hoistedGlobal;
  c14_j_y = NULL;
  sf_mex_assign(&c14_j_y, sf_mex_create("y", &c14_i_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c14_y, 8, c14_j_y);
  sf_mex_assign(&c14_st, c14_y, false);
  return c14_st;
}

static void set_sim_state_c14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance, const mxArray *c14_st)
{
  const mxArray *c14_u;
  chartInstance->c14_doneDoubleBufferReInit = true;
  c14_u = sf_mex_dup(c14_st);
  *chartInstance->c14_F_SL = c14_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c14_u, 0)), "F_SL");
  *chartInstance->c14_F_SL_f = c14_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c14_u, 1)), "F_SL_f");
  *chartInstance->c14_F_SL_m = c14_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c14_u, 2)), "F_SL_m");
  *chartInstance->c14_F_SL_r = c14_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c14_u, 3)), "F_SL_r");
  *chartInstance->c14_F_tL = c14_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c14_u, 4)), "F_tL");
  *chartInstance->c14_a_fL = c14_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c14_u, 5)), "a_fL");
  *chartInstance->c14_a_mL = c14_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c14_u, 6)), "a_mL");
  *chartInstance->c14_a_rL = c14_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c14_u, 7)), "a_rL");
  chartInstance->c14_is_active_c14_SS6_Estimation2 = c14_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c14_u, 8)),
     "is_active_c14_SS6_Estimation2");
  sf_mex_destroy(&c14_u);
  c14_update_debugger_state_c14_SS6_Estimation2(chartInstance);
  sf_mex_destroy(&c14_st);
}

static void finalize_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_FzL, 0U);
  chartInstance->c14_sfEvent = CALL_EVENT;
  c14_chartstep_c14_SS6_Estimation2(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SS6_Estimation2MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_F_SL, 1U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_F_tL, 2U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_U, 3U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_V, 4U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_Slip, 5U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_r, 6U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_a_fL, 7U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_a_rL, 8U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_a_mL, 9U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_F_SL_f, 10U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_F_SL_r, 11U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_F_SL_m, 12U);
}

static void mdl_start_c14_SS6_Estimation2(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c14_chartstep_c14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance)
{
  real_T c14_hoistedGlobal;
  real_T c14_b_hoistedGlobal;
  real_T c14_c_hoistedGlobal;
  real_T c14_d_hoistedGlobal;
  real_T c14_e_hoistedGlobal;
  real_T c14_b_FzL;
  real_T c14_b_U;
  real_T c14_b_V;
  real_T c14_b_Slip;
  real_T c14_b_r;
  uint32_T c14_debug_family_var_map[65];
  real_T c14_M;
  real_T c14_Ms;
  real_T c14_Izz;
  real_T c14_Ixx;
  real_T c14_Ixz;
  real_T c14_a;
  real_T c14_b;
  real_T c14_hcg;
  real_T c14_e;
  real_T c14_Tw;
  real_T c14_R;
  real_T c14_I_t;
  real_T c14_I_e;
  real_T c14_A_f;
  real_T c14_K_bf;
  real_T c14_zi1;
  real_T c14_z2;
  real_T c14_zi3;
  real_T c14_zi4;
  real_T c14_zi5;
  real_T c14_Lbpsi;
  real_T c14_Lbp;
  real_T c14_C_a;
  real_T c14_C_i;
  real_T c14_mu;
  real_T c14_e_r;
  real_T c14_K_rsf;
  real_T c14_K_rsr;
  real_T c14_C1;
  real_T c14_C2;
  real_T c14_C3;
  real_T c14_C_s1;
  real_T c14_C_d;
  real_T c14_d;
  real_T c14_n;
  real_T c14_rho;
  real_T c14_g;
  real_T c14_c;
  real_T c14_Fz;
  real_T c14_U_tl;
  real_T c14_U_tr;
  real_T c14_alpha;
  real_T c14_S_f;
  real_T c14_f_S;
  real_T c14_F_tL_f;
  real_T c14_S_r;
  real_T c14_F_tL_r;
  real_T c14_S_m;
  real_T c14_F_tL_m;
  real_T c14_F_L[6];
  real_T c14_nargin = 5.0;
  real_T c14_nargout = 8.0;
  real_T c14_b_F_SL;
  real_T c14_b_F_tL;
  real_T c14_b_a_fL;
  real_T c14_b_a_rL;
  real_T c14_b_a_mL;
  real_T c14_b_F_SL_f;
  real_T c14_b_F_SL_r;
  real_T c14_b_F_SL_m;
  real_T c14_A;
  real_T c14_B;
  real_T c14_x;
  real_T c14_y;
  real_T c14_b_x;
  real_T c14_b_y;
  real_T c14_c_x;
  real_T c14_c_y;
  real_T c14_d_y;
  real_T c14_d_x;
  real_T c14_e_x;
  real_T c14_b_A;
  real_T c14_b_B;
  real_T c14_f_x;
  real_T c14_e_y;
  real_T c14_g_x;
  real_T c14_f_y;
  real_T c14_h_x;
  real_T c14_g_y;
  real_T c14_h_y;
  real_T c14_i_x;
  real_T c14_j_x;
  real_T c14_c_A;
  real_T c14_c_B;
  real_T c14_k_x;
  real_T c14_i_y;
  real_T c14_l_x;
  real_T c14_j_y;
  real_T c14_m_x;
  real_T c14_k_y;
  real_T c14_l_y;
  real_T c14_n_x;
  real_T c14_o_x;
  real_T c14_d_A;
  real_T c14_p_x;
  real_T c14_q_x;
  real_T c14_r_x;
  real_T c14_m_y;
  real_T c14_s_x;
  real_T c14_t_x;
  real_T c14_u_x;
  real_T c14_v_x;
  real_T c14_w_x;
  real_T c14_x_x;
  real_T c14_y_x;
  real_T c14_ab_x;
  real_T c14_d0;
  real_T c14_e_A;
  real_T c14_d1;
  real_T c14_d_B;
  real_T c14_bb_x;
  real_T c14_n_y;
  real_T c14_cb_x;
  real_T c14_o_y;
  real_T c14_db_x;
  real_T c14_p_y;
  real_T c14_eb_x;
  real_T c14_fb_x;
  real_T c14_f_A;
  real_T c14_e_B;
  real_T c14_gb_x;
  real_T c14_q_y;
  real_T c14_hb_x;
  real_T c14_r_y;
  real_T c14_ib_x;
  real_T c14_s_y;
  real_T c14_g_A;
  real_T c14_f_B;
  real_T c14_jb_x;
  real_T c14_t_y;
  real_T c14_kb_x;
  real_T c14_u_y;
  real_T c14_lb_x;
  real_T c14_v_y;
  real_T c14_h_A;
  real_T c14_mb_x;
  real_T c14_nb_x;
  real_T c14_ob_x;
  real_T c14_w_y;
  real_T c14_pb_x;
  real_T c14_qb_x;
  real_T c14_rb_x;
  real_T c14_sb_x;
  real_T c14_tb_x;
  real_T c14_ub_x;
  real_T c14_vb_x;
  real_T c14_wb_x;
  real_T c14_d2;
  real_T c14_i_A;
  real_T c14_d3;
  real_T c14_g_B;
  real_T c14_xb_x;
  real_T c14_x_y;
  real_T c14_yb_x;
  real_T c14_y_y;
  real_T c14_ac_x;
  real_T c14_ab_y;
  real_T c14_bc_x;
  real_T c14_cc_x;
  real_T c14_j_A;
  real_T c14_h_B;
  real_T c14_dc_x;
  real_T c14_bb_y;
  real_T c14_ec_x;
  real_T c14_cb_y;
  real_T c14_fc_x;
  real_T c14_db_y;
  real_T c14_k_A;
  real_T c14_i_B;
  real_T c14_gc_x;
  real_T c14_eb_y;
  real_T c14_hc_x;
  real_T c14_fb_y;
  real_T c14_ic_x;
  real_T c14_gb_y;
  real_T c14_l_A;
  real_T c14_jc_x;
  real_T c14_kc_x;
  real_T c14_lc_x;
  real_T c14_hb_y;
  real_T c14_mc_x;
  real_T c14_nc_x;
  real_T c14_oc_x;
  real_T c14_pc_x;
  real_T c14_qc_x;
  real_T c14_rc_x;
  real_T c14_sc_x;
  real_T c14_tc_x;
  real_T c14_uc_x;
  real_T c14_vc_x;
  real_T c14_wc_x;
  real_T c14_xc_x;
  real_T c14_m_A;
  real_T c14_j_B;
  real_T c14_yc_x;
  real_T c14_ib_y;
  real_T c14_ad_x;
  real_T c14_jb_y;
  real_T c14_bd_x;
  real_T c14_kb_y;
  real_T c14_cd_x;
  real_T c14_dd_x;
  real_T c14_n_A;
  real_T c14_k_B;
  real_T c14_ed_x;
  real_T c14_lb_y;
  real_T c14_fd_x;
  real_T c14_mb_y;
  real_T c14_gd_x;
  real_T c14_nb_y;
  real_T c14_o_A;
  real_T c14_l_B;
  real_T c14_hd_x;
  real_T c14_ob_y;
  real_T c14_id_x;
  real_T c14_pb_y;
  real_T c14_jd_x;
  real_T c14_qb_y;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
  c14_hoistedGlobal = *chartInstance->c14_FzL;
  c14_b_hoistedGlobal = *chartInstance->c14_U;
  c14_c_hoistedGlobal = *chartInstance->c14_V;
  c14_d_hoistedGlobal = *chartInstance->c14_Slip;
  c14_e_hoistedGlobal = *chartInstance->c14_r;
  c14_b_FzL = c14_hoistedGlobal;
  c14_b_U = c14_b_hoistedGlobal;
  c14_b_V = c14_c_hoistedGlobal;
  c14_b_Slip = c14_d_hoistedGlobal;
  c14_b_r = c14_e_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 65U, 65U, c14_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_M, 0U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_Ms, 1U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_Izz, 2U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_Ixx, 3U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_Ixz, 4U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_a, 5U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_b, 6U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_hcg, 7U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_e, 8U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_Tw, 9U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_R, 10U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_I_t, 11U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_I_e, 12U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_A_f, 13U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_K_bf, 14U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_zi1, 15U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_z2, 16U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_zi3, 17U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_zi4, 18U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_zi5, 19U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_Lbpsi, 20U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_Lbp, 21U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_C_a, 22U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_C_i, 23U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_mu, 24U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_e_r, 25U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_K_rsf, 26U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_K_rsr, 27U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_C1, 28U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_C2, 29U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_C3, 30U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_C_s1, 31U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_C_d, 32U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_d, 33U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_n, 34U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_rho, 35U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_g, 36U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_c, 37U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_Fz, 38U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_U_tl, 39U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_U_tr, 40U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_alpha, 41U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_S_f, 42U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_f_S, 43U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_F_tL_f, 44U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_S_r, 45U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_F_tL_r, 46U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_S_m, 47U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_F_tL_m, 48U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_F_L, 49U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 50U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 51U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_b_FzL, 52U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_b_U, 53U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_b_V, 54U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_b_Slip, 55U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_b_r, 56U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_b_F_SL, 57U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_b_F_tL, 58U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_b_a_fL, 59U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_b_a_rL, 60U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_b_a_mL, 61U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_b_F_SL_f, 62U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_b_F_SL_r, 63U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_b_F_SL_m, 64U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_M = 1280.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_Ms = 1160.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_Izz = 2500.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_Ixx = 750.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_Ixz = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_a = 1.203;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_b = 1.217;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_hcg = 0.5;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_e = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_Tw = 1.33;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_R = 0.3;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 2);
  c14_I_t = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 3);
  c14_I_e = 0.136;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 3);
  c14_A_f = 2.1;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 3);
  c14_K_bf = 0.55;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 3);
  c14_zi1 = 13.56;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 3);
  c14_z2 = 7.5;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 3);
  c14_zi3 = 5.37;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 3);
  c14_zi4 = 4.22;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 3);
  c14_zi5 = 3.28;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 4);
  c14_Lbpsi = 45000.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 4);
  c14_Lbp = 2600.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 4);
  c14_C_a = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 4);
  c14_C_i = 20000.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 4);
  c14_mu = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 4);
  c14_e_r = 0.015;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 4);
  c14_K_rsf = -0.05;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 4);
  c14_K_rsr = 0.1;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 5);
  c14_C1 = -6.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 5);
  c14_C2 = 59.16;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 5);
  c14_C3 = 25.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 5);
  c14_C_s1 = 1.38;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 5);
  c14_K_rsf = 0.444;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 5);
  c14_C_d = 0.32;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 5);
  c14_d = 0.014;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 5);
  c14_n = 0.85;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 6);
  c14_rho = 1.204;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 6);
  c14_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 6);
  c14_c = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 11);
  c14_Fz = c14_b_FzL;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 16);
  c14_U_tl = c14_b_U + 0.665 * c14_b_r;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 17);
  c14_U_tr = c14_b_U - 0.665 * c14_b_r;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 20);
  c14_A = c14_b_V + 1.203 * c14_b_r;
  c14_B = c14_b_U + 0.665 * c14_b_r;
  c14_x = c14_A;
  c14_y = c14_B;
  c14_b_x = c14_x;
  c14_b_y = c14_y;
  c14_c_x = c14_b_x;
  c14_c_y = c14_b_y;
  c14_d_y = c14_c_x / c14_c_y;
  c14_d_x = c14_d_y;
  c14_e_x = c14_d_x;
  c14_e_x = muDoubleScalarAtan(c14_e_x);
  c14_b_a_fL = -c14_e_x;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 21);
  c14_b_A = 1.217 * c14_b_r - c14_b_V;
  c14_b_B = c14_b_U + 0.665 * c14_b_r;
  c14_f_x = c14_b_A;
  c14_e_y = c14_b_B;
  c14_g_x = c14_f_x;
  c14_f_y = c14_e_y;
  c14_h_x = c14_g_x;
  c14_g_y = c14_f_y;
  c14_h_y = c14_h_x / c14_g_y;
  c14_i_x = c14_h_y;
  c14_b_a_rL = c14_i_x;
  c14_j_x = c14_b_a_rL;
  c14_b_a_rL = c14_j_x;
  c14_b_a_rL = muDoubleScalarAtan(c14_b_a_rL);
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 22);
  c14_c_A = 0.2 * c14_b_r - c14_b_V;
  c14_c_B = c14_b_U + 0.665 * c14_b_r;
  c14_k_x = c14_c_A;
  c14_i_y = c14_c_B;
  c14_l_x = c14_k_x;
  c14_j_y = c14_i_y;
  c14_m_x = c14_l_x;
  c14_k_y = c14_j_y;
  c14_l_y = c14_m_x / c14_k_y;
  c14_n_x = c14_l_y;
  c14_b_a_mL = c14_n_x;
  c14_o_x = c14_b_a_mL;
  c14_b_a_mL = c14_o_x;
  c14_b_a_mL = muDoubleScalarAtan(c14_b_a_mL);
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 24);
  c14_alpha = c14_b_a_fL;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 26);
  c14_d_A = c14_Fz;
  c14_p_x = c14_d_A;
  c14_q_x = c14_p_x;
  c14_r_x = c14_q_x;
  c14_m_y = c14_r_x / 3.0;
  c14_s_x = c14_alpha;
  c14_t_x = c14_s_x;
  c14_t_x = muDoubleScalarTan(c14_t_x);
  c14_u_x = c14_alpha;
  c14_v_x = c14_u_x;
  c14_v_x = muDoubleScalarTan(c14_v_x);
  c14_w_x = c14_alpha;
  c14_x_x = c14_w_x;
  c14_x_x = muDoubleScalarTan(c14_x_x);
  c14_y_x = c14_alpha;
  c14_ab_x = c14_y_x;
  c14_ab_x = muDoubleScalarTan(c14_ab_x);
  c14_d0 = c14_b_Slip * c14_b_Slip + c14_t_x * c14_v_x;
  c14_b_sqrt(chartInstance, &c14_d0);
  c14_e_A = 0.85 * c14_m_y * (1.0 - 0.015 * c14_U_tl * c14_d0) * (1.0 -
    c14_b_Slip);
  c14_d1 = 4.0E+8 * c14_b_Slip * c14_b_Slip + 4.0E+8 * c14_x_x * c14_ab_x;
  c14_b_sqrt(chartInstance, &c14_d1);
  c14_d_B = 2.0 * c14_d1;
  c14_bb_x = c14_e_A;
  c14_n_y = c14_d_B;
  c14_cb_x = c14_bb_x;
  c14_o_y = c14_n_y;
  c14_db_x = c14_cb_x;
  c14_p_y = c14_o_y;
  c14_S_f = c14_db_x / c14_p_y;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 28);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c14_S_f, 1.0, -1, 2U,
        c14_S_f < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 29);
    c14_f_S = c14_S_f * (2.0 - c14_S_f);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 31);
    c14_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 33);
  c14_eb_x = c14_alpha;
  c14_fb_x = c14_eb_x;
  c14_fb_x = muDoubleScalarTan(c14_fb_x);
  c14_f_A = 20000.0 * c14_fb_x * c14_f_S;
  c14_e_B = 1.0 - c14_b_Slip;
  c14_gb_x = c14_f_A;
  c14_q_y = c14_e_B;
  c14_hb_x = c14_gb_x;
  c14_r_y = c14_q_y;
  c14_ib_x = c14_hb_x;
  c14_s_y = c14_r_y;
  c14_b_F_SL_f = c14_ib_x / c14_s_y;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 34);
  c14_g_A = 20000.0 * c14_b_Slip * c14_f_S;
  c14_f_B = 1.0 - c14_b_Slip;
  c14_jb_x = c14_g_A;
  c14_t_y = c14_f_B;
  c14_kb_x = c14_jb_x;
  c14_u_y = c14_t_y;
  c14_lb_x = c14_kb_x;
  c14_v_y = c14_u_y;
  c14_F_tL_f = c14_lb_x / c14_v_y;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 37);
  c14_alpha = c14_b_a_rL;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 39);
  c14_h_A = c14_Fz;
  c14_mb_x = c14_h_A;
  c14_nb_x = c14_mb_x;
  c14_ob_x = c14_nb_x;
  c14_w_y = c14_ob_x / 3.0;
  c14_pb_x = c14_alpha;
  c14_qb_x = c14_pb_x;
  c14_qb_x = muDoubleScalarTan(c14_qb_x);
  c14_rb_x = c14_alpha;
  c14_sb_x = c14_rb_x;
  c14_sb_x = muDoubleScalarTan(c14_sb_x);
  c14_tb_x = c14_alpha;
  c14_ub_x = c14_tb_x;
  c14_ub_x = muDoubleScalarTan(c14_ub_x);
  c14_vb_x = c14_alpha;
  c14_wb_x = c14_vb_x;
  c14_wb_x = muDoubleScalarTan(c14_wb_x);
  c14_d2 = c14_b_Slip * c14_b_Slip + c14_qb_x * c14_sb_x;
  c14_b_sqrt(chartInstance, &c14_d2);
  c14_i_A = 0.85 * c14_w_y * (1.0 - 0.015 * c14_U_tl * c14_d2) * (1.0 -
    c14_b_Slip);
  c14_d3 = 4.0E+8 * c14_b_Slip * c14_b_Slip + 4.0E+8 * c14_ub_x * c14_wb_x;
  c14_b_sqrt(chartInstance, &c14_d3);
  c14_g_B = 2.0 * c14_d3;
  c14_xb_x = c14_i_A;
  c14_x_y = c14_g_B;
  c14_yb_x = c14_xb_x;
  c14_y_y = c14_x_y;
  c14_ac_x = c14_yb_x;
  c14_ab_y = c14_y_y;
  c14_S_r = c14_ac_x / c14_ab_y;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 41);
  if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c14_S_r, 1.0, -1, 2U,
        c14_S_r < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 42);
    c14_f_S = c14_S_r * (2.0 - c14_S_r);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 44);
    c14_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 46);
  c14_bc_x = c14_alpha;
  c14_cc_x = c14_bc_x;
  c14_cc_x = muDoubleScalarTan(c14_cc_x);
  c14_j_A = 20000.0 * c14_cc_x * c14_f_S;
  c14_h_B = 1.0 - c14_b_Slip;
  c14_dc_x = c14_j_A;
  c14_bb_y = c14_h_B;
  c14_ec_x = c14_dc_x;
  c14_cb_y = c14_bb_y;
  c14_fc_x = c14_ec_x;
  c14_db_y = c14_cb_y;
  c14_b_F_SL_r = c14_fc_x / c14_db_y;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 47);
  c14_k_A = 20000.0 * c14_b_Slip * c14_f_S;
  c14_i_B = 1.0 - c14_b_Slip;
  c14_gc_x = c14_k_A;
  c14_eb_y = c14_i_B;
  c14_hc_x = c14_gc_x;
  c14_fb_y = c14_eb_y;
  c14_ic_x = c14_hc_x;
  c14_gb_y = c14_fb_y;
  c14_F_tL_r = c14_ic_x / c14_gb_y;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 50);
  c14_alpha = c14_b_a_mL;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 52);
  c14_l_A = c14_Fz;
  c14_jc_x = c14_l_A;
  c14_kc_x = c14_jc_x;
  c14_lc_x = c14_kc_x;
  c14_hb_y = c14_lc_x / 3.0;
  c14_mc_x = c14_alpha;
  c14_nc_x = c14_mc_x;
  c14_nc_x = muDoubleScalarTan(c14_nc_x);
  c14_oc_x = c14_alpha;
  c14_pc_x = c14_oc_x;
  c14_pc_x = muDoubleScalarTan(c14_pc_x);
  c14_qc_x = c14_b_Slip * c14_b_Slip + c14_nc_x * c14_pc_x;
  c14_rc_x = c14_qc_x;
  if (c14_rc_x < 0.0) {
    c14_eml_error(chartInstance);
  }

  c14_rc_x = muDoubleScalarSqrt(c14_rc_x);
  c14_sc_x = c14_alpha;
  c14_tc_x = c14_sc_x;
  c14_tc_x = muDoubleScalarTan(c14_tc_x);
  c14_uc_x = c14_alpha;
  c14_vc_x = c14_uc_x;
  c14_vc_x = muDoubleScalarTan(c14_vc_x);
  c14_wc_x = 4.0E+8 * c14_b_Slip * c14_b_Slip + 4.0E+8 * c14_tc_x * c14_vc_x;
  c14_xc_x = c14_wc_x;
  if (c14_xc_x < 0.0) {
    c14_eml_error(chartInstance);
  }

  c14_xc_x = muDoubleScalarSqrt(c14_xc_x);
  c14_m_A = 0.85 * c14_hb_y * (1.0 - 0.015 * c14_U_tl * c14_rc_x) * (1.0 -
    c14_b_Slip);
  c14_j_B = 2.0 * c14_xc_x;
  c14_yc_x = c14_m_A;
  c14_ib_y = c14_j_B;
  c14_ad_x = c14_yc_x;
  c14_jb_y = c14_ib_y;
  c14_bd_x = c14_ad_x;
  c14_kb_y = c14_jb_y;
  c14_S_m = c14_bd_x / c14_kb_y;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 54);
  if (CV_EML_IF(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 2, c14_S_m, 1.0, -1, 2U,
        c14_S_m < 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 55);
    c14_f_S = c14_S_m * (2.0 - c14_S_m);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 57);
    c14_f_S = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 59);
  c14_cd_x = c14_alpha;
  c14_dd_x = c14_cd_x;
  c14_dd_x = muDoubleScalarTan(c14_dd_x);
  c14_n_A = 20000.0 * c14_dd_x * c14_f_S;
  c14_k_B = 1.0 - c14_b_Slip;
  c14_ed_x = c14_n_A;
  c14_lb_y = c14_k_B;
  c14_fd_x = c14_ed_x;
  c14_mb_y = c14_lb_y;
  c14_gd_x = c14_fd_x;
  c14_nb_y = c14_mb_y;
  c14_b_F_SL_m = c14_gd_x / c14_nb_y;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 60);
  c14_o_A = 20000.0 * c14_b_Slip * c14_f_S;
  c14_l_B = 1.0 - c14_b_Slip;
  c14_hd_x = c14_o_A;
  c14_ob_y = c14_l_B;
  c14_id_x = c14_hd_x;
  c14_pb_y = c14_ob_y;
  c14_jd_x = c14_id_x;
  c14_qb_y = c14_pb_y;
  c14_F_tL_m = c14_jd_x / c14_qb_y;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 63);
  c14_b_F_SL = (c14_b_F_SL_f + c14_b_F_SL_r) + c14_b_F_SL_m;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 64);
  c14_b_F_tL = (c14_F_tL_f + c14_F_tL_r) + c14_F_tL_m;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 65);
  c14_F_L[0] = c14_F_tL_f;
  c14_F_L[1] = c14_F_tL_r;
  c14_F_L[2] = c14_F_tL_m;
  c14_F_L[3] = c14_b_F_SL_f;
  c14_F_L[4] = c14_b_F_SL_r;
  c14_F_L[5] = c14_b_F_SL_m;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, -65);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c14_F_SL = c14_b_F_SL;
  *chartInstance->c14_F_tL = c14_b_F_tL;
  *chartInstance->c14_a_fL = c14_b_a_fL;
  *chartInstance->c14_a_rL = c14_b_a_rL;
  *chartInstance->c14_a_mL = c14_b_a_mL;
  *chartInstance->c14_F_SL_f = c14_b_F_SL_f;
  *chartInstance->c14_F_SL_r = c14_b_F_SL_r;
  *chartInstance->c14_F_SL_m = c14_b_F_SL_m;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
}

static void initSimStructsc14_SS6_Estimation2
  (SFc14_SS6_Estimation2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c14_machineNumber, uint32_T
  c14_chartNumber, uint32_T c14_instanceNumber)
{
  (void)c14_machineNumber;
  (void)c14_chartNumber;
  (void)c14_instanceNumber;
}

static const mxArray *c14_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  real_T c14_u;
  const mxArray *c14_y = NULL;
  SFc14_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc14_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_u = *(real_T *)c14_inData;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", &c14_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, false);
  return c14_mxArrayOutData;
}

static real_T c14_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_b_F_SL_m, const char_T *c14_identifier)
{
  real_T c14_y;
  emlrtMsgIdentifier c14_thisId;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_b_F_SL_m),
    &c14_thisId);
  sf_mex_destroy(&c14_b_F_SL_m);
  return c14_y;
}

static real_T c14_b_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId)
{
  real_T c14_y;
  real_T c14_d4;
  (void)chartInstance;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), &c14_d4, 1, 0, 0U, 0, 0U, 0);
  c14_y = c14_d4;
  sf_mex_destroy(&c14_u);
  return c14_y;
}

static void c14_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_b_F_SL_m;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y;
  SFc14_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc14_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c14_b_F_SL_m = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_b_F_SL_m),
    &c14_thisId);
  sf_mex_destroy(&c14_b_F_SL_m);
  *(real_T *)c14_outData = c14_y;
  sf_mex_destroy(&c14_mxArrayInData);
}

static const mxArray *c14_b_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_i0;
  real_T c14_b_inData[6];
  int32_T c14_i1;
  real_T c14_u[6];
  const mxArray *c14_y = NULL;
  SFc14_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc14_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  for (c14_i0 = 0; c14_i0 < 6; c14_i0++) {
    c14_b_inData[c14_i0] = (*(real_T (*)[6])c14_inData)[c14_i0];
  }

  for (c14_i1 = 0; c14_i1 < 6; c14_i1++) {
    c14_u[c14_i1] = c14_b_inData[c14_i1];
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, false);
  return c14_mxArrayOutData;
}

static void c14_c_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[6])
{
  real_T c14_dv0[6];
  int32_T c14_i2;
  (void)chartInstance;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), c14_dv0, 1, 0, 0U, 1, 0U, 1, 6);
  for (c14_i2 = 0; c14_i2 < 6; c14_i2++) {
    c14_y[c14_i2] = c14_dv0[c14_i2];
  }

  sf_mex_destroy(&c14_u);
}

static void c14_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_F_L;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y[6];
  int32_T c14_i3;
  SFc14_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc14_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c14_F_L = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_F_L), &c14_thisId, c14_y);
  sf_mex_destroy(&c14_F_L);
  for (c14_i3 = 0; c14_i3 < 6; c14_i3++) {
    (*(real_T (*)[6])c14_outData)[c14_i3] = c14_y[c14_i3];
  }

  sf_mex_destroy(&c14_mxArrayInData);
}

const mxArray *sf_c14_SS6_Estimation2_get_eml_resolved_functions_info(void)
{
  const mxArray *c14_nameCaptureInfo = NULL;
  c14_nameCaptureInfo = NULL;
  sf_mex_assign(&c14_nameCaptureInfo, sf_mex_createstruct("structure", 2, 14, 1),
                false);
  c14_info_helper(&c14_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c14_nameCaptureInfo);
  return c14_nameCaptureInfo;
}

static void c14_info_helper(const mxArray **c14_info)
{
  const mxArray *c14_rhs0 = NULL;
  const mxArray *c14_lhs0 = NULL;
  const mxArray *c14_rhs1 = NULL;
  const mxArray *c14_lhs1 = NULL;
  const mxArray *c14_rhs2 = NULL;
  const mxArray *c14_lhs2 = NULL;
  const mxArray *c14_rhs3 = NULL;
  const mxArray *c14_lhs3 = NULL;
  const mxArray *c14_rhs4 = NULL;
  const mxArray *c14_lhs4 = NULL;
  const mxArray *c14_rhs5 = NULL;
  const mxArray *c14_lhs5 = NULL;
  const mxArray *c14_rhs6 = NULL;
  const mxArray *c14_lhs6 = NULL;
  const mxArray *c14_rhs7 = NULL;
  const mxArray *c14_lhs7 = NULL;
  const mxArray *c14_rhs8 = NULL;
  const mxArray *c14_lhs8 = NULL;
  const mxArray *c14_rhs9 = NULL;
  const mxArray *c14_lhs9 = NULL;
  const mxArray *c14_rhs10 = NULL;
  const mxArray *c14_lhs10 = NULL;
  const mxArray *c14_rhs11 = NULL;
  const mxArray *c14_lhs11 = NULL;
  const mxArray *c14_rhs12 = NULL;
  const mxArray *c14_lhs12 = NULL;
  const mxArray *c14_rhs13 = NULL;
  const mxArray *c14_lhs13 = NULL;
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("mrdivide"), "name", "name",
                  0);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1410829248U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c14_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs0), "rhs", "rhs",
                  0);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs0), "lhs", "lhs",
                  0);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 1);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1389739374U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c14_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs1), "rhs", "rhs",
                  1);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs1), "lhs", "lhs",
                  1);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c14_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs2), "rhs", "rhs",
                  2);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs2), "lhs", "lhs",
                  2);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1395949856U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c14_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs3), "rhs", "rhs",
                  3);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs3), "lhs", "lhs",
                  3);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c14_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs4), "rhs", "rhs",
                  4);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs4), "lhs", "lhs",
                  4);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1386445552U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c14_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs5), "rhs", "rhs",
                  5);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs5), "lhs", "lhs",
                  5);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1410829370U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c14_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs6), "rhs", "rhs",
                  6);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs6), "lhs", "lhs",
                  6);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("atan"), "name", "name", 7);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1395346496U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c14_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs7), "rhs", "rhs",
                  7);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs7), "lhs", "lhs",
                  7);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("eml_scalar_atan"), "name",
                  "name", 8);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1286840318U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c14_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs8), "rhs", "rhs",
                  8);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs8), "lhs", "lhs",
                  8);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(""), "context", "context", 9);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("tan"), "name", "name", 9);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/tan.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1395346504U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c14_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs9), "rhs", "rhs",
                  9);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs9), "lhs", "lhs",
                  9);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/tan.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("eml_scalar_tan"), "name",
                  "name", 10);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_tan.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c14_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(""), "context", "context", 11);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("sqrt"), "name", "name", 11);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c14_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("eml_error"), "name", "name",
                  12);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c14_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 13);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c14_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs13), "lhs", "lhs",
                  13);
  sf_mex_destroy(&c14_rhs0);
  sf_mex_destroy(&c14_lhs0);
  sf_mex_destroy(&c14_rhs1);
  sf_mex_destroy(&c14_lhs1);
  sf_mex_destroy(&c14_rhs2);
  sf_mex_destroy(&c14_lhs2);
  sf_mex_destroy(&c14_rhs3);
  sf_mex_destroy(&c14_lhs3);
  sf_mex_destroy(&c14_rhs4);
  sf_mex_destroy(&c14_lhs4);
  sf_mex_destroy(&c14_rhs5);
  sf_mex_destroy(&c14_lhs5);
  sf_mex_destroy(&c14_rhs6);
  sf_mex_destroy(&c14_lhs6);
  sf_mex_destroy(&c14_rhs7);
  sf_mex_destroy(&c14_lhs7);
  sf_mex_destroy(&c14_rhs8);
  sf_mex_destroy(&c14_lhs8);
  sf_mex_destroy(&c14_rhs9);
  sf_mex_destroy(&c14_lhs9);
  sf_mex_destroy(&c14_rhs10);
  sf_mex_destroy(&c14_lhs10);
  sf_mex_destroy(&c14_rhs11);
  sf_mex_destroy(&c14_lhs11);
  sf_mex_destroy(&c14_rhs12);
  sf_mex_destroy(&c14_lhs12);
  sf_mex_destroy(&c14_rhs13);
  sf_mex_destroy(&c14_lhs13);
}

static const mxArray *c14_emlrt_marshallOut(const char * c14_u)
{
  const mxArray *c14_y = NULL;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c14_u)), false);
  return c14_y;
}

static const mxArray *c14_b_emlrt_marshallOut(const uint32_T c14_u)
{
  const mxArray *c14_y = NULL;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", &c14_u, 7, 0U, 0U, 0U, 0), false);
  return c14_y;
}

static real_T c14_sqrt(SFc14_SS6_Estimation2InstanceStruct *chartInstance,
  real_T c14_x)
{
  real_T c14_b_x;
  c14_b_x = c14_x;
  c14_b_sqrt(chartInstance, &c14_b_x);
  return c14_b_x;
}

static void c14_eml_error(SFc14_SS6_Estimation2InstanceStruct *chartInstance)
{
  int32_T c14_i4;
  static char_T c14_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c14_u[30];
  const mxArray *c14_y = NULL;
  int32_T c14_i5;
  static char_T c14_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c14_b_u[4];
  const mxArray *c14_b_y = NULL;
  (void)chartInstance;
  for (c14_i4 = 0; c14_i4 < 30; c14_i4++) {
    c14_u[c14_i4] = c14_cv0[c14_i4];
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 10, 0U, 1U, 0U, 2, 1, 30),
                false);
  for (c14_i5 = 0; c14_i5 < 4; c14_i5++) {
    c14_b_u[c14_i5] = c14_cv1[c14_i5];
  }

  c14_b_y = NULL;
  sf_mex_assign(&c14_b_y, sf_mex_create("y", c14_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c14_y, 14, c14_b_y));
}

static const mxArray *c14_c_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_u;
  const mxArray *c14_y = NULL;
  SFc14_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc14_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_u = *(int32_T *)c14_inData;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", &c14_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, false);
  return c14_mxArrayOutData;
}

static int32_T c14_d_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId)
{
  int32_T c14_y;
  int32_T c14_i6;
  (void)chartInstance;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), &c14_i6, 1, 6, 0U, 0, 0U, 0);
  c14_y = c14_i6;
  sf_mex_destroy(&c14_u);
  return c14_y;
}

static void c14_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_b_sfEvent;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  int32_T c14_y;
  SFc14_SS6_Estimation2InstanceStruct *chartInstance;
  chartInstance = (SFc14_SS6_Estimation2InstanceStruct *)chartInstanceVoid;
  c14_b_sfEvent = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_b_sfEvent),
    &c14_thisId);
  sf_mex_destroy(&c14_b_sfEvent);
  *(int32_T *)c14_outData = c14_y;
  sf_mex_destroy(&c14_mxArrayInData);
}

static uint8_T c14_e_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_b_is_active_c14_SS6_Estimation2, const
  char_T *c14_identifier)
{
  uint8_T c14_y;
  emlrtMsgIdentifier c14_thisId;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c14_b_is_active_c14_SS6_Estimation2), &c14_thisId);
  sf_mex_destroy(&c14_b_is_active_c14_SS6_Estimation2);
  return c14_y;
}

static uint8_T c14_f_emlrt_marshallIn(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId)
{
  uint8_T c14_y;
  uint8_T c14_u0;
  (void)chartInstance;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), &c14_u0, 1, 3, 0U, 0, 0U, 0);
  c14_y = c14_u0;
  sf_mex_destroy(&c14_u);
  return c14_y;
}

static void c14_b_sqrt(SFc14_SS6_Estimation2InstanceStruct *chartInstance,
  real_T *c14_x)
{
  if (*c14_x < 0.0) {
    c14_eml_error(chartInstance);
  }

  *c14_x = muDoubleScalarSqrt(*c14_x);
}

static void init_dsm_address_info(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc14_SS6_Estimation2InstanceStruct
  *chartInstance)
{
  chartInstance->c14_FzL = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c14_F_SL = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c14_F_tL = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c14_U = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c14_V = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c14_Slip = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c14_r = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    4);
  chartInstance->c14_a_fL = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c14_a_rL = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c14_a_mL = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c14_F_SL_f = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c14_F_SL_r = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 7);
  chartInstance->c14_F_SL_m = (real_T *)ssGetOutputPortSignal_wrapper
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

void sf_c14_SS6_Estimation2_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1154274981U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2536049805U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1250073863U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2283542334U);
}

mxArray* sf_c14_SS6_Estimation2_get_post_codegen_info(void);
mxArray *sf_c14_SS6_Estimation2_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("XtLWDp4g3cmASKsPhW5TVH");
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
    mxArray* mxPostCodegenInfo = sf_c14_SS6_Estimation2_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c14_SS6_Estimation2_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c14_SS6_Estimation2_jit_fallback_info(void)
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

mxArray *sf_c14_SS6_Estimation2_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c14_SS6_Estimation2_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c14_SS6_Estimation2(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x9'type','srcId','name','auxInfo'{{M[1],M[5],T\"F_SL\",},{M[1],M[24],T\"F_SL_f\",},{M[1],M[26],T\"F_SL_m\",},{M[1],M[25],T\"F_SL_r\",},{M[1],M[15],T\"F_tL\",},{M[1],M[18],T\"a_fL\",},{M[1],M[21],T\"a_mL\",},{M[1],M[19],T\"a_rL\",},{M[8],M[0],T\"is_active_c14_SS6_Estimation2\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 9, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c14_SS6_Estimation2_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc14_SS6_Estimation2InstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc14_SS6_Estimation2InstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SS6_Estimation2MachineNumber_,
           14,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"FzL");
          _SFD_SET_DATA_PROPS(1,2,0,1,"F_SL");
          _SFD_SET_DATA_PROPS(2,2,0,1,"F_tL");
          _SFD_SET_DATA_PROPS(3,1,1,0,"U");
          _SFD_SET_DATA_PROPS(4,1,1,0,"V");
          _SFD_SET_DATA_PROPS(5,1,1,0,"Slip");
          _SFD_SET_DATA_PROPS(6,1,1,0,"r");
          _SFD_SET_DATA_PROPS(7,2,0,1,"a_fL");
          _SFD_SET_DATA_PROPS(8,2,0,1,"a_rL");
          _SFD_SET_DATA_PROPS(9,2,0,1,"a_mL");
          _SFD_SET_DATA_PROPS(10,2,0,1,"F_SL_f");
          _SFD_SET_DATA_PROPS(11,2,0,1,"F_SL_r");
          _SFD_SET_DATA_PROPS(12,2,0,1,"F_SL_m");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1708);
        _SFD_CV_INIT_EML_IF(0,1,0,888,897,919,938);
        _SFD_CV_INIT_EML_IF(0,1,1,1184,1193,1215,1234);
        _SFD_CV_INIT_EML_IF(0,1,2,1482,1491,1513,1532);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,891,896,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,1187,1192,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,1485,1490,-1,2);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)c14_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)c14_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)c14_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)c14_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)c14_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)c14_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)c14_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)c14_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c14_FzL);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c14_F_SL);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c14_F_tL);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c14_U);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c14_V);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c14_Slip);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c14_r);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c14_a_fL);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c14_a_rL);
        _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c14_a_mL);
        _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c14_F_SL_f);
        _SFD_SET_DATA_VALUE_PTR(11U, chartInstance->c14_F_SL_r);
        _SFD_SET_DATA_VALUE_PTR(12U, chartInstance->c14_F_SL_m);
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
  return "RjoCiBM5jPheWLwnBtCDIE";
}

static void sf_opaque_initialize_c14_SS6_Estimation2(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc14_SS6_Estimation2InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c14_SS6_Estimation2((SFc14_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
  initialize_c14_SS6_Estimation2((SFc14_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c14_SS6_Estimation2(void *chartInstanceVar)
{
  enable_c14_SS6_Estimation2((SFc14_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c14_SS6_Estimation2(void *chartInstanceVar)
{
  disable_c14_SS6_Estimation2((SFc14_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c14_SS6_Estimation2(void *chartInstanceVar)
{
  sf_gateway_c14_SS6_Estimation2((SFc14_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c14_SS6_Estimation2(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c14_SS6_Estimation2((SFc14_SS6_Estimation2InstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c14_SS6_Estimation2(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c14_SS6_Estimation2((SFc14_SS6_Estimation2InstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c14_SS6_Estimation2(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc14_SS6_Estimation2InstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SS6_Estimation2_optimization_info();
    }

    finalize_c14_SS6_Estimation2((SFc14_SS6_Estimation2InstanceStruct*)
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
  initSimStructsc14_SS6_Estimation2((SFc14_SS6_Estimation2InstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c14_SS6_Estimation2(SimStruct *S)
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
    initialize_params_c14_SS6_Estimation2((SFc14_SS6_Estimation2InstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c14_SS6_Estimation2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SS6_Estimation2_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      14);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,14,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,14,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,14);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,14,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,14,8);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,14);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1252726637U));
  ssSetChecksum1(S,(970383298U));
  ssSetChecksum2(S,(3933728023U));
  ssSetChecksum3(S,(2477505309U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c14_SS6_Estimation2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c14_SS6_Estimation2(SimStruct *S)
{
  SFc14_SS6_Estimation2InstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc14_SS6_Estimation2InstanceStruct *)utMalloc(sizeof
    (SFc14_SS6_Estimation2InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc14_SS6_Estimation2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c14_SS6_Estimation2;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c14_SS6_Estimation2;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c14_SS6_Estimation2;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c14_SS6_Estimation2;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c14_SS6_Estimation2;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c14_SS6_Estimation2;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c14_SS6_Estimation2;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c14_SS6_Estimation2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c14_SS6_Estimation2;
  chartInstance->chartInfo.mdlStart = mdlStart_c14_SS6_Estimation2;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c14_SS6_Estimation2;
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

void c14_SS6_Estimation2_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c14_SS6_Estimation2(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c14_SS6_Estimation2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c14_SS6_Estimation2(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c14_SS6_Estimation2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
