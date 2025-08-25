#include <math.h>
#include <vc_or_std_simd.h>  // this includes <Vc/Vc> or a Vc-emulating wrapper of <experimental/simd> if available
#include <iostream> 
/*
   There are a total of 71 entries in the algebraic variable array.
   There are a total of 56 entries in each of the rate and state variable arrays.
   There are a total of 105 entries in the constant variable array.
 */
/*
 * VOI is time in component cell (millisecond).
 * CONSTANTS[0] is C_m in component wal_environment (microF_per_cm2).
 * CONSTANTS[1] is gam in component wal_environment (dimensionless).
 * CONSTANTS[2] is R_a in component wal_environment (ohm_cm2).
 * CONSTANTS[3] is tsi in component wal_environment (centi_metre).
 * CONSTANTS[4] is tsi2 in component wal_environment (centi_metre).
 * CONSTANTS[5] is tsi3 in component wal_environment (centi_metre).
 * CONSTANTS[6] is FF in component wal_environment (C_per_mol).
 * CONSTANTS[7] is tau_K in component wal_environment (millisecond).
 * CONSTANTS[8] is tau_Na in component wal_environment (millisecond).
 * CONSTANTS[9] is f_T in component wal_environment (dimensionless).
 * CONSTANTS[10] is tau_K2 in component wal_environment (millisecond).
 * CONSTANTS[11] is tau_Na2 in component wal_environment (millisecond).
 * CONSTANTS[12] is I_K_rest in component wal_environment (microA_per_cm2).
 * CONSTANTS[13] is I_Na_rest in component wal_environment (microA_per_cm2).
 * CONSTANTS[14] is alpha_h_bar in component wal_environment (per_millisecond).
 * CONSTANTS[15] is alpha_m_bar in component wal_environment (per_millisecond_per_millivolt).
 * CONSTANTS[16] is alpha_n_bar in component wal_environment (per_millisecond_per_millivolt).
 * CONSTANTS[17] is beta_h_bar in component wal_environment (per_millisecond).
 * CONSTANTS[18] is beta_m_bar in component wal_environment (per_millisecond).
 * CONSTANTS[19] is beta_n_bar in component wal_environment (per_millisecond).
 * CONSTANTS[20] is V_m in component wal_environment (millivolt).
 * CONSTANTS[21] is V_n in component wal_environment (millivolt).
 * CONSTANTS[22] is V_h in component wal_environment (millivolt).
 * CONSTANTS[23] is V_a in component wal_environment (millivolt).
 * CONSTANTS[24] is V_S_inf in component wal_environment (millivolt).
 * CONSTANTS[25] is V_h_K_inf in component wal_environment (millivolt).
 * CONSTANTS[26] is A_a in component wal_environment (millivolt).
 * CONSTANTS[27] is A_S_inf in component wal_environment (millivolt).
 * CONSTANTS[28] is A_h_K_inf in component wal_environment (millivolt).
 * CONSTANTS[29] is K_alpha_h in component wal_environment (millivolt).
 * CONSTANTS[30] is K_beta_h in component wal_environment (millivolt).
 * CONSTANTS[31] is K_alpha_m in component wal_environment (millivolt).
 * CONSTANTS[32] is K_alpha_n in component wal_environment (millivolt).
 * CONSTANTS[33] is K_beta_m in component wal_environment (millivolt).
 * CONSTANTS[34] is K_beta_n in component wal_environment (millivolt).
 * CONSTANTS[35] is RR in component wal_environment (milliJ_per_degreeK_per_mol).
 * CONSTANTS[36] is TT in component wal_environment (degreeK).
 * CONSTANTS[37] is g_Cl_bar in component wal_environment (milliS_per_cm2).
 * CONSTANTS[38] is g_K_bar in component wal_environment (milliS_per_cm2).
 * CONSTANTS[39] is g_Na_bar in component wal_environment (milliS_per_cm2).
 * CONSTANTS[40] is G_K in component wal_environment (milliS_per_cm2).
 * CONSTANTS[41] is del in component wal_environment (dimensionless).
 * CONSTANTS[42] is K_K in component wal_environment (milliM2).
 * CONSTANTS[43] is K_S in component wal_environment (milliM2).
 * CONSTANTS[44] is K_m_K in component wal_environment (milliM).
 * CONSTANTS[45] is K_m_Na in component wal_environment (milliM).
 * CONSTANTS[46] is S_i in component wal_environment (milliM).
 * CONSTANTS[47] is J_NaK_bar in component wal_environment (micro_mol_per_cm2_per_second).
 * CONSTANTS[48] is V_tau in component wal_environment (millivolt).
 * ALGEBRAIC[0] is I_T in component wal_environment (microA_per_cm2).
 * STATES[0] is vS in component wal_environment (millivolt).
 * STATES[1] is vT in component wal_environment (millivolt).
 * ALGEBRAIC[51] is I_ionic_s in component wal_environment (microA_per_cm2).
 * ALGEBRAIC[70] is I_ionic_t in component wal_environment (microA_per_cm2).
 * STATES[2] is K_t in component wal_environment (milliM).
 * STATES[3] is K_i in component wal_environment (milliM).
 * STATES[4] is K_e in component wal_environment (milliM).
 * STATES[5] is Na_i in component wal_environment (milliM).
 * STATES[6] is Na_t in component wal_environment (milliM).
 * STATES[7] is Na_e in component wal_environment (milliM).
 * ALGEBRAIC[13] is E_K in component wal_environment (millivolt).
 * ALGEBRAIC[25] is E_K_t in component wal_environment (millivolt).
 * ALGEBRAIC[26] is Cl_i in component wal_environment (milliM).
 * ALGEBRAIC[27] is Cl_o in component wal_environment (milliM).
 * ALGEBRAIC[28] is Cl_i_t in component wal_environment (milliM).
 * ALGEBRAIC[29] is Cl_o_t in component wal_environment (milliM).
 * ALGEBRAIC[30] is J_K in component wal_environment (milliV_milliM).
 * ALGEBRAIC[31] is J_K_t in component wal_environment (milliV_milliM).
 * CONSTANTS[49] is eta_Cl in component wal_environment (dimensionless).
 * CONSTANTS[50] is eta_IR in component wal_environment (dimensionless).
 * CONSTANTS[51] is eta_DR in component wal_environment (dimensionless).
 * CONSTANTS[52] is eta_Na in component wal_environment (dimensionless).
 * CONSTANTS[53] is eta_NaK in component wal_environment (dimensionless).
 * ALGEBRAIC[36] is I_Cl in component sarco_Cl_channel (microA_per_cm2).
 * ALGEBRAIC[41] is I_IR in component sarco_IR_channel (microA_per_cm2).
 * ALGEBRAIC[43] is I_DR in component sarco_DR_channel (microA_per_cm2).
 * ALGEBRAIC[46] is I_Na in component sarco_Na_channel (microA_per_cm2).
 * ALGEBRAIC[50] is I_NaK in component sarco_NaK_channel (microA_per_cm2).
 * ALGEBRAIC[55] is I_Cl_t in component t_Cl_channel (microA_per_cm2).
 * ALGEBRAIC[60] is I_IR_t in component t_IR_channel (microA_per_cm2).
 * ALGEBRAIC[62] is I_DR_t in component t_DR_channel (microA_per_cm2).
 * ALGEBRAIC[65] is I_Na_t in component t_Na_channel (microA_per_cm2).
 * ALGEBRAIC[69] is I_NaK_t in component t_NaK_channel (microA_per_cm2).
 * ALGEBRAIC[32] is I_HH in component wal_environment (microA_per_cm2).
 * ALGEBRAIC[33] is a in component sarco_Cl_channel (dimensionless).
 * ALGEBRAIC[34] is J_Cl in component sarco_Cl_channel (milliV_milliM).
 * ALGEBRAIC[35] is g_Cl in component sarco_Cl_channel (milliS_per_cm2).
 * ALGEBRAIC[37] is K_R in component sarco_IR_channel (milliM).
 * ALGEBRAIC[38] is g_IR_bar in component sarco_IR_channel (milliS_per_cm2).
 * ALGEBRAIC[39] is y in component sarco_IR_channel (dimensionless).
 * ALGEBRAIC[40] is g_IR in component sarco_IR_channel (milliS_per_cm2).
 * ALGEBRAIC[1] is alpha_n in component sarco_DR_channel (per_millisecond).
 * ALGEBRAIC[14] is beta_n in component sarco_DR_channel (per_millisecond).
 * ALGEBRAIC[2] is h_K_inf in component sarco_DR_channel (dimensionless).
 * ALGEBRAIC[15] is tau_h_K in component sarco_DR_channel (millisecond).
 * STATES[8] is n in component sarco_DR_channel (dimensionless).
 * STATES[9] is h_K in component sarco_DR_channel (dimensionless).
 * ALGEBRAIC[42] is g_DR in component sarco_DR_channel (milliS_per_cm2).
 * ALGEBRAIC[3] is alpha_h in component sarco_Na_channel (per_millisecond).
 * ALGEBRAIC[16] is beta_h in component sarco_Na_channel (per_millisecond).
 * ALGEBRAIC[4] is alpha_m in component sarco_Na_channel (per_millisecond).
 * ALGEBRAIC[17] is beta_m in component sarco_Na_channel (per_millisecond).
 * ALGEBRAIC[5] is S_inf in component sarco_Na_channel (dimensionless).
 * ALGEBRAIC[18] is tau_S in component sarco_Na_channel (millisecond).
 * STATES[10] is m in component sarco_Na_channel (dimensionless).
 * STATES[11] is h in component sarco_Na_channel (dimensionless).
 * STATES[12] is S in component sarco_Na_channel (dimensionless).
 * ALGEBRAIC[45] is g_Na in component sarco_Na_channel (milliS_per_cm2).
 * ALGEBRAIC[44] is J_Na in component sarco_Na_channel (milliV_milliM).
 * ALGEBRAIC[47] is sig in component sarco_NaK_channel (dimensionless).
 * ALGEBRAIC[48] is f1 in component sarco_NaK_channel (dimensionless).
 * ALGEBRAIC[49] is I_NaK_bar in component sarco_NaK_channel (microA_per_cm2).
 * ALGEBRAIC[52] is a_t in component t_Cl_channel (dimensionless).
 * ALGEBRAIC[53] is J_Cl_t in component t_Cl_channel (milliV_milliM).
 * ALGEBRAIC[54] is g_Cl_t in component t_Cl_channel (milliS_per_cm2).
 * ALGEBRAIC[56] is K_R_t in component t_IR_channel (milliM).
 * ALGEBRAIC[57] is g_IR_bar_t in component t_IR_channel (milliS_per_cm2).
 * ALGEBRAIC[58] is y_t in component t_IR_channel (dimensionless).
 * ALGEBRAIC[59] is g_IR_t in component t_IR_channel (milliS_per_cm2).
 * ALGEBRAIC[6] is alpha_n_t in component t_DR_channel (per_millisecond).
 * ALGEBRAIC[19] is beta_n_t in component t_DR_channel (per_millisecond).
 * ALGEBRAIC[7] is h_K_inf_t in component t_DR_channel (dimensionless).
 * ALGEBRAIC[20] is tau_h_K_t in component t_DR_channel (millisecond).
 * STATES[13] is n_t in component t_DR_channel (dimensionless).
 * STATES[14] is h_K_t in component t_DR_channel (dimensionless).
 * ALGEBRAIC[61] is g_DR_t in component t_DR_channel (milliS_per_cm2).
 * ALGEBRAIC[8] is alpha_h_t in component t_Na_channel (per_millisecond).
 * ALGEBRAIC[21] is beta_h_t in component t_Na_channel (per_millisecond).
 * ALGEBRAIC[9] is alpha_m_t in component t_Na_channel (per_millisecond).
 * ALGEBRAIC[22] is beta_m_t in component t_Na_channel (per_millisecond).
 * ALGEBRAIC[10] is S_inf_t in component t_Na_channel (dimensionless).
 * ALGEBRAIC[23] is tau_S_t in component t_Na_channel (millisecond).
 * STATES[15] is m_t in component t_Na_channel (dimensionless).
 * STATES[16] is h_t in component t_Na_channel (dimensionless).
 * STATES[17] is S_t in component t_Na_channel (dimensionless).
 * ALGEBRAIC[64] is g_Na_t in component t_Na_channel (milliS_per_cm2).
 * ALGEBRAIC[63] is J_Na_t in component t_Na_channel (milliV_milliM).
 * ALGEBRAIC[66] is sig_t in component t_NaK_channel (dimensionless).
 * ALGEBRAIC[67] is f1_t in component t_NaK_channel (dimensionless).
 * ALGEBRAIC[68] is I_NaK_bar_t in component t_NaK_channel (microA_per_cm2).
 * STATES[18] is O_0 in component sternrios (dimensionless).
 * STATES[19] is O_1 in component sternrios (dimensionless).
 * STATES[20] is O_2 in component sternrios (dimensionless).
 * STATES[21] is O_3 in component sternrios (dimensionless).
 * STATES[22] is O_4 in component sternrios (dimensionless).
 * CONSTANTS[54] is k_L in component sternrios (per_millisecond).
 * CONSTANTS[55] is k_Lm in component sternrios (per_millisecond).
 * CONSTANTS[56] is f in component sternrios (dimensionless).
 * CONSTANTS[57] is alpha1 in component sternrios (per_millisecond).
 * CONSTANTS[58] is K in component sternrios (millivolt).
 * CONSTANTS[59] is Vbar in component sternrios (millivolt).
 * STATES[23] is C_0 in component sternrios (dimensionless).
 * STATES[24] is C_1 in component sternrios (dimensionless).
 * STATES[25] is C_2 in component sternrios (dimensionless).
 * STATES[26] is C_3 in component sternrios (dimensionless).
 * STATES[27] is C_4 in component sternrios (dimensionless).
 * ALGEBRAIC[11] is k_C in component sternrios (per_millisecond).
 * ALGEBRAIC[24] is k_Cm in component sternrios (per_millisecond).
 * CONSTANTS[60] is nu_SR in component razumova (micromolar_per_millisecond_micrometre3).
 * CONSTANTS[61] is K_SR in component razumova (micromolar).
 * CONSTANTS[62] is L_e in component razumova (micrometre3_per_millisecond).
 * CONSTANTS[63] is tau_R in component razumova (micrometre3_per_millisecond).
 * CONSTANTS[64] is tau_SR_R in component razumova (micrometre3_per_millisecond).
 * CONSTANTS[65] is L_x in component razumova (micrometre).
 * CONSTANTS[66] is R_R in component razumova (micrometre).
 * CONSTANTS[99] is V_o in component razumova (micrometre3).
 * CONSTANTS[101] is V_1 in component razumova (micrometre3).
 * CONSTANTS[102] is V_2 in component razumova (micrometre3).
 * CONSTANTS[100] is V_SR in component razumova (micrometre3).
 * CONSTANTS[103] is V_SR1 in component razumova (micrometre3).
 * CONSTANTS[104] is V_SR2 in component razumova (micrometre3).
 * CONSTANTS[67] is k_T_on in component razumova (per_micromolar_per_millisecond).
 * CONSTANTS[68] is k_T_off in component razumova (per_millisecond).
 * CONSTANTS[69] is T_tot in component razumova (micromolar).
 * CONSTANTS[70] is k_P_on in component razumova (per_micromolar_per_millisecond).
 * CONSTANTS[71] is k_P_off in component razumova (per_millisecond).
 * CONSTANTS[72] is P_tot in component razumova (micromolar).
 * CONSTANTS[73] is k_Mg_on in component razumova (per_micromolar_per_millisecond).
 * CONSTANTS[74] is k_Mg_off in component razumova (per_millisecond).
 * CONSTANTS[75] is k_Cs_on in component razumova (per_micromolar_per_millisecond).
 * CONSTANTS[76] is k_Cs_off in component razumova (per_millisecond).
 * CONSTANTS[77] is Cs_tot in component razumova (micromolar).
 * CONSTANTS[78] is k_CATP_on in component razumova (per_micromolar_per_millisecond).
 * CONSTANTS[79] is k_CATP_off in component razumova (per_millisecond).
 * CONSTANTS[80] is k_MATP_on in component razumova (per_micromolar_per_millisecond).
 * CONSTANTS[81] is k_MATP_off in component razumova (per_millisecond).
 * CONSTANTS[82] is tau_ATP in component razumova (micrometre3_per_millisecond).
 * CONSTANTS[83] is tau_Mg in component razumova (micrometre3_per_millisecond).
 * CONSTANTS[84] is k_0_on in component razumova (per_millisecond).
 * CONSTANTS[85] is k_0_off in component razumova (per_millisecond).
 * CONSTANTS[86] is k_Ca_on in component razumova (per_millisecond).
 * CONSTANTS[87] is k_Ca_off in component razumova (per_millisecond).
 * CONSTANTS[88] is f_o in component razumova (per_millisecond).
 * CONSTANTS[89] is f_p in component razumova (per_millisecond).
 * CONSTANTS[90] is h_o in component razumova (per_millisecond).
 * CONSTANTS[91] is h_p in component razumova (per_millisecond).
 * CONSTANTS[92] is g_o in component razumova (per_millisecond).
 * CONSTANTS[93] is b_p in component razumova (per_millisecond).
 * CONSTANTS[94] is k_p in component razumova (micrometre3_per_millisecond).
 * CONSTANTS[95] is A_p in component razumova (per_milliM3_per_millisecond).
 * CONSTANTS[96] is B_p in component razumova (per_milliM2_per_millisecond).
 * CONSTANTS[97] is PP in component razumova (milliM2).
 * ALGEBRAIC[12] is T_0 in component razumova (micromolar).
 * STATES[28] is Ca_1 in component razumova (micromolar).
 * STATES[29] is Ca_SR1 in component razumova (micromolar).
 * STATES[30] is Ca_2 in component razumova (micromolar).
 * STATES[31] is Ca_SR2 in component razumova (micromolar).
 * STATES[32] is Ca_T_2 in component razumova (micromolar).
 * STATES[33] is Ca_P1 in component razumova (micromolar).
 * STATES[34] is Ca_P2 in component razumova (micromolar).
 * STATES[35] is Mg_P1 in component razumova (micromolar).
 * STATES[36] is Mg_P2 in component razumova (micromolar).
 * STATES[37] is Ca_Cs1 in component razumova (micromolar).
 * STATES[38] is Ca_Cs2 in component razumova (micromolar).
 * STATES[39] is Ca_ATP1 in component razumova (micromolar).
 * STATES[40] is Ca_ATP2 in component razumova (micromolar).
 * STATES[41] is Mg_ATP1 in component razumova (micromolar).
 * STATES[42] is Mg_ATP2 in component razumova (micromolar).
 * STATES[43] is ATP1 in component razumova (micromolar).
 * STATES[44] is ATP2 in component razumova (micromolar).
 * STATES[45] is Mg1 in component razumova (micromolar).
 * STATES[46] is Mg2 in component razumova (micromolar).
 * STATES[47] is Ca_CaT2 in component razumova (micromolar).
 * STATES[48] is D_0 in component razumova (micromolar).
 * STATES[49] is D_1 in component razumova (micromolar).
 * STATES[50] is D_2 in component razumova (micromolar).
 * STATES[51] is A_1 in component razumova (micromolar).
 * STATES[52] is A_2 in component razumova (micromolar).
 * STATES[53] is P in component razumova (milliM).
 * STATES[54] is P_SR in component razumova (milliM).
 * STATES[55] is P_C_SR in component razumova (milliM).
 * CONSTANTS[98] is i2 in component razumova (micrometre3_per_millisecond).
 * RATES[0] is d/dt vS in component wal_environment (millivolt).
 * RATES[1] is d/dt vT in component wal_environment (millivolt).
 * RATES[3] is d/dt K_i in component wal_environment (milliM).
 * RATES[2] is d/dt K_t in component wal_environment (milliM).
 * RATES[4] is d/dt K_e in component wal_environment (milliM).
 * RATES[5] is d/dt Na_i in component wal_environment (milliM).
 * RATES[6] is d/dt Na_t in component wal_environment (milliM).
 * RATES[7] is d/dt Na_e in component wal_environment (milliM).
 * RATES[8] is d/dt n in component sarco_DR_channel (dimensionless).
 * RATES[9] is d/dt h_K in component sarco_DR_channel (dimensionless).
 * RATES[10] is d/dt m in component sarco_Na_channel (dimensionless).
 * RATES[11] is d/dt h in component sarco_Na_channel (dimensionless).
 * RATES[12] is d/dt S in component sarco_Na_channel (dimensionless).
 * RATES[13] is d/dt n_t in component t_DR_channel (dimensionless).
 * RATES[14] is d/dt h_K_t in component t_DR_channel (dimensionless).
 * RATES[15] is d/dt m_t in component t_Na_channel (dimensionless).
 * RATES[16] is d/dt h_t in component t_Na_channel (dimensionless).
 * RATES[17] is d/dt S_t in component t_Na_channel (dimensionless).
 * RATES[23] is d/dt C_0 in component sternrios (dimensionless).
 * RATES[18] is d/dt O_0 in component sternrios (dimensionless).
 * RATES[24] is d/dt C_1 in component sternrios (dimensionless).
 * RATES[19] is d/dt O_1 in component sternrios (dimensionless).
 * RATES[25] is d/dt C_2 in component sternrios (dimensionless).
 * RATES[20] is d/dt O_2 in component sternrios (dimensionless).
 * RATES[26] is d/dt C_3 in component sternrios (dimensionless).
 * RATES[21] is d/dt O_3 in component sternrios (dimensionless).
 * RATES[27] is d/dt C_4 in component sternrios (dimensionless).
 * RATES[22] is d/dt O_4 in component sternrios (dimensionless).
 * RATES[28] is d/dt Ca_1 in component razumova (micromolar).
 * RATES[29] is d/dt Ca_SR1 in component razumova (micromolar).
 * RATES[30] is d/dt Ca_2 in component razumova (micromolar).
 * RATES[31] is d/dt Ca_SR2 in component razumova (micromolar).
 * RATES[32] is d/dt Ca_T_2 in component razumova (micromolar).
 * RATES[33] is d/dt Ca_P1 in component razumova (micromolar).
 * RATES[34] is d/dt Ca_P2 in component razumova (micromolar).
 * RATES[35] is d/dt Mg_P1 in component razumova (micromolar).
 * RATES[36] is d/dt Mg_P2 in component razumova (micromolar).
 * RATES[37] is d/dt Ca_Cs1 in component razumova (micromolar).
 * RATES[38] is d/dt Ca_Cs2 in component razumova (micromolar).
 * RATES[39] is d/dt Ca_ATP1 in component razumova (micromolar).
 * RATES[40] is d/dt Ca_ATP2 in component razumova (micromolar).
 * RATES[41] is d/dt Mg_ATP1 in component razumova (micromolar).
 * RATES[42] is d/dt Mg_ATP2 in component razumova (micromolar).
 * RATES[43] is d/dt ATP1 in component razumova (micromolar).
 * RATES[44] is d/dt ATP2 in component razumova (micromolar).
 * RATES[45] is d/dt Mg1 in component razumova (micromolar).
 * RATES[46] is d/dt Mg2 in component razumova (micromolar).
 * RATES[47] is d/dt Ca_CaT2 in component razumova (micromolar).
 * RATES[48] is d/dt D_0 in component razumova (micromolar).
 * RATES[49] is d/dt D_1 in component razumova (micromolar).
 * RATES[50] is d/dt D_2 in component razumova (micromolar).
 * RATES[51] is d/dt A_1 in component razumova (micromolar).
 * RATES[52] is d/dt A_2 in component razumova (micromolar).
 * RATES[53] is d/dt P in component razumova (milliM).
 * RATES[54] is d/dt P_SR in component razumova (milliM).
 * RATES[55] is d/dt P_C_SR in component razumova (milliM).
 */

using Vc::double_v; 

/* This file was created by opendihu at 2025/8/25 17:41:48.
 * It is designed for the FastMonodomainSolver.
  */

// helper functions
Vc::double_v exponential(Vc::double_v x);
Vc::double_v logarithmic(Vc::double_v x);
Vc::double_v pow2(Vc::double_v x);
Vc::double_v pow3(Vc::double_v x);
Vc::double_v pow4(Vc::double_v x);
Vc::double_v powReciprocal1(Vc::double_v x);

Vc::double_v exponential(Vc::double_v x)
{
  return Vc::exp(x);
}

Vc::double_v logarithmic(Vc::double_v x)
{
  return Vc::log(x);
}

Vc::double_v pow2(Vc::double_v x)
{
  return x*x;
}
Vc::double_v pow3(Vc::double_v x)
{
  return x*(pow2(x));
}

Vc::double_v pow4(Vc::double_v x)
{
  return pow2(pow2(x));
}

Vc::double_v powReciprocal1(Vc::double_v x)
{
  return 1./(x);
}

// set initial values for all states
#ifdef __cplusplus
extern "C"
#endif

void initializeStates(Vc::double_v states[]) 
{
  states[0] = -79.974;
  states[1] = -80.2;
  states[2] = 5.9;
  states[3] = 150.9;
  states[4] = 5.9;
  states[5] = 12.7;
  states[6] = 133;
  states[7] = 133;
  states[8] = 0.009466;
  states[9] = 0.9952;
  states[10] = 0.0358;
  states[11] = 0.4981;
  states[12] = 0.581;
  states[13] = 0.009466;
  states[14] = 0.9952;
  states[15] = 0.0358;
  states[16] = 0.4981;
  states[17] = 0.581;
  states[18] = 0;
  states[19] = 0;
  states[20] = 0;
  states[21] = 0;
  states[22] = 0;
  states[23] = 1;
  states[24] = 0;
  states[25] = 0;
  states[26] = 0;
  states[27] = 0;
  states[28] = 0.1;
  states[29] = 1500;
  states[30] = 0.1;
  states[31] = 1500;
  states[32] = 25;
  states[33] = 615;
  states[34] = 615;
  states[35] = 811;
  states[36] = 811;
  states[37] = 16900;
  states[38] = 16900;
  states[39] = 0.4;
  states[40] = 0.4;
  states[41] = 7200;
  states[42] = 7200;
  states[43] = 799.6;
  states[44] = 799.6;
  states[45] = 1000;
  states[46] = 1000;
  states[47] = 3;
  states[48] = 0.8;
  states[49] = 1.2;
  states[50] = 3;
  states[51] = 0.3;
  states[52] = 0.23;
  states[53] = 0.23;
  states[54] = 0.23;
  states[55] = 0.23;
}

// compute one Heun step
#ifdef __cplusplus
extern "C"
#endif

void compute0DInstance(Vc::double_v states[], std::vector<Vc::double_v> &parameters, double currentTime, double timeStepWidth, bool stimulate,
                       bool storeAlgebraicsForTransfer, std::vector<Vc::double_v> &algebraicsForTransfer, const std::vector<int> &algebraicsForTransferIndices, double valueForStimulatedPoint) 
{
  // assert that Vc::double_v::size() is the same as in opendihu, otherwise there will be problems
  if (Vc::double_v::size() != 4)
  {
    std::cout << "Fatal error in compiled library of source file \"src/shorten_ocallaghan_davidson_soboleva_2007_fast_monodomain.c\", size of SIMD register in compiled code (" << Vc::double_v::size() << ") does not match opendihu code (4)." << std::endl;
    std::cout << "Delete library such that it will be regenerated with the correct compile options!" << std::endl;
    exit(1);
  }

  // define constants
  const double constant0 = 1.0;
  const double constant1 = 4.8;
  const double constant2 = 150;
  const double constant3 = 0.000001;
  const double constant4 = 0.0025;
  const double constant5 = 0.0005;
  const double constant6 = 96485;
  const double constant7 = 350;
  const double constant8 = 350;
  const double constant9 = 0.0032;
  const double constant10 = 21875;
  const double constant11 = 21875;
  const double constant12 = 1.02;
  const double constant13 = -1.29;
  const double constant14 = 0.0081;
  const double constant15 = 0.288;
  const double constant16 = 0.0131;
  const double constant17 = 4.38;
  const double constant18 = 1.38;
  const double constant19 = 0.067;
  const double constant20 = -46;
  const double constant21 = -40;
  const double constant22 = -45;
  const double constant23 = 70;
  const double constant24 = -78;
  const double constant25 = -40;
  const double constant26 = 150;
  const double constant27 = 5.8;
  const double constant28 = 7.5;
  const double constant29 = 14.7;
  const double constant30 = 9;
  const double constant31 = 10;
  const double constant32 = 7;
  const double constant33 = 18;
  const double constant34 = 40;
  const double constant35 = 8314.41;
  const double constant36 = 293;
  const double constant37 = 19.65;
  const double constant38 = 64.8;
  const double constant39 = 804;
  const double constant40 = 11.1;
  const double constant41 = 0.4;
  const double constant42 = 950;
  const double constant43 = 1;
  const double constant44 = 1;
  const double constant45 = 13;
  const double constant46 = 10;
  const double constant47 = 0.000621;
  const double constant48 = 90;
  const double constant49 = 0.1;
  const double constant50 = 1.0;
  const double constant51 = 0.45;
  const double constant52 = 0.1;
  const double constant53 = 0.1;
  const double constant54 = 0.002;
  const double constant55 = 1000;
  const double constant56 = 0.2;
  const double constant57 = 0.2;
  const double constant58 = 4.5;
  const double constant59 = -20;
  const double constant60 = 4.875;
  const double constant61 = 1;
  const double constant62 = 0.00002;
  const double constant63 = 0.75;
  const double constant64 = 0.75;
  const double constant65 = 1.1;
  const double constant66 = 0.5;
  const double constant67 = 0.04425;
  const double constant68 = 0.115;
  const double constant69 = 140;
  const double constant70 = 0.0417;
  const double constant71 = 0.0005;
  const double constant72 = 1500;
  const double constant73 = 0.000033;
  const double constant74 = 0.003;
  const double constant75 = 0.000004;
  const double constant76 = 0.005;
  const double constant77 = 31000;
  const double constant78 = 0.15;
  const double constant79 = 30;
  const double constant80 = 0.0015;
  const double constant81 = 0.15;
  const double constant82 = 0.375;
  const double constant83 = 1.5;
  const double constant84 = 0;
  const double constant85 = 0.15;
  const double constant86 = 0.15;
  const double constant87 = 0.05;
  const double constant88 = 1.5;
  const double constant89 = 15;
  const double constant90 = 0.24;
  const double constant91 = 0.18;
  const double constant92 = 0.12;
  const double constant93 = 0.00002867;
  const double constant94 = 0.00000362;
  const double constant95 = 1;
  const double constant96 = 0.0001;
  const double constant97 = 6;
  const double constant98 = 300;
  const double constant99 =  0.950000*constant65* 3.14159265358979*pow(constant66, 2.00000);
  const double constant100 =  0.0500000*constant65* 3.14159265358979*pow(constant66, 2.00000);
  const double constant101 =  0.0100000*constant99;
  const double constant102 =  0.990000*constant99;
  const double constant103 =  0.0100000*constant100;
  const double constant104 =  0.990000*constant100;

  // compute new rates, rhs(y_n)
  const double_v rate28 = (((( ( constant98*(states[18]+states[19]+states[20]+states[21]+states[22]))*((states[29] - states[28])/constant101) -  constant60*((states[28]/(states[28]+constant61))/constant101))+ constant62*((states[29] - states[28])/constant101))+ - constant63*((states[28] - states[30])/constant101))+- ( ( constant70*states[28])*((constant72+- states[33])+- states[35])+ - constant71*states[33]))+- ( ( constant78*states[28])*states[43]+ - constant79*states[39]);
  const double_v rate29 = ((( - ( constant98*(states[18]+states[19]+states[20]+states[21]+states[22]))*((states[29] - states[28])/constant103)+ constant60*((states[28]/(states[28]+constant61))/constant103))+ - constant62*((states[29] - states[28])/constant103))+ - constant64*((states[29] - states[31])/constant103))+- ( ( constant75*states[29])*(constant77 - states[37])+ - constant76*states[37]);
  const double_v rate31 = ((( constant60*((states[30]/(states[30]+constant61))/constant104)+ - constant62*((states[31]+- states[30])/constant104))+ constant64*((states[29]+- states[31])/constant104))+- ( ( constant75*states[31])*(constant77+- states[38])+ - constant76*states[38])) -  (1000.00/1.00000)*( constant95*( states[54]*(0.00100000/1.00000)*states[31] - constant97)*(Vc::iif( states[54]*(0.00100000/1.00000)*states[31] - constant97>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero)))*(0.00100000/1.00000)*states[54]*states[31] -  constant96*states[55]*(constant97 -  states[54]*(0.00100000/1.00000)*states[31])*(Vc::iif(constant97 -  states[54]*(0.00100000/1.00000)*states[31]>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero))));
  const double_v rate33 =  ( constant70*states[28])*((constant72+- states[33])+- states[35])+ - constant71*states[33];
  const double_v rate34 =  ( constant70*states[30])*((constant72+- states[34])+- states[36])+ - constant71*states[34];
  const double_v rate35 =  ( constant73*(constant72+- states[33]+- states[35]))*states[45]+ - constant74*states[35];
  const double_v rate36 =  ( constant73*(constant72+- states[34]+- states[36]))*states[46]+ - constant74*states[36];
  const double_v rate37 =  ( constant75*states[29])*(constant77+- states[37])+ - constant76*states[37];
  const double_v rate38 =  ( constant75*states[31])*(constant77+- states[38])+ - constant76*states[38];
  const double_v rate39 = ( ( constant78*states[28])*states[43]+ - constant79*states[39])+ - constant82*((states[39]+- states[40])/constant101);
  const double_v rate40 = ( ( constant78*states[30])*states[44]+ - constant79*states[40])+ constant82*((states[39]+- states[40])/constant102);
  const double_v rate41 = ( ( constant80*states[45])*states[43]+ - constant81*states[41])+ - constant82*((states[41]+- states[42])/constant101);
  const double_v rate42 = ( ( constant80*states[46])*states[44]+ - constant81*states[42])+ constant82*((states[41]+- states[42])/constant102);
  const double_v rate43 = (- ( ( constant78*states[28])*states[43]+ - constant79*states[39])+- ( ( constant80*states[45])*states[43]+ - constant81*states[41]))+ - constant82*((states[43]+- states[44])/constant101);
  const double_v rate44 = (- ( ( constant78*states[30])*states[44]+ - constant79*states[40])+- ( ( constant80*states[46])*states[44]+ - constant81*states[42]))+ constant82*((states[43]+- states[44])/constant102);
  const double_v rate45 = (- ( ( constant73*(constant72+- states[33]+- states[35]))*states[45]+ - constant74*states[35])+- ( ( constant80*states[45])*states[43]+ - constant81*states[41]))+ - constant83*((states[45]+- states[46])/constant101);
  const double_v rate46 = (- ( ( constant73*(constant72+- states[34]+- states[36]))*states[46]+ - constant74*states[36])+- ( ( constant80*states[46])*states[44]+ - constant81*states[42]))+ constant83*((states[45]+- states[46])/constant102);
  const double_v rate47 = (( ( constant67*states[30])*states[32]+ - constant68*states[47])+ - constant86*states[47])+ constant87*states[50];
  const double_v rate49 = (((( constant67*states[30]*states[48]+ - constant68*states[49])+ constant84*states[32])+ - constant85*states[49])+ ( - constant67*states[30])*states[49])+ constant68*states[50];
  const double_v rate50 = ((((( constant67*states[30]*states[49]+ - constant68*states[50])+ constant86*states[47])+ - constant87*states[50])+ - constant88*states[50])+ constant89*states[51])+ constant92*states[52];
  const double_v rate51 = (( constant88*states[50]+ - constant89*states[51])+ constant91*states[52])+ - constant90*states[51];
  const double_v rate52 = ( - constant91*states[52]+ constant90*states[51])+ - constant92*states[52];
  const double_v rate53 =  (0.00100000/1.00000)*( constant90*states[51] -  constant91*states[52])+ -1.00000*constant93*states[53]+ -1.00000*constant94*((states[53] - states[54])/constant102);
  const double_v rate54 =  constant94*((states[53] - states[54])/constant104) -  1.00000*( constant95*( states[54]*(0.00100000/1.00000)*states[31] - constant97)*(Vc::iif( states[54]*(0.00100000/1.00000)*states[31] - constant97>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero)))*(0.00100000/1.00000)*states[54]*states[31] -  constant96*states[55]*(constant97 -  states[54]*(0.00100000/1.00000)*states[31])*(Vc::iif(constant97 -  states[54]*(0.00100000/1.00000)*states[31]>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero))));
  const double_v rate55 =  1.00000*( constant95*( states[54]*(0.00100000/1.00000)*states[31] - constant97)*(Vc::iif( states[54]*(0.00100000/1.00000)*states[31] - constant97>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero)))*(0.00100000/1.00000)*states[54]*states[31] -  constant96*states[55]*(constant97 -  states[54]*(0.00100000/1.00000)*states[31])*(Vc::iif(constant97 -  states[54]*(0.00100000/1.00000)*states[31]>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero))));
  const double_v algebraic12 = constant69+- states[32]+- states[47]+- states[48]+- states[49]+- states[50]+- states[51]+- states[52];
  const double_v rate30 = (((( - constant60*((states[30]/(states[30]+constant61))/constant102)+ constant62*((states[31]+- states[30])/constant102))+ constant63*((states[28] - states[30])/constant102))+- ((((((( constant67*states[30]*algebraic12+ - constant68*states[32])+ constant67*states[30]*states[32])+ - constant68*states[47])+ constant67*states[30]*states[48])+ - constant68*states[49])+ constant67*states[30]*states[49])+ - constant68*states[50]))+- ( ( constant70*states[30])*(constant72+- states[34]+- states[36])+ - constant71*states[34]))+- ( ( constant78*states[30])*states[44]+ - constant79*states[40]);
  const double_v rate32 = (((( ( constant67*states[30])*algebraic12+ - constant68*states[32])+ ( - constant67*states[30])*states[32])+ constant68*states[47])+ - constant84*states[32])+ constant85*states[49];
  const double_v rate48 = (( ( - constant67*states[30])*states[48]+ constant68*states[49])+ constant84*algebraic12)+ - constant85*states[48];
  const double_v algebraic1 =  constant16*((states[0] - constant21)/(1.00000 - exponential(- ((states[0] - constant21)/constant32))));
  const double_v algebraic14 =  constant19*exponential(- ((states[0] - constant21)/constant34));
  const double_v rate8 =  algebraic1*(1.00000 - states[8]) -  algebraic14*states[8];
  const double_v algebraic2 = 1.00000/(1.00000+exponential((states[0] - constant25)/constant28));
  const double_v algebraic15 =  1000.00*exponential(- ((states[0]+40.0000)/25.7500));
  const double_v rate9 = (algebraic2 - states[9])/algebraic15;
  const double_v algebraic4 =  constant15*((states[0] - constant20)/(1.00000 - exponential(- ((states[0] - constant20)/constant31))));
  const double_v algebraic17 =  constant18*exponential(- ((states[0] - constant20)/constant33));
  const double_v rate10 =  algebraic4*(1.00000 - states[10]) -  algebraic17*states[10];
  const double_v algebraic3 =  constant14*exponential(- ((states[0] - constant22)/constant29));
  const double_v algebraic16 = constant17/(1.00000+exponential(- ((states[0] - constant22)/constant30)));
  const double_v rate11 =  algebraic3*(1.00000 - states[11]) -  algebraic16*states[11];
  const double_v algebraic5 = 1.00000/(1.00000+exponential((states[0] - constant24)/constant27));
  const double_v algebraic18 = 8571.00/(0.200000+ 5.65000*pow2((states[0]+constant48)/100.000));
  const double_v rate12 = (algebraic5 - states[12])/algebraic18;
  const double_v algebraic6 =  constant16*((states[1] - constant21)/(1.00000 - exponential(- ((states[1] - constant21)/constant32))));
  const double_v algebraic19 =  constant19*exponential(- ((states[1] - constant21)/constant34));
  const double_v rate13 =  algebraic6*(1.00000 - states[13]) -  algebraic19*states[13];
  const double_v algebraic7 = 1.00000/(1.00000+exponential((states[1] - constant25)/constant28));
  const double_v algebraic20 =  1.00000*exponential(- ((states[1]+40.0000)/25.7500));
  const double_v rate14 = (algebraic7 - states[14])/algebraic20;
  const double_v algebraic9 =  constant15*((states[1] - constant20)/(1.00000 - exponential(- ((states[1] - constant20)/constant31))));
  const double_v algebraic22 =  constant18*exponential(- ((states[1] - constant20)/constant33));
  const double_v rate15 =  algebraic9*(1.00000 - states[15]) -  algebraic22*states[15];
  const double_v algebraic8 =  constant14*exponential(- ((states[1] - constant22)/constant29));
  const double_v algebraic21 = constant17/(1.00000+exponential(- ((states[1] - constant22)/constant30)));
  const double_v rate16 =  algebraic8*(1.00000 - states[16]) -  algebraic21*states[16];
  const double_v algebraic10 = 1.00000/(1.00000+exponential((states[1] - constant24)/constant27));
  const double_v algebraic23 = 8571.00/(0.200000+ 5.65000*pow2((states[1]+constant48)/100.000));
  const double_v rate17 = (algebraic10 - states[17])/algebraic23;
  const double_v algebraic11 =  0.500000*constant57*exponential((states[1] - constant59)/( 8.00000*constant58));
  const double_v algebraic24 =  0.500000*constant57*exponential((constant59 - states[1])/( 8.00000*constant58));
  const double_v rate23 =  - constant54*states[23]+ constant55*states[18]+ -4.00000*algebraic11*states[23]+ algebraic24*states[24];
  const double_v rate18 =  constant54*states[23]+ - constant55*states[18]+( -4.00000*algebraic11*states[18])/constant56+ constant56*algebraic24*states[19];
  const double_v rate24 =  4.00000*algebraic11*states[23]+ - algebraic24*states[24]+( - constant54*states[24])/constant56+ constant56*constant55*states[19]+ -3.00000*algebraic11*states[24]+ 2.00000*algebraic24*states[25];
  const double_v rate19 = ( constant54*states[24])/constant56+ - constant55*constant56*states[19]+( 4.00000*algebraic11*states[18])/constant56+ - constant56*algebraic24*states[19]+( -3.00000*algebraic11*states[19])/constant56+ 2.00000*constant56*algebraic24*states[20];
  const double_v rate25 =  3.00000*algebraic11*states[24]+ -2.00000*algebraic24*states[25]+( - constant54*states[25])/pow2(constant56)+ pow2(constant56)*constant55*states[20]+ -2.00000*algebraic11*states[25]+ 3.00000*algebraic24*states[26];
  const double_v rate20 = ( 3.00000*algebraic11*states[19])/constant56+ -2.00000*constant56*algebraic24*states[20]+( constant54*states[25])/pow2(constant56)+ - constant55*pow2(constant56)*states[20]+( -2.00000*algebraic11*states[20])/constant56+ 3.00000*constant56*algebraic24*states[21];
  const double_v rate26 =  2.00000*algebraic11*states[25]+ -3.00000*algebraic24*states[26]+( - constant54*states[26])/pow3(constant56)+ constant55*pow3(constant56)*states[21]+ - algebraic11*states[26]+ 4.00000*algebraic24*states[27];
  const double_v rate21 = ( constant54*states[26])/pow3(constant56)+ - constant55*pow3(constant56)*states[21]+( 2.00000*algebraic11*states[20])/constant56+ -3.00000*algebraic24*constant56*states[21]+( - algebraic11*states[21])/constant56+ 4.00000*constant56*algebraic24*states[22];
  const double_v rate27 =  algebraic11*states[26]+ -4.00000*algebraic24*states[27]+( - constant54*states[27])/pow4(constant56)+ constant55*pow4(constant56)*states[22];
  const double_v rate22 = ( algebraic11*states[21])/constant56+ -4.00000*constant56*algebraic24*states[22]+( constant54*states[27])/pow4(constant56)+ - constant55*pow4(constant56)*states[22];
  const double_v algebraic30 =  states[0]*((states[3] -  states[4]*exponential(( -1.00000*constant6*states[0])/( constant35*constant36)))/(1.00000 - exponential(( -1.00000*constant6*states[0])/( constant35*constant36))));
  const double_v algebraic13 =  (( constant35*constant36)/constant6)*logarithmic(states[4]/states[3]);
  const double_v algebraic37 =  states[4]*exponential( ( - constant41*algebraic13)*(constant6/( constant35*constant36)));
  const double_v algebraic38 =  constant40*(pow2(algebraic37)/(constant42+pow2(algebraic37)));
  const double_v algebraic39 = 1.00000 - powReciprocal1(1.00000+( constant43*(1.00000+pow2(algebraic37)/constant42))/( pow2(constant46)*exponential(( 2.00000*(1.00000 - constant41)*states[0]*constant6)/( constant35*constant36))));
  const double_v algebraic40 =  algebraic38*algebraic39;
  const double_v algebraic41 =  algebraic40*(Vc::iif(algebraic30>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero)))*(algebraic30/50.0000);
  const double_v algebraic42 =  ( constant38*pow4(states[8]))*states[9];
  const double_v algebraic43 =  algebraic42*(algebraic30/50.0000);
  const double_v algebraic47 =  (1.00000/7.00000)*(exponential(states[7]/67.3000) - 1.00000);
  const double_v algebraic48 = powReciprocal1(1.00000+ 0.120000*exponential( -0.100000*states[0]*(constant6/( constant35*constant36)))+ 0.0400000*algebraic47*exponential(- ( states[0]*(constant6/( constant35*constant36)))));
  const double_v algebraic49 =  constant6*(constant47/( pow2(1.00000+constant44/states[4])*pow3(1.00000+constant45/states[5])));
  const double_v algebraic50 =  algebraic49*algebraic48;
  const double_v rate4 = (algebraic41+algebraic43+constant12+ - 2.00000*algebraic50)/( (1000.00/1.00000)*constant6*constant5)+(states[2] - states[4])/constant10;
  const double_v algebraic45 =  ( ( constant39*pow3(states[10]))*states[11])*states[12];
  const double_v algebraic44 =  states[0]*((states[5] -  states[7]*exponential(( -1.00000*constant6*states[0])/( constant35*constant36)))/(1.00000 - exponential(( -1.00000*constant6*states[0])/( constant35*constant36))));
  const double_v algebraic46 =  algebraic45*(algebraic44/75.0000);
  const double_v rate7 = (algebraic46+constant13+ 3.00000*algebraic50)/( (1000.00/1.00000)*constant6*constant5)+(states[6] - states[7])/constant11;
  const double_v algebraic0 =  (1000.00/1.00000)*((states[0] - states[1])/constant2);
  const double_v algebraic26 = 156.500/(5.00000+exponential(( - constant6*algebraic13)/( constant35*constant36)));
  const double_v algebraic27 = 156.500 -  5.00000*algebraic26;
  const double_v algebraic34 =  states[0]*((algebraic26 -  algebraic27*exponential(( constant6*states[0])/( constant35*constant36)))/(1.00000 - exponential(( constant6*states[0])/( constant35*constant36))));
  const double_v algebraic33 = 1.00000/(1.00000+exponential((states[0] - constant23)/constant26));
  const double_v algebraic35 =  constant37*pow4(algebraic33);
  const double_v algebraic36 =  algebraic35*(algebraic34/45.0000);
    // (not assigning to a parameter) algebraics = (VOI>=0.00000&&VOI<0.500000 Vc::iif(VOI>=0.00000&&VOI<0.500000 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=50.0000&&VOI<50.5000 Vc::iif( VOI>=50.0000&&VOI<50.5000 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=100.000&&VOI<100.500 Vc::iif( VOI>=100.000&&VOI<100.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=150.000&&VOI<150.500 Vc::iif( VOI>=150.000&&VOI<150.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=200.000&&VOI<200.500 Vc::iif( VOI>=200.000&&VOI<200.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=250.000&&VOI<250.500 Vc::iif( VOI>=250.000&&VOI<250.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=300.000&&VOI<300.500 Vc::iif( VOI>=300.000&&VOI<300.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=350.000&&VOI<350.500 Vc::iif( VOI>=350.000&&VOI<350.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=400.000&&VOI<400.500 Vc::iif( VOI>=400.000&&VOI<400.500 , (Vc::double_v(Vc::One)*( 150.000 )), Vc::double_v(Vc::Zero)))))))))));
  const double_v algebraic51 = algebraic36+algebraic41+algebraic43+algebraic46+algebraic50+- parameters[0];
  const double_v rate0 = - ((algebraic51+algebraic0)/constant0);
  const double_v algebraic31 =  states[1]*((states[3] -  states[2]*exponential(( -1.00000*constant6*states[1])/( constant35*constant36)))/(1.00000 - exponential(( -1.00000*constant6*states[1])/( constant35*constant36))));
  const double_v algebraic25 =  (( constant35*constant36)/constant6)*logarithmic(states[2]/states[3]);
  const double_v algebraic56 =  states[2]*exponential( ( - constant41*algebraic25)*(constant6/( constant35*constant36)));
  const double_v algebraic57 =  constant40*(pow2(algebraic56)/(constant42+pow2(algebraic56)));
  const double_v algebraic58 = 1.00000 - powReciprocal1(1.00000+( constant43*(1.00000+pow2(algebraic56)/constant42))/( pow2(constant46)*exponential(( 2.00000*(1.00000 - constant41)*states[1]*constant6)/( constant35*constant36))));
  const double_v algebraic59 =  algebraic57*algebraic58;
  const double_v algebraic60 =  constant50*algebraic59*(algebraic31/50.0000);
  const double_v algebraic61 =  ( constant38*pow4(states[13]))*states[14];
  const double_v algebraic62 =  constant51*algebraic61*(algebraic31/50.0000);
  const double_v algebraic66 =  (1.00000/7.00000)*(exponential(states[6]/67.3000) - 1.00000);
  const double_v algebraic67 = powReciprocal1(1.00000+ 0.120000*exponential( -0.100000*states[1]*(constant6/( constant35*constant36)))+ 0.0400000*algebraic66*exponential(- ( states[1]*(constant6/( constant35*constant36)))));
  const double_v algebraic68 =  constant6*(constant47/( pow2(1.00000+constant44/states[2])*pow3(1.00000+constant45/states[5])));
  const double_v algebraic69 =  constant53*algebraic68*algebraic67;
  const double_v rate3 =  - constant9*((algebraic60+algebraic62+constant12+ - 2.00000*algebraic69)/( (1000.00/1.00000)*constant6*constant3)) - (algebraic41+algebraic43+constant12+ -2.00000*algebraic50)/( (1000.00/1.00000)*constant6*constant4);
  const double_v rate2 = (algebraic60+algebraic62+constant12+ - 2.00000*algebraic69)/( (1000.00/1.00000)*constant6*constant3) - (states[2] - states[4])/constant7;
  const double_v algebraic64 =  ( ( constant39*pow3(states[15]))*states[16])*states[17];
  const double_v algebraic63 =  states[1]*((states[5] -  states[6]*exponential(( -1.00000*constant6*states[1])/( constant35*constant36)))/(1.00000 - exponential(( -1.00000*constant6*states[1])/( constant35*constant36))));
  const double_v algebraic65 =  constant52*algebraic64*(algebraic63/75.0000);
  const double_v rate5 =  - constant9*((algebraic65+constant13+ 3.00000*algebraic69)/( (1000.00/1.00000)*constant6*constant3)) - (algebraic46+constant13+ 3.00000*algebraic50)/( (1000.00/1.00000)*constant6*constant4);
  const double_v rate6 = (algebraic65+constant13+ 3.00000*algebraic69)/( (1000.00/1.00000)*constant6*constant3) - (states[6] - states[7])/constant8;
  const double_v algebraic28 = 156.500/(5.00000+exponential(( - constant6*algebraic25)/( constant35*constant36)));
  const double_v algebraic29 = 156.500 -  5.00000*algebraic28;
  const double_v algebraic53 =  states[1]*((algebraic28 -  algebraic29*exponential(( constant6*states[1])/( constant35*constant36)))/(1.00000 - exponential(( constant6*states[1])/( constant35*constant36))));
  const double_v algebraic52 = 1.00000/(1.00000+exponential((states[1] - constant23)/constant26));
  const double_v algebraic54 =  constant37*pow4(algebraic52);
  const double_v algebraic55 =  constant49*algebraic54*(algebraic53/45.0000);
  const double_v algebraic70 = algebraic55+algebraic60+algebraic62+algebraic65+algebraic69;
  const double_v rate1 = - ((algebraic70 - algebraic0/constant1)/constant0);

  // algebraic step
  // compute y* = y_n + dt*rhs(y_n), y_n = state, rhs(y_n) = rate, y* = algebraicState
  double_v algebraicState0 = states[0] + timeStepWidth*rate0;
  const double_v algebraicState1 = states[1] + timeStepWidth*rate1;
  const double_v algebraicState2 = states[2] + timeStepWidth*rate2;
  const double_v algebraicState3 = states[3] + timeStepWidth*rate3;
  const double_v algebraicState4 = states[4] + timeStepWidth*rate4;
  const double_v algebraicState5 = states[5] + timeStepWidth*rate5;
  const double_v algebraicState6 = states[6] + timeStepWidth*rate6;
  const double_v algebraicState7 = states[7] + timeStepWidth*rate7;
  const double_v algebraicState8 = states[8] + timeStepWidth*rate8;
  const double_v algebraicState9 = states[9] + timeStepWidth*rate9;
  const double_v algebraicState10 = states[10] + timeStepWidth*rate10;
  const double_v algebraicState11 = states[11] + timeStepWidth*rate11;
  const double_v algebraicState12 = states[12] + timeStepWidth*rate12;
  const double_v algebraicState13 = states[13] + timeStepWidth*rate13;
  const double_v algebraicState14 = states[14] + timeStepWidth*rate14;
  const double_v algebraicState15 = states[15] + timeStepWidth*rate15;
  const double_v algebraicState16 = states[16] + timeStepWidth*rate16;
  const double_v algebraicState17 = states[17] + timeStepWidth*rate17;
  const double_v algebraicState18 = states[18] + timeStepWidth*rate18;
  const double_v algebraicState19 = states[19] + timeStepWidth*rate19;
  const double_v algebraicState20 = states[20] + timeStepWidth*rate20;
  const double_v algebraicState21 = states[21] + timeStepWidth*rate21;
  const double_v algebraicState22 = states[22] + timeStepWidth*rate22;
  const double_v algebraicState23 = states[23] + timeStepWidth*rate23;
  const double_v algebraicState24 = states[24] + timeStepWidth*rate24;
  const double_v algebraicState25 = states[25] + timeStepWidth*rate25;
  const double_v algebraicState26 = states[26] + timeStepWidth*rate26;
  const double_v algebraicState27 = states[27] + timeStepWidth*rate27;
  const double_v algebraicState28 = states[28] + timeStepWidth*rate28;
  const double_v algebraicState29 = states[29] + timeStepWidth*rate29;
  const double_v algebraicState30 = states[30] + timeStepWidth*rate30;
  const double_v algebraicState31 = states[31] + timeStepWidth*rate31;
  const double_v algebraicState32 = states[32] + timeStepWidth*rate32;
  const double_v algebraicState33 = states[33] + timeStepWidth*rate33;
  const double_v algebraicState34 = states[34] + timeStepWidth*rate34;
  const double_v algebraicState35 = states[35] + timeStepWidth*rate35;
  const double_v algebraicState36 = states[36] + timeStepWidth*rate36;
  const double_v algebraicState37 = states[37] + timeStepWidth*rate37;
  const double_v algebraicState38 = states[38] + timeStepWidth*rate38;
  const double_v algebraicState39 = states[39] + timeStepWidth*rate39;
  const double_v algebraicState40 = states[40] + timeStepWidth*rate40;
  const double_v algebraicState41 = states[41] + timeStepWidth*rate41;
  const double_v algebraicState42 = states[42] + timeStepWidth*rate42;
  const double_v algebraicState43 = states[43] + timeStepWidth*rate43;
  const double_v algebraicState44 = states[44] + timeStepWidth*rate44;
  const double_v algebraicState45 = states[45] + timeStepWidth*rate45;
  const double_v algebraicState46 = states[46] + timeStepWidth*rate46;
  const double_v algebraicState47 = states[47] + timeStepWidth*rate47;
  const double_v algebraicState48 = states[48] + timeStepWidth*rate48;
  const double_v algebraicState49 = states[49] + timeStepWidth*rate49;
  const double_v algebraicState50 = states[50] + timeStepWidth*rate50;
  const double_v algebraicState51 = states[51] + timeStepWidth*rate51;
  const double_v algebraicState52 = states[52] + timeStepWidth*rate52;
  const double_v algebraicState53 = states[53] + timeStepWidth*rate53;
  const double_v algebraicState54 = states[54] + timeStepWidth*rate54;
  const double_v algebraicState55 = states[55] + timeStepWidth*rate55;



  // if stimulation, set value of Vm (state0)
  if (stimulate)
  {
    for (int i = 0; i < std::min(3,(int)Vc::double_v::size()); i++)
    {
      algebraicState0[i] = valueForStimulatedPoint;
    }
  }
  // compute new rates, rhs(y*)
  const double_v algebraicRate28 = (((( ( constant98*(algebraicState18+algebraicState19+algebraicState20+algebraicState21+algebraicState22))*((algebraicState29 - algebraicState28)/constant101) -  constant60*((algebraicState28/(algebraicState28+constant61))/constant101))+ constant62*((algebraicState29 - algebraicState28)/constant101))+ - constant63*((algebraicState28 - algebraicState30)/constant101))+- ( ( constant70*algebraicState28)*((constant72+- algebraicState33)+- algebraicState35)+ - constant71*algebraicState33))+- ( ( constant78*algebraicState28)*algebraicState43+ - constant79*algebraicState39);
  const double_v algebraicRate29 = ((( - ( constant98*(algebraicState18+algebraicState19+algebraicState20+algebraicState21+algebraicState22))*((algebraicState29 - algebraicState28)/constant103)+ constant60*((algebraicState28/(algebraicState28+constant61))/constant103))+ - constant62*((algebraicState29 - algebraicState28)/constant103))+ - constant64*((algebraicState29 - algebraicState31)/constant103))+- ( ( constant75*algebraicState29)*(constant77 - algebraicState37)+ - constant76*algebraicState37);
  const double_v algebraicRate31 = ((( constant60*((algebraicState30/(algebraicState30+constant61))/constant104)+ - constant62*((algebraicState31+- algebraicState30)/constant104))+ constant64*((algebraicState29+- algebraicState31)/constant104))+- ( ( constant75*algebraicState31)*(constant77+- algebraicState38)+ - constant76*algebraicState38)) -  (1000.00/1.00000)*( constant95*( algebraicState54*(0.00100000/1.00000)*algebraicState31 - constant97)*(Vc::iif( algebraicState54*(0.00100000/1.00000)*algebraicState31 - constant97>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero)))*(0.00100000/1.00000)*algebraicState54*algebraicState31 -  constant96*algebraicState55*(constant97 -  algebraicState54*(0.00100000/1.00000)*algebraicState31)*(Vc::iif(constant97 -  algebraicState54*(0.00100000/1.00000)*algebraicState31>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero))));
  const double_v algebraicRate33 =  ( constant70*algebraicState28)*((constant72+- algebraicState33)+- algebraicState35)+ - constant71*algebraicState33;
  const double_v algebraicRate34 =  ( constant70*algebraicState30)*((constant72+- algebraicState34)+- algebraicState36)+ - constant71*algebraicState34;
  const double_v algebraicRate35 =  ( constant73*(constant72+- algebraicState33+- algebraicState35))*algebraicState45+ - constant74*algebraicState35;
  const double_v algebraicRate36 =  ( constant73*(constant72+- algebraicState34+- algebraicState36))*algebraicState46+ - constant74*algebraicState36;
  const double_v algebraicRate37 =  ( constant75*algebraicState29)*(constant77+- algebraicState37)+ - constant76*algebraicState37;
  const double_v algebraicRate38 =  ( constant75*algebraicState31)*(constant77+- algebraicState38)+ - constant76*algebraicState38;
  const double_v algebraicRate39 = ( ( constant78*algebraicState28)*algebraicState43+ - constant79*algebraicState39)+ - constant82*((algebraicState39+- algebraicState40)/constant101);
  const double_v algebraicRate40 = ( ( constant78*algebraicState30)*algebraicState44+ - constant79*algebraicState40)+ constant82*((algebraicState39+- algebraicState40)/constant102);
  const double_v algebraicRate41 = ( ( constant80*algebraicState45)*algebraicState43+ - constant81*algebraicState41)+ - constant82*((algebraicState41+- algebraicState42)/constant101);
  const double_v algebraicRate42 = ( ( constant80*algebraicState46)*algebraicState44+ - constant81*algebraicState42)+ constant82*((algebraicState41+- algebraicState42)/constant102);
  const double_v algebraicRate43 = (- ( ( constant78*algebraicState28)*algebraicState43+ - constant79*algebraicState39)+- ( ( constant80*algebraicState45)*algebraicState43+ - constant81*algebraicState41))+ - constant82*((algebraicState43+- algebraicState44)/constant101);
  const double_v algebraicRate44 = (- ( ( constant78*algebraicState30)*algebraicState44+ - constant79*algebraicState40)+- ( ( constant80*algebraicState46)*algebraicState44+ - constant81*algebraicState42))+ constant82*((algebraicState43+- algebraicState44)/constant102);
  const double_v algebraicRate45 = (- ( ( constant73*(constant72+- algebraicState33+- algebraicState35))*algebraicState45+ - constant74*algebraicState35)+- ( ( constant80*algebraicState45)*algebraicState43+ - constant81*algebraicState41))+ - constant83*((algebraicState45+- algebraicState46)/constant101);
  const double_v algebraicRate46 = (- ( ( constant73*(constant72+- algebraicState34+- algebraicState36))*algebraicState46+ - constant74*algebraicState36)+- ( ( constant80*algebraicState46)*algebraicState44+ - constant81*algebraicState42))+ constant83*((algebraicState45+- algebraicState46)/constant102);
  const double_v algebraicRate47 = (( ( constant67*algebraicState30)*algebraicState32+ - constant68*algebraicState47)+ - constant86*algebraicState47)+ constant87*algebraicState50;
  const double_v algebraicRate49 = (((( constant67*algebraicState30*algebraicState48+ - constant68*algebraicState49)+ constant84*algebraicState32)+ - constant85*algebraicState49)+ ( - constant67*algebraicState30)*algebraicState49)+ constant68*algebraicState50;
  const double_v algebraicRate50 = ((((( constant67*algebraicState30*algebraicState49+ - constant68*algebraicState50)+ constant86*algebraicState47)+ - constant87*algebraicState50)+ - constant88*algebraicState50)+ constant89*algebraicState51)+ constant92*algebraicState52;
  const double_v algebraicRate51 = (( constant88*algebraicState50+ - constant89*algebraicState51)+ constant91*algebraicState52)+ - constant90*algebraicState51;
  const double_v algebraicRate52 = ( - constant91*algebraicState52+ constant90*algebraicState51)+ - constant92*algebraicState52;
  const double_v algebraicRate53 =  (0.00100000/1.00000)*( constant90*algebraicState51 -  constant91*algebraicState52)+ -1.00000*constant93*algebraicState53+ -1.00000*constant94*((algebraicState53 - algebraicState54)/constant102);
  const double_v algebraicRate54 =  constant94*((algebraicState53 - algebraicState54)/constant104) -  1.00000*( constant95*( algebraicState54*(0.00100000/1.00000)*algebraicState31 - constant97)*(Vc::iif( algebraicState54*(0.00100000/1.00000)*algebraicState31 - constant97>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero)))*(0.00100000/1.00000)*algebraicState54*algebraicState31 -  constant96*algebraicState55*(constant97 -  algebraicState54*(0.00100000/1.00000)*algebraicState31)*(Vc::iif(constant97 -  algebraicState54*(0.00100000/1.00000)*algebraicState31>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero))));
  const double_v algebraicRate55 =  1.00000*( constant95*( algebraicState54*(0.00100000/1.00000)*algebraicState31 - constant97)*(Vc::iif( algebraicState54*(0.00100000/1.00000)*algebraicState31 - constant97>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero)))*(0.00100000/1.00000)*algebraicState54*algebraicState31 -  constant96*algebraicState55*(constant97 -  algebraicState54*(0.00100000/1.00000)*algebraicState31)*(Vc::iif(constant97 -  algebraicState54*(0.00100000/1.00000)*algebraicState31>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero))));
  const double_v algebraicAlgebraic12 = constant69+- algebraicState32+- algebraicState47+- algebraicState48+- algebraicState49+- algebraicState50+- algebraicState51+- algebraicState52;
  const double_v algebraicRate30 = (((( - constant60*((algebraicState30/(algebraicState30+constant61))/constant102)+ constant62*((algebraicState31+- algebraicState30)/constant102))+ constant63*((algebraicState28 - algebraicState30)/constant102))+- ((((((( constant67*algebraicState30*algebraicAlgebraic12+ - constant68*algebraicState32)+ constant67*algebraicState30*algebraicState32)+ - constant68*algebraicState47)+ constant67*algebraicState30*algebraicState48)+ - constant68*algebraicState49)+ constant67*algebraicState30*algebraicState49)+ - constant68*algebraicState50))+- ( ( constant70*algebraicState30)*(constant72+- algebraicState34+- algebraicState36)+ - constant71*algebraicState34))+- ( ( constant78*algebraicState30)*algebraicState44+ - constant79*algebraicState40);
  const double_v algebraicRate32 = (((( ( constant67*algebraicState30)*algebraicAlgebraic12+ - constant68*algebraicState32)+ ( - constant67*algebraicState30)*algebraicState32)+ constant68*algebraicState47)+ - constant84*algebraicState32)+ constant85*algebraicState49;
  const double_v algebraicRate48 = (( ( - constant67*algebraicState30)*algebraicState48+ constant68*algebraicState49)+ constant84*algebraicAlgebraic12)+ - constant85*algebraicState48;
  const double_v algebraicAlgebraic1 =  constant16*((algebraicState0 - constant21)/(1.00000 - exponential(- ((algebraicState0 - constant21)/constant32))));
  const double_v algebraicAlgebraic14 =  constant19*exponential(- ((algebraicState0 - constant21)/constant34));
  const double_v algebraicRate8 =  algebraicAlgebraic1*(1.00000 - algebraicState8) -  algebraicAlgebraic14*algebraicState8;
  const double_v algebraicAlgebraic2 = 1.00000/(1.00000+exponential((algebraicState0 - constant25)/constant28));
  const double_v algebraicAlgebraic15 =  1000.00*exponential(- ((algebraicState0+40.0000)/25.7500));
  const double_v algebraicRate9 = (algebraicAlgebraic2 - algebraicState9)/algebraicAlgebraic15;
  const double_v algebraicAlgebraic4 =  constant15*((algebraicState0 - constant20)/(1.00000 - exponential(- ((algebraicState0 - constant20)/constant31))));
  const double_v algebraicAlgebraic17 =  constant18*exponential(- ((algebraicState0 - constant20)/constant33));
  const double_v algebraicRate10 =  algebraicAlgebraic4*(1.00000 - algebraicState10) -  algebraicAlgebraic17*algebraicState10;
  const double_v algebraicAlgebraic3 =  constant14*exponential(- ((algebraicState0 - constant22)/constant29));
  const double_v algebraicAlgebraic16 = constant17/(1.00000+exponential(- ((algebraicState0 - constant22)/constant30)));
  const double_v algebraicRate11 =  algebraicAlgebraic3*(1.00000 - algebraicState11) -  algebraicAlgebraic16*algebraicState11;
  const double_v algebraicAlgebraic5 = 1.00000/(1.00000+exponential((algebraicState0 - constant24)/constant27));
  const double_v algebraicAlgebraic18 = 8571.00/(0.200000+ 5.65000*pow2((algebraicState0+constant48)/100.000));
  const double_v algebraicRate12 = (algebraicAlgebraic5 - algebraicState12)/algebraicAlgebraic18;
  const double_v algebraicAlgebraic6 =  constant16*((algebraicState1 - constant21)/(1.00000 - exponential(- ((algebraicState1 - constant21)/constant32))));
  const double_v algebraicAlgebraic19 =  constant19*exponential(- ((algebraicState1 - constant21)/constant34));
  const double_v algebraicRate13 =  algebraicAlgebraic6*(1.00000 - algebraicState13) -  algebraicAlgebraic19*algebraicState13;
  const double_v algebraicAlgebraic7 = 1.00000/(1.00000+exponential((algebraicState1 - constant25)/constant28));
  const double_v algebraicAlgebraic20 =  1.00000*exponential(- ((algebraicState1+40.0000)/25.7500));
  const double_v algebraicRate14 = (algebraicAlgebraic7 - algebraicState14)/algebraicAlgebraic20;
  const double_v algebraicAlgebraic9 =  constant15*((algebraicState1 - constant20)/(1.00000 - exponential(- ((algebraicState1 - constant20)/constant31))));
  const double_v algebraicAlgebraic22 =  constant18*exponential(- ((algebraicState1 - constant20)/constant33));
  const double_v algebraicRate15 =  algebraicAlgebraic9*(1.00000 - algebraicState15) -  algebraicAlgebraic22*algebraicState15;
  const double_v algebraicAlgebraic8 =  constant14*exponential(- ((algebraicState1 - constant22)/constant29));
  const double_v algebraicAlgebraic21 = constant17/(1.00000+exponential(- ((algebraicState1 - constant22)/constant30)));
  const double_v algebraicRate16 =  algebraicAlgebraic8*(1.00000 - algebraicState16) -  algebraicAlgebraic21*algebraicState16;
  const double_v algebraicAlgebraic10 = 1.00000/(1.00000+exponential((algebraicState1 - constant24)/constant27));
  const double_v algebraicAlgebraic23 = 8571.00/(0.200000+ 5.65000*pow2((algebraicState1+constant48)/100.000));
  const double_v algebraicRate17 = (algebraicAlgebraic10 - algebraicState17)/algebraicAlgebraic23;
  const double_v algebraicAlgebraic11 =  0.500000*constant57*exponential((algebraicState1 - constant59)/( 8.00000*constant58));
  const double_v algebraicAlgebraic24 =  0.500000*constant57*exponential((constant59 - algebraicState1)/( 8.00000*constant58));
  const double_v algebraicRate23 =  - constant54*algebraicState23+ constant55*algebraicState18+ -4.00000*algebraicAlgebraic11*algebraicState23+ algebraicAlgebraic24*algebraicState24;
  const double_v algebraicRate18 =  constant54*algebraicState23+ - constant55*algebraicState18+( -4.00000*algebraicAlgebraic11*algebraicState18)/constant56+ constant56*algebraicAlgebraic24*algebraicState19;
  const double_v algebraicRate24 =  4.00000*algebraicAlgebraic11*algebraicState23+ - algebraicAlgebraic24*algebraicState24+( - constant54*algebraicState24)/constant56+ constant56*constant55*algebraicState19+ -3.00000*algebraicAlgebraic11*algebraicState24+ 2.00000*algebraicAlgebraic24*algebraicState25;
  const double_v algebraicRate19 = ( constant54*algebraicState24)/constant56+ - constant55*constant56*algebraicState19+( 4.00000*algebraicAlgebraic11*algebraicState18)/constant56+ - constant56*algebraicAlgebraic24*algebraicState19+( -3.00000*algebraicAlgebraic11*algebraicState19)/constant56+ 2.00000*constant56*algebraicAlgebraic24*algebraicState20;
  const double_v algebraicRate25 =  3.00000*algebraicAlgebraic11*algebraicState24+ -2.00000*algebraicAlgebraic24*algebraicState25+( - constant54*algebraicState25)/pow2(constant56)+ pow2(constant56)*constant55*algebraicState20+ -2.00000*algebraicAlgebraic11*algebraicState25+ 3.00000*algebraicAlgebraic24*algebraicState26;
  const double_v algebraicRate20 = ( 3.00000*algebraicAlgebraic11*algebraicState19)/constant56+ -2.00000*constant56*algebraicAlgebraic24*algebraicState20+( constant54*algebraicState25)/pow2(constant56)+ - constant55*pow2(constant56)*algebraicState20+( -2.00000*algebraicAlgebraic11*algebraicState20)/constant56+ 3.00000*constant56*algebraicAlgebraic24*algebraicState21;
  const double_v algebraicRate26 =  2.00000*algebraicAlgebraic11*algebraicState25+ -3.00000*algebraicAlgebraic24*algebraicState26+( - constant54*algebraicState26)/pow3(constant56)+ constant55*pow3(constant56)*algebraicState21+ - algebraicAlgebraic11*algebraicState26+ 4.00000*algebraicAlgebraic24*algebraicState27;
  const double_v algebraicRate21 = ( constant54*algebraicState26)/pow3(constant56)+ - constant55*pow3(constant56)*algebraicState21+( 2.00000*algebraicAlgebraic11*algebraicState20)/constant56+ -3.00000*algebraicAlgebraic24*constant56*algebraicState21+( - algebraicAlgebraic11*algebraicState21)/constant56+ 4.00000*constant56*algebraicAlgebraic24*algebraicState22;
  const double_v algebraicRate27 =  algebraicAlgebraic11*algebraicState26+ -4.00000*algebraicAlgebraic24*algebraicState27+( - constant54*algebraicState27)/pow4(constant56)+ constant55*pow4(constant56)*algebraicState22;
  const double_v algebraicRate22 = ( algebraicAlgebraic11*algebraicState21)/constant56+ -4.00000*constant56*algebraicAlgebraic24*algebraicState22+( constant54*algebraicState27)/pow4(constant56)+ - constant55*pow4(constant56)*algebraicState22;
  const double_v algebraicAlgebraic30 =  algebraicState0*((algebraicState3 -  algebraicState4*exponential(( -1.00000*constant6*algebraicState0)/( constant35*constant36)))/(1.00000 - exponential(( -1.00000*constant6*algebraicState0)/( constant35*constant36))));
  const double_v algebraicAlgebraic13 =  (( constant35*constant36)/constant6)*logarithmic(algebraicState4/algebraicState3);
  const double_v algebraicAlgebraic37 =  algebraicState4*exponential( ( - constant41*algebraicAlgebraic13)*(constant6/( constant35*constant36)));
  const double_v algebraicAlgebraic38 =  constant40*(pow2(algebraicAlgebraic37)/(constant42+pow2(algebraicAlgebraic37)));
  const double_v algebraicAlgebraic39 = 1.00000 - powReciprocal1(1.00000+( constant43*(1.00000+pow2(algebraicAlgebraic37)/constant42))/( pow2(constant46)*exponential(( 2.00000*(1.00000 - constant41)*algebraicState0*constant6)/( constant35*constant36))));
  const double_v algebraicAlgebraic40 =  algebraicAlgebraic38*algebraicAlgebraic39;
  const double_v algebraicAlgebraic41 =  algebraicAlgebraic40*(Vc::iif(algebraicAlgebraic30>0.00000 , Vc::double_v(Vc::One), Vc::double_v(Vc::Zero)))*(algebraicAlgebraic30/50.0000);
  const double_v algebraicAlgebraic42 =  ( constant38*pow4(algebraicState8))*algebraicState9;
  const double_v algebraicAlgebraic43 =  algebraicAlgebraic42*(algebraicAlgebraic30/50.0000);
  const double_v algebraicAlgebraic47 =  (1.00000/7.00000)*(exponential(algebraicState7/67.3000) - 1.00000);
  const double_v algebraicAlgebraic48 = powReciprocal1(1.00000+ 0.120000*exponential( -0.100000*algebraicState0*(constant6/( constant35*constant36)))+ 0.0400000*algebraicAlgebraic47*exponential(- ( algebraicState0*(constant6/( constant35*constant36)))));
  const double_v algebraicAlgebraic49 =  constant6*(constant47/( pow2(1.00000+constant44/algebraicState4)*pow3(1.00000+constant45/algebraicState5)));
  const double_v algebraicAlgebraic50 =  algebraicAlgebraic49*algebraicAlgebraic48;
  const double_v algebraicRate4 = (algebraicAlgebraic41+algebraicAlgebraic43+constant12+ - 2.00000*algebraicAlgebraic50)/( (1000.00/1.00000)*constant6*constant5)+(algebraicState2 - algebraicState4)/constant10;
  const double_v algebraicAlgebraic45 =  ( ( constant39*pow3(algebraicState10))*algebraicState11)*algebraicState12;
  const double_v algebraicAlgebraic44 =  algebraicState0*((algebraicState5 -  algebraicState7*exponential(( -1.00000*constant6*algebraicState0)/( constant35*constant36)))/(1.00000 - exponential(( -1.00000*constant6*algebraicState0)/( constant35*constant36))));
  const double_v algebraicAlgebraic46 =  algebraicAlgebraic45*(algebraicAlgebraic44/75.0000);
  const double_v algebraicRate7 = (algebraicAlgebraic46+constant13+ 3.00000*algebraicAlgebraic50)/( (1000.00/1.00000)*constant6*constant5)+(algebraicState6 - algebraicState7)/constant11;
  const double_v algebraicAlgebraic0 =  (1000.00/1.00000)*((algebraicState0 - algebraicState1)/constant2);
  const double_v algebraicAlgebraic26 = 156.500/(5.00000+exponential(( - constant6*algebraicAlgebraic13)/( constant35*constant36)));
  const double_v algebraicAlgebraic27 = 156.500 -  5.00000*algebraicAlgebraic26;
  const double_v algebraicAlgebraic34 =  algebraicState0*((algebraicAlgebraic26 -  algebraicAlgebraic27*exponential(( constant6*algebraicState0)/( constant35*constant36)))/(1.00000 - exponential(( constant6*algebraicState0)/( constant35*constant36))));
  const double_v algebraicAlgebraic33 = 1.00000/(1.00000+exponential((algebraicState0 - constant23)/constant26));
  const double_v algebraicAlgebraic35 =  constant37*pow4(algebraicAlgebraic33);
  const double_v algebraicAlgebraic36 =  algebraicAlgebraic35*(algebraicAlgebraic34/45.0000);
    // (not assigning to a parameter) algebraics = (VOI>=0.00000&&VOI<0.500000 Vc::iif(VOI>=0.00000&&VOI<0.500000 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=50.0000&&VOI<50.5000 Vc::iif( VOI>=50.0000&&VOI<50.5000 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=100.000&&VOI<100.500 Vc::iif( VOI>=100.000&&VOI<100.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=150.000&&VOI<150.500 Vc::iif( VOI>=150.000&&VOI<150.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=200.000&&VOI<200.500 Vc::iif( VOI>=200.000&&VOI<200.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=250.000&&VOI<250.500 Vc::iif( VOI>=250.000&&VOI<250.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=300.000&&VOI<300.500 Vc::iif( VOI>=300.000&&VOI<300.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=350.000&&VOI<350.500 Vc::iif( VOI>=350.000&&VOI<350.500 , (Vc::double_v(Vc::One)*( 150.000 )), VOI>=400.000&&VOI<400.500 Vc::iif( VOI>=400.000&&VOI<400.500 , (Vc::double_v(Vc::One)*( 150.000 )), Vc::double_v(Vc::Zero)))))))))));
  const double_v algebraicAlgebraic51 = algebraicAlgebraic36+algebraicAlgebraic41+algebraicAlgebraic43+algebraicAlgebraic46+algebraicAlgebraic50+- parameters[0];
  const double_v algebraicRate0 = - ((algebraicAlgebraic51+algebraicAlgebraic0)/constant0);
  const double_v algebraicAlgebraic31 =  algebraicState1*((algebraicState3 -  algebraicState2*exponential(( -1.00000*constant6*algebraicState1)/( constant35*constant36)))/(1.00000 - exponential(( -1.00000*constant6*algebraicState1)/( constant35*constant36))));
  const double_v algebraicAlgebraic25 =  (( constant35*constant36)/constant6)*logarithmic(algebraicState2/algebraicState3);
  const double_v algebraicAlgebraic56 =  algebraicState2*exponential( ( - constant41*algebraicAlgebraic25)*(constant6/( constant35*constant36)));
  const double_v algebraicAlgebraic57 =  constant40*(pow2(algebraicAlgebraic56)/(constant42+pow2(algebraicAlgebraic56)));
  const double_v algebraicAlgebraic58 = 1.00000 - powReciprocal1(1.00000+( constant43*(1.00000+pow2(algebraicAlgebraic56)/constant42))/( pow2(constant46)*exponential(( 2.00000*(1.00000 - constant41)*algebraicState1*constant6)/( constant35*constant36))));
  const double_v algebraicAlgebraic59 =  algebraicAlgebraic57*algebraicAlgebraic58;
  const double_v algebraicAlgebraic60 =  constant50*algebraicAlgebraic59*(algebraicAlgebraic31/50.0000);
  const double_v algebraicAlgebraic61 =  ( constant38*pow4(algebraicState13))*algebraicState14;
  const double_v algebraicAlgebraic62 =  constant51*algebraicAlgebraic61*(algebraicAlgebraic31/50.0000);
  const double_v algebraicAlgebraic66 =  (1.00000/7.00000)*(exponential(algebraicState6/67.3000) - 1.00000);
  const double_v algebraicAlgebraic67 = powReciprocal1(1.00000+ 0.120000*exponential( -0.100000*algebraicState1*(constant6/( constant35*constant36)))+ 0.0400000*algebraicAlgebraic66*exponential(- ( algebraicState1*(constant6/( constant35*constant36)))));
  const double_v algebraicAlgebraic68 =  constant6*(constant47/( pow2(1.00000+constant44/algebraicState2)*pow3(1.00000+constant45/algebraicState5)));
  const double_v algebraicAlgebraic69 =  constant53*algebraicAlgebraic68*algebraicAlgebraic67;
  const double_v algebraicRate3 =  - constant9*((algebraicAlgebraic60+algebraicAlgebraic62+constant12+ - 2.00000*algebraicAlgebraic69)/( (1000.00/1.00000)*constant6*constant3)) - (algebraicAlgebraic41+algebraicAlgebraic43+constant12+ -2.00000*algebraicAlgebraic50)/( (1000.00/1.00000)*constant6*constant4);
  const double_v algebraicRate2 = (algebraicAlgebraic60+algebraicAlgebraic62+constant12+ - 2.00000*algebraicAlgebraic69)/( (1000.00/1.00000)*constant6*constant3) - (algebraicState2 - algebraicState4)/constant7;
  const double_v algebraicAlgebraic64 =  ( ( constant39*pow3(algebraicState15))*algebraicState16)*algebraicState17;
  const double_v algebraicAlgebraic63 =  algebraicState1*((algebraicState5 -  algebraicState6*exponential(( -1.00000*constant6*algebraicState1)/( constant35*constant36)))/(1.00000 - exponential(( -1.00000*constant6*algebraicState1)/( constant35*constant36))));
  const double_v algebraicAlgebraic65 =  constant52*algebraicAlgebraic64*(algebraicAlgebraic63/75.0000);
  const double_v algebraicRate5 =  - constant9*((algebraicAlgebraic65+constant13+ 3.00000*algebraicAlgebraic69)/( (1000.00/1.00000)*constant6*constant3)) - (algebraicAlgebraic46+constant13+ 3.00000*algebraicAlgebraic50)/( (1000.00/1.00000)*constant6*constant4);
  const double_v algebraicRate6 = (algebraicAlgebraic65+constant13+ 3.00000*algebraicAlgebraic69)/( (1000.00/1.00000)*constant6*constant3) - (algebraicState6 - algebraicState7)/constant8;
  const double_v algebraicAlgebraic28 = 156.500/(5.00000+exponential(( - constant6*algebraicAlgebraic25)/( constant35*constant36)));
  const double_v algebraicAlgebraic29 = 156.500 -  5.00000*algebraicAlgebraic28;
  const double_v algebraicAlgebraic53 =  algebraicState1*((algebraicAlgebraic28 -  algebraicAlgebraic29*exponential(( constant6*algebraicState1)/( constant35*constant36)))/(1.00000 - exponential(( constant6*algebraicState1)/( constant35*constant36))));
  const double_v algebraicAlgebraic52 = 1.00000/(1.00000+exponential((algebraicState1 - constant23)/constant26));
  const double_v algebraicAlgebraic54 =  constant37*pow4(algebraicAlgebraic52);
  const double_v algebraicAlgebraic55 =  constant49*algebraicAlgebraic54*(algebraicAlgebraic53/45.0000);
  const double_v algebraicAlgebraic70 = algebraicAlgebraic55+algebraicAlgebraic60+algebraicAlgebraic62+algebraicAlgebraic65+algebraicAlgebraic69;
  const double_v algebraicRate1 = - ((algebraicAlgebraic70 - algebraicAlgebraic0/constant1)/constant0);


  // final step
  // y_n+1 = y_n + 0.5*[rhs(y_n) + rhs(y*)]
  states[0] += 0.5*timeStepWidth*(rate0 + algebraicRate0);
  states[1] += 0.5*timeStepWidth*(rate1 + algebraicRate1);
  states[2] += 0.5*timeStepWidth*(rate2 + algebraicRate2);
  states[3] += 0.5*timeStepWidth*(rate3 + algebraicRate3);
  states[4] += 0.5*timeStepWidth*(rate4 + algebraicRate4);
  states[5] += 0.5*timeStepWidth*(rate5 + algebraicRate5);
  states[6] += 0.5*timeStepWidth*(rate6 + algebraicRate6);
  states[7] += 0.5*timeStepWidth*(rate7 + algebraicRate7);
  states[8] += 0.5*timeStepWidth*(rate8 + algebraicRate8);
  states[9] += 0.5*timeStepWidth*(rate9 + algebraicRate9);
  states[10] += 0.5*timeStepWidth*(rate10 + algebraicRate10);
  states[11] += 0.5*timeStepWidth*(rate11 + algebraicRate11);
  states[12] += 0.5*timeStepWidth*(rate12 + algebraicRate12);
  states[13] += 0.5*timeStepWidth*(rate13 + algebraicRate13);
  states[14] += 0.5*timeStepWidth*(rate14 + algebraicRate14);
  states[15] += 0.5*timeStepWidth*(rate15 + algebraicRate15);
  states[16] += 0.5*timeStepWidth*(rate16 + algebraicRate16);
  states[17] += 0.5*timeStepWidth*(rate17 + algebraicRate17);
  states[18] += 0.5*timeStepWidth*(rate18 + algebraicRate18);
  states[19] += 0.5*timeStepWidth*(rate19 + algebraicRate19);
  states[20] += 0.5*timeStepWidth*(rate20 + algebraicRate20);
  states[21] += 0.5*timeStepWidth*(rate21 + algebraicRate21);
  states[22] += 0.5*timeStepWidth*(rate22 + algebraicRate22);
  states[23] += 0.5*timeStepWidth*(rate23 + algebraicRate23);
  states[24] += 0.5*timeStepWidth*(rate24 + algebraicRate24);
  states[25] += 0.5*timeStepWidth*(rate25 + algebraicRate25);
  states[26] += 0.5*timeStepWidth*(rate26 + algebraicRate26);
  states[27] += 0.5*timeStepWidth*(rate27 + algebraicRate27);
  states[28] += 0.5*timeStepWidth*(rate28 + algebraicRate28);
  states[29] += 0.5*timeStepWidth*(rate29 + algebraicRate29);
  states[30] += 0.5*timeStepWidth*(rate30 + algebraicRate30);
  states[31] += 0.5*timeStepWidth*(rate31 + algebraicRate31);
  states[32] += 0.5*timeStepWidth*(rate32 + algebraicRate32);
  states[33] += 0.5*timeStepWidth*(rate33 + algebraicRate33);
  states[34] += 0.5*timeStepWidth*(rate34 + algebraicRate34);
  states[35] += 0.5*timeStepWidth*(rate35 + algebraicRate35);
  states[36] += 0.5*timeStepWidth*(rate36 + algebraicRate36);
  states[37] += 0.5*timeStepWidth*(rate37 + algebraicRate37);
  states[38] += 0.5*timeStepWidth*(rate38 + algebraicRate38);
  states[39] += 0.5*timeStepWidth*(rate39 + algebraicRate39);
  states[40] += 0.5*timeStepWidth*(rate40 + algebraicRate40);
  states[41] += 0.5*timeStepWidth*(rate41 + algebraicRate41);
  states[42] += 0.5*timeStepWidth*(rate42 + algebraicRate42);
  states[43] += 0.5*timeStepWidth*(rate43 + algebraicRate43);
  states[44] += 0.5*timeStepWidth*(rate44 + algebraicRate44);
  states[45] += 0.5*timeStepWidth*(rate45 + algebraicRate45);
  states[46] += 0.5*timeStepWidth*(rate46 + algebraicRate46);
  states[47] += 0.5*timeStepWidth*(rate47 + algebraicRate47);
  states[48] += 0.5*timeStepWidth*(rate48 + algebraicRate48);
  states[49] += 0.5*timeStepWidth*(rate49 + algebraicRate49);
  states[50] += 0.5*timeStepWidth*(rate50 + algebraicRate50);
  states[51] += 0.5*timeStepWidth*(rate51 + algebraicRate51);
  states[52] += 0.5*timeStepWidth*(rate52 + algebraicRate52);
  states[53] += 0.5*timeStepWidth*(rate53 + algebraicRate53);
  states[54] += 0.5*timeStepWidth*(rate54 + algebraicRate54);
  states[55] += 0.5*timeStepWidth*(rate55 + algebraicRate55);

  if (stimulate)
  {
    for (int i = 0; i < std::min(3,(int)Vc::double_v::size()); i++)
    {
      states[0][i] = valueForStimulatedPoint;
    }
  }
  // store algebraics for transfer
  if (storeAlgebraicsForTransfer)
  {
    for (int i = 0; i < algebraicsForTransferIndices.size(); i++)
    {
      const int algebraic = algebraicsForTransferIndices[i];
      switch (algebraic)
      {
        case 0:
          algebraicsForTransfer[i] = algebraicAlgebraic0;
          break;
        case 1:
          algebraicsForTransfer[i] = algebraicAlgebraic1;
          break;
        case 2:
          algebraicsForTransfer[i] = algebraicAlgebraic2;
          break;
        case 3:
          algebraicsForTransfer[i] = algebraicAlgebraic3;
          break;
        case 4:
          algebraicsForTransfer[i] = algebraicAlgebraic4;
          break;
        case 5:
          algebraicsForTransfer[i] = algebraicAlgebraic5;
          break;
        case 6:
          algebraicsForTransfer[i] = algebraicAlgebraic6;
          break;
        case 7:
          algebraicsForTransfer[i] = algebraicAlgebraic7;
          break;
        case 8:
          algebraicsForTransfer[i] = algebraicAlgebraic8;
          break;
        case 9:
          algebraicsForTransfer[i] = algebraicAlgebraic9;
          break;
        case 10:
          algebraicsForTransfer[i] = algebraicAlgebraic10;
          break;
        case 11:
          algebraicsForTransfer[i] = algebraicAlgebraic11;
          break;
        case 12:
          algebraicsForTransfer[i] = algebraicAlgebraic12;
          break;
        case 13:
          algebraicsForTransfer[i] = algebraicAlgebraic13;
          break;
        case 14:
          algebraicsForTransfer[i] = algebraicAlgebraic14;
          break;
        case 15:
          algebraicsForTransfer[i] = algebraicAlgebraic15;
          break;
        case 16:
          algebraicsForTransfer[i] = algebraicAlgebraic16;
          break;
        case 17:
          algebraicsForTransfer[i] = algebraicAlgebraic17;
          break;
        case 18:
          algebraicsForTransfer[i] = algebraicAlgebraic18;
          break;
        case 19:
          algebraicsForTransfer[i] = algebraicAlgebraic19;
          break;
        case 20:
          algebraicsForTransfer[i] = algebraicAlgebraic20;
          break;
        case 21:
          algebraicsForTransfer[i] = algebraicAlgebraic21;
          break;
        case 22:
          algebraicsForTransfer[i] = algebraicAlgebraic22;
          break;
        case 23:
          algebraicsForTransfer[i] = algebraicAlgebraic23;
          break;
        case 24:
          algebraicsForTransfer[i] = algebraicAlgebraic24;
          break;
        case 25:
          algebraicsForTransfer[i] = algebraicAlgebraic25;
          break;
        case 26:
          algebraicsForTransfer[i] = algebraicAlgebraic26;
          break;
        case 27:
          algebraicsForTransfer[i] = algebraicAlgebraic27;
          break;
        case 28:
          algebraicsForTransfer[i] = algebraicAlgebraic28;
          break;
        case 29:
          algebraicsForTransfer[i] = algebraicAlgebraic29;
          break;
        case 30:
          algebraicsForTransfer[i] = algebraicAlgebraic30;
          break;
        case 31:
          algebraicsForTransfer[i] = algebraicAlgebraic31;
          break;
        // case 32: is a parameter
        case 33:
          algebraicsForTransfer[i] = algebraicAlgebraic33;
          break;
        case 34:
          algebraicsForTransfer[i] = algebraicAlgebraic34;
          break;
        case 35:
          algebraicsForTransfer[i] = algebraicAlgebraic35;
          break;
        case 36:
          algebraicsForTransfer[i] = algebraicAlgebraic36;
          break;
        case 37:
          algebraicsForTransfer[i] = algebraicAlgebraic37;
          break;
        case 38:
          algebraicsForTransfer[i] = algebraicAlgebraic38;
          break;
        case 39:
          algebraicsForTransfer[i] = algebraicAlgebraic39;
          break;
        case 40:
          algebraicsForTransfer[i] = algebraicAlgebraic40;
          break;
        case 41:
          algebraicsForTransfer[i] = algebraicAlgebraic41;
          break;
        case 42:
          algebraicsForTransfer[i] = algebraicAlgebraic42;
          break;
        case 43:
          algebraicsForTransfer[i] = algebraicAlgebraic43;
          break;
        case 44:
          algebraicsForTransfer[i] = algebraicAlgebraic44;
          break;
        case 45:
          algebraicsForTransfer[i] = algebraicAlgebraic45;
          break;
        case 46:
          algebraicsForTransfer[i] = algebraicAlgebraic46;
          break;
        case 47:
          algebraicsForTransfer[i] = algebraicAlgebraic47;
          break;
        case 48:
          algebraicsForTransfer[i] = algebraicAlgebraic48;
          break;
        case 49:
          algebraicsForTransfer[i] = algebraicAlgebraic49;
          break;
        case 50:
          algebraicsForTransfer[i] = algebraicAlgebraic50;
          break;
        case 51:
          algebraicsForTransfer[i] = algebraicAlgebraic51;
          break;
        case 52:
          algebraicsForTransfer[i] = algebraicAlgebraic52;
          break;
        case 53:
          algebraicsForTransfer[i] = algebraicAlgebraic53;
          break;
        case 54:
          algebraicsForTransfer[i] = algebraicAlgebraic54;
          break;
        case 55:
          algebraicsForTransfer[i] = algebraicAlgebraic55;
          break;
        case 56:
          algebraicsForTransfer[i] = algebraicAlgebraic56;
          break;
        case 57:
          algebraicsForTransfer[i] = algebraicAlgebraic57;
          break;
        case 58:
          algebraicsForTransfer[i] = algebraicAlgebraic58;
          break;
        case 59:
          algebraicsForTransfer[i] = algebraicAlgebraic59;
          break;
        case 60:
          algebraicsForTransfer[i] = algebraicAlgebraic60;
          break;
        case 61:
          algebraicsForTransfer[i] = algebraicAlgebraic61;
          break;
        case 62:
          algebraicsForTransfer[i] = algebraicAlgebraic62;
          break;
        case 63:
          algebraicsForTransfer[i] = algebraicAlgebraic63;
          break;
        case 64:
          algebraicsForTransfer[i] = algebraicAlgebraic64;
          break;
        case 65:
          algebraicsForTransfer[i] = algebraicAlgebraic65;
          break;
        case 66:
          algebraicsForTransfer[i] = algebraicAlgebraic66;
          break;
        case 67:
          algebraicsForTransfer[i] = algebraicAlgebraic67;
          break;
        case 68:
          algebraicsForTransfer[i] = algebraicAlgebraic68;
          break;
        case 69:
          algebraicsForTransfer[i] = algebraicAlgebraic69;
          break;
        case 70:
          algebraicsForTransfer[i] = algebraicAlgebraic70;
          break;

      }
    }
  }
}

// compute the rhs for a single instance, this can be used for computation of the equilibrium values of the states
#ifdef __cplusplus
extern "C"
#endif
void computeCellMLRightHandSideSingleInstance(void *context, double t, double *states, double *rates, double *algebraics, double *parameters)
{
  double VOI = t;   /* current simulation time */

  /* define constants */
  double CONSTANTS[105];
  CONSTANTS[0] = 1.0;
  CONSTANTS[1] = 4.8;
  CONSTANTS[2] = 150;
  CONSTANTS[3] = 0.000001;
  CONSTANTS[4] = 0.0025;
  CONSTANTS[5] = 0.0005;
  CONSTANTS[6] = 96485;
  CONSTANTS[7] = 350;
  CONSTANTS[8] = 350;
  CONSTANTS[9] = 0.0032;
  CONSTANTS[10] = 21875;
  CONSTANTS[11] = 21875;
  CONSTANTS[12] = 1.02;
  CONSTANTS[13] = -1.29;
  CONSTANTS[14] = 0.0081;
  CONSTANTS[15] = 0.288;
  CONSTANTS[16] = 0.0131;
  CONSTANTS[17] = 4.38;
  CONSTANTS[18] = 1.38;
  CONSTANTS[19] = 0.067;
  CONSTANTS[20] = -46;
  CONSTANTS[21] = -40;
  CONSTANTS[22] = -45;
  CONSTANTS[23] = 70;
  CONSTANTS[24] = -78;
  CONSTANTS[25] = -40;
  CONSTANTS[26] = 150;
  CONSTANTS[27] = 5.8;
  CONSTANTS[28] = 7.5;
  CONSTANTS[29] = 14.7;
  CONSTANTS[30] = 9;
  CONSTANTS[31] = 10;
  CONSTANTS[32] = 7;
  CONSTANTS[33] = 18;
  CONSTANTS[34] = 40;
  CONSTANTS[35] = 8314.41;
  CONSTANTS[36] = 293;
  CONSTANTS[37] = 19.65;
  CONSTANTS[38] = 64.8;
  CONSTANTS[39] = 804;
  CONSTANTS[40] = 11.1;
  CONSTANTS[41] = 0.4;
  CONSTANTS[42] = 950;
  CONSTANTS[43] = 1;
  CONSTANTS[44] = 1;
  CONSTANTS[45] = 13;
  CONSTANTS[46] = 10;
  CONSTANTS[47] = 0.000621;
  CONSTANTS[48] = 90;
  CONSTANTS[49] = 0.1;
  CONSTANTS[50] = 1.0;
  CONSTANTS[51] = 0.45;
  CONSTANTS[52] = 0.1;
  CONSTANTS[53] = 0.1;
  CONSTANTS[54] = 0.002;
  CONSTANTS[55] = 1000;
  CONSTANTS[56] = 0.2;
  CONSTANTS[57] = 0.2;
  CONSTANTS[58] = 4.5;
  CONSTANTS[59] = -20;
  CONSTANTS[60] = 4.875;
  CONSTANTS[61] = 1;
  CONSTANTS[62] = 0.00002;
  CONSTANTS[63] = 0.75;
  CONSTANTS[64] = 0.75;
  CONSTANTS[65] = 1.1;
  CONSTANTS[66] = 0.5;
  CONSTANTS[67] = 0.04425;
  CONSTANTS[68] = 0.115;
  CONSTANTS[69] = 140;
  CONSTANTS[70] = 0.0417;
  CONSTANTS[71] = 0.0005;
  CONSTANTS[72] = 1500;
  CONSTANTS[73] = 0.000033;
  CONSTANTS[74] = 0.003;
  CONSTANTS[75] = 0.000004;
  CONSTANTS[76] = 0.005;
  CONSTANTS[77] = 31000;
  CONSTANTS[78] = 0.15;
  CONSTANTS[79] = 30;
  CONSTANTS[80] = 0.0015;
  CONSTANTS[81] = 0.15;
  CONSTANTS[82] = 0.375;
  CONSTANTS[83] = 1.5;
  CONSTANTS[84] = 0;
  CONSTANTS[85] = 0.15;
  CONSTANTS[86] = 0.15;
  CONSTANTS[87] = 0.05;
  CONSTANTS[88] = 1.5;
  CONSTANTS[89] = 15;
  CONSTANTS[90] = 0.24;
  CONSTANTS[91] = 0.18;
  CONSTANTS[92] = 0.12;
  CONSTANTS[93] = 0.00002867;
  CONSTANTS[94] = 0.00000362;
  CONSTANTS[95] = 1;
  CONSTANTS[96] = 0.0001;
  CONSTANTS[97] = 6;
  CONSTANTS[98] = 300;
  CONSTANTS[99] =  0.950000*CONSTANTS[65]* 3.14159265358979*pow(CONSTANTS[66], 2.00000);
  CONSTANTS[100] =  0.0500000*CONSTANTS[65]* 3.14159265358979*pow(CONSTANTS[66], 2.00000);
  CONSTANTS[101] =  0.0100000*CONSTANTS[99];
  CONSTANTS[102] =  0.990000*CONSTANTS[99];
  CONSTANTS[103] =  0.0100000*CONSTANTS[100];
  CONSTANTS[104] =  0.990000*CONSTANTS[100];

  rates[28] = (((( ( CONSTANTS[98]*(states[18]+states[19]+states[20]+states[21]+states[22]))*((states[29] - states[28])/CONSTANTS[101]) -  CONSTANTS[60]*((states[28]/(states[28]+CONSTANTS[61]))/CONSTANTS[101]))+ CONSTANTS[62]*((states[29] - states[28])/CONSTANTS[101]))+ - CONSTANTS[63]*((states[28] - states[30])/CONSTANTS[101]))+- ( ( CONSTANTS[70]*states[28])*((CONSTANTS[72]+- states[33])+- states[35])+ - CONSTANTS[71]*states[33]))+- ( ( CONSTANTS[78]*states[28])*states[43]+ - CONSTANTS[79]*states[39]);
  rates[29] = ((( - ( CONSTANTS[98]*(states[18]+states[19]+states[20]+states[21]+states[22]))*((states[29] - states[28])/CONSTANTS[103])+ CONSTANTS[60]*((states[28]/(states[28]+CONSTANTS[61]))/CONSTANTS[103]))+ - CONSTANTS[62]*((states[29] - states[28])/CONSTANTS[103]))+ - CONSTANTS[64]*((states[29] - states[31])/CONSTANTS[103]))+- ( ( CONSTANTS[75]*states[29])*(CONSTANTS[77] - states[37])+ - CONSTANTS[76]*states[37]);
  rates[31] = ((( CONSTANTS[60]*((states[30]/(states[30]+CONSTANTS[61]))/CONSTANTS[104])+ - CONSTANTS[62]*((states[31]+- states[30])/CONSTANTS[104]))+ CONSTANTS[64]*((states[29]+- states[31])/CONSTANTS[104]))+- ( ( CONSTANTS[75]*states[31])*(CONSTANTS[77]+- states[38])+ - CONSTANTS[76]*states[38])) -  (1000.00/1.00000)*( CONSTANTS[95]*( states[54]*(0.00100000/1.00000)*states[31] - CONSTANTS[97])*( states[54]*(0.00100000/1.00000)*states[31] - CONSTANTS[97]>0.00000 ? 1.00000 : 0.00000)*(0.00100000/1.00000)*states[54]*states[31] -  CONSTANTS[96]*states[55]*(CONSTANTS[97] -  states[54]*(0.00100000/1.00000)*states[31])*(CONSTANTS[97] -  states[54]*(0.00100000/1.00000)*states[31]>0.00000 ? 1.00000 : 0.00000));
  rates[33] =  ( CONSTANTS[70]*states[28])*((CONSTANTS[72]+- states[33])+- states[35])+ - CONSTANTS[71]*states[33];
  rates[34] =  ( CONSTANTS[70]*states[30])*((CONSTANTS[72]+- states[34])+- states[36])+ - CONSTANTS[71]*states[34];
  rates[35] =  ( CONSTANTS[73]*(CONSTANTS[72]+- states[33]+- states[35]))*states[45]+ - CONSTANTS[74]*states[35];
  rates[36] =  ( CONSTANTS[73]*(CONSTANTS[72]+- states[34]+- states[36]))*states[46]+ - CONSTANTS[74]*states[36];
  rates[37] =  ( CONSTANTS[75]*states[29])*(CONSTANTS[77]+- states[37])+ - CONSTANTS[76]*states[37];
  rates[38] =  ( CONSTANTS[75]*states[31])*(CONSTANTS[77]+- states[38])+ - CONSTANTS[76]*states[38];
  rates[39] = ( ( CONSTANTS[78]*states[28])*states[43]+ - CONSTANTS[79]*states[39])+ - CONSTANTS[82]*((states[39]+- states[40])/CONSTANTS[101]);
  rates[40] = ( ( CONSTANTS[78]*states[30])*states[44]+ - CONSTANTS[79]*states[40])+ CONSTANTS[82]*((states[39]+- states[40])/CONSTANTS[102]);
  rates[41] = ( ( CONSTANTS[80]*states[45])*states[43]+ - CONSTANTS[81]*states[41])+ - CONSTANTS[82]*((states[41]+- states[42])/CONSTANTS[101]);
  rates[42] = ( ( CONSTANTS[80]*states[46])*states[44]+ - CONSTANTS[81]*states[42])+ CONSTANTS[82]*((states[41]+- states[42])/CONSTANTS[102]);
  rates[43] = (- ( ( CONSTANTS[78]*states[28])*states[43]+ - CONSTANTS[79]*states[39])+- ( ( CONSTANTS[80]*states[45])*states[43]+ - CONSTANTS[81]*states[41]))+ - CONSTANTS[82]*((states[43]+- states[44])/CONSTANTS[101]);
  rates[44] = (- ( ( CONSTANTS[78]*states[30])*states[44]+ - CONSTANTS[79]*states[40])+- ( ( CONSTANTS[80]*states[46])*states[44]+ - CONSTANTS[81]*states[42]))+ CONSTANTS[82]*((states[43]+- states[44])/CONSTANTS[102]);
  rates[45] = (- ( ( CONSTANTS[73]*(CONSTANTS[72]+- states[33]+- states[35]))*states[45]+ - CONSTANTS[74]*states[35])+- ( ( CONSTANTS[80]*states[45])*states[43]+ - CONSTANTS[81]*states[41]))+ - CONSTANTS[83]*((states[45]+- states[46])/CONSTANTS[101]);
  rates[46] = (- ( ( CONSTANTS[73]*(CONSTANTS[72]+- states[34]+- states[36]))*states[46]+ - CONSTANTS[74]*states[36])+- ( ( CONSTANTS[80]*states[46])*states[44]+ - CONSTANTS[81]*states[42]))+ CONSTANTS[83]*((states[45]+- states[46])/CONSTANTS[102]);
  rates[47] = (( ( CONSTANTS[67]*states[30])*states[32]+ - CONSTANTS[68]*states[47])+ - CONSTANTS[86]*states[47])+ CONSTANTS[87]*states[50];
  rates[49] = (((( CONSTANTS[67]*states[30]*states[48]+ - CONSTANTS[68]*states[49])+ CONSTANTS[84]*states[32])+ - CONSTANTS[85]*states[49])+ ( - CONSTANTS[67]*states[30])*states[49])+ CONSTANTS[68]*states[50];
  rates[50] = ((((( CONSTANTS[67]*states[30]*states[49]+ - CONSTANTS[68]*states[50])+ CONSTANTS[86]*states[47])+ - CONSTANTS[87]*states[50])+ - CONSTANTS[88]*states[50])+ CONSTANTS[89]*states[51])+ CONSTANTS[92]*states[52];
  rates[51] = (( CONSTANTS[88]*states[50]+ - CONSTANTS[89]*states[51])+ CONSTANTS[91]*states[52])+ - CONSTANTS[90]*states[51];
  rates[52] = ( - CONSTANTS[91]*states[52]+ CONSTANTS[90]*states[51])+ - CONSTANTS[92]*states[52];
  rates[53] =  (0.00100000/1.00000)*( CONSTANTS[90]*states[51] -  CONSTANTS[91]*states[52])+ -1.00000*CONSTANTS[93]*states[53]+ -1.00000*CONSTANTS[94]*((states[53] - states[54])/CONSTANTS[102]);
  rates[54] =  CONSTANTS[94]*((states[53] - states[54])/CONSTANTS[104]) -  1.00000*( CONSTANTS[95]*( states[54]*(0.00100000/1.00000)*states[31] - CONSTANTS[97])*( states[54]*(0.00100000/1.00000)*states[31] - CONSTANTS[97]>0.00000 ? 1.00000 : 0.00000)*(0.00100000/1.00000)*states[54]*states[31] -  CONSTANTS[96]*states[55]*(CONSTANTS[97] -  states[54]*(0.00100000/1.00000)*states[31])*(CONSTANTS[97] -  states[54]*(0.00100000/1.00000)*states[31]>0.00000 ? 1.00000 : 0.00000));
  rates[55] =  1.00000*( CONSTANTS[95]*( states[54]*(0.00100000/1.00000)*states[31] - CONSTANTS[97])*( states[54]*(0.00100000/1.00000)*states[31] - CONSTANTS[97]>0.00000 ? 1.00000 : 0.00000)*(0.00100000/1.00000)*states[54]*states[31] -  CONSTANTS[96]*states[55]*(CONSTANTS[97] -  states[54]*(0.00100000/1.00000)*states[31])*(CONSTANTS[97] -  states[54]*(0.00100000/1.00000)*states[31]>0.00000 ? 1.00000 : 0.00000));
  algebraics[12] = CONSTANTS[69]+- states[32]+- states[47]+- states[48]+- states[49]+- states[50]+- states[51]+- states[52];
  rates[30] = (((( - CONSTANTS[60]*((states[30]/(states[30]+CONSTANTS[61]))/CONSTANTS[102])+ CONSTANTS[62]*((states[31]+- states[30])/CONSTANTS[102]))+ CONSTANTS[63]*((states[28] - states[30])/CONSTANTS[102]))+- ((((((( CONSTANTS[67]*states[30]*algebraics[12]+ - CONSTANTS[68]*states[32])+ CONSTANTS[67]*states[30]*states[32])+ - CONSTANTS[68]*states[47])+ CONSTANTS[67]*states[30]*states[48])+ - CONSTANTS[68]*states[49])+ CONSTANTS[67]*states[30]*states[49])+ - CONSTANTS[68]*states[50]))+- ( ( CONSTANTS[70]*states[30])*(CONSTANTS[72]+- states[34]+- states[36])+ - CONSTANTS[71]*states[34]))+- ( ( CONSTANTS[78]*states[30])*states[44]+ - CONSTANTS[79]*states[40]);
  rates[32] = (((( ( CONSTANTS[67]*states[30])*algebraics[12]+ - CONSTANTS[68]*states[32])+ ( - CONSTANTS[67]*states[30])*states[32])+ CONSTANTS[68]*states[47])+ - CONSTANTS[84]*states[32])+ CONSTANTS[85]*states[49];
  rates[48] = (( ( - CONSTANTS[67]*states[30])*states[48]+ CONSTANTS[68]*states[49])+ CONSTANTS[84]*algebraics[12])+ - CONSTANTS[85]*states[48];
  algebraics[1] =  CONSTANTS[16]*((states[0] - CONSTANTS[21])/(1.00000 - exp(- ((states[0] - CONSTANTS[21])/CONSTANTS[32]))));
  algebraics[14] =  CONSTANTS[19]*exp(- ((states[0] - CONSTANTS[21])/CONSTANTS[34]));
  rates[8] =  algebraics[1]*(1.00000 - states[8]) -  algebraics[14]*states[8];
  algebraics[2] = 1.00000/(1.00000+exp((states[0] - CONSTANTS[25])/CONSTANTS[28]));
  algebraics[15] =  1000.00*exp(- ((states[0]+40.0000)/25.7500));
  rates[9] = (algebraics[2] - states[9])/algebraics[15];
  algebraics[4] =  CONSTANTS[15]*((states[0] - CONSTANTS[20])/(1.00000 - exp(- ((states[0] - CONSTANTS[20])/CONSTANTS[31]))));
  algebraics[17] =  CONSTANTS[18]*exp(- ((states[0] - CONSTANTS[20])/CONSTANTS[33]));
  rates[10] =  algebraics[4]*(1.00000 - states[10]) -  algebraics[17]*states[10];
  algebraics[3] =  CONSTANTS[14]*exp(- ((states[0] - CONSTANTS[22])/CONSTANTS[29]));
  algebraics[16] = CONSTANTS[17]/(1.00000+exp(- ((states[0] - CONSTANTS[22])/CONSTANTS[30])));
  rates[11] =  algebraics[3]*(1.00000 - states[11]) -  algebraics[16]*states[11];
  algebraics[5] = 1.00000/(1.00000+exp((states[0] - CONSTANTS[24])/CONSTANTS[27]));
  algebraics[18] = 8571.00/(0.200000+ 5.65000*pow((states[0]+CONSTANTS[48])/100.000, 2.00000));
  rates[12] = (algebraics[5] - states[12])/algebraics[18];
  algebraics[6] =  CONSTANTS[16]*((states[1] - CONSTANTS[21])/(1.00000 - exp(- ((states[1] - CONSTANTS[21])/CONSTANTS[32]))));
  algebraics[19] =  CONSTANTS[19]*exp(- ((states[1] - CONSTANTS[21])/CONSTANTS[34]));
  rates[13] =  algebraics[6]*(1.00000 - states[13]) -  algebraics[19]*states[13];
  algebraics[7] = 1.00000/(1.00000+exp((states[1] - CONSTANTS[25])/CONSTANTS[28]));
  algebraics[20] =  1.00000*exp(- ((states[1]+40.0000)/25.7500));
  rates[14] = (algebraics[7] - states[14])/algebraics[20];
  algebraics[9] =  CONSTANTS[15]*((states[1] - CONSTANTS[20])/(1.00000 - exp(- ((states[1] - CONSTANTS[20])/CONSTANTS[31]))));
  algebraics[22] =  CONSTANTS[18]*exp(- ((states[1] - CONSTANTS[20])/CONSTANTS[33]));
  rates[15] =  algebraics[9]*(1.00000 - states[15]) -  algebraics[22]*states[15];
  algebraics[8] =  CONSTANTS[14]*exp(- ((states[1] - CONSTANTS[22])/CONSTANTS[29]));
  algebraics[21] = CONSTANTS[17]/(1.00000+exp(- ((states[1] - CONSTANTS[22])/CONSTANTS[30])));
  rates[16] =  algebraics[8]*(1.00000 - states[16]) -  algebraics[21]*states[16];
  algebraics[10] = 1.00000/(1.00000+exp((states[1] - CONSTANTS[24])/CONSTANTS[27]));
  algebraics[23] = 8571.00/(0.200000+ 5.65000*pow((states[1]+CONSTANTS[48])/100.000, 2.00000));
  rates[17] = (algebraics[10] - states[17])/algebraics[23];
  algebraics[11] =  0.500000*CONSTANTS[57]*exp((states[1] - CONSTANTS[59])/( 8.00000*CONSTANTS[58]));
  algebraics[24] =  0.500000*CONSTANTS[57]*exp((CONSTANTS[59] - states[1])/( 8.00000*CONSTANTS[58]));
  rates[23] =  - CONSTANTS[54]*states[23]+ CONSTANTS[55]*states[18]+ -4.00000*algebraics[11]*states[23]+ algebraics[24]*states[24];
  rates[18] =  CONSTANTS[54]*states[23]+ - CONSTANTS[55]*states[18]+( -4.00000*algebraics[11]*states[18])/CONSTANTS[56]+ CONSTANTS[56]*algebraics[24]*states[19];
  rates[24] =  4.00000*algebraics[11]*states[23]+ - algebraics[24]*states[24]+( - CONSTANTS[54]*states[24])/CONSTANTS[56]+ CONSTANTS[56]*CONSTANTS[55]*states[19]+ -3.00000*algebraics[11]*states[24]+ 2.00000*algebraics[24]*states[25];
  rates[19] = ( CONSTANTS[54]*states[24])/CONSTANTS[56]+ - CONSTANTS[55]*CONSTANTS[56]*states[19]+( 4.00000*algebraics[11]*states[18])/CONSTANTS[56]+ - CONSTANTS[56]*algebraics[24]*states[19]+( -3.00000*algebraics[11]*states[19])/CONSTANTS[56]+ 2.00000*CONSTANTS[56]*algebraics[24]*states[20];
  rates[25] =  3.00000*algebraics[11]*states[24]+ -2.00000*algebraics[24]*states[25]+( - CONSTANTS[54]*states[25])/pow(CONSTANTS[56], 2.00000)+ pow(CONSTANTS[56], 2.00000)*CONSTANTS[55]*states[20]+ -2.00000*algebraics[11]*states[25]+ 3.00000*algebraics[24]*states[26];
  rates[20] = ( 3.00000*algebraics[11]*states[19])/CONSTANTS[56]+ -2.00000*CONSTANTS[56]*algebraics[24]*states[20]+( CONSTANTS[54]*states[25])/pow(CONSTANTS[56], 2.00000)+ - CONSTANTS[55]*pow(CONSTANTS[56], 2.00000)*states[20]+( -2.00000*algebraics[11]*states[20])/CONSTANTS[56]+ 3.00000*CONSTANTS[56]*algebraics[24]*states[21];
  rates[26] =  2.00000*algebraics[11]*states[25]+ -3.00000*algebraics[24]*states[26]+( - CONSTANTS[54]*states[26])/pow(CONSTANTS[56], 3.00000)+ CONSTANTS[55]*pow(CONSTANTS[56], 3.00000)*states[21]+ - algebraics[11]*states[26]+ 4.00000*algebraics[24]*states[27];
  rates[21] = ( CONSTANTS[54]*states[26])/pow(CONSTANTS[56], 3.00000)+ - CONSTANTS[55]*pow(CONSTANTS[56], 3.00000)*states[21]+( 2.00000*algebraics[11]*states[20])/CONSTANTS[56]+ -3.00000*algebraics[24]*CONSTANTS[56]*states[21]+( - algebraics[11]*states[21])/CONSTANTS[56]+ 4.00000*CONSTANTS[56]*algebraics[24]*states[22];
  rates[27] =  algebraics[11]*states[26]+ -4.00000*algebraics[24]*states[27]+( - CONSTANTS[54]*states[27])/pow(CONSTANTS[56], 4.00000)+ CONSTANTS[55]*pow(CONSTANTS[56], 4.00000)*states[22];
  rates[22] = ( algebraics[11]*states[21])/CONSTANTS[56]+ -4.00000*CONSTANTS[56]*algebraics[24]*states[22]+( CONSTANTS[54]*states[27])/pow(CONSTANTS[56], 4.00000)+ - CONSTANTS[55]*pow(CONSTANTS[56], 4.00000)*states[22];
  algebraics[30] =  states[0]*((states[3] -  states[4]*exp(( -1.00000*CONSTANTS[6]*states[0])/( CONSTANTS[35]*CONSTANTS[36])))/(1.00000 - exp(( -1.00000*CONSTANTS[6]*states[0])/( CONSTANTS[35]*CONSTANTS[36]))));
  algebraics[13] =  (( CONSTANTS[35]*CONSTANTS[36])/CONSTANTS[6])*log(states[4]/states[3]);
  algebraics[37] =  states[4]*exp( ( - CONSTANTS[41]*algebraics[13])*(CONSTANTS[6]/( CONSTANTS[35]*CONSTANTS[36])));
  algebraics[38] =  CONSTANTS[40]*(pow(algebraics[37], 2.00000)/(CONSTANTS[42]+pow(algebraics[37], 2.00000)));
  algebraics[39] = 1.00000 - pow(1.00000+( CONSTANTS[43]*(1.00000+pow(algebraics[37], 2.00000)/CONSTANTS[42]))/( pow(CONSTANTS[46], 2.00000)*exp(( 2.00000*(1.00000 - CONSTANTS[41])*states[0]*CONSTANTS[6])/( CONSTANTS[35]*CONSTANTS[36]))), -1.00000);
  algebraics[40] =  algebraics[38]*algebraics[39];
  algebraics[41] =  algebraics[40]*(algebraics[30]>0.00000 ? 1.00000 : 0.00000)*(algebraics[30]/50.0000);
  algebraics[42] =  ( CONSTANTS[38]*pow(states[8], 4.00000))*states[9];
  algebraics[43] =  algebraics[42]*(algebraics[30]/50.0000);
  algebraics[47] =  (1.00000/7.00000)*(exp(states[7]/67.3000) - 1.00000);
  algebraics[48] = pow(1.00000+ 0.120000*exp( -0.100000*states[0]*(CONSTANTS[6]/( CONSTANTS[35]*CONSTANTS[36])))+ 0.0400000*algebraics[47]*exp(- ( states[0]*(CONSTANTS[6]/( CONSTANTS[35]*CONSTANTS[36])))), -1.00000);
  algebraics[49] =  CONSTANTS[6]*(CONSTANTS[47]/( pow(1.00000+CONSTANTS[44]/states[4], 2.00000)*pow(1.00000+CONSTANTS[45]/states[5], 3.00000)));
  algebraics[50] =  algebraics[49]*algebraics[48];
  rates[4] = (algebraics[41]+algebraics[43]+CONSTANTS[12]+ - 2.00000*algebraics[50])/( (1000.00/1.00000)*CONSTANTS[6]*CONSTANTS[5])+(states[2] - states[4])/CONSTANTS[10];
  algebraics[45] =  ( ( CONSTANTS[39]*pow(states[10], 3.00000))*states[11])*states[12];
  algebraics[44] =  states[0]*((states[5] -  states[7]*exp(( -1.00000*CONSTANTS[6]*states[0])/( CONSTANTS[35]*CONSTANTS[36])))/(1.00000 - exp(( -1.00000*CONSTANTS[6]*states[0])/( CONSTANTS[35]*CONSTANTS[36]))));
  algebraics[46] =  algebraics[45]*(algebraics[44]/75.0000);
  rates[7] = (algebraics[46]+CONSTANTS[13]+ 3.00000*algebraics[50])/( (1000.00/1.00000)*CONSTANTS[6]*CONSTANTS[5])+(states[6] - states[7])/CONSTANTS[11];
  algebraics[0] =  (1000.00/1.00000)*((states[0] - states[1])/CONSTANTS[2]);
  algebraics[26] = 156.500/(5.00000+exp(( - CONSTANTS[6]*algebraics[13])/( CONSTANTS[35]*CONSTANTS[36])));
  algebraics[27] = 156.500 -  5.00000*algebraics[26];
  algebraics[34] =  states[0]*((algebraics[26] -  algebraics[27]*exp(( CONSTANTS[6]*states[0])/( CONSTANTS[35]*CONSTANTS[36])))/(1.00000 - exp(( CONSTANTS[6]*states[0])/( CONSTANTS[35]*CONSTANTS[36]))));
  algebraics[33] = 1.00000/(1.00000+exp((states[0] - CONSTANTS[23])/CONSTANTS[26]));
  algebraics[35] =  CONSTANTS[37]*pow(algebraics[33], 4.00000);
  algebraics[36] =  algebraics[35]*(algebraics[34]/45.0000);
    // (not assigning to a parameter) algebraics = (VOI>=0.00000&&VOI<0.500000 VOI>=0.00000&&VOI<0.500000 ? 150.000 : VOI>=50.0000&&VOI<50.5000  VOI>=50.0000&&VOI<50.5000 ? 150.000 : VOI>=100.000&&VOI<100.500  VOI>=100.000&&VOI<100.500 ? 150.000 : VOI>=150.000&&VOI<150.500  VOI>=150.000&&VOI<150.500 ? 150.000 : VOI>=200.000&&VOI<200.500  VOI>=200.000&&VOI<200.500 ? 150.000 : VOI>=250.000&&VOI<250.500  VOI>=250.000&&VOI<250.500 ? 150.000 : VOI>=300.000&&VOI<300.500  VOI>=300.000&&VOI<300.500 ? 150.000 : VOI>=350.000&&VOI<350.500  VOI>=350.000&&VOI<350.500 ? 150.000 : VOI>=400.000&&VOI<400.500  VOI>=400.000&&VOI<400.500 ? 150.000 : 0.00000);
  algebraics[51] = algebraics[36]+algebraics[41]+algebraics[43]+algebraics[46]+algebraics[50]+- parameters[0];
  rates[0] = - ((algebraics[51]+algebraics[0])/CONSTANTS[0]);
  algebraics[31] =  states[1]*((states[3] -  states[2]*exp(( -1.00000*CONSTANTS[6]*states[1])/( CONSTANTS[35]*CONSTANTS[36])))/(1.00000 - exp(( -1.00000*CONSTANTS[6]*states[1])/( CONSTANTS[35]*CONSTANTS[36]))));
  algebraics[25] =  (( CONSTANTS[35]*CONSTANTS[36])/CONSTANTS[6])*log(states[2]/states[3]);
  algebraics[56] =  states[2]*exp( ( - CONSTANTS[41]*algebraics[25])*(CONSTANTS[6]/( CONSTANTS[35]*CONSTANTS[36])));
  algebraics[57] =  CONSTANTS[40]*(pow(algebraics[56], 2.00000)/(CONSTANTS[42]+pow(algebraics[56], 2.00000)));
  algebraics[58] = 1.00000 - pow(1.00000+( CONSTANTS[43]*(1.00000+pow(algebraics[56], 2.00000)/CONSTANTS[42]))/( pow(CONSTANTS[46], 2.00000)*exp(( 2.00000*(1.00000 - CONSTANTS[41])*states[1]*CONSTANTS[6])/( CONSTANTS[35]*CONSTANTS[36]))), -1.00000);
  algebraics[59] =  algebraics[57]*algebraics[58];
  algebraics[60] =  CONSTANTS[50]*algebraics[59]*(algebraics[31]/50.0000);
  algebraics[61] =  ( CONSTANTS[38]*pow(states[13], 4.00000))*states[14];
  algebraics[62] =  CONSTANTS[51]*algebraics[61]*(algebraics[31]/50.0000);
  algebraics[66] =  (1.00000/7.00000)*(exp(states[6]/67.3000) - 1.00000);
  algebraics[67] = pow(1.00000+ 0.120000*exp( -0.100000*states[1]*(CONSTANTS[6]/( CONSTANTS[35]*CONSTANTS[36])))+ 0.0400000*algebraics[66]*exp(- ( states[1]*(CONSTANTS[6]/( CONSTANTS[35]*CONSTANTS[36])))), -1.00000);
  algebraics[68] =  CONSTANTS[6]*(CONSTANTS[47]/( pow(1.00000+CONSTANTS[44]/states[2], 2.00000)*pow(1.00000+CONSTANTS[45]/states[5], 3.00000)));
  algebraics[69] =  CONSTANTS[53]*algebraics[68]*algebraics[67];
  rates[3] =  - CONSTANTS[9]*((algebraics[60]+algebraics[62]+CONSTANTS[12]+ - 2.00000*algebraics[69])/( (1000.00/1.00000)*CONSTANTS[6]*CONSTANTS[3])) - (algebraics[41]+algebraics[43]+CONSTANTS[12]+ -2.00000*algebraics[50])/( (1000.00/1.00000)*CONSTANTS[6]*CONSTANTS[4]);
  rates[2] = (algebraics[60]+algebraics[62]+CONSTANTS[12]+ - 2.00000*algebraics[69])/( (1000.00/1.00000)*CONSTANTS[6]*CONSTANTS[3]) - (states[2] - states[4])/CONSTANTS[7];
  algebraics[64] =  ( ( CONSTANTS[39]*pow(states[15], 3.00000))*states[16])*states[17];
  algebraics[63] =  states[1]*((states[5] -  states[6]*exp(( -1.00000*CONSTANTS[6]*states[1])/( CONSTANTS[35]*CONSTANTS[36])))/(1.00000 - exp(( -1.00000*CONSTANTS[6]*states[1])/( CONSTANTS[35]*CONSTANTS[36]))));
  algebraics[65] =  CONSTANTS[52]*algebraics[64]*(algebraics[63]/75.0000);
  rates[5] =  - CONSTANTS[9]*((algebraics[65]+CONSTANTS[13]+ 3.00000*algebraics[69])/( (1000.00/1.00000)*CONSTANTS[6]*CONSTANTS[3])) - (algebraics[46]+CONSTANTS[13]+ 3.00000*algebraics[50])/( (1000.00/1.00000)*CONSTANTS[6]*CONSTANTS[4]);
  rates[6] = (algebraics[65]+CONSTANTS[13]+ 3.00000*algebraics[69])/( (1000.00/1.00000)*CONSTANTS[6]*CONSTANTS[3]) - (states[6] - states[7])/CONSTANTS[8];
  algebraics[28] = 156.500/(5.00000+exp(( - CONSTANTS[6]*algebraics[25])/( CONSTANTS[35]*CONSTANTS[36])));
  algebraics[29] = 156.500 -  5.00000*algebraics[28];
  algebraics[53] =  states[1]*((algebraics[28] -  algebraics[29]*exp(( CONSTANTS[6]*states[1])/( CONSTANTS[35]*CONSTANTS[36])))/(1.00000 - exp(( CONSTANTS[6]*states[1])/( CONSTANTS[35]*CONSTANTS[36]))));
  algebraics[52] = 1.00000/(1.00000+exp((states[1] - CONSTANTS[23])/CONSTANTS[26]));
  algebraics[54] =  CONSTANTS[37]*pow(algebraics[52], 4.00000);
  algebraics[55] =  CONSTANTS[49]*algebraics[54]*(algebraics[53]/45.0000);
  algebraics[70] = algebraics[55]+algebraics[60]+algebraics[62]+algebraics[65]+algebraics[69];
  rates[1] = - ((algebraics[70] - algebraics[0]/CONSTANTS[1])/CONSTANTS[0]);
}
