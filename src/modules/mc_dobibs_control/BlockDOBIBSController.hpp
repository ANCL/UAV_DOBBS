#pragma once

#include <px4_posix.h>
#include <px4_defines.h>
#include <controllib/uorb/blocks.hpp>
#include <math.h>
#include <vector>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
using namespace std;


using namespace control;

class BlockDOBIBSController : public control::BlockDOBIBSLoop
{
public:
    BlockDOBIBSController() :
        BlockDOBIBSLoop(NULL,"DOBIBS"),
        _i_u(this,"I_U_PAR"),
        _i_df_0(this,"I_DF_1"),
        _i_df_1(this,"I_DF_2"),
        _i_df_2(this,"I_DF_3"),
        _i_dtau_0(this,"I_DF_1"),
        _i_dtau_1(this,"I_DF_2"),
        _i_dtau_2(this,"I_DF_3"),
        _fds(),
        _t(0),
        pd(),
        dpd(),
        ddpd(),
        dddpd(),
        ddddpd()
    {
        _fds[0].fd = _pos.getHandle();
        _fds[0].events = POLLIN;
        switch_traj = param_find("MC_DOBIBS_TRAJ");
        param_get(switch_traj, &_switch_traj);
        param_get(param_find("DOBIBS_K1"), &k1);
        param_get(param_find("DOBIBS_K2"), &k2);
        param_get(param_find("DOBIBS_K3"), &k3);
        param_get(param_find("DOBIBS_K4"), &k4);
        param_get(param_find("DOBIBS_FDOG"), &k_df);
        param_get(param_find("DOBIBS_TAUDOG"), &k_dtau);
        param_get(param_find("DOBIBS_J1NF"), &J1_nf);
        param_get(param_find("DOBIBS_J2NF"), &J2_nf);
        param_get(param_find("DOBIBS_J3NF"), &J3_nf);
        param_get(param_find("DOBIBS_PSI_K1"), &k_psi_1);
        param_get(param_find("DOBIBS_PSI_K2"), &k_psi_2);

            }    void update();
    int parameters_update();

private:
    BlockIntegral _i_u;
    BlockIntegral _i_df_0;
    BlockIntegral _i_df_1;
    BlockIntegral _i_df_2;
    BlockIntegral _i_dtau_0;
    BlockIntegral _i_dtau_1;
    BlockIntegral _i_dtau_2;
    px4_pollfd_struct_t _fds[1];
    uint64_t _t;

    // params
    param_t switch_traj;


    // Integral Backstepping  Gains
    float k1;
    float k2;
    float k3;
    float k4;
    //Observer gain
    float k_df;
    float k_dtau;
    float J1_nf;
    float J2_nf;
    float J3_nf;
    float k_psi_1;
    float k_psi_2;



    float obs=0;
    float u=15.69;
    float u_dot=0;
    float d_zdf_0=0;
    float z_df_0=0;
    float d_zdf_1=0;
    float z_df_1=0;
    float d_zdf_2=0;
    float z_df_2=0;
    float d_zdtau_0=0;
    float z_dtau_0=0;
    float d_zdtau_1=0;
    float z_dtau_1=0;
    float d_zdtau_2=0;
    float z_dtau_2=0;
    float traj_t=0;
    float T=20;
    int traj=0;
    float switch_time=0;

    int _switch_traj;
    matrix::Vector<float, 3>  pd;
    matrix::Vector<float, 3>  dpd;
    matrix::Vector<float, 3>  ddpd;
    matrix::Vector<float, 3>  dddpd;
    matrix::Vector<float, 3>  ddddpd;   
};
