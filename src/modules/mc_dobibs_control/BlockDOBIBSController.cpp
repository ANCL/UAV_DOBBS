#include "BlockDOBIBSController.hpp"
#include <uORB/topics/battery_status.h>

int BlockDOBIBSController::parameters_update(){
        param_get(param_find("MC_DOBIBS_TRAJ"), &_switch_traj);
        param_get(param_find("DOBIBS_IBS_K1"), &k1);
        param_get(param_find("DOBIBS_IBS_K2"), &k2);
        param_get(param_find("DOBIBS_IBS_K3"), &k3);
        param_get(param_find("DOBIBS_IBS_K4"), &k4);
        param_get(param_find("DOBIBS_IBS_K5"), &k5);
        param_get(param_find("DOBIBS_BS_K1"), &k1_bs);
        param_get(param_find("DOBIBS_BS_K2"), &k2_bs);
        param_get(param_find("DOBIBS_BS_K3"), &k3_bs);
        param_get(param_find("DOBIBS_BS_K4"), &k4_bs);
        param_get(param_find("DOBIBS_FDOG"), &k_df);
        param_get(param_find("DOBIBS_TAUDOG"), &k_dtau);
        param_get(param_find("DOBIBS_DOBMU"), &DOBMU);
        param_get(param_find("DOBIBS_J1NF"), &J1_nf);
        param_get(param_find("DOBIBS_J2NF"), &J2_nf);
        param_get(param_find("DOBIBS_J3NF"), &J3_nf);
        param_get(param_find("DOBIBS_PSI_K1"), &k_psi_1);
        param_get(param_find("DOBIBS_PSI_K2"), &k_psi_2);
        return OK;
}

void BlockDOBIBSController::update(){
        parameters_update();
        // wait for an image feature, timeout every 500 ms
        int poll_ret = px4_poll(_fds,(sizeof(_fds) / sizeof(_fds[0])), 500);// to check if

        if (poll_ret <= 0)
        {
                if (poll_ret == 0) warn("time out");
                else warn("poll error %d, %d", poll_ret, errno);
                _att_sp.get().valid=false;
        }
        else {
                uint64_t t1 = hrt_absolute_time();

                static uint64_t last_run = 0;
                float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
                last_run = hrt_absolute_time();

                _t=t1;
                //Set message timestamp
                _att_sp.get().timestamp = t1;
                _control_sp.get().timestamp = t1;
                // check for sane values of dt
                if (dt>1.0f || dt<0) {
                        warn("dt=%3.3f",(double)dt);
                        _att_sp.get().valid=false;
                } else {
                        // set dt for all child blocks
                        setDt(dt);

                        // check for new updates   //_param_update is a topic structure
                        if (_param_update.updated())
                        {
                                updateParams();
                        }
                        // get new information from subscriptions
                        updateSubscriptions();
                        
                        //initialization
                        float m = 1.6;
                        float g = 9.81;


                        matrix::Matrix<float, 3, 3>  J;

                        J(0,0)=0.03;
                        J(0,1)=0.00;
                        J(0,2)=0.00;

                        J(1,0)=0.00;
                        J(1,1)=0.03;
                        J(1,2)=0.00;

                        J(2,0)=0.00;
                        J(2,1)=0.00;
                        J(2,2)=0.05;



                        //variable state definition
                        matrix::Vector<float, 3>  p;
                        p(0)=_pos.get().x;
                        p(1)=_pos.get().y;
                        p(2)=_pos.get().z;

                        matrix::Vector<float, 3>  v;

                        v(0) = _pos.get().vx;
                        v(1) = _pos.get().vy;
                        v(2) = _pos.get().vz;

                        matrix::Eulerf euler = matrix::Quatf(_att.get().q);// converting quaternion to Euler angles
                        float phi =euler.phi();
                        float theta = euler.theta();
                        float psi = euler.psi();

                        matrix::Vector<float, 3>  w;
                        w(0) = _att.get().rollspeed;
                        w(1) = _att.get().pitchspeed;
                        w(2) = _att.get().yawspeed;

                        // matrix::Dcm<float> R_att(_att.get().q);// not working!

                        matrix::Matrix<float, 3, 3>  R;

                        R(0,0)=cos(psi)*cos(theta);
                        R(0,1)=-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
                        R(0,2)=sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);

                        R(1,0)=sin(psi)*cos(theta);
                        R(1,1)=cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
                        R(1,2)=-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);

                        R(2,0)=-sin(theta);
                        R(2,1)=cos(theta)*sin(phi);
                        R(2,2)=cos(theta)*cos(phi);

                      //  matrix::Matrix<float, 4>  quad_input;

                        float quad_input[4];

                        //vector and matrix definition
                        matrix::Matrix<float, 3, 3>  Ie3;
                        Ie3(0,0)=1;
                        Ie3(0,1)=0;
                        Ie3(0,2)=0;

                        Ie3(1,0)=0;
                        Ie3(1,1)=1;
                        Ie3(1,2)=0;

                        Ie3(2,0)=0;
                        Ie3(2,1)=0;
                        Ie3(2,2)=0;

                        matrix::Matrix<float, 3, 3>  Sw;
                        Sw(0,0)=0;
                        Sw(0,1)=-w(2);
                        Sw(0,2)=w(1);

                        Sw(1,0)=w(2);
                        Sw(1,1)=0;
                        Sw(1,2)=-w(0);

                        Sw(2,0)=-w(1);
                        Sw(2,1)=w(0);
                        Sw(2,2)=0;

                        matrix::Vector<float, 3>  n1;
                        n1(0)=1;
                        n1(1)=0;
                        n1(2)=0;

                        matrix::Vector<float, 3>  n2;
                        n2(0)=0;
                        n2(1)=1;
                        n2(2)=0;

                        matrix::Vector<float, 3>  n3;
                        n3(0)=0;
                        n3(1)=0;
                        n3(2)=1;

                        // desired trajectory

                        if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL2 && traj == 0){
                            traj=1;
                            traj_t=t1;
                            switch_time=t1;

                        }

                        if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL1 && traj == 1){
                            switch_time=t1;
                        }


                        float T_d=20.0;
                        if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL1){
                            traj=0;
                            T=T_d;
                        }

                        float t=(t1-traj_t)/1000000.0f;
                        float A=3.00*traj;
                        float B=3.00*traj;
                        float pi=3.14;

                        if (traj==1 && T>10.0f){
                            T=T_d-t;
                        }
                        
                        if (_switch_traj == 0){
                            //  Fig 8
                            
                            pd(0)=A*sinf(2*pi*t/T);
                            pd(1)=B*sinf(4*pi*t/T)/4;
                            pd(2)=-1.1f+0.3f*sinf(2*pi*t/T);

                            dpd(0)=(2*A*pi*cosf((2*pi*t)/T))/T;
                            dpd(1)=(B*pi*cosf((4*pi*t)/T))/T;
                            dpd(2)=0;

                            
                            ddpd(0)=-(4*A*(pi*pi)*sinf((2*pi*t)/T))/(T*T);
                            ddpd(1)=-(4*B*(pi*pi)*sinf((4*pi*t)/T))/(T*T);
                            ddpd(2)=0;

                            
                            dddpd(0)=-(8*A*(pi*pi*pi)*cosf((2*pi*t)/T))/(T*T*T);
                            dddpd(1)=-(16*B*(pi*pi*pi)*cosf((4*pi*t)/T))/(T*T*T);
                            dddpd(2)=0;

                            
                            ddddpd(0)=(16*A*(pi*pi*pi*pi)*sinf((2*pi*t)/T))/(T*T*T*T);
                            ddddpd(1)=(64*B*(pi*pi*pi*pi)*sinf((4*pi*t)/T))/(T*T*T*T);
                            ddddpd(2)=0;
                        }else if (_switch_traj == 1){
                           // Circle traj
                           
                           pd(0)=A*sinf(2*pi*t/T);
                           pd(1)=B*cosf(2*pi*t/T);
                           pd(2)=-1.1f;

                           dpd(0)=2*pi*A*cosf(2*pi*t/T)/T;
                           dpd(1)=-2*pi*B*sinf(2*pi*t/T)/T;
                           dpd(2)=0;

                           ddpd(0)=-4*(pi*pi)*A*sinf(2*pi*t/T)/(T*T);
                           ddpd(1)=-4*(pi*pi)*B*cosf(2*pi*t/T)/(T*T);
                           ddpd(2)=0;

                           dddpd(0)=-8*A*(pi*pi*pi)*cosf(2*pi*t/T)/(T*T*T);
                           dddpd(1)=8*A*(pi*pi*pi)*sinf(2*pi*t/T)/(T*T*T);
                           dddpd(2)=0;

                           ddddpd(0)=16*A*(pi*pi*pi*pi)*sinf(2*pi*t/T)/(T*T*T*T);
                           ddddpd(1)=16*A*(pi*pi*pi*pi)*cosf(2*pi*t/T)/(T*T*T*T);
                           ddddpd(2)=0;
                        }else if (_switch_traj == 2){
                           //Setpoint 

                           pd(0)=0.0f;
                           pd(1)=0.0f;
                           pd(2)=-0.85f;

                           dpd(0)=0;
                           dpd(1)=0;
                           dpd(2)=0;

                           ddpd(0)=0;
                           ddpd(1)=0;
                           ddpd(2)=0;

                           dddpd(0)=0;
                           dddpd(1)=0;
                           dddpd(2)=0;

                           ddddpd(0)=0;
                           ddddpd(1)=0;
                           ddddpd(2)=0;
                        }

                        float d11=0.3;

                        matrix::Matrix<float, 3, 3>  D;
                        D(0,0)=d11;
                        D(0,1)=0;
                        D(0,2)=0;

                        D(1,0)=0;
                        D(1,1)=d11;
                        D(1,2)=0;

                        D(2,0)=0;
                        D(2,1)=0;
                        D(2,2)=0;\







// ----------------------------- Disturbance Observer--------------------------------------------------

                        
                        matrix::Vector<float, 3>  z_df;
                        z_df(0)=  z_df_0;
                        z_df(1)=  z_df_1;
                        z_df(2)=  z_df_2;


                        matrix::Vector<float, 3>  z_dtau;
                        z_dtau(0)=  z_dtau_0;
                        z_dtau(1)=  z_dtau_1;
                        z_dtau(2)=  z_dtau_2;


//----------------------------------------- Controller -----------------------------------

                        //float k_dtau=0.2;


                        matrix::Vector<float, 3> er= p - pd;

                        matrix::Vector<float, 3>  df_hat=z_df+k_df*m*v;

                        matrix::Vector<float, 3>  dtau_hat=z_dtau+k_dtau*J*w;

                        matrix::Vector<float, 3>  Ws;

                       Ws(0)=2.0f*df_hat(0);
                       Ws(1)=2.0f*df_hat(1);
                       Ws(2)=0;

                        matrix::Vector<float, 3>  va;
                        va=v-Ws;


                        // reseting the integrals
                        if (!(_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL2)){
                        _int_er_0.setzero();
                        _int_er_1.setzero();
                        _int_er_2.setzero();
                        _i_df_0.setzero();
                        _i_df_1.setzero();
                        _i_df_2.setzero();
                        _i_dtau_0.setzero();
                        _i_dtau_1.setzero();
                        _i_dtau_2.setzero();
                        _i_u.setconstant(15.7f);
                         }

//-------------------------------Intergal backstepping----------------------------------------

                        matrix::Vector<float, 3> delta1;//=0;

                        delta1(0)=_int_er_0.update(er(0));
                        delta1(1)=_int_er_1.update(er(1));
                        delta1(2)=_int_er_2.update(er(2));

                        matrix::Vector<float, 3> delta1_dot=p-pd;


//                        matrix::Vector<float, 3> delta1_dot;
//                        delta1(0)=0;
//                        delta1(1)=0;
//                        delta1(2)=0;

//                        delta1_dot(0)=0;
//                        delta1_dot(1)=0;
//                        delta1_dot(2)=0;



                        matrix::Vector<float, 3> drag=-R*D*R.T()*va;

                        matrix::Vector<float, 3> alpha1=-k1*delta1+pd;

                        matrix::Vector<float, 3> delta2=p-alpha1;

                        matrix::Vector<float, 3> delta2_dot=v+k1*(p-pd)-dpd;

                        matrix::Vector<float, 3> alpha2=-delta1-k1*(p-pd)+dpd-k2*delta2;

                        matrix::Vector<float, 3> delta3=m*v-m*alpha2;

                        matrix::Vector<float, 3> alpha3=delta2/m+k3*delta3+m*g*n3-d11*va+df_hat+m*delta1_dot+m*k1*(v-dpd)-m*ddpd+m*k2*delta2_dot;

                        matrix::Vector<float, 3>  delta4=alpha3-us*R*n3;

                        matrix::Vector<float, 3> beta1=delta2_dot/m+k3*(m*g*n3-u*R*n3-R*D*R.T()*va+df_hat+m*delta1_dot+m*k1*(v-dpd)-m*ddpd+m*k2*delta2_dot)-d11*(m*g*n3-u*R*n3-R*D*R.T()*va+df_hat)/m;
                        matrix::Vector<float, 3> beta2=m*(v-dpd)+k1*(m*g*n3-u*R*n3-R*D*R.T()*va+df_hat-m*ddpd)-m*dddpd+k2*(m*g*n3-u*R*n3-R*D*R.T()*va+df_hat+m*k1*(v-dpd)-m*ddpd);
                        matrix::Vector<float, 3> beta=beta1+beta2;

                        matrix::Vector<float, 3> gamma2=DOBMU*(delta3+(k_df+k1+k2+k3)*delta4);

                        float k=DOBMU*((1/(m*m)+k3*(k_df+m*k1+k2)+(k1+k2)*k_df)+2+k4*(k1+k2+k3+k_df)+(k1+k2+k3+k_df)*(k1+k2+k3+k_df));

                        matrix::Matrix<float, 1 , 1>  usddot=n3.T()*R.T()*(beta+delta3+k4*delta4+gamma2);
                        float usd_dot=usddot(0,0);

                        matrix::Matrix<float, 1 , 1>  udot=usddot+d11*n3.T()*R.T()*(m*g*n3-u*R*n3-R*D*R.T()*va+df_hat)/m+d11*va.T()*R*Sw*n3;

                        u_dot=udot(0,0);
                        u = _i_u.update(u_dot);

                        matrix::Matrix<float, 1 , 1>  vaTRn3=va.T()*R*n3;
                        us=u-d11*vaTRn3(0,0);

                        matrix::Vector<float, 3> alpha4=R* Ie3*R.T()*(beta+delta3+k4*delta4+gamma2);

                        matrix::Vector<float, 3> delta5=alpha4-us*R*Sw*n3;

                        matrix::Vector<float, 3> mu =gamma2+k*R*Ie3*R.T()*delta5;


                        matrix::Vector<float, 3> beta_dot_hat1=(m*g*n3-u*R*n3-R*D*R.T()*va+df_hat+m*k1*(v-dpd)-m*ddpd)/(m*m)
                                                               +k3*(-u_dot*R*n3-u*R*Sw*n3+mu-R*Sw*D*R.T()*va-R*D*Sw.T()*R.T()*va-R*D*R.T()*(m*g*n3-u*R*n3-R*D*R.T()*va+df_hat)
                                                                   + m*(v-dpd)+k1*(m*g*n3-us*R*n3-d11*va+df_hat-m*ddpd)-m*dddpd+k2*(m*g*n3-us*R*n3-d11*va+df_hat+m*k1*(v-dpd)-m*ddpd));

                        matrix::Vector<float, 3> beta_dot_hat2=-d11*(-u_dot*R*n3-u*R*Sw*n3+mu-R*Sw*D*R.T()*va-R*D*Sw.T()*R.T()*va-R*D*R.T()*(m*g*n3-u*R*n3-R*D*R.T()*va+df_hat))/m
                                                               +(m*g*n3-us*R*n3-d11*va+df_hat-m*ddpd)+k1*(-u_dot*R*n3-u*R*Sw*n3+mu-R*Sw*D*R.T()*va-R*D*Sw.T()*R.T()*va-R*D*R.T()*(m*g*n3-u*R*n3-R*D*R.T()*va+df_hat)+mu-m*dddpd)-m*ddddpd+
                                                                k2*(-u_dot*R*n3-u*R*Sw*n3+mu-R*Sw*D*R.T()*va-R*D*Sw.T()*R.T()*va-R*D*R.T()*(m*g*n3-u*R*n3-R*D*R.T()*va+df_hat)+mu+k1*(m*g*n3-us*R*n3-d11*va+df_hat-m*ddpd)-m*dddpd);

                        matrix::Vector<float, 3>  beta_dot_hat=beta_dot_hat1+beta_dot_hat2;

                        matrix::Vector<float, 3>  delta3_dot_hat=m*g*n3-us*R*n3-d11*va+df_hat+m*delta1_dot+m*k1*(v-dpd)-m*ddpd+m*k2*delta2_dot;

                        matrix::Vector<float, 3>  delta4_dot_hat=mu+beta-usd_dot*R*n3-us*R*Sw*n3;

                        matrix::Vector<float, 3>  gamma2_hat_dot=DOBMU*(delta3_dot_hat+(k1+k2+k3+k_df)*delta4_dot_hat);

                        matrix::Vector<float, 3>  alpha4_dot_hat=R*Sw*Ie3*R.T()*(beta+delta3+k4*delta4+gamma2)+
                                                                 R*Ie3*Sw.T()*R.T()*(beta+delta3+k4*delta4+gamma2)+
                                                                 R*Ie3*R.T()*(beta_dot_hat+delta3_dot_hat+k4*delta4_dot_hat+gamma2_hat_dot);

                        matrix::Vector<float, 3> nu=k*R*Ie3*R.T()*delta4;

                      //  matrix::Vector<float, 3> wddot=alpha4_dot_hat - u_dot*R*Sw*n3 + delta4 +k5*delta5-u*R*Sw*Sw*n3;

                        matrix::Matrix<float, 1 , 1>  wddot0;
                        wddot0= -(n2.T()*R.T())*(alpha4_dot_hat - usd_dot*R*Sw*n3 + delta4 +k5*delta5-us*R*Sw*Sw*n3+nu)/us;

                        matrix::Matrix<float, 1 , 1>  wddot1;
                        wddot1= (n1.T()*R.T())*(alpha4_dot_hat - usd_dot*R*Sw*n3 + delta4 +k5*delta5-us*R*Sw*Sw*n3+nu)/us;


                        //matrix::Matrix<float, 1 , 1> V=delta1.T()*delta1+delta2.T()*delta2+delta3.T()*delta3+delta4.T()*delta4+delta5.T()*delta5;

//-------------------------------Backstepping----------------------------------------

//                          matrix::Vector<float, 3> delta_1 = p - pd;

//                          matrix::Vector<float, 3> alpha_1= dpd-k1_bs * delta_1;

//                          matrix::Vector<float, 3> delta_2 = m * v - m* alpha_1;

//                          matrix::Vector<float, 3> alpha_2= m*g*n3 +df_hat-m*ddpd+m*k1_bs*v-m*k1_bs*dpd + delta_1/m + k2_bs * delta_2;

//                          matrix::Vector<float, 3> delta_3 = alpha_2 - u*R*n3;

//                          matrix::Vector<float, 3> beta1=-m*dddpd+k1_bs*m*g*n3-k1_bs*u*R*n3+k1_bs*df_hat-m*k1_bs*ddpd+v/m-dpd/m;

//                         matrix::Vector<float, 3> beta2=k2_bs*m*g*n3-k2_bs*u*R*n3+k2_bs*df_hat-k2_bs*m*ddpd+k2_bs*m*k1_bs*v-k2_bs*m*k1_bs*dpd;

//                         matrix::Vector<float, 3> beta=beta1+beta2;
//                         //---
//                         //float DOB=0.5;// turning on/off mu term

//                         matrix::Vector<float, 3> gamma_2=DOBMU*(delta_2+(k_df+k1_bs+k2_bs)*delta_3);
//                         //---

//                         matrix::Vector<float, 3> s=beta+delta_2+k3_bs*delta_3+gamma_2;

//                         matrix::Matrix<float, 3, 3> RIRT = R*Ie3*R.T();// hard coding this termS

//                         matrix::Vector<float, 3> uRSwn3 =  u*R*Sw*n3;// hard code ....

//                         matrix::Vector<float, 3> delta_4 = RIRT*s - uRSwn3;

//                         //-------
//                         float k=DOBMU*((k1_bs*k_df+1/(m*m)+k2_bs*k_df+m*k1_bs*k2_bs)+1+k3_bs*(k_df+k1_bs+k2_bs)+1+(k_df+k1_bs+k2_bs)*(k_df+k1_bs+k2_bs));
//                         matrix::Vector<float, 3> mu =gamma_2+k*RIRT*delta_4;
//                         //------

//                         matrix::Matrix<float, 1 , 1>  udot= n3.T()*R.T()*s;

//                         u_dot=udot(0,0);
//                         u = _i_u.update(u_dot);

////                         matrix::Vector<float, 3> beta_dot1=-m*ddddpd-k1_bs*u_dot*R*n3-k1_bs*uRSwn3-m*k1_bs*dddpd+(m*g*n3-u*R*n3+df_hat)/(m*m)-ddpd/m-k2_bs*u_dot*R*n3;

////                         matrix::Vector<float, 3> beta_dot2=-k2_bs*uRSwn3-k2_bs*m*dddpd+k2_bs*k1_bs*(m*g*n3-u*R*n3+df_hat)-k1_bs*k2_bs*m*ddpd;


//                         matrix::Vector<float, 3> beta_dot1=-m*ddddpd-k1_bs*u_dot*R*n3-k1_bs*uRSwn3+k1_bs*mu-m*k1_bs*dddpd+(m*g*n3-u*R*n3+df_hat)/(m*m)-ddpd/m-k2_bs*u_dot*R*n3;

//                         matrix::Vector<float, 3> beta_dot2=-k2_bs*uRSwn3+k2_bs*mu-k2_bs*m*dddpd+k2_bs*k1_bs*(m*g*n3-u*R*n3+df_hat)-k1_bs*k2_bs*m*ddpd;



////                         matrix::Vector<float, 3> delta_2_3_dot=m*g*n3-u*R*n3+df_hat-m*ddpd+m*k1_bs*v-m*k1_bs*dpd+k3_bs*(beta-u_dot*R*n3-uRSwn3);

////                         matrix::Vector<float, 3> alpha_3_dot=R*Sw*Ie3*R.T()*s+R*Ie3*Sw.T()*(R.T())*s+RIRT*(beta_dot1+beta_dot2+delta_2_3_dot);


//                         matrix::Vector<float, 3> delta_2_3_dot=m*g*n3-u*R*n3+df_hat-m*ddpd+m*k1_bs*v-m*k1_bs*dpd+k3_bs*(mu+beta-u_dot*R*n3-uRSwn3);//delta2_dot+k3*delta_3_dot

//                         matrix::Vector<float, 3> gamma_2_hat_dot=DOBMU*(m*g*n3-u*R*n3+df_hat-m*ddpd+m*k1_bs*v-m*k1_bs*dpd+(k_df+k1_bs+k2_bs)*(mu+beta-u_dot*R*n3-uRSwn3));

//                         matrix::Vector<float, 3> alpha_3_dot=R*Sw*Ie3*R.T()*s+R*Ie3*Sw.T()*(R.T())*s+RIRT*(beta_dot1+beta_dot2+delta_2_3_dot+gamma_2_hat_dot);

//                         matrix::Vector<float, 3> nu=k*RIRT*delta_3;

//                       //  matrix::Vector<float, 3> wddot=alpha_3_dot-u_dot*R*Sw*n3-u*R*Sw*Sw*n3+delta_3+k4_bs*(delta_4);

//                         matrix::Matrix<float, 1 , 1>  wddot0;
//                         wddot0= -(n2.T()*R.T())*(alpha_3_dot-u_dot*R*Sw*n3-u*R*Sw*Sw*n3+delta_3+k4_bs*(delta_4)+nu)/u;

//                         matrix::Matrix<float, 1 , 1>  wddot1;
//                         wddot1= (n1.T()*R.T())*(alpha_3_dot-u_dot*R*Sw*n3-u*R*Sw*Sw*n3+delta_3+k4_bs*(delta_4)+nu)/u;

//-------------------------DOB integral action--------------------------------------------------------

                        matrix::Vector<float, 3>  h=-k_df*(df_hat)-k_df*(m*g*n3-u*R*n3-R*D*R.T()*va);//+mu;

                                d_zdf_0 = h(0);
                        z_df_0 = _i_df_0.update(d_zdf_0);

                                d_zdf_1 = h(1);
                        z_df_1 = _i_df_1.update(d_zdf_1);

                                d_zdf_2 = h(2);
                        z_df_2 = _i_df_2.update(d_zdf_2);

//-----------------------Yaw control----------------------------------------------------------------------
                       // float k_psi_1=1.5;
                        //float k_psi_2=2.3;
                        float psi_d=0;
                        float psi_d_dot=0;
                        float psi_d_ddot=0;
                        float ep_1=psi-psi_d;
                        float alpha_psi_1=psi_d_dot-k_psi_1*ep_1;

                        float ep_2=w(2)-alpha_psi_1;
                        float cos_theta= cos(theta);
                        float sin_theta= sin(theta);
                        float sin_phi=sin(phi);
                        float cos_phi=cos(phi);
                        float phi_dot=w(0);
                        float theta_dot=w(1);

                        float n3Wdot=(cos_phi*phi_dot/cos_theta+sin_phi/(cos_theta*cos_theta)*sin_theta*theta_dot)*w(1)+(-sin_phi*phi_dot/cos_theta+cos_phi/(cos_theta*cos_theta)*sin_theta*theta_dot)*w(2);
                        float alpha_psi_dot=psi_d_ddot-k_psi_1*(w(2)-psi_d_dot);
                        float wddot2;

                        wddot2= cos_theta*(alpha_psi_dot-ep_1-k_psi_2*ep_2-n3Wdot-sin_phi* wddot1(0,0)/cos_theta)/cos_phi;


                        matrix::Vector<float, 3> wddot;
                        wddot(0)=wddot0(0,0);
                        wddot(1)=wddot1(0,0);
                        wddot(2)=wddot2;

                        matrix::Vector<float, 3>  wCrossJw;
                        wCrossJw(0)= J(2,2)*w(1)*w(2)-J(1,1)*w(1)*w(2);
                        wCrossJw(1)= J(0,0)*w(0)*w(2)-J(2,2)*w(0)*w(2);
                        wCrossJw(2)= J(1,1)*w(0)*w(1)-J(0,0)*w(0)*w(1);

                        matrix::Vector<float, 3> tau=J*wddot+wCrossJw-dtau_hat;

                        //------------------Torque dis intergation-------------------

                        matrix::Vector<float, 3>  h_tau=-k_dtau*(dtau_hat)-k_dtau*(-wCrossJw+tau);

                                d_zdtau_0 = h_tau(0);
                        z_dtau_0 = _i_dtau_0.update(d_zdtau_0);

                                d_zdtau_1 = h_tau(1);
                        z_dtau_1 = _i_dtau_1.update(d_zdtau_1);

                                d_zdtau_2 = h_tau(2);
                        z_dtau_2 = _i_dtau_2.update(d_zdtau_2);

                        quad_input[0] =J1_nf*tau(0);
                        quad_input[1] =J2_nf*tau(1);
                        quad_input[2] =J3_nf*tau(2);
                        quad_input[3] = u;

                         // normalization and saturation of thrust and torques
                        float nor_thrust=(quad_input[3]-15.7f)/60.0f+0.51f;

                        float sat_thrust;

                        if (nor_thrust > 0.85f) {
                        sat_thrust = 0.85f;

                        } else if (nor_thrust < 0.25f) {
                                sat_thrust = 0.25f;
                        }
                        else{
                            sat_thrust=nor_thrust;
                        }

                        if (quad_input[0] > 0.21f) {
                        quad_input[0] = 0.21f;

                        } else if (quad_input[0] < -0.21f) {
                                quad_input[0] = -0.21f;
                        }

                        if (quad_input[1] > 0.14f) {
                        quad_input[1] = 0.14f;

                        } else if (quad_input[1] < -0.14f) {
                                quad_input[1] = -0.14f;
                        }

                        if (quad_input[2] > 0.15f) {
                        quad_input[2] = 0.15f;

                        } else if (quad_input[2] < -0.15f) {
                                quad_input[2] = -0.15f;
                        }

                        // publishing the controls to the secondary control setpoint
                        _control_sp.get().control[0] = quad_input[0];
                        _control_sp.get().control[1] = quad_input[1];
                        _control_sp.get().control[2] = quad_input[2];
                        _control_sp.get().control[3] = sat_thrust;




                    // publishing data for logging
                        _att_sp.get().roll   = phi;
                        _att_sp.get().pitch  = theta;
                        _att_sp.get().yaw    = psi;
                        _att_sp.get().thrust = u;
                        _att_sp.get().valid  = true;

                        _att_sp.get().er[0] =er(0);
                        _att_sp.get().er[1] =er(1);
                        _att_sp.get().er[2] =er(2);
                        _att_sp.get().er[3] =ep_1;

                        _att_sp.get().pd[0] =pd(0);
                        _att_sp.get().pd[1] =pd(1);
                        _att_sp.get().pd[2] =pd(2);

                        _att_sp.get().p[0] =p(0);
                        _att_sp.get().p[1] =p(1);
                        _att_sp.get().p[2] =p(2);

                        _att_sp.get().df[0] =df_hat(0);
                        _att_sp.get().df[1] =df_hat(1);
                        _att_sp.get().df[2] =df_hat(2);

                        _att_sp.get().dtau[0] =dtau_hat(0);
                        _att_sp.get().dtau[1] =dtau_hat(1);
                        _att_sp.get().dtau[2] =dtau_hat(2);

                        _att_sp.get().drag[0] =drag(0);
                        _att_sp.get().drag[1] =drag(1);
                        _att_sp.get().drag[2] =drag(2);


                        _att_sp.get().st =switch_time;

        }
}
        //update all publications
        updatePublications();

}
