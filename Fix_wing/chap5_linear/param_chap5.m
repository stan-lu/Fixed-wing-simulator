close all;clc;clear;
P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% UAV无人机参数
%机体物理参数
P.mass = 13.5;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = 0.1204;
%空气动力系数
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

% 风参数
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 200;%空间波长，非稳定干扰气流德莱登传递函数需要参数
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; %沿机体坐标轴干扰强度
P.sigma_v = 1.06;
P.sigma_w = 0.7;

% r1 - r8
P.r = P.Jx*P.Jz-P.Jxz^2;
P.r1 = P.Jxz*(P.Jx-P.Jy+P.Jz)/P.r;
P.r2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/P.r;
P.r3 = P.Jz/P.r;
P.r4 = P.Jxz/P.r;
P.r5 = (P.Jz-P.Jx)/P.Jy;
P.r6 = P.Jxz/P.Jy;
P.r7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/P.r;
P.r8 = P.Jx/P.r;

% C parameters on p.62
P.C_p_0 = P.r3 * P.C_ell_0 + P.r4 * P.C_n_0;
P.C_p_beta = P.r3 * P.C_ell_beta + P.r4 * P.C_n_beta;
P.C_p_p = P.r3 * P.C_ell_p + P.r4 * P.C_n_p;
P.C_p_r = P.r3 * P.C_ell_r + P.r4 * P.C_n_r;
P.C_p_delta_a = P.r3 * P.C_ell_delta_a + P.r4 * P.C_n_delta_a;
P.C_p_delta_r = P.r3 * P.C_ell_delta_r + P.r4 * P.C_n_delta_r;
P.C_r_0 = P.r4 * P.C_ell_0 + P.r8 * P.C_n_0;
P.C_r_beta = P.r4 * P.C_ell_beta + P.r8 * P.C_n_beta;
P.C_r_p = P.r4 * P.C_ell_p + P.r8 * P.C_n_p;
P.C_r_r = P.r4 * P.C_ell_r + P.r8 * P.C_n_r;
P.C_r_delta_a = P.r4 * P.C_ell_delta_a + P.r8 * P.C_n_delta_a;
P.C_r_delta_r = P.r4 * P.C_ell_delta_r + P.r8 * P.C_n_delta_r;

%计算修正条件，初始空速
P.Va0 = 35;
gamma = 0*pi/180;  % 期望飞行轨迹角
R     = 200;         % 期望转弯半径

% 自动驾驶仪的采样速率
P.Ts = 0.01;
% P.tau = 0.5;

% 初始条件下的首次采集
P.pn0    = 0;  % 初始北向位置
P.pe0    = 0;  
P.pd0    = 0;  
P.u0     = P.Va0; 
P.v0     = 0;  
P.w0     = 0;  
P.phi0   = 0;  
P.theta0 = 0;  
P.psi0   = 0;  
P.p0     = 0;  
P.q0     = 0; 
P.r0     = 0; 


% 运行修正控制
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;


% 为修正条件设置初始条件
% 初始条件
P.pn0    = 0;  
P.pe0    = 0;  
P.pd0    = 0;  
P.u0     = x_trim(4);  
P.v0     = x_trim(5); 
P.w0     = x_trim(6);  
P.phi0   = x_trim(7);  
P.theta0 = x_trim(8);  
P.psi0   = x_trim(9); 
P.p0     = x_trim(10);  
P.q0     = x_trim(11); 
P.r0     = x_trim(12); 

% 计算不同的传递函数
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

% 修正条件下运动方程线性化
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);
eig_lon = eig(A_lon);%纵向状态空间模型特征值
eig_lat = eig(A_lat);%横向状态空间模型特征值
