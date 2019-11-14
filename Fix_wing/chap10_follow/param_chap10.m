close all;clc;clear;
% 使用这些修正值用于无人机
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
P.wind_n = 3;%3;
P.wind_e = -3;%2;
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

% C 第五章参数
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
R     = inf;         % 期望转弯半径

% 自动驾驶仪的采样速率
P.Ts = 0.01;
% P.tau = 0.5;

% 初始条件下的首次采集
P.pn0    = 1000;  % 初始北向位置
P.pe0    = 1000;  
P.pd0    = -100;  
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

% 高度控制状态机参数
P.altitude_take_off_zone = 50;
P.altitude_hold_zone = 10;



% 所需变量参数
u = P.x_trim(4);
v = P.x_trim(5);
w = P.x_trim(6);
Va_trim = sqrt(u^2 + v^2 + w^2);
theta_trim = P.x_trim(8);
alpha_trim = atan(w/u);
delta_e_trim = P.u_trim(1);
delta_t_trim = P.u_trim(4);
Va = P.Va0;

% 滚动姿态保持
P.delta_a_max = 60 * pi / 180;
P.phi_max = 15 * pi / 180;
P.roll_max = 55 * pi / 180;
a_phi1 = -0.5 * P.rho * Va_trim^2 * P.S_wing * P.b * P.C_p_p * P.b / (2 * Va_trim);
a_phi2 = 0.5 * P.rho * Va_trim^2 * P.S_wing * P.b * P.C_p_delta_a;
P.kp_phi = P.delta_a_max / P.phi_max * sign(a_phi2);
omega_phi = sqrt(abs(a_phi2) * P.delta_a_max / P.phi_max);
zeta_phi = 1.2;   % 设计的参数
P.kd_phi = (2 * zeta_phi * omega_phi - a_phi1) / a_phi2;
P.ki_phi = 0.2;    % 设计的参数

% 航向保持
W_chi = 8;  % 设计的参数
omega_chi = 1 / W_chi * omega_phi;
zeta_chi = 0.707;   % 设计的参数
Vg = P.Va0;
P.kp_chi = 2 * zeta_chi * omega_chi * Vg / P.gravity;
P.ki_chi = omega_chi^2 * Vg / P.gravity;

% 侧滑保持
P.delta_r_max= 45 *pi/180;
P.e_beta_max= 20 *pi/180;
a_beta1=-P.rho*Va*P.S_wing*P.C_Y_beta/2/P.mass;
a_beta2=P.rho*Va*P.S_wing*P.C_Y_delta_r/2/P.mass;
P.kp_beta=P.delta_r_max/P.e_beta_max*sign(a_beta2);
zeta_beta=0.707; % 设计参数
P.ki_beta=(a_beta1+a_beta2*P.kp_beta)/a_beta2/2/zeta_beta;

% 俯仰姿态保持
P.delta_e_max = 45 * pi / 180;
P.e_theta_max = 20 * pi / 180;
a_theta1 = -P.rho * Va_trim^2 * P.c * P.S_wing * P.C_m_q * P.c / (2 * P.Jy * 2 * Va_trim);
a_theta2 = -P.rho * Va_trim^2 * P.c * P.S_wing * P.C_m_alpha / (2 * P.Jy);
a_theta3 = P.rho * Va_trim^2 * P.c * P.S_wing * P.C_m_delta_e / (2 * P.Jy);
P.kp_theta = P.delta_e_max / P.e_theta_max * sign(a_theta3);
omega_theta = sqrt(a_theta2 + P.delta_e_max / P.e_theta_max * abs(a_theta3));
zeta_theta = 0.707;     % 设计的参数
P.kd_theta = (2 * zeta_theta * omega_theta - a_theta1) / a_theta3;
P.theta_max = 20 * pi / 180;
P.pitch_max = 40 * pi / 180;
P.ki_theta = 0;

% 利用油门的空速控制
a_V1 = P.rho * Va_trim *P.S_wing/ P.mass ...
       * (P.C_D_0 + P.C_D_alpha * alpha_trim + P.C_D_delta_e * delta_e_trim) ...
       + P.rho * P.S_prop / P.mass * P.C_prop * Va_trim;
a_V2 = P.rho * P.S_prop / P.mass * P.C_prop * P.k_motor^2 * delta_t_trim;
a_V3 = P.gravity * cos(theta_trim - alpha_trim);
omega_v = 5;   % 设计的参数
zeta_v = 0.707;     % 设计的参数
P.delta_t_max = 1;
P.delta_t_min = 0;
P.ki_v = omega_v^2 / a_V2;
P.kp_v = (2 * zeta_v * omega_v - a_V1) / a_V2;

% 利用俯仰指令的空速控制
W_v2 = 7;   % 设计的参数
zeta_v2 = 0.707;    % 设计的参数
omega_v2 = 1 / W_v2 * omega_theta;
K_theta_dc = P.kp_theta * a_theta3 / (a_theta2 + P.kp_theta * a_theta3);
P.ki_v2 = -omega_v2^2/(K_theta_dc * P.gravity);
P.kp_v2 = (a_V1 - 2 * zeta_v2 * omega_v2) / (K_theta_dc * P.gravity);

% 利用俯仰指令的高度控制
W_h = 15;   % 设计的参数
omega_h = 1 / W_h * omega_theta;
Va = P.Va0;
zeta_h = 1.2;     % 设计的参数
P.h_max = 1000;
P.h_min = 0;
P.ki_h = omega_h^2 / (K_theta_dc * Va);
P.kp_h = 2 * zeta_h * omega_h / (K_theta_dc * Va);

% 传感器
P.Ts_gps=1;
P.bias_gyro_x=0;
P.bias_gyro_y=0;
P.bias_gyro_z=0;

% 运动制导的自动驾驶仪系数
P.b_chi=0.2;
P.b_chidot=0.55;
P.b_h=0.6;
P.b_hdot=1.2;
P.b_Va=1;
P.gamma_max=pi/3; % 最大爬升角

% 路径制导的参数
P.k_path=0.027; 
P.chi_inf=pi/2; % (0,pi/2]
P.k_orbit=3.8; 