% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position,
%   pehat    - estimated East position,
%   hhat     - estimated altitude,
%   Vahat    - estimated airspeed,
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle,
%   thetahat - estimated pitch angel,
%   chihat   - estimated course,
%   phat     - estimated roll rate,
%   qhat     - estimated pitch rate,
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed,
%   wnhat    - estimate of North wind,
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
%
%
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

% rename inputs
y_gyro_x      = uu(1);
y_gyro_y      = uu(2);
y_gyro_z      = uu(3);
y_accel_x     = uu(4);
y_accel_y     = uu(5);
y_accel_z     = uu(6);
y_static_pres = uu(7);
y_diff_pres   = uu(8);
y_gps_n       = uu(9);
y_gps_e       = uu(10);
y_gps_h       = uu(11);
y_gps_Vg      = uu(12);
y_gps_course  = uu(13);
t             = uu(14);


% not estimating these states
alphahat = 0;
betahat  = 0;
bxhat    = 0;
byhat    = 0;
bzhat    = 0;

% estimated states
if t==0
    phat=LPF(y_gyro_x,1800,1,P);
    qhat=LPF(y_gyro_y,1000,1,P);
    rhat=LPF(y_gyro_z,1400,1,P);
    hhat=LPF(y_static_pres,800,1,P)/P.rho/P.gravity;
    Vahat=sqrt(2*LPF(y_diff_pres,500,1,P)/P.rho);
else
    phat=LPF(y_gyro_x,1800,0,P);
    qhat=LPF(y_gyro_y,1000,0,P);
    rhat=LPF(y_gyro_z,1400,0,P);
    hhat=LPF(y_static_pres,800,0,P)/P.rho/P.gravity;
    Vahat=sqrt(2*LPF(y_diff_pres,500,0,P)/P.rho);
    
end

pnhat=0;
pehat=0;

phihat=0;
thetahat=0;
chihat=0;
Vghat=0;
wnhat=0;
wehat=0;
psihat=0;

xhat = [...
    pnhat;...
    pehat;...
    hhat;...
    Vahat;...
    alphahat;...
    betahat;...
    phihat;...
    thetahat;...
    chihat;...
    phat;...
    qhat;...
    rhat;...
    Vghat;...
    wnhat;...
    wehat;...
    psihat;...
    bxhat;...
    byhat;...
    bzhat;...
    ];
end

function y_hat=LPF(u,a,flag,P)
% 低通滤波器
% u 输入量, a 截止频率, flag 指示是否初始化
% y_hat 过滤量（输出量）

persistent y
if flag==1
    y=0;
end

alpha_LPF=exp(-a*P.Ts);
y=alpha_LPF*y+(1-alpha_LPF)*u;

y_hat=y;

end
