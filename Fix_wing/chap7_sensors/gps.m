% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB

function y = gps(uu, P)

    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
    % 计算高斯马尔科夫过程误差
    persistent v_n
    persistent v_e
    persistent v_h
    persistent eta_GPS
    
    sigma_GPS=[2.1;2.1;4];
    k_GPS=[1/16000;1/16000;1/16000];
    T_s=[1;1;1];
    
    if t==0
        v_n=0;
        v_e=0;
        v_h=0;
        eta_GPS=[0;0;0];
    else
        v_n=exp(-k_GPS(1)*T_s(1))*v_n+eta_GPS(1);
        v_e=exp(-k_GPS(2)*T_s(2))*v_e+eta_GPS(2);
        v_h=exp(-k_GPS(3)*T_s(3))*v_h+eta_GPS(3);
        eta_GPS=[normrnd(0,sigma_GPS(1)^2);normrnd(0,sigma_GPS(2)^2);
            normrnd(0,sigma_GPS(3)^2)];
    end
    
    
    % construct North, East, and altitude GPS measurements
    y_gps_n = pn+v_n;
    y_gps_e = pe+v_e; 
    y_gps_h = -pd+v_h; 
    
    % construct groundspeed and course measurements
    V_g=sqrt((Va*cos(psi)+wn)^2+(Va*sin(psi)+we)^2);
    sigma_V=0.03;
    sigma_V_g=sigma_V;
    sigma_chi=sigma_V/V_g;
    y_gps_Vg     = V_g+normrnd(0,sigma_V_g^2);
    if sigma_chi~=inf
        y_gps_course = atan2(Va*sin(psi)+we,Va*cos(psi)+wn)+normrnd(0,sigma_chi^2);
    else
        y_gps_course = atan2(Va*sin(psi)+we,Va*cos(psi)+wn);
    end

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



