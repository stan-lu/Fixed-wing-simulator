% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%
%  Revised:
%   3/5/2010  - RB 
%   5/14/2010 - RB

function y = sensors(uu, P)

    % relabel the inputs
%    pn      = uu(1);
%    pe      = uu(2);
    pd      = uu(3);
%    u       = uu(4);
%    v       = uu(5);
%    w       = uu(6);
    phi     = uu(7);
    theta   = uu(8);
%    psi     = uu(9);
    p       = uu(10);
    q       = uu(11);
    r       = uu(12);
    F_x     = uu(13);
    F_y     = uu(14);
    F_z     = uu(15);
%    M_l     = uu(16);
%    M_m     = uu(17);
%    M_n     = uu(18);
    Va      = uu(19);
%    alpha   = uu(20);
%    beta    = uu(21);
%    wn      = uu(22);
%    we      = uu(23);
%    wd      = uu(24);
    
    % simulate rate gyros (units are rad/sec)
    sigma_gyro=0.13*pi/180;
    y_gyro_x = p+normrnd(0,sigma_gyro^2);
    y_gyro_y = q+normrnd(0,sigma_gyro^2);
    y_gyro_z = r+normrnd(0,sigma_gyro^2);

    % simulate accelerometers (units of g)
    sigma_accel=0.0025*P.gravity;
    y_accel_x = F_x/P.mass + P.gravity*sin(theta) + normrnd(0,sigma_accel^2);
    y_accel_y = F_y/P.mass - P.gravity*cos(theta)*sin(phi)+normrnd(0,sigma_accel^2);
    y_accel_z = F_z/P.mass - P.gravity*cos(theta)*cos(phi)+normrnd(0,sigma_accel^2);

    % simulate pressure sensors
    beta_abspress=0.125*1000; % (units of Pa)
    sigma_abspress=0.01*1000; 
    beta_diffpress=0.02*1000;
    sigma_diffpress=0.02*1000;
    
    y_static_pres = -P.rho*P.gravity*pd+beta_abspress+normrnd(0,sigma_abspress^2);
    y_diff_pres = 0.5*P.rho*Va^2+beta_diffpress+normrnd(0,sigma_diffpress^2);

    % construct total output
    y = [...
        y_gyro_x;...
        y_gyro_y;...
        y_gyro_z;...
        y_accel_x;...
        y_accel_y;...
        y_accel_z;...
        y_static_pres;...
        y_diff_pres;...
    ];

end



