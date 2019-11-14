function [x_trim,u_trim] = compute_trim(filename, Va, gamma, R)
% Va is the desired airspeed (m/s)
% gamma is the desired flight path angle (radians)航迹角，水平面与地速矢量Vg的夹角
% R is the desired radius (m) - use (+) for right handed orbit, 
%                                   (-) for left handed orbit


% add stuff here已加
x0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
u0 = [0; 0; 0; 1];
y0 = [Va; gamma; 0];
ix = [];
iu = [];
iy = [1, 3];
dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va/R; 0; 0; 0];
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

% compute trim conditions计算修正条件
[x_trim,u_trim,y_trim,dx_trim] = trim(filename,x0,u0,y0,ix,iu,iy,dx0,idx);

% check to make sure that the linearization worked (should be small)
% 检查以确保线性化工作，即模足够小
norm(dx_trim(3:end)-dx0(3:end))

