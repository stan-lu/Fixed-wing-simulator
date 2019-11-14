function plotMAVStateVariables(uu)
%
% modified 12/11/2009 - RB

% process inputs to function
pn          = uu(1);             % North position (meters)
pe          = uu(2);             % East position (meters)
if abs(pe)<1e-6
    pe = 0;
end
h           = -uu(3);            % altitude (meters)
if abs(h)<1e-6
    h = 0;
end
u           = uu(4);             % body velocity along x-axis (meters/s)
v           = uu(5);             % body velocity along y-axis (meters/s)
w           = uu(6);             % body velocity along z-axis (meters/s)
phi         = 180/pi*uu(7);      % roll angle (degrees)
theta       = 180/pi*uu(8);      % pitch angle (degrees)
psi         = uu(9);             % yaw angle (degrees)
p           = 180/pi*uu(10);     % body angular rate along x-axis (degrees/s)
q           = 180/pi*uu(11);     % body angular rate along y-axis (degrees/s)
r           = 180/pi*uu(12);     % body angular rate along z-axis (degrees/s)
Va          = uu(13);            % airspeed (m/s)
alpha       = 180/pi*uu(14);     % angle of attack (degrees)攻角
beta        = 180/pi*uu(15);     % side slip angle (degrees)侧滑角
wn          = uu(16);            % wind in the North direction
we          = uu(17);            % wind in the East direction
wd          = uu(18);            % wind in the Down direction
pn_c        = uu(19);            % commanded North position (meters)控制北向位置
pe_c        = uu(20);            % commanded East position (meters)
h_c         = uu(21);            % commanded altitude (meters)
Va_c        = uu(22);            % commanded airspeed (meters/s)
alpha_c     = 180/pi*uu(23);     % commanded angle of attack (degrees)
beta_c      = 180/pi*uu(24);     % commanded side slip angle (degrees)
phi_c       = 180/pi*uu(25);     % commanded roll angle (degrees)
theta_c     = 180/pi*uu(26);     % commanded pitch angle (degrees)
chi_c       = 180/pi*uu(27);     % commanded course (degrees)
p_c         = 180/pi*uu(28);     % commanded body angular rate along x-axis (degrees/s)
q_c         = 180/pi*uu(29);     % commanded body angular rate along y-axis (degrees/s)
r_c         = 180/pi*uu(30);     % commanded body angular rate along z-axis (degrees/s)
pn_hat      = uu(31);            % estimated North position (meters)估计北向位置
pe_hat      = uu(32);            % estimated East position (meters)
h_hat       = uu(33);            % estimated altitude (meters)
Va_hat      = uu(34);            % estimated airspeed (meters/s)
alpha_hat   = 180/pi*uu(35);     % estimated angle of attack (degrees)
beta_hat    = 180/pi*uu(36);     % estimated side slip angle (degrees)
phi_hat     = 180/pi*uu(37);     % estimated roll angle (degrees)
theta_hat   = 180/pi*uu(38);     % estimated pitch angle (degrees)
chi_hat     = 180/pi*uu(39);     % estimated course (degrees)
p_hat       = 180/pi*uu(40);     % estimated body angular rate along x-axis (degrees/s)
q_hat       = 180/pi*uu(41);     % estimated body angular rate along y-axis (degrees/s)
r_hat       = 180/pi*uu(42);     % estimated body angular rate along z-axis (degrees/s)
%    Vg_hat      = uu(43);            % estimated groundspeed
%    wn_hat      = uu(44);            % estimated North wind
%    we_hat      = uu(45);            % estimated East wind
%    psi_hat     = 180/pi*uu(46);     % estimated heading估计航向
%    bx_hat      = uu(47);            % estimated x-gyro biasX轴偏压估计
%    by_hat      = uu(48);            % estimated y-gyro bias
%    bz_hat      = uu(49);            % estimated z-gyro bias
delta_e     = 180/pi*uu(50);     % elevator angle (degrees)升降翼变化
delta_a     = 180/pi*uu(51);     % aileron angle (degrees)副翼变化
delta_r     = 180/pi*uu(52);     % rudder angle (degrees)方向翼变化
delta_t     = uu(53);            % throttle setting (unitless)油门
t           = uu(54);            % simulation time模拟时间

Va_c = 35;
% h_c = Va_c * t * sind(30);
% pn_c = 200 * sin(35/200*t);
% pe_c = 200 * (1 - cos(35/200*t));

% compute course angle计算航向角
chi = 180/pi*atan2(Va*sin(psi)+we, Va*cos(psi)+wn);

% define persistent variables 定义持续变量
persistent pn_handle
persistent pe_handle
persistent h_handle
persistent Va_handle
persistent alpha_handle
persistent beta_handle
persistent phi_handle
persistent theta_handle
persistent chi_handle
persistent p_handle
persistent q_handle
persistent r_handle
persistent delta_e_handle
persistent delta_a_handle
persistent delta_r_handle
persistent delta_t_handle


% first time function is called, initialize plot and persistent vars
%调用第一个函数，初始化图和持续变量
if t==0
    if 1 % 1 为画在同一个窗口
         % 0 为画在不同窗口
        figure(2), clf
        subplot(8,2,1)
        hold on
        pn_handle = graph_y(t, pn, 'p_n', []);
        xlabel('t(s)')
        subplot(8,2,2)
        hold on
        Va_handle = graph_y_yd(t, Va, Va_c, 'V_a', []);
        xlabel('t(s)')
        subplot(8,2,3)
        hold on
        pe_handle = graph_y(t, pe, 'p_e', []);
        xlabel('t(s)')
        subplot(8,2,4)
        hold on
        alpha_handle = graph_y(t, alpha, '\alpha', []);
        xlabel('t(s)')
        subplot(8,2,5)
        hold on
        h_handle = graph_y_yd(t, h, h_c, 'h', []);
        xlabel('t(s)')
        subplot(8,2,6)
        hold on
        beta_handle = graph_y_yd(t, beta, beta_c, '\beta', []);
        xlabel('t(s)')
        subplot(8,2,7)
        hold on
        phi_handle = graph_y_yd(t, phi, phi_c, '\phi', []);
        xlabel('t(s)')
        subplot(8,2,8)
        hold on
        p_handle = graph_y_yd(t, p, p_c, 'p', []);
        xlabel('t(s)')
        subplot(8,2,9)
        hold on
        theta_handle = graph_y_yd(t, theta, theta_c, '\theta', []);
        xlabel('t(s)')
        subplot(8,2,10)
        hold on
        q_handle = graph_y_yd(t, q, q_c, 'q', []);
        xlabel('t(s)')
        subplot(8,2,11)
        hold on
        chi_handle = graph_y_yd(t, chi, chi_c, '\chi', []);
        xlabel('t(s)')
        subplot(8,2,12)
        hold on
        r_handle = graph_y_yd(t, r, r_c, 'r', []);
        xlabel('t(s)')
        subplot(8,2,13)
        hold on
        delta_e_handle = graph_y(t, delta_e, '\delta_e', []);
        xlabel('t(s)')
        subplot(8,2,14)
        hold on
        delta_a_handle = graph_y(t, delta_a, '\delta_a', []);
        xlabel('t(s)')
        subplot(8,2,15)
        hold on
        delta_r_handle = graph_y(t, delta_r, '\delta_r', []);
        xlabel('t(s)')
        subplot(8,2,16)
        hold on
        delta_t_handle = graph_y(t, delta_t, '\delta_t', []);
        xlabel('t(s)')
    else
        figure(2), clf
        subplot(3,1,1)
        hold on
        pn_handle = graph_y(t, pn, 'p_n/m', []);
        xlabel('t(s)')
        grid on
        subplot(3,1,2)
        hold on
        pe_handle = graph_y(t, pe, 'p_e/m', []);
        xlabel('t(s)')
        grid on
        subplot(3,1,3)
        hold on
        h_handle = graph_y_yd(t, h, h_c, 'h/m', []);
        xlabel('t(s)')
        grid on
        
        figure(3), clf
        hold on
        Va_handle = graph_y_yd(t, Va, Va_c, 'V_a/m·s^{-1}', []);
        xlabel('t(s)')
        grid on
        
        figure(4), clf
        subplot(2,1,1)
        hold on
        alpha_handle = graph_y(t, alpha, '\alpha/°', []);
        xlabel('t(s)')
        grid on
        subplot(2,1,2)
        hold on
        beta_handle = graph_y_yd(t, beta, beta_c, '\beta/°', []);
        xlabel('t(s)')
        grid on
        
        figure(5), clf
        subplot(3,1,1)
        hold on
        phi_handle = graph_y_yd(t, phi, phi_c, '\phi/°', []);
        xlabel('t(s)')
        grid on
        subplot(3,1,2)
        hold on
        theta_handle = graph_y_yd(t, theta, theta_c, '\theta/°', []);
        xlabel('t(s)')
        grid on
        subplot(3,1,3)
        hold on
        chi_handle = graph_y_yd(t, chi, chi_c, '\chi/°', []);
        xlabel('t(s)')
        grid on
        
        figure(6), clf
        subplot(3,1,1)
        hold on
        p_handle = graph_y_yd(t, p, p_c, 'p/°·s^{-1}', []);
        xlabel('t(s)')       
        grid on
        subplot(3,1,2)
        hold on
        q_handle = graph_y_yd(t, q, q_c, 'q/°·s^{-1}', []);
        xlabel('t(s)')
        grid on
        subplot(3,1,3)
        hold on
        r_handle = graph_y_yd(t, r, r_c, 'r/°·s^{-1}', []);
        xlabel('t(s)')
        grid on
        
        figure(7), clf
        subplot(2,2,1)
        hold on
        delta_e_handle = graph_y(t, delta_e, '\delta_e/°', []);
        xlabel('t(s)')
        grid on
        subplot(2,2,2)
        hold on
        delta_a_handle = graph_y(t, delta_a, '\delta_a/°', []);
        xlabel('t(s)')
        grid on
        subplot(2,2,3)
        hold on
        delta_r_handle = graph_y(t, delta_r, '\delta_r/°', []);
        xlabel('t(s)')
        grid on
        subplot(2,2,4)
        hold on
        delta_t_handle = graph_y(t, delta_t, '\delta_t', []);
        xlabel('t(s)')
        grid on
    end
    
    % at every other time step, redraw state variables
    %在每隔一个时间步，重绘状态变量
else
    graph_y(t, pn, 'p_n', pn_handle);
    graph_y(t, pe, 'p_e', pe_handle);
    graph_y_yd(t, h, h_c, 'h', h_handle);
    graph_y_yd(t, Va, Va_c, 'V_a', Va_handle);
    graph_y(t, alpha, '\alpha', alpha_handle);
    graph_y_yd(t, beta, beta_c, '\beta', beta_handle);
    graph_y_yd(t, phi, phi_c, '\phi', phi_handle);
    graph_y_yd(t, theta, theta_c, '\theta', theta_handle);
    graph_y_yd(t, chi, chi_c, '\chi', chi_handle);
    graph_y_yd(t, p, p_c, 'p', p_handle);
    graph_y_yd(t, q, q_c, 'q', q_handle);
    graph_y_yd(t, r, r_c, 'r', r_handle);
    graph_y(t, delta_e, '\delta_e',delta_e_handle);
    graph_y(t, delta_a, '\delta_a',delta_a_handle);
    graph_y(t, delta_r, '\delta_r',delta_r_handle);
    graph_y(t, delta_t, '\delta_t',delta_t_handle);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, lab, handle)

if isempty(handle)
    handle    = plot(t,y,'k','LineWidth',1.5);
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)

if isempty(handle)
    handle(1)    = plot(t,y,'k','LineWidth',1.5);
    handle(2)    = plot(t,yd,'m--','LineWidth',1.5);
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its
% desired value yd in red, lab is the label on the graph
%绘制变量y为蓝色，其估计值为绿色，其期望值YD为红色，Lab是图上的标记。
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)

if isempty(handle),
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g--');
    handle(3)   = plot(t,yd,'r-.');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);
    %drawnow
end

%
%=============================================================================
% sat
% saturates the input between high and low饱和高与低之间的输入
%=============================================================================
%
function out=sat(in, low, high)

if in < low,
    out = low;
elseif in > high,
    out = high;
else
    out = in;
end

% end sat


