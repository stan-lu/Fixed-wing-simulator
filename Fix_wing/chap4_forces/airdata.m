function out=airdata(uu)
%
% Fake air data - this will be replaced with real air data in Chapter 4
%
% modified 12/11/2009 - RB

    % process inputs to function
 % 状态变量
    pn          = uu(1);             % 惯性北向位置 （米）
    pe          = uu(2);             % 惯性东向位置 (米)
    h           = -uu(3);            % 高度 (米，与惯性地向位置相反)
    u           = uu(4);             % 机体x轴速度 (米/秒)
    v           = uu(5);             % 机体y轴速度 (米/秒)
    w           = uu(6);             % 机体z轴速度(米/秒)
    phi         = 180/pi*uu(7);      % 滚转角 (度)   
    theta       = 180/pi*uu(8);      % 俯仰角(度)
    psi         = 180/pi*uu(9);      % 偏航角 (度)
    p           = 180/pi*uu(10);     % 机体x轴滚转速度 (度/秒)
    q           = 180/pi*uu(11);     % 机体y轴俯仰速度 (度/秒)
    r           = 180/pi*uu(12);     % 机体z轴偏航速度 (度/秒)

    Va = sqrt(u^2+v^2+w^2);%空速
    alpha = atan2(w,u);%机翼与空速矢量夹角，攻角
    beta  = asin(v);%速度矢量与机体i-k平面夹角，侧滑角
    wn    = 0;%风的北向分量
    we    = 0;%风的东向分量
    wd    = 0;%风的地向分量
    
    out = [Va; alpha; beta; wn; we; wd];
    