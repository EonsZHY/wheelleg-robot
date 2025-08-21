
clear; clc;
syms phi1 phi4 d_phi1 d_phi4 % 定义符号变量（角度及其导数）
syms l1 l2 l3 l4 l5         % 定义符号常数（连杆长度）

% 给定的常数值（用于后续数值计算和验证）
l1_val = 0.075;
l2_val = 0.125;
l3_val = 0.125;
l4_val = 0.075;
l5_val = 0.150;
% 添加实数假设
assume(phi1, 'real');
assume(phi4, 'real');
assume(d_phi1, 'real');
assume(d_phi4, 'real');
assume(l1, 'real'); assume(l1 > 0); % 杆长不仅为实数，还大于0
assume(l2, 'real'); assume(l2 > 0);
assume(l3, 'real'); assume(l3 > 0);
assume(l4, 'real'); assume(l4 > 0);
assume(l5, 'real'); assume(l5 > 0);
%% 第二步：建立机构的几何关系（位置分析）
% 1. 计算点D和点B的坐标
XD = l4 * cos(phi4);
YD = l4 * sin(phi4);
XB = l1 * cos(phi1);
YB = l1 * sin(phi1);

% 2. 计算B, D之间的距离
lBD = sqrt((XD - XB)^2 + (YD - YB)^2);

% 3. 求解phi2 (使用余弦定理，得到闭合方程的参数)
A0 = 2 * l2 * (XD - XB);
B0 = 2 * l2 * (YD - YB);
C0 = l2^2 + lBD^2 - l3^2;

% phi2有两个解，根据装配模式选择其中一个。
% 注意：这个表达式定义了phi2与phi1, phi4的函数关系 phi2 = f(phi1, phi4)
phi2 = 2 * atan2( (B0 - sqrt(A0^2 + B0^2 - C0^2)), (A0 + C0) );

% 4. 计算点C的坐标 (核心比例关系)
XC = (l5 / l4) * (XB + l2 * cos(phi2)); % XB = l1*cos(phi1)
YC = (l5 / l4) * (YB + l2 * sin(phi2)); % YB = l1*sin(phi1)

% 5. 计算输出角度phi0
phi0 = atan2(YC, XC);

%% 第三步：速度分析（符号求导）
% 关键：phi0是phi1和phi4的函数，而phi1和phi4又是时间t的函数。
% 根据链式法则，d(phi0)/dt = d(phi0)/d(phi1) * d(phi1)/dt + d(phi0)/d(phi4) * d(phi4)/dt
% 即 d_phi0 = diff(phi0, phi1) * d_phi1 + diff(phi0, phi4) * d_phi4


% 计算偏导数 ∂(phi0)/∂(phi1) 和 ∂(phi0)/∂(phi4)
dphi0_dphi1 = diff(phi0, phi1);
dphi0_dphi4 = diff(phi0, phi4);

% 构建完整的角速度表达式
d_phi0_sym = dphi0_dphi1 * d_phi1 + dphi0_dphi4 * d_phi4;