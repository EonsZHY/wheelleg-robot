clear; clc;
syms phi1 phi4 d_phi1 d_phi4 % 定义符号变量（角度及其导数）
syms l1 l2 l3 l4 l5         % 定义符号常数（连杆长度）

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
assumeAlso(A0^2 + B0^2 - C0^2 >= 0);
phi2 = 2 * atan( (B0 - sqrt(A0^2 + B0^2 - C0^2)), (A0 + C0) );

% % 计算偏导数 ∂(phi2)/∂(phi1) 和 ∂(phi2)/∂(phi4)
d_phi2_d_phi1 = diff(phi2,phi1);
d_phi2_d_phi4 = diff(phi2,phi4);

simplify(d_phi2_d_phi1);
simplify(d_phi2_d_phi4);

disp(d_phi2_d_phi1);
disp(d_phi2_d_phi4);