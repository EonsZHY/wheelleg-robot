clear;
syms phi1(t) phi2(t) phi3(t) phi4(t) phi_dot_1 phi_dot_4 l1 l2 l3 l4 l5 F Tp phi0 L0

% 定义坐标和角度
x_B = l1*cos(phi1);
y_B = l1*sin(phi1);
x_C = l5/l4*(x_B + l2*cos(phi2));
y_C = l5/l4*(y_B + l2*sin(phi2));
x_D = -l4*cos(phi4);
y_D = l4*sin(phi4);

% % 定义 phi0 和 L0（不展开）
% phi0 = atan2(y_C, x_C);  % 保持 atan2 形式
% L0 = sqrt(x_C^2 + y_C^2); % 保持 sqrt 形式

% 计算速度
x_dot_B = diff(x_B, t);
y_dot_B = diff(y_B, t);
x_dot_C = diff(x_C, t);
y_dot_C = diff(y_C, t);
x_dot_D = diff(x_D, t);
y_dot_D = diff(y_D, t);

% 计算 phi_dot_2（替换微分项）
phi_dot_2 = ((x_dot_D - x_dot_B)*cos(phi3) + (y_dot_D - y_dot_B)*sin(phi3)) / (l2*sin(phi3 - phi2));
x_dot_C = subs(x_dot_C, diff(phi2, t), phi_dot_2);
x_dot_C = subs(x_dot_C, [diff(phi1, t), diff(phi4, t)], [phi_dot_1, phi_dot_4]);
y_dot_C = subs(y_dot_C, diff(phi2, t), phi_dot_2);
y_dot_C = subs(y_dot_C, [diff(phi1, t), diff(phi4, t)], [phi_dot_1, phi_dot_4]);

% 构建速度向量和广义速度
x_dot = [x_dot_C; y_dot_C];
q_dot = [phi_dot_1; phi_dot_4];

% 计算雅可比矩阵 J（不展开 L0 和 phi0）
J = simplify(jacobian(x_dot, q_dot));

% 定义旋转矩阵 R 和变换矩阵 M（直接使用 phi0 和 L0 的符号形式）
R = [cos(phi0 - pi/2), -sin(phi0 - pi/2);
     sin(phi0 - pi/2),  cos(phi0 - pi/2)];
M = [0, -1/L0;
     1,     0];

% 计算 T = J^T * R * M
T = simplify(J.' * R * M);
inv_T = simplify(inv(T));


