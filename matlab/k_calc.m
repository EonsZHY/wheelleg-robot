
tic
clear all
clc
%% 

% 定义机器人机体参数
syms R_w;                % 驱动轮半径
syms R_l;               % 驱动轮轮距/2
syms l_l l_r;           % 左右腿长
syms l_wl l_wr;          % 驱动轮质心到左右腿部质心距离
syms l_bl l_br;          % 机体质心到左右腿部质心距离
syms l_c;                % 机体质心到腿部关节中心点距离
syms m_w m_l m_b;        % 驱动轮质量 腿部质量 机体质量
syms I_w;                % 驱动轮转动惯量           (自然坐标系法向)
syms I_ll I_lr;          % 左右腿部转动惯量    (自然坐标系法向，实际上会变化)
syms I_b;                % 机体转动惯量             (自然坐标系法向)
syms I_z;               % 机器人z轴转动惯量        (简化为常量)

%定义变量及其倒数
syms theta_wl dtheta_wl ddtheta_wl;         %左腿驱动轮转角及其倒数
syms theta_wr dtheta_wr ddtheta_wr;        %右腿驱动轮转角及其倒数
syms theta_ll dtheta_ll ddtheta_ll;         %左腿倾斜角及其倒数
syms theta_lr dtheta_lr ddtheta_lr;         %右腿倾斜角及其倒数
syms theta_b dtheta_b ddtheta_b;            %机体倾斜角
syms s ds dds;                              %机器人水平方向移动距离
syms s_b ds_b dds_b;                        %机体质心水平方向移动距离
syms phi dphi ddphi;                      %偏航角yaw
syms h_b dh_b ddh_b;                        %机体质心竖直方向移动距离
syms s_ll ds_ll dds_ll;                     %左腿腿部质心水平方向移动距离
syms s_lr ds_lr dds_lr;                    %右腿腿部之心水平方向移动距离
syms h_ll dh_ll ddh_ll;                    %左腿腿部质心竖直方向移动距离
syms h_lr dh_lr ddh_lr;                    %右腿腿部质心竖直方向移动距离
syms T_wl T_wr T_bl T_br;                   %驱动轮转矩（腿-轮）  腿部转矩（机体-腿）
syms F_w_sl F_w_sr;                        %驱动轮对腿水平方向作用力
syms F_w_hl F_w_hr;                         %驱动轮对腿竖直方向作用力
syms F_b_sl F_b_sr;                         %腿对机体水平方向作用力
syms F_b_hl F_b_hr;                        %腿对机体竖直方向作用力
syms g;                                    %重力加速度
%% 输入机器人的基本参数
% R_w=0.091;
% R_l=0.2275;
% l_c=0.02569;
% m_w=1.62;
% m_l=0.92;
% m_b=11.92;
% I_w=0.005874;
% I_b=0.287855;
% I_z=0.5;
% g=9.8;
R_w=0.086;
R_l=0.2275;
l_c=0.2125;
m_w=1.18;
m_l=1.11;
m_b=10.3;
I_w=m_w*R_w^2;
I_b=m_b*(0.3^2+0.12^2)/12;
I_z=0.182;
g=9.8;

%% 输入Q、R矩阵
%Q_cost=diag([50 150 50 120 10 10 10 10 2500 50]);
%Q_cost=diag([10 10 10 10 150 120 150 120 2000 50]);
Q_cost=diag([150 100 150 100 10 10 10 10 150 100]);
R_cost=diag([3 3 0.25 0.25]);


%% 解方程，求控制矩阵A、B

% ddot_s=ddot_s_body+L_l/2*sin(theta_l_l)*dot_theta_l_l^2-L_l/2*cos(theta_l_l)*ddot_theta_l_l+L_r/2*sin(theta_l_r)*dot_theta_l_r^2-L_r/2*cos(theta_l_r)*ddot_theta_l_r-L_c*sin(theta_body)*dot_theta_body^2+L_c*cos(theta_body)*ddot_theta_body;
% ddot_theta_w_r=+Width_body_half/R_wheel*ddot_phi+ddot_s/R_wheel+L_l*cos(theta_l_l)*ddot_theta_l_l/(2*R_wheel)-L_l*sin(theta_l_l)*dot_theta_l_l^2/(2*R_wheel)-L_r*cos(theta_l_r)*ddot_theta_l_r/(2*R_wheel)+L_r*sin(theta_l_r)*dot_theta_l_r^2/(2*R_wheel);
% ddot_theta_w_l=-Width_body_half/R_wheel*ddot_phi+ddot_s/R_wheel-L_l*cos(theta_l_l)*ddot_theta_l_l/(2*R_wheel)+L_l*sin(theta_l_l)*dot_theta_l_l^2/(2*R_wheel)+L_r*cos(theta_l_r)*ddot_theta_l_r/(2*R_wheel)-L_r*sin(theta_l_r)*dot_theta_l_r^2/(2*R_wheel);
% ddot_h_body=-L_l/2*sin(theta_l_l)*ddot_theta_l_l-L_l/2*cos(theta_l_l)*dot_theta_l_l^2-L_r/2*sin(theta_l_r)*ddot_theta_l_r-L_r/2*cos(theta_l_r)*dot_theta_l_r^2-L_c*cos(theta_body)*dot_theta_body^2-L_c*sin(theta_body)*ddot_theta_body;
% ddot_h_l_l=ddot_h_body+L_c*cos(theta_body)*dot_theta_body^2+L_c*sin(theta_body)*ddot_theta_body+L_body_l*cos(theta_l_l)*dot_theta_l_l^2+L_body_l*sin(theta_l_l)*ddot_theta_l_l;
% ddot_h_l_r=ddot_h_body+L_c*cos(theta_body)*dot_theta_body^2+L_c*sin(theta_body)*ddot_theta_body+L_body_r*cos(theta_l_r)*dot_theta_l_r^2+L_body_r*sin(theta_l_r)*ddot_theta_l_r;
% ddot_s_l_l=R_wheel*ddot_theta_w_l+L_wheel_l*cos(theta_l_l)*ddot_theta_l_l-L_wheel_l*sin(theta_l_l)*dot_theta_l_l^2;
% ddot_s_l_r=R_wheel*ddot_theta_w_r+L_wheel_r*cos(theta_l_r)*ddot_theta_l_r-L_wheel_r*sin(theta_l_r)*dot_theta_l_r^2;

%运动学关系式
dds = dds_b+l_l/2*sin(theta_ll)*dtheta_ll^2-l_l/2*cos(theta_ll)*ddtheta_ll+l_r/2*sin(theta_lr)*dtheta_lr^2-l_r/2*cos(theta_lr)*ddtheta_lr-l_c*sin(theta_b)*dtheta_b^2+l_c*cos(theta_b)*ddtheta_b;
ddtheta_wr = +R_l/R_w*ddphi+dds/R_w+l_l*cos(theta_ll)*ddtheta_ll/(2*R_w)-l_l*sin(theta_ll)*dtheta_ll^2/(2*R_w)-l_r*cos(theta_lr)*ddtheta_lr/(2*R_w)+l_r*sin(theta_lr)*dtheta_lr^2/(2*R_w);
ddtheta_wl = -R_l/R_w*ddphi+dds/R_w-l_l*cos(theta_ll)*ddtheta_ll/(2*R_w)+l_l*sin(theta_ll)*dtheta_ll^2/(2*R_w)+l_r*cos(theta_lr)*ddtheta_lr/(2*R_w)-l_r*sin(theta_lr)*dtheta_lr^2/(2*R_w);
ddh_b = -l_l/2*sin(theta_ll)*ddtheta_ll-l_l/2*cos(theta_ll)*dtheta_ll^2-l_r/2*sin(theta_lr)*ddtheta_lr-l_r/2*cos(theta_lr)*dtheta_lr^2-l_c*cos(theta_b)*dtheta_b^2-l_c*sin(theta_b)*ddtheta_b;
ddh_ll = ddh_b+l_c*cos(theta_b)*dtheta_b^2+l_c*sin(theta_b)*ddtheta_b+l_bl*cos(theta_ll)*dtheta_ll^2+l_bl*sin(theta_ll)*ddtheta_ll;
ddh_lr = ddh_b+l_c*cos(theta_b)*dtheta_b^2+l_c*sin(theta_b)*ddtheta_b+l_br*cos(theta_lr)*dtheta_lr^2+l_br*sin(theta_lr)*ddtheta_lr;
dds_ll = R_w*ddtheta_wl+l_wl*cos(theta_ll)*ddtheta_ll-l_wl*sin(theta_ll)*dtheta_ll^2;
dds_lr = R_w*ddtheta_wr+l_wr*cos(theta_lr)*ddtheta_lr-l_wr*sin(theta_lr)*dtheta_lr^2;

%动力学关系式
% F_body_h_l=(M_body*ddot_h_body+M_body*g+M_leg*ddot_h_l_r-M_leg*ddot_h_l_l)/2;
% F_body_h_r=(M_body*ddot_h_body+M_body*g-M_leg*ddot_h_l_r+M_leg*ddot_h_l_l)/2;
% F_wheel_h_l=F_body_h_l+M_leg*g+M_leg*ddot_h_l_l;
% F_wheel_h_r=F_body_h_r+M_leg*g+M_leg*ddot_h_l_r;
% F_wheel_s_l=(T_wheel_l-(I_wheel+M_wheel*R_wheel^2)*ddot_theta_w_l)/R_wheel;
% F_wheel_s_r=(T_wheel_r-(I_wheel+M_wheel*R_wheel^2)*ddot_theta_w_r)/R_wheel;
% F_body_s_l=F_wheel_s_l-M_leg*ddot_s_l_l;
% F_body_s_r=F_wheel_s_r-M_leg*ddot_s_l_r;

F_b_hl = (m_b*ddh_b+m_b*g+m_l*ddh_lr-m_l*ddh_ll)/2;
F_b_hr = (m_b*ddh_b+m_b*g-m_l*ddh_lr+m_l*ddh_ll)/2;
F_w_hl = F_b_hl+m_l*g+m_l*ddh_ll;
F_w_hr = F_b_hr+m_l*g+m_l*ddh_lr;
F_w_sl = (T_wl-(I_w+m_w*R_w^2)*ddtheta_wl)/R_w;
F_w_sr = (T_wr-(I_w+m_w*R_w^2)*ddtheta_wr)/R_w;
F_b_sl = F_w_sl-m_l*dds_ll;
F_b_sr = F_w_sr-m_l*dds_lr;

% equ1=F_body_s_l+F_body_s_r-M_body*ddot_s_body;
% equ2=I_leg_l*ddot_theta_l_l-(F_wheel_h_l*L_wheel_l+F_body_h_l*L_body_l)*sin(theta_l_l)+(F_wheel_s_l*L_wheel_l+F_body_s_l*L_body_l)*cos(theta_l_l)+T_wheel_l-T_body_l;
% equ3=I_leg_r*ddot_theta_l_r-(F_wheel_h_r*L_wheel_r+F_body_h_r*L_body_r)*sin(theta_l_r)+(F_wheel_s_r*L_wheel_r+F_body_s_r*L_body_r)*cos(theta_l_r)+T_wheel_r-T_body_r;
% equ4=I_body*ddot_theta_body-T_body_l-T_body_r-(F_body_s_l+F_body_s_r)*L_c*cos(theta_body)-(F_body_h_l+F_body_h_r)*L_c*sin(theta_body);
% equ5=I_z*ddot_phi+(M_wheel*R_wheel*ddot_theta_w_l+F_wheel_s_l-M_wheel*R_wheel*ddot_theta_w_r-F_wheel_s_r)*Width_body_half;

equ1 = F_b_sl+F_b_sr-m_b*dds_b;
equ2 = I_ll*ddtheta_ll-(F_w_hl*l_wl+F_b_hl*l_bl)*sin(theta_ll)+(F_w_sl*l_wl+F_b_sl*l_bl)*cos(theta_ll)+T_wl-T_bl;
equ3 = I_lr*ddtheta_lr-(F_w_hr*l_wr+F_b_hr*l_br)*sin(theta_lr)+(F_w_sr*l_wr+F_b_sr*l_br)*cos(theta_lr)+T_wr-T_br;
equ4 = I_b*ddtheta_b-T_bl-T_br-(F_b_sl+F_b_sr)*l_c*cos(theta_b)-(F_b_hl+F_b_hr)*l_c*sin(theta_b);
equ5 = I_z*ddphi+(m_w*R_w*ddtheta_wl+F_w_sl-m_w*R_w*ddtheta_wr-F_w_sr)*R_l;

[dds_b,ddphi,ddtheta_ll,ddtheta_lr,ddtheta_b] = solve([equ1, equ2, equ3, equ4, equ5], [dds_b,ddphi,ddtheta_ll,ddtheta_lr,ddtheta_b]);
Ja = jacobian([ds_b,dds_b, dphi, ddphi, dtheta_ll, ddtheta_ll, dtheta_lr, ddtheta_lr, dtheta_b, ddtheta_b], [s_b, ds_b, phi, dphi, theta_ll, dtheta_ll, theta_lr, dtheta_lr, theta_b, dtheta_b]);
Jb = jacobian([ds_b,dds_b, dphi, ddphi, dtheta_ll, ddtheta_ll, dtheta_lr, ddtheta_lr, dtheta_b, ddtheta_b], [T_wl, T_wr, T_bl, T_br]);

A = simplify(vpa(subs(Ja, [s_b, ds_b, phi, dphi, theta_ll, dtheta_ll, theta_lr, dtheta_lr, theta_b, dtheta_b], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0])));
B = simplify(vpa(subs(Jb, [s_b, ds_b, phi, dphi, theta_ll, dtheta_ll, theta_lr, dtheta_lr, theta_b, dtheta_b], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0])));

disp(A);
disp(B);

% %% 对不同腿长下的LQR进行计算
% 
% %机器人腿部参数数据采集
% % 第一列为左腿腿长范围区间中所有小数点精度0.01的长度，例如：0.09，0.18，单位：m
% % 第二列为l_wl，单位：m
% % 第三列为l_bl，单位：m
% % 第四列为I_ll，单位：kg m^2
% 
% 
% 
% leg_data_l=[1, 1, 1, 1;
%             1.01, 1, 1, 1;
%             1.02, 1, 1, 1;
%             1.03, 1, 1, 1;
%             1.04, 1, 1, 1;
%             1.05, 1, 1, 1;
%             1.06, 1, 1, 1;
%             1.07, 1, 1, 1;
%             1.08, 1, 1, 1;
%             1.09, 1, 1, 1;
%             1.1, 1, 1, 1;
%             1.11, 1, 1, 1;
%             1.12, 1, 1, 1;
%             1.13, 1, 1, 1;
%             1.14, 1, 1, 1;
%             1.15, 1, 1, 1;
%             1.16, 1, 1, 1;
%             1.17, 1, 1, 1;
%             1.18, 1, 1, 1;
%             1.19, 1, 1, 1;
%             1.2, 1, 1, 1;
%             1, 1, 1, 1;
%             1, 1, 1, 1;
%             1, 1, 1, 1;
%             1, 1, 1, 1;
%             1, 1, 1, 1;
%             1, 1, 1, 1;
%             1, 1, 1, 1;
%             1, 1, 1, 1;
%             1, 1, 1, 1];
% 
% 
% 
% leg_data_r = leg_data_l;
%% 在不同腿长下对K矩阵进行拟合

leg_length = 0.1:0.01:0.4;
% sample_size = size(leg_data_l,1)^2; % 单个K_ij拟合所需要的样本数
sample_size = size(leg_length,2)^2;
% length = size(leg_data_l,1); % 测量腿部数据集的行数
length = size(leg_length,2);
K=zeros(sample_size,40);
K_test=zeros(sample_size,40);
x1=zeros(sample_size,1);
x2=zeros(sample_size,1);
x1sq=zeros(sample_size,1);
x2sq=zeros(sample_size,1);
x1x2=zeros(sample_size,1);

% for i=1:length
%     % l_varl = leg_data_l(i,1);
%     % l_wl_ac = leg_data_l(i,2);
%     % l_bl_ac = leg_data_l(i,3);
%     % I_ll_ac = leg_data_l(i,4);
%     for j=1:length
%         l_varr = leg_data_r(j,1);
%         l_wr_ac = leg_data_r(j,2);
%         l_br_ac = leg_data_r(j,3);
%         I_lr_ac = leg_data_r(j,4);
%         trans_A=subs(A,[l_l l_r l_wl l_wr l_bl l_br I_ll I_lr],[l_varl l_varr l_wl_ac l_wr_ac l_bl_ac l_br_ac I_ll_ac I_lr_ac]);
%         trans_B=subs(B,[l_l l_r l_wl l_wr l_bl l_br I_ll I_lr],[l_varl l_varr l_wl_ac l_wr_ac l_bl_ac l_br_ac I_ll_ac I_lr_ac]);
%     %kk为不同时刻的反馈增益矩阵
%         KK=lqrd(trans_A,trans_B,Q_cost,R_cost,0.001);
%         KK_t=KK.';
%         K((i-1)*30+j,:)=KK_t(:);
%         x1((i-1)*30+j,1)=l_varl;
%         x2((i-1)*30+j,1)=l_varr;
%         x1sq((i-1)*30+j,1)=l_varl^2;
%         x2sq((i-1)*30+j,1)=l_varr^2;
%         x1x2((i-1)*30+j,1)=l_varl*l_varr;
%     end
% end
for i=1:length
    l_varl = 0.1+(i-1)*0.01;
    l_wl_ac = l_varl/2;
    l_bl_ac = l_varl/2;
    I_ll_ac = m_l*((l_varl)^2+0.05^2)/12;
    for j=1:length
    l_varr = 0.1+(j-1)*0.01;
    l_wr_ac = l_varr/2;
    l_br_ac = l_varr/2;
    I_lr_ac = m_l*((l_varr)^2+0.05^2)/12;
    trans_A=subs(A,[l_l l_r l_wl l_wr l_bl l_br I_ll I_lr],[l_varl l_varr l_wl_ac l_wr_ac l_bl_ac l_br_ac I_ll_ac I_lr_ac]);
    trans_B=subs(B,[l_l l_r l_wl l_wr l_bl l_br I_ll I_lr],[l_varl l_varr l_wl_ac l_wr_ac l_bl_ac l_br_ac I_ll_ac I_lr_ac]);
    %kk为不同时刻的反馈增益矩阵
     KK=lqrd(trans_A,trans_B,Q_cost,R_cost,0.001);
     KK_t=KK.';
     K((i-1)*length+j,:)=KK_t(:);
     x1((i-1)*length+j,1)=l_varl;
     x2((i-1)*length+j,1)=l_varr;
     x1sq((i-1)*length+j,1)=l_varl^2;
     x2sq((i-1)*length+j,1)=l_varr^2;
     x1x2((i-1)*length+j,1)=l_varl*l_varr;
     end
 end
K_cons = zeros(40,6);
% 系数拟合
X = [x1, x2, x1sq, x2sq, x1x2];
%决定每个kij的拟合方法
for k=1:40
    mdl = fitlm(X, K(:,k));
    disp(mdl.Coefficients.Estimate');
    K_cons(k,:)=mdl.Coefficients.Estimate'; 

end

%% 提取k_cons的每一行作为向量
for i = 1:4
    for j = 1:10
        idx = (i-1)*10 + j; %对应于kij在k_cons中的行号
        name = sprintf('a%d%d',i,j);  %构造aij向量,对应于kij的六个拟合系数
        eval([name ' = K_cons(idx, :);']);
    end
end
