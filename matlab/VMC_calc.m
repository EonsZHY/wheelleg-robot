function [T1,T2] = VMC_calc(F0,Tp,phi1,phi4)
% syms T1 T2 F0 Tp phi1 phi4 XE l1 l2 l3 l4
l5=0.108;
l1=0.15;
l2=0.25;
l3=0.25;
l4=0.15;

y_D = l4*sin(phi4);
y_B = l1*sin(phi1);
x_D = l4*cos(phi4);
x_B = l1*cos(phi1);
lBD = sqrt((x_D - x_B)*(x_D - x_B) + (y_D - y_B)*(y_D - y_B));
A0 = 2*l2*(x_D - x_B);
B0 = 2*l2*(y_D - y_B);
C0 = l2*l2 + lBD*lBD - l3*l3;
phi2 = 2*atan2((B0 + sqrt(A0*A0 + B0*B0 - C0*C0)),A0 + C0);
phi3 = atan2(YB-YD+l2*sin(phi2),XB-XD+l2*cos(phi2));
x_C = l1*cos(phi1) + l2*cos(phi2);
y_C = l1*sin(phi1) + l2*sin(phi2);
L0 = sqrt(x_C^2+y_C^2);
phi0 = atan2(YC,XC);
j11 = (l1*l5*sin(phi0-phi3)*sin(phi1-phi2))/l4*sin(phi3-phi2);
j12 = (l1*l5*cos(phi0-phi3)*sin(phi1-phi2))/L0*l4*sin(phi3-phi2);
j21 = (l5*sin(phi0-phi2)*sin(phi3-phi4))/sin(phi3-phi2);
j22 = (l5*cos(phi0-phi2)*sin(phi3-phi4))/L0*sin(phi3-phi2);
J = [j11 j12; j21 j22];
F = [F0;Tp];
T = J * F;
T1 = T(1,1);
T2 = T(2,1);
end

