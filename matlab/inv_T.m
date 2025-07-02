clear;
syms l1 l4 l5 L0 phi0 phi1 phi2 phi3 phi4
j11 = (l1*l5*sin(phi0-phi3)*sin(phi1-phi2))/(l4*sin(phi3-phi2));
j12 = (l1*l5*cos(phi0-phi3)*sin(phi1-phi2))/(L0*l4*sin(phi3-phi2));
j21 = (l5*sin(phi0-phi2)*sin(phi3-phi4))/sin(phi3-phi2);
j22 = (l5*cos(phi0-phi2)*sin(phi3-phi4))/(L0*sin(phi3-phi2));

T = [j11 j12; j21 j22];
T1 = simplify(inv(T));
