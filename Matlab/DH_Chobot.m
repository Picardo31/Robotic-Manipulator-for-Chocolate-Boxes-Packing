function [O_T_1, i1_T_2, i2_T_3, i3_T_4, i4_T_T, O_T_T] = DH_Chobot(phi1,phi2,phi3,phi4)
syms theta1 theta2 theta3 theta4

alpha1= 0;
a_i1 = 0;
d_i1 = 0;
phi1 = phi1*pi/180;

O_T_1 = [cos(phi1) -sin(phi1) 0 a_i1; 
        sin(phi1)*cos(alpha1) cos(phi1)*cos(alpha1) -sin(alpha1) -d_i1*sin(alpha1);
        sin(phi1)*sin(alpha1) cos(phi1)*sin(alpha1) cos(alpha1) d_i1*cos(alpha1);
        0 0 0 1];
alpha2 = pi/2;
a_i2 = 0;
d_i2 = 0;
phi2 = phi2*pi/180;

i1_T_2 = [cos(phi2) -sin(phi2) 0 a_i2; 
        sin(phi2)*round(cos(alpha2)) cos(phi2)*round(cos(alpha2)) -round(sin(alpha2)) -d_i2*round(sin(alpha2));
        sin(phi2)*round(sin(alpha2)) cos(phi2)*round(sin(alpha2)) round(cos(alpha2)) d_i1*round(cos(alpha2));
        0 0 0 1];
O_T_2 = O_T_1*i1_T_2;

alpha3 = 0;
a_i3 = 120;
d_i3 = 0;
phi3 = phi3*pi/180;

i2_T_3 = [cos(phi3) -sin(phi3) 0 a_i3; 
        sin(phi3)*round(cos(alpha3)) cos(phi3)*round(cos(alpha3)) -round(sin(alpha3)) -d_i3*round(sin(alpha3));
        sin(phi3)*round(sin(alpha3)) cos(phi3)*round(sin(alpha3)) round(cos(alpha3)) d_i3*round(cos(alpha3));
        0 0 0 1];
O_T_3 = O_T_1*i1_T_2*i2_T_3;

alpha4 = 0;
a_i4 = 120;
d_i4 = 0;
%phi4 = abs(phi3)-phi2;
phi4 = phi4*pi/180;

i3_T_4 = [cos(phi4) -sin(phi4) 0 a_i4; 
        sin(phi4)*round(cos(alpha4)) cos(phi4)*round(cos(alpha4)) -round(sin(alpha4)) -d_i2*round(sin(alpha4));
        sin(phi4)*round(sin(alpha4)) cos(phi4)*round(sin(alpha4)) round(cos(alpha4)) d_i2*round(cos(alpha4));
        0 0 0 1];
    
O_T_4 = O_T_1*i1_T_2*i2_T_3*i3_T_4;

a = 58.49;
b = -28.45;
i4_T_T = [1 0 0 a; 0 1 0 b; 0 0 1 0; 0 0 0 1];
O_T_T = O_T_1*i1_T_2*i2_T_3*i3_T_4*i4_T_T;

end