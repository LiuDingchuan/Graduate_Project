syms phi0(t) phi1(t) phi2(t) phi3(t) phi4(t) phi_dot_1 phi_dot_4 
syms l0 l1 l2 l3 l4 l5 theta0 theta1 theta2 theta3 theta4;


A0 = 2 * l2 * (xd - xb);
B0 = 2 * l2 * (yd - yb);
C0 = l2.^2 + lbd.^2 - l3.^2;
phi2 = 2*atan((B0+sqrt(A0^2+B0^2-C0^2)/(A0+C0)));

x_C = l1*cos(phi1)+l2*cos(phi2);
y_C = l1*sin(phi1)+l2*sin(phi2);

l0 = sqrt((x_C-l5/2)^2+y_C^2);
phi0 = atan(y_C/(x_C - l5/2));

x_dot = [l0; phi0];
q_dot = [phi1; phi4];