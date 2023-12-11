function [T1, T2] = VMC(F, Tp)
syms phi0(t) phi1(t) phi2(t) phi3(t) phi4(t) phi_dot_1 phi_dot_4 
syms l0 l1 l2 l3 l4 l5 theta0 theta1 theta2 theta3 theta4;
x_B = l1*cos(phi1);
y_B = l1*sin(phi1);
x_C = x_B+l2*cos(phi2);
y_C = y_B+l2*sin(phi2);
x_D = l5+l4*cos(phi4);
y_D = l4*sin(phi4);
x_dot_B = diff(x_B,t);
y_dot_B = diff(y_B,t);
x_dot_C = diff(x_C,t);
y_dot_C = diff(y_C,t);
x_dot_D = diff(x_D,t);
y_dot_D = diff(y_D,t);

phi_dot_2 = ((x_dot_D-x_dot_B)*cos(phi3)+(y_dot_D-y_dot_B)*sin(phi3))/l2/sin(phi3-phi2);

x_dot_C = subs(x_dot_C,diff(phi2,t),phi_dot_2);
x_dot_C = subs(x_dot_C,...
    [diff(phi1,t),diff(phi4,t)],...
    [phi_dot_1,phi_dot_4])
y_dot_C = subs(y_dot_C,diff(phi2,t),phi_dot_2);
y_dot_C = subs(y_dot_C,...
    [diff(phi1,t),diff(phi4,t)],...
    [phi_dot_1,phi_dot_4])

x_dot = [x_dot_C; y_dot_C];
q_dot = [phi_dot_1; phi_dot_4];
x_dot = simplify(collect(x_dot,q_dot));%如果 x_dot 是一个多项式，其中包含多个项，每个项都是由 q_dot 中的变量乘以一个系数得到的。那么，使用 collect(x_dot,q_dot) 将把这些项按照它们所乘以的变量进行分组。然后，使用 simplify 函数可以进一步化简结果。
J = simplify(jacobian(x_dot,q_dot))%simplify是用来化简符号表达式的（很好用，可以把公式简化成人能看懂的排列）

R = [cos(-phi0) -sin(-phi0);
     sin(-phi0)  cos(-phi0)];
M = [0    1/l0;
     -1 0];
T = simplify(J.'*R*M)
symdisp(T)

T = subs(T, ...
    [phi0(t), phi1(t), phi2(t), phi3(t), phi4(t)], ...
    [theta0, theta1, theta2, theta3, theta4]);
torque = double(T)*[F;Tp];
disp(double(T));
T1 = torque(1);
T2 = torque(2);
end