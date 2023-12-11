%闭链五杆的正解，根据电机的角度求足端末点的坐标
%u1为l1从x轴顺时针转的角度，为负值；u4为l4从x轴逆时针转的角度
function [x, y, u2, u3] = Zjie(u1, u4, pitch)%选择以l5/2处为零点进行建模
global l1 l2 l3 l4 l5;
xb = l1 * cos(u1) - l5/2;
yb = l1 * sin(u1);
xd = l5/2 + l4 * cos(u4);
yd = l4 * sin(u4);
lbd = sqrt((xd - xb).^2 + (yd - yb).^2);
A0 = 2 * l2 * (xd - xb);
B0 = 2 * l2 * (yd - yb);
C0 = l2.^2 + lbd.^2 - l3.^2;
D0 = l3.^2 + lbd.^2 - l2.^2;
u2 = 2 * atan((B0 + sqrt(A0.^2 + B0.^2 - C0.^2))/(A0 + C0));%rad\
u3 = pi - 2 * atan((-B0 + sqrt(A0.^2 + B0.^2 - D0.^2))/(A0 + D0));
xc = xb + l2 * cos(u2);
yc = yb + l2 * sin(u2);
R = [cos(pitch), -sin(pitch);
    sin(pitch), cos(pitch)];
v = R*[xc;yc];
x = v(1)
y = v(2)

end