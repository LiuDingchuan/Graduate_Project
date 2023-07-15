%返回摆杆的质心坐标和转动惯量
%还是得封装啊，不然出错概率太高了
function [x, y, I] = Simplify_model(pitch, theta1, theta4, xa, ya, xb, yb, xc, yc, xd, yd, xe, ye)
global l1 l2 l3 l4 l5;
global ml1 ml2 ml3 ml4;
global Il1 Il2 Il3 Il4;
%求质心位置
syms mx_l1 my_l1 mx_l2 my_l2 mx_l3 my_l3 mx_l4 my_l4;
mx_l1 = 0.43965*(xb-xa)+xa;
my_l1 = 0.43965*(yb-ya)+ya;
mx_l2 = 0.4752*(xc-xb)+xb;
my_l2 = 0.4752*(yc-yb)+yb;
mx_l3 = 0.4752*(xc-xd)+xd;
my_l3 = 0.4752*(yc-yd)+yd;
mx_l4 = 0.43965*(xd-xe)+xe;
my_l4 = 0.43965*(yd-ye)+ye;
x = (mx_l1*ml1 + mx_l2*ml2 + mx_l3*ml3 + mx_l4*ml4)/(ml1+ml2+ml3+ml4);
y = (my_l1*ml1 + my_l2*ml2 + my_l3*ml3 + my_l4*ml4)/(ml1+ml2+ml3+ml4);
d1 = sqrt((x - mx_l1)^2 + (y - my_l1)^2) * 0.001;%mm-> m
d2 = sqrt((x - mx_l2)^2 + (y - my_l2)^2) * 0.001;
d3 = sqrt((x - mx_l3)^2 + (y - my_l3)^2) * 0.001;
d4 = sqrt((x - mx_l4)^2 + (y - my_l4)^2) * 0.001;

I = (Il1 + ml1 *d1^2) + (Il2 + ml2 * d2^2) + (Il3 + ml3 * d3^2) + (Il4 + ml4 * d4^2);%平行轴定理
end