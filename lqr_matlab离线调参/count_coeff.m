%通过编码器和imu得到pitch轴数据和髋部两电机的编码器数据进行解析
%最后得到简化后的腿部摆杆质心位置和转动惯量
clc
clear
format short
global l1 l2 l3 l4 l5;
l1 = 180;%单位为mm
l2 = 200;
l3 = 200;
l4 = 180;
l5 = 120;
global ml1 ml2 ml3 ml4;
ml1 = 0.28*2;%单位是kg
ml2 = 0.124*2;
ml3 = 0.124*2;
ml4 = 0.28*2;
global Il1 Il2 Il3 Il4;
Il1 = 0.000858771 * 2;
Il2 = 0.000469931 * 2;
Il3 = Il2 * 2;
Il4 = Il1 * 2;

num_data = 20;
L_lib = [];
K_lib = cell(2,6);%创造一个空的2*6的cell数组
for i = 1:2
   for j = 1:6
       K_lib{i,j} = rand(1, num_data);% 给每个元素赋值为一个大小为 21 的随机数数组
   end
end
for i = 1 : 1 : num_data
    theta = 59 + i;
    syms theta1 theta2 theta3 theta4 theta5 pitch;
    syms xa ya xb yb xd yd xc yc xe ye;
    syms xl1 yl1 xl2 yl2 xl3 yl3 xl4 yl4 xp yp Ip;
    %已知量
    theta1 = deg2rad(-theta) + pi;
    theta4 = deg2rad(theta);
    pitch = deg2rad(0);
    %求解未知量
    xa = -l5/2;
    ya = 0;
    xb = -l5/2 + l1*cos(theta1);
    yb = l1*sin(theta1);
    xc = xb + l2*cos(theta2);
    yc = yb + l2*sin(theta2);
    xd = l5/2 + l4*cos(theta4);
    yd = l4*sin(theta4);
    xe = l5/2;
    ye = 0;
    
    eqn1 = xc == xd + l3*cos(theta3);
    eqn2 = yc == yd + l3*sin(theta3);
    
    sol = solve([eqn1, eqn2], [theta2, theta3]);
    
    theta2 = double(simplify(sol.theta2));
    theta3 = double(simplify(sol.theta3));
    %五连杆正解得到足端点
    [xc, yc, u2, u3] = Zjie(theta1, theta4, 0)
    %简化为单杆的质心坐标和转动惯量
    [xp, yp, Ip] = Simplify_model(pitch, theta1, theta4, xa, ya, xb, yb, xc, yc, xd, yd, xe, ye);
    %再接下来是根据上面的进行LQR线性化
    [K, L] = model_LQR(xc, yc, xp, yp, Ip);
    
    for j = 1 : 1 : 12
        K_lib{j, i} = K(j);
    end

    L_lib = [L_lib; 0.001*L];
end
%拟合多项式，Kij和三阶L的关系式
K_coefficient = [];
for i = 1:1:12
    column_K = cell2mat(K_lib(i,:))';
    K_coefficient = [K_coefficient; polyfit(L_lib, column_K, 3)];%polyfit(x, y, n)多项式拟合，注意参数别写反了
end
disp(K_coefficient)


