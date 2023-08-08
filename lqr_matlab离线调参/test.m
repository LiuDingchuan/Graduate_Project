clc 
clear all

H1=[1,0];
Pk_1=[1,0;0,1];
C1=H1*Pk_1*H1'

%% 时间序列设定
delta_t=0.01;   
t=0:delta_t:30;
N = length(t); 
sz=[2,N];
X=zeros(sz);
Z=zeros(sz);

%%  矩阵的设定
A=[1 delta_t;0 1];
H=[1 0;0 1];
w1=1*randn(2,N);
w2=1*randn(2,N);
X=[0;1];
%% 实际信号和测量信号
for i=2:N
    X(:,i)=A*X(:,i-1)+w1(:,i-1);
    Z(:,i)=H*X(:,i)+w2(:,i);
end
plot(t,X(1,:),'g')
hold on
plot(t,Z(1,:),'r')
%% 噪声
Q=[0.1 0;0 0.1];
R=[0.1 0;0 0.1];
n=size(Q);
m=size(R);
%% 观测值数据初始化
xhat=zeros(sz);
xhatminus=zeros(sz);
P=zeros(n);
Pminus=zeros(n);
K=zeros(n(1),m(1));
I=eye(n);
xhat=[2;0];
P=[1 0;0 1];
%% 卡尔曼滤波�
for k=2:N
    xhatminus(:,k)=A*xhat(:,k-1);
    Pminus=A*P*A'+Q;
    % 滤波
    K = Pminus*H'*inv( H*Pminus*H'+R );
    xhat(:,k)=xhatminus(:,k)+K*(Z(:,k)-H*xhatminus(:,k));
    P=(I-K*H)*Pminus;
end
figure
plot(t,X(1,:),'g')
hold on
plot(t,Z(1,:),'r')
hold on 
plot(t,xhat(1,:),'b','linewidth',0.5)
figure
plot(t,X(2,:),'g')
hold on
plot(t,Z(2,:),'r')
hold on 
plot(t,xhat(2,:),'b','linewidth',0.5)