data = load('data2.dat');
t = data(:,1);
TorqueL = data(:,2);
TorqueR = data(:,3);
velocity_now = data(:, 4);
velocity_set = data(:, 5);
theta0 = data(:,6);
theta0_dot = data(:, 7);
pitch = data(:, 8);
pitch_dot = data(:, 9);
roll = data(:, 10);
roll_set = data(:, 11);
yaw_dot = data(:, 12);
yaw_dset = data(:, 13);
L0_L = data(:, 14);
L0_R = data(:, 15);
theta0_L = data(:, 16);
theta0_R = data(:, 17);
L0_L_set = data(:, 18);
L0_R_set = data(:, 19);

figure;
subplot(3,3,1);
plot(t, TorqueL, t, TorqueR);
legend("Torque Left" ,"Torque Right");
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

subplot(3,3,2);
plot(t, TorqueR);
legend("Torque Right")
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

subplot(3,3,3);
ylim([0 2]);
plot(t, velocity_now, 'b', t, velocity_set, 'r');
legend('v_{now}', 'v_{set}');
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

subplot(3, 3, 4);
plot(t, theta0_L, t, theta0_R);
legend("theta0_L", "theta0_R");
xlabel("t(s)");
ylabel("theta0(rad)");
grid on;

subplot(3, 3, 5);
plot(t, pitch);
legend("pitch");
xlabel("t(s)");
ylabel("pitch(rad)");
grid on;

subplot(3, 3, 6);
plot(t, roll, t, roll_set);
legend("roll", "roll_{set}");
xlabel("t(s)");
ylabel("roll(rad)");
grid on;

subplot(3, 3, 7);
plot(t, yaw_dot, t, yaw_dset);
legend("yaw_{dot}", "yaw_{dset}");
xlabel("t(s)");
ylabel("yaw dot(rad/s)");
grid on;

% subplot(3, 3, 8);
% plot(t, L0_L*1000, t, L0_L*1000);
% legend("L0 L","L0_R");
% xlabel("t(s)");
% ylabel("L0 L(mm)");
% grid on;

subplot(3, 3, 8);
plot(t, L0_L*1000, t, L0_L_set*1000);
legend("L0 L","L0 Lset");
xlabel("t(s)");
ylabel("L0(mm)");
grid on;

subplot(3, 3, 9);
plot(t, L0_R*1000, t, L0_R_set*1000);
legend("L0 R","L0 Rset");
xlabel("t(s)");
ylabel("L0(mm)");
grid on;

