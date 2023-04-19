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

figure;
subplot(7,1,1);
plot(t, TorqueL);
legend("Torque Left");
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

subplot(7,1,2);
plot(t, TorqueR);
legend("Torque Right")
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

subplot(7,1,3);
ylim([0 2]);
plot(t, velocity_now, 'b', t, velocity_set, 'r');
legend('v_{now}', 'v_{set}');
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

subplot(7, 1, 4);
plot(t, theta0);
legend("theta0");
xlabel("t(s)");
ylabel("theta0");
grid on;

subplot(7, 1, 5);
plot(t, theta0_dot);
legend("theta0 dot");
xlabel("t(s)");
ylabel("theta0 dot");
grid on;

subplot(7, 1, 6);
plot(t, pitch);
legend("pitch");
xlabel("t(s)");
ylabel("pitch");
grid on;

subplot(7, 1, 7);
plot(t, pitch_dot);
legend("pitch dot");
xlabel("t(s)");
ylabel("pitch dot");
grid on;

