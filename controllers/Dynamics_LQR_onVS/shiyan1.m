data = load('data2.dat');
t = data(:,1);
TorqueL = data(:,2);
TorqueR = data(:,3);

figure;
subplot(2,1,1);
plot(t, TorqueL);
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;
subplot(2,1,2);
plot(t, TorqueR);
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

