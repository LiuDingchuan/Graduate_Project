plot(data.t, data.pitch+0.0064);
xlabel("t(s)");
ylabel("pitch(rad)");
grid on;

%%
plot(data.t, data.L_speed);
xlabel("t(s)");
ylabel("velocity(m/s)");
grid on;
%%
plot(data.t, data.robot_x);
xlabel("t(s)");
ylabel("positionX(m)");
grid on;
%%
plot(data.t, data.L_Torque);
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;
