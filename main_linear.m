%% Utilities

clc
clear
close all

%% Initialization of Variables for Simulink

clk_in = clock;

disp('defining variables for simulation through simulink')

% Simulation time
T = 20;

% Desired values -> we want that the link is standing still
qd = [pi/3; -pi/7; pi/4];
qd_dot = [0; 0; 0];
qd_2dot = [0; 0; 0];
qd_3dot = [0; 0; 0];
qd_4dot = [0; 0; 0];

% Initial conditions for the integrators for q and theta variables
q0 = qd;
theta0 = q0;
qd0 = [0; 0; 0];
thetad0 = [0; 0; 0];

% External force parameters acting from T1 to T2 inside T.
% F is acting along the orthogonal direction wrt the link where it acts
T1 = 2;
T2 = 3;
link = 3;
switch link
    case 1
        F = 1000*[-sin(q0(1)); cos(q0(1))];
    case 2
        F = 1000*[-sin(q0(1)+q0(2)); cos(q0(1)+q0(2))];
    case 3
        F = 1000*[-sin(q0(1)+q0(2)+q0(3)); cos(q0(1)+q0(2)+q0(3))];
end

% Inertia parameters 
m = 1;      % mass of the link
mm = 1;     % mass of the motor
d = 0.5;    % distance of the CoM
I = 3.33;   % inertia of the link
g0 = 9.81;  % gravity term

param = [m mm d I g0]';

% Parameters of matrices
d_par = 500;
kp_par = 800;
kd_par = 500;

% Damping matrix
D = diag([d_par d_par d_par]);

% PD parameters
Kp = diag([kp_par kp_par kp_par]);
Kd = diag([kd_par kd_par kd_par]);

% Motor parameters 
m_m = 1;        % motor mass
I_m = 0.01;     % motor inertia
k = 500;        % constant for transmission stiffness
k_new = 250;    % desired transmission stiffness
k_r = 9;        % reduction ratio

motor = [I_m*k_r^2, k, k_new];

% Inertia matrix motor
B = diag([I_m*k_r^2,I_m*k_r^2,I_m*k_r^2]);
invB = inv(B);


%% ESP control simulation

disp('simulating using ESP control');

out = sim('ESP_linear_stiff');

t_ESP = out.tout;
dim = max(size(t_ESP));

qd_vec = [qd(1)*ones([1 dim]); qd(2)*ones([1 dim]); qd(3)*ones([1 dim]);]';

q_ESP_sim = out.q;
q_ESP = zeros(dim, 3);

q_ESP(:, 1) = q_ESP_sim(1, 1, :);
q_ESP(:, 2) = q_ESP_sim(2, 1, :);
q_ESP(:, 3) = q_ESP_sim(3, 1, :);

q_dot_ESP_sim = out.q_dot;
q_dot_ESP = zeros(dim, 3);

q_dot_ESP(:, 1) = q_dot_ESP_sim(1, 1, :);
q_dot_ESP(:, 2) = q_dot_ESP_sim(2, 1, :);
q_dot_ESP(:, 3) = q_dot_ESP_sim(3, 1, :);

q_2dot_ESP_sim = out.q_2dot;
q_2dot_ESP = zeros(dim, 3);

q_2dot_ESP(:, 1) = q_2dot_ESP_sim(1, 1, :);
q_2dot_ESP(:, 2) = q_2dot_ESP_sim(2, 1, :);
q_2dot_ESP(:, 3) = q_2dot_ESP_sim(3, 1, :);

u_ESP = out.u;

%% ESP+ control simulation

disp('simulating using ESP+ control');

out2 = sim('ESPp_linear_stiff');

t_ESPp = out2.tout;
dim2 = max(size(t_ESPp));

qd_vec2 = [qd(1)*ones([1 dim2]); qd(2)*ones([1 dim2]); qd(3)*ones([1 dim2]);]';

q_ESPp_sim = out2.q;
q_ESPp = zeros(dim2, 3);

q_ESPp(:, 1) = q_ESPp_sim(1, 1, :);
q_ESPp(:, 2) = q_ESPp_sim(2, 1, :);
q_ESPp(:, 3) = q_ESPp_sim(3, 1, :);

q_dot_ESPp_sim = out2.q_dot;
q_dot_ESPp = zeros(dim2, 3);

q_dot_ESPp(:, 1) = q_dot_ESPp_sim(1, 1, :);
q_dot_ESPp(:, 2) = q_dot_ESPp_sim(2, 1, :);
q_dot_ESPp(:, 3) = q_dot_ESPp_sim(3, 1, :);

q_2dot_ESPp_sim = out2.q_2dot;
q_2dot_ESPp = zeros(dim2, 3);

q_2dot_ESPp(:, 1) = q_2dot_ESPp_sim(1, 1, :);
q_2dot_ESPp(:, 2) = q_2dot_ESPp_sim(2, 1, :);
q_2dot_ESPp(:, 3) = q_2dot_ESPp_sim(3, 1, :);

u_ESPp = out2.u;

save('Workspace');

%% Saving variables for CoppeliaSim

% ESP control
ESP_ID = fopen('ESP.txt','w');

for i=1:dim
    q1 = q_ESP(i,1); q2 = q_ESP(i,2); q3 = q_ESP(i,3);
    dq1 = q_dot_ESP(i,1); dq2 = q_dot_ESP(i,2); dq3 = q_dot_ESP(i,3);
    fprintf(ESP_ID,'%f \t %f \t %f \t %f \t %f \t %f \n',q1,q2,q3,dq1,dq2,dq3);
end

% ESPp control
ESPp_ID = fopen('ESPp.txt','w');

for i=1:dim
    q1 = q_ESPp(i,1); q2 = q_ESPp(i,2); q3 = q_ESPp(i,3);
    dq1 = q_dot_ESPp(i,1); dq2 = q_dot_ESPp(i,2); dq3 = q_dot_ESPp(i,3);
    fprintf(ESPp_ID,'%f \t %f \t %f \t %f \t %f \t %f \n',q1,q2,q3,dq1,dq2,dq3);
end

fclose(ESPp_ID);

%% Plots

fclose(ESP_ID);

figure('name', 'Q PROFILES')

subplot(121)
plot(t_ESP, q_ESP - qd_vec, 'linewidth', 2); hold on; grid;
title('\textbf{$\tilde{q}$ profile with ESP}','interpreter', 'latex');
ylabel('\textbf{$\tilde{q}$ [rad]} ','interpreter', 'latex');
xlabel('time [s]');
legend('\textbf{$\tilde{q}_1$}', '\textbf{$\tilde{q}_2$}', '\textbf{$\tilde{q}_3$}', 'location', 'northeast','interpreter', 'latex');

subplot(122)
plot(t_ESPp, q_ESPp - qd_vec2, 'linewidth', 2); hold on; grid;
title('\textbf{$\tilde{q}$ profile with ESP+}','interpreter', 'latex');
ylabel('\textbf{$\tilde{q}$ [rad]} ','interpreter', 'latex');
xlabel('time [s]');
legend('\textbf{$\tilde{q}_1$}', '\textbf{$\tilde{q}_2$}', '\textbf{$\tilde{q}_3$}', 'location', 'northeast','interpreter', 'latex');

set(gcf, 'Position', [100 100 1000 500]);

saveas(gcf, 'q_tilda', 'fig');
saveas(gcf, 'q_tilda', 'epsc');


figure('name', 'Q_DOT PROFILES');

subplot(121)
plot(t_ESP, q_dot_ESP, 'linewidth', 2); hold on; grid;
title('$\dot{q}$ profile with ESP','interpreter', 'latex');
ylabel('$\dot{q}$ [rad/s]','interpreter', 'latex');
xlabel('time [s]');
legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$', 'location', 'northeast','interpreter', 'latex');

subplot(122)
plot(t_ESPp, q_dot_ESPp, 'linewidth', 2); hold on; grid;
title('$\dot{q}$ profile with ESP+','interpreter', 'latex');
ylabel('$\dot{q}$ [rad/s]','interpreter', 'latex');
xlabel('time [s]');
legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$', 'location', 'northeast','interpreter', 'latex');

set(gcf, 'Position', [100 100 1000 500]);

saveas(gcf, 'q_dot', 'fig');
saveas(gcf, 'q_dot', 'epsc');


figure('name', 'Q_2DOT PROFILES');

subplot(121)
plot(t_ESP, q_2dot_ESP, 'linewidth', 2); hold on; grid;
title('$\ddot{q}$ profile with ESP','interpreter', 'latex');
ylabel('$\ddot{q} \ [rad/s^2]$','interpreter', 'latex');
xlabel('time [s]');
legend('$\ddot{q}_1$', '$\ddot{q}_2$', '$\ddot{q}_3$', 'location', 'northeast','interpreter', 'latex');

subplot(122)
plot(t_ESPp, q_2dot_ESPp, 'linewidth', 2); hold on; grid;
title('$\ddot{q}$ profile with ESP+','interpreter', 'latex');
ylabel('$\ddot{q} \ [rad/s^2]$','interpreter', 'latex');
xlabel('time [s]');
legend('$\ddot{q}_1$', '$\ddot{q}_2$', '$\ddot{q}_3$', 'location', 'northeast','interpreter', 'latex');

set(gcf, 'Position', [100 100 1000 500]);


saveas(gcf, 'q_2dot', 'fig');
saveas(gcf, 'q_2dot', 'epsc');


figure('name', 'U PROFILES');

subplot(121)
plot(t_ESP, u_ESP, 'linewidth', 2); hold on; grid;
title('u profile with ESP');
ylabel('u [Nm]');
xlabel('time [s]');
legend('u_1', 'u_2', 'u_3', 'location', 'northeast');

subplot(122)
plot(t_ESPp, u_ESPp, 'linewidth', 2); hold on; grid;
title('u profile with ESP+');
ylabel('u [Nm]');
xlabel('time [s]');
legend('u_1', 'u_2', 'u_3', 'location', 'northeast');

set(gcf, 'Position', [100 100 1000 500]);

saveas(gcf, 'u', 'fig');
saveas(gcf, 'u', 'epsc');

clk_fn = clock;
execution_time = clk_fn - clk_in;
execution_time(4:6)