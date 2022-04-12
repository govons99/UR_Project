%% Utilities
clc
clear
close all

%% Initialization of Variables for Simulink

clk_in = clock;

disp('defining variables for simulation through simulink')

% simulation time
T = 10;

% desired values -> we want that the link is standing still
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

% external force parameters acting from T1 to T2 inside T.
% F is acting along the orthogonal direction wrt the link where it acts
T1 = 2;
T2 = 3;
link = 2;
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

% Computation of the damping matrix
D = diag([500 500 500]);

% PD parameters
Kp = diag([800 800 800]);
Kd = diag([500 500 500]);

% Fixed Point Iteration parameters
eta0 = 0;
eps = 1e-15;
max_it = 100;

fpi_params = [eta0 eps max_it];

% Motor parameters 
m_m = 1;    % motor masse
I_m = 0.01; % motor inertia
k = 1000;   % constant for transmission stiffness
k_r = 18;   % [??]

motor = [I_m*k_r^2, k];

% Inertia matrix motor
B = diag([I_m*k_r^2,I_m*k_r^2,I_m*k_r^2]);
invB = inv(B);

%% ESP control simulation

disp('simulating using ESP control');

out = sim('planar_3R_u_ESP');

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

%% ESP+ constrol simulation

disp('simulating using ESP+ control');

out2 = sim('planar_3R_u_ESPp');

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

%% PD control simulation

disp('simulating using simple PD control');

out3 = sim('planar_3R_u_PD');

t_PD = out3.tout;
dim3 = max(size(t_PD));

qd_vec3 = [qd(1)*ones([1 dim3]); qd(2)*ones([1 dim3]); qd(3)*ones([1 dim3]);]';

q_PD_sim = out3.q;
q_PD = zeros(dim3, 3);

q_PD(:, 1) = q_PD_sim(1, 1, :);
q_PD(:, 2) = q_PD_sim(2, 1, :);
q_PD(:, 3) = q_PD_sim(3, 1, :);

q_dot_PD_sim = out3.q_dot;
q_dot_PD = zeros(dim3, 3);

q_dot_PD(:, 1) = q_dot_PD_sim(1, 1, :);
q_dot_PD(:, 2) = q_dot_PD_sim(2, 1, :);
q_dot_PD(:, 3) = q_dot_PD_sim(3, 1, :);

q_2dot_PD_sim = out3.q_2dot;
q_2dot_PD = zeros(dim3, 3);

q_2dot_PD(:, 1) = q_2dot_PD_sim(1, 1, :);
q_2dot_PD(:, 2) = q_2dot_PD_sim(2, 1, :);
q_2dot_PD(:, 3) = q_2dot_PD_sim(3, 1, :);

u_PD = out3.u;

save('Workspace');

%robot_video;

%% Saving variables for CoppeliaSim

% ESP control
ESP_ID = fopen('ESP.txt','w');

for i=1:dim
    q1 = q_ESP(i,1); q2 = q_ESP(i,2); q3 = q_ESP(i,3);
    dq1 = q_dot_ESP(i,1); dq2 = q_dot_ESP(i,2); dq3 = q_dot_ESP(i,3);
    fprintf(ESP_ID,'%f \t %f \t %f \t %f \t %f \t %f \n',q1,q2,q3,dq1,dq2,dq3);
end

fclose(ESP_ID);

% ESPp control
ESPp_ID = fopen('ESPp.txt','w');

for i=1:dim
    q1 = q_ESPp(i,1); q2 = q_ESPp(i,2); q3 = q_ESPp(i,3);
    dq1 = q_dot_ESPp(i,1); dq2 = q_dot_ESPp(i,2); dq3 = q_dot_ESPp(i,3);
    fprintf(ESPp_ID,'%f \t %f \t %f \t %f \t %f \t %f \n',q1,q2,q3,dq1,dq2,dq3);
end

fclose(ESPp_ID);

% PD control
PD_ID = fopen('PD.txt','w');

for i=1:dim
    q1 = q_PD(i,1); q2 = q_PD(i,2); q3 = q_PD(i,3);
    dq1 = q_dot_PD(i,1); dq2 = q_dot_PD(i,2); dq3 = q_dot_PD(i,3);
    fprintf(PD_ID,'%f \t %f \t %f \t %f \t %f \t %f \n',q1,q2,q3,dq1,dq2,dq3);
end

fclose(PD_ID);


%% Plots

figure('name', 'Q PROFILES')

subplot(131)
plot(t_ESP, q_ESP - qd_vec, 'linewidth', 2); hold on; grid;
t1 = title('\textbf{$\tilde{q}$ profile with ESP}'); set(t1, 'interpreter', 'latex');
y1 = ylabel('\textbf{$\tilde{q}$ [rad]} '); set(y1, 'interpreter', 'latex');
xlabel('time [s]');
l1 = legend('\textbf{$\tilde{q}_1$}', '\textbf{$\tilde{q}_2$}', '\textbf{$\tilde{q}_3$}', 'location', 'northeast');
set(l1, 'interpreter', 'latex');

subplot(132)
plot(t_ESPp, q_ESPp - qd_vec2, 'linewidth', 2); hold on; grid;
t2 = title('\textbf{$\tilde{q}$ profile with ESP}'); set(t2, 'interpreter', 'latex');
y2 = ylabel('\textbf{$\tilde{q}$ [rad]} '); set(y2, 'interpreter', 'latex');
xlabel('time [s]');
l2 = legend('\textbf{$\tilde{q}_1$}', '\textbf{$\tilde{q}_2$}', '\textbf{$\tilde{q}_3$}', 'location', 'northeast');
set(l2, 'interpreter', 'latex');

subplot(133)
plot(t_PD, q_PD - qd_vec3, 'linewidth', 2); hold on; grid;
t3 = title('\textbf{$\tilde{q}$ profile with PD}'); set(t3, 'interpreter', 'latex');
y3 = ylabel('\textbf{$\tilde{q}$ [rad]} '); set(y3, 'interpreter', 'latex');
xlabel('time [s]');
l3 = legend('\textbf{$\tilde{q}_1$}', '\textbf{$\tilde{q}_2$}', '\textbf{$\tilde{q}_3$}', 'location', 'northeast');
set(l3, 'interpreter', 'latex');


figure('name', 'Q_DOT PROFILES');

subplot(131)
plot(t_ESP, q_dot_ESP, 'linewidth', 2); hold on; grid;
title('q_{dot} profile with ESP');
ylabel('q_{dot} [rad/s]');
xlabel('time [s]');
legend('q_{1,dot}', 'q_{2,dot}', 'q_{3,dot}', 'location', 'northeast');

subplot(132)
plot(t_ESPp, q_dot_ESPp, 'linewidth', 2); hold on; grid;
title('q_{dot} profile with ESP+');
ylabel('q_{dot} [rad/s]');
xlabel('time [s]');
legend('q_{1,dot}', 'q_{2,dot}', 'q_{3,dot}', 'location', 'northeast');

subplot(133)
plot(t_PD, q_dot_PD, 'linewidth', 2); hold on; grid;
title('q_{dot} profile with PD');
ylabel('q_{dot} [rad/s]');
xlabel('time [s]');
legend('q_{1,dot}', 'q_{2,dot}', 'q_{3,dot}', 'location', 'northeast');


figure('name', 'Q_2DOT PROFILES');

subplot(131)
plot(t_ESP, q_2dot_ESP, 'linewidth', 2); hold on; grid;
title('q_{2dot} profile with ESP');
ylabel('q_{2dot} [rad/s^2]');
xlabel('time [s]');
legend('q_{1,2dot}', 'q_{2,2dot}', 'q_{3,2dot}', 'location', 'northeast');

subplot(132)
plot(t_ESPp, q_2dot_ESPp, 'linewidth', 2); hold on; grid;
title('q_{2dot} profile with ESP+');
ylabel('q_{2dot} [rad/s^2]');
xlabel('time [s]');
legend('q_{1,2dot}', 'q_{2,2dot}', 'q_{3,2dot}', 'location', 'northeast');

subplot(133)
plot(t_PD, q_2dot_PD, 'linewidth', 2); hold on; grid;
title('q_{2dot} profile with PD');
ylabel('q_{2dot} [rad/s^2]');
xlabel('time [s]');
legend('q_{1,2dot}', 'q_{2,2dot}', 'q_{3,2dot}', 'location', 'northeast');


figure('name', 'U PROFILES');

subplot(131)
plot(t_ESP, u_ESP, 'linewidth', 2); hold on; grid;
title('u profile with ESP');
ylabel('u [Nm]');
xlabel('time [s]');
legend('u_1', 'u_2', 'u_3', 'location', 'northeast');

subplot(132)
plot(t_ESPp, u_ESPp, 'linewidth', 2); hold on; grid;
title('u profile with ESP+');
ylabel('u [Nm]');
xlabel('time [s]');
legend('u_1', 'u_2', 'u_3', 'location', 'northeast');

subplot(133)
plot(t_PD, u_PD, 'linewidth', 2); hold on; grid;
title('u profile with PD');
ylabel('u [Nm]');
xlabel('time [s]');
legend('u_1', 'u_2', 'u_3', 'location', 'northeast');

clk_fn = clock;
execution_time = clk_fn - clk_in