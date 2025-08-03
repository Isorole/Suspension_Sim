clc; clear; close all;

% Parameters
m_s = 250;  % Sprung Mass (kg)
m_u = 40;   % Unsprung Mass (kg)
k_s = 15000; % Suspension Stiffness (N/m)
c_s = 1200;  % Suspension Damping (Ns/m)
k_t = 200000; % Tire Stiffness (N/m)

% State-Space Model
A = [0 1 0 0;
    -k_s/m_s -c_s/m_s k_s/m_s c_s/m_s;
    0 0 0 1;
    k_s/m_u c_s/m_u (-k_s-k_t)/m_u -c_s/m_u];

B = [0; 0; 0; k_t/m_u];

C = eye(4); % Output all states
D = zeros(4,1);

% Create state-space system
sys = ss(A,B,C,D);

% Time Simulation
t = 0:0.01:5; % 5 seconds simulation
road_input = zeros(size(t));
road_input(t >= 1) = 0.05; % 5 cm bump at 1 sec

% Simulate System Response
[y,t,x] = lsim(sys, road_input, t);

% Plot Results
figure;
subplot(2,1,1);
plot(t, x(:,1), 'r', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Chassis Displacement (m)');
title('Suspension Response - Chassis');

subplot(2,1,2);
plot(t, x(:,3), 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Wheel Displacement (m)');
title('Suspension Response - Wheel');
