%% Minimum-Time Coordinated Trajectory for Two Robots
clear all; close all; clc;

%% Path parameters
Pi = [0.4; 0.3]; % Initial point [m]
L = 0.8; % Path length [m]
r = norm(Pi); % Radius of the circle [m]
theta_i = atan2(Pi(2), Pi(1)); % Initial angle [rad]
delta_theta = L/r; % Total angle to traverse [rad]
theta_f = theta_i + delta_theta; % Final angle [rad]

%% Robot parameters
% Robot a (RP type)
V_a1_max = 1; % rad/s (joint 1 velocity limit)
V_a2_max = 0.7; % m/s (joint 2 velocity limit)
A_a1_max = 3; % rad/s² (joint 1 acceleration limit)
A_a2_max = 5; % m/s² (joint 2 acceleration limit)

% Robot b (PP type)
V_b_max = 0.6; % m/s (both joints velocity limit)
Ob = [0.7; 0.6]; % Base position of robot b [m]

%% Time-optimal trajectory planning
% We'll use a trapezoidal velocity profile for the path parameter theta

% First, find the maximum possible theta_dot and theta_ddot considering both robots' constraints

% For robot a (RP):
% From kinematics: ra(t) = r (constant)
% Theta_a(t) = theta(t)
% ra_dot = 0
% theta_a_dot = theta_dot
% theta_a_ddot = theta_ddot

% For robot b (PP):
% xb(t) = r*cos(theta(t)) - Ob(1)
% yb(t) = r*sin(theta(t)) - Ob(2)
% xb_dot = -r*sin(theta)*theta_dot
% yb_dot = r*cos(theta)*theta_dot
% xb_ddot = -r*cos(theta)*theta_dot^2 - r*sin(theta)*theta_ddot
% yb_ddot = -r*sin(theta)*theta_dot^2 + r*cos(theta)*theta_ddot

% Velocity constraints:
% For robot a: |theta_dot| ≤ V_a1_max
% For robot b: sqrt(xb_dot^2 + yb_dot^2) = r*theta_dot ≤ V_b_max

% The limiting factor is the minimum of these constraints
max_theta_dot = min(V_a1_max, V_b_max/r); % rad/s

% Acceleration constraints are more complex - we'll use an iterative approach
% to find the minimum time that satisfies all constraints

% Use fminbnd to find the minimum time that satisfies all constraints
options = optimset('TolX',1e-6,'Display','iter');
T_min = fminbnd(@(T) trajectory_cost(T, theta_i, theta_f, r, Ob, V_a1_max, V_a2_max, A_a1_max, A_a2_max, V_b_max),...
                delta_theta/max_theta_dot, 10, options);

% Now compute the optimal trajectory
[~, t, theta, theta_dot, theta_ddot] = trajectory_cost(T_min, theta_i, theta_f, r, Ob, V_a1_max, V_a2_max, A_a1_max, A_a2_max, V_b_max);

%% Compute robot trajectories
% Robot a (RP)
theta_a = theta;
ra = r*ones(size(theta_a));
theta_a_dot = theta_dot;
ra_dot = zeros(size(theta_a_dot));
theta_a_ddot = theta_ddot;
ra_ddot = zeros(size(theta_a_ddot));

% Robot b (PP)
xb = r*cos(theta) - Ob(1);
yb = r*sin(theta) - Ob(2);
xb_dot = -r*sin(theta).*theta_dot;
yb_dot = r*cos(theta).*theta_dot;
xb_ddot = -r*cos(theta).*theta_dot.^2 - r*sin(theta).*theta_ddot;
yb_ddot = -r*sin(theta).*theta_dot.^2 + r*cos(theta).*theta_ddot;

%% Plot results
figure;

% Robot a joint positions
subplot(2,2,1);
plot(t, theta_a, 'b', t, ra, 'r');
title('Robot a: Joint Positions');
xlabel('Time [s]');
ylabel('Position');
legend('\theta_a [rad]', 'r_a [m]');
grid on;

% Robot a joint velocities
subplot(2,2,2);
plot(t, theta_a_dot, 'b', t, ra_dot, 'r');
title('Robot a: Joint Velocities');
xlabel('Time [s]');
ylabel('Velocity [rad/s or m/s]');
legend('\theta_a dot', 'r_a dot');
grid on;

% Robot b joint positions
subplot(2,2,3);
plot(t, xb, 'b', t, yb, 'r');
title('Robot b: Joint Positions');
xlabel('Time [s]');
ylabel('Position [m]');
legend('x_b', 'y_b');
grid on;

% Robot b joint velocities
subplot(2,2,4);
plot(t, xb_dot, 'b', t, yb_dot, 'r');
title('Robot b: Joint Velocities');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('x_b dot', 'y_b dot');
grid on;

sgtitle(['Minimum-Time Coordinated Trajectory (T = ', num2str(T_min), ' s)']);

%% Helper function to compute trajectory cost and check constraints
function [cost, t, theta, theta_dot, theta_ddot] = trajectory_cost(T, theta_i, theta_f, r, Ob, V_a1_max, V_a2_max, A_a1_max, A_a2_max, V_b_max)
    delta_theta = theta_f - theta_i;
    
    % Design a trapezoidal velocity profile for theta
    % We'll use maximum acceleration to reach maximum velocity as quickly as possible
    
    % First estimate maximum possible theta_dot and theta_ddot
    max_theta_dot = min(V_a1_max, V_b_max/r);
    max_theta_ddot = A_a1_max; % Robot a's angular acceleration is limiting
    
    % Compute the trapezoidal profile
    [t, theta, theta_dot, theta_ddot] = trapveltraj([theta_i; theta_f], 100, 'EndTime', T, 'MaxVelocity', max_theta_dot, 'Acceleration', max_theta_ddot);
    theta = theta(1,:)';
    theta_dot = theta_dot(1,:)';
    theta_ddot = theta_ddot(1,:)';
    
    % Check constraints for both robots
    % Robot a constraints are automatically satisfied by construction
    
    % Robot b constraints
    xb = r*cos(theta) - Ob(1);
    yb = r*sin(theta) - Ob(2);
    xb_dot = -r*sin(theta).*theta_dot;
    yb_dot = r*cos(theta).*theta_dot;
    xb_ddot = -r*cos(theta).*theta_dot.^2 - r*sin(theta).*theta_ddot;
    yb_ddot = -r*sin(theta).*theta_dot.^2 + r*cos(theta).*theta_ddot;
    
    % Velocity constraints
    b_vel = sqrt(xb_dot.^2 + yb_dot.^2);
    b_vel_violation = max(b_vel - V_b_max);
    
    % Since we limited theta_dot based on V_b_max, this should be zero
    
    % If any constraints are violated, add penalty to cost
    cost = T + 1e6*(max(b_vel_violation, 0));
end