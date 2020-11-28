%% 3D Multibody-Dynamics Simulation Library
% Peter Racioppo

%% Initialization
clear %all;
close all;
clc;

% Add folders
addpath(genpath('Constraints'));
addpath(genpath('Solids'));
addpath(genpath('Jacobian'));
addpath(genpath('Solver'));

t_0 = 0;                       % Start Time (s)
t_final = 5;                   % End Time (s)

n = 14;                        % Number of Variables
m = 12;                        % Number of Constraints
nb = n/7;                      % Number of Bodies

m1 = 10;                       % Mass of Body 1 (kg)
m2 = 10;                       % Mass of Body 2 (kg)
L1 = 1;                        % Half length of Body 1 (m)
L2 = 1;                        % Half length of Body 2 (m)
I1 = m1*((2*L1)^2)/12;         % Inertia of Body 1 (kg*m^2)
I2 = m2*((2*L2)^2)/12;         % Inertia of Body 2 (kg*m^2)
g = 9.81;                      % Acceleration due to Gravity (m*s^-2)
% Note: Inertias must be taken in local frames

% Mass Matrix
M = zeros(nb*3, nb*3);
M = f_M(1, m1, M);
M = f_M(2, m2, M);

% Inertia Matrix
J = zeros(nb*3, nb*3);
J = f_J(1, [I1; 0; I1], J);
J = f_J(2, [I2; 0; I2], J);

% Orientation Vectors
fi = [1; 0; 0];
gi = [0; 1; 0];
hj = [0; 0; 1];

% Local Vectors
sA0 = [0; 0; 0];
sA1 = [0; L1; 0];
sB1 = [0; -L1; 0];
sB2 = [0; L2; 0];

% Applied Force Vector
Fa = [0; -M(1,1)*9.81; 0; 0; -M(4,4)*9.81; 0];

% Applied Torque Vector
n_p = zeros(nb*3, 1);

%% Parameters Structure

length = [0,L1,L2];
mass = [0, m1, m2];
inertia = [0, I1, I2];

i=0;
while i <= 2
    Params.(['body', num2str(i)]).num = i;
    Params.(['body', num2str(i)]).length = length(i+1);
    Params.(['body', num2str(i)]).mass = mass(i+1);
    Params.(['body', num2str(i)]).inertia = inertia(i+1);
i=i+1;    
end

Params.body0.sA0 = sA0;
Params.body1.sA1 = sA1;
Params.body1.sB1 = sB1;
Params.body2.sB2 = sB2;

Params.Fa = Fa;
Params.n_p = n_p;

Params.M = M;
Params.J = J;

Params.fi = fi;
Params.gi = gi;
Params.hj = hj;

%% Initial Parameters

% Double Pendulum

% Initial Position 1 (suspended from ceiling)
x1 = 0;
y1 = -L1;
z1 = 0;
phi1 = 0;
x2 = L2;
y2 = -2*L1;
z2 = 0;
phi2 = pi/2;

% Initial Position 2 (bipedal robot)
% x1 = L1*cos(phi1-pi/2);
% y1 = L1*sin(phi1-pi/2);
% z1 = 0;
% phi1 = 5*pi/6;
% x2 = 2*L1*cos(pi/3)+L2*cos(phi2-pi/2);
% y2 = 2*L1*sin(pi/3)+L2*sin(phi2-pi/2);
% z2 = 0;
% phi2=pi/3;

r0 = [x1; y1; z1; x2; y2; z2];                     % Initial Position Vector
phi0 = [phi1; phi2];                               % Initial Link Angles
u = [0; 0; 1];                                     % u-vector, in definition of Euler Parameters
p0 = [f_euler(u, phi0(1)); f_euler(u, phi0(2))];   % Initial Euler Parameters

r_p0 = zeros(6,1);                  % Initial Linear Velocities
p_p0 = zeros(nb*4, 1);              % Initial Time Derivatives of Euler Parameters

lambda = zeros(m - nb, 1);          % Initial lambda's (Lagrange Multipliers)
lambdap = zeros(nb, 1);             % Initial lambdap's (2nd set of Lagrange Multipliers)

Yn = [r_p0; p_p0; lambda; lambdap; r0; p0];      % Initial Y (Total State Vector)

%% Solver

% BuildMechanism. Constructs the Jacobian, Gamma, and the force vector, Qa.
[M, J, PHI_r, PHI_p, PHI_p_dp, Gamma, Gamma_p, Fa, n_p] = BuildMechanism(Yn, n, m, Params);

% f_DAE0. Builds and solves a system of algebraic-differential equations to give the
% derivative of the state-vector for any given time.
f_DAE = @(t_0, Yn)f_DAE0(t_0, Yn, n, m, Params, PHI_r, PHI_p, PHI_p_dp, Gamma, Gamma_p);

% Evalconstraints. Constructs the constraint vector.
f_Evalconstraints = @(t_0, Yn)Evalconstraints(Yn, n, m, Params);

%NumericalAnalysis. Solves for position, velocity, and acceleration for each unit of time.
[T, Y, PHI_T, qpp] = NumericalAnalysis(f_DAE, f_Evalconstraints, Yn, t_final, n, m);

%% Plots
% 
% % Norm of the constraints
% figure;
% plot(T, PHI_T);
% xlabel('Time (s)', 'FontSize', 20);
% ylabel('Norm of PHI (m)', 'FontSize', 20);
% title('Norm of the Constraint Vector', 'FontSize', 20);
% set(gca, 'FontSize', 15);
% grid on;
% 
% % Trajectories
% figure;
% plot(Y(:, 27), Y(:, 28), 'b');
% hold on;
% plot(Y(:, 30), Y(:, 31), 'r');
% xlabel('X Position (m)', 'FontSize', 20);
% ylabel('Y Position (m)', 'FontSize', 20);
% title('Trajectories of CGs', 'FontSize', 20);
% legend('CG 1', 'CG 2');
% set(gca, 'FontSize', 15);
% grid on;
% 
% % Positions vs Time
% figure;
% plot(T, Y(:, 27), 'b');
% hold on;
% plot(T, Y(:, 28), 'r');
% plot(T, Y(:, 30), 'g');
% plot(T, Y(:, 31), 'black');
% xlabel('Time (s)', 'FontSize', 20);
% ylabel('Displacement (m)', 'FontSize', 20);
% title('X & Y Displacements vs Time', 'FontSize', 20);
% legend('X,CG1', 'Y,CG1', 'X,CG2', 'Y,CG2');
% set(gca, 'FontSize', 15);
% grid on;
% 
% % Euler Parameters vs Time
% figure;
% plot(T, Y(:, 33), 'b');
% hold on;
% plot(T, Y(:, 8), 'r');
% plot(T, Y(:, 9), 'g');
% plot(T, Y(:, 10), 'black');
% plot(T, Y(:, 11), '-.b');
% plot(T, Y(:, 12), '-.r');
% plot(T, Y(:, 13), '-.g');
% plot(T, Y(:, 14), '-.black');
% xlabel('Time (s)', 'FontSize', 20);
% ylabel('Phi (rad)', 'FontSize', 20);
% title('Euler Parameters vs Time (m)', 'FontSize', 20);
% legend('e0, Link1','e1, Link1','e2, Link1','e3, Link1','e0, Link2','e1, Link2','e2, Link2','e3, Link2');
% set(gca, 'FontSize', 15);
% grid on;
% 
% % Velocities vs Time
% figure;
% plot(T, Y(:, 1), 'b');
% hold on;
% plot(T, Y(:, 2), 'r');
% plot(T, Y(:, 4), 'g');
% plot(T, Y(:, 5), 'black');
% xlabel('Time (s)', 'FontSize', 20);
% ylabel('Velocity (m/s)', 'FontSize', 20);
% title('X & Y Velocities vs Time', 'FontSize', 20);
% legend('X,CG1', 'Y,CG1', 'X,CG2', 'Y,CG2');
% set(gca, 'FontSize', 15);
% grid on;
% 
% % Time Derivative of Euler Parameters vs Time
% figure;
% plot(T, Y(:, 7), 'b');
% hold on;
% plot(T, Y(:, 8), 'r');
% plot(T, Y(:, 9), 'g');
% plot(T, Y(:, 10), 'black');
% plot(T, Y(:, 11), '-.b');
% plot(T, Y(:, 12), '-.r');
% plot(T, Y(:, 13), '-.g');
% plot(T, Y(:, 14), '-.black');
% xlabel('Time (s)', 'FontSize', 20);
% ylabel('Velocity (m/s)', 'FontSize', 20);
% title('Euler Parameter Velocities vs Time', 'FontSize', 20);
% legend('e0, Link1','e1, Link1','e2, Link1','e3, Link1','e0, Link2','e1, Link2','e2, Link2','e3, Link2');
% set(gca, 'FontSize', 15);
% grid on;
% 
% % Accelerations vs Time
% figure;
% plot(T, qpp(:, 1), 'b');
% hold on;
% plot(T, qpp(:, 2), 'r');
% plot(T, qpp(:, 4), 'g');
% plot(T, qpp(:, 5), 'black');
% xlabel('Time (s)', 'FontSize', 20);
% ylabel('Acceleration (m/s^2)', 'FontSize', 20);
% title('X & Y Accelerations vs Time', 'FontSize', 20);
% legend('X,CG1', 'Y,CG1', 'X,CG2', 'Y,CG2');
% set(gca, 'FontSize', 15);
% grid on;
% 
% % Angular Accelerations vs Time
% figure;
% plot(T, qpp(:, 7), 'b');
% hold on;
% plot(T, Y(:, 8), 'r');
% plot(T, Y(:, 9), 'g');
% plot(T, Y(:, 10), 'black');
% plot(T, Y(:, 11), '-.b');
% plot(T, Y(:, 12), '-.r');
% plot(T, Y(:, 13), '-.g');
% plot(T, Y(:, 14), '-.black');
% xlabel('Time (s)', 'FontSize',20);
% ylabel('Accel (m/s^2)', 'FontSize', 20);
% title('Euler Parameter Accelerations vs Time', 'FontSize', 20);
% legend('e0, Link1','e1, Link1','e2, Link1','e3, Link1','e0, Link2','e1, Link2','e2, Link2','e3, Link2');
% set(gca, 'FontSize', 15);
% grid on;

animate_pendulum(T,Y)
