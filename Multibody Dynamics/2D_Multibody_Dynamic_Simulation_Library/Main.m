%% 2D Multibody Dynamics Simulation Library
% Peter Racioppo

%% Initializations
close all;
clear all;
clc

addpath(genpath('Constraints'));
addpath(genpath('Solids'));
addpath(genpath('f_DAE0'));
addpath(genpath('Jacobian'));

%Time
t = 0; % Initial Time
t_final = 10; % End Time

n = 6; % Number of generalized coordinates
nb = n/3;
m = 4; % Number of constraints

% Initial parameters
L1=1;       % Length 1
L2=1;       % Length 2
mass1=10;   % Mass 1
mass2=10;   % Mass 2
Inertia1=mass1*((2*L1)^2)/3;   % Inertia 1
Inertia2=mass2*((2*L2)^2)/3;   % Inertia 2

% Constructs the mass matrix M
M=eye(n,n);
M=f_mass(1,mass1,Inertia1,M);
M=f_mass(2,mass2,Inertia2,M);

% Local Position Vectors
sO0=[0;   0];   % Point O from frame 0
sO1=[0;  L1];   % Point O from frame 1
sA1=[0; -L1];   % Point A from frame 1
sA2=[0;  L2];   % Point A from frame 2
s=[sO0,sO1,sA1,sA2]; % Concatenation of local vectors, for passing to functions

% Initial Position 1 (suspended from ceiling)
%q=[0;-L1;0;L2;-2*L1;pi/2];

% Initial Position 2 (bipedal robot)
phi1=5*pi/6;
x1=L1*cos(phi1-pi/2);
y1=L1*sin(phi1-pi/2);
phi2=pi/3;
x2=2*L1*cos(pi/3)+L2*cos(phi2-pi/2);
y2=2*L1*sin(pi/3)+L2*sin(phi2-pi/2);
q=[x1;y1;phi1;x2;y2;phi2];

%qp=zeros(n,1);
v0=[0;0]; % Initial velocity of the degrees of freedom
qp=InitV(v0,s,n,m,q); %Initial velocity
lambda=zeros(m,1); %Initial lambda

% Initial State vector
Yi=[qp;lambda;q];

%% Solver
% Build Mechanism. Constructs the Jacobian, Gamma, and the force vector, Qa
[M,PHIq,Gamma,Qa] = BuildMechanism(Yi,n,m,M,s);

% f_DAE0. Builds and solves the algebraic-differential equation to give the
% derivative of the state-vector for any given time.
f_DAE = @(t,Yn)f_DAE0(t,Yn,M,PHIq,Gamma,Qa,n,m,s);

%  Evalconstraints. Constructs the constraint vector.
f_Evalconstraints = @(t,Yn)Evalconstraints(Yn,n,m,s);

% NumericalAnalysis. Solves for position, velocity, and acceleration for
% each unit of time.
[T,Y,PHI_T,qpp] = NumericalAnalysis(f_DAE,f_Evalconstraints,Yi,t_final,n,m);


%% Plots

animate_pendulum(T,Y,nb,n)
hold on;

%   % Norm of the constraints
%   figure
%   plot(PHI_T(:,1),PHI_T(:,2),'black');
%   %plot(T,PHI_T,'black');
%   xlabel('Time (s)','FontSize',15);
%   ylabel('Norm of Phi (m)','FontSize',15);
%   title('Norm of the Constraints vs. Time','FontSize',15);
%   set(gca, 'FontSize',15);
%   grid on;
%   
%   % Trajectories
%   figure
%   plot(Y(:,11),Y(:,12),'b');
%   hold on;
%   plot(Y(:,14),Y(:,15),'r');
%   xlabel('X Position (m)','FontSize',15);
%   ylabel('Y Position (m)','FontSize',15);
%   title('Trajectories of CGs','FontSize',15);
%   legend('CG 1','CG 2');
%   set(gca, 'FontSize',15);
%   grid on;
%   
%  % Positions vs Time
%   figure
%   plot(T,Y(:,11),'b');
%   hold on;
%   plot(T,Y(:,12),'r');
%   plot(T,Y(:,14),'g');
%   plot(T,Y(:,15),'black');
%   xlabel('Time (s)','FontSize',15);
%   ylabel('X and Y Positions (m)','FontSize',15);
%   title('X and Y Positions of CGs vs. Time','FontSize',15);
%   legend('X1 Position','Y1 Position','X2 Position','Y2 Position');
%   set(gca, 'FontSize',15);
%   grid on;
%   
%   % Link Angles vs Time
%   figure
%   plot(T,Y(:,13),'b');
%   hold on;
%   plot(T,Y(:,16),'r');
%   xlabel('Time (s)','FontSize',15);
%   ylabel('Angle (Radians)','FontSize',15);
%   title('Link Angles vs. Time','FontSize',15);
%   legend('Phi1','Phi2');
%   set(gca, 'FontSize',15);
%   grid on;
%   
%   % Velocities vs Time
%   figure
%   plot(T,Y(:,1),'b');
%   hold on;
%   plot(T,Y(:,2),'r');
%   plot(T,Y(:,4),'g');
%   plot(T,Y(:,5),'black');
%   xlabel('Time (s)','FontSize',15);
%   ylabel('X and Y Velocities (m/s)','FontSize',15);
%   title('X and Y Velocities of CGs vs. Time','FontSize',15);
%   legend('X1 Vel','Y1 Vel','X2 Vel','Y2 Vel');
%   set(gca, 'FontSize',15);
%   grid on;
%   
%   % Angular Velocities vs Time
%   figure
%   plot(T,Y(:,3),'b');
%   hold on;
%   plot(T,Y(:,6),'r');
%   xlabel('Time (s)','FontSize',15);
%   ylabel('Angular Velocity (Hz)','FontSize',15);
%   title('Angular Velocities of CGs vs. Time','FontSize',15);
%   legend('Phi1 Vel','Phi2 Vel');
%   set(gca, 'FontSize',15);
%   grid on;
% 
%   % Accelerations vs Time
%   figure;
%   plot(T, qpp(:, 1), 'b');
%   hold on;
%   plot(T, qpp(:, 2), 'r');
%   plot(T, qpp(:, 4), 'g');
%   plot(T, qpp(:, 5), 'black');
%   xlabel('Time t in (s)');
%   ylabel('Linear Acceleration (m/s^2)');
%   title('Linear Accelerations of CGs vs. Time');
%   legend('X1 Accel', 'Y1 Accel', 'X2 Accel', 'Y2 Accel');
%   grid on;
%   
%   % Angular Accelerations vs Time
%   figure;
%   plot(T, qpp(:, 3), 'b');
%   hold on;
%   plot(T, qpp(:, 6), 'r');
%   xlabel('Time (s)');
%   ylabel('Angle (1/s^2)');
%   title('Angular Acceleration');
%   legend('Phi1 Accel', 'Phi2 Accel');
%   grid on;
