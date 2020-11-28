%% 2D Multibody Dynamic Simulation of a Hex Thing
% Peter Racioppo

%% Initializations
close all;
%clear all;
clc

tic  % Starts Timer, for counting computational cost

% Add paths
addpath(genpath('Constraints'));
addpath(genpath('Solids'));
addpath(genpath('f_DAE0'));
addpath(genpath('Jacobian'));

% Time
t = 0;         % Initial Time
t_final = 10;   % End Time

% Penalty Formulation? (n = 0, y = 1)
PF = 0;

% Coordinate and Constraint Definitions
nb = 8;         % Number of bodies
n = nb*3;       % Number of generalized coordinates
m = nb*2+2+3;   % Number of constraints

% Slider-Crank Parameters
r = 0.5;    % Crank Length
l = 1;      % Coupler Length
f = 5;      % Frequency (Hz)
SC = [r;l;f];

% Model Parameters
g = 9.81;                       % Gravitational Acceleration
L1 = 1;                         % Length 1
L2 = 1;                         % Length 2
L3 = 1;                         % Length 3
L4 = 1;                         % Length 4
L5 = 1;                         % Length 5
L6 = 1;                         % Length 6
L7 = 1;                         % Length 7
L8 = 1;                         % Length 8
LU = 0.33;                      % Length of uppermost mass
LSx = 0.1;                      % x-length, side mass
LSy = 0.1;                      % y-length, side mass
mass1 = 1;                      % Mass 1
mass2 = 1;                      % Mass 2
mass3 = 1;                      % Mass 3
mass4 = 1;                      % Mass 4
mass5 = 1;                      % Mass 5
mass6 = 1;                      % Mass 6
mass7 = 1;                      % Mass 7
mass8 = 1;                      % Mass 8
Inertia1 = mass1*((2*L1)^2)/3;  % Inertia 1
Inertia2 = mass2*((2*L2)^2)/3;  % Inertia 2
Inertia3 = mass3*((2*L3)^2)/3;  % Inertia 3
Inertia4 = mass4*((2*L4)^2)/3;  % Inertia 4
Inertia5 = mass5*((2*L5)^2)/3;  % Inertia 5
Inertia6 = mass6*((2*L6)^2)/3;  % Inertia 6
Inertia7 = mass7*((2*L7)^2)/3;  % Inertia 7
Inertia8 = mass8*((2*L8)^2)/3;  % Inertia 8

% Body Indices
body1 = 1;
body2 = 2;
body3 = 3;
body4 = 4;
body5 = 5;
body6 = 6;
body7 = 7;
body8 = 8;

% Constructs the mass matrix M
M = eye(n,n);
M = f_mass(body1,mass1,Inertia1,M);
M = f_mass(body2,mass2,Inertia2,M);
M = f_mass(body3,mass3,Inertia3,M);
M = f_mass(body4,mass4,Inertia4,M);
M = f_mass(body5,mass5,Inertia5,M);
M = f_mass(body6,mass6,Inertia6,M);
M = f_mass(body7,mass7,Inertia7,M);
M = f_mass(body8,mass8,Inertia8,M);

% Local Position Vectors
sA1 = [0; L1];   % Point A from frame 1
sA2 = [0;-L2];   % Point A from frame 2
sB2 = [0; L2];   % Point B from frame 2
sB3 = [0;-L3];   % Point B from frame 3
sC3 = [0; L3];   % Point C from frame 3
sC4 = [LU/2;-L4];   % Point C from frame 4
sD4 = [LU/2; L4];   % Point D from frame 4
sD5 = [0;-L5];   % Point D from frame 5
sE5 = [0; L5];   % Point E from frame 5
sE6 = [0;-L6];   % Point E from frame 6
sF6 = [0; L6];   % Point F from frame 6
sF1 = [0;-L1];   % Point F from frame 1
sB7 = [0; L7];   % Point B from frame 7
sG7 = [0;-L7];   % Point G from frame 7
sE8 = [0; L8];   % Point E from frame 8
sG8 = [0;-L8];   % Point G from frame 8

s = [sA1,sA2,sB2,sB3,sC3,sC4,sD4,sD5,sE5,sE6,sF6,sF1,sB7,sG7,sE8,sG8]; % Concatenation of local vectors, for passing to functions

% Translational Joint Vectors
V11 = [0;1];      
V12 = [0;1];      

v = [V11,V12];  % Concatenation of vectors, for passing to functions

%% Initial Conditions

% Initial Link Orientations
phi1_0 = 90*(pi/180);
phi2_0 = 30*(pi/180);
phi3_0 = -30*(pi/180);
phi4_0 = (90+180)*(pi/180);
phi5_0 = (30+180)*(pi/180);
phi6_0 = (-30+180)*(pi/180);
phi7_0 = 90*(pi/180);
phi8_0 = -90*(pi/180);

% Rotation Matrices
A1 = f_RM(phi1_0);
A2 = f_RM(phi2_0);
A3 = f_RM(phi3_0);
A4 = f_RM(phi4_0);
A5 = f_RM(phi5_0);
A6 = f_RM(phi6_0);
A7 = f_RM(phi7_0);
A8 = f_RM(phi8_0);

% Initial Positions
%r1 = [0;0];  % Starting from zero
r1 = [0;1];  % Starting from full extension
r2 = r1+A1*sA1+A2*(-sA2);
r3 = r2+A2*sB2+A3*(-sB3);
r4 = r3+A3*sC3+A4*(-sC4);
r5 = r4+A4*sD4+A5*(-sD5);
r6 = r5+A5*sE5+A6*(-sE6);
r7 = r2+A2*sB2+A7*(-sB7);
r8 = r5+A5*sE5+A8*(-sE8);

% Initial Position Vector
q = [r1;phi1_0;...
     r2;phi2_0;...
     r3;phi3_0;...
     r4;phi4_0;...
     r5;phi5_0;...
     r6;phi6_0;...
     r7;phi7_0;...
     r8;phi8_0];

% Initial Velocity Vector
qp = zeros(n,1);         % Initial velocity set to zero vector
%v0 = [0; 0; 0];         % Initial angular velocities
%qp = InitV(v0,s,n,m,q); % Initial velocity

lambda = zeros(m,1);    %Initial lambda

% Initial State vector
Yi = [qp;lambda;q];

%% Solver
% Build Mechanism. Constructs the Jacobian, Gamma, and the global force vector, Qa
[M,PHIq,Gamma] = BuildMechanism(Yi,n,m,M,s,v,g,t,SC);

% f_DAE0. Builds and solves the algebraic-differential equation to give the
% derivative of the state-vector for any given time.
f_DAE = @(t,Yn)f_DAE0(t,Yn,Yi,M,PHIq,Gamma,n,m,s,v,g,PF,SC);

%  Evalconstraints. Constructs the constraint vector.
f_Evalconstraints = @(t,Yn)Evalconstraints(Yn,n,m,s);

% NumericalAnalysis. Solves for position, velocity, and acceleration for
% each unit of time.
[T,Y,PHI_T,qpp] = NumericalAnalysis(f_DAE,f_Evalconstraints,Yi,t_final,n,m);

%% Coordinate Definitions

q  = Y(:, n+m+1:n+m+n);  % Position Components of Y
qp = Y(:, 1:n);          % Velocity Components of Y

%% Horizontal Difference & FFT

xLS = q(:,1+3*(body7-1))-L7; % X position, left joint
xRS = q(:,1+3*(body8-1))+L8; % X position, right joint
Diff = xRS-xLS; % X difference

% FFT Setup
Sig = Diff;  % Signal in the time domain

% Cutting out badly behaved data
% (ie transients and instabilities)
% And Normalizing the signal
start = 572;  % Starting element of T (the time vector)
stop = 4368;  % Ending element of T
TimePeriod = T(stop)-T(start);  % Total time of well-behaved interval
Sig1 = Sig(start:stop);  % Signal over well-behaved interval
SigSize = max(Sig1)-min(Sig1);  % Max-min value
ZeroPoint = min(Sig1)+SigSize/2;  % Starts mid-point at zero
Sig1 = (Sig1-ZeroPoint)/(SigSize/2);  % Normalized signal
T1 = T(start:stop)-T(start);  % Time vector over well-behaved interval

%Fs = 1000;          % Test
Fs = length(Sig1);  % Frequency
Tfft = TimePeriod/Fs;  % Total time
LSig = Fs;  % Length of signal
nT = 2^nextpow2(LSig);  % Padding with zeros to length n, to improve accuracy
%tfft = (0:LSig-1)*Tfft;  % Time vector, for use in test case
%TestSig = cos(2*pi*50*tfft);  % Test

% Attempt 1
Yfft = fft(Sig1,nT);  % FFT
%Yfft = fft(TestSig,nT);  % Test
f_fft = Fs*(0:(nT/2))/(nT*TimePeriod);  % Frequency Range
P_fft = abs(Yfft/nT);  % Vector of Magnitudes
PN_fft = P_fft(1:nT/2+1);  % Magnitudes, removing padded zeros

% % Attempt 2 (didn't work)
% Fsvec = (0:(Fs/nT):(Fs/2-Fs/nT));
% dim = 2;
% Yfft = fft(Sig1,nT,dim);
% %Yfft = fft(TestSig,nT,dim);  % Test
% P_fft = abs(Yfft/nT);
% P2_fft = P_fft(:,1:nT/2+1);
% P2_fft(:,2:end-1) = 2*P2_fft(:,2:end-1);
% PN_fft = P2_fft(1,1:nT/2);

% % Attempt 3 (inaccurate)
% %Yfft = fft(TestSig);  % Test
% Yfft = fft(Sig1);
% mag = abs(Yfft);
% P_fft = angle(Yfft);
% freq = (0:length(Yfft)-1)*50/length(Yfft);

%% Plots

% % Animation of the Hexagon Thing
% Animate_Hex(T,Y,n,m,s,LU,LSx,LSy)

% % Norm of the constraints
% figure
% plot(PHI_T(:,1),PHI_T(:,2),'black');
% hold on;
% xlabel('Time (s)','FontSize',15);
% ylabel('Norm of Phi (m)','FontSize',15);
% title('Norm of the Constraints vs. Time','FontSize',15);
% set(gca, 'FontSize',15);
% grid on;
% hold on;

% Plot of Horizontal Difference
figure
%plot(T,Sig,'b');  % Plot of signal
plot(T1,Sig1,'b');  % Plot of signal, in good interval and normalized
%plot(tfft(1:length(tfft)),Sig1(1:length(tfft)));  % Test
%plot(tfft(1:100),TestSig(1:100));  % Test
xlabel('Time','FontSize',15);
ylabel('Diff','FontSize',15);
title('X Diff vs Time','FontSize',15);
set(gca, 'FontSize',15);
%axis equal;
grid on;
%xlim([0 0.1]);

% Plot of FFT
figure
plot(f_fft,PN_fft);  % Attempt 1
%plot(Fsvec,PN_fft);  % Attempt 2
%plot(freq,mag);      % Attempt 3
xlabel('Freq (Hz)','FontSize',15);
ylabel('|P(f)|','FontSize',15);
title('FFT','FontSize',15);
set(gca, 'FontSize',15);
%axis equal;
grid on;
xlim([0 10]);

% ------------------------------------------

%   % Trajectories
%   figure
%   plot(q(:,1+3*(body1-1)),q(:,2+3*(body1-1)),'b');
%   hold on;
%   plot(q(:,1+3*(body2-1)),q(:,2+3*(body2-1)),'r');
%   plot(q(:,1+3*(body3-1)),q(:,2+3*(body3-1)),'g');
%   plot(q(:,1+3*(body4-1)),q(:,2+3*(body4-1)),'m');
%   plot(q(:,1+3*(body5-1)),q(:,2+3*(body5-1)),'y');
%   plot(q(:,1+3*(body6-1)),q(:,2+3*(body6-1)),'black');
%   xlabel('X Position (m)','FontSize',15);
%   ylabel('Y Position (m)','FontSize',15);
%   title('Trajectories of CGs','FontSize',15);
%   legend('Body1','Body2','Body3','Body4','Body5','Body6');
%   set(gca, 'FontSize',15);
%   axis equal;
%   grid on;

%  % Positions vs Time
%   figure
%   plot(T,q(:,1+3*(body1-1)),'b');
%   hold on;
%   plot(T,q(:,2+3*(body1-1)),'r');
%   plot(T,q(:,1+3*(body2-1)),'g');
%   plot(T,q(:,2+3*(body2-1)),'m');
%   plot(T,q(:,1+3*(body3-1)), 'y');
%   plot(T,q(:,2+3*(body3-1)), 'black');
%   plot(T,q(:,1+3*(body4-1)), '--b');
%   plot(T,q(:,2+3*(body4-1)), '--r');
%   plot(T,q(:,1+3*(body5-1)), '--g');
%   plot(T,q(:,2+3*(body5-1)), '--m');
%   plot(T,q(:,1+3*(body6-1)), '--y');
%   plot(T,q(:,2+3*(body6-1)), '--black');
%   xlabel('Time (s)','FontSize',15);
%   ylabel('X and Y Positions (m)','FontSize',15);
%   title('X and Y Positions of Bodies vs. Time','FontSize',15);
%   legend('X1','Y1','X2','Y2','X3','Y3','X4','Y4','X5','Y5','X6','Y6');
%   set(gca, 'FontSize',15);
%   grid on;

%   % Link Angles vs Time
%   figure
%   plot(T, q(:,3+3*(body1-1)),'b');
%   hold on;
%   plot(T, q(:,3+3*(body2-1)),'r');
%   plot(T, q(:,3+3*(body3-1)),'g');
%   plot(T, q(:,3+3*(body4-1)),'m');
%   plot(T, q(:,3+3*(body5-1)),'y');
%   plot(T, q(:,3+3*(body6-1)),'black');
%   xlabel('Time (s)','FontSize',15);
%   ylabel('Angle (Radians)','FontSize',15);
%   title('Link Angles vs. Time','FontSize',15);
%   legend('Phi1','Phi2','Phi3','Phi4','Phi5','Phi6');
%   set(gca, 'FontSize',15);
%   grid on;

%   % Velocities vs Time
%   figure
%   plot(T,qp(:,1+3*(body1-1)),'b');
%   hold on;
%   plot(T,qp(:,2+3*(body1-1)),'r');
%   plot(T,qp(:,1+3*(body2-1)),'g');
%   plot(T,qp(:,2+3*(body2-1)),'black');
%   plot(T,qp(:,1+3*(body3-1)), 'y');
%   plot(T,qp(:,2+3*(body3-1)), 'm');
%   plot(T,qp(:,1+3*(body4-1)),'b');
%   plot(T,qp(:,2+3*(body4-1)),'--r');
%   plot(T,qp(:,1+3*(body5-1)),'--g');
%   plot(T,qp(:,2+3*(body5-1)),'--black');
%   plot(T,qp(:,1+3*(body6-1)), '--y');
%   plot(T,qp(:,2+3*(body6-1)), '--m');
%   xlabel('Time (s)','FontSize',15);
%   ylabel('X and Y Velocities (m/s)','FontSize',15);
%   title('X and Y Velocities of Bodies vs. Time','FontSize',15);
%   legend('X1','Y1','X2','Y2','X3','Y3','X4','Y4','X5','Y5','X6','Y6');
%   set(gca, 'FontSize',15);
%   grid on;

%   % Angular Velocities vs Time
%   figure
%   plot(T, qp(:,3+3*(body1-1)),'b');
%   hold on;
%   plot(T, qp(:,3+3*(body2-1)),'r');
%   plot(T, qp(:,3+3*(body3-1)),'g');
%   plot(T, qp(:,3+3*(body4-1)),'m');
%   plot(T, qp(:,3+3*(body5-1)),'y');
%   plot(T, qp(:,3+3*(body6-1)),'black');
%   xlabel('Time (s)','FontSize',15);
%   ylabel('Angular Velocity (Hz)','FontSize',15);
%   title('Angular Velocities of Bodies vs. Time','FontSize',15);
%   legend('Phi1','Phi2','Phi3','Phi4','Phi5','Phi6');
%   set(gca, 'FontSize',15);
%   grid on;

%   % Accelerations vs Time
%   figure;
%   plot(T, qpp(:, 1), 'b');
%   hold on;
%   plot(T, qpp(:, 2), 'r');
%   plot(T, qpp(:, 4), 'g');
%   plot(T, qpp(:, 5), 'm');
%   plot(T, qpp(:, 7), 'y');
%   plot(T, qpp(:, 8), 'black');
%   plot(T, qpp(:, 10), '--b');
%   plot(T, qpp(:, 11), '--r');
%   plot(T, qpp(:, 13), '--g');
%   plot(T, qpp(:, 14), '--m');
%   plot(T, qpp(:, 16), '--y');
%   plot(T, qpp(:, 17), '--black');
%   xlabel('Time t in (s)');
%   ylabel('Linear Acceleration (m/s^2)');
%   title('Linear Accelerations of Bodies vs. Time');
%   legend('X1','Y1','X2','Y2','X3','Y3','X4','Y4','X5','Y5','X6','Y6');
%   grid on;

%   % Angular Accelerations vs Time
%   figure;
%   plot(T, qpp(:, 3), 'b');
%   hold on;
%   plot(T, qpp(:, 6), 'r');
%   plot(T, qpp(:, 9), 'g');
%   plot(T, qpp(:, 12), 'm');
%   plot(T, qpp(:, 15), 'y');
%   plot(T, qpp(:, 18), 'black');
%   xlabel('Time (s)');
%   ylabel('Angle (1/s^2)');
%   title('Angular Acceleration');
%   legend('Phi1','Phi2','Phi3','Phi4','Phi5','Phi6');
%   grid on;

toc  % Stops Timer
