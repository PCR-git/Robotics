%% 2D Multibody Dynamic Simulation of a Snake Robot (6 links) %%
% Peter Racioppo

%% -------- INITIALIZATIONS -------- %%

close all;
%clear all;
clc

% ---------------- %

tic  % Starts Timer, for counting computational cost

% Add paths
addpath(genpath('Initializations'));
addpath(genpath('Constraints'));
addpath(genpath('Solids'));
addpath(genpath('Jacobian'));
addpath(genpath('Friction'));
addpath(genpath('Control'));
addpath(genpath('Solver'));
addpath(genpath('Plots'));
addpath(genpath('Global'));
addpath(genpath('Symbolics'));

%% Model Parameters

% Time
t_f = 175;   % End Time
% NOTE: start time = zero, by default

% Coordinates & Constraints
nb = 6;         % Number of bodies
n = nb*3;       % Number of generalized coordinates
m = (nb-1)*2;   % Number of constraints

% ---------------- %

% Model Parameters
L = 1;     % Link Length
mass = 1;  % Link Mass
% Lengths, masses, & inertias
[LVec,MVec,IVec] = f_ModelParameters(nb,L,mass);
L1 = LVec(1); L2 = LVec(2); L3 = LVec(3); L4 = LVec(4); L5 = LVec(5); L6 = LVec(6);
g = 9.81;   % Gravitational Acceleration
Ck = 0.5; %0  % Torsional Spring Constant
time2steps = 0.03;  % Approx number of seconds per time-step in ODE solver

% ---------------- %

% Constructs the mass matrix M
M = eye(n,n);
ii = 1;
while ii <= nb
    M = f_mass(ii,MVec(ii),IVec(ii),M);
    ii = ii+1;
end

% ---------------- %

% Local Position Vectors
sO0 = [0;   0];   % Point O from frame 0
sO1 = [0;  L1];   % Point O from frame 1
sA1 = [0; -L1];   % Point A from frame 1
sA2 = [0;  L2];   % Point A from frame 2
sB2 = [0; -L2];   % Point B from frame 2
sB3 = [0;  L3];   % Point B from frame 3
sC3 = [0; -L3];   % Point C from frame 3
sC4 = [0;  L4];   % Point C from frame 4
sD4 = [0; -L4];   % Point D from frame 4
sD5 = [0;  L5];   % Point D from frame 5
sE5 = [0; -L5];   % Point E from frame 5
sE6 = [0;  L6];   % Point E from frame 6
s = [sO0,sO1,sA1,sA2,sB2,sB3,sC3,sC4,sD4,sD5,sE5,sE6]; % Concatenation of local vectors, for passing to functions

% ---------------- %

% Penalty Formulation? (for more complex systems)
PF = 0;  % (n = 0, y = 1)

%% -------- CONTROL -------- %%

% Control Parameters
heading0 = 0;            % Initial Heading, in degrees
DoA = 1;                 % Direction of Approach of snake to first path
                         % DoA = 1 or -1
CatchDistance = 2*L*nb;  % In case of missed way point, distance to continue
                         % before switching to next way point

Amp = 130; %70 %90 %100 %120 % Total Amplitude of Actuation Force
A = 0.4; %1 %0.3 %0.35   % Amplitude of sine wave

w = 2; %2             % Angular Frequency
offset = 0*(pi/180);  % Phase offset
P = 0.8;              % Proportional Gain
D = 1; %0.4 %0.8      % Derivative Gain
Delta = 18*L; %10     % Look-ahead Distance
                      % NOTE: There evidently exists some minimum Delta,
                        % DeltaMin, such that the steering is unstable
                        % for Delta < DeltaMin.
Ah = 0.15; %0.5 %0.2  % Heading Multiplier
Ph = 1;               % Proportional Heading Gain
Dh = 0.0*Ph;          % Derivative Heading Gain
Ih = 0*0.005;         % Integral Heading Gain
MaxTorque = Amp;      % Max-torque which the motors can supply
WPtol = 2;            % Way Point Tolerance
AF = 1;               % Defines the extent to which the feedforward
                        % controller accounts for friction
AK = 1;               % Defines the extent to which the feedforward
                        % controller accounts for elasticity
                        % NOTE: The values of AF & AK equate to:
                        % 0: No Feedforward, <1 Undercompensating,
                        % 1: Perfectly Compensating, >1: Overcompensating
SmallAngle = 0;       % Determines whether to use small angle approx
                        % in the feedforward controller (0 = n, 1 = y)
                        % NOTE: control values must evidently be retuned
                        % if this is used (which isn't currently done)

% Defines vector to hold control data
Control = [Amp;A;w;offset;P;D;Delta;Ah;Ph;Dh;Ih;MaxTorque;AF;AK;SmallAngle];

%% -------- WAY POINTS -------- %%

% Way Points
rw1 = [0;0];
rw2 = [0;100]; % [0;-50]
rw3 = [100;0]; % [50;0]
rw4 = [30;0]; % [15;0]

% WayPts vector, for passing to functions
WayPts = [rw1,rw2,rw3,rw4];

% ---------------- %

% Angle between way points
WPang1 = 0;
WPang2 = DoA*f_WPangle(rw1,rw2);
WPang3 = f_WPangle(rw2,rw3);
WPang4 = f_WPangle(rw3,rw4);
WPang5 = 0;

% Determining the shortest direction to turn to go to next waypoint
% (i.e. turn through the acute angle)
WPang = [WPang1;WPang2;WPang3;WPang4;WPang5];
pp = 1;
while pp < length(WPang)
    if WPang(pp)-WPang(pp+1) > 180
        WPang(pp+1) = 360+WPang(pp+1);
    elseif WPang(pp)-WPang(pp+1) < -180
        WPang(pp+1) = WPang(pp+1)-360;
    end
    pp = pp + 1;
end
WPang(1) = [];
WPang(end) = WPang(end-1);

% ---------------- %

% Way Point Coordinates
wx1 = rw1(1); wy1 = rw1(2);
wx2 = rw2(1); wy2 = rw2(2);
wx3 = rw3(1); wy3 = rw3(2);
wx4 = rw4(1); wy4 = rw4(2);

%% -------- INITIAL CONDITIONS -------- %%

% Initial Position Vector
q = f_InitialPosition(heading0,LVec,rw1);

% Initial Velocity
%qp = zeros(n,1);        % Initial velocity set to zero vector
v0 = zeros(nb,1);        % Initial angular velocities
qp = f_InitV(v0,s,n,m);  % Initial velocity
% Given initial angular velocities, InitV generates velocities
% in all the coordinates which are consistent with the constraints.

%Initial lambda (Lagrange multipliers) (start from zero)
lambda = zeros(m,1);

% Initial State vector
Yi1 = [qp;lambda;q];

%% -------- FRICTION -------- %%

% Friction Type: (1 = Viscous, 2 = Coulomb, 3 = 1+2, 4 = 3+Stribeck Effect)
FricType = 1;

% Friction Coefficients (from experimental measurements)
% (Back friction = bf; Forward friction = ff; Side friction = sf)
[bf,ff,sf] = f_FricCoeff;

% Friction Parameters (guessed)
Cviscous = 25; % Coeff. of Viscous Friction
Ccoulomb = 1;  % Coeff. of Coulomb Friction

% Defines vector to hold friction data
Fr = [bf;ff;sf;FricType;Cviscous;Ccoulomb];

%% -------- Disturbance Forces -------- %%

% Adding Disturbance Force
% Disturb(1) = magnitude;
% Disturb(2) = x component of disturbance, from 0 to 1
% Disturb(3) = y component of disturbance, from 0 to 1
% Disturb(4) = Type: 1 = constant, 2 = sine wave
% Disturb(5) = Begin Time
% Disturb(6) = End Time
Disturb = [0,1,0,1,30,40];

% ---------------- %

% Adding random noise to external forces
NoiseScale = 0;      % Noise Off
%NoiseScale = 1/10;  % Noise Scaling (noise-to-signal)
ONES = ones(n,(t_f*10+1));  % Vector of ones
%Noise = awgn(ONES,10)*NoiseScale;  % Adds Gaussian noise to force vector Q
Noise = 0*ONES;  % Zero Noise
% NOTE: The scalar argument denotes the signal-to-noise ratio.
% NOTE2: Laptop doesn't have Communications System Toolbox, can't read awgn

%% Set Global Variables

% Sets Global Counter Variable i
setGlobali(0);

% Sets Global Cross Track Error
setGlobalCrossTrackError(1,0);

% Sets Global Cross Track Error Integral
setGlobalCrossTrackErrorINT(0);

% Sets Error
setGlobalError(1,[0;0]);

% Sets U Array (control inputs)
%U = cell(1,2);
setGlobalU(1,0,0);

%% -------- SOLVER -------- %%
% Builds and Solves the Matrix DAE for the System.
% Stops, updates initial conditions, and restarts as each waypoint.

% ---------------------- Go from 0 to 1 ---------------------- %
Go = 1;    % (1=go, 0=stop)
Ye0 = 1;   % Dummy variable for event function
Te = 0;    % Start time
WPNum = 0; % 0th Way Point

% Solves the system
[T1,Y1,Te1,Ye1,PHI_T1,qpp1] = Solver(t_f-Te,Te,Yi1,M,LVec,n,m,s,g,time2steps,Control,Fr,PF,rw1,rw2,Ck,Disturb,Noise,WPtol,Go,WPang,WPNum,CatchDistance);

% ---------------------- Go from 1 to 2 ---------------------- %
Go = 1;  % Go? (1=go, 0=stop)

% Defines the time and state of the last event as the new initial conditions
Te = Te+Te1;           % Start time
Yi2 = transpose(Ye1);  % Initial position
WPNum = 1;             % 1st Way Point

% Solves the system
if isempty(Ye1) == 0
    % If last waypoint was reached
    [T2,Y2,Te2,Ye2,PHI_T2,qpp2] = Solver(t_f-Te,Te,Yi2,M,LVec,n,m,s,g,time2steps,Control,Fr,PF,rw2,rw3,Ck,Disturb,Noise,WPtol,Go,WPang,WPNum,CatchDistance);
else
    % If last waypoint was missed
    [T2,Y2,Te2,Ye2,PHI_T2,qpp2] = f_Replace(T1,Y1,Te1,Ye1,PHI_T1,qpp1);
end

% ---------------------- Go from 2 to 3 ---------------------- %
Go = 1;  % Go? (1=go, 0=stop)

% Defines the time and state of the last event as the new initial conditions
Te = Te+Te2;           % Start time
Yi3 = transpose(Ye2);  % Initial position
WPNum = 2;             % 2nd Way Point

% Solves the system
if isempty(Ye2) == 0
    % If last waypoint was reached
    [T3,Y3,Te3,Ye3,PHI_T3,qpp3] = Solver(t_f-Te,Te,Yi3,M,LVec,n,m,s,g,time2steps,Control,Fr,PF,rw3,rw4,Ck,Disturb,Noise,WPtol,Go,WPang,WPNum,CatchDistance);
else
    % If last waypoint was missed
    [T3,Y3,Te3,Ye3,PHI_T3,qpp3] = f_Replace(T2,Y2,Te2,Ye2,PHI_T2,qpp2);
end

% ---------------------- Stop ---------------------- %
Go = 0;  % Go? (1=go, 0=stop)

% Defines the time and state of the last event as the new initial conditions
Te = Te+Te3;              % Start time
Yistop = transpose(Ye3);  % Initial position
WPNum = 3;                % 3rd Way Point

% Solves the system
if isempty(Ye3) == 0
    % If last waypoint was reached
    [Tstop,Ystop,Testop,Yestop,PHI_Tstop,qppstop] = Solver(t_f-Te,Te,Yistop,M,LVec,n,m,s,g,time2steps,Control,Fr,PF,WayPts(:,end-1),WayPts(:,end),Ck,Disturb,Noise,WPtol,Go,WPang,WPNum,CatchDistance);
else
    % If last waypoint was missed
    [Tstop,Ystop,Testop,Yestop,PHI_Tstop,qppstop] = f_Replace(T3,Y3,Te3,Ye3,PHI_T3,qpp3);
end

%% -------- Coordinate Definitions -------- %%

% Defining Time Vector, State Vector, Accel Vector, & PHI_T vector,
% depending on the number of waypoints
[T,Y,PHI_T,qpp] = f_Definitions(Ye1,Ye2,Ye3,T1,T2,T3,Tstop,Y1,Y2,Y3,Ystop,PHI_T1,PHI_T2,PHI_T3,PHI_Tstop,qpp1,qpp2,qpp3,qppstop);

% ----------------------------------------
% Coordinates
q  = Y(:, n+nb*2-2+1:n+nb*2-2+n);  % Position Components of Y
qp = Y(:, 1:n);                    % Velocity Components of Y

% Prints total travel time
Tend = T(end);
TimeofTravel = sprintf('Total Travel Time: %s seconds',num2str(Tend));
disp(TimeofTravel);

% Buffer (for animation and trajectory plot)
% (Amount of extra-space on plot, from WayPt furthest from origin)
buffer = 12;

%% -------- PLOTS -------- %%

% Animation of the Snake Robot
Animate_Snake(T,Y,n,m,WayPts,WPtol,buffer,FricType);

% % Norm of the constraints
% NormofConstraints(PHI_T)

% Trajectory
Trajectory(T,q,L1,WayPts,WPtol,buffer);

% % Position, Velocity, & Acceleration Plots
% % 1.) Positions vs Time
% % 2.) Link Angles vs Time
% % 3.) Velocities vs Time
% % 4.) Angular Velocities vs Time
% % 5.) Accelerations vs Time
% % 6.) Accelerations vs Time
% PlotActivation = [1,1,1,1,1,1];  % (0=off, 1=on)
% PosVelAccel(T,q,qp,qpp,PlotActivation);

% % Heading (link angle avg.) vs Time
% Heading(n,T,q);

% % Average Y Velocity (for straight, forward motion)
% AvgVel(n,T,qp);

% % Errors (for each link)
% Errors(Tend);

% % Control Signals
% ControlSignals(Tend);

% ----------------------------------------

toc  % Stops Timer

