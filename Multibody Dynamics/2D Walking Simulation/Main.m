%% Spider Robot %%
% (8 links, Under-Actuated) %
% (with model data taken from SolidWorks) %
% Peter Racioppo, 2017 %

%% -------- INITIALIZATIONS -------- %%

close all;
%clear all;
% clc

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
t_f = 20;  % End Time
% NOTE: start time = zero, by default

% Coordinates & Constraints
nb = 8;            % Number of bodies
n = nb*3;          % Number of generalized coordinates
m = (nb-1)*2 + 5;  % Number of constraints
% (There are (nb-1)*2 revolute constraints)
% (& 5 more if extra algebraic constraints on the link angles are included)

% ---------------- %

% Model Parameters

% Lengths, masses, & inertias

% MP1: Head connected base compartment, base forward
% MP2: Head connected to end connector; head forward
% MP#: Head connected to end connector; head backward
% [LVec,MVec,IVec] = f_ModelParameters1(nb);
[LVec,MVec,IVec] = f_ModelParameters2(nb);

L1 = LVec(1); L2 = LVec(2); L3 = LVec(3); L4 = LVec(4); L5 = LVec(5); L6 = LVec(6); L7 = LVec(7); L8 = LVec(8);
g = 9.81;  % Gravitational acceleration constant

MaxAng = 40*(pi/180);  % Maximum allowed angle between links 1 & 2
Pradius1 = 6/1000;     % Pulley radius 1 (smallest)
Pradius2 = 11/1000;    % Pulley radius 2
Pradius3 = 16/1000;    % Pulley radius 3 (largest)

LinkRadius = 35/1000;  % Radius of exterior of rotator links

MechData = [Pradius1,Pradius2,Pradius3,LinkRadius,0];  % Mechanism Data

% Di = distance between wire termination point on (i-1)th link
% and connection point on ith link
D1 = sqrt(L2^2+LinkRadius^2);
D2 = sqrt(L2^2+LinkRadius^2);
D3 = sqrt(0.9*L2^2+LinkRadius^2);

T1 = Pradius1/D1;
T2 = (Pradius2/D2-Pradius1/D1);
T3 = (Pradius3/D3-Pradius2/D2);

% Angle Constraints, specified as ratios with angle 1
% b/a = theta_2 / theta_1
% c/a = theta_3 / theta_1
a = 1;
b = a*(T2/T1);
c = a*(T3/T1);

% % switch a and c, since motor module is inside and connector is outside
% a = c;
% c = 1;

Ratio = [a,b,c,T1,T2,T3];

% Spring Constants
Cka = 0;       % Torsional Spring
Ck1 = 1838.8;  % Linear Spring 1
Ck2 = 1838.8;  % Linear Spring 2
Ck3 = 1838.8;  % Linear Spring 3

% Ck2 = (Pradius2/Pradius1)*Ck1;  % Linear Spring 2
% Ck3 = (Pradius3/Pradius1)*Ck1;  % Linear Spring 3
Ck = 1*[Ck1,Ck2,Ck3,Cka];         % Vector of spring constants

time2steps = 0.03;  % Approx number of seconds per time-step in ODE solver

% ---------------- %

% Constructs the mass matrix, M
M = eye(n,n);
ii = 1;
while ii <= nb
    M = f_mass(ii,MVec(ii),IVec(ii),M);
    ii = ii+1;
end

% ---------------- %

% Local Position Vectors
sO0 = [0;   0];   % Point O from frame 0
sA1 = [0;  L1];   % Point A from frame 1
sB1 = [0; -L1];   % Point B from frame 1
sB2 = [0;  L2];   % Point B from frame 2
sC2 = [0; -L2];   % Point C from frame 2
sC3 = [0;  L3];   % Point C from frame 3
sD3 = [0; -L3];   % Point D from frame 3
sD4 = [0;  L4];   % Point D from frame 4
sE4 = [0; -L4];   % Point E from frame 4
sE5 = [0;  L5];   % Point E from frame 5
sF5 = [0; -L5];   % Point F from frame 5
sF6 = [0;  L6];   % Point F from frame 6
sG6 = [0; -L6];   % Point G from frame 6
sG7 = [0;  L7];   % Point G from frame 7
sH7 = [0; -L7];   % Point H from frame 7
sH8 = [0;  L8];   % Point H from frame 8
sI8 = [0; -L8];   % Point I from frame 8
s = [sO0,sA1,sB1,sB2,sC2,sC3,sD3,sD4,sE4,sE5,sF5,sF6,sG6,sG7,sH7,sH8,sI8];  % Concatenation of local vectors, for passing to functions

% ---------------- %

% Cable Lengths at Equilibrium
CableLength0 = InitialCableLengths(LVec,s,MechData,Ratio);
MechData(6) = CableLength0(1); MechData(7) = CableLength0(2); MechData(8) = CableLength0(3);

% ---------------- %

% Penalty Formulation? (for more complex systems)
PF = 0;  % (n = 0, y = 1)

%% -------- CONTROL -------- %%

% Control Parameters
Angle = 30;               % First joint angle
DoA = 1;                 % Direction of Approach of snake to first path
                         % DoA = 1 or -1
CatchDistance = 2*L1*nb;  % In case of missed way point, distance to continue
                           % before switching to next way point
Amp = 0.32;               % Total Amplitude of Actuation Force
% Amplitude of sine wave:
% A = 0.5;  % High Friction
A = 1;      % Low friction
w = 5;                   % Angular Frequency
offset = 0*(pi/180);     % Phase offset
P = 1; %0.8              % Proportional Gain
D = 1; %0.4              % Derivative Gain
Delta = 7*L1; %24        % Look-ahead Distance
                         % NOTE: There evidently exists some minimum Delta,
                           % DeltaMin, such that the steering is unstable
                           % for Delta < DeltaMin.
Ah = 0.4; %0.4 %0.2      % Heading Multiplier
Ph = 1;                  % Proportional Heading Gain
Dh = 0*Ph;               % Derivative Heading Gain
Ih = 0*0.005;            % Integral Heading Gain
MaxTorque = Amp;         % Max-torque which the motors can supply
WPtol = 3.5;               % Way Point Tolerance
AF = 1;                  % Defines the extent to which the feedforward
                         % controller accounts for friction
AK = 1;                  % Defines the extent to which the feedforward
                         % controller accounts for elasticity
                         % NOTE: The values of AF & AK equate to:
                           % 0: No Feedforward, <1 Undercompensating,
                           % 1: Perfectly Compensating, >1: Overcompensating
SmallAngle = 0;          % Determines whether to use small angle approx
                           % in the feedforward controller (0 = n, 1 = y)
                           % NOTE: control values must evidently be retuned
                           % if this is used (which isn't currently done)
Controller = 2;          % Controller Type. 1 = Link Angle Average;
                         % 2 = Angle Sum; 3 = Amplitude
Mode = 3;                % Mode: 1 = Stand, 2 = Lay, 3 = Walk

% Defines vector to hold control data
Control = [Amp;A;w;offset;P;D;Delta;Ah;Ph;Dh;Ih;MaxTorque;AF;AK;SmallAngle;Angle;Mode];

%% -------- WAY POINTS -------- %%

% Way Points

% Original WPs (triangular)
rw1 = [0;0];
rw2 = [0;10]; % [0;-50]
rw3 = [5;0]; % [50;0]
rw4 = [0;0]; % [15;0]

% WayPts vector, for passing to functions
WayPts = [rw1,rw2,rw3,rw4];

dummy = WayPts(:,end)+[1;1];
WayPts = [WayPts,dummy];

% ---------------- %

% Angle between way points
WPang = zeros(length(WayPts+1),1);
i = 1;
while i < length(WayPts)
    WPang(i+1) = DoA*f_WPangle(WayPts(:,i),WayPts(:,i+1));
    i = i+1;
end

% Determining the shortest direction to turn to go to next waypoint
% (i.e. turn through the acute angle)
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
q = f_InitialPosition(Angle,LVec,rw1,Ratio);

% Initial Velocity
%qp = zeros(n,1);        % Initial velocity set to zero vector
v0 = zeros(nb-4,1);      % Initial angular velocities
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

% Sets Global Torque (control inputs)
Torquevec0 = [0,0];
setGlobalTorque(1,Torquevec0);

%% -------- SOLVER -------- %%
% Builds and Solves the Matrix DAE for the System.
% Stops, updates initial conditions, and restarts at each waypoint.

Go = 1;    % (1=go, 0=stop)
Te = 0;    % Start time
WPNum = 0; % 0th Way Point
LWP = length(WayPts);

Ye1 = 0; % Dummy Ye1

T = 0;
Y = [];
PHI_T = [];
qpp = [];

counter = 1;
while counter <= length(WayPts)

% Solves the system  
if isempty(Ye1) == 0
    % If last waypoint was reached
    [T1,Y1,Te1,Ye1,PHI_T1,qpp1] = Solver(t_f-Te,Te,Yi1,M,LVec,n,m,s,g,time2steps,Control,Fr,PF,WayPts(:,counter),WayPts(:,counter+1),Ck,Disturb,Noise,WPtol,Go,WPang,WPNum,CatchDistance,MaxAng,MechData,Ratio,Controller);
else
    % If last waypoint was missed
    [T1,Y1,Te1,Ye1,PHI_T1,qpp1] = f_Replace(T1,Y1,Te1,Ye1,PHI_T1,qpp1);
end

T = [T;T(end)+T1];
Y = [Y;Y1];
PHI_T = [PHI_T;PHI_T1];
qpp = [qpp;qpp1];

WPNum = WPNum+1;
counter = counter+1;

% Defines the time and state of the last event as the new initial conditions
Te = Te+Te1;           % Start time
Yi1 = transpose(Ye1);  % Initial position

if counter < length(WayPts) - 1
    Go = 1;
else
    Go = 0;
end

end

T(1) = [];

%% -------- Coordinate Definitions -------- %%

% Coordinates
%q  = Y(:, n+nb*2-2+1:n+nb*2-2+n);  % Position Components of Y
q = Y(:, n+m+1:end);
qp = Y(:, 1:n);                     % Velocity Components of Y

% Prints total travel time
Tend = T(end);
if t_f ~= 1
    TimeofTravel = sprintf('Total Travel Time: %s seconds',num2str(Tend));
else
    TimeofTravel = sprintf('Total Travel Time: %s second',num2str(Tend));
end
disp(TimeofTravel);

% Buffer (for animation and trajectory plot)
% (Amount of extra-space on plot, from WayPt furthest from origin)
% buffer = 2*L*(nb+2); % i.e. A bit more than one body length of extra space on all sides.
buffer = L1*(nb+4); % i.e. A bit more than one body length of extra space on all sides.

toc % Stops timer after compuations are finished

%% -------- PLOTS -------- %%

% Animation of the Snake Robot
Animate_Tripod(T,Y,n,m,s,WayPts,WPtol,buffer,FricType,'b');  % Only Links

% % Trajectory Plot
%Trajectory(T,q,L1,WayPts,WPtol,buffer/2,'b');

% Relative Angles, for the 2 modules
% RelAngles(T,q,MaxAng);

% Motor Torques
% PlotMotorTorque(Tend,getGlobalTorque);

 %Cable Displacements
% PlotCableDisplacement(T,Y,n,m,s,MechData);

% Cable Displacement Differences
% (The difference between cable displacement on the left side
%  and right sides. Since the two sides are connected, a nonzero
%  value indicates that the cable length is not sufficient to
%  allow a particular motion and introduces nonlinear behaviour.
%  Springs to allow cable flexibility and feedforward control
%  terms are added to compensate.)
% PlotCableDisplacementDifferences(T,Y,n,m,s,MechData);

% % Norm of the constraints
% % (Indicates fidelity of simulation, as DAE solver drifts over time)
% NormofConstraints(PHI_T);

% % Poincare Map
% figure;
% plot3(q(:,3),q(:,6),q(:,9));
% xlabel('{\it\theta_{1}}','FontSize',20,'FontName','Times New Roman');
% ylabel('{\it\theta_{2}}','FontSize',20,'FontName','Times New Roman');
% zlabel('{\it\theta_{3}}','FontSize',20,'FontName','Times New Roman');

% -----------------------------------

% Other Plots

% % Position, Velocity, & Acceleration Plots
% % 1.) X Positions vs Time
% % 2.) Y Positions vs Time
% % 3.) Link Angles vs Time
% % 4.) X Velocities vs Time
% % 5.) Y Velocities vs Time
% % 6.) Angular Velocities vs Time
% % 7.) X Accelerations vs Time
% % 8.) Y Accelerations vs Time
% % 9.) Angular Acclerations vs Time
% PlotActivation = [0,0,0,0,0,0,0,0,0];  % (0=off, 1=on)
% PosVelAccel(T,q,qp,qpp,PlotActivation);

% % Heading (link angle avg.) vs Time
% Heading(n,T,q);

% % % Average Y Velocity (for straight, forward motion)
% AvgVel(n,T,qp);

% -----------------------------------

% beep

% save('File1','T','Y','q')
% 
% Var1 = load('File1.mat');
% Var2 = load('File2.mat');
% 
% T1 = Var1.T;
% T2 = Var2.T;
% Y1 = Var1.Y;
% Y2 = Var2.Y;
% q1 = Var1.q;
% q2 = Var2.q;
% 
% figure;
% Animate_Snake(T1(length(T1)),Y1(length(Y1),:),n,m,s,WayPts,WPtol,buffer,FricType,'b');
% hold on;
% Animate_Snake(T2(length(T2)),Y2(length(Y2),:),n,m,s,WayPts,WPtol,buffer,FricType,'r');
% hold on;
% Trajectory(T1,q1,L1,WayPts,WPtol,buffer/2,'b');
% hold on;
% Trajectory(T2,q2,L1,WayPts,WPtol,buffer/2,'r');
% % xlabel('{\itx}-position','FontSize',38,'FontName','Times New Roman');
% % ylabel('{\ity}-position','FontSize',38,'FontName','Times New Roman');
% hold on;
