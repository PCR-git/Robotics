% Builds and Solves the Matrix DAE for the System

function [Ypn] = f_DAE0(t,Yn,M,LVec,PHIq,Gamma,n,m,s,g,time2steps,Control,Fr,PF,rwi,rwj,Ck,Disturb,Noise,Go,WPang,WPNum)
%% Initializations

% Increments Global Counter Variable
setGlobali(getGlobali+1);

% Calls Build Mechanism
if t > 0
    [M,PHIq,Gamma] = BuildMechanism(Yn,n,m,M,s);
end

% Length Definitions
L1 = LVec(1);
L2 = LVec(2);
L3 = LVec(3);
L4 = LVec(4);
L5 = LVec(5);
L6 = LVec(6);

% Coordinates and Constraints
nb = n/3;             % Number of bodies
q = Yn((n+m+1):end);  % Position
qp = Yn(1:n);         % Velocity

% Position Definitions
x1 = q(1);  y1 = q(2);  phi1 = q(3);
x2 = q(4);  y2 = q(5);  phi2 = q(6);
x3 = q(7);  y3 = q(8);  phi3 = q(9);
x4 = q(10); y4 = q(11); phi4 = q(12);
x5 = q(13); y5 = q(14); phi5 = q(15);
x6 = q(16); y6 = q(17); phi6 = q(18);

% Velocity Definitions
x1p = qp(1);  y1p = qp(2);  phi1p = qp(3);
x2p = qp(4);  y2p = qp(5);  phi2p = qp(6);
x3p = qp(7);  y3p = qp(8);  phi3p = qp(9);
x4p = qp(10); y4p = qp(11); phi4p = qp(12);
x5p = qp(13); y5p = qp(14); phi5p = qp(15);
x6p = qp(16); y6p = qp(17); phi6p = qp(18);

%% Rotation Matrices (angles in radians)

% Local to Global
R1 = f_RM(phi1);
R2 = f_RM(phi2);
R3 = f_RM(phi3);
R4 = f_RM(phi4);
R5 = f_RM(phi5);
R6 = f_RM(phi6);

% Global to Local
R1T = f_RM(-phi1);
R2T = f_RM(-phi2);
R3T = f_RM(-phi3);
R4T = f_RM(-phi4);
R5T = f_RM(-phi5);
R6T = f_RM(-phi6);

%% Global to Local Definitions

rl1 = R1T*[x1;y1];  % Position vector of center of link 1
rl2 = R2T*[x2;y2];  % Position vector of center of link 2
rl3 = R3T*[x3;y3];  % Position vector of center of link 3
rl4 = R4T*[x4;y4];  % Position vector of center of link 4
rl5 = R5T*[x5;y5];  % Position vector of center of link 5
rl6 = R6T*[x6;y6];  % Position vector of center of link 6
rl = [rl1;rl2;rl3;rl4;rl5;rl6]; % Local Position Vector

rlp1 = R1T*[x1p;y1p];   % Velocity vector of center of link 1
rlp2 = R2T*[x2p;y2p];   % Velocity vector of center of link 2
rlp3 = R3T*[x3p;y3p];   % Velocity vector of center of link 3
rlp4 = R4T*[x4p;y4p];   % Velocity vector of center of link 4
rlp5 = R5T*[x5p;y5p];   % Velocity vector of center of link 5
rlp6 = R6T*[x6p;y6p];   % Velocity vector of center of link 6
rlp = [rlp1;rlp2;rlp3;rlp4;rlp5;rlp6]; % Local Velocity Vector

%% ----------- Forces ----------- %%

%% Elastic Force

Fk1 = -Ck*(phi2-phi1);
Fk2 = -Ck*(phi3-phi2);
Fk3 = -Ck*(phi4-phi3);
Fk4 = -Ck*(phi5-phi4);
Fk5 = -Ck*(phi6-phi5);

Fk = [0;0;-Fk1/2;0;0;Fk1/2-Fk2/2;0;0;Fk2/2-Fk3/2;...
      0;0;Fk3/2-Fk4/2;0;0;Fk4/2-Fk5/2;0;0;Fk5/2];

%% Frictional Forces

% Friction Type
FricType = Fr(4);

% Friction Parameters
Cviscous = Fr(5);
Ccoulomb = Fr(6);

% Calculating the effective velocity of each link due to
% a pure rotation about its center (assumed to be its COM).
VelEff1 = f_EffVel(L1,phi1,phi1p);
VelEff2 = f_EffVel(L2,phi2,phi2p);
VelEff3 = f_EffVel(L3,phi3,phi3p);
VelEff4 = f_EffVel(L4,phi4,phi4p);
VelEff5 = f_EffVel(L5,phi5,phi5p);
VelEff6 = f_EffVel(L6,phi6,phi6p);
VelEff = [VelEff1;VelEff2;VelEff3;VelEff4;VelEff5;VelEff6];

if FricType == 1;  % Viscous Friction
    % Linear Friction
    Ffrlin = f_Viscous(Fr,rlp,Cviscous);

    % Rotational Friction
    Ffrrot = f_Viscous(Fr,VelEff,Cviscous);

elseif FricType == 2;  % Coulomb Friction
    % Nonlinear friction parameters (guessed)
    staticthreshold = 0.01;
    staticratio = Ccoulomb;
    %coulombratio = 0.8;
    coulombratio = staticratio;
    NFP = [staticthreshold,staticratio,coulombratio];

    % Linear Friction
    Ffrlin = f_Coulomb(Fr,rlp,NFP,M,g);
    
    % Rotational Friction
    Ffrrot = f_Coulomb(Fr,VelEff,NFP,M,g);

elseif FricType == 3 || FricType == 4;  % Viscous+Coulomb+Stribeck
    % Nonlinear friction parameters (guessed)
    staticthreshold = 0.01;
    %staticthreshold = 0.3;
    staticratio = Ccoulomb;
    viscousratio = staticratio;
    stribeckratio = staticratio-viscousratio;
    viscousslope = 1;

    % Stribeck Effect Off
    if FricType == 3
        % Nonlinear friction parameters (guessed)
        backthreshold = staticthreshold;
        forwardthreshold = staticthreshold;
        sidethreshold = staticthreshold;
    
    % Stribeck Effect On
    elseif FricType == 4
        % Nonlinear friction parameters (guessed)
        backthreshold = 0.25;
        forwardthreshold = 0.1;
        %forwardthreshold = backthreshold*(ff/bf);
        sidethreshold = 0.15;
        %sidethreshold = backthreshold*(ff/sf);  
    end

    NFP = [staticthreshold,staticratio,stribeckratio,viscousslope,backthreshold,forwardthreshold,sidethreshold];
    
    % Linear Friction
    Ffrlin = f_ViscousCoulombStribeck(Fr,rlp,NFP,M,g);
    
    % Rotational Friction
    Ffrrot = f_ViscousCoulombStribeck(Fr,VelEff,NFP,M,g);
    
else
    error('Wrong Friction Input')
end

% Components of linear friction
Ff1 = Ffrlin(:,1);
Ff2 = Ffrlin(:,2);
Ff3 = Ffrlin(:,3);
Ff4 = Ffrlin(:,4);
Ff5 = Ffrlin(:,5);
Ff6 = Ffrlin(:,6);

% Frictional forces on each link (rotated into global frame)
force1 = R1*Ff1;
force2 = R2*Ff2;
force3 = R3*Ff3;
force4 = R4*Ff4;
force5 = R5*Ff5;
force6 = R6*Ff6;

% Components of rotational friction
Fa1 = Ffrrot(:,1);
Fa2 = Ffrrot(:,2);
Fa3 = Ffrrot(:,3);
Fa4 = Ffrrot(:,4);
Fa5 = Ffrrot(:,5);
Fa6 = Ffrrot(:,6);

% Torques on each link
torque1 = -L1*norm(Fa1)*sign(qp(3));
torque2 = -L2*norm(Fa2)*sign(qp(6));
torque3 = -L3*norm(Fa3)*sign(qp(9));
torque4 = -L4*norm(Fa4)*sign(qp(12));
torque5 = -L5*norm(Fa5)*sign(qp(15));
torque6 = -L6*norm(Fa6)*sign(qp(18));

% Total Frictional Force Vector
Ff = [force1; torque1; force2; torque2; force3; torque3;...
      force4; torque4; force5; torque5; force6; torque6];

%% Actuation

% ---------------

% Control definitions
Amp = Control(1);          % Total amplitude of control signal
A = Control(2);            % Amplitude of sine wave
w = Control(3);            % Angular frequency of sine wave
offset = Control(4);       % Initial offset of sine wave
P = Control(5);            % Proportional Gain
D = Control(6);            % Derivative Gain
Delta = Control(7);        % Delta (denominator in heading controller)
Ah = Control(8);           % Amplitude of heading control term
Ph = Control(9);           % Proportional Gain in heading controller
%Dh = Control(10);         % Derivative Gain in heading controller; Not used
Ih = Control(11);          % Integral Gain in heading controller
MaxTorque = Control(12);   % Max torque which motors can supply
AF = Control(13);          % Defines the extent to which the feedforward
                             % controller accounts for friction
AK = Control(14);          % Defines the extent to which the feedforward
                             % controller accounts for elasticity
SmallAngle = Control(15);  % Determines whether to use small angle approx
                             % in the feedforward controller

% ---------------

% Defining a vector to store vector, for passing into controller
Fpass = AF*Ff + AK*Fk;

% ---------------

% Optimal phase shift, d, as a function of the number of links
if nb <= 3;
    d = pi/2;
%elseif nb == 6;
%    d = 0.7854;
else
    d = (90*real((nb-2.6)^-0.55) - 7)*(pi/180);  % Approx optimal function
end

% ---------------

% Unsigned Cross Track Error
PY = f_CrossTrackError(q,rwi,rwj);
py_3 = PY(3);

% Relative Angle
RelAngle = f_RelativeAngle(q,rwi,rwj);
angle_3 = RelAngle(3);

% ---------------

% Signed Cross Track Error
CrossTrackError = -sign(angle_3)*py_3;

% ---------------

% Integral of cross track error.
CrossTrackErrorINT = getGlobalCrossTrackErrorINT;  % Integral of Cross Track Error

% The max contribution in the heading controller
% from the integral term, to prevent the snake
% from banking too hard.
Ihmax = 1.5*Ph;

MaxCrossTrackError = Ihmax/Ih;
if CrossTrackErrorINT > MaxCrossTrackError
    CrossTrackErrorINT = MaxCrossTrackError;
elseif CrossTrackErrorINT < -MaxCrossTrackError
    CrossTrackErrorINT = -MaxCrossTrackError;
end

% ---------------

% Heading
heading = (1/nb)*(phi1+phi2+phi3+phi4+phi5+phi6);  % Heading (average link angle)

% Theta Ref
WPangle = WPang(WPNum+1)*(pi/180);  % Angle of lines between way points
%theta_ref = -atan(CrossTrackError/Delta)+(WPangle)*(pi/180);                 % Without Integral Term
theta_ref = -atan((Ph*CrossTrackError+Ih*CrossTrackErrorINT)/Delta)+WPangle;  % With Integral Term

HeadingError = heading-theta_ref;

% ---------------

% Turns off integral heading controller if
% a) Cross Track Error is too big, or
% b) Heading Error is too big
% This avoids instabilities caused by the large
% integral errors which occur during the transient
% states when the snake is switching targets.

% threshold1 = 0;
CTEthreshold = 1*L1;
% HEthreshold = 0*(pi/180);
HEthreshold = 3*(pi/180);

if py_3 > CTEthreshold || abs(HeadingError) > HEthreshold
    CrossTrackError = -getGlobalCrossTrackErrorINT;
end

% ---------------

phinought = Ah*(HeadingError);  % Heading Controller

% ---------------

% Set Cross Track Error as a global variable
setGlobalCrossTrackError(getGlobali,CrossTrackError);

% Set the integral of Cross Track Error as a global variable
setGlobalCrossTrackErrorINT(CrossTrackError*(time2steps)+CrossTrackErrorINT);

% ---------------

% Errors
error1 = (A*sin(w*t+offset-0*d))+phinought-(phi2-phi1);
error2 = (A*sin(w*t+offset-1*d))+phinought-(phi3-phi2);
% error3 = (A*sin(w*t+offset-2*d))+phinought-(phi4-phi3);
% error4 = (A*sin(w*t+offset-3*d))+phinought-(phi5-phi4);
% error5 = (A*sin(w*t+offset-4*d))+phinought-(phi6-phi5);
Error = [error1;error2];
%Error = [error1;error2;error3;error4;error5];

% Set Error as a global variable
setGlobalError(getGlobali,Error);

% ---------------

% PD Controller
u1b = Amp*(P*((A*sin(w*t+offset-0*d))+phinought-(phi2-phi1))+D*w*(A*cos(w*t+offset-0*d)-(phi2p-phi1p)));
u2b = Amp*(P*((A*sin(w*t+offset-1*d))+phinought-(phi3-phi2))+D*w*(A*cos(w*t+offset-1*d)-(phi3p-phi2p)));
u3b = Amp*(P*((A*sin(w*t+offset-2*d))+phinought-(phi4-phi3))+D*w*(A*cos(w*t+offset-2*d)-(phi4p-phi3p)));
u4b = Amp*(P*((A*sin(w*t+offset-3*d))+phinought-(phi5-phi4))+D*w*(A*cos(w*t+offset-3*d)-(phi5p-phi4p)));
u5b = Amp*(P*((A*sin(w*t+offset-4*d))+phinought-(phi6-phi5))+D*w*(A*cos(w*t+offset-4*d)-(phi6p-phi5p)));
Ub = [u1b;u2b;u3b;u4b;u5b];

% ---------------

% Feedforward Controller (partially linearized feedback)
Ui = FeedForward(Yn,n,m,M,LVec,Fpass,Ub,Go,SmallAngle);

% ---------------

% Checking that prescribed torque inputs are no higher
% than the max torque which the motors can supply
u1 = f_MaxTorque(MaxTorque,Ui,1);
u2 = f_MaxTorque(MaxTorque,Ui,2);
u3 = f_MaxTorque(MaxTorque,Ui,3);
u4 = f_MaxTorque(MaxTorque,Ui,4);
u5 = f_MaxTorque(MaxTorque,Ui,5);

% Set U as a global variable
setGlobalU(getGlobali,u1,u2);

% ---------------

% Total Actuation Force Vector, in state space
% Here, the actuation is transformed into the state space
Fact1 = -u1/2;
Fact2 = u1/2-u2/2;
Fact3 = u2/2-u3/2;
Fact4 = u3/2-u4/2;
Fact5 = u4/2-u5/2;
Fact6 = u5/2;

% Reassembling the Vector
Fact = [0;0;Fact1;...
        0;0;Fact2;...
        0;0;Fact3;...
        0;0;Fact4;...
        0;0;Fact5;...
        0;0;Fact6];

%% Disturbance
Disturbance = f_Disturbance(n,t,Disturb);

%% Total Force Vector
% (Friction + Elasticity + Actuation + Disturbance + Noise)
Q = Ff + Fk + Fact + Disturbance + Noise(:,round(t*10)+1);

%% Zero elements of the LHS matrix
Zr13 = zeros(n,n);
Zr31 = zeros(n,n);
Zr22 = zeros(m,m);
Zr23 = zeros(m,n);
Zr32 = zeros(n,m);
Id = eye(n);

%% Builds and solves the Matrix DAE for the system

% PHIq transpose
PHIqT = transpose(PHIq);

if PF == 0  % Reg Formulation
    LHS = [  M  , PHIqT, Zr13;...
            PHIq,  Zr22, Zr23;...
            Zr31,  Zr32,  Id];    
    RHS = [Q; Gamma ; qp];
elseif PF == 1  % Penalty Formulation, for more complicated systems
    alpha = 10^-6;
    zeta = 1;
    omega = 10;

    M_Bar = f_MBar(M,PHIq,alpha);

    Q_Bar = f_QBar(Yn,Q,PHIq,alpha,omega,zeta,t,n,m);

    LHS = [M_Bar, PHIqT, Zr13;...
           PHIq ,  Zr22, Zr23;...
           Zr31 ,  Zr32,  Id];
    RHS = [Q_Bar; Gamma ; qp];
else
    error('Do you want the Penalty Formulation or not, ya donk?');
end

% Inverts the matrix equation, giving solution
Ypn = LHS\RHS;

end
