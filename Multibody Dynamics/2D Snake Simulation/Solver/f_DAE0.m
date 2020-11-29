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

% Mass and Inertia Definitions
m1 = M(1,1);
m2 = M(4,4);
m3 = M(7,7);
I1 = M(3,3);
I2 = M(6,6);
I3 = M(9,9);

% Coordinates and Constraints
nb = n/3;             % Number of bodies
q = Yn((n+m+1):end);  % Position
qp = Yn(1:n);         % Velocity

% Position Definitions
x1 = q(1); y1 = q(2); phi1 = q(3);
x2 = q(4); y2 = q(5); phi2 = q(6);
x3 = q(7); y3 = q(8); phi3 = q(9);

% Velocity Definitions
x1p = qp(1); y1p = qp(2); phi1p = qp(3);
x2p = qp(4); y2p = qp(5); phi2p = qp(6);
x3p = qp(7); y3p = qp(8); phi3p = qp(9);

%% Rotation Matrices (angles in radians)

% Local to Global
R1 = f_RM(phi1);
R2 = f_RM(phi2);
R3 = f_RM(phi3);

% Global to Local
R1T = f_RM(-phi1);
R2T = f_RM(-phi2);
R3T = f_RM(-phi3);

%% Global to Local Definitions

rl1 = R1T*[x1;y1];  % Position vector of center of upper link
rl2 = R2T*[x2;y2];  % Position vector of center of lower link
rl3 = R3T*[x3;y3];  % Position vector of center of lower link
rl = [rl1;rl2;rl3]; % Local Position Vector

rlp1 = R1T*[x1p;y1p];   % Velocity vector of center of upper link
rlp2 = R2T*[x2p;y2p];   % Velocity vector of center of lower link
rlp3 = R3T*[x3p;y3p];   % Velocity vector of center of lower link
rlp = [rlp1;rlp2;rlp3]; % Local Velocity Vector

%% ----------- Forces ----------- %%

%% Elastic Force

Fk1 = -Ck*(phi2-phi1);
Fk2 = -Ck*(phi3-phi2);

Fk = [0;0;-0.5*(Fk1);0;0;0.5*Fk1-0.5*Fk2;0;0;0.5*Fk2];

%% Frictional Forces

% Friction Type
FricType = Fr(4);

% Friction Parameters
Cviscous = Fr(5);
Ccoulomb = Fr(6);

% NOTE: Forces due to translation of the COM and pure rotation about
% the COM must be accounted for separately. We divide the mass of a link
% between N masses of mass m/N, distributed at equal intervals along the
% link, and calculate the fraction of the link length such that masses
% located at this distance from the center on both sides of the link would
% have an equivalent absolute value of linear velocity. This procedure
% gives a multiplier of: (2/(2N+1))*integral{(N-n)/N dn} = N/(2N+1).
% Taking the limit as N--> infinity gives: 1/2. Thus, the variables
% VelEff, below, have a multiplier of 1/2 in front.

% Calculating the effective velocity of each link due to
% a pure rotation about its center (assumed to be its COM).
VelEff1 = (1/2)*L1*phi1p*[cos(phi1);-sin(phi1)];
VelEff2 = (1/2)*L2*phi2p*[cos(phi2);-sin(phi2)];
VelEff3 = (1/2)*L3*phi3p*[cos(phi3);-sin(phi3)];
VelEff = [VelEff1;VelEff2;VelEff3];

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

% Frictional forces on each link (rotated into global frame)
force1 = R1*Ff1;
force2 = R2*Ff2;
force3 = R3*Ff3;

% Components of rotational friction
Fa1 = Ffrrot(:,1);
Fa2 = Ffrrot(:,2);
Fa3 = Ffrrot(:,3);

% Torques on each link
torque1 = -L1*norm(Fa1)*sign(qp(3));
torque2 = -L2*norm(Fa2)*sign(qp(6));
torque3 = -L3*norm(Fa3)*sign(qp(9));

% Total Frictional Force Vector
Ff = [force1; torque1; force2; torque2; force3; torque3];

%% Actuation

% ---------------

% Control definitions
Amp = Control(1);
A = Control(2);
w = Control(3);
offset = Control(4);
P = Control(5);
D = Control(6);
Delta = Control(7);
Ah = Control(8);
Ph = Control(9);
Dh = Control(10);
Ih = Control(11);
MaxTorque = Control(12);
AF = Control(13);
AK = Control(14);

% ---------------

% Defining force components, for passing into controller

Fpass = AF*Ff + AK*Fk;

f1 = Fpass(1);
f2 = Fpass(2);
f3 = Fpass(3);
f4 = Fpass(4);
f5 = Fpass(5);
f6 = Fpass(6);
f7 = Fpass(7);
f8 = Fpass(8);
f9 = Fpass(9);

% ---------------

% Optimal phase shift, d, as a function of the number of links
if nb <= 3;
    d = pi/2;
else
    d = (90*real((nb-2.6)^-0.55) - 7)*(pi/180);  % Approx optimal function
end

% ---------------

% Unsigned Cross Track Error
PY = f_CrossTrackError(q,rwi,rwj);
%py_1 = PY(1);
py_2 = PY(2);
%py_3 = PY(3);

% Relative Angle
RelAngle = f_RelativeAngle(q,rwi,rwj);
% angle_1 = RelAngle(1);
angle_2 = RelAngle(2);
%angle_3 = RelAngle(3);

% ---------------

% Signed Cross Track Error
%CrossTrackError = -sign(angle_1)*py_2;
CrossTrackError = -sign(angle_2)*py_2;

% ---------------

CrossTrackErrorINT = getGlobalCrossTrackErrorINT;  % Integral of Cross Track Error

% The max contribution in the heading controller
% from the integral term, to prevent the snake
% from banking too hard, resulting in out of
% control spiralling.

MaxIh = 2.1;

if Ih*CrossTrackErrorINT > MaxIh
    CrossTrackErrorINT = MaxIh/Ih;
elseif Ih*CrossTrackErrorINT < -MaxIh
    CrossTrackErrorINT = -MaxIh/Ih;
end

% ---------------

% Heading
heading = (1/nb)*(phi1+phi2+phi3);  % Heading (average link angle)

% Theta Ref
WPangle = WPang(WPNum+1);
%theta_ref = -atan(CrossTrackError/Delta)+(WPangle)*(pi/180);
theta_ref = -atan((Ph*CrossTrackError+Ih*CrossTrackErrorINT)/Delta)+(WPangle)*(pi/180);

CrossTrackError;
Ih*CrossTrackErrorINT;
% ---------------

% Heading Controller
% (PID)

% Proportional
HeadingError = heading-theta_ref;

% ---------------

% !!!!!!!!!!!!!!!!!!!!
% Doesn't have desired effect. Breaks mechanism.
threshold1 = 0;
threshold2 = 0*(pi/180);
if py_2 > threshold1 || HeadingError > threshold2
    Ih = 0;
    setGlobalCrossTrackErrorINT(0);
end

% ---------------

% Derivative
headingp = (1/nb)*(phi1p+phi2p+phi3p);  % Derivative of Heading
theta_refp = 0;  % Referance Angular Velocity
HeadingErrorDer = headingp-theta_refp;

% ------------- -------------

phinought = Ah*(HeadingError);  % Heading Controller

% ------------- Approach 1 -------------
%Integral
% Ih = -Ph*0.3;
HeadingErrorINT = getGlobalHeadingErrorINT;  % Integral of Heading Error
% HeadingErrorINT2 = abs(HeadingError)*sign(HeadingErrorINT);

%phinought = 0.7*Ph*(heading-theta_ref)+Dh*(headingp-theta_refp);  % Heading Controller
%phinought = Ph*(HeadingError)+Dh*(HeadingErrorDer)+0*Ih*(HeadingErrorINT2);  % Heading Controller

% % ------------- Approach 2 -------------
% Ih1 = 0.0001;
% Ih2 = 0.3*Ph;
% CrossTrackErrorINT = getGlobalCrossTrackErrorINT;  % Integral of Cross Track Error
% CrossTrackErrorINT2 = -atan(Ih1*CrossTrackErrorINT/Delta);
% phinought = Ph*(HeadingError)+Dh*(HeadingErrorDer)+Ih2*(CrossTrackErrorINT2);  % Heading Controller
% phinought = Ph*(HeadingError)+Ih*(CrossTrackErrorINT2);  % Heading Controller
% 
% CrossTrackError/Delta;
% Ih1*CrossTrackErrorINT/Delta;
% 
% -atan(CrossTrackError/Delta);
% -atan(Ih1*CrossTrackErrorINT/Delta);
% 
% Ph*(HeadingError);
% Ih2*(CrossTrackErrorINT2);

% ------------- ------------- -------------

% Set Cross Track Error as a global variable
setGlobalCrossTrackError(getGlobali,CrossTrackError);

% Set Heading Error as a global variable
setGlobalHeadingError(getGlobali,HeadingError);

% Set the integral of Cross Track Error as a global variable
%setGlobalCrossTrackErrorINT(CrossTrackError*(3/329)+CrossTrackErrorINT);
setGlobalCrossTrackErrorINT(CrossTrackError*(time2steps)+CrossTrackErrorINT);

% Set the integral of Heading Error as a global variable
setGlobalHeadingErrorINT(HeadingError+HeadingErrorINT);

% ---------------

% Errors
error1 = (A*sin(w*t+offset))+phinought-(phi2-phi1);
error2 = (A*sin(w*t+offset-d))+phinought-(phi3-phi2);
Error = [error1;error2];

% Set Error as a global variable
setGlobalError(getGlobali,Error);

% ---------------

% Get the integral of Error
ErrorINT = getGlobalErrorINT;

% Set the integral of Error as a global variable
setGlobalErrorINT(Error*(0.0091)+ErrorINT);

% ---------------

% Theta1 = cell2mat(getGlobalTheta1);
% Theta2 = cell2mat(getGlobalTheta2);
% setGlobalTheta1(getGlobali,phi2-phi1);
% setGlobalTheta2(getGlobali,phi3-phi2);

Theta1 = phi2-phi1;
Theta2 = phi3-phi2;

setGlobalTheta1(getGlobali,Theta1);
setGlobalTheta2(getGlobali,Theta2);

Theta1INT = getGlobalTheta1INT;
Theta2INT = getGlobalTheta2INT;

setGlobalTheta1INT(Theta1*(150/4278)+Theta1INT);
setGlobalTheta2INT(Theta2*(150/4278)+Theta2INT);

% ---------------

% PD controller, with partially linearized feedback
u1b = Amp*(P*((A*sin(w*t+offset))+phinought-(phi2-phi1))+D*w*(A*cos(w*t+offset)-(phi2p-phi1p)));
u2b = Amp*(P*((A*sin(w*t+offset-d))+phinought-(phi3-phi2))+D*w*(A*cos(w*t+offset-d)-(phi3p-phi2p)));

% I = 0.01;

% % PD controller, with partially linearized feedback
% u1b = Amp*(P*((A*sin(w*t+offset))+phinought-(phi2-phi1))+D*w*(A*cos(w*t+offset)-(phi2p-phi1p))+0*I*ErrorINT(1));
% u2b = Amp*(P*((A*sin(w*t+offset-d))+phinought-(phi3-phi2))+D*w*(A*cos(w*t+offset-d)-(phi3p-phi2p))+0*I*ErrorINT(2));

% % PD controller, with partially linearized feedback
% u1b = Amp*(P*Error(1)+D*w*(A*cos(w*t+offset)-(phi2p-phi1p))-I*ErrorINT(1));
% u2b = Amp*(P*Error(2)+D*w*(A*cos(w*t+offset-d)-(phi3p-phi2p))-I*ErrorINT(2));

% P*(A*sin(w*t+offset))
% ErrorINT(1)
% ErrorINT(2)

% I = 0.1;

% % PD controller, with partially linearized feedback
% u1b = Amp*(P*((A*sin(w*t+offset))+phinought-(phi2-phi1))+D*w*(A*cos(w*t+offset)-(phi2p-phi1p)) + I*((-A/w)*cos(w*t+offset)+phinought*t-Theta1INT));
% u2b = Amp*(P*((A*sin(w*t+offset-d))+phinought-(phi3-phi2))+D*w*(A*cos(w*t+offset-d)-(phi3p-phi2p)) + I*((-A/w)*cos(w*t+offset)+phinought*t-Theta2INT));

% ---------------

% Lambda Vector (Lagrange multipliers)
Lambda = [-(2*f4*m1 - 2*f1*m3 - 2*f1*m2 + 2*f7*m1 + m1*m2*u1b*cos(phi1) - m1*m2*u1b*cos(phi2) + m1*m3*u1b*cos(phi1) + m1*m2*u2b*cos(phi2) - 2*m1*m3*u1b*cos(phi2) + 2*m1*m3*u2b*cos(phi2) - m1*m3*u2b*cos(phi3) + 2*L1*m1*m2*phi1p^2*sin(phi1) + 2*L1*m1*m3*phi1p^2*sin(phi1) + 2*L2*m1*m2*phi2p^2*sin(phi2) + 4*L2*m1*m3*phi2p^2*sin(phi2) + 2*L3*m1*m3*phi3p^2*sin(phi3))/(2*(m1 + m2 + m3));...
           (2*f2*m2 + 2*f2*m3 - 2*f5*m1 - 2*f8*m1 - m1*m2*u1b*sin(phi1) + m1*m2*u1b*sin(phi2) - m1*m3*u1b*sin(phi1) - m1*m2*u2b*sin(phi2) + 2*m1*m3*u1b*sin(phi2) - 2*m1*m3*u2b*sin(phi2) + m1*m3*u2b*sin(phi3) + 2*L1*m1*m2*phi1p^2*cos(phi1) + 2*L1*m1*m3*phi1p^2*cos(phi1) + 2*L2*m1*m2*phi2p^2*cos(phi2) + 4*L2*m1*m3*phi2p^2*cos(phi2) + 2*L3*m1*m3*phi3p^2*cos(phi3))/(2*(m1 + m2 + m3));...
          -(2*f7*m1 - 2*f4*m3 - 2*f1*m3 + 2*f7*m2 + m1*m3*u1b*cos(phi1) - 2*m1*m3*u1b*cos(phi2) + 2*m1*m3*u2b*cos(phi2) - m2*m3*u1b*cos(phi2) - m1*m3*u2b*cos(phi3) + m2*m3*u2b*cos(phi2) - m2*m3*u2b*cos(phi3) + 2*L1*m1*m3*phi1p^2*sin(phi1) + 4*L2*m1*m3*phi2p^2*sin(phi2) + 2*L2*m2*m3*phi2p^2*sin(phi2) + 2*L3*m1*m3*phi3p^2*sin(phi3) + 2*L3*m2*m3*phi3p^2*sin(phi3))/(2*(m1 + m2 + m3));...
           (2*f2*m3 + 2*f5*m3 - 2*f8*m1 - 2*f8*m2 - m1*m3*u1b*sin(phi1) + 2*m1*m3*u1b*sin(phi2) - 2*m1*m3*u2b*sin(phi2) + m2*m3*u1b*sin(phi2) + m1*m3*u2b*sin(phi3) - m2*m3*u2b*sin(phi2) + m2*m3*u2b*sin(phi3) + 2*L1*m1*m3*phi1p^2*cos(phi1) + 4*L2*m1*m3*phi2p^2*cos(phi2) + 2*L2*m2*m3*phi2p^2*cos(phi2) + 2*L3*m1*m3*phi3p^2*cos(phi3) + 2*L3*m2*m3*phi3p^2*cos(phi3))/(2*(m1 + m2 + m3))];

% Components of Lambda Vector
lambda1 = Lambda(1);
lambda2 = Lambda(2);
lambda3 = Lambda(3);
lambda4 = Lambda(4);

% Total Actuation
if Go == 1  % Go
    Ui = [ (2*(I2*f3 + I3*f3 - I1*f6 - I1*f9 + I1*lambda1*cos(phi2) - I2*lambda1*cos(phi1) - I3*lambda1*cos(phi1) + I1*lambda3*cos(phi2) + I1*lambda3*cos(phi3) + I1*lambda2*sin(phi2) - I2*lambda2*sin(phi1) - I3*lambda2*sin(phi1) + I1*lambda4*sin(phi2) + I1*lambda4*sin(phi3)))/(I1 + I2 + I3);...
          -(2*(I1*f9 - I3*f6 - I3*f3 + I2*f9 + I3*lambda1*cos(phi1) + I3*lambda1*cos(phi2) - I1*lambda3*cos(phi3) - I2*lambda3*cos(phi3) + I3*lambda3*cos(phi2) + I3*lambda2*sin(phi1) + I3*lambda2*sin(phi2) - I1*lambda4*sin(phi3) - I2*lambda4*sin(phi3) + I3*lambda4*sin(phi2)))/(I1 + I2 + I3)];
elseif Go == 0  % Stop
    Ui = [0;0];  % Turn off motors
%     Ui = [5*(0-(phi2p-phi1p));...
%          5*(0-(phi3p-phi2p))];  % Actively Stop
end

% Components of Actuation Vector
u1 = f_MaxTorque(MaxTorque,Ui,1);
u2 = f_MaxTorque(MaxTorque,Ui,2);

% Set U as a global variable
setGlobalU(getGlobali,u1,u2);

% ---------------

% Total Actuation Force Vector
% Here, the actuation is transformed into the state space
Fact1 = -0.5*u1;
Fact2 = 0.5*(u1-u2);
Fact3 = 0.5*u2;

Fact = [0;0;Fact1;...
        0;0;Fact2;...
        0;0;Fact3];
    
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
