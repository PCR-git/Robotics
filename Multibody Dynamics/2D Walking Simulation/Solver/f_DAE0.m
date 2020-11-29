% Builds and Solves the Matrix DAE for the System
% Accounts for all forces exerted on the mechanism
% Specifies control law

function [Ypn] = f_DAE0(t,Yn,M,LVec,PHIq,Gamma,n,m,s,g,time2steps,Control,Fr,PF,rwi,rwj,Ck,Disturb,Noise,Go,WPang,WPNum,MaxAng,MechData,Ratio,Controller)
%% Initializations

% Increments Global Counter Variable
setGlobali(getGlobali+1);

% Calls Build Mechanism
if t > 0
    [M,PHIq,Gamma] = BuildMechanism(Yn,n,m,M,s,MechData,Ratio);
end

% Length Definitions
L1 = LVec(1);
L2 = LVec(2);
L3 = LVec(3);
L4 = LVec(4);
L5 = LVec(5);
L6 = LVec(6);
L7 = LVec(7);
L8 = LVec(8);

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
x7 = q(19); y7 = q(20); phi7 = q(21);
x8 = q(22); y8 = q(23); phi8 = q(24);
X = [x1,x2,x3,x4,x5,x6,x7,x8];
Y = [y1,y2,y3,y4,y5,y6,y7,y8];

% Velocity Definitions
x1p = qp(1);  y1p = qp(2);  phi1p = qp(3);
x2p = qp(4);  y2p = qp(5);  phi2p = qp(6);
x3p = qp(7);  y3p = qp(8);  phi3p = qp(9);
x4p = qp(10); y4p = qp(11); phi4p = qp(12);
x5p = qp(13); y5p = qp(14); phi5p = qp(15);
x6p = qp(16); y6p = qp(17); phi6p = qp(18);
x7p = qp(19); y7p = qp(20); phi7p = qp(21);
x8p = qp(22); y8p = qp(23); phi8p = qp(24);

%% Rotation Matrices (angles in radians)

% Local to Global
R1 = f_RM(phi1);
R2 = f_RM(phi2);
R3 = f_RM(phi3);
R4 = f_RM(phi4);
R5 = f_RM(phi5);
R6 = f_RM(phi6);
R7 = f_RM(phi7);
R8 = f_RM(phi8);

% Global to Local
R1T = f_RM(-phi1);
R2T = f_RM(-phi2);
R3T = f_RM(-phi3);
R4T = f_RM(-phi4);
R5T = f_RM(-phi5);
R6T = f_RM(-phi6);
R7T = f_RM(-phi7);
R8T = f_RM(-phi8);

%% Global to Local Definitions

rl1 = R1T*[x1;y1];  % Position vector of center of link 1
rl2 = R2T*[x2;y2];  % Position vector of center of link 2
rl3 = R3T*[x3;y3];  % Position vector of center of link 3
rl4 = R4T*[x4;y4];  % Position vector of center of link 4
rl5 = R5T*[x5;y5];  % Position vector of center of link 5
rl6 = R6T*[x6;y6];  % Position vector of center of link 6
rl7 = R7T*[x7;y7];  % Position vector of center of link 7
rl8 = R8T*[x8;y8];  % Position vector of center of link 8
rl = [rl1;rl2;rl3;rl4;rl5;rl6;rl7;rl8]; % Local Position Vector

rlp1 = R1T*[x1p;y1p];   % Velocity vector of center of link 1
rlp2 = R2T*[x2p;y2p];   % Velocity vector of center of link 2
rlp3 = R3T*[x3p;y3p];   % Velocity vector of center of link 3
rlp4 = R4T*[x4p;y4p];   % Velocity vector of center of link 4
rlp5 = R5T*[x5p;y5p];   % Velocity vector of center of link 5
rlp6 = R6T*[x6p;y6p];   % Velocity vector of center of link 6
rlp7 = R7T*[x7p;y7p];   % Velocity vector of center of link 7
rlp8 = R8T*[x8p;y8p];   % Velocity vector of center of link 8
rlp = [rlp1;rlp2;rlp3;rlp4;rlp5;rlp6;rlp7;rlp8]; % Local Velocity Vector

%% ----------- Forces ----------- %%

%% Gravitational Forces
Fg = zeros(length(q),1);
ig = 1;
while ig <= length(q)
    if mod(ig+1,3) == 0
        % Fg(ig,1) = -(g-accel(ig))*M(ig,ig);
        Fg(ig,1) = -g*M(ig,ig);
    else
        Fg(ig,1) = 0;
    end
    ig = ig + 1;
end

%% Floor

Foot1 = y1+R1*[0;L1];
Foot2 = y8+R8*[0;-L8];
Joint1 = y2+R2*[0;L2];
Joint2 = y3+R3*[0;L3];
Joint3 = y4+R4*[0;L4];
Joint4 = y5+R5*[0;L5];
Joint5 = y6+R6*[0;L6];
Joint6 = y7+R7*[0;L7];
Joint7 = y8+R8*[0;L8];
% Points = [Foot1;0;Joint1;0;Joint2;0;Joint3;0;Joint5;0;Joint6;0;Joint7;0;Foot2;0];
% Points = [Joint1;0;Joint2;0;Joint3;0;Joint4;0;Joint5;0;Joint6;0;Joint7;0];
Points1 = [Foot1;0;Joint1;0;Joint2;0;Joint3;0;Joint4;0;Joint5;0;Joint6;0;Joint7;0];
Points2 = [Joint1;0;Joint2;0;Joint3;0;Joint4;0;Joint5;0;Joint6;0;Joint7;0;Foot2;0];

% Floor contact with COMs
Fflvec1 = zeros(length(q),1);
iF = 1;
k = 100000;
while iF < length(q)
    if mod(iF+1,3) == 0 && q(iF) < 0.01
        Fflvec1(iF) = -k*q(iF);
    else
        Fflvec1(iF) = 0;
    end
    iF = iF + 1;
end

Fflvec2 = zeros(length(q),1);
% Floor contact with Feet
if Foot1(2) < 0.01
    Fflvec2(2) = -k*Foot1(2);
else
    Fflvec2(2) = Fflvec2(2);
end

if Foot2(2) < 0.01
    Fflvec2(23) = -k*Foot2(2);
else
    Fflvec2(23) = Fflvec2(23);
end

% Floor contact with Joints, attempt 1
% iPt = 1;
% while iPt < length(q)
%    if mod(iPt+1,3) == 0 && Points(iPt) < 0.01
% %        Fflvec(iPt) = -k*Points(iPt);
%          Fflvec(iPt) = Fflvec(iPt)-k*Points(iPt);
% %        if iPt == 2
% %             Fflvec(iPt) = Fflvec(iPt)-k*Points(iPt);
% %        elseif iPt == length(Points)-1
% %            Fflvec(iPt-3) = Fflvec(iPt-3)-k*Points(iPt);
% %        else
% %            Fflvec(iPt) = Fflvec(iPt)-0.5*k*Points(iPt);
% %            Fflvec(iPt-3) = Fflvec(iPt-3)-0.5*k*Points(iPt-3);
% %        end
%        
%        if q(iPt+1)-pi/2 > 0 && q(iPt+1)-pi/2 < pi
%             sign1 = -1;
%        else
%             sign1 = 1;
%        end
%        Fflvec(iPt+1) = -k*Points(iPt)*LVec((iPt+1)/3)*sign1*sin(q(iPt+1));
%         
% %         q(iPt+1)*(180/pi)
% %         q*(180/pi)
%     else
%         Fflvec(iPt) = Fflvec(iPt);
%     end
%         iPt = iPt + 1;
% end

Fflvec3 = zeros(length(q),1);
% Floor contact with Joints, attempt 2
iPt = 1;
while iPt < length(q)
   if mod(iPt+1,3) == 0 && Points1(iPt) < 0.01
         Fflvec3(iPt) = Fflvec3(iPt)-k*Points1(iPt);
       
       if q(iPt+1)-pi/2 > 0 && q(iPt+1)-pi/2 < pi
            sign1 = -1;
       else
            sign1 = 1;
       end
       Fflvec3(iPt+1) = -k*Points1(iPt)*LVec((iPt+1)/3)*sign1*sin(q(iPt+1));
      
    else
        Fflvec3(iPt) = Fflvec3(iPt);
    end
        iPt = iPt + 1;
end

Fflvec4 = zeros(length(q),1);
iPt = 1;
while iPt < length(q)
   if mod(iPt+1,3) == 0 && Points2(iPt) < 0.01
         Fflvec4(iPt) = Fflvec4(iPt)-k*Points2(iPt);
       
       if q(iPt+1)-pi/2 > 0 && q(iPt+1)-pi/2 < pi
            sign1 = -1;
       else
            sign1 = 1;
       end
       Fflvec4(iPt+1) =  -k*Points2(iPt)*LVec((iPt+1)/3)*sign1*sin(q(iPt+1));
    else
        Fflvec4(iPt) = Fflvec4(iPt);
    end
        iPt = iPt + 1;
end

% % Floor contact with Joints, attempt 3
% iPt = 2;
% while iPt < length(q)-3
%    if mod(iPt+1,3) == 0 && Points(iPt) < 0.01
%          Fflvec(iPt) = Fflvec(iPt)-0.5*k*Points(iPt);
%        
%        if q(iPt+1)-pi/2 > 0 && q(iPt+1)-pi/2 < pi
%             sign1 = -1;
%        else
%             sign1 = 1;
%        end
%        Fflvec(iPt+1) = -0.5*k*Points(iPt)*LVec((iPt+1)/3)*sign1*sin(q(iPt+1));
%       
%     else
%         Fflvec(iPt) = Fflvec(iPt);
%     end
%         iPt = iPt + 1;
% end

% Fflvec

% iF = 1;
% while iF < length(q)
%     if mod(iF+1,3) == 0
%         Fflvec(iF,1) = Ffl;
%     end
%     iF = iF + 1;
% end

% % Removing the effect of the spring if it's pulling down, close to zero
% iF = 1;
% while iF < length(q)
%     if mod(iF+1,3) == 0 && Fflvec1(iF) < 0 && q(iF) > -0.01
%         Fflvec1(iF) = 0;
%     end
%     iF = iF + 1;
% end
% 
% if Fflvec2(2) < 0 && Foot1(2) > -0.01
%     Fflvec2(2) = 0;
% end
% 
% if Fflvec2(23) < 0 && Foot2(2) > -0.01
%     Fflvec2(23) = 0;
% end
% 
% iPt = 1;
% while iPt < length(q)
%    if mod(iPt+1,3) == 0 && Fflvec3(iPt) < 0 && Points1(iPt) > -0.01
%        Fflvec3(iPt) = 0;
%    end
%    iPt = iPt + 1;
% end
% 
% iPt = 1;
% while iPt < length(q)
%    if mod(iPt+1,3) == 0 && Fflvec4(iPt) < 0 && Points2(iPt) > -0.01
%        Fflvec4(iPt) = 0;
%    end
%    iPt = iPt + 1;
% end

Fflvec = Fflvec1+Fflvec2+Fflvec3+Fflvec4;

Fg = Fg + Fflvec;

%% Elastic Forces

% % Elastic force vector, from torsional springs at each joint
% Fka = TorsionalElasticity(Ck,q);

% Radius of rotator links
LinkRadius = MechData(4);

% Cable Displacements
% [DisplacementA, DisplacementB] = CableDisplacement(q,s,MechData);
[DisplacementA, DisplacementB] = CableDisplacement2(q,MechData);

% Elastic force vector, from linear springs in series with cables,
% given the cable displacements
Fk = CableElasticity(Ck, LinkRadius, DisplacementA, DisplacementB);

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
VelEff7 = f_EffVel(L7,phi7,phi7p);
VelEff8 = f_EffVel(L8,phi8,phi8p);
VelEff = [VelEff1;VelEff2;VelEff3;VelEff4;VelEff5;VelEff6;VelEff7;VelEff8];

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
    error('Wrong Friction Input');
end

% Components of linear friction
Ff1 = Ffrlin(:,1);
Ff2 = Ffrlin(:,2);
Ff3 = Ffrlin(:,3);
Ff4 = Ffrlin(:,4);
Ff5 = Ffrlin(:,5);
Ff6 = Ffrlin(:,6);
Ff7 = Ffrlin(:,7);
Ff8 = Ffrlin(:,8);

% Frictional forces on each link (rotated into global frame)
force1 = R1*Ff1;
force2 = R2*Ff2;
force3 = R3*Ff3;
force4 = R4*Ff4;
force5 = R5*Ff5;
force6 = R6*Ff6;
force7 = R7*Ff7;
force8 = R8*Ff8;

% Components of rotational friction
Fa1 = Ffrrot(:,1);
Fa2 = Ffrrot(:,2);
Fa3 = Ffrrot(:,3);
Fa4 = Ffrrot(:,4);
Fa5 = Ffrrot(:,5);
Fa6 = Ffrrot(:,6);
Fa7 = Ffrrot(:,7);
Fa8 = Ffrrot(:,8);

% Torques on each link
torque1 = -0.5*L1*norm(Fa1)*sign(qp(3));
torque2 = -0.5*L2*norm(Fa2)*sign(qp(6));
torque3 = -0.5*L3*norm(Fa3)*sign(qp(9));
torque4 = -0.5*L4*norm(Fa4)*sign(qp(12));
torque5 = -0.5*L5*norm(Fa5)*sign(qp(15));
torque6 = -0.5*L6*norm(Fa6)*sign(qp(18));
torque7 = -0.5*L7*norm(Fa7)*sign(qp(21));
torque8 = -0.5*L8*norm(Fa8)*sign(qp(24));

% Total Frictional Force Vector
Ff = [force1; torque1; force2; torque2; force3; torque3; force4; torque4;...
      force5; torque5; force6; torque6; force7; torque7; force8; torque8];

% Add damper to spring
% Friction applies when near floor
iff = 1;
while iff < length(q)
    if mod(iff+1,3) == 0 && q(iff) < 0.01
        Ff(iff,1) = 1*Ff(iff,1);
    elseif mod(iff+2,3) == 0 && q(iff+1) < 0.01
        Ff(iff,1) = 10*Ff(iff,1);
    elseif mod(iff,3) == 0 && q(iff-1) < 0.01
        Ff(iff,1) = 1*Ff(iff,1);
    else
        Ff(iff,1) = Ff(iff,1);
    end
    iff = iff + 1;
end

% iff = 1;
% while iff < length(q)
%     if mod(iff+1,3) == 0 && Points1(iff) < 0.01
%         Ff(iff,1) = Ff(iff,1)+0.5*Ff1(iff,1);
%     elseif mod(iff+2,3) == 0 && Points1(iff+1) < 0.01
%         Ff(iff,1) = Ff(iff,1)+5*Ff1(iff,1);
%     elseif mod(iff,3) == 0 && Points1(iff-1) < 0.01
%         Ff(iff,1) = Ff(iff,1)+0.5*Ff1(iff,1);
%     else
%         Ff(iff,1) = 0*Ff1(iff,1);
%     end
%     iff = iff + 1;
% end
% 
% iff = 1;
% while iff < length(q)
%     if mod(iff+1,3) == 0 && Points2(iff) < 0.01
%         Ff(iff,1) = Ff(iff,1)+0.5*Ff1(iff,1);
%     elseif mod(iff+2,3) == 0 && Points2(iff+1) < 0.01
%         Ff(iff,1) = Ff(iff,1)+5*Ff1(iff,1);
%     elseif mod(iff,3) == 0 && Points2(iff-1) < 0.01
%         Ff(iff,1) = Ff(iff,1)+0.5*Ff1(iff,1);
%     else
%         Ff(iff,1) = 0*Ff1(iff,1);
%     end
%     iff = iff + 1;
% end

% --------------------
% Foot Friction
% Applies only in x-direction
% Foot1 = y1+R1*[0;L1];
% Foot2 = y8+R8*[0;-L8];

R1p = f_RMp(phi1);
R8p = f_RMp(phi8);

if Foot1(2) < 0.01
    Foot1p = [x1p;y1p] + R1p*phi1p*[0;L1];
else
    Foot1p = [0;0];
end

if Foot2(2) < 0.01
    Foot2p = [x8p;y8p] + R8p*phi8p*[0;-L8];
else
    Foot2p = [0;0];
end

FricFoot = [-Foot1p(1);0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;-Foot2p(1);0;0];

if Foot1p(1) > 0
    FricFoot(1) = 0*FricFoot(1);
else
    FricFoot(1) = 10*FricFoot(1);
end

if Foot2p(1) < 0
    FricFoot(21) = 0*FricFoot(21);
else
    FricFoot(21) = 10*FricFoot(21);
end

iPt = 1;
while iPt < length(q)
   if mod(iPt+2,3) == 0
       if q(iPt+2)-pi/2 > 0 && q(iPt+2)-pi/2 < pi
            sign1 = -1;
       else
            sign1 = 1;
       end
       FricFoot(iPt+1) = FricFoot(iPt)*LVec((iPt+2)/3)*sign1*sin(q(iPt+2));
    else
        FricFoot(iPt) = FricFoot(iPt);
    end
        iPt = iPt + 1;
end

Ff = Ff + FricFoot;

%% Actuation

% ---------------

% Control definitions
Amp = Control(1);          % Total amplitude of control signal
A = Control(2);            % Amplitude of sine wave
w = Control(3);            % Angular frequency of sine wave
offset = Control(4);       % Initial offset of sine wave
P = Control(5);            % Proportional Gain
D = Control(6);            % Derivative Gain
Angle = Control(16);       % Alpha 0
Mode = Control(17);        % Robot mode

% ---------------

d = 1.5;
phinought = 0;

% ---------------

%% Motor Torques
% Needed motor torques, given desired joint torques

% % Link Angle Average Controller
% [TorqueM1, TorqueM2] = MotorTorque(Uvec,MechData);

if Go == 1 && Controller == 2
% Angle Sum Controller
%     % P Controller
%     TorqueM1 = Amp*(P*((A*sin(w*t+offset-0*d))+phinought-(phi4-phi1)));
%     TorqueM2 = Amp*(P*((A*sin(w*t+offset-1*d))+phinought-(phi8-phi5)));

A1 = 2*A;
A2 = A;

% PD Controller
TorqueM1 = Amp*(P*((A1*sin(w*t+offset-0*d))+phinought-(phi4-phi1))+D*w*(A1*cos(w*t+offset-0*d)-(phi4p-phi1p)));
TorqueM2 = Amp*(P*((A2*sin(w*t+offset-1*d))+phinought-(phi8-phi5))+D*w*(A2*cos(w*t+offset-1*d)-(phi8p-phi5p)));
elseif Go == 1 && Controller ==3
% Amplitude Controller
theta1 = phi2-phi1;
theta2 = phi3-phi2;
theta5 = phi6-phi5;
theta6 = phi7-phi6;
amp1 = cos((pi-theta2)/2)+cos((pi-theta2)/2-theta1);
amp2 = cos((pi-theta6)/2)+cos((pi-theta6)/2-theta5);

theta1p = phi2p-phi1p;
theta2p = phi3p-phi2p;
theta5p = phi6p-phi5p;
theta6p = phi7p-phi6p;

amp1p = (theta2p/2)*sin((pi-theta2)/2)+(theta2p/2+theta1p)*sin((pi-theta2)/2-theta1);
amp2p = (theta6p/2)*sin((pi-theta6)/2)+(theta6p/2+theta5p)*sin((pi-theta6)/2-theta5);

% % P Amplitude Controller
% TorqueM1 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*sin(w*t-0*d))+phinought)-amp1));
% TorqueM2 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*sin(w*t-1*d))+phinought)-amp2));

% PD Amplitude Controller
TorqueM1 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*sin(w*t-0*d))+phinought)-amp1)+D*((1/sin(1/2))*0.5*A*w*cos(0*d-w*t)*cos(0.5*A*sin(0*d-w*t))-amp1p));
TorqueM2 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*sin(w*t-1*d))+phinought)-amp2)+D*((1/sin(1/2))*0.5*A*w*cos(1*d-w*t)*cos(0.5*A*sin(1*d-w*t))-amp2p));
else
TorqueM1 = 0;
TorqueM2 = 0;
end

theta1 = phi2-phi1;
theta2 = phi3-phi2;
theta3 = phi4-phi3;
% theta4 = phi5-phi4;
theta5 = phi6-phi5;
theta6 = phi7-phi6;
theta7 = phi8-phi7;
phi = phi4;
alpha1 = theta1+theta2+theta3 + phi;
% alpha2 = pi - (theta5+theta6+theta7) + phi;
alpha2 = theta5+theta6+theta7 + phi;

% TauM = Amp;

% alpha1 = pi/2;
% alpha2 = pi/2;
% alpha1 = 0;

if t > 0.1
    TauM1 = f_Loading(alpha1,M,g,LVec,Ratio,MechData);
    TauM2 = f_Loading(alpha2,M,g,LVec,Ratio,MechData);
%     TauM1 = Amp;
%     TauM2 = Amp;
else
    TauM1 = 0;
    TauM2 = 0;
end
    
% TauM = Amp/3;

% PD Controller
theta1p = phi2p-phi1p;
theta2p = phi3p-phi2p;
theta3p = phi4p-phi3p;
% theta4p = phi5p-phi4p;
theta5p = phi6p-phi5p;
theta6p = phi7p-phi6p;
theta7p = phi8p-phi7p;

% alpha1des = 0;
% alpha2des = 0;
% alpha1pdes = 0;
% alpha2pdes = 0;

TC = 1;
phaseshift1 = pi;
phaseshift2 = 0;
if Mode == 1
    alpha1des = (pi/2)*(exp(-t/TC));
    alpha2des = (pi/2)*(exp(-t/TC));
elseif Mode == 2
    alpha1des = (pi/2)*(1-exp(-t/TC));
    alpha2des = (pi/2)*(1-exp(-t/TC));
elseif Mode == 3
    alpha1des = (pi/4)*(sin(w*t+phaseshift1)+abs(sin(w*t+phaseshift1)))-pi/4;
    alpha2des = pi*(sin(w*t+phaseshift2)+abs(sin(w*t+phaseshift2)))-pi/2;
elseif Mode == 4
    if t < 10
    alpha1des = (pi/2)*(exp(-t/TC));
    alpha2des = (pi/2)*(exp(-t/TC));
    elseif t >= 10 && t < 30
    alpha1des = (pi/4)*(sin(w*t+phaseshift1)+abs(sin(w*t+phaseshift1)))-pi/4;
    alpha2des = pi*(sin(w*t+phaseshift2)+abs(sin(w*t+phaseshift2)))-pi/2;
    else
    alpha1des = (pi/2)*(1-exp(-t/TC));
    alpha2des = (pi/2)*(1-exp(-t/TC));
    end
end

alpha1pdes = 0;
alpha2pdes = 0;

phip = phi4p;
alpha1p = theta1p+theta2p+theta3p+phip;
alpha2p = theta5p+theta6p+theta7p+phip;
PD1 = 4*(P*(alpha1-alpha1des) + D*(alpha1p-alpha1pdes));
PD2 = 4*(P*(alpha2-alpha2des) + D*(alpha2p-alpha2pdes));

% TorqueM1 = 0;
% TorqueM2 = 0;

% TorqueM1 = -Amp;
% TorqueM2 = -Amp;

% TorqueM1 = -TauM1;
% TorqueM2 = -TauM2;

TorqueM1 = -PD1;
TorqueM2 = -PD2;

% TorqueM1 = -(TauM1+PD1);
% TorqueM2 = -(TauM2+PD2);

% % Check that prescribed torque inputs are no higher
% % than the max torque which the motors can supply
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% TorqueM1 = f_MaxTorque(MaxTorque,TorqueM1);
% TorqueM2 = f_MaxTorque(MaxTorque,TorqueM2);
TorqueVec = [TorqueM1;TorqueM2];

% % Turn off motors if mechanism nears mechanical limits
% [TorqueM1, TorqueM2] = f_MaxAng(MaxAng,TorqueM1,TorqueM2,phi1,phi2,phi5,phi6);

% Set Torque as a global variable
setGlobalTorque(getGlobali,TorqueVec);

% Transform motor torques to joint torques
JointTorqueVec = MotorToJoints(M,TorqueM1,TorqueM2,MechData);

if Controller == 1
JointTorque1 = u1;
JointTorque2 = u2;
JointTorque3 = u3;
JointTorque4 = u4;
JointTorque5 = u5;
JointTorque6 = u6;
JointTorque7 = u7;
elseif Controller == 2 || 3
% Components of Torque
JointTorque1 = JointTorqueVec(1);
JointTorque2 = JointTorqueVec(2);
JointTorque3 = JointTorqueVec(3);
JointTorque4 = JointTorqueVec(4);
JointTorque5 = JointTorqueVec(5);
JointTorque6 = JointTorqueVec(6);
JointTorque7 = JointTorqueVec(7);
else
    error('Invalid Controller Entry');
end

%% Total Actuation Force Vector, in state space
% Here, the actuation is transformed into the state space

Fact1 = -JointTorque1/2;
Fact2 = JointTorque1/2-JointTorque2/2;
Fact3 = JointTorque2/2-JointTorque3/2;
Fact4 = JointTorque3/2-JointTorque4/2;
Fact5 = JointTorque4/2-JointTorque5/2;
Fact6 = JointTorque5/2-JointTorque6/2;
Fact7 = JointTorque6/2-JointTorque7/2;
Fact8 = JointTorque7/2;

% Assembling the generalized actuation vector
Fact = [0;0;Fact1;...
        0;0;Fact2;...
        0;0;Fact3;...
        0;0;Fact4;...
        0;0;Fact5;...
        0;0;Fact6;...
        0;0;Fact7;...
        0;0;Fact8];

%% Reaction Forces

% Fr = zeros(length(q),1);
% ir = 1;
% while ir < length(q)
%     if mod(ir+1,3) == 0 && q(ir) < 0.01
%         Fr(ir,1) = -Fact(ir+1,1)*cos(q(ir));
%     elseif mod(ir+2,3) == 0 && q(ir+1) < 0.01
%         Fr(ir,1) = -Fact(ir+2,1)*sin(q(ir));
%     else
%         Fr(ir,1) = 0;
%     end
%     ir = ir + 1;
% end

%% Disturbance
% Disturbance = f_Disturbance(n,t,Disturb);

%% 
% iF = 1;
% while iF < length(q)
%     if mod(iF+1,3) == 0 && q(iF) < 0.05
%         Ff = 1*Ff;        
%     else
%         Ff = 0*Ff;
%     end
%     iF = iF + 1;
% end

%% Total Generalized Force Vector
% (Gravity + Friction + Actuation + Ground Reaction)
% Fact

Q = Fg + Ff + Fact;

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
