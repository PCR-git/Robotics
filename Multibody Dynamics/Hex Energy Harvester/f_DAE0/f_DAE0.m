% Solves for the derivative of the state vector

function [Ypn] = f_DAE0(t,Yn,Yi,M,PHIq,Gamma,n,m,s,v,g,PF,SC)

% Calls Build Mechanism
if t > 0
    [M,PHIq,Gamma] = BuildMechanism(Yn,n,m,M,s,v,g,t,SC);
end

%nb = n/3;             % Number of bodies
q = Yn((n+m+1):end);  % Position
qp = Yn(1:n);         % Velocity

qi = Yi((n+m+1):end);

% x1 = q(1);
% y1 = q(2);
phi1 = q(3);
% x2 = q(4);
% y2 = q(5);
phi2 = q(6);
% x3 = q(7);
% y3 = q(8);
phi3 = q(9);
% x4 = q(10);
% y4 = q(11);
phi4 = q(12);
% x5 = q(13);
% y5 = q(14);
phi5 = q(15);
% x6 = q(16);
% y6 = q(17);
phi6 = q(18);
x7 = q(19);
y7 = q(20);
phi7 = q(21);
x8 = q(22);
y8 = q(23);
phi8 = q(24);

% x1p = qp(1);
% y1p = qp(2);
phi1p = qp(3);
% x2p = qp(4);
% y2p = qp(5);
phi2p = qp(6);
% x3p = qp(7);
% y3p = qp(8);
phi3p = qp(9);
% x4p = qp(10);
% y4p = qp(11);
phi4p = qp(12);
% x5p = qp(13);
% y5p = qp(14);
phi5p = qp(15);
% x6p = qp(16);
% y6p = qp(17);
phi6p = qp(18);
x7p = qp(19);
y7p = qp(20);
phi7p = qp(21);
x8p = qp(22);
y8p = qp(23);
phi8p = qp(24);

% Initial Positions
X_0 = [qi(1);qi(4);qi(7);qi(10);qi(13);qi(16);qi(19);qi(22)];
%Y_0 = [qi(2);qi(5);qi(8);qi(11);qi(14);qi(17);qi(20);qi(23)];
Phi_0 = [qi(3);qi(6);qi(9);qi(12);qi(15);qi(18);qi(21);qi(24)];

% Mass definitions
% m1 = M(1,1);
% m2 = M(4,4);
% m3 = M(7,7);
% m4 = M(10,10);
% m5 = M(13,13);
% m6 = M(16,16);
% m7 = M(19,19);
% m8 = M(22,22);

% I1 = M(3,3);
% I2 = M(6,6);
% I3 = M(9,9);
% I4 = M(12,12);
% I5 = M(15,15);
% I6 = M(18,18);
% I7 = M(21,21);
% I8 = M(24,24);

%% Rotation Matrices (angles in radians)
% R1 = f_RM(phi1);
% R2 = f_RM(phi2);
% R3 = f_RM(phi3);
% R4 = f_RM(phi4);
% R5 = f_RM(phi5);
% R6 = f_RM(phi6);
R7 = f_RM(phi7);
R8 = f_RM(phi8);

% R1T = f_RM(-phi1);
% R2T = f_RM(-phi2);
% R3T = f_RM(-phi3);
% R4T = f_RM(-phi4);
% R5T = f_RM(-phi5);
% R6T = f_RM(-phi6);
R7T = f_RM(-phi7);
R8T = f_RM(-phi8);

%% Global to Local Definitions

% Position vectors
% rl1 = R1T*[x1;y1];
% rl2 = R2T*[x2;y2];
% rl3 = R3T*[x3;y3];
% rl4 = R4T*[x4;y4];
% rl5 = R5T*[x5;y5];
% rl6 = R6T*[x6;y6];
rl7 = R7T*[x7;y7];
rl8 = R8T*[x8;y8];

% xl1 = rl1(1);
% yl1 = rl1(2);
% xl2 = rl2(1);
% yl2 = rl2(2);
% xl3 = rl3(1);
% yl3 = rl3(2);
% xl4 = rl4(1);
% yl4 = rl4(2);
% xl5 = rl5(1);
% yl5 = rl5(2);
% xl6 = rl6(1);
% yl6 = rl6(2);
%xl7 = rl7(1);
yl7 = rl7(2);
%xl8 = rl8(1);
yl8 = rl8(2);

% Velocity Vectors
% rlp1 = R1T*[x1p;y1p];
% rlp2 = R2T*[x2p;y2p];
% rlp3 = R3T*[x3p;y3p];
% rlp4 = R4T*[x4p;y4p];
% rlp5 = R5T*[x5p;y5p];
% rlp6 = R6T*[x6p;y6p];
rlp7 = R7T*[x7p;y7p];
rlp8 = R8T*[x8p;y8p];
 
% xlp1 = rlp1(1);
% ylp1 = rlp1(2);
% xlp2 = rlp2(1);
% ylp2 = rlp2(2);
% xlp3 = rlp3(1);
% ylp3 = rlp3(2);
% xlp4 = rlp4(1);
% ylp4 = rlp4(2);
% xlp5 = rlp5(1);
% ylp5 = rlp5(2);
% xlp6 = rlp6(1);
% ylp6 = rlp6(2);
%xlp7 = rlp7(1);
ylp7 = rlp7(2);
%xlp8 = rlp8(1);
ylp8 = rlp8(2);

%% Elastic Forces

phi1_0 = Phi_0(1);
phi2_0 = Phi_0(2);
phi3_0 = Phi_0(3);
phi4_0 = Phi_0(4);
phi5_0 = Phi_0(5);
phi6_0 = Phi_0(6);
%phi7_0 = Phi_0(7);
%phi8_0 = Phi_0(8);

Ck = 10;
Fk = -Ck*[0;0;(phi1-phi1_0);0;0;(phi2-phi2_0);0;0;(phi3-phi3_0);0;0;(phi4-phi4_0);0;0;(phi5-phi5_0);0;0;(phi6-phi6_0);0;0;0;0;0;0];

%% Spring-Damper

% Spring
Ks = 50;

x7_0 = X_0(7);
x8_0 = X_0(8);

%r7 = [x7;y7];
%r8 = [x8;y8];
%N = norm(r8-r7);
N = yl7-(-yl8);
Eq = x8_0-x7_0;

Fsl = Ks*[0;(N-Eq)];
Fs7 = R7*(-0.5*Fsl);
Fs8 = R8*(-0.5*Fsl);

Fs = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;Fs7;0;Fs8;0];

% Damper
Kd = 10;

Fdl = Kd*(ylp7-(-ylp8));
Fd7 = R7*[0;-Fdl];
Fd8 = R8*[0;-Fdl];

Fd = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;Fd7;0;Fd8;0];

Fsd = Fs + Fd;

%% Frictional Forces (simple)
% Angular friction (viscous)
Kv = 1;
Fa1 = phi1p;
Fa2 = phi2p;
Fa3 = phi3p;
Fa4 = phi4p;
Fa5 = phi5p;
Fa6 = phi6p;
Fa7 = phi7p;
Fa8 = phi8p;

Ff = -0*Kv*[0;0;Fa1;0;0;Fa2;0;0;Fa3;0;0;Fa4;0;0;Fa5;0;0;Fa6;0;0;Fa7;0;0;Fa8];

%% Total Force Vector
% Elastic force + frictional force + spring-damper
Q = Fk + Ff + Fsd;

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

if PF == 0
    LHS = [  M  , PHIqT, Zr13;...
            PHIq,  Zr22, Zr23;...
            Zr31,  Zr32,  Id];
    RHS = [Q; Gamma ; qp];
elseif PF == 1
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
        
Ypn = LHS\RHS;

end
