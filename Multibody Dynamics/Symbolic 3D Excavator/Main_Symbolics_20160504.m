close all
clear all
clc

syms x0 y0 z0 x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4 x5 y5 z5
syms dx0 dy0 dz0 dx1 dy1 dz1 dx2 dy2 dz2 dx3 dy3 dz3 dx4 dy4 dz4 dx5 dy5 dz5
syms ddx0 ddy0 ddz0 ddx1 ddy1 ddz1 ddx2 ddy2 ddz2 ddx3 ddy3 ddz3 ddx4 ddy4 ddz4 ddx5 ddy5 ddz5

syms a0 b0 c0 d0 a1 b1 c1 d1 a2 b2 c2 d2 a3 b3 c3 d3 a4 b4 c4 d4 a5 b5 c5 d5
syms da0 db0 dc0 dd0 da1 db1 dc1 dd1 da2 db2 dc2 dd2 da3 db3 dc3 dd3 da4 db4 dc4 dd4 da5 db5 dc5 dd5
syms dda0 ddb0 ddc0 ddd0 dda1 ddb1 ddc1 ddd1 dda2 ddb2 ddc2 ddd2 dda3 ddb3 ddc3 ddd3 dda4 ddb4 ddc4 ddd4 dda5 ddb5 ddc5 ddd5
syms da0 db0 dc0 da1 db1 dc1 da2 db2 dc2 da3 db3 dc3 da4 db4 dc4 da5 db5 dc5

syms e0_0 e1_0 e2_0 e3_0
syms e0_1 e1_1 e2_1 e3_1
syms e0_2 e1_2 e2_2 e3_2
syms e0_3 e1_3 e2_3 e3_3
syms e0_4 e1_4 e2_4 e3_4
syms e0_5 e1_5 e2_5 e3_5
syms t

%%  Local Vectors
mmTOm = 10^(-3);

% Ground
sOg_p = [0; 0; 0]*mmTOm;

% Body 0
sF0_p = [322; 1338; 0]*mmTOm;
sO0_p = [0; 0; 0]*mmTOm;
sP0_p = [792; 980; 0]*mmTOm;
sR0_p = [322; 1338; 0]*mmTOm;

% Body 1
sF1_p = [-2545; 296; 0]*mmTOm;
sQ1_p = [-445; 296; 0]*mmTOm;
sR1_p = [1027; 79; 0]*mmTOm;
sA1_p = [2433; -1470; 0]*mmTOm;

% Body 2
sS2_p = [-1393; 142; 0]*mmTOm;
sA2_p = [-863; -88; 0]*mmTOm;
sT2_p = [-787; 401; 0]*mmTOm;
sC2_p = [1330; -58; 0]*mmTOm;
sB2_p = [1690; -58; 0]*mmTOm;

% Body 3
sB3_p = [-766; -440; 0]*mmTOm;
sE3_p = [-754; -59; 0]*mmTOm;

% Body 4
sC4_p = [-589/2; 0; 0]*mmTOm;
sD4_p = [589/2; 0; 0]*mmTOm;

% Body 5
sD5_p = [-589/2; 0; 0]*mmTOm;
sE5_p = [589/2; 0; 0]*mmTOm;


%%  Symbolic Position Vectors
r0 = [x0; y0; z0];
r1 = [x1; y1; z1];
r2 = [x2; y2; z2];
r3 = [x3; y3; z3];
r4 = [x4; y4; z4];
r5 = [x5; y5; z5];

rg = [0; 0; 0];


%%  Symbolic Velocity Vectors
dr0 = [dx0; dy0; dz0];
dr1 = [dx1; dy1; dz1];
dr2 = [dx2; dy2; dz2];
dr3 = [dx3; dy3; dz3];
dr4 = [dx4; dy4; dz4];
dr5 = [dx5; dy5; dz5];

dp0 = [da0; db0; dc0; dd0];
dp1 = [da1; db1; dc1; dd1];
dp2 = [da2; db2; dc2; dd2];
dp3 = [da3; db3; dc3; dd3];
dp4 = [da4; db4; dc4; dd4];
dp5 = [da5; db5; dc5; dd5];

w0_p = [a0; b0; c0];
w1_p = [a1; b1; c1];
w2_p = [a2; b2; c2];
w3_p = [a3; b3; c3];
w4_p = [a4; b4; c4];
w5_p = [a5; b5; c5];

%%  Symbolic Acceleration Vectors
ddr0 = [ddx0; ddy0; ddz0];
ddr1 = [ddx1; ddy1; ddz1];
ddr2 = [ddx2; ddy2; ddz2];
ddr3 = [ddx3; ddy3; ddz3];
ddr4 = [ddx4; ddy4; ddz4];
ddr5 = [ddx5; ddy5; ddz5];

ddp0 = [dda0; ddb0; ddc0; ddd0];
ddp1 = [dda1; ddb1; ddc1; ddd1];
ddp2 = [dda2; ddb2; ddc2; ddd2];
ddp3 = [dda3; ddb3; ddc3; ddd3];
ddp4 = [dda4; ddb4; ddc4; ddd4];
ddp5 = [dda5; ddb5; ddc5; ddd5];

dw0_p = [da0; db0; dc0];
dw1_p = [da1; db1; dc1];
dw2_p = [da2; db2; dc2];
dw3_p = [da3; db3; dc3];
dw4_p = [da4; db4; dc4];
dw5_p = [da5; db5; dc5];

%%  Symbolic Euler Parameters
%[e0_0,e1_0,e2_0,e3_0] = findEulerParams(phi,0,0);

p0 = [e0_0; e1_0; e2_0; e3_0];
p1 = [e0_1; e1_1; e2_1; e3_1];
p2 = [e0_2; e1_2; e2_2; e3_2];
p3 = [e0_3; e1_3; e2_3; e3_3];
p4 = [e0_4; e1_4; e2_4; e3_4];
p5 = [e0_5; e1_5; e2_5; e3_5];

q = [r0; p0; r1; p1; r2; p2; r3; p3; r4; p4; r5; p5];

f_p = [1; 0; 0];
g_p = [0; 1; 0];
h_p = [0; 0; 1];

%%  Symbolic Rotation Vectors
Ag = eye(3);
A0 = f_AMatrix(e0_0,e1_0,e2_0,e3_0);
A1 = f_AMatrix(e0_1,e1_1,e2_1,e3_1);
A2 = f_AMatrix(e0_2,e1_2,e2_2,e3_2);
A3 = f_AMatrix(e0_3,e1_3,e2_3,e3_3);
A4 = f_AMatrix(e0_4,e1_4,e2_4,e3_4);
A5 = f_AMatrix(e0_5,e1_5,e2_5,e3_5);

%%  Semi-rotation Matrices
G0 = f_GMatrix(e0_0,e1_0,e2_0,e3_0);
G1 = f_GMatrix(e0_1,e1_1,e2_1,e3_1);
G2 = f_GMatrix(e0_2,e1_2,e2_2,e3_2);
G3 = f_GMatrix(e0_3,e1_3,e2_3,e3_3);
G4 = f_GMatrix(e0_4,e1_4,e2_4,e3_4);
G5 = f_GMatrix(e0_5,e1_5,e2_5,e3_5);

%%  Distance
d01 = r1 + A1*sQ1_p - r0 - A0*sP0_p;
d12 = r2 + A2*sS2_p - r1 - A1*sR1_p;
d24 = r4 + A4*sD4_p - r2 - A2*sT2_p;

s1 = 2.35 - 0.5*t;
s2 = 2.10 + 0.45*sin(10*t);
s3 = 2.00 + 0.3*sin(10*t);
s4 = cos(t/2);

%%  Constraint Vector
PHI = [r0 + A0*sO0_p - rg - Ag*sOg_p;...    % Spherical  == ground (i) = Body 0 (j) at Point O
       transpose(f_p)*transpose(Ag)*A0*g_p; % Dot 1  == ground (i) = Body 0 (j)
       transpose(h_p)*transpose(Ag)*A0*g_p; % Dot 1  == ground (i) = Body 0 (j)
       r1 + A1*sF1_p - r0 - A0*sF0_p;       % Spherical  == Body 0 (i) = Body 1 (j) at Point F
       transpose(f_p)*transpose(A0)*A1*h_p; % Dot 1  == Body 0 (i) = Body 1 (j)
       transpose(g_p)*transpose(A0)*A1*h_p; % Dot 1  == Body 0 (i) = Body 1 (j)
       r2 + A2*sA2_p - r1 - A1*sA1_p;       % Spherical  == Body 1 (i) = Body 2 (j) at Point A
       transpose(f_p)*transpose(A1)*A2*h_p; % Dot 1  == Body 1 (i) = Body 2 (j)
       transpose(g_p)*transpose(A1)*A2*h_p; % Dot 1  == Body 1 (i) = Body 2 (j)
       r3 + A3*sB3_p - r2 - A2*sB2_p;       % Spherical  == Body 2 (i) = Body 3 (j) at Point B
       transpose(f_p)*transpose(A2)*A3*h_p; % Dot 1  == Body 2 (i) = Body 3 (j)
       transpose(g_p)*transpose(A2)*A3*h_p; % Dot 1  == Body 2 (i) = Body 3 (j)
       r4 + A4*sC4_p - r2 - A2*sC2_p;       % Spherical  == Body 2 (i) = Body 4 (j) at Point C
       %transpose(f_p)*transpose(A2)*A4*h_p;    % Dot 1  == Body 2 (i) = Body 4 (j) <- Redundant constraint
       %transpose(g_p)*transpose(A2)*A4*h_p;    % Dot 1  == Body 2 (i) = Body 4 (j) <- Redundant constraint
       r5 + A5*sD5_p - r4 - A4*sD4_p;       % Spherical  == Body 4 (i) = Body 5 (j) at Point D
       transpose(f_p)*transpose(A4)*A5*h_p; % Dot 1  == Body 4 (i) = Body 5 (j)
       transpose(g_p)*transpose(A4)*A5*h_p; % Dot 1  == Body 4 (i) = Body 5 (j)
       r3 + A3*sE3_p - r5 - A5*sE5_p;       % Spherical  == Body 5 (i) = Body 3 (j) at Point E
       transpose(f_p)*transpose(A5)*A3*h_p;    % Dot 1  == Body 5 (i) = Body 3 (j) <- Redundant constraint (along x axis)
       %transpose(g_p)*transpose(A5)*A3*h_p; % Dot 1  == Body 5 (i) = Body 3 (j) <- OR could be this one??
       transpose(d01)*d01 - s1^2;   % Spherical-spherical Body 0 (i) = Body 1 (j)
       transpose(d12)*d12 - s2^2;   % Spherical-spherical Body 1 (i) = Body 2 (j)
       transpose(d24)*d24 - s3^2;   % Spherical-spherical Body 2 (i) = Body 4 (j)
       e0_0 - cos(t/2);             % e0 at the base
       transpose(p0)*p0 - 1;
       transpose(p1)*p1 - 1;
       transpose(p2)*p2 - 1;
       transpose(p3)*p3 - 1;
       transpose(p4)*p4 - 1;
       transpose(p5)*p5 - 1];

PHIq = jacobian(PHI,q);

%%  Elimination of Redundant Constraints

% Initial Estimate of the Initial Positions
q_cabin  = [0 0 0 1 0 0 0]';
q_boom   = [1.2 3.2 0 cos(pi/8) 0 0 sin(pi/8)]';
q_stick  = [5 3.3 3.0 cos(pi/8) 0 0 sin(pi/8)]';
q_bucket = [6 4 0 cos(pi/4) 0 0 sin(pi/4)]';
q_rod1   = [6 0 0 cos(pi/4) 0 0 sin(pi/4)]';
q_rod2   = [6 0 0 0.94 0 0 -0.32]';
tVal     = 0;

values = [q_cabin' q_boom' q_stick' q_bucket' q_rod1' q_rod2' tVal];

%%  Perform Newton-Raphson to Determine the Actual Initial Position
var0 = [x0 y0 z0 e0_0 e1_0 e2_0 e3_0];
var1 = [x1 y1 z1 e0_1 e1_1 e2_1 e3_1];
var2 = [x2 y2 z2 e0_2 e1_2 e2_2 e3_2];
var3 = [x3 y3 z3 e0_3 e1_3 e2_3 e3_3];
var4 = [x4 y4 z4 e0_4 e1_4 e2_4 e3_4];
var5 = [x5 y5 z5 e0_5 e1_5 e2_5 e3_5];

variables = [var0 var1 var2 var3 var4 var5 t];
   

error = 1;
count = 0;
while (error) > 1e-6
    qeval    = double(subs(q,variables,values));
    PHIeval  = double(subs(PHI,variables,values));
    PHIqeval = double(subs(PHIq,variables,values));
    
    PHIevalKin = PHIeval(1:32,:);
    PHIevalPP  = PHIeval(37:end,:);
    
    PHINoDriving = [PHIevalKin; PHIevalPP];
    
    PHIqevalKin = PHIqeval(1:32,:);
    PHIqevalPP  = PHIqeval(37:end,:);
    
    PHIqNoDriving = [PHIqevalKin; PHIqevalPP];
    
    dqeval = PHIqNoDriving\-PHINoDriving;
    
    qevalNew = dqeval + qeval;
    error = qevalNew - qeval;
    error = norm(error);
    
    qeval  = qevalNew;
    values = [qeval' tVal];
    count = count + 1;
end
fprintf('Number of iterations: %i \n',count)
disp(qevalNew);

PHI_t = diff(PHI,t);

%%  Kinematics Simulation

vD = 2*[s1*diff(s1,t); s2*diff(s2,t); s3*diff(s3,t); 0.5*s4*diff(s4,t)]; % Have to change this to add in the fourth driving constraint

i = 1;
tstart = 0;
tmax = 1;
step = 0.1;

for tVal = tstart:step:tmax
    
    qeval     = double(subs(q,variables,values));
    PHIeval   = double(subs(PHI,variables,values));
    PHIqeval  = double(subs(PHIq,variables,values));
    vDeval    = double(subs(vD,variables,values));
    PHIteval  = double(subs(PHI_t, [t, variables], [tVal,values]));
    
    % Position
    dqeval = -PHIqeval\PHIeval;
    
    qNew = qeval + dqeval;
    
    Position = zeros(tmax/step,42);
    Position(i,:) = qeval;
    
    r0eval = Position(1:3,i);
    r1eval = Position(8:10,i);
    r2eval = Position(15:17,i);
    r3eval = Position(22:24,i);
    r4eval = Position(29:31,i);
    r5eval = Position(36:38,i);
    
    % Velocity
    
    v = - PHIqeval \ PHIteval;
    
    Velocity = zeros(tmax/step,42);
    Velocity(i,:) = v.';
    
    dr0Eval = Velocity(1:3,i);
    dp0Eval  = Velocity(4:6,i);
    dr1Eval = Velocity(7:9,i);
    dp1Eval  = Velocity(10:12,i);
    dr2Eval = Velocity(13:15,i);
    dp2Eval  = Velocity(16:18,i);
    dr3Eval = Velocity(19:21,i);
    dp3Eval  = Velocity(22:24,i);
    dr4Eval = Velocity(25:27,i);
    dp4Eval  = Velocity(28:30,i);
    dr5Eval = Velocity(31:33,i);
    dp5Eval  = Velocity(34:36,i);
    
    G0eval = double(subs(G0,variables,values));
    G1eval = double(subs(G1,variables,values));
    G2eval = double(subs(G2,variables,values));
    G3eval = double(subs(G3,variables,values));
    G4eval = double(subs(G4,variables,values));
    G5eval = double(subs(G5,variables,values));
    
    w0Eval = 2*G0Eval*dp0Eval;
    w1Eval = 2*G1Eval*dp1Eval;
    w2Eval = 2*G2Eval*dp2Eval;
    w3Eval = 2*G3Eval*dp3Eval;
    w4Eval = 2*G4Eval*dp4Eval;
    w5Eval = 2*G5Eval*dp5Eval;

% Acceleration

    % Rotation Matrix
    A0 = f_AMatrix(qeval(4),qeval(5),qeval(6),qeval(7));
    A1 = f_AMatrix(qeval(11),qeval(12),qeval(13),qeval(14));
    A2 = f_AMatrix(qeval(18),qeval(19),qeval(20),qeval(21));
    A4 = f_AMatrix(qeval(32),qeval(33),qeval(34),qeval(35));
    
    % Gamma Kinematics
    % i= 0 
    
    wg = zeros(3,1);

    % Driving1 ai aj
    % Driving2 ai aj
    gammaS_1   = gammaS(wg,sOg_p,Ag,w0Eval,sO0_p,A0);
    gammaD1_11 = gammaD1(wg,f_p,Ag,w0Eval,g_p,A0);
    gammaD1_12 = gammaD1(wg,h_p,Ag,w0Eval,g_p,A0);
    
    gammaS_2   = gammaS(w0Eval,sF0_p,A0,w1Eval,sF1_p,A1);
    gammaD1_21 = gammaD1(w0Eval,f_p,A0,w1Eval,h_p,A1);
    gammaD1_22 = gammaD1(w0Eval,g_p,A0,w1Eval,h_p,A1);
    
    gammaS_3   = gammaS(w1Eval,sA1_p,A1,w2Eval,sA2_p,A2);
    gammaD1_31 = gammaD1(w1Eval,f_p,A0,w2Eval,h_p,A2);
    gammaD1_32 = gammaD1(w1Eval,g_p,A0,w2Eval,h_p,A2);
    
    gammaS_4   = gammaS(w2Eval,sB2_p,A2,w3Eval,sB3_p,A3);
    gammaD1_41 = gammaD1(w2Eval,f_p,A2,w3Eval,h_p,A3);
    gammaD1_42 = gammaD1(w2Eval,g_p,A2,w3Eval,h_p,A3);
    
    gammaS_5  = gammaS(w2Eval,sC2_p,A2,w4Eval,sC4_p,A4);
    % gammaD1_51 = gammaD1(w2Eval,f_p,A2,w4Eval,h_p,A4);
    % gammaD1_52 = gammaD1(w2Eval,g_p,A2,w3Eval,h_p,A4);
    
    gammaS_6  = gammaS(w4Eval,sD4_p,A4,w5Eval,sD5_p,A5);
    gammaD1_61 = gammaD1(w4Eval,f_p,A4,w5Eval,h_p,A5);
    gammaD1_62 = gammaD1(w4Eval,g_p,A4,w5Eval,h_p,A5);
    
    gammaS_7  = gammaS(w5Eval,sE5_p,A5,w3Eval,sE3_p,A3);
    gammaD1_71 = gammaD1(w5Eval,f_p,A5,w3Eval,h_p,A3);
    % gammaD1_72 = gammaD1(w5Eval,g_p,A5,w3Eval,h_p,A3);
    
    gammaK = [gammaS_1; gammaD1_11; gammaD1_12; ...
              gammaS_2; gammaD1_21; gammaD1_22; ...
              gammaS_3; gammaD1_31; gammaD1_32; ...
              gammaS_4; gammaD1_41; gammaD1_42; ...
              gammaS_5; ...
              gammaS_6; gammaD1_61; gammaD1_62; ...
              gammaS_7; gammaD1_71];
     
    % SSd1 -> Between bodies 0 and 1
    % SSd2 -> 12
    % SSd3 -> 24
    % Gamma Dynamics
    gamma_SSd1 = gammaSSd(r0eval,dr0Eval,w0Eval,sP0_p,A0,r1eval,dr1Eval,w1Eval,sQ1_p,A1,s1,t);
    gamma_SSd2 = gammaSSd(r1eval,dr1Eval,w1Eval,sR1_p,A2,r2eval,dr2Eval,w2Eval,sS2_p,A2,s2,t);
    gamma_SSd3 = gammaSSd(r2eval,dr2Eval,w2Eval,sT2_p,A4,r4eval,dr4Eval,w4Eval,sD4_p,A4,s3,t);
    gamma_rot = (-1/4)*cos(tVal/2);
    
    gammaD = [gamma_SSd1'; gamma_SSd2'; gamma_SSd3'; gamma_rot'];

    gammaP0 = f_gammaP(dp0Eval);
    gammaP1 = f_gammaP(dp1Eval);
    gammaP2 = f_gammaP(dp2Eval);
    gammaP3 = f_gammaP(dp3Eval);
    gammaP4 = f_gammaP(dp4Eval);
    gammaP5 = f_gammaP(dp5Eval);
    
    gammaP = [gammaP0; gammaP1; gammaP2; gammaP3;
              gammaP4; gammaP5];

    Gamma = [gammaK; gammaD; gammaP];
    
    Acc = PHIqeval \ Gamma;
    
    Acceleration = zeros(tmax/step,42);
    Acceleration(i,:) = Acc.';
    
    % Update the q vector
    qeval = qNew;
    values = qeval;
    i = i + 1;
end

