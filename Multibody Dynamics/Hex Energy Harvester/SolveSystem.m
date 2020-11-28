function SolveSystem

close all;
clear all;
clc;

syms L1 L2 L3 ...
     m1 I1 m2 I2 m3 I3 ...
     x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 ...
     x1p y1p phi1p x2p y2p phi2p x3p y3p phi3p ...
     x1pp y1pp phi1pp x2pp y2pp phi2pp x3pp y3pp phi3pp ...
     lambda1 lambda2 lambda3 lambda4 ...
     u1 u2 ...
     u1b u2b ...
     f1 f2 f3 f4 f5 f6 f7 f8 f9

L1 = 1;
L2 = 1;
L3 = 1;
m1 = 1;
m2 = 1;
m3 = 1;
I1 = m1*((2*L1)^2)/3;
I2 = m2*((2*L2)^2)/3;
I3 = m3*((2*L3)^2)/3;

phi1pp = -0.5*u1b;
phi2pp = 0.5*(u1b-u2b);
phi3pp = 0.5*u2b;

%qp = [x1p;y1p;phi1p;x2p;y2p;phi2p;x3p;y3p;phi3p];
qpp = [x1pp;y1pp;phi1pp;x2pp;y2pp;phi2pp;x3pp;y3pp;phi3pp];
 
nb = 3;         % Number of bodies
%n = nb*3;      % Number of generalized coordinates
m = (nb-1)*2;   % Number of constraints 

M1 = [m1, 0, 0;...
     0, m1, 0;...
     0, 0, I1];...
M2 = [m2, 0, 0;...
      0, m2, 0;...
      0, 0, I2];...
M3 = [m3, 0, 0;...
       0, m3, 0;...
       0, 0, I3];...
Z3 = zeros(3);
M = [M1,Z3,Z3;...
     Z3,M2,Z3;...
     Z3,Z3,M3];

PHIq = [1, 0, cos(phi1), -1,  0, cos(phi2),  0,  0,         0;...
        0, 1, sin(phi1),  0, -1, sin(phi2),  0,  0,         0;...
        0, 0,         0,  1,  0, cos(phi2), -1,  0, cos(phi3);...
        0, 0,         0,  0,  1, sin(phi2),  0, -1, sin(phi3)];
    
PHIqT = transpose(PHIq);

Lambda = [lambda1; lambda2; lambda3; lambda4];

Q = [f1;f2;-0.5*u1;f4;f5;0.5*(u1-u2);f7;f8;0.5*u2];

Gamma = [L1*sin(phi1)*phi1p^2 + L2*sin(phi2)*phi2p^2;...
        -L1*cos(phi1)*phi1p^2 - L2*cos(phi2)*phi2p^2;...
         L2*sin(phi2)*phi2p^2 + L3*sin(phi3)*phi3p^2;...
        -L2*cos(phi2)*phi2p^2 - L3*cos(phi3)*phi3p^2];

% Zr13 = zeros(n,n);
% Zr31 = zeros(n,n);
Zr22 = zeros(m,m);
% Zr23 = zeros(m,n);
% Zr32 = zeros(n,m);
% Id = eye(n);

% LHS = [  M  , PHIqT, Zr13;...
%         PHIq,  Zr22, Zr23;...
%         Zr31,  Zr32,   Id];   
% RHS = [Q; Gamma ; qp];
% Y = (inv(LHS))*RHS
% % Y = LHS\RHS

% LHS = [  M  , PHIqT, Zr13;...
%         PHIq,  Zr22, Zr23;...
%         Zr31,  Zr32,   Id];
% Y = [qp;];
% RHS = [Q; Gamma ; qp];

LHS = [  M , PHIqT;...
       PHIq, Zr22];
RHS = [Q;Gamma];
Y = [qpp;Lambda];

EQN = LHS*Y == RHS;

%PHIdd = [EQN(3);EQN(6);EQN(9)];
% [A,B] = equationsToMatrix([EQN(3),EQN(6),EQN(9)],...
%         [phi1pp,phi2pp,phi3pp]);
% X = linsolve(A,B)

EQN([3,6,9]) = [];

Eqn1  = EQN(1);
Eqn2  = EQN(2);
Eqn3  = EQN(3);
Eqn4  = EQN(4);
Eqn5  = EQN(5);
Eqn6  = EQN(6);
Eqn7  = EQN(7);
Eqn8  = EQN(8);
Eqn9  = EQN(9);
Eqn10 = EQN(10);

[A,B] = equationsToMatrix([Eqn1,Eqn2,Eqn3,Eqn4,Eqn5,Eqn6,Eqn7,Eqn8,Eqn9,Eqn10],...
        [x1pp,y1pp,x2pp,y2pp,x3pp,y3pp,lambda1,lambda2,lambda3,lambda4]);
 
X = linsolve(A,B);

Lambda = [X(7);X(8);X(9);X(10)];
