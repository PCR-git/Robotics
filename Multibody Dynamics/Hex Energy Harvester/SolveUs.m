close all;
clear all;
clc;

syms m1 I1 m2 I2 m3 I3...
     x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 ...
     x1p y1p phi1p x2p y2p phi2p x3p y3p phi3p ...
     x1pp y1pp phi1pp x2pp y2pp phi2pp x3pp y3pp phi3pp ...
     lambda1 lambda2 lambda3 lambda4 ...
     u1 u2...
     u1b u2b

PHIdd = [(4*phi1pp)/3 + lambda1*cos(phi1) + lambda2*sin(phi1) == -u1/2;...
         (4*phi2pp)/3 + lambda1*cos(phi2) + lambda3*cos(phi2) + lambda2*sin(phi2) + lambda4*sin(phi2) == u1/2 - u2/2;...
         (4*phi3pp)/3 + lambda3*cos(phi3) + lambda4*sin(phi3) == u2/2];

eqn1 = PHIdd(1);
eqn2 = PHIdd(2);
eqn3 = PHIdd(3);

[A,B] = equationsToMatrix([eqn1,eqn2,eqn3],[phi1pp,phi2pp,phi3pp]);
X = linsolve(A,B);

DPhi1 = collect(X(2)-X(1),[u1,u2]);
DPhi2 = collect(X(3)-X(2),[u1,u2]);

Eqn1 = (3*u1)/4 - (3*u2)/8 == -((3*lambda1*cos(phi1))/4 - (3*lambda1*cos(phi2))/4 - (3*lambda3*cos(phi2))/4 + (3*lambda2*sin(phi1))/4 - (3*lambda2*sin(phi2))/4 - (3*lambda4*sin(phi2))/4)+u1b;
Eqn2 = (3*u2)/4 - (3*u1)/8 == -((3*lambda1*cos(phi2))/4 + (3*lambda3*cos(phi2))/4 - (3*lambda3*cos(phi3))/4 + (3*lambda2*sin(phi2))/4 + (3*lambda4*sin(phi2))/4 - (3*lambda4*sin(phi3))/4)+u2b;

[C,D] = equationsToMatrix([Eqn1,Eqn2],[u1,u2]);
Y = linsolve(C,D);

u1 = collect(Y(1),[u1b,u2b]);
u2 = collect(Y(2),[u1b,u2b]);

U = [u1;u2]

