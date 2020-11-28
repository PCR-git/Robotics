function SolveTen(EQN)

%close all;
%clear all;
clc;

syms m1 I1 m2 I2 m3 I3...
     x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 ...
     x1p y1p phi1p x2p y2p phi2p x3p y3p phi3p ...
     x1pp y1pp phi1pp x2pp y2pp phi2pp x3pp y3pp phi3pp ...
     lambda1 lambda2 lambda3 lambda4 ...
     u1b u2b...
     f1 f2 f3 f4 f5 f6 f7 f8 f9 f10
 
%  % Definitions
%  N1 = -0.5*u1b;
%  N2 = 0.5*(u1b-u2b);
%  N3 = 0.5*u2b;
%  c1 = cos(phi1);
%  s1 = sin(phi1);
%  c2 = cos(phi2);
%  s2 = sin(phi2); 
%  c3 = cos(phi3);
%  s3 = sin(phi3);
 
 % TEN EQUATIONS OF MOTION
%  Eqn1  = x1pp - x2pp == -N1*c1+N2*c2+phi1p^2*s1+phi2p^2*s2;
%  Eqn2  = y1pp - y2pp == -N1*s1-N2*s2-phi1p^2*c1-phi2p^2*c2;
%  Eqn3  = x2pp - x3pp == -N2*c2-N3*c3+phi2p^2*s2+phi3p^2*s3;
%  Eqn4  = y2pp - y3pp == -N2*s2-N3*s3-phi2p^2*c2-phi3p^2*c3;
%  Eqn5  =        x1pp == (f1-lambda1)/m1;
%  Eqn6  =        y1pp == (f2-lambda2)/m1;
%  Eqn7  =        x2pp == (f4+(lambda1-lambda3))/m2;
%  Eqn8  =        y2pp == (f5+(lambda2-lambda4))/m2;
%  Eqn9  =        x3pp == (f7-lambda3)/m3;
%  Eqn10 =        y3pp == (f8-lambda4)/m3;
 
% Eqn1 = EQN(1);
% Eqn2 = EQN(2);
% Eqn3 = EQN(3);
% Eqn4 = EQN(4);
% Eqn5 = EQN(5);
% Eqn6 = EQN(6);
% Eqn7 = EQN(7);
% Eqn8 = EQN(8);
% Eqn9 = EQN(9);
% Eqn10 = EQN(10);
 
[A,B] = equationsToMatrix([Eqn1,Eqn2,Eqn3,Eqn4,Eqn5,Eqn6,Eqn7,Eqn8,Eqn9,Eqn10],...
        [x1pp,y1pp,x2pp,y2pp,x3pp,y3pp,lambda1,lambda2,lambda3,lambda4]);
 
X = linsolve(A,B)

% x1pp    = (2*f1 + 2*f4 - 2*f7 + m2*u1b*cos(phi1) + m2*u1b*cos(phi2) - m3*u1b*cos(phi1) - m2*u2b*cos(phi2) + m3*u2b*cos(phi3) + 2*m2*phi1p^2*sin(phi1) - 2*m3*phi1p^2*sin(phi1) + 2*m2*phi2p^2*sin(phi2) - 4*m3*phi2p^2*sin(phi2) - 2*m3*phi3p^2*sin(phi3))/(2*(m1 + m2 - m3));
% y1pp    = (2*f2 + 2*f5 - 2*f8 + m2*u1b*sin(phi1) - m2*u1b*sin(phi2) - m3*u1b*sin(phi1) + m2*u2b*sin(phi2) + 2*m3*u1b*sin(phi2) - 2*m3*u2b*sin(phi2) + m3*u2b*sin(phi3) - 2*m2*phi1p^2*cos(phi1) + 2*m3*phi1p^2*cos(phi1) - 2*m2*phi2p^2*cos(phi2) + 4*m3*phi2p^2*cos(phi2) + 2*m3*phi3p^2*cos(phi3))/(2*(m1 + m2 - m3));
% x2pp    = -(2*f7 - 2*f4 - 2*f1 + m1*u1b*cos(phi1) + m1*u1b*cos(phi2) - m1*u2b*cos(phi2) - m3*u1b*cos(phi2) + m3*u2b*cos(phi2) - m3*u2b*cos(phi3) + 2*m1*phi1p^2*sin(phi1) + 2*m1*phi2p^2*sin(phi2) + 2*m3*phi2p^2*sin(phi2) + 2*m3*phi3p^2*sin(phi3))/(2*(m1 + m2 - m3));
% y2pp    = (2*f2 + 2*f5 - 2*f8 - m1*u1b*sin(phi1) + m1*u1b*sin(phi2) - m1*u2b*sin(phi2) + m3*u1b*sin(phi2) - m3*u2b*sin(phi2) + m3*u2b*sin(phi3) + 2*m1*phi1p^2*cos(phi1) + 2*m1*phi2p^2*cos(phi2) + 2*m3*phi2p^2*cos(phi2) + 2*m3*phi3p^2*cos(phi3))/(2*(m1 + m2 - m3));
% x3pp    = -(2*f7 - 2*f4 - 2*f1 + m1*u1b*cos(phi1) - m2*u1b*cos(phi2) - m1*u2b*cos(phi3) + m2*u2b*cos(phi2) - m2*u2b*cos(phi3) + 2*m1*phi1p^2*sin(phi1) + 4*m1*phi2p^2*sin(phi2) + 2*m2*phi2p^2*sin(phi2) + 2*m1*phi3p^2*sin(phi3) + 2*m2*phi3p^2*sin(phi3))/(2*(m1 + m2 - m3));
% y3pp    = (2*f2 + 2*f5 - 2*f8 - m1*u1b*sin(phi1) + 2*m1*u1b*sin(phi2) - 2*m1*u2b*sin(phi2) + m2*u1b*sin(phi2) + m1*u2b*sin(phi3) - m2*u2b*sin(phi2) + m2*u2b*sin(phi3) + 2*m1*phi1p^2*cos(phi1) + 4*m1*phi2p^2*cos(phi2) + 2*m2*phi2p^2*cos(phi2) + 2*m1*phi3p^2*cos(phi3) + 2*m2*phi3p^2*cos(phi3))/(2*(m1 + m2 - m3));
% lambda1 = (2*f1*m2 - 2*f1*m3 - 2*f4*m1 + 2*f7*m1 - m1*m2*u1b*cos(phi1) - m1*m2*u1b*cos(phi2) + m1*m3*u1b*cos(phi1) + m1*m2*u2b*cos(phi2) - m1*m3*u2b*cos(phi3) - 2*m1*m2*phi1p^2*sin(phi1) + 2*m1*m3*phi1p^2*sin(phi1) - 2*m1*m2*phi2p^2*sin(phi2) + 4*m1*m3*phi2p^2*sin(phi2) + 2*m1*m3*phi3p^2*sin(phi3))/(2*(m1 + m2 - m3));
% lambda2 = -(2*f2*m3 - 2*f2*m2 + 2*f5*m1 - 2*f8*m1 + m1*m2*u1b*sin(phi1) - m1*m2*u1b*sin(phi2) - m1*m3*u1b*sin(phi1) + m1*m2*u2b*sin(phi2) + 2*m1*m3*u1b*sin(phi2) - 2*m1*m3*u2b*sin(phi2) + m1*m3*u2b*sin(phi3) - 2*m1*m2*phi1p^2*cos(phi1) + 2*m1*m3*phi1p^2*cos(phi1) - 2*m1*m2*phi2p^2*cos(phi2) + 4*m1*m3*phi2p^2*cos(phi2) + 2*m1*m3*phi3p^2*cos(phi3))/(2*(m1 + m2 - m3));
% lambda3 = (2*f7*m1 - 2*f4*m3 - 2*f1*m3 + 2*f7*m2 + m1*m3*u1b*cos(phi1) - m2*m3*u1b*cos(phi2) - m1*m3*u2b*cos(phi3) + m2*m3*u2b*cos(phi2) - m2*m3*u2b*cos(phi3) + 2*m1*m3*phi1p^2*sin(phi1) + 4*m1*m3*phi2p^2*sin(phi2) + 2*m2*m3*phi2p^2*sin(phi2) + 2*m1*m3*phi3p^2*sin(phi3) + 2*m2*m3*phi3p^2*sin(phi3))/(2*(m1 + m2 - m3));
% lambda4 = -(2*f2*m3 + 2*f5*m3 - 2*f8*m1 - 2*f8*m2 - m1*m3*u1b*sin(phi1) + 2*m1*m3*u1b*sin(phi2) - 2*m1*m3*u2b*sin(phi2) + m2*m3*u1b*sin(phi2) + m1*m3*u2b*sin(phi3) - m2*m3*u2b*sin(phi2) + m2*m3*u2b*sin(phi3) + 2*m1*m3*phi1p^2*cos(phi1) + 4*m1*m3*phi2p^2*cos(phi2) + 2*m2*m3*phi2p^2*cos(phi2) + 2*m1*m3*phi3p^2*cos(phi3) + 2*m2*m3*phi3p^2*cos(phi3))/(2*(m1 + m2 - m3));
% 
% StateVector = [x1pp;y1pp;x2pp;y2pp;x3pp;y3pp;lambda1;lambda2;lambda3;lambda4];

