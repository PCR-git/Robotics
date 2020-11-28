% Specifies the Jacobian PHIq, Gamma (from the acceleration equation),
% and the force vector Qa

function [M,PHIq,Gamma] = BuildMechanism(Y,n,m,M,s,v,~,t,SC)

r = SC(1);
l = SC(2);
f = SC(3);

% q = Y((n+m+1):end);  % Position
% qp = Y(1:n);

% x1 = q(1);
% y1 = q(2);
% 
% x1p = qp(1);
% y1p = qp(2);
% phi1p = qp(3);

body1 = 1;
body2 = 2;
body3 = 3;
body4 = 4;
body5 = 5;
body6 = 6;
body7 = 7;
body8 = 8;

sA1 = s(:,1);
sA2 = s(:,2);
sB2 = s(:,3);
sB3 = s(:,4);
sC3 = s(:,5);
sC4 = s(:,6);
sD4 = s(:,7);
sD5 = s(:,8);
sE5 = s(:,9);
sE6 = s(:,10);
sF6 = s(:,11);
sF1 = s(:,12);
sB7 = s(:,13);
sG7 = s(:,14);
sE8 = s(:,15);
%sG8 = s(:,16);

V11 = v(:,1);
V12 = v(:,2);

% Constructs the Jacobian (each row corresponds to a constraint)  
PHIqd = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
         0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
         0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
     
PHIq = [JAC_revolute(Y,body1,sA1,body2,sA2,n,m);...
        JAC_revolute(Y,body2,sB2,body3,sB3,n,m);...
        JAC_revolute(Y,body3,sC3,body4,sC4,n,m);...
        JAC_revolute(Y,body4,sD4,body5,sD5,n,m);...
        JAC_revolute(Y,body5,sE5,body6,sE6,n,m);...
        JAC_revolute(Y,body6,sF6,body1,sF1,n,m);...
        JAC_revolute(Y,body2,sB2,body7,sB7,n,m);...
        JAC_revolute(Y,body6,sE6,body8,sE8,n,m);...
        JAC_translational(Y,body7,body8,sG7,V11,V12,n,m);...
        PHIqd];

% Constructs Gamma (each row corresponds to a constraint)
% Gammad, for slider crank
% Gammad = [0; 
%           (4*pi^2*f^2*r^2*sin(2*pi*f*t)^2)/(- r^2*sin(2*pi*f*t)^2 + l^2)^(1/2) - (4*pi^2*f^2*r^2*cos(2*pi*f*t)^2)/(- r^2*sin(2*pi*f*t)^2 + l^2)^(1/2) - 4*pi^2*f^2*r*cos(2*pi*f*t) - (4*pi^2*f^2*r^4*cos(2*pi*f*t)^2*sin(2*pi*f*t)^2)/(- r^2*sin(2*pi*f*t)^2 + l^2)^(3/2);...
%           0];

% Gammad, for sin excitation (Doesn't work for some reason)
% Gammad = [0;...
%           -4*pi^2*f^2*r*sin(2*pi*f*t);...
%           0];

% Gammad, for cos excitation
Gammad = [0;...
          -4*pi^2*f^2*r*cos(2*pi*f*t);...
          0];

Gamma = [Gamma_revolute(Y,body1,sA1,body2,sA2,n,m);...
         Gamma_revolute(Y,body2,sB2,body3,sB3,n,m);...
         Gamma_revolute(Y,body3,sC3,body4,sC4,n,m);...
         Gamma_revolute(Y,body4,sD4,body5,sD5,n,m);...
         Gamma_revolute(Y,body5,sE5,body6,sE6,n,m);...
         Gamma_revolute(Y,body6,sF6,body1,sF1,n,m);...
         Gamma_revolute(Y,body2,sB2,body7,sB7,n,m);...
         Gamma_revolute(Y,body6,sE6,body8,sE8,n,m);...
         Gamma_translational(Y,body7,body8,V11,sG7,n,m);...
         Gammad];

end