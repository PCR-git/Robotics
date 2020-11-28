clc

syms t ...
     x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 x4 y4 phi4 x5 y5 phi5 x6 y6 phi6 x7 y7 phi7 x8 y8 phi8 ...
     x1p y1p phi1p x2p y2p phi2p x3p y3p phi3p x4p y4p phi4p x5p y5p phi5p x6p y6p phi6p x7p y7p phi7p x8p y8p phi8p ...
     r l f

w = (2*pi*f);
theta = w*t;

% Excitation Type
%x = r*cos(theta)+sqrt(l^2-r^2*(sin(theta))^2);  % Slider Crank
%x = r*sin(theta);  % Sin theta
x = r*cos(theta);  % Cos theta

q = [x1;y1;phi1;x2;y2;phi2;x3;y3;phi3;x4;y4;phi4;x5;y5;phi5;x6;y6;phi6;x7;y7;phi7;x8;y8;phi8];
qp = [x1p;y1p;phi1p;x2p;y2p;phi2p;x3p;y3p;phi3p;x4p;y4p;phi4p;x5p;y5p;phi5p;x6p;y6p;phi6p;x7p;y7p;phi7p;x8p;y8p;phi8p];
PHId = [x1;y1-x;phi1];

PHIqd = jacobian(PHId,q)

t1 = -jacobian(PHIqd*qp,q)*qp;
t2 = -2*diff(PHIqd,t)*qp;
t3 = -diff(diff(PHId,t),t);

gammad = t1+t2+t3

