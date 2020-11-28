
clc

syms Amp P A w t phi gamma D d ...
     L1 L2 L3 ...
     m1 I1 m2 I2 m3 I3 ...
     x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 ...
     x1p y1p phi1p x2p y2p phi2p x3p y3p phi3p ...
     x1pp y1pp phi1pp x2pp y2pp phi2pp x3pp y3pp phi3pp ...
     lambda1 lambda2 lambda3 lambda4 ...
     u1 u2 ...
     u1b u2b ...
     f1 f2 f3 f4 f5 f6 f7 f8 f9

u1b = Amp*(P*((A*sin(w*t+phi))+gamma-(phi2-phi1))+D*w*(A*cos(w*t+phi)-(phi2p-phi1p)));
u2b = Amp*(P*((A*sin(w*t+phi-d))+gamma-(phi3-phi2))+D*w*(A*cos(w*t+phi-d)-(phi3p-phi2p)));
 
Lambda = [- sin(phi2)*phi2p^2 + (2*f1)/3 - f4/3 - f7/3 - (2*phi1p^2*sin(phi1))/3 - (phi3p^2*sin(phi3))/3 - (u1b*cos(phi1))/3 + (u1b*cos(phi2))/2 - (u2b*cos(phi2))/2 + (u2b*cos(phi3))/6;...
         (2*cos(phi1)*phi1p^2)/3 + cos(phi2)*phi2p^2 + (cos(phi3)*phi3p^2)/3 + (2*f2)/3 - f5/3 - f8/3 - (u1b*sin(phi1))/3 + (u1b*sin(phi2))/2 - (u2b*sin(phi2))/2 + (u2b*sin(phi3))/6;...
          - sin(phi2)*phi2p^2 + f1/3 + f4/3 - (2*f7)/3 - (phi1p^2*sin(phi1))/3 - (2*phi3p^2*sin(phi3))/3 - (u1b*cos(phi1))/6 + (u1b*cos(phi2))/2 - (u2b*cos(phi2))/2 + (u2b*cos(phi3))/3;...
         (cos(phi1)*phi1p^2)/3 + cos(phi2)*phi2p^2 + (2*cos(phi3)*phi3p^2)/3 + f2/3 + f5/3 - (2*f8)/3 - (u1b*sin(phi1))/6 + (u1b*sin(phi2))/2 - (u2b*sin(phi2))/2 + (u2b*sin(phi3))/3];

lambda1 = Lambda(1);
lambda2 = Lambda(2);
lambda3 = Lambda(3);
lambda4 = Lambda(4);

U = [(16*u1b)/9 + (8*u2b)/9 + (2*lambda1*cos(phi2))/3 - (4*lambda1*cos(phi1))/3 + (2*lambda3*cos(phi2))/3 + (2*lambda3*cos(phi3))/3 - (4*lambda2*sin(phi1))/3 + (2*lambda2*sin(phi2))/3 + (2*lambda4*sin(phi2))/3 + (2*lambda4*sin(phi3))/3;...
     (8*u1b)/9 + (16*u2b)/9 + (4*lambda3*cos(phi3))/3 - (2*lambda1*cos(phi2))/3 - (2*lambda3*cos(phi2))/3 - (2*lambda1*cos(phi1))/3 - (2*lambda2*sin(phi1))/3 - (2*lambda2*sin(phi2))/3 - (2*lambda4*sin(phi2))/3 + (4*lambda4*sin(phi3))/3];

u1 = U(1);
u2 = U(2);

phi1pp = - (3*u1)/8 - (3*lambda1*cos(phi1))/4 - (3*lambda2*sin(phi1))/4
phi2pp = (3*u1)/8 - (3*u2)/8 - (3*lambda1*cos(phi2))/4 - (3*lambda3*cos(phi2))/4 - (3*lambda2*sin(phi2))/4 - (3*lambda4*sin(phi2))/4
phi3pp = (3*u2)/8 - (3*lambda3*cos(phi3))/4 - (3*lambda4*sin(phi3))/4

%y1pp = - cos(phi2)*phi2p^2 + f2/3 + f5/3 + f8/3 - (2*phi1p^2*cos(phi1))/3 - (phi3p^2*cos(phi3))/3 + (u1b*sin(phi1))/3 - (u1b*sin(phi2))/2 + (u2b*sin(phi2))/2 - (u2b*sin(phi3))/6;
%y2pp = (cos(phi1)*phi1p^2)/3 + f2/3 + f5/3 + f8/3 - (phi3p^2*cos(phi3))/3 - (u1b*sin(phi1))/6 - (u2b*sin(phi3))/6;
%y3pp = (cos(phi1)*phi1p^2)/3 + cos(phi2)*phi2p^2 + (2*cos(phi3)*phi3p^2)/3 + f2/3 + f5/3 + f8/3 - (u1b*sin(phi1))/6 + (u1b*sin(phi2))/2 - (u2b*sin(phi2))/2 + (u2b*sin(phi3))/3;

%y1pp = f2/3 + f5/3 + f8/3 - (2*phi1p^2*cos(phi1))/3 - phi2p^2*cos(phi2) - (phi3p^2*cos(phi3))/3 + (Amp*sin(phi2)*(P*(gamma + phi2 - phi3 - A*sin(d - phi - t*w)) + D*w*(phi2p - phi3p + A*cos(d - phi - t*w))))/2 - (Amp*sin(phi3)*(P*(gamma + phi2 - phi3 - A*sin(d - phi - t*w)) + D*w*(phi2p - phi3p + A*cos(d - phi - t*w))))/6 + (Amp*sin(phi1)*(P*(gamma + phi1 - phi2 + A*sin(phi + t*w)) + D*w*(phi1p - phi2p + A*cos(phi + t*w))))/3 - (Amp*sin(phi2)*(P*(gamma + phi1 - phi2 + A*sin(phi + t*w)) + D*w*(phi1p - phi2p + A*cos(phi + t*w))))/2;
y2pp = f2/3 + f5/3 + f8/3 + (phi1p^2*cos(phi1))/3 - (phi3p^2*cos(phi3))/3 - (Amp*sin(phi3)*(P*(gamma + phi2 - phi3 - A*sin(d - phi - t*w)) + D*w*(phi2p - phi3p + A*cos(d - phi - t*w))))/6 - (Amp*sin(phi1)*(P*(gamma + phi1 - phi2 + A*sin(phi + t*w)) + D*w*(phi1p - phi2p + A*cos(phi + t*w))))/6;
%y3pp = f2/3 + f5/3 + f8/3 + (phi1p^2*cos(phi1))/3 + phi2p^2*cos(phi2) + (2*phi3p^2*cos(phi3))/3 - (Amp*sin(phi2)*(P*(gamma + phi2 - phi3 - A*sin(d - phi - t*w)) + D*w*(phi2p - phi3p + A*cos(d - phi - t*w))))/2 + (Amp*sin(phi3)*(P*(gamma + phi2 - phi3 - A*sin(d - phi - t*w)) + D*w*(phi2p - phi3p + A*cos(d - phi - t*w))))/3 - (Amp*sin(phi1)*(P*(gamma + phi1 - phi2 + A*sin(phi + t*w)) + D*w*(phi1p - phi2p + A*cos(phi + t*w))))/6 + (Amp*sin(phi2)*(P*(gamma + phi1 - phi2 + A*sin(phi + t*w)) + D*w*(phi1p - phi2p + A*cos(phi + t*w))))/2;

%Vel = int(y2pp, t);
%Vel = (t*cos(phi1)*phi1p^2)/3 - (Amp*D*t*w*sin(phi1)*phi1p)/6 + (f2*t)/3 + (f5*t)/3 + (f8*t)/3 - (phi3p^2*t*cos(phi3))/3 + (A*Amp*D*sin(d - phi - t*w)*sin(phi3))/6 - (A*Amp*D*sin(phi + t*w)*sin(phi1))/6 - (Amp*P*gamma*t*sin(phi1))/6 - (Amp*P*gamma*t*sin(phi3))/6 - (Amp*P*phi1*t*sin(phi1))/6 + (Amp*P*phi2*t*sin(phi1))/6 - (Amp*P*phi2*t*sin(phi3))/6 + (Amp*P*phi3*t*sin(phi3))/6 + (Amp*D*phi2p*t*w*sin(phi1))/6 - (Amp*D*phi2p*t*w*sin(phi3))/6 + (Amp*D*phi3p*t*w*sin(phi3))/6 + (A*Amp*P*cos(d - phi - t*w)*sin(phi3))/(6*w) + (A*Amp*P*cos(phi + t*w)*sin(phi1))/(6*w);

%Pos = int(Vel,t);
Pos = (t^2*(f2*w + f5*w + f8*w + phi1p^2*w*cos(phi1) - phi3p^2*w*cos(phi3) - (Amp*D*phi1p*w^2*sin(phi1))/2 + (Amp*D*phi2p*w^2*sin(phi1))/2 - (Amp*D*phi2p*w^2*sin(phi3))/2 + (Amp*D*phi3p*w^2*sin(phi3))/2 - (Amp*P*gamma*w*sin(phi1))/2 - (Amp*P*gamma*w*sin(phi3))/2 - (Amp*P*phi1*w*sin(phi1))/2 + (Amp*P*phi2*w*sin(phi1))/2 - (Amp*P*phi2*w*sin(phi3))/2 + (Amp*P*phi3*w*sin(phi3))/2))/(6*w) + (A*Amp*D*cos(d - phi - t*w)*sin(phi3))/(6*w) - (A*Amp*P*sin(d - phi - t*w)*sin(phi3))/(6*w^2) + (A*Amp*D*cos(phi + t*w)*sin(phi1))/(6*w) + (A*Amp*P*sin(phi + t*w)*sin(phi1))/(6*w^2);

T = (2*pi)/w;
%Vavg = subs(Pos,t,T)/T

% Average Velocity over a Period
Vavg = (w*((2*pi^2*(f2*w + f5*w + f8*w + phi1p^2*w*cos(phi1) - phi3p^2*w*cos(phi3) - (Amp*D*phi1p*w^2*sin(phi1))/2 + (Amp*D*phi2p*w^2*sin(phi1))/2 - (Amp*D*phi2p*w^2*sin(phi3))/2 + (Amp*D*phi3p*w^2*sin(phi3))/2 - (Amp*P*gamma*w*sin(phi1))/2 - (Amp*P*gamma*w*sin(phi3))/2 - (Amp*P*phi1*w*sin(phi1))/2 + (Amp*P*phi2*w*sin(phi1))/2 - (Amp*P*phi2*w*sin(phi3))/2 + (Amp*P*phi3*w*sin(phi3))/2))/(3*w^3) + (A*Amp*D*cos(phi)*sin(phi1))/(6*w) + (A*Amp*P*sin(phi)*sin(phi1))/(6*w^2) + (A*Amp*D*sin(phi3)*cos(d - phi))/(6*w) - (A*Amp*P*sin(d - phi)*sin(phi3))/(6*w^2)))/(2*pi);

%diff(Vavg,d)

OptD = -(w*((A*Amp*P*sin(phi3)*cos(d - phi))/(6*w^2) + (A*Amp*D*sin(d - phi)*sin(phi3))/(6*w)))/(2*pi);

