% MBD Simulation of Quadruped Leg %
% 5 BODIES %

close all;
%clear all;
clc

% ---------------------------------------------- %

starttime = 0;
endtime = 4;

nb = 5;
n = nb*3;
m = 2*7;

body0 = 0;
body1 = 1;
body2 = 2;
body3 = 3;
body4 = 4;
body5 = 5;

% Initializations
syms x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 x4 y4 phi4 x5 y5 phi5...
     x1t y1t phi1t x2t y2t phi2t x3t y3t phi3t x4t y4t phi4t...
     x5t y5t phi5t t

% !!!!!!!!!!!!!! CHECK VALUES !!!!!!!!!!!!
% Initial Positions
x10 = 0.15;
y10 = -0.06;
x20 = 0.06;
y20 = -0.09;
x30 = -0.03;
y30 = -0.08;
x40 = -0.02;
y40 = -0.26;
x50 = 0.06;
y50 = -0.16;
% Initial position vector
q0 = [x10;y10;phi10;x20;y20;phi20;x30;y30;phi30;x40;y40;phi40;x50;y50;phi50];
 
% !!!!!!!!!!!!!! CHECK VALUES !!!!!!!!!!!!
% Initial angles, in radians
% ALL ANGLES ARE MEASURED FROM THE GLOBAL FRAME
phi10 = 0*(pi/180);
phi20 = -2.54*(pi/180);
phi30 = -19.89*(pi/180);
phi40 = -22.08*(pi/180);
phi50 = -58.78*(pi/180);

% Lengths of links
AB = 0.04;
BC = 0.18;
%CD = ;
%CE = ;
DE = 0.18;
BF = 0.23;
%EF = ;
EG = 0.29;
FG = 0.24;

% Local position vectors
sA0 = [0;0];
sA1 = [AB/2;0];
sB1 = [-AB/2; 0];
sB2 = [-BC/2;0];
sC3 = [0;0];
sC2 = [BC/2;0];
sD3 = [DE/2;0];
sE3 = [-DE/2;0];
sE4 = [EG/2;0];
sF4 = [0;0];
sF5 = [-BF/2;0];
sB5 = [BF/2;0];
rAD = [-0.15  ; 0.06]; % !!!!!!!!!!!!!! CHECK VALUES !!!!!!!!!!!!
s = [sA1';sB1';sB2';sC3';sC2';sD3';sE3';sE4';sF4';sF5';sB5';rAD'];

q = [x1;y1;phi1;x2;y2;phi2;x3;y3;phi3;x4;y4;phi4;x5;y5;phi5];    % Global position vector

% Body Frames
r1=[x1;y1];
r2=[x2;y2];
r3=[x3;y3];
r4=[x4;y4];
r5=[x5;y5];

% Rotation Matrices
A1 = f_RM(phi1);
A2 = f_RM(phi2);
A3 = f_RM(phi3);
A4 = f_RM(phi4);
A5 = f_RM(phi5);

%A1=[cos(phi1),-sin(phi1); sin(phi1),cos(phi1)];
%A2=[cos(phi2),-sin(phi2); sin(phi2),cos(phi2)];
%A3=[cos(phi3),-sin(phi3); sin(phi3),cos(phi3)];
%A4=[cos(phi4),-sin(phi4); sin(phi4),cos(phi4)];
%A5=[cos(phi5),-sin(phi5); sin(phi5),cos(phi5)];

% ---------------------------------------------- %

% Constraints

% Elements of the Constraint Vector

% Phi1=r1+A1*sA1;            % A
% Phi2=r2+A2*sB2-r1-A1*sB1;  % B
% Phi3=r3+A3*sC3-r2-A2*sC2;  % C
% Phi4=r3+A3*sD3-rAD;        % D
% Phi5=r4+A4*sE4-r3-A3*sE3;  % E
% Phi6=r5+A5*sF5-r4-A4*sF4;  % F
% Phi7=r5+A5*sB5-r1-A1*sB1;  % B
% Phi8 = phi1+pi-t;        % Driving constraint

Phi1 = f_Revolute(q,body0,sA0,body1,sA1);
Phi2 = f_Revolute(q,body1,sB1,body2,sB2);
Phi3 = f_Revolute(q,body2,sC2,body3,sC3);
Phi4 = f_Revolute(q,body0,sA0,body3,sD3)-rAD;
Phi5 = f_Revolute(q,body3,sE3,body4,sE4);
Phi6 = f_Revolute(q,body4,sF4,body5,sF5);
Phi7 = f_Revolute(q,body1,sB1,body5,sB5);
Phi8 = q(3)+pi-t;        % Driving constraint

PHI=[Phi1;Phi2;Phi3;Phi4;Phi5;Phi6;Phi7;Phi8];  % Assembled constraint vector

% ---------------------------------------------- %

% Derivatives; for later use in the dynamics section
PHIq = jacobian(PHI,q);
PHIt = diff(PHI,t);
PHItt = diff(PHIt,t);
PHIqt = diff(PHIq,t);
qt = [x1t;y1t;phi1t;x2t;y2t;phi2t;x3t;y3t;phi3t;x4t;y4t;phi4t;x5t;y5t;phi5t];
Jqpqqp = jacobian(PHIq*qt,q)*qt;

% ---------------------------------------------- %

% Determining the initial position, using the Newton-Rapshon Method

%  Constraint vector at t=0
PHI0 = subs(PHI, t, 0);

% Newton's Method (Newton/Raphson)
i = 0;
while (norm(double(subs(PHI0, [t;q],[0;q0]))) > 1e-4)
    J = double(subs(PHIq,[t;q],[0;q0]));
    dq = -J\(double(subs(PHI0,q,q0)));
    q0 = dq + q0;
    i = i+1;
end
q0;
% Initial Position, after solution by Newton's Method
% Check that these values are sensible

% ---------------------------------------------- %

% Initial velocity and acceleration

PHIt0 = double(subs(PHIt,t,0));
PHItt0 = double(subs(PHItt,t,0));
PHIq0 = double(subs(PHIq,[t;q],[0;q0]));
PHIqt0 = double(subs(PHIqt,[t;q],[0;q0]));

v0 = -PHIq0\PHIt0;    % Initial Velocity

Jqpqqp0 = double(subs(Jqpqqp,[t;q;qt],[0;q0;v0]));

a0 = PHIq0\(Jqpqqp0-2*PHIqt0*v0-PHItt0);    % Initial Acceleration

% ---------------------------------------------- %

% Simulation of motion

numsteps=50;
timestep=(1/numsteps);

pos(1,:)   = q0;
vel(1,:)   = v0;
accel(1,:) = a0;

size=endtime*numsteps+1;

Norm=zeros(1,size);

j=2;
while starttime<endtime
    pos(j,:)=pos(j-1,:)+vel(j-1,:)*timestep+...
        0.5*accel(j-1,:)*timestep^2;
    
    PHIi=double(subs(PHI,[t;q],[starttime;(pos(j,:))']));
    PHIti=double(subs(PHIt,t,starttime));
    PHItti=double(subs(PHItt,t,starttime));
    PHIqi=double(subs(PHIq,[t;q],[starttime;(pos(j,:))']));
    PHIqti=double(subs(PHIqt,[t;q],[starttime;(pos(j,:))']));
    vel(j,:)=-PHIqi\PHIti;
    
    Jqpqqpi=double(subs(Jqpqqp,[t;q;qt],[starttime;(pos(j,:))';(vel(j,:))']));
    accel(j,:)=PHIqi\(Jqpqqpi-2*PHIqti*(vel(j,:))'-PHItti);
 
    Norm(j-1)=norm(PHIi);
    
    starttime=starttime+timestep;
    j=j+1;
end

starttime=linspace(0,1,size);
% Picking out linear and angular elements
linpos=zeros(size,10);
linpos(:,1)=pos(:,1); linpos(:,2)=pos(:,2); linpos(:,3)=pos(:,4);
linpos(:,4)=pos(:,5); linpos(:,5)=pos(:,7); linpos(:,6)=pos(:,8);
linpos(:,7)=pos(:,10); linpos(:,8)=pos(:,11); linpos(:,9)=pos(:,13);
linpos(:,10)=pos(:,14);

angpos=zeros(size,5);
angpos(:,1)=pos(:,3); angpos(:,2)=pos(:,6); angpos(:,3)=pos(:,9);
angpos(:,4)=pos(:,12); angpos(:,4)=pos(:,15);

linvel=zeros(size,10);
linvel(:,1)=vel(:,1); linvel(:,2)=vel(:,2); linvel(:,3)=vel(:,4);
linvel(:,4)=vel(:,5); linvel(:,5)=vel(:,7); linvel(:,6)=vel(:,8);
linvel(:,7)=vel(:,10); linvel(:,8)=vel(:,11); linvel(:,9)=vel(:,13);
linvel(:,10)=vel(:,14);

angvel=zeros(size,5);
angvel(:,1)=vel(:,3); angvel(:,2)=vel(:,6); angvel(:,3)=vel(:,9);
angvel(:,4)=vel(:,12); angvel(:,4)=vel(:,15);

linaccel=zeros(size,10);
linaccel(:,1)=accel(:,1); linaccel(:,2)=accel(:,2); linaccel(:,3)=accel(:,4);
linaccel(:,4)=accel(:,5); linaccel(:,5)=accel(:,7); linaccel(:,6)=accel(:,8);
linaccel(:,7)=accel(:,10); linaccel(:,8)=accel(:,11); linaccel(:,9)=accel(:,13);
linaccel(:,10)=accel(:,14);

angaccel=zeros(size,5);
angaccel(:,1)=accel(:,3); angaccel(:,2)=accel(:,6); angaccel(:,3)=accel(:,9);
angaccel(:,4)=accel(:,12); angaccel(:,4)=accel(:,15);

% PLOTS

% f1=figure(1);
% plot(time,linpos);
% grid on;
% legend('x1','y1','x2','y2','x3','y3','x4','y4');
% title('Linear Displacements vs Time');
% xlabel('Time (sec)');
% ylabel('Position (m)');
% 
% f2=figure(2);
% plot(time,angpos);
% grid on;
% legend('phi1', 'phi2', 'phi3','phi4');
% title('Angular Displacements vs Time');
% xlabel('Time (sec)');
% ylabel('Angle (radians)');

% f3=figure(3);
% plot(pos(:,1),pos(:,2),'r');
% hold on;
% plot(pos(:,4),pos(:,5),'g');
% plot(pos(:,7),pos(:,8),'b');
% plot(pos(:,13),pos(:,14),'y');
% plot(pos(:,10),pos(:,11),'black');
% grid on;
% legend('Pt B','Pt C','Pt E','Pt F','Pt G');
% title('Trajectories');
% xlabel('X-Position (m)');
% ylabel('Y-Position (m)');

% f4=figure(4);
% plot(time,linvel);
% grid on;
% legend('vx1','vy1','vx2','vy2','vx3','vy3','vx4','vy4');
% title('Linear Velocities vs Time');
% xlabel('Time (sec)');
% ylabel('Velocity (m/s)');
% 
% f5=figure(5);
% plot(time,angvel);
% grid on;
% legend('phi1p', 'phi2p', 'phi3p', 'phi4');
% title('Angular Velocities vs Time');
% xlabel('Time (sec)');
% ylabel('Angular Velocity (radians/s)');
% 
% f6=figure(6);
% plot(time,linaccel);
% grid on;
% legend('ax1','ay1','ax2','ay2','ax3','ay3','ax4','ay4');
% title('Linear Accelerations vs Time');
% xlabel('Time (sec)');
% ylabel('Accel (m/s^2)');
% 
% f7=figure(7);
% plot(time,angaccel);
% grid on;
% legend('phi1pp', 'phi2pp', 'phi3pp','phi4pp');
% title('Angular Accelerations vs Time');
% xlabel('Time (sec)');
% ylabel('Angular Accel (radians/s^2)');
%
% f8=figure(8);
% plot(time,Norm);
% grid on;
% title ('Norm of the Constraints vs Time');
% xlabel ('Time (s)');
% ylabel('Norm');

animate_leg(size,pos,s)
