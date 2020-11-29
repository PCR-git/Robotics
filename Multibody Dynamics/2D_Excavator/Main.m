% Peter Racioppo
% Multibody Dynamics
% Dynamics of the 3 DOF Excavator

close all;
clear %all;
clc

% Declares symbolic variables
syms x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 x4 y4 phi4 x5 y5 phi5 t ...
     x1t y1t phi1t x2t y2t phi2t x3t y3t phi3t x4t y4t phi4t...
     x5t y5t phi5t

 % Local vectors
sO0=[+0.000; +0.000];
sO1=[-2.545; +0.296];
sA1=[+2.433; -1.470];
sA2=[-0.863; -0.088];
sB2=[+1.690; -0.058];
sB3=[-0.766; -0.440];
sC2=[+1.330; -0.058];
sC4=[-0.295; +0.000];
sD4=[+0.295; +0.000];
sD5=[-0.295; +0.000];
sE3=[-0.754; -0.059];
sE5=[+0.295; +0.000];

sQ1=[-0.445; +0.292];
sS2=[-1.393; +0.142];
sR1=[+1.027; +0.079];
%sD4=[+0.295; +0.000];
sT2=[-0.787; +0.401];

rP0=[+0.470; -0.358];

% Driving Constraints
d1=(2.35-0.50*t)^2;
d2=(2.10+0.45*sin(10*t))^2;
d3=(2.00+0.30*sin(10*t))^2;

q= [x1;y1;phi1;x2;y2;phi2;x3;y3;phi3;x4;y4;phi4;x5;y5;phi5];

% Elements of the constraint vector
Phi1 = f_Revolute(0,sO0,1,sO1,q);
Phi2 = f_Revolute(1,sA1,2,sA2,q);
Phi3 = f_Revolute(2,sB2,3,sB3,q);
Phi4 = f_Revolute(2,sC2,4,sC4,q);
Phi5 = f_Revolute(4,sD4,5,sD5,q);
Phi6 = f_Revolute(3,sE3,5,sE5,q);
Phi7 = transpose((f_Revolute(0,sO0,1,sQ1,q)-rP0))*(f_Revolute(0,sO0,1,sQ1,q)-rP0)-d1;
Phi8 = transpose(f_Revolute(1,sR1,2,sS2,q))*f_Revolute(1,sR1,2,sS2,q)-d2;
Phi9 = transpose(f_Revolute(2,sT2,4,sD4,q))*f_Revolute(2,sT2,4,sD4,q)-d3;

% The constraint vector
PHI=[Phi1;Phi2;Phi3;Phi4;Phi5;Phi6;Phi7;Phi8;Phi9];

% Derivatives, for later use computing dynamics
PHIq=jacobian(PHI,q);
PHIt=diff(PHI,t);
PHItt=diff(PHIt,t);
PHIqt=diff(PHIq,t);
qt= [x1t;y1t;phi1t;x2t;y2t;phi2t;x3t;y3t;phi3t;x4t;y4t;phi4t;x5t;y5t;phi5t];
Jqpqqp=jacobian(PHIq*qt,q)*qt;


% PROBLEM 1: Initial position, using the Newton-Rapshon Method

PHI0 = subs(PHI, t, 0);

% Initial positions
q0 = [1; 2; pi/3; 4.5; 3; -pi*(65/180); 5.5; 1; -pi*(70/180); 5; 2; pi*(35/180); 5.5; 1.8; pi*(3/2)];

while (sum(abs(double(subs(PHI0, q, q0)))) > 1e-4)
    J= double(subs(PHIq,[t;q],[0;q0]));
    dq = -J\(double(subs(PHI0,[t;q],[0;q0])));
    q0 = dq + q0;
end
q0


% PROBLEM 2: Initial velocity and acceleration

PHIt0=double(subs(PHIt,t,0));
PHItt0=double(subs(PHItt,t,0));
PHIq0=double(subs(PHIq,[t;q],[0;q0]));
PHIqt0=double(subs(PHIqt,[t;q],[0;q0]));
v0=-PHIq0\PHIt0;

Jqpqqp0=double(subs(Jqpqqp,[t;q;qt],[0;q0;v0]));
a0=PHIq0\(-Jqpqqp0-2*PHIqt0*v0-PHItt0);


% PROBLEM 3: Simulate 1 second of motion

time=0;
timestep=0.01;
j=2;

% Initial conditions
pos(1,:)   = q0;
vel(1,:)   = v0;
accel(1,:) = a0;

% Position, velocity, and acceleration are computed consecutively over
% small time increments. New values are concatenated to form a matrix,
% which stores the values for plotting.

while time<1
    pos(j,:)=pos(j-1,:)+vel(j-1,:)*timestep+...
         0.5*accel(j-1,:)*timestep^2;

    PHIti=double(subs(PHIt,t,time));
    PHItti=double(subs(PHItt,t,time));
    PHIqi=double(subs(PHIq,[t;q],[time;(pos(j,:))']));
    PHIqti=double(subs(PHIqt,[t;q],[time;(pos(j,:))']));
    vel(j,:)=-PHIqi\PHIti;
    
    Jqpqqpi=double(subs(Jqpqqp,[t;q;qt],[time;(pos(j,:))';(vel(j,:))']));
    accel(j,:)=PHIqi\(-Jqpqqpi-2*PHIqti*(vel(j,:))'-PHItti);
 
    time=time+timestep;
    j=j+1;
end

time=linspace(0,1,101);

% Picks out linear and angular elements of the position, velocity, and
% acceleration vectors

linpos=zeros(101,10);
linpos(:,1)=pos(:,1); linpos(:,2)=pos(:,2); linpos(:,3)=pos(:,4);
linpos(:,4)=pos(:,5); linpos(:,5)=pos(:,7); linpos(:,6)=pos(:,8);
linpos(:,7)=pos(:,10); linpos(:,8)=pos(:,11); linpos(:,9)=pos(:,13);
linpos(:,10)=pos(:,14);

angpos=zeros(101,5);
angpos(:,1)=pos(:,3); angpos(:,2)=pos(:,6); angpos(:,3)=pos(:,9);
angpos(:,4)=pos(:,12); angpos(:,5)=pos(:,15);

linvel=zeros(101,10);
linvel(:,1)=vel(:,1); linvel(:,2)=vel(:,2); linvel(:,3)=vel(:,4);
linvel(:,4)=vel(:,5); linvel(:,5)=vel(:,7); linvel(:,6)=vel(:,8);
linvel(:,7)=vel(:,10); linvel(:,8)=vel(:,11); linvel(:,9)=vel(:,13);
linvel(:,10)=vel(:,14);

angvel=zeros(101,5);
angvel(:,1)=vel(:,3); angvel(:,2)=vel(:,6); angvel(:,3)=vel(:,9);
angvel(:,4)=vel(:,12); angvel(:,5)=vel(:,15);

linaccel=zeros(101,10);
linaccel(:,1)=accel(:,1); linaccel(:,2)=accel(:,2); linaccel(:,3)=accel(:,4);
linaccel(:,4)=accel(:,5); linaccel(:,5)=accel(:,7); linaccel(:,6)=accel(:,8);
linaccel(:,7)=accel(:,10); linaccel(:,8)=accel(:,11); linaccel(:,9)=accel(:,13);
linaccel(:,10)=accel(:,14);

angaccel=zeros(101,5);
angaccel(:,1)=accel(:,3); angaccel(:,2)=accel(:,6); angaccel(:,3)=accel(:,9);
angaccel(:,4)=accel(:,12); angaccel(:,5)=accel(:,15);

% PLOTS
f1=figure(1);
plot(time,linpos);
grid on;
legend('x1','y1','x2','y2','x3','y3','x4','y4','x5','y5');
title('Positions of Centers of Gravity vs Time');
xlabel('Time (sec)');
ylabel('Position (m)');

f2=figure(2);
plot(time,angpos);
grid on;
legend('phi1', 'phi2', 'phi3', 'phi4', 'phi5');
title('Angular Positions of Centers of Gravity vs Time');
xlabel('Time (sec)');
ylabel('Angle (radians)');

f3=figure(3);
plot(pos(:,1),pos(:,2));
hold on;
plot(pos(:,4),pos(:,5));
plot(pos(:,7),pos(:,8));
plot(pos(:,10),pos(:,11));
plot(pos(:,13),pos(:,14));
grid on;
legend('CG 1','CG 2','CG 3','CG 4','CG 5');
title('Trajectories of Centers of Gravity');
xlabel('X-Position (m)');
ylabel('Y-Position (m)');

f4=figure(4);
plot(time,linvel);
grid on;
legend('vx1','vy1','vx2','vy2','vx3','vy3','vx4','vy4','vx5','vy5');
title('Linear Velocities of Centers of Gravity vs Time');
xlabel('Time (sec)');
ylabel('Velocity (m/s)');

f5=figure(5);
plot(time,angvel);
grid on;
legend('phi1p', 'phi2p', 'phi3p', 'phi4p', 'phi5p');
title('Angular Velocities of Centers of Gravity vs Time');
xlabel('Time (sec)');
ylabel('Angular Velocity (radians/s)');

f6=figure(6);
plot(time,linaccel);
grid on;
legend('ax1','ay1','ax2','ay2','ax3','ay3','ax4','ay4','ax5','ay5');
title('Linear Accelerations of Centers of Gravity vs Time');
xlabel('Time (sec)');
ylabel('Acceleration (m/s^2)');

f7=figure(7);
plot(time,angaccel);
grid on;
legend('phi1pp', 'phi2pp', 'phi3pp', 'phi4pp', 'phi5pp');
title('Angular Accelerations of Centers of Gravity vs Time');
xlabel('Time (sec)');
ylabel('Angular Acceleration (radians/s^2)');

