% Sets up initial position vector

function q = f_InitialPosition(angle,LVec,rw1)

% Keep angles in bounds
if abs(angle) > 360
   error('Input angle must be between -360 & 360 degrees');
end

% Convert angle to radians
ang = angle*(pi/180);

% Length Definitions
L1 = LVec(1);
L2 = LVec(2);
L3 = LVec(3);
L4 = LVec(4);
L5 = LVec(5);
L6 = LVec(6);

% Way Point 1
RW1 = [rw1;0;rw1;0;rw1;0;rw1;0;rw1;0;rw1;0];

% Initial Position Vector

a1 = [L1*sin(ang);-L1*cos(ang)];
a2 = [L2*sin(ang);-L2*cos(ang)];
a3 = [L3*sin(ang);-L3*cos(ang)];
a4 = [L4*sin(ang);-L4*cos(ang)];
a5 = [L5*sin(ang);-L5*cos(ang)];
a6 = [L6*sin(ang);-L6*cos(ang)];

r1 = a1;
r2 = 2*a1+a2;
r3 = 2*(a1+a2)+a3;
r4 = 2*(a1+a2+a3)+a4;
r5 = 2*(a1+a2+a3+a4)+a5;
r6 = 2*(a1+a2+a3+a4+a5)+a6;

q = [r1;ang;r2;ang;r3;ang;r4;ang;r5;ang;r6;ang] + RW1;
 
end