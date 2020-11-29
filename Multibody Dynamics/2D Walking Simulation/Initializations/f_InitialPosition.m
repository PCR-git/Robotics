% Sets up initial position vector

function q = f_InitialPosition(angle,LVec,rw1,Ratio)

angle = -angle;

a = Ratio(1);
b = Ratio(2);
c = Ratio(3);

% Keep angles in bounds
if abs(angle) > 360
   error('Input angle must be between -360 & 360 degrees');
end

% Length Definitions
L1 = LVec(1);
L2 = LVec(2);
L3 = LVec(3);
L4 = LVec(4);
L5 = LVec(5);
L6 = LVec(6);
L7 = LVec(7);
L8 = LVec(8);

% Way Point 1
RW1 = [rw1;0;rw1;0;rw1;0;rw1;0;rw1;0;rw1;0;rw1;0;rw1;0];

% Initial Position Vector

% a1 = [L1*sin(ang);-L1*cos(ang)];
% a2 = [L2*sin(ang);-L2*cos(ang)];
% a3 = [L3*sin(ang);-L3*cos(ang)];
% a4 = [L4*sin(ang);-L4*cos(ang)];
% a5 = [L5*sin(ang);-L5*cos(ang)];
% a6 = [L6*sin(ang);-L6*cos(ang)];
% a7 = [L7*sin(ang);-L7*cos(ang)];
% a8 = [L8*sin(ang);-L8*cos(ang)];
% 
% r1 = a1;
% r2 = 2*a1+a2;
% r3 = 2*(a1+a2)+a3;
% r4 = 2*(a1+a2+a3)+a4;
% r5 = 2*(a1+a2+a3+a4)+a5;
% r6 = 2*(a1+a2+a3+a4+a5)+a6;
% r7 = 2*(a1+a2+a3+a4+a5+a6)+a7;
% r8 = 2*(a1+a2+a3+a4+a5+a6+a7)+a8;
% 
% q = [r1;ang;r2;ang;r3;ang;r4;ang;r5;ang;r6;ang;r7;ang;r8;ang] + RW1;

%angle = -2;
anga4 = pi/2;
anga3 = anga4-angle*(pi/180);
anga2 = anga3-(b/a)*angle*(pi/180);
anga1 = anga2-(c/a)*angle*(pi/180);
angb1 = anga4;
angb2 = anga4+angle*(pi/180);
angb3 = angb2+(b/a)*angle*(pi/180);
angb4 = angb3+(b/a)*angle*(pi/180);

a1 = [L1*sin(anga1);-L1*cos(anga1)];
a2 = [L2*sin(anga2);-L2*cos(anga2)];
a3 = [L3*sin(anga3);-L3*cos(anga3)];
a4 = [L4*sin(anga4);-L4*cos(anga4)];
a5 = [L5*sin(angb1);-L5*cos(angb1)];
a6 = [L6*sin(angb2);-L6*cos(angb2)];
a7 = [L7*sin(angb3);-L7*cos(angb3)];
a8 = [L8*sin(angb4);-L8*cos(angb4)];

r1 = a1;
r2 = 2*a1+a2;
r3 = 2*(a1+a2)+a3;
r4 = 2*(a1+a2+a3)+a4;
r5 = 2*(a1+a2+a3+a4)+a5;
r6 = 2*(a1+a2+a3+a4+a5)+a6;
r7 = 2*(a1+a2+a3+a4+a5+a6)+a7;
r8 = 2*(a1+a2+a3+a4+a5+a6+a7)+a8;

q = [r1;anga1;r2;anga2;r3;anga3;r4;anga4;r5;angb1;r6;angb2;r7;angb3;r8;angb4] + RW1;

end