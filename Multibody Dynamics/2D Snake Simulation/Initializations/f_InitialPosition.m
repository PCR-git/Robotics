function q = f_InitialPosition(LengthVec,rw1)

% Offset, to start from Way Point 1
RW1 = [rw1;0;rw1;0;rw1;0];

L1 = LengthVec(1);
L2 = LengthVec(2);
L3 = LengthVec(3);

% Initial Position 1 (straight, 0 degrees)
q = [0;-L1;0;0;-2*L1-L2;0;0;-2*L1-2*L2-L3;0] + RW1;

% % Initial Position 2 (straight, 45 degrees)
% q = [L1*sind(45);-L1*cosd(45);45*(pi/180);...
%      2*L1*sind(45)+L2*sind(45);-2*L1*cosd(45)-L2*cosd(45);45*(pi/180);...
%      2*L1*sind(45)+2*L2*sind(45)+L3*sind(45);-2*L1*cosd(45)-2*L2*cosd(45)-L3*cosd(45);45*(pi/180)] + RW1;

% % Initial Position 3 (straight, 90 degrees)
% q = [-L1;0;-pi/2;-2*L1-L2;0;-pi/2;-2*L1-2*L2-L3;0;-pi/2] + RW1;

% % Initial Position 4 (straight, -180 degrees)
% q = [0;L1;-pi;0;2*L1+L2;-pi;0;2*L1+2*L2+L3;-pi] + RW1;

% % Initial Position 5 (L shape)
% q = [0;-L1;0;L2;-2*L1;pi/2;2*L2+L3;-2*L1;pi/2] + RW1;

% % Initial Position 6 (inverse-L shape)
%q = [L1;0;pi/2;2*L1+L2;0;pi/2;2*L1+2*L2;-L3;0] + RW1;

end