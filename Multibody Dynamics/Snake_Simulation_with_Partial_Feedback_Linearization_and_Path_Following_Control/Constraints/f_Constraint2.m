% Returns the difference between local measurements of position
% perpendicular to the translational joints, which shows how far
% the joint has deviated from being perfectly straight.
% The function thus gives a measure of the extent to which the
% mechanism has "fallen apart," and can be used as a measure of
% the validity of the simulation at time t.

function [phi] = f_Constraint2(Y,n,m)

[~,~,q] = f_StateVar(Y,n,m);

% Rotation Matrices (angles in radians)
% (Global to Local)
R1i = f_RM(-q(3));
R2i = f_RM(-q(6));

% Global to Local Definitions
r2 = R1i*[q(1);q(2)];
r5 = R2i*[q(4);q(5)];
x2 = r2(1);
x5 = r5(1);

phi = x5-x2;
end