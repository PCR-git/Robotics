% Rotation matrix between global and local frames

function R = f_RM(angle)

R=[cos(angle),-sin(angle); sin(angle),cos(angle)];

end