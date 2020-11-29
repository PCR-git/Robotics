% Rotation matrix between global and local frames

function AMatrix = f_RM(angle)

AMatrix = [cos(angle),-sin(angle); sin(angle),cos(angle)];

end