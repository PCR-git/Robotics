% Rotation matrix between global and local frames

function AMatrix = f_RMp(angle)

AMatrix = [-sin(angle),-cos(angle); cos(angle),-sin(angle)];

end