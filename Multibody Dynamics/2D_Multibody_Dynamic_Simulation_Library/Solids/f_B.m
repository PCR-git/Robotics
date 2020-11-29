% Derivative of the rotation matrix

function BMatrix = f_B(angle)

BMatrix=[-sin(angle),-cos(angle); cos(angle),-sin(angle)];

end