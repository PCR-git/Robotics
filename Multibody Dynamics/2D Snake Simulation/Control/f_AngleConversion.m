% Angle Conversions
% Changes angles to run from -180 to 180 degrees
% rather than from -270 to 90
function angle = f_AngleConversion(angle)

if angle <= -180 && angle >= -270
    angle = (angle+360);
end

if abs(angle) > 180
    angle = -angle;
end

% if angle > 180
%     error ('!');
% elseif angle < -180
%     angle = 360+angle;
% end

end