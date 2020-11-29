% Relative Angle
function RelAngle = f_RelativeAngle(q,rwi,rwj)

% Position Definitions
x1 = q(1);  y1 = q(2);
% x2 = q(4);  y2 = q(5);
% x3 = q(7);  y3 = q(8);
% x4 = q(10); y4 = q(11);
% x5 = q(13); y5 = q(14);
% x6 = q(16); y6 = q(17);
% x7 = q(19); y7 = q(20);
% x8 = q(22); y8 = q(23);

% Way Point Definitions
xwi = rwi(1);
ywi = rwi(2);
xwj = rwj(1);
ywj = rwj(2);

% Vector between waypoints
v1 = [xwj-xwi; ywj-ywi];

% Vectors between link frames and first way point
v2_1 = [x1-xwi; y1-ywi];
%v2_2 = [x2-xwi; y2-ywi];
%v2_3 = [x3-xwi; y3-ywi];
%v2_4 = [x4-xwi; y4-ywi];
%v2_5 = [x5-xwi; y5-ywi];
%v2_6 = [x6-xwi; y6-ywi];
%v2_7 = [x7-xwi; y7-ywi];
%v2_8 = [x8-xwi; y8-ywi];

% Angle of vector between way points, from global frame
WPangle = -atan2(v1(1), v1(2))*(180/pi);

% Angles of vectors between link frames and first way point,
% from global frame
angle2_1 = -atan2(v2_1(1), v2_1(2))*(180/pi);
%angle2_2 = -atan2(v2_2(1), v2_2(2))*(180/pi);
%angle2_3 = -atan2(v2_3(1), v2_3(2))*(180/pi);
%angle2_4 = -atan2(v2_4(1), v2_4(2))*(180/pi);
%angle2_5 = -atan2(v2_5(1), v2_5(2))*(180/pi);
%angle2_6 = -atan2(v2_6(1), v2_6(2))*(180/pi);
%angle2_7 = -atan2(v2_7(1), v2_7(2))*(180/pi);
%angle2_8 = -atan2(v2_8(1), v2_8(2))*(180/pi);

% Angles between the link frames and the line between the waypoints
% These tell us which side of the line we're on.
% We need this information to determine the sign of the
% atan function, below.
angle_1a = (WPangle - angle2_1);
%angle_2a = (WPangle - angle2_2);
%angle_3a = (WPangle - angle2_3);
%angle_4a = (WPangle - angle2_4);
%angle_5a = (WPangle - angle2_5);
%angle_6a = (WPangle - angle2_6);
%angle_7a = (WPangle - angle2_7);
%angle_8a = (WPangle - angle2_8);

% Angle Conversions
% Changes angles to run from -180 to 180 degrees
% rather than from -270 to 90
angle_1 = f_AngleConversion(angle_1a);
%angle_2 = f_AngleConversion(angle_2a);
%angle_3 = f_AngleConversion(angle_3a);
%angle_4 = f_AngleConversion(angle_4a);
%angle_5 = f_AngleConversion(angle_5a);
%angle_6 = f_AngleConversion(angle_6a);
%angle_7 = f_AngleConversion(angle_7a);
%angle_8 = f_AngleConversion(angle_8a);

%RelAngle = [angle_1;angle_2;angle_3;angle_4;angle_5;angle_6;angle_7;angle_8];

RelAngle = angle_1;

end

