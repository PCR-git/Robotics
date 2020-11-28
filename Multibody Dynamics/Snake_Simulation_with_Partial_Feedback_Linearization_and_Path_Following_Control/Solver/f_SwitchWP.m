% In case of a missed way point, changes way point target if snake
% moves a certain distance past the way point, in the direction of the
% vector from the previous way point.

function switchWP = f_SwitchWP(Y,n,m,rwi,rwj,WPtol,CatchDistance)

[~,~,q] = f_StateVar(Y,n,m);

R1 = f_RM(q(3));  % Rotation matrix, 1st local frame
vv = R1*[0;1];  % Vector from local frame 1 to head
A = [q(1)+vv(1);q(2)+vv(2)];  % Location of head

WPdist = norm([A(1);A(2)]-[rwj(1);rwj(2)]);  % Distance between way points

u = [0;1];  % Unit vector, pointing at 0 degrees in global frame
v = rwj-rwi;  % vector between waypoints
slope = v(2)/v(1);  % slope of boundary line

% If the way points have the same y-coordinate, but different
% x-coordinates, trigger when the head passes the way-point's x-coord.
% + buffer, within a 0.5 margin
if v(2) == 0 && v(1) ~= 0
    
    % if WPdist <= WPtol  % Only acceptance circles
    % if abs(rwj(1)+buffer*sign(v(1)) - A(1)) <= 0.5;  % Only boundary lines
    if WPdist <= WPtol || abs(rwj(1)+CatchDistance*sign(v(1)) - A(1)) <= 0.5;  % Both
        switchWP = 0;  % Trigger
    else
        switchWP = 1;  % Don't Trigger
    end
% Else, trigger when the head passes the line perpdincular to the desired
% path + buffer, within a 0.5 margin
else
    slopeI = -1/slope;  % slope of boundary line
    Theta = atan2(v(2), v(1)) - atan2(u(2), u(1));  % Angle of v, from global
    h = f_RM(Theta)*[0;CatchDistance];  % a distance equal to the buffer past the
                                 % head of the snake, in the 1st local frame
    point = rwj + h;  % point through which boundary line passes
    b = point(2) - slopeI*point(1);  % y intercept of boundary line
    
    % if WPdist <= WPtol  % Only acceptance circles
    % if abs(A(2) - (slopeI*A(1)+b)) <= 0.5;  % Only boundary lines
    if WPdist <= WPtol || abs(A(2) - (slopeI*A(1)+b)) <= 0.5;  % Both
        switchWP = 0;  % Trigger
    else
        switchWP = 1;  % Don't Trigger
    end
    
end

end
