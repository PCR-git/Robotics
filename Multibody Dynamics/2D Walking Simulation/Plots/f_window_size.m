% Sets Window Size for Animation & Trajectory Plot

function [x_low, x_high, y_low, y_high] = f_window_size(WayPts,buffer)

% Position vectors of the way points
rw1 = WayPts(:,1);
rw2 = WayPts(:,2);
rw3 = WayPts(:,3);
rw4 = WayPts(:,4);

% Norms of the way point position vectors
Norms = [norm(rw1),norm(rw2),norm(rw3),norm(rw4)];
MaxNorms = max(Norms);

% Finds the waypoint furthest from origin
if MaxNorms == Norms(1);
    FarPoint = rw1;
elseif MaxNorms == Norms(2);
    FarPoint = rw2;
elseif MaxNorms == Norms(3);
    FarPoint = rw3;
elseif MaxNorms == Norms(4);
    FarPoint = rw4;
else
    error('wtf');
end

% Way Point Furthest from Origin
x_far = FarPoint(1);
y_far = FarPoint(2);
    
% Sets the size of the window
if y_far >= 0 && x_far >= 0
    if y_far >= x_far
        y_high = y_far+buffer;
        y_low  = -buffer;
        x_high = x_far + (y_high-y_low)/2;
        x_low  = x_far - (y_high-y_low)/2;
    else
        x_high = x_far+buffer;
        x_low  = -buffer;
        y_high = y_far + (x_high-x_low)/2;
        y_low  = y_far - (x_high-x_low)/2;
    end
elseif y_far >= 0 && x_far < 0
    if y_far >= -x_far
        y_high = y_far+buffer;
        y_low  = -buffer;
        x_high = x_far + (y_high-y_low)/2;
        x_low  = x_far - (y_high-y_low)/2;
    else
        x_high = buffer;
        x_low  = x_far-buffer;
        y_high = y_far + (x_high-x_low)/2;
        y_low  = y_far - (x_high-x_low)/2;
    end
elseif y_far < 0 && x_far >= 0
    if -y_far >= x_far
        y_high = buffer;
        y_low  = y_far-buffer;
        x_high = x_far + (y_high-y_low)/2;
        x_low  = x_far - (y_high-y_low)/2;
    else
        x_high = x_far+buffer;
        x_low  = -buffer;
        y_high = y_far + (x_high-x_low)/2;
        y_low  = y_far - (x_high-x_low)/2;
    end    
elseif y_far < 0 && x_far < 0
    if -y_far >= -x_far
        y_high = buffer;
        y_low  = y_far-buffer;
        x_high = x_far + (y_high-y_low)/2;
        x_low  = x_far - (y_high-y_low)/2;
    else
        x_high = buffer;
        x_low  = x_far-buffer;
        y_high = y_far + (x_high-x_low)/2;
        y_low  = y_far - (x_high-x_low)/2;
    end
end

end