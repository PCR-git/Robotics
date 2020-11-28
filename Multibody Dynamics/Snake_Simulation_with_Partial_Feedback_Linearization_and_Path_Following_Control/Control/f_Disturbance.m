% Produces a disturbance force

function Disturbance = f_Disturbance(n,t,Disturb)

Mag = Disturb(1);  % Magnitude of disturbance
x = Disturb(2);  % x component of disturbance, from 0 to 1
y = Disturb(3);  % y component of disturbance, from 0 to 1
Type = Disturb(4);  % 1 = constant, 2 = sine wave
Begin = Disturb(5);  % Begin time
End = Disturb(6);  % End time

Base = zeros(n,1);

% x-components of disturbance
i = 1;
while i < n
    Base(i,1) = x*Mag;
    i = i+3;
end

% y-components of disturbance
j = 2;
while j < n
    Base(j,1) = y*Mag;
    j = j+3;
end

if t >= Begin && t <= End
    if Type == 1  % Constant disturbance
        Disturbance = Base;
    elseif Type == 2  % Sinusoidal disturbance
        Disturbance = Base*sin(t);
    end
else
    Disturbance = 0*Base;
end

end
