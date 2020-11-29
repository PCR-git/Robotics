function Disturbance = f_Disturbance(n,t,Disturb)

Mag = Disturb(1);
x = Disturb(2);
y = Disturb(3);
Type = Disturb(4);

Base = zeros(n,1);

i = 1;
while i < n
    Base(i,1) = x*Mag;
    i=i+3;
end

j = 2;
while j < n
    Base(j,1) = y*Mag;
    j=j+3;
end

if Type == 1
    Disturbance = Base;
elseif Type == 2
    if t > 30 && t < 40
        Disturbance = Base*heaviside(1);
    else
        Disturbance = 0*Base;
    end
elseif Type == 3
        Disturbance = Base*sin(t);
end

end