% Heading (link angle avg.)

function Heading(n,T,q)

% Stores the (global) link angles (in radians)
m1 = length(q(:,1));
Ang = zeros(m1,n/3);
kk = 3;
while kk <= n
    Ang(:,kk/3) = q(:,kk);
    kk = kk+3;
end
%Angd = Ang*(180/pi);

% Stores the average of the (global) link angles (in radians)
avgh = zeros(m1,1);
ll = 1;
while ll <= m1
    avgh(ll,1) = sum(Ang(ll,:))/(n/3);
    ll = ll+1;
end
avghd = avgh*(180/pi);

% Plot
figure
plot(T,avghd,'black');
xlabel('Time (s)','FontSize',15);
ylabel('Heading (Degrees)','FontSize',15);
title('Heading vs Time');
set(gca, 'FontSize',15);
grid on;

end
