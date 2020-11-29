% Average x and y velocities (for straight motion)

function AvgVel(n,T,qp)

% Stores the y components of the link velocities
[m1,~] = size(qp);
%Velx = zeros(m1,n/3);
Vely = zeros(m1,n/3);
mm = 1;
while mm <= n
    %Velx(:,(mm+2)/3) = qp(:,mm);
    Vely(:,(mm+2)/3) = qp(:,mm+1);
    mm=mm+3;
end

% Stores the average y velocities
%avgvx = zeros(m1,1);
avgvy = zeros(m1,1);
nn = 1;
while nn <= m1
    %avgvx(nn,1) = sum(Velx(nn,:))/(n/3);
    avgvy(nn,1) = sum(Vely(nn,:))/(n/3);
    nn = nn+1;
end

SteadVel = mean(avgvy(end-100:end,1))
SteadyState = SteadVel*ones(1,length(T));

% Plot
figure
%plot(T, avgvy,'b');
%hold on;
plot(T, avgvy,'b');
hold on;
plot(T,SteadyState,'k');
xlabel('Time (s)','FontSize',15);
ylabel('Average Y-Velocity','FontSize',15);
title('Average Y-Velocity vs Time','FontSize',15);
%legend('x-vel','y-vel');
set(gca, 'FontSize',15);
grid on;

end
