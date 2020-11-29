%Trajectory Plot

function Trajectory(T,q,L1,WayPts,WPtol,buffer)

% Position Definitions
x1 = q(:,1); y1 = q(:,2); phi1 = q(:,3);
x2 = q(:,4); y2 = q(:,5); phi2 = q(:,6);
x3 = q(:,7); y3 = q(:,8); phi3 = q(:,9);

% Head Trajectory
s_pa = [0; L1];
pA = zeros(length(T),2);
jj = 1;
while jj <= length(T)
    pA(jj,:) = transpose(f_r(q(jj,:), 1, s_pa));   % Coords of Pt A (head)
    jj = jj+1;
end

% Position vectors of each way point
rw1 = WayPts(:,1);
rw2 = WayPts(:,2);
rw3 = WayPts(:,3);
rw4 = WayPts(:,4);

% Way Point Coordinates
wx1 = rw1(1); wy1 = rw1(2);
wx2 = rw2(1); wy2 = rw2(2);
wx3 = rw3(1); wy3 = rw3(2);
wx4 = rw4(1); wy4 = rw4(2);

% Desired Trajectories (between way points) (for plotting)
xx12 = linspace(wx1,wx2,50); yy12 = linspace(wy1,wy2,50);  % 1 to 2
xx23 = linspace(wx2,wx3,50); yy23 = linspace(wy2,wy3,50);  % 2 to 3
xx34 = linspace(wx3,wx4,50); yy34 = linspace(wy3,wy4,50);  % 3 to 4

% Plot
figure
plot(pA(:,1),pA(:,2),'b'); % Head Trajec.
hold on;
%plot(x1,y1,'b');  % COM1 Trajec.
%hold on;
%plot(x2,y2,'r');  % COM2 Trajec.
%plot(x3,y3,'g');  % COM3 Trajec.
plot(wx1,wy1,'bo','LineWidth', 3); % WayPoint 1
plot(wx2,wy2,'ro','LineWidth', 3); % WayPoint 2
plot(wx3,wy3,'go','LineWidth', 3); % WayPoint 3
plot(wx4,wy4,'mo','LineWidth', 3); % WayPoint 4
plot(xx12,yy12,'black--','LineWidth',1); % Desired Trajec. 1 to 2
plot(xx23,yy23,'black--','LineWidth',1); % Desired Trajec. 2 to 3
plot(xx34,yy34,'black--','LineWidth',1); % Desired Trajec. 3 to 4
f_circle(wx1,wy1,WPtol,'b');  % Acceptance Circle 1
f_circle(wx2,wy2,WPtol,'r');  % Acceptance Circle 2
f_circle(wx3,wy3,WPtol,'g');  % Acceptance Circle 3
f_circle(wx4,wy4,WPtol,'m');  % Acceptance Circle 4
xlabel('X Position','FontSize',20);
ylabel('Y Position','FontSize',20);
%title('Trajectories of Body Frames','FontSize',15);
title('Snake Trajectory','FontSize',20);
%legend('Body 1','Body 2','Body 3');

% Sets window size
[x_low, x_high, y_low, y_high] = f_window_size(WayPts,buffer);
xlim([x_low x_high]);
ylim([y_low y_high]);
  
%set(gca, 'FontSize',15);
axis equal;
grid on;

end