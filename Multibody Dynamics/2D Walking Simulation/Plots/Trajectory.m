%Trajectory Plot

function Trajectory(T,q,L1,WayPts,WPtol,buffer,Color)

% % Position Definitions
% x1 = q(:,1);  y1 = q(:,2);  phi1 = q(:,3);
% x2 = q(:,4);  y2 = q(:,5);  phi2 = q(:,6);
% x3 = q(:,7);  y3 = q(:,8);  phi3 = q(:,9);
% x4 = q(:,10); y4 = q(:,11); phi4 = q(:,12);
% x5 = q(:,13); y5 = q(:,14); phi5 = q(:,15);
% x6 = q(:,16); y6 = q(:,17); phi6 = q(:,18);

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
% figure
plot(pA(:,1),pA(:,2),Color,'Linewidth',1); % Head Trajec.
hold on;
%plot(x1,y1,'b');  % COM1 Trajec.
%hold on;
%plot(x2,y2,'r');  % COM2 Trajec.
%plot(x3,y3,'g');  % COM3 Trajec.
plot(wx1,wy1,'o','MarkerEdgeColor','black','MarkerFaceColor','black','MarkerSize', 6); % WP 1
plot(wx2,wy2,'o','MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0],'MarkerSize', 6); % WP 2
plot(wx3,wy3,'o','MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0],'MarkerSize', 6); % WP 3
plot(wx4,wy4,'o','MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0],'MarkerSize', 6); % WP 4
plot(xx12,yy12,'black--','LineWidth',1); % Desired Trajec. 1 to 2
plot(xx23,yy23,'black--','LineWidth',1); % Desired Trajec. 2 to 3
plot(xx34,yy34,'black--','LineWidth',1); % Desired Trajec. 3 to 4
% f_circle(wx1,wy1,WPtol,'r');  % Acceptance Circle 1
f_circle(wx2,wy2,WPtol,'k');  % Acceptance Circle 2
f_circle(wx3,wy3,WPtol,'k');  % Acceptance Circle 3
f_circle(wx4,wy4,WPtol,'k');  % Acceptance Circle 4
xlabel('{\itx}-position ({\itL})','FontSize',38,'FontName','Times New Roman');
ylabel('{\ity}-position ({\itL})','FontSize',38,'FontName','Times New Roman');
%title('Trajectories of Body Frames','FontSize',15);
% title('Snake Trajectory','FontSize',20,'FontName','Times New Roman');
% legend('Body 1','Body 2','Body 3');
% LEG = legend('1','2');
% set(LEG,'FontSize',38);

% Sets window size
%[x_low, x_high, y_low, y_high] = f_window_size(WayPts,buffer);
[x_low,x_high,y_low,y_high] = WindowSizer2(WayPts,buffer);
xlim([x_low x_high]);
ylim([y_low y_high]);

% set(gca, 'FontSize',38);
axis equal;
grid on;

end