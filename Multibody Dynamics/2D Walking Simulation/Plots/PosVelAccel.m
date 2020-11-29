% Plots the following:
% 1.) X Positions vs Time
% 2.) Y Positions vs Time
% 3.) Link Angles vs Time
% 4.) X Velocities vs Time
% 5.) Y Velocities vs Time
% 6.) Angular Velocities vs Time
% 7.) X Accelerations vs Time
% 8.) Y Accelerations vs Time
% 9.) Angular Acclerations vs Time

function PosVelAccel(T,q,qp,qpp,Activation)

% Positions Definitions
x1 = q(:,1);  y1 = q(:,2);  phi1 = q(:,3);
x2 = q(:,4);  y2 = q(:,5);  phi2 = q(:,6);
x3 = q(:,7);  y3 = q(:,8);  phi3 = q(:,9);
x4 = q(:,10); y4 = q(:,11); phi4 = q(:,12);
x5 = q(:,13); y5 = q(:,14); phi5 = q(:,15);
x6 = q(:,16); y6 = q(:,17); phi6 = q(:,18);
x7 = q(:,19); y7 = q(:,20); phi7 = q(:,21);
x8 = q(:,22); y8 = q(:,23); phi8 = q(:,24);

% Velocity Definitions
x1p = qp(:,1);  y1p = qp(:,2);  phi1p = qp(:,3);
x2p = qp(:,4);  y2p = qp(:,5);  phi2p = qp(:,6);
x3p = qp(:,7);  y3p = qp(:,8);  phi3p = qp(:,9);
x4p = qp(:,10); y4p = qp(:,11); phi4p = qp(:,12);
x5p = qp(:,13); y5p = qp(:,14); phi5p = qp(:,15);
x6p = qp(:,16); y6p = qp(:,17); phi6p = qp(:,18);
x7p = qp(:,19); y7p = qp(:,20); phi7p = qp(:,21);
x8p = qp(:,22); y8p = qp(:,23); phi8p = qp(:,24);

% Acceleration Definitions
x1pp = qpp(:,1);  y1pp = qpp(:,2);  phi1pp = qpp(:,3);
x2pp = qpp(:,4);  y2pp = qpp(:,5);  phi2pp = qpp(:,6);
x3pp = qpp(:,7);  y3pp = qpp(:,8);  phi3pp = qpp(:,9);
x4pp = qpp(:,10); y4pp = qpp(:,11); phi4pp = qpp(:,12);
x5pp = qpp(:,13); y5pp = qpp(:,14); phi5pp = qpp(:,15);
x6pp = qpp(:,16); y6pp = qpp(:,17); phi6pp = qpp(:,18);
x7pp = qpp(:,19); y7pp = qpp(:,20); phi7pp = qpp(:,21);
x8pp = qpp(:,22); y8pp = qpp(:,23); phi8pp = qpp(:,24);

% Plots:

% X Positions vs Time
if Activation(1) == 1
    figure
    plot(T,x1,'b--');
    hold on;
    plot(T,x2,'r+');
    plot(T,x3,'go');
    plot(T,x4,'y*');
    plot(T,x5,'m.');
    plot(T,x6,'blackx');
    plot(T,x7,'bs');
    plot(T,x8,'rd');
    xlabel('Time (s)','FontSize',15);
    ylabel('X Coords (m)','FontSize',15);
    title('X Coordinates of COMs vs. Time','FontSize',15);
    legend('x1','x2','x3','x4','x5','x6','x7','x8');
    set(gca, 'FontSize',15);
    grid on;
end

% Y Positions vs Time
if Activation(2) == 1
    figure
    plot(T,y1,'b--');
    hold on;
    plot(T,y2,'r+');
    plot(T,y3,'go');
    plot(T,y4,'y*');
    plot(T,y5,'m.');
    plot(T,y6,'blackx');
    plot(T,y7,'bs');
    plot(T,y8,'rd');
    xlabel('Time (s)','FontSize',15);
    ylabel('Y Coords (m)','FontSize',15);
    title('Y Coordinates of COMs vs. Time','FontSize',15);
    legend('y1','y2','y3','y4','y5','y6','y7','y8');
    set(gca, 'FontSize',15);
    grid on;
end

% Link Angles vs Time
if Activation(3) == 1
    figure
    plot(T, phi1,'b--');
    hold on;
    %plot(T, phi2,'r+');
    %plot(T, phi3,'go');
    %plot(T, phi4,'y*');
    plot(T, phi5,'r.');
    %plot(T, phi6,'blackx');
    %plot(T, phi7,'bs');
    %plot(T, phi8,'rd');
    xlabel('Time (s)','FontSize',15);
    ylabel('Angle (Radians)','FontSize',15);
    title('Link Angles vs. Time','FontSize',15);
    legend('phi1','phi2','phi3');
    set(gca, 'FontSize',15);
    grid on;
end

% X Velocities vs Time
if Activation(4) == 1
    figure
    plot(T,x1p,'b--');
    hold on;
    plot(T,x2p,'r+');
    plot(T,x3p,'go');
    plot(T,x4p,'y*');
    plot(T,x5p,'m.');
    plot(T,x6p,'blackx');
    plot(T,x7p,'bs');
    plot(T,x8p,'rd');
    xlabel('Time (s)','FontSize',15);
    ylabel('X Velocities (m/s)','FontSize',15);
    title('X Velocities of COMs vs. Time','FontSize',15);
    legend('x1','x2','x3','x4','x5','x6','x7','x8');
    set(gca, 'FontSize',15);
    grid on;
end

% Y Velocities vs Time
if Activation(5) == 1
    figure
    plot(T,y1p,'b--');
    hold on;
    plot(T,y2p,'r+');
    plot(T,y3p,'go');
    plot(T,y4p,'y*');
    plot(T,y5p,'m.');
    plot(T,y6p,'blackx');
    plot(T,y7p,'bs');
    plot(T,y8p,'rd');
    xlabel('Time (s)','FontSize',15);
    ylabel('Y Velocities (m/s)','FontSize',15);
    title('Y Velocities of COMs vs. Time','FontSize',15);
    legend('y1','y2','y3','y4','y5','y6','y7','y8');
    set(gca, 'FontSize',15);
    grid on;
end

% Angular Velocities vs Time
if Activation(6) == 1
    figure
    plot(T, phi1p,'b--');
    hold on;
    %plot(T, phi2p,'r+');
    %plot(T, phi3p,'go');
    %plot(T, phi4p,'y*');
    plot(T, phi5p,'r.');
    %plot(T, phi6p,'blackx');
    %plot(T, phi7p,'bs');
    %plot(T, phi8p,'rd');
    xlabel('Time (s)','FontSize',15);
    ylabel('Angular Velocity (Hz)','FontSize',15);
    title('Angular Velocities of COMs vs. Time','FontSize',15);
    legend('phi1','phi2');
    set(gca, 'FontSize',15);
    grid on;
end

% X Accelerations vs Time
if Activation(7) == 1
    figure;
    plot(T,x1pp,'b--');
    hold on;
    plot(T,x2pp,'r+');
    plot(T,x3pp,'go');
    plot(T,x4pp,'y*');
    plot(T,x5pp,'m.');
    plot(T,x6pp,'blackx');
    plot(T,x7pp,'bs');
    plot(T,x8pp,'rd');
    xlabel('Time t in (s)');
    ylabel('Acceleration (m/s^2)');
    title('X Accelerations of COMs vs. Time');
    legend('x1','x2','x3','x4','x5','x6','x7','x8');
    grid on;
end

% Y Accelerations vs Time
if Activation(8) == 1
    figure;
    plot(T,y1pp,'b--');
    hold on;
    plot(T,y2pp,'r+');
    plot(T,y3pp,'go');
    plot(T,y4pp,'y*');
    plot(T,y5pp,'m.');
    plot(T,y6pp,'blackx');
    plot(T,y7pp,'bs');
    plot(T,y8pp,'rd');
    xlabel('Time t in (s)');
    ylabel('Acceleration (m/s^2)');
    title('Y Accelerations of COMs vs. Time');
    legend('y1','y2','y3','y4','y5','y6','y7','y8');
    grid on;
end

% Angular Accelerations vs Time
if Activation(9) == 1
    figure;
      plot(T, phi1pp,'b--');
    hold on;
    %plot(T, phi2pp,'r+');
    %plot(T, phi3pp,'go');
    %plot(T, phi4pp,'y*');
    plot(T, phi5pp,'r.');
    %plot(T, phi6pp,'blackx');
    %plot(T, phi7pp,'bs');
    %plot(T, phi8pp,'rd');
    xlabel('Time (s)');
    ylabel('Angle (1/s^2)');
    title('Angular Acceleration');
    legend('phi1', 'phi2');
    grid on;
end

end
