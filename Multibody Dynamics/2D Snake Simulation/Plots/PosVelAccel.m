% Plots the following:
% 1.) Positions vs Time
% 2.) Link Angles vs Time
% 3.) Velocities vs Time
% 4.) Angular Velocities vs Time
% 5.) Accelerations vs Time
% 6.) Accelerations vs Time

function PosVelAccel(T,q,qp,qpp,Activation)

% Positions Definitions
x1 = q(:,1); y1 = q(:,2); phi1 = q(:,3);
x2 = q(:,4); y2 = q(:,5); phi2 = q(:,6);
x3 = q(:,7); y3 = q(:,8); phi3 = q(:,9);

% Velocity Definitions
x1p = qp(:,1); y1p = qp(:,2); phi1p = qp(:,3);
x2p = qp(:,4); y2p = qp(:,5); phi2p = qp(:,6);
x3p = qp(:,7); y3p = qp(:,8); phi3p = qp(:,9);

% Plots:

% Positions vs Time
if Activation(1) == 1
    figure
    plot(T,x1,'b');
    hold on;
    plot(T,y1,'r');
    plot(T,x2,'g');
    plot(T,y2,'black');
    plot(T,x3, 'y');
    plot(T,y3, 'm');
    xlabel('Time (s)','FontSize',15);
    ylabel('X and Y Positions (m)','FontSize',15);
    title('X and Y Positions of COMs vs. Time','FontSize',15);
    legend('x1','y1','x2','y2','x3','y3');
    set(gca, 'FontSize',15);
    grid on;
end

% Link Angles vs Time
if Activation(2) == 1
    figure
    plot(T, phi1,'b');
    hold on;
    plot(T, phi2,'r');
    plot(T, phi3,'g');
    xlabel('Time (s)','FontSize',15);
    ylabel('Angle (Radians)','FontSize',15);
    title('Link Angles vs. Time','FontSize',15);
    legend('phi1','phi2','phi3');
    set(gca, 'FontSize',15);
    grid on;
end

% Velocities vs Time
if Activation(3) == 1
    figure
    plot(T,x1p,'b');
    hold on;
    plot(T,y1p,'r');
    plot(T,x2p,'g');
    plot(T,y2p,'black');
    plot(T,x3p, 'y');
    plot(T,y3p, 'm');
    xlabel('Time (s)','FontSize',15);
    ylabel('X and Y Velocities (m/s)','FontSize',15);
    title('X and Y Velocities of COMs vs. Time','FontSize',15);
    legend('x1','y1','x2','y2','x3','y3');
    set(gca, 'FontSize',15);
    grid on;
end

% Angular Velocities vs Time
if Activation(4) == 1
    figure
    plot(T, phi1p,'b');
    hold on;
    plot(T, phi2p,'r');
    plot(T, phi3p,'g');
    xlabel('Time (s)','FontSize',15);
    ylabel('Angular Velocity (Hz)','FontSize',15);
    title('Angular Velocities of COMs vs. Time','FontSize',15);
    legend('phi1','phi2','phi2');
    set(gca, 'FontSize',15);
    grid on;
end

% Accelerations vs Time
if Activation(5) == 1
    figure;
    plot(T, qpp(:, 1), 'b');
    hold on;
    plot(T, qpp(:, 2), 'r');
    plot(T, qpp(:, 4), 'g');
    plot(T, qpp(:, 5), 'black');
    plot(T, qpp(:, 7), 'y');
    plot(T, qpp(:, 8), 'm');
    xlabel('Time t in (s)');
    ylabel('Linear Acceleration (m/s^2)');
    title('Linear Accelerations of COMs vs. Time');
    legend('x1','y1','x2','y2','x3','y3');
    grid on;
end

% Angular Accelerations vs Time
if Activation(6) == 1
    figure;
    plot(T, qpp(:, 3), 'b');
    hold on;
    plot(T, qpp(:, 6), 'r');
    plot(T, qpp(:, 9), 'g');
    xlabel('Time (s)');
    ylabel('Angle (1/s^2)');
    title('Angular Acceleration');
    legend('phi1', 'phi2', 'phi3');
    grid on;
end

end
