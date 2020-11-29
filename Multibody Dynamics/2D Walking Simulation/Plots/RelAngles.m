% Plots Relative Angles, for the two modules

function RelAngles(T,q,MaxAng)

% Angle, generalized coordinates
phi1 = q(:,3);
phi2 = q(:,6);
phi3 = q(:,9);
phi4 = q(:,12);
phi5 = q(:,15);
phi6 = q(:,18);
phi7 = q(:,21);
phi8 = q(:,24);

% Relative angles
%RelAng1 = phi2-phi1;
%RelAng2 = phi3-phi2;
RelAng3 = phi4-phi3;
% RelAng4 = phi5-phi4;
%RelAng5 = phi6-phi5;
% RelAng6 = phi7-phi6;
RelAng7 = phi8-phi7;

% Maximum angle
MaxAngVec = MaxAng*ones(1,length(T));

% Minimum angle
MinAngVec = -MaxAngVec;

% Plot of Relative Angles vs Time
figure
%plot(T, RelAng1,'go');
%hold on;
%plot(T, RelAng2,'r+');
plot(T, RelAng3,'b');
hold on;
%plot(T, RelAng4,'y*');
%plot(T, RelAng5,'bs');
%plot(T, RelAng6,'blackx');
plot(T, RelAng7,'r--');
plot(T, MaxAngVec,'black');
plot(T, MinAngVec,'black');
xlabel('Time (s)','FontSize',15);
ylabel('Relative Angles (Radians)','FontSize',15);
title('Relative Link Angles vs. Time','FontSize',15);
legend('Angle 1','Angle 2','UpperLim','LowerLim');
%legend('1','2','3','4','5','6','7');
set(gca, 'FontSize',15);
grid on;

end
