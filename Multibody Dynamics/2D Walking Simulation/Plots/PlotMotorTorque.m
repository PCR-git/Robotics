% Plots total motor torques

function PlotMotorTorque(Tend,GlobalTorque)

% Time Definition
numsteps = getGlobali;
%stepsize = Tend/numsteps;
Tvec = linspace(0,Tend,numsteps);

% Torque global variable
Torque = cell2mat(GlobalTorque);

% Torques for each motor
TorqueM1 = transpose(Torque(:,1));
TorqueM2 = transpose(Torque(:,2));

% --------

% Plot

figure;
plot(Tvec,TorqueM1, 'b');
hold on;
plot(Tvec,TorqueM2, 'r--');

xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Motor Torques');
legend('Motor 1', 'Motor 2');
grid on;

end
