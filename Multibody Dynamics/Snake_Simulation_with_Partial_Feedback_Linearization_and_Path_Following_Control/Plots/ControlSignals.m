% Control Signals

function ControlSignals(Tend)

% Time and Control Signal Definitions
numsteps = getGlobali;
%stepsize = Tend/numsteps;
Tvec = linspace(0,Tend,numsteps);
U = cell2mat(getGlobalU);
u1vec = transpose(U(:,1));
u2vec = transpose(U(:,2));

% Plot
figure;
plot(Tvec,u1vec, 'b');
hold on;
plot(Tvec,u2vec, 'r');
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Control Signals (Motor Torques)');
legend('Theta1', 'Theta2');
grid on;

end