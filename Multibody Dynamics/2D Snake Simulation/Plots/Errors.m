% Errors

function Errors(Tend)

% Time and Error Definitions
numsteps = getGlobali;
%stepsize = Tend/numsteps;
Tvec = linspace(0,Tend,numsteps);
Error = cell2mat(getGlobalError);
Error1vec = transpose(Error(:,1));
Error2vec = transpose(Error(:,2));

% Plot
figure;
plot(Tvec,Error1vec, 'b');
hold on;
plot(Tvec,Error2vec, 'r');
xlabel('Time (s)');
ylabel('Error (m))');
title('Errors');
legend('Theta1', 'Theta2');
grid on;

end