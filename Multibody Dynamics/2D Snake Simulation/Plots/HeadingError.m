% Heading Error

function HeadingError(Tend)

% Time and Error Definitions
numsteps = getGlobali;
%stepsize = Tend/numsteps;
Tvec = linspace(0,Tend,numsteps);
HeadingError = cell2mat(getGlobalHeadingError);

% Plot
figure;
plot(Tvec,HeadingError, 'black');
xlabel('Time (s)');
ylabel('Heading Error (radians))');
title('Heading Error');
grid on;

end