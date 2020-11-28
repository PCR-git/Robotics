% This function was used to find a polynomial approximatation
% to the curve given in Liljeback, et al. for the optimal phase
% shift between link angles, as a function of the number of links,
% in order to maximize the forward velocity of the snake robot.

% The reciprocal function given here remains within ~5% of the value
% of the actual curve throughout the range of 4-30+ links.

n = 30;         % Size of the x-axis
x = 0:1:30;     % x-values
y = zeros(1,n); % placeholder for y-values

% Function Parameters
a = 90;
b = 2.6;
c = 0.55;
d = 7;

i=1;
while i<=n+1
y(i) = a*real((x(i)-b)^-c) - d;   % Approximate function
i=i+1;
end
% Note: the above function was chosen by comparing visually
% with the function given by Liljeback, while the parameters
% a, b, c, & d were varied.

% Plot
plot(x,y)
xlim([0 30]);
ylim([0 90]);
grid on;
xlabel('Number of Links, N');
ylabel('Optimal Phase Shift');
title('Optimal Phase Shift to Maximize Forward Velocity');
