% Norm of the constraints

function NormofConstraints(PHI_T)

figure
plot(PHI_T(:,1),PHI_T(:,2),'black');
%plot(PHI_T(:,1),(PHI_T(:,2)/2)*100,'black');
hold on;
xlabel('Time (s)','FontSize',15);
ylabel('Norm of Phi (m)','FontSize',15);
%ylabel('Error, as percent of link length','FontSize',15);
title('Norm of the Constraints vs. Time','FontSize',15);
set(gca, 'FontSize',15);
grid on;
hold on;

end