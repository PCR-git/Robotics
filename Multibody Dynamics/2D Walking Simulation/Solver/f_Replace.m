% f_Replace
% If the last waypoint was missed, replace the time vector, position
% vector, end time, end position, jacobian, and acceleration vector
% with zeros, so simulation can be restarted.

function [Tnew,Ynew,Tenew,Yenew,PHI_Tnew,qppnew] = f_Replace(Told,Yold,Teold,Yeold,PHI_Told,qppold)

Tnew = 0*Told;
Ynew = 0*Yold;
Tenew = 0*Teold;
Yenew = 0*Yeold;
PHI_Tnew = 0*PHI_Told;
qppnew = 0*qppold;

end