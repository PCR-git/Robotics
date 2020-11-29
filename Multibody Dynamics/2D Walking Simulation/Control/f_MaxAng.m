% Turn off motors if mechanism nears mechanical limits

function [TorqueM1Lim, TorqueM2Lim] = f_MaxAng(MaxAng,TorqueM1Lim,TorqueM2Lim,phi1,phi2,phi5,phi6)

% Angle limit, 2 degrees from mechanical limit
AngleLim = MaxAng-(2*(pi/180));

% Relative angles for each module
RelAng1 = phi2-phi1;
RelAng2 = phi6-phi5;

% If Module 1 nears mechanical limits
% and is trying to bend toward its limits
% turn off motor 1.
if RelAng1 >= AngleLim && TorqueM1Lim > 0
        TorqueM1Lim = 0;
elseif RelAng1 <= -AngleLim && TorqueM1Lim < 0
        TorqueM1Lim = 0;
end

% If Module 2 nears mechanical limits
% and is trying to bend toward its limits
% turn off motor 2.
if RelAng2 >= AngleLim && TorqueM2Lim > 0
        TorqueM2Lim = 0;
elseif RelAng2 <= -AngleLim && TorqueM2Lim < 0
        TorqueM2Lim = 0;
end

end
