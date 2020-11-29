function TauM = f_Loading(alpha,M,g,LVec,Ratio,MechData)

% Length Definitions
L1 = LVec(1);
L2 = LVec(2);
L3 = LVec(3);
L4 = LVec(4);

% Radius of rotator links
LinkRadius = MechData(4);

T1 = Ratio(4);
T2 = Ratio(5);
T3 = Ratio(6);
Pradius1 = MechData(1);
Pradius2 = MechData(2);
Pradius3 = MechData(3);

thetap = -alpha/(T1+T2+T3);
theta1 = T1*thetap;
theta2 = T2*thetap;
theta3 = T3*thetap;

% Taug3 = M(10,10)*g*L4*cos(alpha)+...
%         M(7,7)*g*L3*cos(alpha+theta3)+...
%         M(4,4)*g*(2*L3*cos(alpha+theta3)+L2*cos(alpha+theta3+theta2))+...
%         M(1,1)*g*(2*L3*cos(alpha+theta3)+2*L2*cos(alpha+theta3+theta2)+L1*cos(alpha+theta3+theta2+theta1));
% Taug2 = M(7,7)*g*L3*cos(alpha+theta3)+...
%         M(4,4)*g*L2*cos(alpha+theta3+theta2)+...
%         M(1,1)*g*(2*L2*cos(alpha+theta3+theta2)+L1*cos(alpha+theta3+theta2+theta1));
% Taug1 = M(4,4)*g*L2*cos(alpha+theta3+theta2)+...
%         M(1,1)*g*L1*cos(alpha+theta3+theta2+theta1);

Taug3 = M(7,7)*g*L3*cos(alpha+theta3)+...
        M(4,4)*g*(2*L3*cos(alpha+theta3)+L2*cos(alpha+theta3+theta2))+...
        M(1,1)*g*(2*L3*cos(alpha+theta3)+2*L2*cos(alpha+theta3+theta2)+L1*cos(alpha+theta3+theta2+theta1));
Taug2 = M(4,4)*g*L2*cos(alpha+theta3+theta2)+...
        M(1,1)*g*(2*L2*cos(alpha+theta3+theta2)+L1*cos(alpha+theta3+theta2+theta1));
Taug1 = M(1,1)*g*L1*cos(alpha+theta3+theta2+theta1);

Tau3 = Taug3;

Tau2 = Taug2 + Tau3;

Tau1 = Taug1 + Tau2 + Tau3;

Tension1 = Tau1/LinkRadius;
Tension2 = Tau2/LinkRadius;
Tension3 = Tau3/LinkRadius;

TauM = Tension1*Pradius1 + Tension2*Pradius2 + Tension3*Pradius3;

end