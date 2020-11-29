% Computes cable displacements, for modules A & B

function [DisplacementA, DisplacementB] = CableDisplacement2(q,MechData)

% Radius of rotator links
LinkRadius = MechData(4);

% Angle Definitions
phi1 = q(3);
phi2 = q(6);
phi3 = q(9);
phi4 = q(12);
phi5 = q(15);
phi6 = q(18);
phi7 = q(21);
phi8 = q(24);

theta1 = phi2-phi1;
theta2 = phi3-phi2;
theta3 = phi4-phi3;

theta4 = phi6-phi5;
theta5 = phi7-phi6;
theta6 = phi8-phi7;

% Cable Disp.
C1 = theta1*LinkRadius;
C2 = theta2*LinkRadius;
C3 = theta3*LinkRadius;

C4 = theta4*LinkRadius;
C5 = theta5*LinkRadius;
C6 = theta6*LinkRadius;

CA = [C1,C2,C3];
CB = [C4,C5,C6];

DisplacementA = [-CA; CA];
DisplacementB = [-CB; CB];

end
