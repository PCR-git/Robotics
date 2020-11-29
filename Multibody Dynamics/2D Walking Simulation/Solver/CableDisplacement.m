% Computes cable displacements, for modules A & B

function [DisplacementA, DisplacementB] = CableDisplacement(q,s,MechData)

% Radii of the pulley sections
Pradius1 = MechData(1); Pradius2 = MechData(2); Pradius3 = MechData(3);
% Radius of rotator links
LinkRadius = MechData(4);
% Distance from bottom of base to axis of pulley
PulleyDistance = MechData(5);

% Initial Cable Lengths
CableLength10 = MechData(6);
CableLength20 = MechData(7);
CableLength30 = MechData(8);

% Rotation Matrices
R4 = f_RM(q(12)+pi);
R8 = f_RM(q(24)+pi);

% Local Position Vectors
sA1 = s(:,2);
sB1 = s(:,3);
sC2 = s(:,5);
sD3 = s(:,7);
sE4 = s(:,9);
sF5 = s(:,11);
sG6 = s(:,13);
sH7 = s(:,15);
sI8 = s(:,17);
    
% Coordinates of Points
pA = f_r(q, 1, sA1);   % Coords of Pt A
pB = f_r(q, 1, sB1);   % Coords of Pt B
pC = f_r(q, 2, sC2);   % Coords of Pt C
pD = f_r(q, 3, sD3);   % Coords of Pt D
pE = f_r(q, 4, sE4);   % Coords of Pt E
pF = f_r(q, 5, sF5);   % Coords of Pt F
pG = f_r(q, 6, sG6);   % Coords of Pt G
pH = f_r(q, 7, sH7);   % Coords of Pt H
pI = f_r(q, 8, sI8);   % Coords of Pt I

% -------------------------------------------

% Center of multi-diameter pulley A
pP1 = pD + R4*[0;PulleyDistance];

% Center of multi-diameter pulley B
pP2 = pH + R8*[0;PulleyDistance];
    
% Exterior circles

% Circle centers
pCA1 = pA;
pCA2 = pB;
pCA3 = pC;
pCA4 = pD;

pCB1 = pE;
pCB2 = pF;
pCB3 = pG;
pCB4 = pH;

cut = 0*(pi/180); % What portion of the circular link to cut off,
                  % starting at -pi

% Cable Connection Points on pulley A
ApCable1LA = pP1-R4*[Pradius1;0];
ApCable1RA = pP1+R4*[Pradius1;0];
ApCable2LA = pP1-R4*[Pradius2;0];
ApCable2RA = pP1+R4*[Pradius2;0];
ApCable3LA = pP1-R4*[Pradius3;0];
ApCable3RA = pP1+R4*[Pradius3;0];

% Cable Connection Points on pulley B
BpCable1LA = pP2-R8*[Pradius1;0];
BpCable1RA = pP2+R8*[Pradius1;0];
BpCable2LA = pP2-R8*[Pradius2;0];
BpCable2RA = pP2+R8*[Pradius2;0];
BpCable3LA = pP2-R8*[Pradius3;0];
BpCable3RA = pP2+R8*[Pradius3;0];

% Angles of links, module A
ang1a = (q(3)-cut);
ang1b = (q(3)+cut);
ang2a = (q(6)-cut);
ang2b = (q(6)+cut);
ang3a = (q(9)-cut);
ang3b = (q(9)+cut);
ang4a = (q(12)-cut);
ang4b = (q(12)+cut);

% Angles of links, module B
ang5a = (q(15)-cut);
ang5b = (q(15)+cut);
ang6a = (q(18)-cut);
ang6b = (q(18)+cut);
ang7a = (q(21)-cut);
ang7b = (q(21)+cut);
ang8a = (q(24)-cut);
ang8b = (q(24)+cut);

% Connection points on rotators, module A
ApCableLB = pCA4-LinkRadius*(-1)*[cos(ang4a);sin(ang4a)];
ApCableRB = pCA4+LinkRadius*(-1)*[cos(ang4b);sin(ang4b)];
ApCableLC = pCA3-LinkRadius*(-1)*[cos(ang3a);sin(ang3a)];
ApCableRC = pCA3+LinkRadius*(-1)*[cos(ang3b);sin(ang3b)];
ApCableLD = pCA2-LinkRadius*(-1)*[cos(ang2a);sin(ang2a)];
ApCableRD = pCA2+LinkRadius*(-1)*[cos(ang2b);sin(ang2b)];
ApCableLE = pCA1-LinkRadius*(-1)*[cos(ang1a);sin(ang1a)];
ApCableRE = pCA1+LinkRadius*(-1)*[cos(ang1b);sin(ang1b)];

% Connection points on rotators, module B
BpCableLB = pCB4-LinkRadius*(-1)*[cos(ang8a);sin(ang8a)];
BpCableRB = pCB4+LinkRadius*(-1)*[cos(ang8b);sin(ang8b)];
BpCableLC = pCB3-LinkRadius*(-1)*[cos(ang7a);sin(ang7a)];
BpCableRC = pCB3+LinkRadius*(-1)*[cos(ang7b);sin(ang7b)];
BpCableLD = pCB2-LinkRadius*(-1)*[cos(ang6a);sin(ang6a)];
BpCableRD = pCB2+LinkRadius*(-1)*[cos(ang6b);sin(ang6b)];
BpCableLE = pCB1-LinkRadius*(-1)*[cos(ang5a);sin(ang5a)];
BpCableRE = pCB1+LinkRadius*(-1)*[cos(ang5b);sin(ang5b)];

% ------

% Line segments AB, BC, CD, DE, Module A
ACableLength1ABL = norm(ApCableLB - ApCable1LA);
ACableLength1ABR = norm(ApCableRB - ApCable1RA);
ACableLength2ABL = norm(ApCableLB - ApCable2LA);
ACableLength2ABR = norm(ApCableRB - ApCable2RA);
ACableLength3ABL = norm(ApCableLB - ApCable3LA);
ACableLength3ABR = norm(ApCableRB - ApCable3RA);
ACableLengthBCL = norm(ApCableLC - ApCableLB);
ACableLengthBCR = norm(ApCableRC - ApCableRB);
ACableLengthCDL = norm(ApCableLD - ApCableLC);
ACableLengthCDR = norm(ApCableRD - ApCableRC);
ACableLengthDEL = norm(ApCableLE - ApCableLD);
ACableLengthDER = norm(ApCableRE - ApCableRD);

% Line segments AB, BC, CD, DE, Module B
BCableLength1ABL = norm(BpCableLB - BpCable1LA);
BCableLength1ABR = norm(BpCableRB - BpCable1RA);
BCableLength2ABL = norm(BpCableLB - BpCable2LA);
BCableLength2ABR = norm(BpCableRB - BpCable2RA);
BCableLength3ABL = norm(BpCableLB - BpCable3LA);
BCableLength3ABR = norm(BpCableRB - BpCable3RA);
BCableLengthBCL = norm(BpCableLC - BpCableLB);
BCableLengthBCR = norm(BpCableRC - BpCableRB);
BCableLengthCDL = norm(BpCableLD - BpCableLC);
BCableLengthCDR = norm(BpCableRD - BpCableRC);
BCableLengthDEL = norm(BpCableLE - BpCableLD);
BCableLengthDER = norm(BpCableRE - BpCableRD);

% Arc length of rotator circles
CircumRot = 2*LinkRadius*pi/4;

% Cable Lengths, Module A
ACableLength1L = ACableLength1ABL+ACableLengthBCL+CircumRot;
ACableLength1R = ACableLength1ABR+ACableLengthBCR+CircumRot;
ACableLength2L = ACableLength2ABL+ACableLengthBCL+ACableLengthCDL+CircumRot;
ACableLength2R = ACableLength2ABR+ACableLengthBCR+ACableLengthCDR+CircumRot;
ACableLength3L = ACableLength3ABL+ACableLengthBCL+ACableLengthCDL+ACableLengthDEL+LinkRadius;
ACableLength3R = ACableLength3ABR+ACableLengthBCR+ACableLengthCDR+ACableLengthDER+LinkRadius;

% Cable Lengths, Module B
BCableLength1L = BCableLength1ABL+BCableLengthBCL+CircumRot;
BCableLength1R = BCableLength1ABR+BCableLengthBCR+CircumRot;
BCableLength2L = BCableLength2ABL+BCableLengthBCL+BCableLengthCDL+CircumRot;
BCableLength2R = BCableLength2ABR+BCableLengthBCR+BCableLengthCDR+CircumRot;
BCableLength3L = BCableLength3ABL+BCableLengthBCL+BCableLengthCDL+BCableLengthDEL+LinkRadius;
BCableLength3R = BCableLength3ABR+BCableLengthBCR+BCableLengthCDR+BCableLengthDER+LinkRadius;

% Cable Displacement, Module A
ACableDisplacement1L = ACableLength1L-CableLength10;
ACableDisplacement1R = ACableLength1R-CableLength10;
ACableDisplacement2L = ACableLength2L-CableLength20;
ACableDisplacement2R = ACableLength2R-CableLength20;
ACableDisplacement3L = ACableLength3L-CableLength30;
ACableDisplacement3R = ACableLength3R-CableLength30;

% Cable Displacement, Module B
BCableDisplacement1L = BCableLength1L-CableLength10;
BCableDisplacement1R = BCableLength1R-CableLength10;
BCableDisplacement2L = BCableLength2L-CableLength20;
BCableDisplacement2R = BCableLength2R-CableLength20;
BCableDisplacement3L = BCableLength3L-CableLength30;
BCableDisplacement3R = BCableLength3R-CableLength30;

% ------

% Only the cables under tension exert an elastic force.
% Determining which cables are under tension:
% Module A:

if ACableDisplacement1L > ACableDisplacement1R
    DisplacementA1short = ACableDisplacement1R;
    DisplacementA1long  = ACableDisplacement1L;
elseif ACableDisplacement1L < ACableDisplacement1R
    DisplacementA1short = ACableDisplacement1L;
    DisplacementA1long  = ACableDisplacement1R;
else
    DisplacementA1short = 0;
    DisplacementA1long  = 0;
end

if ACableDisplacement2L > ACableDisplacement2R
    DisplacementA2short = ACableDisplacement2R;
    DisplacementA2long  = ACableDisplacement2L;
elseif ACableDisplacement2L < ACableDisplacement2R
    DisplacementA2short = ACableDisplacement2L;
    DisplacementA2long  = ACableDisplacement2R;
else
    DisplacementA2short = 0;
    DisplacementA2long  = 0;
end

if ACableDisplacement3L > ACableDisplacement3R
    DisplacementA3short = ACableDisplacement3R;
    DisplacementA3long  = ACableDisplacement3L;
elseif ACableDisplacement3L < ACableDisplacement3R
    DisplacementA3short = ACableDisplacement3L;
    DisplacementA3long  = ACableDisplacement3R;
else
    DisplacementA3short = 0;
    DisplacementA3long  = 0;
end

% -------
% Module 2:

if BCableDisplacement1L > BCableDisplacement1R
    DisplacementB1short = BCableDisplacement1R;
    DisplacementB1long  = -BCableDisplacement1L;
elseif BCableDisplacement1L < BCableDisplacement1R
    DisplacementB1short = -BCableDisplacement1L;
    DisplacementB1long  = BCableDisplacement1R;
else
    DisplacementB1short = 0;
    DisplacementB1long  = 0;
end

if BCableDisplacement2L > BCableDisplacement2R
    DisplacementB2short = BCableDisplacement2R;
    DisplacementB2long  = -BCableDisplacement2L;
elseif BCableDisplacement2L < BCableDisplacement2R
    DisplacementB2short = -BCableDisplacement2L;
    DisplacementB2long  = BCableDisplacement2R;
else
    DisplacementB2short = 0;
    DisplacementB2long  = 0;
end

if BCableDisplacement3L > BCableDisplacement3R
    DisplacementB3short = BCableDisplacement3R;
    DisplacementB3long  = -BCableDisplacement3L;
elseif BCableDisplacement3L < BCableDisplacement3R
    DisplacementB3short = -BCableDisplacement3L;
    DisplacementB3long  = BCableDisplacement3R;
else
    DisplacementB3short = 0;
    DisplacementB3long  = 0;
end

% Cable Displacements
DisplacementAshort = [DisplacementA3short,DisplacementA2short,DisplacementA1short];
DisplacementBshort = [DisplacementB3short,DisplacementB2short,DisplacementB1short];

DisplacementAlong = [DisplacementA3long,DisplacementA2long,DisplacementA1long];
DisplacementBlong = [DisplacementB3long,DisplacementB2long,DisplacementB1long];

DisplacementA = [DisplacementAshort; DisplacementAlong];
DisplacementB = [DisplacementBshort; DisplacementBlong];

end
