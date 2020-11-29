% Computes Initial Cable Lengths

function CableLength0 = InitialCableLengths(LVec,s,MechData,Ratio)

% Local Position Vectors
sA1 = s(:,2);
sB1 = s(:,3);
sC2 = s(:,5);
sD3 = s(:,7);

% Radii
Pradius1 = MechData(1);
Pradius2 = MechData(2);
Pradius3 = MechData(3);
LinkRadius = MechData(4);
PulleyDistance = MechData(5);

% Initial Position Vector
q0 = f_InitialPosition(0,LVec,[0;0],Ratio);
    
% Rotation Matrices
R4 = f_RM(q0(12)+pi);
    
% Coordinates of Points
pA = f_r(q0, 1, sA1);    % Coords of Pt A
pB = f_r(q0, 1, sB1);   % Coords of Pt B
pC = f_r(q0, 2, sC2);   % Coords of Pt C
pD = f_r(q0, 3, sD3);   % Coords of Pt D

% Center of multi-diameter pulley 1
pP1 = pD + R4*[0;PulleyDistance];
   
% Plotting exterior circles
pCA1 = pA;
pCA2 = pB;
pCA3 = pC;
pCA4 = pD;

cut = 0*(pi/180);

% Cable Connection Points on pulley A
ApCable1LA = pP1+R4*[Pradius1;0];
ApCable1RA = pP1-R4*[Pradius1;0];
ApCable2LA = pP1+R4*[Pradius2;0];
ApCable2RA = pP1-R4*[Pradius2;0];
ApCable3LA = pP1+R4*[Pradius3;0];
ApCable3RA = pP1-R4*[Pradius3;0];

% Angles of links, module A
ang1a = (q0(3)-cut);
ang1b = (q0(3)+cut);
ang2a = (q0(6)-cut);
ang2b = (q0(6)+cut);
ang3a = (q0(9)-cut);
ang3b = (q0(9)+cut);
ang4a = (q0(12)-cut);
ang4b = (q0(12)+cut);

% Connection points on rotators, module A
ApCableLB = pCA4+LinkRadius*(-1)*[cos(ang4a);sin(ang4a)];
ApCableRB = pCA4-LinkRadius*(-1)*[cos(ang4b);sin(ang4b)];
ApCableLC = pCA3+LinkRadius*(-1)*[cos(ang3a);sin(ang3a)];
ApCableRC = pCA3-LinkRadius*(-1)*[cos(ang3b);sin(ang3b)];
ApCableLD = pCA2+LinkRadius*(-1)*[cos(ang2a);sin(ang2a)];
ApCableRD = pCA2-LinkRadius*(-1)*[cos(ang2b);sin(ang2b)];
ApCableLE = pCA1+LinkRadius*(-1)*[cos(ang1a);sin(ang1a)];
ApCableRE = pCA1-LinkRadius*(-1)*[cos(ang1b);sin(ang1b)];

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

% Arc length of rotator circles
CircumRot = 2*LinkRadius*pi/4;

% Cable Lengths, Module A
ACableLength1L0 = ACableLength1ABL+ACableLengthBCL+CircumRot;
ACableLength1R0 = ACableLength1ABR+ACableLengthBCR+CircumRot;
ACableLength2L0 = ACableLength2ABL+ACableLengthBCL+ACableLengthCDL+CircumRot;
ACableLength2R0 = ACableLength2ABR+ACableLengthBCR+ACableLengthCDR+CircumRot;
ACableLength3L0 = ACableLength3ABL+ACableLengthBCL+ACableLengthCDL+ACableLengthDEL+LinkRadius;
ACableLength3R0 = ACableLength3ABR+ACableLengthBCR+ACableLengthCDR+ACableLengthDER+LinkRadius;

% % Ensuring that initial cable lengths are the same for the left and right sides
% if ACableLength1L0 == ACableLength1R0
%     CableLength10 = ACableLength1L0;
% else
%     error('Error in InitialCableLengths')
% end
% 
% if ACableLength2L0 == ACableLength2R0
%     CableLength20 = ACableLength2L0;
% else
%     error('Error in InitialCableLengths')
% end
% 
% if ACableLength3L0 == ACableLength3R0
%     CableLength30 = ACableLength3L0;
% else
%     error('Error in InitialCableLengths')
% end

CableLength10 = ACableLength1L0;
CableLength20 = ACableLength2L0;
CableLength30 = ACableLength3L0;

% Cable lengths at equilibrium
CableLength0 = [CableLength10, CableLength20, CableLength30];

end
