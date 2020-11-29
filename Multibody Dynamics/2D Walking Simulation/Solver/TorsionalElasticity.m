% This functions calculates the elastic force vector, due to torsional
% springs at each joint.

function Fka = TorsionalElasticity(Ck,q)

% Torsional spring constant
Cka = Ck(4);

% Angle Definitions
phi1 = q(3); phi2 = q(6); phi3 = q(9); phi4 = q(12);
phi5 = q(15); phi6 = q(18); phi7 = q(21); phi8 = q(24);

% Elastic force vector
Fka1 = -Cka*(phi2-phi1);
Fka2 = -Cka*(phi3-phi2);
Fka3 = -Cka*(phi4-phi3);
%Fka4 = -Cka*(phi5-phi4);
Fka4 = 0;
Fka5 = -Cka*(phi6-phi5);
Fka6 = -Cka*(phi7-phi6);
Fka7 = -Cka*(phi8-phi7);

% Going to generalized coordinates
Fka = [0;0;-Fka1/2;0;0;Fka1/2-Fka2/2;0;0;Fka2/2-Fka3/2;0;0;Fka3/2-Fka4/2;...
      0;0;Fka4/2-Fka5/2;0;0;Fka5/2-Fka6/2;0;0;Fka6/2-Fka7/2;0;0;Fka7/2];
  
end
