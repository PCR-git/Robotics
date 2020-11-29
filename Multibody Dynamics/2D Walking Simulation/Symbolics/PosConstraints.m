clc

syms t ...
     x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 x4 y4 phi4 x5 y5 phi5 x6 y6 phi6 x7 y7 phi7 x8 y8 phi8 ...
     x1p y1p phi1p x2p y2p phi2p x3p y3p phi3p x4p y4p phi4p x5p y5p phi5p x6p y6p phi6p x7p y7p phi7p x8p y8p phi8p ...
     r l f...
     r1 r2 r3...
     L1 L2 L3 L4 L5 L6 L7 L8...
     LinkRadius...
     a b c
 
% Pulley Radii
% r1 = 1;
% r2 = 2;
% r3 = 3;

q = [x1;y1;phi1;x2;y2;phi2;x3;y3;phi3;x4;y4;phi4;x5;y5;phi5;x6;y6;phi6;x7;y7;phi7;x8;y8;phi8];
qp = [x1p;y1p;phi1p;x2p;y2p;phi2p;x3p;y3p;phi3p;x4p;y4p;phi4p;x5p;y5p;phi5p;x6p;y6p;phi6p;x7p;y7p;phi7p;x8p;y8p;phi8p];
% Actuation A in Link 1 and B in Link 5:
%PHId = [phi3-(phi2+(r2/r1-1)*(phi2-phi1)); phi4-(phi3+(r3/r2-1)*(phi3-phi1)); phi5-phi4; phi7-(phi6+(r2/r1-1)*(phi6-phi5)); phi8-(phi7+(r3/r2-1)*(phi7-phi5))];
% Actuation A in Link 4 and B in Link 8:
% 1->4, 2->3, 3->2, 4->1; 5->8, 6->7, 7->6, 8->5
% PHId = [phi2-(phi3+(r2/r1-1)*(phi3-phi4)); phi1-(phi2+(r3/r2-1)*(phi2-phi4)); phi5-phi4; phi6-(phi7+(r2/r1-1)*(phi7-phi8)); phi5-(phi6+(r3/r2-1)*(phi6-phi8))];

D1 = sqrt(L1^2+LinkRadius^2);
D2 = sqrt(L2^2+LinkRadius^2);
D3 = sqrt(L3^2+LinkRadius^2);

a1 = phi2-phi1;
b1 = phi3-phi2;
c1 = phi4-phi3;

a2 = phi6-phi5;
b2 = phi7-phi6;
c2 = phi8-phi7;

% Pradius1 = 1/5;
% Pradius2 = (alpha2/alpha1)*(b1/a1+1)*Pradius1;
% Pradius3 = ((c1/a1)+(alpha1/alpha2)*(Pradius2/Pradius1)) *(alpha3/alpha1)*Pradius1;

% PHId = [phi2-(phi3+(r2/r1-1)*(phi3-phi4)); phi1-(phi2+(r3/r2-1)*(phi2-phi4)); phi5-phi4; phi6-(phi7+(r2/r1-1)*(phi7-phi8)); phi5-(phi6+(r3/r2-1)*(phi6-phi8))];

% PHId = [r2*D1 - (r1*D2)*(b1/a1+1); D1*r3 - (D3*r1)*((r2/r1)*(D1/D2)+c1/a1);phi5-phi4;D1*r2 - (D2*r1)*(b2/a2+1); D1*r3 - (D3*r1)*((r2/r1)*(D1/D2)+c2/a2)];

% PHId = [(phi2-phi1)*b-(phi3-phi2)*a; (phi2-phi1)*c-(phi4-phi3)*a; phi5-phi4; (phi6-phi5)*b-(phi7-phi6)*a; (phi6-phi5)*c-(phi8-phi7)*a]

PHId = [(phi2-phi1)*b-(phi3-phi2)*a; (phi2-phi1)*c-(phi4-phi3)*a; phi5-phi4; (phi8-phi7)*c-(phi6-phi5)*a; (phi8-phi7)*b-(phi7-phi6)*a]

PHIqd = jacobian(PHId,q)

t1 = -jacobian(PHIqd*qp,q)*qp;
t2 = -2*diff(PHIqd,t)*qp;
t3 = -diff(diff(PHId,t),t);

gammad = t1+t2+t3

