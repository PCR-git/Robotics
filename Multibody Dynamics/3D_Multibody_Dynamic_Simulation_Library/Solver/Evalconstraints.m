% Assembles the constraint vector, PHI

function [PHI] = Evalconstraints(Y, n, m, Params)

Body0 = Params.body0;
Body1 = Params.body1;
Body2 = Params.body2;

fi = Params.fi;
gi = Params.gi;
hj = Params.hj;

PHI_revolute1 = f_PHI_revolute(Y, Body0.sA0, fi, gi, Body0.num, Body1.sA1, hj, Body1.num, n, m);
PHI_revolute2 = f_PHI_revolute(Y, Body1.sB1, fi, gi, Body1.num, Body2.sB2, hj, Body2.num, n, m);

PHI = [PHI_revolute1; PHI_revolute2];

end
