% Specifies the Jacobian (PHIq), Gamma (from the acceleration equation),
% and the force vector (Qa)

function [M, J, PHI_r, PHI_p, PHI_p_dp, Gamma, Gamma_p, Fa, n_p] = BuildMechanism(Y, n, m, Params)

% Call out elements of Params structure
Body0 = Params.body0;
Body1 = Params.body1;
Body2 = Params.body2;

Fa = Params.Fa;
n_p = Params.n_p;

M = Params.M;
J = Params.J;

fi = Params.fi;
gi = Params.gi;
hj = Params.hj;


% for ii = 1:1:(length(fieldnames(Revolute)))  
%     rev = Revolute.(['rev' num2str(ii)]);
%     bodyi = Params.body1.num;
%     bodyj = Params.body2.num;
%     si_p  = Params.body1.sp.(['n' num2str(Params.sp1)]);
%     sj_p  = Params.body2.sp.(['n' num2str(Params.sp2)]);
%     
%     [JAC_revolute_dr, Jac_revolute_dp] = JAC_revolute(Y, si_p, fi, gi, bodyi, sj_p, hj, bodyj, n, m);
%     PHI_r = vertcat(PHI_r, JAC_revolute_dr);
%     PHI_p = vertcat(PHI_p, Jac_revolute_dp);
%     
%     
%     gamma_revolute = f_gamma_revolute(Y, si_p, fi, gi, bodyi, sj_p, hj, bodyj, n, m);
%     Gamma = vertcat(Gamma, gamma_revolute);
%     
%     if isempty(strfind(rev.wc, 's')) == 1
%         PHI_r((end - 4):(end - 2), :) = [];
%         PHI_p((end - 4):(end - 2), :) = [];
%         Gamma((end - 4):(end - 2), :) = [];
%     end
%     if isempty(strfind(rev.wc, '1')) == 1
%         PHI_r((end - 1), :) = [];
%         PHI_p((end - 1), :) = [];
%         Gamma((end - 1), :) = [];
%     end
%     if isempty(strfind(rev.wc, '2')) == 1
%         PHI_r((end), :) = [];
%         PHI_p((end), :) = [];
%         Gamma((end), :) = [];
%     end
% end



% Assembles the revolute sub-Jacobian
[JAC_revolute_dr1, Jac_revolute_dp1, ~] = JAC_revolute(Y, Body0.sA0, fi, gi, Body0.num, Body1.sA1, hj, Body1.num, n, m);
[JAC_revolute_dr2, Jac_revolute_dp2, ~] = JAC_revolute(Y, Body1.sB1, fi, gi, Body1.num, Body2.sB2, hj, Body2.num, n, m);

PHI_r = [JAC_revolute_dr1; JAC_revolute_dr2];
PHI_p = [Jac_revolute_dp1; Jac_revolute_dp2];
%PHI_p_dp = [Jac_revolute_p_dp1; Jac_revolute_p_dp2];

[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);
PHI_p_dp = 2*blkdiag((p(1:4).'), (p(5:8).'));

gamma_revolute1 = f_gamma_revolute(Y, Body0.sA0, fi, gi, Body0.num, Body1.sA1, hj, Body1.num, n, m);
gamma_revolute2 = f_gamma_revolute(Y, Body1.sB1, fi, gi, Body1.num, Body2.sB2, hj, Body2.num, n, m);

Gamma = [gamma_revolute1; gamma_revolute2];

gamma_p1 = f_gamma_p(Y, Body1.num, n, m);
gamma_p2 = f_gamma_p(Y, Body2.num, n, m);

Gamma_p = [gamma_p1; gamma_p2];

end
