% Calculates the derivative of the state vector, Yn

function [Ypn] = f_DAE0(t, Yn, n, m, Params, PHI_r, PHI_p, PHI_p_dp, Gamma, Gamma_p)

% Call out elements of Params structure
Fa = Params.Fa;
n_p = Params.n_p;

M = Params.M;
J = Params.J;

if t > 0                            
    [M, J, PHI_r, PHI_p, PHI_p_dp, Gamma, Gamma_p, Fa, n_p] = BuildMechanism(Yn, n, m, Params);
end

[r_dt, p_dt, ~, ~, ~, p] = f_StateVar(Yn, n, m);

nb = n/7;

% Zero elements of the LHS Matrix
z12 = zeros(3*nb, 4*nb);
z14 = zeros(3*nb, nb);
z15 = zeros(3*nb, 3*nb);
z16 = zeros(3*nb, 4*nb);

z21 = zeros(4*nb, 3*nb);
z25 = zeros(4*nb, 3*nb);
z26 = zeros(4*nb, 4*nb);

z33 = zeros(m - nb, m - nb);
z34 = zeros(m - nb, nb);
z35 = zeros(m - nb, 3*nb);
z36 = zeros(m - nb, 4*nb);

z41 = zeros(nb, 3*nb);
z43 = zeros(nb, m - nb);
z44 = zeros(nb, nb);
z45 = zeros(nb, 3*nb);
z46 = zeros(nb, 4*nb);

z51 = zeros(3*nb, 3*nb);
z52 = zeros(3*nb, 4*nb);
z53 = zeros(3*nb, m - nb);
z54 = zeros(3*nb, nb);
i55 = eye(3*nb);
z56 = zeros(3*nb, 4*nb);

z61 = zeros(4*nb, 3*nb);
z62 = zeros(4*nb, 4*nb);
z63 = zeros(4*nb, m - nb);
z64 = zeros(4*nb, nb);
z65 = zeros(4*nb, 3*nb);
i66 = eye(4*nb);

G = blkdiag(f_GMat(p, 1), f_GMat(p, 2));
G_dt = blkdiag(f_GMat(p_dt, 1), f_GMat(p_dt, 2));

% Matrix DEA for the system
LHS = [   M, z12,  (PHI_r.'), z14, z15, z16; ...
       z21, 4*transpose(G)*J*G, (PHI_p.'), (PHI_p_dp.'), z25, z26; ...
       PHI_r, PHI_p, z33, z34, z35, z36; ...
       z41, PHI_p_dp, z43, z44, z45, z46; ...
       z51, z52, z53, z54, i55, z56; ...
       z61, z62, z63, z64, z65, i66];
RHS = [  Fa;  2*(G.')*n_p + 8*(G_dt.')*J*G_dt*p; Gamma; Gamma_p; r_dt; p_dt];

Ypn =  LHS\RHS;                      % state vector derivative

% Mblock = [M, z12,; ...
%          z21, 4*transpose(G)*J*G];
%     
% PHIP = [PHI_r, PHI_p;...
%        z41, PHI_p_dp];
% 
% LHS0 = [Mblock transpose(PHIP);...
%        PHIP zeros(m,m)];
%
%LHHS = [LHS0, zeros(n+m,n);...
%        zeros(n,n+m), eye(n,n)];

end
