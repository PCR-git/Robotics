% Calculates the effective velocity of each link due to
% a pure rotation about its center (assumed to be its COM).

% NOTE: Forces due to translation of the COM and pure rotation about
% the COM must be accounted for separately. We divide the mass of a link
% between N masses of mass m/N, distributed at equal intervals along the
% link, and calculate the fraction of the link length such that masses
% located at this distance from the center on both sides of the link would
% have an equivalent absolute value of linear velocity. This procedure
% gives a multiplier of: (2/(2N+1))*integral{(N-n)/N dn} = N/(2N+1).
% Taking the limit as N--> infinity gives: 1/2. Thus, the variables
% VelEff, below, have a multiplier of 1/2 in front.

function VelEff1 = f_EffVel(Li,phii,phiip)

VelEff1 = (1/2)*Li*phiip*[cos(phii);-sin(phii)];

end