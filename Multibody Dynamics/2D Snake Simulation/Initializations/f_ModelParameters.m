
function [LVec,MVec,IVec] = f_ModelParameters(nb,L,mass)

% Preallocate vectors
LVec = zeros(1,nb);
MVec = zeros(1,nb);
IVec = zeros(1,nb);

% Model Parameters
i = 1;
while i <= nb
    LVec(i) = L;                 % Length n
    MVec(i) = mass;              % Mass n
    IVec(i) = mass*((2*L)^2)/3;  % Inertia n
    
    i = i+1;
end

end