% Sets length, mass, and moment of inertia for each link
% MP2: Head connected to end connector; head forward

function [LVec,MVec,IVec] = f_ModelParameters2(nb)

% Preallocate vectors
LVec = zeros(1,nb);
MVec = zeros(1,nb);
IVec = zeros(1,nb);

% Model Parameters

LVec(1,1) = 0.06188;
LVec(1,2) = 0.0503;
LVec(1,3) = 0.0503;
LVec(1,4) = 0.07788;
LVec(1,5) = 0.07788;
LVec(1,6) = 0.0503;
LVec(1,7) = 0.0503;
LVec(1,8) = 0.06188;

LVec = LVec/2;

MVec(1,1) = 0.10508;
MVec(1,2) = 0.19775;
MVec(1,3) = 0.19775;
MVec(1,4) = 0.31932;
MVec(1,5) = 0.31932;
MVec(1,6) = 0.19775;
MVec(1,7) = 0.19775;
MVec(1,8) = 0.10508;

IVec(1,1) = 0.0002;
IVec(1,2) = 0.0014;
IVec(1,3) = 0.0014;
IVec(1,4) = 0.0042;
IVec(1,5) = 0.0042;
IVec(1,6) = 0.0014;
IVec(1,7) = 0.0014;
IVec(1,8) = 0.0002;

end
