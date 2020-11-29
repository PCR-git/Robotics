
% Viscous Friction
function Ffr = f_Viscous(Fr,rlp,Cv)

    Ff1 = f_Vn(Fr,rlp,Cv,1);
    Ff2 = f_Vn(Fr,rlp,Cv,2);
    Ff3 = f_Vn(Fr,rlp,Cv,3);
 
    Ffr = [Ff1,Ff2,Ff3];

end