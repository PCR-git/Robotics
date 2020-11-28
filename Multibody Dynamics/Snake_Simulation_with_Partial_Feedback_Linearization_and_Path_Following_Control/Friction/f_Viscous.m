
% Viscous Friction
function Ffr = f_Viscous(Fr,rlp,Cv)

    Ff1 = f_Vn(Fr,rlp,Cv,1);
    Ff2 = f_Vn(Fr,rlp,Cv,2);
    Ff3 = f_Vn(Fr,rlp,Cv,3);
    Ff4 = f_Vn(Fr,rlp,Cv,4);
    Ff5 = f_Vn(Fr,rlp,Cv,5);
    Ff6 = f_Vn(Fr,rlp,Cv,6);
 
    Ffr = [Ff1,Ff2,Ff3,Ff4,Ff5,Ff6];

end