% Coulomb Friction
function Ffr = f_Coulomb(Fr,rlp,NFP,M,g)

Ff1 = f_Cn(Fr,rlp,NFP,M,g,1);
Ff2 = f_Cn(Fr,rlp,NFP,M,g,2);
Ff3 = f_Cn(Fr,rlp,NFP,M,g,3);

Ffr = [Ff1,Ff2,Ff3];
    
end