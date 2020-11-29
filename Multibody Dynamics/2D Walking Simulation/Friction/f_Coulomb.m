% Coulomb Friction
function Ffr = f_Coulomb(Fr,rlp,NFP,M,g)

Ff1 = f_Cn(Fr,rlp,NFP,M,g,1);
Ff2 = f_Cn(Fr,rlp,NFP,M,g,2);
Ff3 = f_Cn(Fr,rlp,NFP,M,g,3);
Ff4 = f_Cn(Fr,rlp,NFP,M,g,4);
Ff5 = f_Cn(Fr,rlp,NFP,M,g,5);
Ff6 = f_Cn(Fr,rlp,NFP,M,g,6);
Ff7 = f_Cn(Fr,rlp,NFP,M,g,7);
Ff8 = f_Cn(Fr,rlp,NFP,M,g,8);

Ffr = [Ff1,Ff2,Ff3,Ff4,Ff5,Ff6,Ff7,Ff8];
    
end