
function Ffr = f_ViscousCoulombStribeck(Fr,rlp,NFP,M,g)

Ff1 = f_VCSn(Fr,rlp,NFP,M,g,1);
Ff2 = f_VCSn(Fr,rlp,NFP,M,g,2);
Ff3 = f_VCSn(Fr,rlp,NFP,M,g,3);
Ff4 = f_VCSn(Fr,rlp,NFP,M,g,4);
Ff5 = f_VCSn(Fr,rlp,NFP,M,g,5);
Ff6 = f_VCSn(Fr,rlp,NFP,M,g,6);
    
Ffr = [Ff1,Ff2,Ff3,Ff4,Ff5,Ff6];
    
end