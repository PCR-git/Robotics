function A=f_A(e0,e1,e2,e3)

A = 2*[((e0^2)+(e1^2)-0.5) , ((e1*e2)-(e0*e3))   , ((e1*e3)+(e0*e2));
       ((e2*e1)+(e0*e3))   , ((e0^2)+(e2^2)-0.5) , ((e2*e3)-(e0*e1));
       ((e3*e1)-(e0*e2))   , ((e3*e2)+(e0*e1))   , ((e0^2)+(e3^2)-0.5)];
end