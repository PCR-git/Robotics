function G = f_GMat(p, body)

e = f_e(p, body);

e0 = e(1);
e1 = e(2);
e2 = e(3);
e3 = e(4);

G = [-e1,  e0,  e3, -e2;...
     -e2, -e3,  e0,  e1;...
     -e3,  e2, -e1, e0];
