function GMatrix = f_GMatrix(e0, e1, e2, e3)

GMatrix = [-e1,  e0,  e3, -e2;
           -e2, -e3,  e0,  e1;
           -e3,  e2, -e1,  e0];

end