function [e0,e1,e2,e3] = findEulerParams(P,O,Y)
e0 = cos(O/2)*cos((P+Y)/2);
e1 = sin(O/2)*cos((P-Y)/2);
e2 = sin(O/2)*cos((P-Y)/2);
e3 = cos(O/2)*sin((P+Y)/2);
end