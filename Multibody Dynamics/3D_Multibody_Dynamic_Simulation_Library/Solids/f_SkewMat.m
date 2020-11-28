function SkewMat = f_SkewMat(a)

l = length(a);

if l == 3
    SkewMat = [    0, -a(3),  a(2); ...
                   a(3),     0, -a(1); ...
                  -a(2),  a(1),    0];
else
    error('Skew Matrix: a must be 3x1!');
end
