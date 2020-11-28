% Creates a skew-symmetric matrix from a vector

function Skew = f_Skew(a)

l = length(a);

if l == 3
    Skew = [    0, -a(3),  a(2); ...
             a(3),     0, -a(1); ...
            -a(2),  a(1),    0];
else
    error('Skew Matrix: a must be 3x1!');
end
