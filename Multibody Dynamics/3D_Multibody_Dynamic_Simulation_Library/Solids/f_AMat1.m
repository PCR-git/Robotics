function A = f_AMat1(p, body)

if body == 0
    A = eye(3, 3);
else
    E = f_EMatrix(p, body);
    G = f_GMatrix(p, body);
    A = E*(G.');
end
end

