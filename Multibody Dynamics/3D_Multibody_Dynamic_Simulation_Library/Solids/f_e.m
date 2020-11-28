function e = f_e(p, body)

if body == 0
    e0 = 1;
    e1 = 0;
    e2 = 0;
    e3 = 0;
else
    e0 = p(1 + 4*(body - 1));
    e1 = p(2 + 4*(body - 1));
    e2 = p(3 + 4*(body - 1));
    e3 = p(4 + 4*(body - 1));
end

e = [e0; e1; e2; e3];

end
