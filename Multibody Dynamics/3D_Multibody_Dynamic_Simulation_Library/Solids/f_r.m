function r = f_r(r, p, body, s_p)

r = f_cg(r, body) + f_AMat(p, body)*s_p;     % global coord. of A

end
