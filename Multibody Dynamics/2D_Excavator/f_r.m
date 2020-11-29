function rA = f_r(q, body, sA)

%syms theta real
%R = symfun([cos(theta),-sin(theta); sin(theta),cos(theta)], theta);

rA= f_cg(q,body)+f_RM(q, body)*sA;

end