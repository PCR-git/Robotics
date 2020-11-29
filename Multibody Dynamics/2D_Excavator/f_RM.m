function R = f_RM(q, body)

theta=q(3*body);
R = [cos(theta),-sin(theta); sin(theta),cos(theta)];

end