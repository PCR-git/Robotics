% Picks out the angle component from the q vector for each body

function angle = f_angle(q, body)

angle=q(3*body);

end