
function PHI_t_trans

syms t

q1x = sym('qx1(t)');
q1y = sym('qy1(t)');
q1phi = sym('q1phi(t)');
q2x = sym('qx2(t)');
q2y = sym('qy2(t)');
q2phi = sym('q2phi(t)');

PHI_trans = [6*sin(q1phi - q2phi) + cos(q1phi)*(q1x - q2x) + sin(q1phi)*(q1y - q2y);...
             -sin(q1phi - q2phi)];

PHI_t_trans = diff(PHI_trans,t);

end