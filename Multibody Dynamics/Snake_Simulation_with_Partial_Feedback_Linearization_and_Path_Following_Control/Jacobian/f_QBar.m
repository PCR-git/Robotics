function Q_Bar = f_QBar(Yn,Q,PHIq,alpha,omega,zeta,~,n,m)

q = Yn((n+m+1):end); % position
qp = Yn(1:n);        % velocity

q1x = q(1);
q1y = q(2);
q1phi = q(3);
q2x  = q(4);
q2y = q(5);
q2phi = q(6);
q3x  = q(7);
q3y = q(8);
q3phi = q(9);
% q1xp = qp(1);
% q1yp = qp(2);
% q1phip = qp(3);
% q2xp = qp(4);
% q2yp = qp(5);
% q2phip = qp(6);
% q3xp = qp(7);
% q3yp = qp(8);
% q3phip = qp(9);

% No time dependent constraints
PHIqt = [0, 0, 0, 0, 0, 0, 0, 0, 0;...
         0, 0, 0, 0, 0, 0, 0, 0, 0;...
         0, 0, 0, 0, 0, 0, 0, 0, 0;...
         0, 0, 0, 0, 0, 0, 0, 0, 0];  % PHIqt
PHItt = [0;0;0;0];  % PHItt
PHIt = [0;0;0;0];  % PHIt

PHI =  [q1x - q2x + sin(q1phi) + sin(q2phi);...
        q1y - q2y - cos(q1phi) - cos(q2phi);...
        q2x - q3x + sin(q2phi) + sin(q3phi);...
        q2y - q3y - cos(q2phi) - cos(q3phi)];  % PHI

Q_Bar = Q - transpose(PHIq)*alpha*(PHIqt*qp + PHItt + 2*zeta*omega*PHIt + omega^2*PHI);

end