% Cross Track Error, py, for each link
function PY = f_CrossTrackError(q,rw1,rw2)

% Position Definitions
x1 = q(1); y1 = q(2);
x2 = q(4); y2 = q(5);
x3 = q(7); y3 = q(8);

% Way Point Definitions
xw1 = rw1(1);
yw1 = rw1(2);
xw2 = rw2(1);
yw2 = rw2(2);

% Distance between snake frames and way points
K1squared1 = (x1-xw1)^2 + (y1-yw1)^2;
K2squared1 = (x1-xw2)^2 + (y1-yw2)^2;
K1squared2 = (x2-xw1)^2 + (y2-yw1)^2;
K2squared2 = (x2-xw2)^2 + (y2-yw2)^2;
K1squared3 = (x3-xw1)^2 + (y3-yw1)^2;
K2squared3 = (x3-xw2)^2 + (y3-yw2)^2;

% Distance between way points
L = sqrt((xw2-xw1)^2+(yw2-yw1)^2);

% Cross track errors at each link
py_1 = sqrt(abs(K1squared1-(1/(2*L))^2*(K1squared1-K2squared1+L^2)^2)); % Cross-track error
py_2 = sqrt(abs(K1squared2-(1/(2*L))^2*(K1squared2-K2squared2+L^2)^2)); % Cross-track error
py_3 = sqrt(abs(K1squared3-(1/(2*L))^2*(K1squared3-K2squared3+L^2)^2)); % Cross-track error

PY = [py_1,py_2,py_3];

end

