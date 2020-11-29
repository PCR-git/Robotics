% Cross Track Error, py, for each link
function PY = f_CrossTrackError(q,rw1,rw2)

% Position Definitions
x1 = q(1);  y1 = q(2);
%x2 = q(4);  y2 = q(5);
%x3 = q(7);  y3 = q(8);
%x4 = q(10); y4 = q(11);
%x5 = q(13); y5 = q(14);
%x6 = q(16); y6 = q(17);
%x7 = q(19); y7 = q(20);
%x8 = q(22); y8 = q(23);

% Way Point Definitions
xw1 = rw1(1);
yw1 = rw1(2);
xw2 = rw2(1);
yw2 = rw2(2);

% Distance between snake frames and way points
K1squared1 = (x1-xw1)^2 + (y1-yw1)^2;
K2squared1 = (x1-xw2)^2 + (y1-yw2)^2;
%K1squared2 = (x2-xw1)^2 + (y2-yw1)^2;
%K2squared2 = (x2-xw2)^2 + (y2-yw2)^2;
%K1squared3 = (x3-xw1)^2 + (y3-yw1)^2;
%K2squared3 = (x3-xw2)^2 + (y3-yw2)^2;
%K1squared4 = (x4-xw1)^2 + (y4-yw1)^2;
%K2squared4 = (x4-xw2)^2 + (y4-yw2)^2;
%K1squared5 = (x5-xw1)^2 + (y5-yw1)^2;
%K2squared5 = (x5-xw2)^2 + (y5-yw2)^2;
%K1squared6 = (x6-xw1)^2 + (y6-yw1)^2;
%K2squared6 = (x6-xw2)^2 + (y6-yw2)^2;
%K1squared7 = (x7-xw1)^2 + (y7-yw1)^2;
%K2squared7 = (x7-xw2)^2 + (y7-yw2)^2;
%K1squared8 = (x8-xw1)^2 + (y8-yw1)^2;
%K2squared8 = (x8-xw2)^2 + (y8-yw2)^2;

% Distance between way points
L = sqrt((xw2-xw1)^2+(yw2-yw1)^2);

% Cross track errors at each link
py_1 = sqrt(abs(K1squared1-(1/(2*L))^2*(K1squared1-K2squared1+L^2)^2)); % Cross-track error
%py_2 = sqrt(abs(K1squared2-(1/(2*L))^2*(K1squared2-K2squared2+L^2)^2)); % Cross-track error
%py_3 = sqrt(abs(K1squared3-(1/(2*L))^2*(K1squared3-K2squared3+L^2)^2)); % Cross-track error
%py_4 = sqrt(abs(K1squared4-(1/(2*L))^2*(K1squared4-K2squared4+L^2)^2)); % Cross-track error
%py_5 = sqrt(abs(K1squared5-(1/(2*L))^2*(K1squared5-K2squared5+L^2)^2)); % Cross-track error
%py_6 = sqrt(abs(K1squared6-(1/(2*L))^2*(K1squared6-K2squared6+L^2)^2)); % Cross-track error
%py_7 = sqrt(abs(K1squared7-(1/(2*L))^2*(K1squared7-K2squared7+L^2)^2)); % Cross-track error
%py_8 = sqrt(abs(K1squared8-(1/(2*L))^2*(K1squared8-K2squared8+L^2)^2)); % Cross-track error

%PY = [py_1,py_2,py_3,py_4,py_5,py_6,py_7,py_8];
PY = py_1;

end

