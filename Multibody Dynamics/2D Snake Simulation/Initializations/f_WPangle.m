function WPangle = f_WPangle(rwi,rwj)

xwi = rwi(1);
ywi = rwi(2);
xwj = rwj(1);
ywj = rwj(2);

% Vector between waypoints
v1 = [xwj-xwi; ywj-ywi];

WPangle = -atan2(v1(1), v1(2))*(180/pi);

end