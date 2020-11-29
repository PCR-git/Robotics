% Plots a circle

function f_circle(x,y,r,colour)

hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
plot(xunit, yunit,colour);

hold off

end