% Plots a circle

function f_arc(x,y,r,colour,ang,cut)

hold on;
th = ang+cut-pi:pi/50:ang+cut;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
plot(xunit, yunit,colour,'LineWidth', 2.8);

hold off;

end