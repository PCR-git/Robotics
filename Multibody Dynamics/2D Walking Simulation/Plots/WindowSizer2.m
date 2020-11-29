% Sets window limits for plots

function [x_low,x_high,y_low,y_high] = WindowSizer2(WayPts,buffer)

rw1 = WayPts(:,1);
rw2 = WayPts(:,2);
rw3 = WayPts(:,3);
rw4 = WayPts(:,4);

MinX = min([rw1(1),rw2(1),rw3(1),rw4(1)]);
MaxX = max([rw1(1),rw2(1),rw3(1),rw4(1)]);

MinY = min([rw1(2),rw2(2),rw3(2),rw4(2)]);
MaxY = max([rw1(2),rw2(2),rw3(2),rw4(2)]);

XDist = MaxX-MinX;
YDist = MaxY-MinY;

Dist = max(XDist,YDist)+2*buffer;

% XCenter = (rw1(1)+rw2(1)+rw3(1)+rw4(1))/4;
% YCenter = (rw1(2)+rw2(2)+rw3(2)+rw4(2))/4;

XCenter = (MaxX+MinX)/2;
YCenter = (MaxY+MinY)/2;

Center = [XCenter;YCenter];

x_low = Center(1)-Dist/2;
x_high = Center(1)+Dist/2;
y_low = Center(2)-Dist/2;
y_high = Center(2)+Dist/2;

end
