% Defining Time Vector, State Vector, Accel Vector, & PHI_T vector,
% depending on the number of waypoints. Here, the contributions at
% each waypoint are pieced together to form vectors for the entire motion.

function [T,Y,PHI_T,qpp] = f_Definitions(Ye1,Ye2,Ye3,T1,T2,T3,Tstop,Y1,Y2,Y3,Ystop,PHI_T1,PHI_T2,PHI_T3,PHI_Tstop,qpp1,qpp2,qpp3,qppstop)

% 1 Waypoint
if isempty(Ye1) == 1
    T = T1;
    Y = Y1;
    PHI_T = PHI_T1;
    qpp = qpp1;
% 2 Waypoints
elseif isempty(Ye1) == 0 && isempty(Ye2) == 1;
    T = [T1;T1(end)+T2];
    Y = [Y1;Y2];
    PHI_T = [PHI_T1;PHI_T2];
    PHI_T(:,1) = T;
    qpp = [qpp1;qpp2];
% 3 Waypoints
elseif isempty(Ye1) == 0 && isempty(Ye2) == 0 && isempty(Ye3) == 1;
    T = [T1;T1(end)+T2;T1(end)+T2(end)+T3];
    Y = [Y1;Y2;Y3];
    PHI_T = [PHI_T1;PHI_T2;PHI_T3];
    PHI_T(:,1) = T;
    qpp = [qpp1; qpp2; qpp3];
% Stopping
elseif isempty(Ye1) == 0 && isempty(Ye2) == 0 && isempty(Ye3) == 0;
    T = [T1;T1(end)+T2;T1(end)+T2(end)+T3;T1(end)+T2(end)+T3(end)+Tstop];
    Y = [Y1;Y2;Y3;Ystop];
    PHI_T = [PHI_T1;PHI_T2;PHI_T3;PHI_Tstop];
    PHI_T(:,1) = T;
    qpp = [qpp1; qpp2; qpp3;qppstop];
end

end
