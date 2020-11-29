% Creates an animation of the worm robot

function animate_pendulum(T,Y,nb,n)

A = 0.1;    % Determines length of pause

pause(1*A)  % Pause between consecutive frames

% Preallocates tracking vectors
pAVec = zeros(3,length(T));
pBVec = zeros(3,length(T));
pCVec = zeros(3,length(T));

i=1;
while i <= length(T)
    %%%%%%%%%%%%%%%%%%%%
    
    % Coordinate Definitions
    
    q = Y(i, n+nb*2+1:n+nb*2+n);  % (Local) Position components of the state vector Y
    
    % Local Vectors
    s_pa = [0;  1];
    s_pb = [0; -1];
    
    % Coordinates of Points
    pA = f_r(q, 1, s_pa);   % Coords of Pt A
    p1 = f_cg(q, 1);        % Coords of CG1
    pB = f_r(q, 1, s_pb);   % Coords of Pt B
    p2 = f_cg(q, 2);        % Coords of CG2
    pC = f_r(q, 2, s_pb);   % Coords of Pt C
        
    pAVec(1,i) = pA(1);     % Stores x-component of trajectory of Pt A
    pAVec(2,i) = pA(2);     % Stores y-component of trajectory of Pt A
    pBVec(1,i) = pB(1);     % Stores x-component of trajectory of Pt B 
    pBVec(2,i) = pB(2);     % Stores y-component of trajectory of Pt B
    pCVec(1,i) = pC(1);     % Stores x-component of trajectory of Pt C
    pCVec(2,i) = pC(2);     % Stores y-component of trajectory of Pt C
    
    %%%%%%%%%%%%%%%%%%%%
    
    % Plots
    
    % Plots of the links
    plot([pA(1) p1(1) pB(1)], [pA(2) p1(2) pB(2)], 'blacko', 'LineWidth', 3);
    hold on;
    plot([pA(1) p1(1) pB(1)], [pA(2) p1(2) pB(2)], 'black',  'LineWidth', 3);
    plot([pB(1) p2(1) pC(1)], [pB(2) p2(2) pC(2)], 'blacko', 'LineWidth', 3);
    plot([pB(1) p2(1) pC(1)], [pB(2) p2(2) pC(2)], 'black',  'LineWidth', 3);
    plot([pB(1) p2(1) pC(1)], [pB(2) p2(2) pC(2)], 'blacko', 'LineWidth', 3);
    
    
    
%     plot([pA(1) p1(1) pB(1)], [pA(2) p1(2) pB(2)], 'black', 'LineWidth', 4);  % Plots vector between Pt A & Pt B
%     hold on;
%     plot([pB(1) p2(1) pC(1)], [pB(2) p2(2) pC(2)], 'black', 'LineWidth', 2);  % Plots vector between Pt B & Pt C
%     
    %plot(pAVec(1,1:i),pAVec(2,1:i), 'g', 'LineWidth', 2);   % Traces head
    plot(pBVec(1,1:i),pBVec(2,1:i), 'b', 'LineWidth', 2);   % Traces end of 1st link
    plot(pCVec(1,1:i),pCVec(2,1:i), 'r', 'LineWidth', 2);   % Traces end of 2nd link
    
    
    % Plot specs
    xlabel('X Position');
    ylabel('Y Position');
    title('Animation of Worm Robot');
    xlim([-5 5]);
    ylim([-5 5]);
    grid on;
    axis square;
    pause(A*(T(2)-T(1)));
    hold off;
    
    i = i+1;
end