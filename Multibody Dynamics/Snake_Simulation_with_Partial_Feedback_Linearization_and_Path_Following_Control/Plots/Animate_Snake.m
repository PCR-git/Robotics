% Animation of the Snake Robot

function Animate_Snake(T,Y,n,m,WayPts,WPtol,buffer,FricType)
%pause(2)

% Removes duplicate elements of T
% (to deal with lags in DAE solver)
[Tu,ia,~] = uniquetol(T,0.0001); % Gives unique values
% c gives vector of unique values
% ia gives vector of first occurences of unique values

% Removes elements of Y corresponding to duplicate values of T
Yu = zeros(length(Tu),2*n+m);
jj = 1;
while jj <= length(ia)
    Yu(jj,:) = Y(ia(jj),:);
    jj = jj+1;
end

% % Takes every Nth point from T and Y to plot, to speed up animation
% N = 5;  % Default 5, 9 for small file
% T = T(1:N:length(T));
% [row, ~] = size(Y);
% Y = Y(1:N:row,:);

% % Takes every Nth point from Tu and Yu to plot, to speed up animation
% N = 5;  % Default 5, 9 for small file
% Tu = Tu(1:N:length(Tu));
% [row, ~] = size(Yu);
% Yu = Yu(1:N:row,:);

% % Creates Video
% SnakeVid = VideoWriter('SnakeAnimation.avi');
% SnakeVid.FrameRate = 25;  % Frame Rate: Default 30, 25 for small file
% SnakeVid.Quality = 50;   % Quality: Default 75/100, 50 for small file
% open(SnakeVid)
% % Sets Animation Parameters
% hfig = figure('Position',[1 0 1680 1050],'MenuBar','none','ToolBar','none','resize','off'); % fullscreen
% % (Resolution set for RML desktop)

% % Preallocates tracking vectors (for T)
% pAVec = zeros(2,length(T));
% pBVec = zeros(2,length(T));
% pCVec = zeros(2,length(T));
% pDVec = zeros(2,length(T));
% pEVec = zeros(2,length(T));
% pFVec = zeros(2,length(T));
% pGVec = zeros(2,length(T));

% Preallocates tracking vectors (for Tu)
pAVec = zeros(2,length(Tu));
pBVec = zeros(2,length(Tu));
pCVec = zeros(2,length(Tu));
pDVec = zeros(2,length(Tu));
pEVec = zeros(2,length(Tu));
pFVec = zeros(2,length(Tu));
pGVec = zeros(2,length(Tu));

% Way Point Definitions
rw1 = WayPts(:,1);
rw2 = WayPts(:,2);
rw3 = WayPts(:,3);
rw4 = WayPts(:,4);

% Way Point Coordinates
wx1 = rw1(1); wy1 = rw1(2);
wx2 = rw2(1); wy2 = rw2(2);
wx3 = rw3(1); wy3 = rw3(2);
wx4 = rw4(1); wy4 = rw4(2);

% Lines between way points
xx12 = linspace(wx1,wx2,50);
yy12 = linspace(wy1,wy2,50);
xx23 = linspace(wx2,wx3,50);
yy23 = linspace(wy2,wy3,50);
xx34 = linspace(wx3,wx4,50);
yy34 = linspace(wy3,wy4,50);

% Sets window size
[x_low, x_high, y_low, y_high] = f_window_size(WayPts,buffer);

i=1;
%while i <= length(T)
while i <= length(Tu)
    
    %q = Y(i, n+m+1:n+m+n);   % (Local) Position components of the state vector Y
    q = Yu(i, n+m+1:n+m+n);   % (Local) Position components of the state vector Yu
    
    %R1 = f_RM(q(3));
    %R2 = f_RM(q(6));
    %R3 = f_RM(q(9));
    
%     if i>1 && isempty(q(i-1,1:n)) == 0 && q(i,1:n) == q(i-1,1:n)
%         q(i,n) = [];
%     end
    
    % Local Vectors
    s_pa = [0;  1];
    s_pb = [0; -1];
    
    % Coordinates of Points
    %pA = [0; 0];           % Coords of Pt O
    pA = f_r(q, 1, s_pa);   % Coords of Pt A
    p1 = f_cg(q, 1);        % Coords of CG1
    pB = f_r(q, 1, s_pb);   % Coords of Pt B
    p2 = f_cg(q, 2);        % Coords of CG2
    pC = f_r(q, 2, s_pb);   % Coords of Pt C
    p3 = f_cg(q, 3);        % Coords of CG3
    pD = f_r(q, 3, s_pb);   % Coords of Pt D
    p4 = f_cg(q, 4);        % Coords of CG4
    pE = f_r(q, 4, s_pb);   % Coords of Pt E
    p5 = f_cg(q, 5);        % Coords of CG5
    pF = f_r(q, 5, s_pb);   % Coords of Pt F
    p6 = f_cg(q, 6);        % Coords of CG6
    pG = f_r(q, 6, s_pb);   % Coords of Pt G
    
    pAVec(1,i) = pA(1);     % Stores x-component of trajectory of Pt A
    pAVec(2,i) = pA(2);     % Stores y-component of trajectory of Pt A
    pBVec(1,i) = pB(1);     % Stores x-component of trajectory of Pt B 
    pBVec(2,i) = pB(2);     % Stores y-component of trajectory of Pt B
    pCVec(1,i) = pC(1);     % Stores x-component of trajectory of Pt C
    pCVec(2,i) = pC(2);     % Stores y-component of trajectory of Pt C
    pDVec(1,i) = pD(1);     % Stores x-component of trajectory of Pt D
    pDVec(2,i) = pD(2);     % Stores y-component of trajectory of Pt D
    pEVec(1,i) = pE(1);     % Stores x-component of trajectory of Pt E
    pEVec(2,i) = pE(2);     % Stores y-component of trajectory of Pt E    
    pFVec(1,i) = pF(1);     % Stores x-component of trajectory of Pt F
    pFVec(2,i) = pF(2);     % Stores y-component of trajectory of Pt F
    pGVec(1,i) = pG(1);     % Stores x-component of trajectory of Pt G
    pGVec(2,i) = pG(2);     % Stores y-component of trajectory of Pt G

    % Plots
    plot([pA(1) p1(1) pB(1)], [pA(2) p1(2) pB(2)], 'black',  'LineWidth', 2.8);              % Plots vector between Pt A & Pt B
    hold on;
    plot([pB(1) p2(1) pC(1)], [pB(2) p2(2) pC(2)], 'black',  'LineWidth', 2.8);              % Plots vector between Pt B & Pt C
    plot([pC(1) p3(1) pD(1)], [pC(2) p3(2) pD(2)], 'black',  'LineWidth', 2.8);              % Plots vector between Pt C & Pt D
    plot([pD(1) p4(1) pE(1)], [pD(2) p4(2) pE(2)], 'black',  'LineWidth', 2.8);              % Plots vector between Pt D & Pt E
    plot([pE(1) p5(1) pF(1)], [pE(2) p5(2) pF(2)], 'black',  'LineWidth', 2.8);              % Plots vector between Pt E & Pt F
    plot([pF(1) p6(1) pG(1)], [pF(2) p6(2) pG(2)], 'black',  'LineWidth', 2.8);              % Plots vector between Pt F & Pt G
    plot([pA(1) pB(1) pC(1) pD(1) pE(1) pF(1) pG(1)], [pA(2) pB(2) pC(2) pD(2) pE(2) pF(2) pG(2)], 'blacko', 'MarkerSize', 4.5);  % Plots points at Pts A, B, C, D, E, & F
    plot([pA(1) pB(1) pC(1) pD(1) pE(1) pF(1) pG(1)], [pA(2) pB(2) pC(2) pD(2) pE(2) pF(2) pG(2)], 'blacko', 'MarkerSize', 5);    % Plots points at Pts A, B, C, D, E, & F
    %plot([p1(1) p2(1) p3(1) p4(1) p5(1) p6(1)], [p1(2) p2(2) p3(2) p4(2) p5(2) p6(2)], 'black*', 'MarkerSize',3);                % Plots points at the local frames
    
    plot(pAVec(1,1:i),pAVec(2,1:i), 'b', 'LineWidth', 1);    % Traces head
    %plot(pBVec(1,1:i),pBVec(2,1:i), 'r', 'LineWidth', 2);   % Traces end of 1st link
    %plot(pCVec(1,1:i),pCVec(2,1:i), 'g', 'LineWidth', 2);   % Traces end of 2nd link
    %plot(pDVec(1,1:i),pDVec(2,1:i), 'y', 'LineWidth', 2);   % Traces end of 3rd link
    %plot(pEVec(1,1:i),pEVec(2,1:i), 'r', 'LineWidth', 2);   % Traces end of 4th link
    %plot(pFVec(1,1:i),pFVec(2,1:i), 'g', 'LineWidth', 2);   % Traces end of 5th link
    %plot(pGVec(1,1:i),pGVec(2,1:i), 'y', 'LineWidth', 2);   % Traces end of 6th link
    
    % % Defines a pt located one body length away, along the axis of 1st link
    % Target = [pA(1);pA(2)] + 2*BL*R1*[0;1];
    % plot([pA(1) Target(1)], [pA(2) Target(2)], 'y--', 'Linewidth', 3);  % Plots field of view

    % Plotting Way Points
    plot(wx1,wy1,'o','MarkerEdgeColor','black','MarkerFaceColor','black','MarkerSize', 6);
    plot(wx2,wy2,'o','MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0],'MarkerSize', 6);
    plot(wx3,wy3,'o','MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0],'MarkerSize', 6);
    plot(wx4,wy4,'o','MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0],'MarkerSize', 6);
%     plot(wx1,wy1,'ro','LineWidth', 3);
%     plot(wx2,wy2,'ro','LineWidth', 3);
%     plot(wx3,wy3,'ro','LineWidth', 3);
%     plot(wx4,wy4,'ro','LineWidth', 3);
    
    % Plotting Desired Trajectories
    plot(xx12,yy12,'black--','LineWidth',0.9);
    plot(xx23,yy23,'black--','LineWidth',0.9);
    plot(xx34,yy34,'black--','LineWidth',0.9);
    
    % Plotting Acceptance Circles
    %f_circle(wx1,wy1,WPtol,[1 0 0]);
    f_circle(wx2,wy2,WPtol,[1 0 0]);
    f_circle(wx3,wy3,WPtol,[1 0 0]);
    f_circle(wx4,wy4,WPtol,[1 0 0]);
    
    % Axes Labels
    xlabel('{\itx}-position','FontSize',20,'FontName','Times New Roman');
    ylabel('{\ity}-position','FontSize',20,'FontName','Times New Roman');
    
    % Title
    if FricType == 1
        title('Simulation of Slithering Locomotion (viscous friction)','FontSize',20,'FontName','Times New Roman');
    elseif FricType == 2
        title('Simulation of Slithering Locomotion (Coulomb friction)','FontSize',20,'FontName','Times New Roman');
    else
        title('Simulation of Slithering Locomotion','FontSize',20,'FontName','Times New Roman');
    end
    
    % Set axis limits
    xlim([x_low x_high]);
    ylim([y_low y_high]);
    
    %set(gca, 'FontSize',15);
    grid on;
    axis equal;
    
    %pause(T(2)-T(1));
    pause(Tu(2)-Tu(1));
    %pause(0.01);
    
    hold off;
    
    i = i+1;
    
%     % Writes each frame to a video file
%     frame = getframe;
% %     frame = getframe(hfig);
%     writeVideo(SnakeVid,frame);
end
%     close(SnakeVid);
end
