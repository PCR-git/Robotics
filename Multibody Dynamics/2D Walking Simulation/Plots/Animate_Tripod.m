% Animation of the Snake Robot

function Animate_Tripod(T,Y,n,m,s,WayPts,WPtol,buffer,~,Color)
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

% Takes every Nth point from Tu and Yu to plot, to speed up animation
N = 10;  % Default 5
Tu = Tu(1:N:length(Tu));
[row, ~] = size(Yu);
Yu = Yu(1:N:row,:);

% % Creates Video
% TripodVid = VideoWriter('TripodAnimation.avi');
% TripodVid.FrameRate = 25;  % Frame Rate: Default 30, 25 for small file
% TripodVid.Quality = 75;   % Quality: Default 75/100, 50 for small file
% open(TripodVid)
% % Sets Animation Parameters
% hfig = figure('Position',[1 0 1680 1050],'MenuBar','none','ToolBar','none','resize','off'); % fullscreen
% % (Resolution set for RML desktop)

% Preallocates tracking vectors (for Tu)
pAVec = zeros(2,length(Tu));
% pBVec = zeros(2,length(Tu));
% pCVec = zeros(2,length(Tu));
% pDVec = zeros(2,length(Tu));
pEVec = zeros(2,length(Tu));
% pFVec = zeros(2,length(Tu));
% pGVec = zeros(2,length(Tu));
% pHVec = zeros(2,length(Tu));
% pIVec = zeros(2,length(Tu));

% Local Position Vectors
sA1 = s(:,2);
sB1 = s(:,3);
sC2 = s(:,5);
sD3 = s(:,7);
sE4 = s(:,9);
sF5 = s(:,11);
sG6 = s(:,13);
sH7 = s(:,15);
sI8 = s(:,17);

% % Sets window size
%[x_low, x_high, y_low, y_high] = f_window_size(WayPts,buffer);

[x_low,x_high,y_low,y_high] = WindowSizer2(WayPts,buffer);

i = 1;
%while i <= length(T)
while i <= length(Tu)
    
    %q = Y(i, n+m+1:n+m+n);   % (Local) Position components of the state vector Y
    q = Yu(i, n+m+1:n+m+n);   % (Local) Position components of the state vector Yu
    
    %R1 = f_RM(q(3));
    %R2 = f_RM(q(6));
    %R3 = f_RM(q(9));
    %R4 = f_RM(q(12));
    
%     if i>1 && isempty(q(i-1,1:n)) == 0 && q(i,1:n) == q(i-1,1:n)
%         q(i,n) = [];
%     end
    
    % Coordinates of Points
    %pO = [0; 0];           % Coords of Pt O
    pA = f_r(q, 1, sA1);    % Coords of Pt A
    p1 = f_cg(q, 1);        % Coords of CG1
    pB = f_r(q, 1, sB1);   % Coords of Pt B
    p2 = f_cg(q, 2);        % Coords of CG2
    pC = f_r(q, 2, sC2);   % Coords of Pt C
    p3 = f_cg(q, 3);        % Coords of CG3
    pD = f_r(q, 3, sD3);   % Coords of Pt D
    p4 = f_cg(q, 4);        % Coords of CG4
    pE = f_r(q, 4, sE4);   % Coords of Pt E
    p5 = f_cg(q, 5);        % Coords of CG5
    pF = f_r(q, 5, sF5);   % Coords of Pt F
    p6 = f_cg(q, 6);        % Coords of CG6
    pG = f_r(q, 6, sG6);   % Coords of Pt G
    p7 = f_cg(q, 7);        % Coords of CG7
    pH = f_r(q, 7, sH7);   % Coords of Pt H
    p8 = f_cg(q, 8);        % Coords of CG8
    pI = f_r(q, 8, sI8);   % Coords of Pt I
    
    pAVec(1,i) = pA(1);     % Stores x-component of trajectory of Pt A
    pAVec(2,i) = pA(2);     % Stores y-component of trajectory of Pt A
%     pBVec(1,i) = pB(1);     % Stores x-component of trajectory of Pt B 
%     pBVec(2,i) = pB(2);     % Stores y-component of trajectory of Pt B
%     pCVec(1,i) = pC(1);     % Stores x-component of trajectory of Pt C
%     pCVec(2,i) = pC(2);     % Stores y-component of trajectory of Pt C
%     pDVec(1,i) = pD(1);     % Stores x-component of trajectory of Pt D
%     pDVec(2,i) = pD(2);     % Stores y-component of trajectory of Pt D
    pEVec(1,i) = pE(1);     % Stores x-component of trajectory of Pt E
    pEVec(2,i) = pE(2);     % Stores y-component of trajectory of Pt E    
%     pFVec(1,i) = pF(1);     % Stores x-component of trajectory of Pt F
%     pFVec(2,i) = pF(2);     % Stores y-component of trajectory of Pt F
%     pGVec(1,i) = pG(1);     % Stores x-component of trajectory of Pt G
%     pGVec(2,i) = pG(2);     % Stores y-component of trajectory of Pt G
%     pHVec(1,i) = pH(1);     % Stores x-component of trajectory of Pt H
%     pHVec(2,i) = pH(2);     % Stores y-component of trajectory of Pt H
%     pIVec(1,i) = pI(1);     % Stores x-component of trajectory of Pt I
%     pIVec(2,i) = pI(2);     % Stores y-component of trajectory of Pt I

    % Plots  
    plot([pA(1) p1(1) pB(1)], [pA(2) p1(2) pB(2)], Color,  'LineWidth', 2.8);  % Plots vector between Pt A & Pt B
    hold on;
    plot([pB(1) p2(1) pC(1)], [pB(2) p2(2) pC(2)], Color,  'LineWidth', 2.8);  % Plots vector between Pt B & Pt C
    plot([pC(1) p3(1) pD(1)], [pC(2) p3(2) pD(2)], Color,  'LineWidth', 2.8);  % Plots vector between Pt C & Pt D
    plot([pD(1) p4(1) pE(1)], [pD(2) p4(2) pE(2)], Color,  'LineWidth', 2.8);  % Plots vector between Pt D & Pt E
    plot([pE(1) p5(1) pF(1)], [pE(2) p5(2) pF(2)], Color,  'LineWidth', 2.8);  % Plots vector between Pt E & Pt F
    plot([pF(1) p6(1) pG(1)], [pF(2) p6(2) pG(2)], Color,  'LineWidth', 2.8);  % Plots vector between Pt F & Pt G
    plot([pG(1) p7(1) pH(1)], [pG(2) p7(2) pH(2)], Color,  'LineWidth', 2.8);  % Plots vector between Pt G & Pt H
    plot([pH(1) p8(1) pI(1)], [pH(2) p8(2) pI(2)], Color,  'LineWidth', 2.8);  % Plots vector between Pt H & Pt I

%     plot([pA(1) pB(1) pC(1) pD(1) pE(1) pF(1) pG(1) pH(1) pI(1)], [pA(2) pB(2) pC(2) pD(2) pE(2) pF(2) pG(2) pH(2) pI(2)], 'blacko', 'MarkerSize', 4.5);  % Plots points at Pts A,B,C,D,E,F,G,H,I
    plot([pA(1) pB(1) pC(1) pD(1) pE(1) pF(1) pG(1) pH(1) pI(1)], [pA(2) pB(2) pC(2) pD(2) pE(2) pF(2) pG(2) pH(2) pI(2)], 'blacko', 'MarkerSize', 5,'MarkerFaceColor','k');  % Plots points at Pts A,B,C,D,E,F,G,H,I
    %plot([p1(1) p2(1) p3(1) p4(1) p5(1) p6(1) p7(1) p8(1)], [p1(2) p2(2) p3(2) p4(2) p5(2) p6(2) p7(2) p8(2)], 'black*', 'MarkerSize',3);                % Plots points at the local frames
    
    plot([pE(1)], [pE(2)], 'blacko', 'MarkerFaceColor','k','MarkerSize', 5);  % Plots points at Pts A,B,C,D,E,F,G,H,I
    
    % Plot ground
    gpt = 0;
%     plot([-1 0 1], [gpt gpt gpt], 'r',  'LineWidth', 3);
    plot([-5 0 5], [gpt gpt gpt], 'k',  'LineWidth', 3);

%     iii = 0.01;
%     while iii < 0.1
%         plot([-1 0 1], [-iii -iii -iii], 'k',  'LineWidth', 12);
%         iii = iii+0.01;
%     end
    
% plot(pEVec(1,1:i),pEVec(2,1:i), 'b', 'LineWidth', 1);  % Traces head

% % Traces 'R', 'M', and 'L' in different colors
%     if pA(1) < 35
%         plot(pAVec(1,1:i),pAVec(2,1:i), 'b', 'LineWidth', 1);  % Traces head
%     elseif pA(1) >= 35 && pA(1) < 114
%         if pA(1) >= 35 && pAVec(1,i-1) < 35
%              i1 = i;
%         end
%         plot(pAVec(1,1:i1),pAVec(2,1:i1), 'b', 'LineWidth', 1);  % Traces head
%         plot(pAVec(1,i1:i),pAVec(2,i1:i), 'r', 'LineWidth', 1);  % Traces head
%     elseif pA(1) >= 114
%         if pA(1) >= 114 && pAVec(1,i-1) < 114
%              i2 = i;
%         end
%         plot(pAVec(1,1:i1),pAVec(2,1:i1), 'b', 'LineWidth', 1);          % Traces head
%         plot(pAVec(1,i1:i2),pAVec(2,i1:i2), 'r', 'LineWidth', 1);        % Traces head
%         plot(pAVec(1,i2:i),pAVec(2,i2:i), 'Color',[0 0.8 0], 'LineWidth', 1);  % Traces head
%     end

    %plot(pBVec(1,1:i),pBVec(2,1:i), 'r', 'LineWidth', 2);   % Traces end of 1st link
    %plot(pCVec(1,1:i),pCVec(2,1:i), 'g', 'LineWidth', 2);   % Traces end of 2nd link
    %plot(pDVec(1,1:i),pDVec(2,1:i), 'y', 'LineWidth', 2);   % Traces end of 3rd link
    %plot(pEVec(1,1:i),pEVec(2,1:i), 'r', 'LineWidth', 2);   % Traces end of 4th link
    %plot(pFVec(1,1:i),pFVec(2,1:i), 'g', 'LineWidth', 2);   % Traces end of 5th link
    %plot(pGVec(1,1:i),pGVec(2,1:i), 'y', 'LineWidth', 2);   % Traces end of 6th link
    %plot(pHVec(1,1:i),pHVec(2,1:i), 'b', 'LineWidth', 2);   % Traces end of 7th link
    %plot(pIVec(1,1:i),pIVec(2,1:i), 'r', 'LineWidth', 2);   % Traces end of 8th link
    
    % % Defines a pt located one body length away, along the axis of 1st link
    % Target = [pA(1);pA(2)] + 2*BL*R1*[0;1];
    % plot([pA(1) Target(1)], [pA(2) Target(2)], 'y--', 'Linewidth', 3);  % Plots field of view

%     % Plotting Way Points
%     kk = 1;
%     while kk < length(WayPts)
%        % Way Pts
%        plot(WayPts(1,kk),WayPts(2,kk),'o','MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0],'MarkerSize', 6);
% %        plot(WayPts(1,kk),WayPts(2,kk),'ro','LineWidth', 3);
% 
%        % Paths btwn Way Pts
%        if kk < length(WayPts)-1
%             xx = linspace(WayPts(1,kk),WayPts(1,kk+1),50);
%             yy = linspace(WayPts(2,kk),WayPts(2,kk+1),50);
%        plot(xx,yy,'black--','LineWidth',0.9);
%        end
% 
%        kk = kk+1;
%     end
    
%     % Plotting Acceptance Circles
%     mm = 1;
%     while mm < length(WayPts)
%       f_circle(WayPts(1,mm),WayPts(2,mm),WPtol,[1 0 0]);
%        mm = mm+1;
%     end
    
%     % Axes Labels
%     xlabel('{\itx}-position ({\itL})','FontSize',38,'FontName','Times New Roman');
%     ylabel('{\ity}-position ({\itL})','FontSize',38,'FontName','Times New Roman');
    
%     % Title
%     if FricType == 1
%         title('Simulation of Slithering Locomotion (viscous friction)','FontSize',20,'FontName','Times New Roman');
%     elseif FricType == 2
%         title('Simulation of Slithering Locomotion (Coulomb friction)','FontSize',20,'FontName','Times New Roman');
%     else
%         title('Simulation of Slithering Locomotion','FontSize',20,'FontName','Times New Roman');
%     end
    
%     Set axis limits
%     xlim([x_low x_high])
%     ylim([y_low y_high])
    
%     xlim([-0.3 0.7]);
%     ylim([-0.3 0.7]);

    xlim([-0.1 1]);
    ylim([-0.1 1]);

    set(gca, 'FontSize',28);
    grid on;
%     axis equal;
    axis square;
    
%     pause(Tu(2)-Tu(1));
    pause(0.02);
    
    hold off;
    
    i = i+1;
     
%     % Writes each frame to a video file
%     frame = getframe;
%     %frame = getframe(hfig); % Displays graph WITH axes
%     writeVideo(TripodVid,frame);
end
%     close(TripodVid);
end
