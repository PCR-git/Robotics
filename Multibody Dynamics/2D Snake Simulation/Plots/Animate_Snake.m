% Animation of the Snake Robot

function Animate_Snake(T,Y,n,m,WayPts,WPtol,buffer)
%pause(2)

% Takes every Nth point from T and Y to plot, to speed up animation
N = 5;
T = T(1:N:length(T));
[row, ~] = size(Y);
Y = Y(1:N:row,:);

% % Creates Video
% SnakeVid = VideoWriter('SnakeAnimation.avi');

% SnakeVid.FrameRate = 30;  % Frame Rate: Default 30
% SnakeVid.Quality = 75;   % Quality: Default 75/100

% open(SnakeVid)

% Preallocates tracking vectors
pAVec = zeros(2,length(T));
pBVec = zeros(2,length(T));
pCVec = zeros(2,length(T));
pDVec = zeros(2,length(T));

% % Sets Animation Parameters
% figure('Position',[1 0 1920 1200],'MenuBar','none','ToolBar','none','resize','off') % fullscreen

% Way Point Definitions
rw1 = WayPts(:,1);
rw2 = WayPts(:,2);
rw3 = WayPts(:,3);
rw4 = WayPts(:,4);

wx1 = rw1(1); wy1 = rw1(2);
wx2 = rw2(1); wy2 = rw2(2);
wx3 = rw3(1); wy3 = rw3(2);
wx4 = rw4(1); wy4 = rw4(2);

xx12 = linspace(wx1,wx2,50);
yy12 = linspace(wy1,wy2,50);
xx23 = linspace(wx2,wx3,50);
yy23 = linspace(wy2,wy3,50);
xx34 = linspace(wx3,wx4,50);
yy34 = linspace(wy3,wy4,50);

% Sets window size
[x_low, x_high, y_low, y_high] = f_window_size(WayPts,buffer);

i=1;
while i <= length(T)
    
    q = Y(i, n+m+1:n+m+n);    % (Local) Position components of the state vector Y
    
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
    %pA = [0; 0];           % Coords of Pt A
    pA = f_r(q, 1, s_pa);   % Coords of Pt A
    p1 = f_cg(q, 1);        % Coords of CG1
    pB = f_r(q, 1, s_pb);   % Coords of Pt B
    p2 = f_cg(q, 2);        % Coords of CG2
    pC = f_r(q, 2, s_pb);   % Coords of Pt C
    p3 = f_cg(q, 3);        % Coords of CG3
    pD = f_r(q, 3, s_pb);   % Coords of Pt D
    
    % Defines a pt located one body length away, along the axis of 1st link
    %Target = [pA(1);pA(2)] + 2*BL*R1*[0;1];
    
    pAVec(1,i) = pA(1);     % Stores x-component of trajectory of Pt A
    pAVec(2,i) = pA(2);     % Stores y-component of trajectory of Pt A
    pBVec(1,i) = pB(1);     % Stores x-component of trajectory of Pt B 
    pBVec(2,i) = pB(2);     % Stores y-component of trajectory of Pt B
    pCVec(1,i) = pC(1);     % Stores x-component of trajectory of Pt C
    pCVec(2,i) = pC(2);     % Stores y-component of trajectory of Pt C
    pDVec(1,i) = pD(1);     % Stores x-component of trajectory of Pt D
    pDVec(2,i) = pD(2);     % Stores y-component of trajectory of Pt D
    
    % Plots
    plot([pA(1) p1(1) pB(1)], [pA(2) p1(2) pB(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt A & Pt B
    hold on;
    plot([pB(1) p2(1) pC(1)], [pB(2) p2(2) pC(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt B & Pt C
    plot([pC(1) p3(1) pD(1)], [pC(2) p3(2) pD(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt C & Pt D
    plot([pA(1) pB(1) pC(1) pD(1)], [pA(2) pB(2) pC(2) pD(2)], 'blacko', 'MarkerSize',5);  % Plots points at Pts A, B, C, & D
    %plot([p1(1) p2(1) p3(1)], [p1(2) p2(2) p3(2)], 'blacko', 'MarkerSize',3);             % Plots points at Pt A, CG1, & Pt B
    
    plot(pAVec(1,1:i),pAVec(2,1:i), 'b', 'LineWidth', 1);    % Traces head
    %plot(pBVec(1,1:i),pBVec(2,1:i), 'r', 'LineWidth', 2);   % Traces end of 1st link
    %plot(pCVec(1,1:i),pCVec(2,1:i), 'g', 'LineWidth', 2);   % Traces end of 2nd link
    %plot(pDVec(1,1:i),pDVec(2,1:i), 'y', 'LineWidth', 2);   % Traces end of 3rd link
    
    %plot([pA(1) Target(1)], [pA(2) Target(2)], 'y--', 'Linewidth', 3);

    % Plotting Way Points and Desired Trajectories
    plot(wx1,wy1,'bo','LineWidth', 3);
    plot(wx2,wy2,'ro','LineWidth', 3);
    plot(wx3,wy3,'go','LineWidth', 3);
    plot(wx4,wy4,'mo','LineWidth', 3);
    
    plot(xx12,yy12,'black--','LineWidth',1);
    plot(xx23,yy23,'black--','LineWidth',1);
    plot(xx34,yy34,'black--','LineWidth',1);
    
    % Plotting Acceptance Circles
    f_circle(wx1,wy1,WPtol,'b');
    f_circle(wx2,wy2,WPtol,'r');
    f_circle(wx3,wy3,WPtol,'g');
    f_circle(wx4,wy4,WPtol,'m');
    
    % Plot specs
    xlabel('X Position','FontSize',20);
    ylabel('Y Position','FontSize',20);
    title('Animation of Snake Robot','FontSize',20);

    % Set axis limits
    xlim([x_low x_high]);
    ylim([y_low y_high]);
    
    %set(gca, 'FontSize',15);
    grid on;
    axis equal;
    
    pause(T(2)-T(1));
    %pause(0.01);
    hold off;
    
    i = i+1;
    
%     % Writes each frame to a video file
%     frame = getframe;
%     writeVideo(SnakeVid,frame);
end

%     close(SnakeVid);
end
