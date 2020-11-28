function animate_leg(T,q,s,FG)
pause(1)

% % Creates Video
% SnakeVid = VideoWriter('SnakeAnimation.avi');
% 
% SnakeVid.FrameRate = 60;  % Frame Rate: Default 30
% SnakeVid.Quality = 100;   % Quality: Default 75/100
% 
% open(SnakeVid)

body1 = 1;
body2 = 2;
body3 = 3;
body4 = 4;
body5 = 5;

sA1 = [s(1,1);s(1,2)];
sB1 = [s(2,1);s(2,2)];
%sB2 = [s(3,1);s(3,2)];
%sC3 = [s(4,1);s(4,2)];
sC2 = [s(5,1);s(5,2)];
sD3 = [s(6,1);s(6,2)];
%sE3 = [s(7,1);s(7,2)];
sE4 = [s(8,1);s(8,2)];
sF4 = [s(9,1);s(9,2)];
%sF5 = [s(10,1);s(10,2)];
%sB5 = [s(11,1);s(11,2)];
%rAD = [s(12,1);s(12,2)];

% Preallocates tracking vectors
pAVec = zeros(2,T);
pBVec = zeros(2,T);
pCVec = zeros(2,T);
pDVec = zeros(2,T);
pEVec = zeros(2,T);
pFVec = zeros(2,T);
pGVec = zeros(2,T);

% % Sets Animation Parameters
% figure('Position',[1 0 1920 1200],'MenuBar','none','ToolBar','none','resize','off') % fullscreen

i=1;
while i <= T

    phi1 = q(i,body1*3);
    phi2 = q(i,body2*3);
    phi3 = q(i,body3*3);
    phi4 = q(i,body4*3);
    %phi5 = q(i,body5*3);
    
    % Rotation Matrices
    A1 = f_RM(phi1);
    A2 = f_RM(phi2);
    A3 = f_RM(phi3);
    A4 = f_RM(phi4);
    %A5 = f_RM(phi5);
    
    % Body frames
    p1 = [q(i,1+3*(body1-1));q(i,2+3*(body1-1))];
    p2 = [q(i,1+3*(body2-1));q(i,2+3*(body2-1))];
    p3 = [q(i,1+3*(body3-1));q(i,2+3*(body3-1))];
    p4 = [q(i,1+3*(body4-1));q(i,2+3*(body4-1))];
    p5 = [q(i,1+3*(body5-1));q(i,2+3*(body5-1))];
     
    % Link Ends
    pA = p1+A1*sA1;
    pB = p1+A1*sB1;
    pC = p2+A2*sC2;
    pD = p3+A3*sD3;
    pE = p4+A4*sE4;
    pF = p4+A4*sF4;
    pG = pF + A4*[FG;0];
    
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
    plot([pA(1) p1(1) pB(1)], [pA(2) p1(2) pB(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt A & Pt B
    hold on;
    plot([pB(1) p2(1) pC(1)], [pB(2) p2(2) pC(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt B & Pt C
    plot([pD(1) pC(1) pE(1)], [pD(2) pC(2) pE(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt C & Pt D
    plot([pE(1) pF(1) pG(1)], [pE(2) pF(2) pG(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt C & Pt D
    plot([pB(1) p5(1) pF(1)], [pB(2) p5(2) pF(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt C & Pt D
    
    %plot(pAVec(1,1:i),pAVec(2,1:i), 'b', 'LineWidth', 2);   % Traces head
    %plot(pBVec(1,1:i),pBVec(2,1:i), 'r', 'LineWidth', 2);   % Traces end of 1st link
    %plot(pCVec(1,1:i),pCVec(2,1:i), 'g', 'LineWidth', 2);   % Traces end of 2nd link
    %plot(pDVec(1,1:i),pDVec(2,1:i), 'y', 'LineWidth', 2);   % Traces end of 3rd link
    
    %plot([pA(1) Target(1)], [pA(2) Target(2)], 'y--', 'Linewidth', 3);

    % Plot specs
    plot(0,0,'blacko','MarkerSize',3);  % Plots Origin
    xlabel('X Position');
    ylabel('Y Position');
    title('Animation of Snake Robot');
    
    x_low = -0.75;
    y_low = -0.75;
    x_high = 1;
    y_high = 0.5;
    axis equal;
    grid on;
    xlim([x_low x_high]);
    ylim([y_low y_high]);

    pause(1);
    hold off;
    
    i = i+1;
    
%     % Writes each frame to a video file
%     frame = getframe;
%     writeVideo(SnakeVid,frame);
end

%     close(SnakeVid);
end
