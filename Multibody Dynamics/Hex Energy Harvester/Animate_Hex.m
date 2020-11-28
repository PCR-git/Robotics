function Animate_Hex(T,Y,n,m,s,LU,LSx,LSy)
pause(2)

% % Creates Video
% HexVid = VideoWriter('HexAnimation.avi');
% 
% HexVid.FrameRate = 30;  % Frame Rate: Default 30
% HexVid.Quality = 100;   % Quality: Default 75/100
% 
% open(HexVid)

% Preallocates tracking vectors
%pAVec = zeros(2,length(T));

% % Sets Window Size
%[x_low, x_high, y_low, y_high] = f_windowsize(r_final);

% % Sets Animation Parameters
% figure('Position',[1 0 1920 1200],'MenuBar','none','ToolBar','none','resize','off') % fullscreen

i=1;
while i <= length(T)
    
    q = Y(i, n+m+1:n+m+n);    % (Local) Position components of the state vector Y
    
    % Local Vectors
    sA1 = s(:,1);
    %sA2 = s(:,2);
    sB2 = s(:,3);
    %sB3 = s(:,4);
    sC3 = s(:,5);
    %sC4 = s(:,6);
    sD4 = s(:,7);
    %sD5 = s(:,8);
    sE5 = s(:,9);
    %sE6 = s(:,10);
    sF6 = s(:,11);
    %sF1 = s(:,12);
    %sB7 = s(:,13);
    %sG7 = s(:,14);
    %sE8 = s(:,15);
    %sG8 = s(:,16);
    
    % Rotation Matrices
    A1 = f_RM(q(3));
    A2 = f_RM(q(6));
    A3 = f_RM(q(9));
    A4 = f_RM(q(12));
    A5 = f_RM(q(15));
    A6 = f_RM(q(18));
    %A7 = f_RM(q(21));
    %A8 = f_RM(q(24)); 

    % Local Frames
    p1 = [q(1);q(2)];
    p2 = [q(4);q(5)];
    p3 = [q(7);q(8)];
    p4 = [q(10);q(11)];
    p5 = [q(13);q(14)];
    p6 = [q(16);q(17)];
    p7 = [q(19);q(20)];
    p8 = [q(22);q(23)];
    
    % Joints
    pA = p1+A1*sA1;
    pB = p2+A2*sB2;
    pC = p3+A3*sC3;
    pD = p4+A4*sD4;
    pE = p5+A5*sE5;
    pF = p6+A6*sF6;
    %pG = p7+A7*sG7;
    pG = pB+(pE-pB)/2;
    
    %pAVec(1,i) = pA(1);     % Stores x-component of trajectory of Pt A
    %pAVec(2,i) = pA(2);     % Stores y-component of trajectory of Pt A
    
    % Plots
    plot([pF(1) p1(1) pA(1)], [pF(2) p1(2) pA(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt F & Pt A
    hold on;
    plot([pA(1) p2(1) pB(1)], [pA(2) p2(2) pB(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt A & Pt B
    plot([pB(1) p3(1) pC(1)], [pB(2) p3(2) pC(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt B & Pt C
    plot([pC(1) p4(1) pD(1)], [pC(2) p4(2) pD(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt C & Pt D
    plot([pD(1) p5(1) pE(1)], [pD(2) p5(2) pE(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt D & Pt E
    plot([pE(1) p6(1) pF(1)], [pE(2) p6(2) pF(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt E & Pt F
    plot([pB(1) p7(1) pG(1)], [pB(2) p7(2) pG(2)], 'black',  'LineWidth', 3);              % Plots vector between Pt B & Pt G
    plot([pG(1) p8(1) pE(1)], [pG(2) p8(2) pE(2)], 'black',  'LineWidth', 5);              % Plots vector between Pt E & Pt G
    
    %plot([pA(1) pB(1) pC(1) pD(1) pE(1) pF(1)], [pA(2) pB(2) pC(2) pD(2) pE(2) pF(2)], 'blacko', 'Linewidth',4);  % Plots revolute joints
    plot([pA(1) pB(1) pC(1) pD(1) pE(1) pF(1)], [pA(2) pB(2) pC(2) pD(2) pE(2) pF(2)], 'blacko', 'MarkerSize',6,'MarkerFaceColor','black');  % Plots revolute joints
    %plot([p1(1) p2(1) p3(1) p4(1) p5(1) p6(1) p7(1) p8(1)], [p1(2) p2(2) p3(2) p4(2) p5(2) p6(2) p7(2) p8(2)], 'blacko','MarkerSize',7); % Plots frames
    
    % Plotting uppermost mass
    pU1 = [pC(1);pC(2)+LU];
    pU2 = [pD(1);pD(2)+LU];
    xxU = [pC(1), pU1(1), pU2(1), pD(1)];
    yyU = [pC(2), pU1(2), pU2(2), pD(2)];
    patch(xxU,yyU,'black');
    
    % Plotting left side mass
    pLS1 = [pB(1)-LSx; pB(2)+LSy];
    pLS2 = [pB(1)+LSx; pB(2)+LSy];
    pLS3 = [pB(1)+LSx; pB(2)-LSy];
    pLS4 = [pB(1)-LSx; pB(2)-LSy];
    xxLS = [pLS1(1),pLS2(1),pLS3(1),pLS4(1)];
    yyLS = [pLS1(2),pLS2(2),pLS3(2),pLS4(2)];
    patch(xxLS,yyLS,'black');
    
    % Plotting right side mass
    pRS1 = [pE(1)-LSx; pE(2)+LSy];
    pRS2 = [pE(1)+LSx; pE(2)+LSy];
    pRS3 = [pE(1)+LSx; pE(2)-LSy];
    pRS4 = [pE(1)-LSx; pE(2)-LSy];
    xxRS = [pRS1(1),pRS2(1),pRS3(1),pRS4(1)];
    yyRS = [pRS1(2),pRS2(2),pRS3(2),pRS4(2)];
    patch(xxRS,yyRS,'black');
    
    %plot(pAVec(1,1:i),pAVec(2,1:i), 'b', 'LineWidth', 2);   % Traces pt 
    
    % Plot specs
    plot(0,0,'blacko','MarkerSize',3);  % Plots Origin
    xlabel('X Position');
    ylabel('Y Position');
    title('Animation of Hexagon Thing');
    axis equal;
    xlim([-4 4]);
    ylim([-2 6]);
    grid on;
    
    pause(T(2)-T(1));
    hold off;
    
    i = i+1;
    
%     % Writes each frame to a video file
%     frame = getframe;
%     writeVideo(HexVid,frame);
end

%     close(HexVid);
end
