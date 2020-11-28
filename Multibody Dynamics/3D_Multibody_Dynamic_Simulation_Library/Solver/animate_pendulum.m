function animate_pendulum(T, Y)
pause(2)

pBVec = zeros(3,length(T));
pCVec = zeros(3,length(T));

i=1;
while i <= length(T)
    r = Y(i, 27:32);
    P = Y(i, 33:40);
    
    s_pb = [0; -1; 0];
    s_pc = [0; -1; 0];
    
    pA = [0; 0; 0];
    p1 = f_cg(r, 1);
    pB = f_r(r, P, 1, s_pb);
    p2 = f_cg(r, 2);
    pC = f_r(r, P, 2, s_pc);
    
    pBVec(1,i) = pB(1);
    pBVec(2,i) = pB(2);
    pBVec(3,i) = pB(3);
    pCVec(1,i) = pC(1);
    pCVec(2,i) = pC(2);
    pCVec(3,i) = pC(3);
    
    plot([pA(1) p1(1) pB(1)], [pA(2) p1(2) pB(2)], 'blacko', 'LineWidth', 3);
    hold on;
    plot([pA(1) p1(1) pB(1)], [pA(2) p1(2) pB(2)], 'black',  'LineWidth', 3);
    plot([pB(1) p2(1) pC(1)], [pB(2) p2(2) pC(2)], 'blacko', 'LineWidth', 3);
    plot([pB(1) p2(1) pC(1)], [pB(2) p2(2) pC(2)], 'black',  'LineWidth', 3);
    plot([pB(1) p2(1) pC(1)], [pB(2) p2(2) pC(2)], 'blacko', 'LineWidth', 3);
    
    plot(pBVec(1,1:i),pBVec(2,1:i), 'b', 'LineWidth', 2);
    plot(pCVec(1,1:i),pCVec(2,1:i), 'r', 'LineWidth', 2);
    
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Animation of Double Pendulum');
    xlim([-4 4]);
    ylim([-4 1]);
    grid on;
    pause(T(2)-T(1));
    hold off;
    
    i = i+1;
end
