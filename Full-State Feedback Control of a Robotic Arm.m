% Applied Linear Systems, Midterm Project %
% Peter Racioppo, 905955118 %

clear all
clc

% ------ Question 1b ------ %

% Masses
Mx = 100;
My = 150;
Mz = 50;
m4 = 20;
m5 = 20;

% Damping-Coefficients
bx = 50;
by = 40;
bz = 10;
b4 = 2.51;
b5 = 3.77;

% Stiffness-Coefficients
kx = 0.2;
ky = 0.1;
k4 = 7.89;
k5 = 7.89;

% Coefficients
a1 = 0.05;
a2 = 0.1;
a3 = 0.1;
a4 = 0.3;
a5 = 0.5;

% Matrix Definitions

A = [0,(1/Mx),0,0,0,0,0,0,0,0,0,0;...
    0,(-bx/Mx),-kx,0,0,0,0,0,0,0,0,0;...
    0,(1/Mx),(-kx/b5),(k5/b5),(-1/m5),0,0,0,0,0,0,0;...
    0,0,(kx/b5),(-k5/b5),0,0,0,0,0,0,0,0;...
    0,0,kx,0,0,0,0,0,0,0,0,0;...
    0,0,0,0,0,0,(1/My),0,0,0,0,0;...
    0,0,0,0,0,0,(-by/My),-ky,0,0,0,0;...
    0,0,0,0,0,0,(1/My),(-ky/b4),(k4/b4),(-1/m4),0,0;...
    0,0,0,0,0,0,0,(ky/b4),(-k4/b4),0,0,0;...
    0,0,0,0,0,0,0,ky,0,0,0,0;...
    0,0,0,0,0,0,0,0,0,0,0,1;...
    0,0,0,0,0,0,0,0,0,0,0,(-bz/Mz)];
    
B = [0,0,0,0,0;...
    0,a2,0,0,0;...
    0,0,0,0,-a5/b5;...
    0,0,0,0,a5/b5;...
    0,0,0,0,0;...
    0,0,0,0,0;...
    a1,0,0,0,0;...
    0,0,0,-a4/b4,0;...
    0,0,0,a4/b4,0;...
    0,0,0,0,0;...
    0,0,0,0,0;...
    0,0,a3/Mz,0,0];

C = [1,0,-1,0,0,0,0,0,0,0,0,0;...
    0,0,0,0,0,1,0,-1,0,0,0,0;...
    0,0,0,0,0,0,0,0,0,0,1,0];

D = [0,0,0,0,0;...
     0,0,0,0,0;...
     0,0,0,0,0];
 
% ------ Question 1c ------ %

sys= ss(A,B,C,D);
sysr=minreal(sys);
[a,b,c,d] = ssdata(sysr);

set(sys,'statename',{'p1','q2','q3','p4','y1','p5',...
     'q6','q7','p8','x2','z','z_prime'});
set(sys,'inputname',{'u1','u2','u3','u4','u5'});
set(sys,'outputname',{'y','x','z'});

% Our state-space representation is already a minimum realization.
 
% ------ Question 2 ------ %
  
P = ctrb(sys);
rank(P);

% The rank of P is 12, so the system is controllable.

% There are (the sum of 5 choose k from 1 to 5) = 31 possible subsets to check.

comb = [1 1 1 1 1;...
    1 1 1 1 0;...
    1 1 1 0 1;...
    1 1 0 1 1;...
    1 0 1 1 1;...
    0 1 1 1 1;... 
    1 1 1 0 0;...
    1 1 0 1 0;...
    1 1 0 0 1;...
    1 0 1 1 0;...
    1 0 1 0 1;...
    1 0 0 1 1;...
    0 1 1 1 0;...
    0 1 1 0 1;...
    0 1 0 1 1;...
    0 0 1 1 1;...
    1 1 0 0 0;...
    1 0 1 0 0;...
    1 0 0 1 0;...
    1 0 0 0 1;...
    0 1 1 0 0;...
    0 1 0 1 0;...
    0 1 0 0 1;...
    0 0 1 1 0;...
    0 0 1 0 1;...
    0 0 0 1 1;...
    1 0 0 0 0;...
    0 1 0 0 0;...
    0 0 1 0 0;...
    0 0 0 1 0;...
    0 0 0 0 1];

for ii = 1:31
    k = 1;
    if comb(ii,1) == 1
        Bn(:,k) = B(:,1);
        k=k+1;
    end
    if comb(ii,2) == 1
        Bn(:,k) = B(:,2);
        k=k+1;
    end
    if comb(ii,3) == 1
        Bn(:,k) = B(:,3);
        k=k+1;
    end
    if comb(ii,4) == 1
        Bn(:,k) = B(:,4);
        k=k+1;
    end
    if comb(ii,5) == 1
        Bn(:,k) = B(:,5);
    end 
    Dn = zeros(3,sum(comb(ii,:)));
    sys_n = ss(A,Bn,C,Dn);
    Pn = ctrb(sys_n);
    controllables(ii)=rank(Pn);
    clear Bn
    clear sys_n
end

controllables;

% Thus, there are four controllable subsets:
% u1,u2,u3,u4,u5; u1,u2,u3,u4; u1,u2,u3,u5; u1,u2,u3
% The three necessary control inputs are u1, u2, & u3.

% ------ Question 3 ------ %

poles=eig(A);
freq1=arrayfun(@norm,poles);

[Wn,zeta]=damp(sysr);

% frequency
freq=Wn/(2*pi);

% damping ratio
dampr=zeta;

% ------ Question 4 ------ %

x_0=[0.6;-150;0;0;0;0.5;300;0;0;0;0.7;-0.09348];
[response,time,~]=initial(sysr,x_0,3);
figure;
plot(time,response)
axis auto;
grid on;
set(gca,'xtick',0:0.25:3);
set(gca,'ytick',0:0.025:1);
xlabel('Time (s)');
ylabel('Distance from Origin (m)');
title('Open Loop Response');
legend('x','y','z');

% ------ Question 5 ------ %

% Matrix definitions for x, y, & z

Ax=[0,(1/Mx),0,0,0;...
    0,(-bx/Mx),-kx,0,0;...
    0,(1/Mx),(-kx/b5),(k5/b5),(-1/m5);...
    0,0,(kx/b5),(-k5/b5),0;...
    0,0,kx,0,0];...
Bx=[0,0;...
    a2,0;...
    0,-a5/b5;...
    0,a5/b5;...
    0,0];...
Cx=[1,0,-1,0,0];
Dx=[0,0];

Ay=[0,(1/My),0,0,0;...
    0,(-by/My),-ky,0,0;...
    0,(1/My),(-ky/b4),(k4/b4),(-1/m4);...
    0,0,(ky/b4),(-k4/b4),0;...
    0,0,ky,0,0];
By=[0,0;...
    a1,0;...
    0,-a4/b4;...
    0,a4/b4;...
    0,0];
Cy=[1,0,-1,0,0];
Dy=[0,0];

Az=[0,1;...
    0,(-bz/Mz)];
Bz=[0;...
    a3/Mz];
Cz=[1,0];
Dz=[0];

% ---------------------

% x

sysx= ss(Ax,Bx,Cx,Dx);
polesx=eig(Ax);

% 0th Iteration

P_0x=[0.0000 + 0.0000i;...
  -2.1457 + 0.0000i;...
  -0.4962 + 0.0000i;...
  -0.0019 + 0.0991i;...
  -0.0019 - 0.0991i];

% Old Poles

% P_1x=[-0.03;...
%   -0.85 + 0.5i;...
%   -0.85 - 0.5i;...
%   -0.85 + 0.5i;...
%   -0.85 - 0.5i];

% minimum time constant is 0.57 seconds.

% 1st Iteration

% P_1x=linspace(-0.9,-1.1,5);

% 2nd Iteration

% ex1=2;
% ex= 0.5*ex1;
% wy= 0.1*ex1;
% n1=0.6;
% n2=0.9;
% 
% P_1x=[-0.5;...
%       n1*(-ex + wy*1i);...
%       n1*(-ex - wy*1i);...
%       n2*(-ex + wy*1i);...
%       n2*(-ex - wy*1i)];

% 3rd Iteration

% ex1=2;
% ex= 0.3*ex1;
% wy= 0.1*ex1;
% n1=0.6;
% n2=0.9;
% 
% P_1x=[-0.5;...
%       n1*(-ex + wy*1i);...
%       n1*(-ex - wy*1i);...
%       n2*(-ex + wy*1i);...
%       n2*(-ex - wy*1i)];
  
% 4th Iteration

% ex1=2;
% ex= 0.3*ex1;
% wy= 0.1*ex1;
% n1=0.6;
% n2=0.9;
% 
% P_1x=[-0.3;...
%       n1*(-ex + wy*1i);...
%       n1*(-ex - wy*1i);...
%       n2*(-ex + wy*1i);...
%       n2*(-ex - wy*1i)];
  
% 5th Iteration

ex1=2;
ex= 0.3*ex1;
wy= 0.1*ex1;
n1=0.6;
n2=0.9;

P_1x=[-0.1;...
      n1*(-ex + wy*1i);...
      n1*(-ex - wy*1i);...
      n2*(-ex + wy*1i);...
      n2*(-ex - wy*1i)];
  
% try to make ratio of u1 to u2 10:1 by changing
%   position of real pole relatives to complex poles

 Gx = place(Ax,Bx,P_1x);

 Bdx=[0,0;...
     0,0;...
     0,0;...
     0,0;...
     0,0];
    
Ddx=[0,0];

% define new system with modified poles
sys_1x= ss(Ax-Bx*Gx,Bdx,Cx-Dx*Gx,Ddx);

% plot the new system's poles
% figure;
% subplot(1,2,1)
% pzmap(sys_1x);

% plot of the closed loop response, Trial 1
x_0=[1,0,0,0,0];
[y1x,tx,x1x]=initial(sys_1x,x_0,35);

% subplot(1,2,2)
% plot(tx,y1x)
% xlabel('Time (s)');
% ylabel('Distance from Origin (m)');
% title('Closed Loop Response, X-Coordinate');

% for ii=1:length(tx)
%     if tx(ii) <=8
%         k=ii;
%     end
% end
% 
% value_8_x= y1x(k)

for ii=1:length(tx)
    if tx(ii) >= 8
        k=ii;
        valuex(k)=y1x(k);
    end
end

maxdiffx=max(abs(valuex-0))

for ii=1:length(tx)
    if tx(ii) >= 0
        k=ii;
        overx(k)=y1x(k);
    end
end

overshootx=max(abs(overx-0))

inputx=Gx*transpose(x1x);
max_x=[max(abs(inputx(1,:))), max(abs(inputx(2,:)))]

u2=inputx(1,:);
u5=inputx(2,:);

% figure;
% subplot(5,2,2)
% plot(tx,u2)
% subplot(5,2,1)
% plot(tx,y1x)
%xlabel('Time (s)');
%ylabel('Distance from Origin (m)');
%title('Closed Loop Response, X-Coordinate');

% -------------------

% y

sysy= ss(Ay,By,Cy,Dy);
polesy=eig(Ay);

% 0th Iteration

P_0y=[0.0000 + 0.0000i;...
  -3.1832 + 0.0000i;...
  -0.2643 + 0.0000i;...
  -0.0012 + 0.0706i;...
  -0.0012 - 0.0706i];

% Old Poles

% P_1y=[-0.021;...
%   -0.51 + 0.35i;...
%   -0.51 - 0.35i;...
%   -0.51 + 0.35i;...
%   -0.51 - 0.35i];

% 1st Iteration

% P_1y=linspace(-.9,-1.1,5);

% 2nd Iteration

% exy1=2;
% exy= 0.3*exy1;
% wyy= 0.1*exy1;
% n1y=0.6;
% n2y=0.9;
% 
% P_1y=[-0.5;...
%       n1y*(-exy + wyy*1i);...
%       n1y*(-exy - wyy*1i);...
%       n2y*(-exy + wyy*1i);...
%       n2y*(-exy - wyy*1i)];

% 3rd Iteration

% exy1=2.4;
% exy= 0.17*exy1;
% wyy= 0.1*exy1;
% n1y=0.7;
% n2y=0.8;
% 
% P_1y=[-0.3;...
%       n1y*(-exy + wyy*1i);...
%       n1y*(-exy - wyy*1i);...
%       n2y*(-exy + wyy*1i);...
%       n2y*(-exy - wyy*1i)];

% 4th Iteration

% exy1=2.7;
% exy= 0.17*exy1;
% wyy= 0.07*exy1;
% n1y=0.6;
% n2y=0.85;
% 
% P_1y=[-0.15;...
%       n1y*(-exy + wyy*1i);...
%       n1y*(-exy - wyy*1i);...
%       n2y*(-exy + wyy*1i);...
%       n2y*(-exy - wyy*1i)];

% 5th Iteration

exy1=2.9;
exy= 0.17*exy1;
wyy= 0.07*exy1;
n1y=0.6;
n2y=0.85;

P_1y=[-0.1;...
      n1y*(-exy + wyy*1i);...
      n1y*(-exy - wyy*1i);...
      n2y*(-exy + wyy*1i);...
      n2y*(-exy - wyy*1i)];
  
Gy = place(Ay,By,P_1y);

Bdy=[0,0;...
    0,0;...
    0,0;...
    0,0;...
    0,0];
    
Ddy=[0,0];

% define new system with modified poles
sys_1y= ss(Ay-By*Gy,Bdy,Cy-Dy*Gy,Ddy);

% plot the new system's poles
% figure;
% subplot(1,2,1)
% pzmap(sys_1y);

% plot of the closed loop response, Trial 1
y_0=[1,0,0,0,0];
[y1y,ty,x1y]=initial(sys_1y,y_0,35);

% subplot(1,2,2)
% plot(ty,y1y)
% axis auto;
% xlabel('Time (s)');
% ylabel('Distance from Origin (m)');
% title('Closed Loop Response, Y-Coordinate');

% for ii=1:length(ty)
%     if ty(ii) <=8
%         k=ii;
%     end
% end

% value_8_y= y1y(k)

for ii=1:length(ty)
    if ty(ii) >= 8
        k=ii;
        valuey(k)=y1y(k);
    end
end

maxdiffy=max(abs(valuey-0))

for ii=1:length(ty)
    if ty(ii) >= 0
        k=ii;
        overy(k)=y1y(k);
    end
end

overshooty=max(abs(overy-0))

inputy=Gy*transpose(x1y);
max_y=[max(abs(inputy(1,:))), max(abs(inputy(2,:)))]

u1=inputy(1,:);
u4=inputy(2,:);

% -------------------

% z

sysz= ss(Az,Bz,Cz,Dz);
polesz=eig(Az);

% 0th Iteration

P_0z=[0;...
      -0.2];

% 1st Iteration

% P_1z=linspace(-0.4,-0.5,2);

% 2nd Iteration

% P_1z= [-0.45+0.2i;...
%        -0.45-0.2i];

% 3rd Iteration

% P_1z= [-0.45+0.4i;...
%        -0.45-0.4i];

% 4th Iteration

% P_1z=[-0.45+0.3i;...
%       -0.45-0.3i];

% 5th Iteration

P_1z=[-0.45+0.29i;...
      -0.45-0.29i];

Gz = place(Az,Bz,P_1z);

Bdz=[0;...
    0];

Ddz=0;

% define new system with modified poles
sys_1z= ss(Az-Bz*Gz,Bdz,Cz-Dz*Gz,Ddz);

% plot the new system's poles
% figure;
% subplot(1,2,1)
% pzmap(sys_1z);

% plot of the closed loop response, Trial 1
z_0=[1,0];
[y1z,tz,x1z]=initial(sys_1z,z_0,10);

% subplot(1,2,2)
% plot(tz,y1z)
% axis auto;
% xlabel('Time (s)');
% ylabel('Distance from Origin (m)');
% title('Closed Loop Response, Z-Coordinate');

% for ii=1:length(tz)
%     if tz(ii) <=8
%         k=ii;
%     end
% end
% 
% value_8_z= y1z(k)

for ii=1:length(tz)
    if tz(ii) >= 8
        k=ii;
        valuez(k)=y1z(k);
    end
end

maxdiffz=max(abs(valuez-0))

for ii=1:length(tz)
    if tz(ii) >= 0
        k=ii;
        overz(k)=y1z(k);
    end
end

overshootz=max(abs(overz-0))

inputz=Gz*transpose(x1z);
max_z=[max(abs(inputz(1,:)))]

u3=inputz(1,:);

% --------------------------------------------

% Tables

% X

Iterationx = ['1';'2';'3';'4';'5'];
Maxinput1x = [1122.61,460.14,237.44,162.72,193.77];
Maxinput2x = [747.03,251.97,66.91,78.00,24.02];
Overshootx = [23.73,11.77,4.77,5.26,2.32];
Distancex = [0.80,1.74,1.40,1.30,0.25];
Specx={'N';'N';'N';'N';'N'};

% -------------------

% Y

Iterationy = ['1';'2';'3';'4';'5'];
Maxinput1y = [3460.90;803.87;396.98;306.98;375.09];
Maxinput2y = [1662.90;313.29;161.24;93.11;69.41];
Overshooty = [46.57;12.15;6.88;4.41;3.53];
Distancey = [1.60;3.67;2.30;0.91;0.51];
Specy = ['N';'N';'N';'N';'N'];

% -------------------

% Z

Iterationz = ['1';'2';'3';'4';'5'];
Maxinputz = [100.00;121.25;181.25;146.25;143.30];
Overshootz = [1.00;1.00;1.00;1.00;1.00];
Distancez = [0.13;0.06;0.03;0.01;0.01];
Specz = ['N';'N';'N';'Y';'Y'];

% -------------------

% Tables, contd.

% Tx = table(Iterationx,Maxinput1x,Maxinput2x,Overshootx,Distancex,...
%     'VariableNames',{'Iteration' 'Maxinput_u1' 'Maxinput_u4' 'Overshoot' 'Distance'});
% 
% Ty = table(Iterationy,Maxinput1y,Maxinput2y,Overshooty,Distancey,Specy,...
%     'VariableNames',{'Iteration' 'Maxinput_u1' 'Maxinput_u4' 'Overshoot' 'Distance' 'Specs'});
% 
% Tz = table(Iterationz,Maxinputz,Overshootz,Distancez, Specz,...
%     'VariableNames',{'Iteration' 'Maxinput_u3' 'Overshoot' 'Distance' 'Specs'});

% --------------------------------------------

% Figure

f = figure;
set(gcf, 'numbertitle', 'off', 'name', 'Final Design');
set(f, 'Units', 'normalized', 'Position', [0.2, 0.1, 0.7, 0.8]);

% subplot(6, 2, [1,  3])
subplot('position',[0.1 0.75 0.35 0.21])
plot(tx, y1x)
title('Output, X-Direction', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Displacement (m)', 'FontSize',15)
set(gca,'fontsize',15)
grid on

% subplot(6, 2, [5,  7])
subplot('position',[0.1 0.42 0.35 0.21])
plot(ty, y1y)
title('Output, Y-Direction', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Displacement (m)', 'FontSize',15)
set(gca,'fontsize',15)
grid on

%subplot(6, 2, [9, 11])
subplot('position',[0.1 0.1 0.35 0.21])
plot(tz, y1z)
title('Output, Z-Direction', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Displacement (m)', 'FontSize',15)
set(gca,'fontsize',15)
grid on

% subplot(6, 2,  4)
subplot('position',[0.55 0.63 0.35 0.10])
plot(tx, u2)
title('Input u2', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Input-Voltage (V)', 'FontSize',15)
set(gca,'fontsize',15)
grid on

% subplot(6, 2,  10)
subplot('position',[0.55 0.05 0.35 0.10])
plot(tx, u5)
title('Input u5', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Input-Voltage (V)', 'FontSize',15)
set(gca,'fontsize',15)
grid on

% subplot(6, 2,  2)
subplot('position',[0.55 0.83 0.35 0.10])
plot(ty, u1)
title('Input u1', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Input-Voltage (V)', 'FontSize',15)
set(gca,'fontsize',15)
grid on

% subplot(6, 2,  8)
subplot('position',[0.55 0.24 0.35 0.10])
plot(ty, u4)
title('Input u4', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Input-Voltage (V)', 'FontSize',15)
set(gca,'fontsize',15)
grid on

% subplot(6, 2, 6)
subplot('position',[0.55 0.44 0.35 0.10])
plot(tz, u3)
title('Input u3', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Input-Voltage (V)', 'FontSize',15)
set(gca,'fontsize',15)
grid on

% -------------------  -------------------
% -------------------  -------------------

% Applied Linear Systems, Final Project %

% Peter Racioppo, 905955118 %

clear all
clc

% ---------------------

% Constant Definitions

% Masses
My = 150;
m4 = 20;

% Damping-Coefficients
by = 40;
b4 = 2.51;

% Stiffness-Coefficients
ky = 0.1;
k4 = 7.89;

% Coefficients
a1 = 0.05;
a4 = 0.3;

% Matrix Definitions

A=[0,(1/My),0,0,0;...
    0,(-by/My),-ky,0,0;...
    0,(1/My),(-ky/b4),(k4/b4),(-1/m4);...
    0,0,(ky/b4),(-k4/b4),0;...
    0,0,ky,0,0];
B=[0,0;...
    a1,0;...
    0,-a4/b4;...
    0,a4/b4;...
    0,0];
C=[1,0,-1,0,0];
D=[0,0];

% ---------------------

%  Define Open Loop System

sys= ss(A,B,C,D);
poles=eig(A);

% y_i=[1,0,0,0,-0.5];
% [yi,ti,xi]=initial(sys,y_i,4);
% figure;
% plot(ti,yi)

% ---------------------

% Problem 1

O=obsv(A,C)';  % ctrb(A',C');
rank(O);

% the observability matrix is full rank (5), so the
% open loop plant is fully observable.

% ---------------------

% Problem 2

% Pole Placement

% Prof. Southward's Poles
% w = 0.8;
% z1 = 0.95;
% z2 = 0.90;
% zw1 = z1*w;
% zw2 = z2*w;

% Poles must be placed further left to satisfy requirements

% New Poles

%w = 0.75;
w = 1.1;
z1 = 0.95;
z2 = 0.90;
zw1 = z1*w;
zw2 = z2*w;

P_closed = [-w,-zw1+w*sqrt(1-z1^2)*1i,-zw1-w*sqrt(1-z1^2)*1i,...
    -zw2+w*sqrt(1-z2^2)*1i,-zw2-w*sqrt(1-z2^2)*1i];

Gi = place(A,B,P_closed);

% Dummy Matrices
Bd=[0,0;...
    0,0;...
    0,0;...
    0,0;...
    0,0];
    
Dd=[0,0];

% Define Closed Loop System
sys_1= ss(A-B*Gi,Bd,C-D*Gi,Dd);

y_0=[0.2,0,0,0,-0.5];
[y1,t,x1]=initial(sys_1,y_0,20);

% -------------------

for ii=1:length(t)
    if t(ii) >= 10
        k=ii;
        value(k)=y1(k);
    end
end

% Maximum distance from origin after 10 seconds
maxdiff=max(abs(value-0));

for ii=1:length(t)
    if t(ii) >= 0
        k=ii;
        over(k)=y1(k);
    end
end

% Overshoot
overshoot=max(abs(over-0));

% Inputs
input=Gi*transpose(x1);
maxinput=[max(abs(input(1,:))), max(abs(input(2,:)))];

u1=input(1,:);
u4=input(2,:);

% -------------------

% Figures for Closed Loop System

f = figure;
set(gcf, 'numbertitle', 'off', 'name', 'Closed Loop Controller');
set(f, 'Units', 'normalized', 'Position', [0.2, 0.1, 0.7, 0.8]);

% Plot of trajectory
subplot('position',[0.1 0.3 0.35 0.5])
plot(t, y1)
hold on;
% Dotted lines, representing limits of required range
plot([0, 20], [ 0.2,  0.2],'--', 'Color',[0.4,0.4,0.4], 'LineWidth', 1.25)
plot([0, 20], [-0.2, -0.2],'--', 'Color',[0.4,0.4,0.4], 'LineWidth', 1.25)
title('Trajectory, Y-Direction', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Displacement (m)', 'FontSize',15)
set(gca,'fontsize',15)
set(gcf,'color','w');
grid on

% Plot of input, u1
subplot('position',[0.55 0.6 0.35 0.30])
plot(t, u1)
title('Input u1', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Input-Voltage (V)', 'FontSize',15)
set(gca,'fontsize',15)
set(gcf,'color','w');
grid on

% Plot of input, u2
subplot('position',[0.55 0.15 0.35 0.30])
plot(t, u4)
title('Input u4', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Input-Voltage (V)', 'FontSize',15)
set(gca,'fontsize',15)
set(gcf,'color','w');
grid on

% -------------------

% Problem 3

% Closed Loop Poles
wa = 1.5;
z1a = 0.95;
z2a = 0.90;
zw1a = z1a*wa;
zw2a = z2a*wa;

% These poles must be placed further to the left than in question 2
% since they will have less time to converge to zero.

P_closeda = [-wa,-zw1a+wa*sqrt(1-z1a^2)*1i,-zw1a-wa*sqrt(1-z1a^2)*1i,...
    -zw2a+wa*sqrt(1-z2a^2)*1i,-zw2a-wa*sqrt(1-z2a^2)*1i];

G = place(A,B,P_closeda);

% Dummy Matrices
Bd=[0,0;...
    0,0;...
    0,0;...
    0,0;...
    0,0];
    
Dd=[0,0];

% Define Closed Loop System
sys_1= ss(A-B*G,Bd,C-D*G,Dd);

y_0=[0.2,0,0,0,-0.5];
[y1,t,x1]=initial(sys_1,y_0,16);

% Observer Poles
wb = wa*5;   % 5 times farther into left plane than feedback poles
z1b = 0.95;
z2b = 0.90;
zw1b = z1b*wb;
zw2b = z2b*wb;

P_obsv = [-wb,-zw1b+wb*sqrt(1-z1b^2)*1i,-zw1b-wb*sqrt(1-z1b^2)*1i,...
    -zw2b+wb*sqrt(1-z2b^2)*1i,-zw2b-wb*sqrt(1-z2b^2)*1i];

K = place(A',C',P_obsv)';

% -------------------

% Observer Matrices

Ao = [ A    zeros(size(A))
      K*C     A-K*C ];

Co = [     C             zeros(size(C))
      zeros(size(C))         C];
  
Bo = [B;...
      B];

Do = [D;...
      D];

% ------------------

% Observer + Controller Matrices

Ac = [ A       -B*G
      K*C   A-K*C-B*G ];

Cc = [     C             -D*G
      zeros(size(C))     C-D*G];

Bc = [Bd;...
      Bd];

Dc = [Dd;...
     Dd];

% ------------------

% Create Observer System
syso = ss(Ao,Bo,Co,Do);

yo_0=[0.2,0,0,0,-0.5,0,0,0,0,0];

[y1,t1,x1]=initial(syso,yo_0,4);

% Find index at which time = 4 seconds
k=find(t1==4);

% Set state vector at k as initial conditions for next part
yc_0=x1(k,1:10);

% State trajectory and observer trajectory
yo=y1(:,1);
yobsvo=y1(:,2);

% ------------------

% Create Observer + Controller System
sysc = ss(Ac,Bc,Cc,Dc);

[y2,t2,x2]=initial(sysc,yc_0,16);

xc=x2(:,1:5);
xobsvc=x2(:,6:10);

% With controller: state trajectory and observer trajectory
yc=y2(:,1);
yobsvc=y2(:,2);

% -------------------

% We've already passed through 4 seconds, so
% we must check at 6 seconds rather than at 10

for ii=1:length(t2)
    if t2(ii) >= 6
        k=ii;
        value1(k)=yobsvc(k);
    end
end

% Max distance from origin after 10 seconds
maxdiff1=max(abs(value1-0));

for ii=1:length(t2)
    if t2(ii) >= 0
        k=ii;
        over1(k)=yc(k);
    end
end

% Overshoot
overshoot1=max(abs(over1-0));

% Inputs
input1=G*transpose(xc);
maxinput1=[max(abs(input1(1,:))), max(abs(input1(2,:)))];

u1c=input1(1,:);
u4c=input1(2,:);

% -------------------

% Open Loop Poles
figure;
pzmap(sys);
title('Open Loop Poles')
set(gcf,'color','w');

% % Closed Loop Controller Poles
% figure;
% pzmap(sys_1);

Go=place(A,B,P_obsv);
sys_obsv= ss(A-B*Go,Bd,C-D*Go,Dd);

% Observer Poles + Controller Poles
figure
%pzmap(sysc)
pzmap(sys_1)
hold on;
pzmap(sys_obsv)
title('Observer Poles + Controller Poles')
set(gcf,'color','w');
legend('Closed Loop Poles', 'Observer Poles')

% -------------------

% Problem 4

% Figure of observer without controller in first 4 seconds
figure;
plot(t1, yobsvo)
hold on;
plot(t1, yo)
title('Output, First Four Seconds', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Displacement (m)', 'FontSize',15)
set(gca,'fontsize',15)
set(gcf,'color','w');
grid on
legend('Observer','Open Loop Trajectory')

% Figures for Observer + Controller System

f1 = figure;
set(gcf, 'numbertitle', 'off', 'name', 'Complete Output Feedback Controller');
set(f1, 'Units', 'normalized', 'Position', [0.2, 0.1, 0.7, 0.8]);

% Figure of Observer and State Trajectory
subplot('position',[0.1 0.3 0.35 0.5])
plot(t1, yobsvo, t2+4, yobsvc,'Color','Red')   % Observer
hold on;
plot(t1, yo, t2+4, yc,'Color','Green')         % State trajectory
plot([0, 20], [ 0.2,  0.2],'--', 'Color',[0.4,0.4,0.4], 'LineWidth', 1.25)
plot([0, 20], [-0.2, -0.2],'--', 'Color',[0.4,0.4,0.4], 'LineWidth', 1.25)
title('Trajectory, Y-Direction', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Displacement (m)', 'FontSize',15)
set(gca,'fontsize',15)
set(gcf,'color','w');
grid on

% Figure of input, u1
subplot('position',[0.55 0.6 0.35 0.30])
plot(t1,t1*0,t2+4, u1c,'Color','Blue')
title('Input u1', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Input-Voltage (V)', 'FontSize',15)
set(gca,'fontsize',15)
grid on
hold on;
line([4 4], [0 u1c(1)],'Color', 'Blue')

% Figure of input, u4
subplot('position',[0.55 0.15 0.35 0.30])
plot(t1,t1*0,t2+4, u4c, 'Color', 'Red')
title('Input u4', 'Fontsize', 15)
xlabel('Time (s)', 'FontSize',15)
ylabel('Input-Voltage (V)', 'FontSize',15)
set(gca,'fontsize',15)
grid on
hold on;
line([4 4], [u4c(1) 0], 'Color', 'Red')
