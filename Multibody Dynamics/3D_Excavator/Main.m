%% Dynamics of the 3D Excavator
% Multibody Dynamics & Simulation
% Peter Racioppo

close all
clear
clc


%% Initializations

% Declaring syms variables
syms x0 y0 z0 x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4 x5 y5 z5       % Position
syms e00 e10 e20 e30 e01 e11 e21 e31 e02 e12 e22 e32 e03 e13...  % Euler Parameters
     e23 e33 e04 e14 e24 e34 e05 e15 e25 e35
syms t                                                           % Time

nb=6; % Number of bodies

% Local Vectors
sF0 = [0.322; 1.338; 0];
sF1 = [-2.545; 0.296; 0];
sOg = [0; 0; 0];
sO0 = [0; 0; 0];
sA1 = [2.433; -1.470; 0];
sA2 = [-0.863; -0.088; 0];
sB2 = [1.690; -0.058; 0];
sB3 = [-0.766; -0.440; 0];
sC2 = [1.330; -0.058; 0];
sC4 = [-0.589/2; 0; 0];
sD4 = [0.589/2; 0; 0];
sD5 = [-0.589/2; 0; 0];
sE3 = [-0.754; -0.059; 0];
sE5 = [0.589/2; 0; 0];

sP0 = [0.792; 0.980; 0];
sQ1 = [-0.445; 0.292; 0];
sR1 = [1.027; 0.079; 0];
sS2 = [-1.393; 0.142; 0];
sT2 = [-0.787; 0.401; 0];

% Initial Position Vectors
rg = [0 ;  0;  0];
r0 = [x0; y0; z0];
r1 = [x1; y1; z1];
r2 = [x2; y2; z2];
r3 = [x3; y3; z3];
r4 = [x4; y4; z4];
r5 = [x5; y5; z5];

%  Euler Parameter Vectors
p0 = [e00; e10; e20; e30];
p1 = [e01; e11; e21; e31];
p2 = [e02; e12; e22; e32];
p3 = [e03; e13; e23; e33];
p4 = [e04; e14; e24; e34];
p5 = [e05; e15; e25; e35];

% Total Initial Position Vector
q = [r0; p0; r1; p1; r2; p2; r3; p3; r4; p4; r5; p5];

% Orientation Vectors
fi = [1; 0; 0];
gi = [0; 1; 0];
hj = [0; 0; 1];

% G Matrices (elemental rotation)
G0 = f_G(e00, e10, e20, e30);
G1 = f_G(e01, e11, e21, e31);
G2 = f_G(e02, e12, e22, e32);
G3 = f_G(e03, e13, e23, e33);
G4 = f_G(e04, e14, e24, e34);
G5 = f_G(e05, e15, e25, e35);

% A Rotation Matrices
Ag = eye(3);
A0 = f_A(e00, e10, e20, e30);
A1 = f_A(e01, e11, e21, e31);
A2 = f_A(e02, e12, e22, e32);
A3 = f_A(e03, e13, e23, e33);
A4 = f_A(e04, e14, e24, e34);
A5 = f_A(e05, e15, e25, e35);


%%  The Constraint Vector

% Distances
d01 = r1 + A1*sQ1 - r0 - A0*sP0;
d12 = r2 + A2*sS2 - r1 - A1*sR1;
d24 = r4 + A4*sD4 - r2 - A2*sT2;

% Driving constraints
s1 = 2.35 - 0.5*t;
s2 = 2.10 + 0.45*sin(10*t);
s3 = 2.00 + 0.3*sin(10*t);
s4 = sin(t/2);

% The total constraint vector
PHI = [
       % Revolute constraint, ground to body 0
       r0 + A0*sO0 - rg - Ag*sOg;...   
       transpose(fi)*transpose(Ag)*A0*gi; 
       transpose(hj)*transpose(Ag)*A0*gi;
       
       % Revolute constraint, body 0 to 1
       r1 + A1*sF1 - r0 - A0*sF0;    
       transpose(fi)*transpose(A0)*A1*hj; 
       transpose(gi)*transpose(A0)*A1*hj; 
       
       % Revolute constraint, body1 to 2
       r2 + A2*sA2 - r1 - A1*sA1;      
       transpose(fi)*transpose(A1)*A2*hj; 
       transpose(gi)*transpose(A1)*A2*hj; 
       
       % Revolute constraint, body 2 to 3
       r3 + A3*sB3 - r2 - A2*sB2;        
       transpose(fi)*transpose(A2)*A3*hj; 
       transpose(gi)*transpose(A2)*A3*hj; 
       
       % Revolute constraint, body 2 to 4
       r4 + A4*sC4 - r2 - A2*sC2;       
       %transpose(fi)*transpose(A2)*A4*hj;    % Redundant constraint
       %transpose(gi)*transpose(A2)*A4*hj;    % Redundant constraint
       % This parallel constraint is not necessary, because the
       % link is already cosntrained to be in the plane, by the
       % other revolute joints.
       
       % Revolute constraint, body 4 to 5
       r5 + A5*sD5 - r4 - A4*sD4;       
       transpose(fi)*transpose(A4)*A5*hj;
       transpose(gi)*transpose(A4)*A5*hj;
       
       r3 + A3*sE3 - r5 - A5*sE5;    
       %transpose(fi)*transpose(A5)*A3*hj;    % Redundant constraint
       transpose(gi)*transpose(A3)*A5*hj; 
       % Either of the dot1 constraints could have been removed
       
       % Distance driving constraints
       transpose(d01)*d01 - (s1)^2;   
       transpose(d12)*d12 - (s2)^2;   
       transpose(d24)*d24 - (s3)^2;  
       
       % Angular driving constraint on the base
       e20 - sin(t/2);
       
        % Constraints on the norms of the euler parameter vectors
       transpose(p0)*p0 - 1;
       transpose(p1)*p1 - 1;
       transpose(p2)*p2 - 1;
       transpose(p3)*p3 - 1;
       transpose(p4)*p4 - 1;
       transpose(p5)*p5 - 1];

 % Jacobian of PHI
PHIq = jacobian(PHI,q);

% Guess of the Initial Positions of each link
q_cabin  = [0; 0; 0; 1; 0; 0; 0];
q_boom   = [1.2; 3.2;   0; cos(pi/8); 0; 0; sin(pi/8)];
q_stick  = [  5; 3.3; 3.0; cos(pi/8); 0; 0; sin(pi/8)];
q_bucket = [  6;   4;   0; cos(pi/4); 0; 0; sin(pi/4)];
q_rod1   = [  6;   0;   0; cos(pi/4); 0; 0; sin(pi/4)];
q_rod2   = [  6;   0;   0;      0.94; 0; 0;     -0.32];
Tsub     = 0;

pos = [q_cabin', q_boom', q_stick', q_bucket', q_rod1', q_rod2', Tsub];     % Guess of initial position

%%  Newton-Raphson method, to find initial positions

% Position Vectors
T0 = [x0, y0, z0, e00, e10, e20, e30];
T1 = [x1, y1, z1, e01, e11, e21, e31];
T2 = [x2, y2, z2, e02, e12, e22, e32];
T3 = [x3, y3, z3, e03, e13, e23, e33];
T4 = [x4, y4, z4, e04, e14, e24, e34];
T5 = [x5, y5, z5, e05, e15, e25, e35];

% Unknown generalized coordinate vector
unknowns = [T0, T1, T2, T3, T4, T5, t];

error = 1;  % Placeholder value for error
ii = 0;     % Counter
while (error) > 1e-4
    q0    = double(subs(   q, unknowns, pos));  % Evaluated Position
    PHI0  = double(subs( PHI, unknowns, pos));  % Evaluated PHI
    PHIq0 = double(subs(PHIq, unknowns, pos));  % Evaluated Jacobian
 
    dq0 = (-PHIq0 \ PHI0)*0.5;  % Increment in position
    
    q0New = dq0 + q0;    % Updated Position
    error = norm(PHI0);  % Error
    
    q0 = q0New;
    pos = [q0New', Tsub];  % New Guess
    ii = ii + 1;
end
q0New;


%%  Kinematics Simulation

i = 1;          % Loop counter
tstart = 0;     % Start time
tmax = 1;       % End time
step = 0.1;     % Step size

% Preallocating pos, vel, and accel matrices
Position = zeros(42, tmax/step);
Velocity = zeros(42, tmax/step);
Acceleration = zeros(42, tmax/step);

vD = 2*[s1*diff(s1, t); s2*diff(s2, t); s3*diff(s3, t); 0.5*diff(s4, t)];

% Loops over position, velocity, and acceleration and solves the velocity and
% acceleration equations at each time step to produce their values as
% functions of time
for Tsub = tstart:step:tmax

    q0     = double(subs(q, unknowns, pos));      % Initial position
    PHI0   = double(subs(PHI, unknowns, pos));    % Initial PHI
    PHIq0  = double(subs(PHIq, unknowns, pos));   % Initial PHIq
    vDsubs = double(subs(vD, unknowns, pos));     % Initial VD
    
    % Updating Position
    dq0 = - PHIq0 \ PHI0;
    qNew = q0 + dq0;
    
    % Storing values in a matrix
    Position(:,i) = q0;
    
    % Subs. in Position
    r0subs = Position(1:3,i);
    r1subs = Position(8:10,i);
    r2subs = Position(15:17,i);
    r3subs = Position(22:24,i);
    r4subs = Position(29:31,i);
    r5subs = Position(36:38,i);
    
    % Subs. in G
    G0subs = double(subs(G0, unknowns, pos));
    G1subs = double(subs(G1, unknowns, pos));
    G2subs = double(subs(G2, unknowns, pos));
    G3subs = double(subs(G3, unknowns, pos));
    G4subs = double(subs(G4, unknowns, pos));
    G5subs = double(subs(G5, unknowns, pos));
    
    wg = zeros(3,1);
    
    % PHIt
    PHIt = [zeros(32,1); vDsubs; zeros(nb,1)];
    
    % Velocity, computed using the velocity equation
    Vel = - PHIq0 \ PHIt;
    
    % Storing values in a matrix
    Velocity(:,i) = Vel;
    
    % Subs. in Velocity
    dr0subs = Velocity(1:3,i);
    dp0subs = Velocity(4:7,i);
    dr1subs = Velocity(8:10,i);
    dp1subs = Velocity(11:14,i);
    dr2subs = Velocity(15:17,i);
    dp2subs = Velocity(18:21,i);
    dr3subs = Velocity(22:24,i);
    dp3subs = Velocity(25:28,i);
    dr4subs = Velocity(29:31,i);
    dp4subs = Velocity(32:35,i);
    dr5subs = Velocity(36:38,i);
    dp5subs = Velocity(39:42,i);
    
    % Rotation Matrices
    A0 = f_A( q0(4), q0(5),  q0(6), q0(7));
    A1 = f_A(q0(11), q0(12), q0(13), q0(14));
    A2 = f_A(q0(18), q0(19), q0(20), q0(21));
    A3 = f_A(q0(25), q0(26), q0(27), q0(28));
    A4 = f_A(q0(32), q0(33), q0(34), q0(35));
    A5 = f_A(q0(39), q0(40), q0(41), q0(42));

    % Angular Velocities
    w0subs = f_w(G0subs, dp0subs);
    w1subs = f_w(G1subs, dp1subs);
    w2subs = f_w(G2subs, dp2subs);
    w3subs = f_w(G3subs, dp3subs);
    w4subs = f_w(G4subs, dp4subs);
    w5subs = f_w(G5subs, dp5subs);
    
    % Elements of gammaK
    % Revolute 1
    gammaS_1   = f_gammaS(wg, sOg, Ag, w0subs, sO0 ,A0);
    gammaD1_11 = f_gammaD1(wg, fi, Ag, w0subs, gi, A0);
    gammaD1_12 = f_gammaD1(wg, hj, Ag, w0subs, gi, A0);
    
    % Revolute 2
    gammaS_2   = f_gammaS(w0subs, sF0, A0, w1subs, sF1, A1);
    gammaD1_21 = f_gammaD1(w0subs, fi, A0, w1subs, hj, A1);
    gammaD1_22 = f_gammaD1(w0subs, gi, A0, w1subs, hj, A1);
    
    % Revolute 3
    gammaS_3   = f_gammaS(w1subs, sA1, A1, w2subs, sA2, A2);
    gammaD1_31 = f_gammaD1(w1subs, fi, A0, w2subs, hj, A2);
    gammaD1_32 = f_gammaD1(w1subs, gi, A0, w2subs, hj, A2);
    
    % Revolute 4
    gammaS_4   = f_gammaS(w2subs, sB2, A2, w3subs, sB3, A3);
    gammaD1_41 = f_gammaD1(w2subs, fi, A2, w3subs, hj, A3);
    gammaD1_42 = f_gammaD1(w2subs, gi, A2, w3subs, hj, A3);
    
    % Revolute 5
    gammaS_5  = f_gammaS(w2subs, sC2, A2, w4subs, sC4, A4);
    % gammaD1_51 = f_gammaD1(w2subs, fi, A2, w4subs, hj, A4);  % Redundant
    % gammaD1_52 = f_gammaD1(w2subs, gi, A2, w3subs, hj, A4);  % Redundant
    
    % Revolute 6
    gammaS_6  = f_gammaS(w4subs, sD4, A4, w5subs, sD5, A5);
    gammaD1_61 = f_gammaD1(w4subs, fi, A4, w5subs, hj, A5);
    gammaD1_62 = f_gammaD1(w4subs, gi, A4, w5subs, hj, A5);
    
    % Revolute 7
    gammaS_7  = f_gammaS(w5subs, sE5, A5, w3subs, sE3, A3);
    % gammaD1_71 = f_gammaD1(w5subs, fi, A5, w3subs, hj, A3);  % Redundant
    gammaD1_72 = f_gammaD1(w5subs, gi, A5, w3subs, hj, A3);
    
    % Note: the redundant lines in Gamma correspond to those in PHI
    
    % gammaK
    gammaK = [gammaS_1; gammaD1_11; gammaD1_12; ...
              gammaS_2; gammaD1_21; gammaD1_22; ...
              gammaS_3; gammaD1_31; gammaD1_32; ...
              gammaS_4; gammaD1_41; gammaD1_42; ...
              gammaS_5; ...
              gammaS_6; gammaD1_61; gammaD1_62; ...
              gammaS_7; gammaD1_72];
  
    % Elements of gammaD
    gamma_SSd1 = f_gammaSSd(r0subs, dr0subs, w0subs, sP0, A0, r1subs, dr1subs, w1subs, sQ1, A1, s1, t);
    gamma_SSd2 = f_gammaSSd(r1subs, dr1subs, w1subs, sR1, A2, r2subs, dr2subs, w2subs, sS2, A2, s2, t);
    gamma_SSd3 = f_gammaSSd(r2subs, dr2subs, w2subs, sT2, A4, r4subs, dr4subs, w4subs, sD4, A4, s3, t);
    gamma_rot = (-1/4)*sin(Tsub/2);
    
    % gammaD
    gammaD = [gamma_SSd1; gamma_SSd2; gamma_SSd3; gamma_rot];
    
    % Elements of gammaP
    gammaP1 = f_gammaP(dp0subs);
    gammaP2 = f_gammaP(dp1subs);
    gammaP3 = f_gammaP(dp2subs);
    gammaP4 = f_gammaP(dp3subs);
    gammaP5 = f_gammaP(dp4subs);
    gammaP6 = f_gammaP(dp5subs);
    
    % gammaP
    gammaP = [gammaP1; gammaP2; gammaP3; gammaP4; gammaP5; gammaP6];

    % Solving the acceleration equation
    LHS = PHIq0;
    RHS = [gammaK; gammaD; gammaP];
    RHSsubs = double(subs(RHS, unknowns, pos));
    Acc = LHS \ RHSsubs;
    
    % Storing Accel values in a matrix
    Acceleration(:,i) = Acc;
    
    % Storing time in a vector
    T(i) = Tsub;
    
    if Tsub ~= 0
          q0 = qNew;
    else
          q0 = q0New;  
    end
    
    pos=[transpose(q0), Tsub];
    
    i = i + 1;
    
end

%%  Plots

% Position of Boom (Body 2)
figure(1)
plot(T, Position(15,:),'b')
hold on;
plot(T, Position(16,:),'r')
plot(T, Position(17,:),'g')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Position (m/s)', 'FontSize', 20);
title('Position vs Time', 'FontSize', 20);
legend('x', 'y', 'z');
set(gca, 'FontSize', 15);
grid on;

% Euler Params of Boom (Body 2)
figure(2)
plot(T, Position(18,:), 'b')
hold on;
plot(T, Position(19,:), 'r')
plot(T, Position(20,:), 'g')
plot(T, Position(21,:), 'black')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Position (m/s)', 'FontSize', 20);
title('Pos of Euler Params vs Time', 'FontSize', 20);
legend('e0', 'e1', 'e2', 'e3');
set(gca, 'FontSize', 15);
grid on;

% Velocity of Boom (Body 2)
figure(3)
plot(T, Velocity(15,:), 'b')
hold on;
plot(T, Velocity(16,:), 'r')
plot(T, Velocity(17,:), 'g')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Velocity (m/s)', 'FontSize', 20);
title('Velocity vs Time', 'FontSize', 20);
legend('x', 'y', 'z');
set(gca, 'FontSize', 15);
grid on;

% Velocity of Euler Params of Boom (Body 2)
figure(4)
plot(T, Velocity(18,:), 'b')
hold on;
plot(T, Velocity(19,:), 'r')
plot(T, Velocity(20,:), 'g')
plot(T, Velocity(21,:), 'black')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Velocity (m/s)', 'FontSize', 20);
title('Vel of Euler Params vs Time', 'FontSize', 20);
legend('e0', 'e1', 'e2', 'e3');
set(gca, 'FontSize', 15);
grid on;

% Accel of Boom (Body 2)
figure(5)
plot(T, Acceleration(15,:), 'b')
hold on;
plot(T, Acceleration(16,:), 'r')
plot(T, Acceleration(17,:), 'g')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Accel (m/s)', 'FontSize', 20);
title('Accel vs Time', 'FontSize', 20);
legend('x', 'y', 'z');
set(gca, 'FontSize', 15);
grid on;

% Accel of Euler Params of Boom (Body 2)
figure(6)
plot(T, Acceleration(18,:), 'b')
hold on;
plot(T, Acceleration(19,:), 'r')
plot(T, Acceleration(20,:), 'g')
plot(T, Acceleration(21,:), 'black')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Accel (m/s)', 'FontSize', 20);
title('Accel of Euler Params vs Time', 'FontSize', 20);
legend('e0', 'e1', 'e2', 'e3');
set(gca, 'FontSize', 15);
grid on;

%%%%%%%%%

% Position of Stick (Body 3)
figure(7)
plot(T, Position(22,:),'b')
hold on;
plot(T, Position(23,:),'r')
plot(T, Position(24,:),'g')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Position (m/s)', 'FontSize', 20);
title('Position vs Time', 'FontSize', 20);
legend('x', 'y', 'z');
set(gca, 'FontSize', 15);
grid on;

% Euler Params of Stick (Body 3)
figure(8)
plot(T, Position(25,:), 'b')
hold on;
plot(T, Position(26,:), 'r')
plot(T, Position(27,:), 'g')
plot(T, Position(28,:), 'black')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Position (m/s)', 'FontSize', 20);
title('Pos of Euler Params vs Time', 'FontSize', 20);
legend('e0', 'e1', 'e2', 'e3');
set(gca, 'FontSize', 15);
grid on;

% Velocity of Stick (Body 3)
figure(9)
plot(T, Velocity(22,:), 'b')
hold on;
plot(T, Velocity(23,:), 'r')
plot(T, Velocity(24,:), 'g')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Velocity (m/s)', 'FontSize', 20);
title('Velocity vs Time', 'FontSize', 20);
legend('x', 'y', 'z');
set(gca, 'FontSize', 15);
grid on;

% Velocity of Euler Params of Stick (Body 3)
figure(10)
plot(T, Velocity(25,:), 'b')
hold on;
plot(T, Velocity(26,:), 'r')
plot(T, Velocity(27,:), 'g')
plot(T, Velocity(28,:), 'black')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Velocity (m/s)', 'FontSize', 20);
title('Vel of Euler Params vs Time', 'FontSize', 20);
legend('e0', 'e1', 'e2', 'e3');
set(gca, 'FontSize', 15);
grid on;

% Acceleration of Stick (Body 3)
figure(11)
plot(T, Acceleration(22,:), 'b')
hold on;
plot(T, Acceleration(23,:), 'r')
plot(T, Acceleration(24,:), 'g')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Accel (m/s)', 'FontSize', 20);
title('Accel vs Time', 'FontSize', 20);
legend('x', 'y', 'z');
set(gca, 'FontSize', 15);
grid on;

% Accel of Euler Params of Stick (Body 3)
figure(12)
plot(T, Acceleration(25,:), 'b')
hold on;
plot(T, Acceleration(26,:), 'r')
plot(T, Acceleration(27,:), 'g')
plot(T, Acceleration(28,:), 'black')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Accel (m/s)', 'FontSize', 20);
title('Accel of Euler Params vs Time', 'FontSize', 20);
legend('e0', 'e1', 'e2', 'e3');
set(gca, 'FontSize', 15);
grid on;

%%%%%%%%%

% Position of Bucket (Body 4)
figure(13)
plot(T, Position(29,:),'b')
hold on;
plot(T, Position(30,:),'r')
plot(T, Position(31,:),'g')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Position (m/s)', 'FontSize', 20);
title('Position vs Time', 'FontSize', 20);
legend('x', 'y', 'z');
set(gca, 'FontSize', 15);
grid on;

% Euler Params of Bucket (Body 4)
figure(14)
plot(T, Position(32,:), 'b')
hold on;
plot(T, Position(33,:), 'r')
plot(T, Position(34,:), 'g')
plot(T, Position(35,:), 'black')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Position (m/s)', 'FontSize', 20);
title('Pos of Euler Params vs Time', 'FontSize', 20);
legend('e0', 'e1', 'e2', 'e3');
set(gca, 'FontSize', 15);
grid on;

% Velocity of Bucket (Body 4)
figure(15)
plot(T, Velocity(29,:), 'b')
hold on;
plot(T, Velocity(30,:), 'r')
plot(T, Velocity(31,:), 'g')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Velocity (m/s)', 'FontSize', 20);
title('Velocity vs Time', 'FontSize', 20);
legend('x', 'y', 'z');
set(gca, 'FontSize', 15);
grid on;

% Velocity of Euler Params of Bucket (Body 4)
figure(16)
plot(T, Velocity(32,:), 'b')
hold on;
plot(T, Velocity(33,:), 'r')
plot(T, Velocity(34,:), 'g')
plot(T, Velocity(35,:), 'black')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Velocity (m/s)', 'FontSize', 20);
title('Vel of Euler Params vs Time', 'FontSize', 20);
legend('e0', 'e1', 'e2', 'e3');
set(gca, 'FontSize', 15);
grid on;

% Accel of Bucket (Body 4)
figure(17)
plot(T, Acceleration(29,:), 'b')
hold on;
plot(T, Acceleration(30,:), 'r')
plot(T, Acceleration(31,:), 'g')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Accel (m/s)', 'FontSize', 20);
title('Accel vs Time', 'FontSize', 20);
legend('x', 'y', 'z');
set(gca, 'FontSize', 15);
grid on;

% Accel of Euler Params of Bucket (Body 4)
figure(18)
plot(T, Acceleration(32,:), 'b')
hold on;
plot(T, Acceleration(33,:), 'r')
plot(T, Acceleration(34,:), 'g')
plot(T, Acceleration(35,:), 'black')
xlabel('Time (s)', 'FontSize', 20);
ylabel('Accel (m/s)', 'FontSize', 20);
title('Accel of Euler Params vs Time', 'FontSize', 20);
legend('e0', 'e1', 'e2', 'e3');
set(gca, 'FontSize', 15);
grid on;