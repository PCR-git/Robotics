% Determines the torque which the motor should supply,
% given desired torques at each joint.

function [MotorTorqueA, MotorTorqueB] = MotorTorque(Uvec,Radii)

% Control Signal Definitions
torqueA1 = Uvec(1);
torqueA2 = Uvec(2);
torqueA3 = Uvec(3);

torqueB1 = Uvec(5);
torqueB2 = Uvec(6);
torqueB3 = Uvec(7);

% --------

% Radii of the pulley sections
r1 = Radii(1); r2 = Radii(2); r3 = Radii(3);
% Radius of rotator links
LinkRadius = Radii(4);
% The pulley radii are the same for all modules
r5 = r1; r6 = r2; r7 = r3;
% The radii of the rotator links are all equal
R1 = LinkRadius; R2 = LinkRadius; R3 = LinkRadius;
R5 = LinkRadius; R6 = LinkRadius; R7 = LinkRadius;

% -------------------
% % Torques for the 1st module
% torqueP3A = (r3/R3)*(torque3A);
% torqueP2A = (r2/R2)*(torque2A+torque3A);
% torqueP1A = (r1/R1)*(torque1A+torque2A+torque3A);
% TorqueM1 = torqueP1A + torqueP2A + torqueP3A;
% 
% % Torques for the 2nd module
% torqueP3B = (r7/R7)*(torque3B);
% torqueP2B = (r6/R6)*(torque2B+torque3B);
% torqueP1B = (r5/R5)*(torque1B+torque2B+torque3B);
% TorqueM2 = torqueP1B + torqueP2B + torqueP3B;
% -------------------

OnetoTwo   = (R2/R1)*(r1*(r3+1))/(r1+r1*r3+r2*r3);
%OnetoThree = (R3/R1)*(r1)/(r1+r1*r3+r2*r3);
%TwotoThree = (OnetoTwo^-1)*OnetoThree;

FivetoSix  = (R6/R5)*(r5*(r7+1))/(r5+r5*r7+r6*r7);

tensionA1 = torqueA1/R1 - torqueA1*(OnetoTwo)/R2;
tensionA2 = torqueA2*(OnetoTwo^-1)/R1 - torqueA2/R2;
tensionA3 = torqueA3/R3;
MotorTorqueA = (3*r1*tensionA1 + 3*r2*tensionA2 + 3*r3*tensionA3)/3;

tensionB1 = torqueB1/R5 - torqueB1*(FivetoSix)/R6;
tensionB2 = torqueB2*(FivetoSix^-1)/R5 - torqueB2/R6;
tensionB3 = torqueB3/R7;
MotorTorqueB = (3*r5*tensionB1 + 3*r6*tensionB2 + 3*r7*tensionB3)/3;

end
