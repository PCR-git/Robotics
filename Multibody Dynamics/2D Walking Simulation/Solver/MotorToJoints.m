 % Transforms the torque applied by the motors
% to the torque experienced at each joint

function Torques = MotorToJoints(~,MotorTorqueA,MotorTorqueB,Radii)

% Radii of the pulley sections
r1 = Radii(1); r2 = Radii(2); r3 = Radii(3);
% Radius of the rotator links
LinkRadius = Radii(4);
% The pulley radii are the same for all modules
r5 = r1; r6 = r2; r7 = r3;

% ------------

% fA1 = MotorTorqueA*r1/3;
% fA2 = MotorTorqueA*r2/3;
% fA3 = MotorTorqueA*r3/3;
% fA1 = MotorTorqueA/r1;
% fA2 = MotorTorqueA/r2;
% fA3 = MotorTorqueA/r3;

% torqueA1 = MotorTorqueA*(LinkRadius/r1);
% torqueA2 = MotorTorqueA*(LinkRadius/r2);
% torqueA3 = MotorTorqueA*(LinkRadius/r3);

torqueA1 = 3*MotorTorqueA*(LinkRadius/r1);
torqueA2 = 2*MotorTorqueA*(LinkRadius/r2);
torqueA3 = 1*MotorTorqueA*(LinkRadius/r3);

% ----

% fB1 = MotorTorqueB/r5;
% fB2 = MotorTorqueB/r6;
% fB3 = MotorTorqueB/r7;

% torqueB1 = MotorTorqueB*(LinkRadius/r1);
% torqueB2 = MotorTorqueB*(LinkRadius/r2);
% torqueB3 = MotorTorqueB*(LinkRadius/r3);

torqueB1 = 3*MotorTorqueB*(LinkRadius/r1);
torqueB2 = 2*MotorTorqueB*(LinkRadius/r2);
torqueB3 = 1*MotorTorqueB*(LinkRadius/r3);

% Vector of all joint torques
Torques = [torqueA3;torqueA2;torqueA1;0;torqueB1;torqueB2;torqueB3];
%Torques = [torqueA1;torqueA1;torqueA1;0;torqueB3;torqueB2;torqueB1];
%Torques = [torqueA3+torqueA2+torqueA1;torqueA2+torqueA1;torqueA1;0;torqueB3+torqueB2+torqueB1;torqueB2+torqueB1;torqueB1];

end
