% Set Global Torque

function setGlobalTorque(i,Torquei)

global TorqueVec;

MotorTorque1 = Torquei(1);
MotorTorque2 = Torquei(2);

TorqueVec{i,1} = MotorTorque1;
TorqueVec{i,2} = MotorTorque2;

end
