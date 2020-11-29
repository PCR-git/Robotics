% Max Torque
% Limits the control inputs to stay beneath a torque limit

function MotorTorque = f_MaxTorque(MaxTorque,MotorTorque)

if MotorTorque > MaxTorque
    MotorTorque = MaxTorque;
elseif MotorTorque < -MaxTorque
    MotorTorque = -MaxTorque;
end

end