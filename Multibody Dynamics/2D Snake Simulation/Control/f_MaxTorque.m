% Max Torque
% Limits the control inputs to stay beneath a torque limit

function un = f_MaxTorque(MaxTorque,Ui,nn)

un = Ui(nn);

if un > MaxTorque
    un = MaxTorque;
elseif un < -MaxTorque
    un = -MaxTorque;
end

end