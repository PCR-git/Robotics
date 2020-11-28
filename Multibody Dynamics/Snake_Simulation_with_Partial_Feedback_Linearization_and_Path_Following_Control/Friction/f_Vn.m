function Ffn = f_Vn(Fr,rlp,Cv,bodyn)

    % Coefficients of Friction
    bf = Fr(1);
    ff = Fr(2);
    sf = Fr(3);

    % X and Y Velocities (local)
    xlpn = rlp(1+(bodyn-1)*2);
    ylpn = rlp(2+(bodyn-1)*2);
    
    % Force on link 1
    Fx1 = -sf*xlpn;
    if ylpn <= 0
        Fy1 = -bf*ylpn;
    else
        Fy1 = -ff*ylpn;
    end

    % Frictional force on each link
    Ffn = Cv*[Fx1;Fy1];

end