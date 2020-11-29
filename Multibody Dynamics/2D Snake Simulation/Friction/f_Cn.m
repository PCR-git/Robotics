function Ffn = f_Cn(Fr,rlp,NFP,M,g,bodyn)

    bf = Fr(1);
    ff = Fr(2);
    sf = Fr(3);

    xlpn = rlp(1+(bodyn-1)*2);
    ylpn = rlp(2+(bodyn-1)*2);
    
    staticthreshold = NFP(1);
    staticratio = NFP(2);
    coulombratio = NFP(3);

    % Mass definitions
    mn = M(bodyn*3,bodyn*3);
    
    % X Force on Link 1
    if abs(xlpn) <= staticthreshold  % Static
        Fx1 = -staticratio*sf*xlpn/staticthreshold;
    else  % Constant
        Fx1 = -coulombratio*sf*sign(xlpn);
    end

    % Y Force on Link 1
    if ylpn >=0 && ylpn <= staticthreshold  % Static, forward
        Fy1 = -staticratio*ff*ylpn/staticthreshold;
    elseif ylpn < 0 && ylpn >= -staticthreshold  % Static, back
        Fy1 = -staticratio*bf*ylpn/staticthreshold;
    elseif ylpn > staticthreshold  % Constant, forward
        Fy1 = -coulombratio*ff*sign(ylpn);
    elseif ylpn < -staticthreshold  % Constant, back
        Fy1 = -coulombratio*bf*sign(ylpn);
    end
    
    % Frictional force on each link
    Ffn = [Fx1;Fy1]*mn*g;
    
end