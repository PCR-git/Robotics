function Ffn = f_VCSn(Fr,rlp,NFP,M,g,bodyn)

    bf = Fr(1);
    ff = Fr(2);
    sf = Fr(3);

    xlpn = rlp(1+(bodyn-1)*2);
    ylpn = rlp(2+(bodyn-1)*2);
    
    staticthreshold = NFP(1);
    staticratio = NFP(2);
    stribeckratio = NFP(3);
    viscousslope = NFP(4);
    backthreshold = NFP(5);
    forwardthreshold = NFP(6);
    sidethreshold = NFP(7);

    % Mass definitions
    mn = M(bodyn*3,bodyn*3);
    
    % X Force on Link 1
    if abs(xlpn) <= staticthreshold  % Static
        Fx1 = -staticratio*sf*xlpn/staticthreshold;
    elseif abs(xlpn) > staticthreshold && abs(xlpn) <= sidethreshold  % Stribeck
        Fx1 = -staticratio*sf*sign(xlpn)+(stribeckratio/(sidethreshold-staticthreshold))*sf*(abs(xlpn)-sidethreshold)*sign(xlpn);
    else  % Viscous
        Fx1 = -staticratio*sf*sign(xlpn)+stribeckratio*sf*sign(xlpn) - viscousslope*sf*(abs(xlpn)-sidethreshold)*sign(xlpn);
    end

    % Y Force on Link 1
    if ylpn >= 0 && ylpn <= staticthreshold  % Static, forward
        Fy1 = -staticratio*ff*ylpn/staticthreshold;
    elseif ylpn < 0 && ylpn >= -staticthreshold  % Static, back
        Fy1 = -staticratio*bf*ylpn/staticthreshold;
    elseif ylpn > staticthreshold && ylpn < forwardthreshold  % Stribeck, forward
        Fy1 = -staticratio*ff*sign(ylpn)+(stribeckratio/(forwardthreshold-staticthreshold))*ff*(ylpn-staticthreshold);
    elseif ylpn < -staticthreshold && ylpn > -backthreshold  % Stribeck, back
        Fy1 = -staticratio*bf*sign(ylpn)+(stribeckratio/(backthreshold-staticthreshold))*bf*(ylpn+staticthreshold);    
    elseif ylpn >= forwardthreshold  % Viscous, forward
        Fy1 = -staticratio*ff*sign(ylpn)+stribeckratio*ff*sign(ylpn) - viscousslope*ff*(ylpn-forwardthreshold);
    elseif ylpn <= -backthreshold  % Viscous, back
        Fy1 = -staticratio*bf*sign(ylpn)+stribeckratio*bf*sign(ylpn) - viscousslope*bf*(ylpn+backthreshold);
    end

    % Frictional force on each link
    Ffn = [Fx1;Fy1]*mn*g;
    
end