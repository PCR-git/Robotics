function A = f_AMat(p, body)

if body == 0
    A = eye(3, 3);
else
    E = f_EMat(p, body);
    G = f_GMat(p, body);
    %A = E*(G.');
    
    e = f_e(p, body);
    
    e0 = e(1,1);
    e1 = e(2,1);
    e2 = e(3,1);
    e3 = e(4,1);
    
    % A Matrix, computed using Mathematica
    %A = [e0^2+e1^2-e2^2-e3^2    2*(e1*e2-e0*e3)        2*(e1*e3+e0*e2);...
    % 2*(e1*e2+e0*e3)        e0^2-e1^2+e2^2-e3^3    2*(e2*e3-e0*e1);...
    % 2*(e1*e3-e0*e2)        2*(e2*e3+e0*e1)        e0^2-e1^2-e2^2+e3^2];
 
    A = 2*[e0^2 + e1^2 - 0.5,   e1*e2-e0*e3,       e1*e3 + e0*e2;...
          e2*e1 + e0*e3,        e0^2 + e2^2 - 0.5,  e2*e3 + -e0*e1;...
          e3*e1-e0*e2,         e3*e2 + e0*e1,      e0^2+e3^2-0.5];
    
end
end

