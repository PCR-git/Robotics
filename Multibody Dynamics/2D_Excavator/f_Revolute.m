
function [phi] = f_Revolute(body1,sA1,body2,sA2,q)

if body1~=0 && body2~=0
    [rA1]=f_r(q,body1,sA1);
    [rA2]=f_r(q,body2,sA2);
else
    if body1==0
        [rA2]=f_r(q,body2,sA2);
        [rA1]=[0 0]';
    else
        error('wrong configuration')
    end
end
phi=rA2-rA1;
end