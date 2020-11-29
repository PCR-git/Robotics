% Returns a single set of revolute constraints

function [phi] = f_Revolute(Y,body1,sA1,body2,sA2,n,m)

[~,~,q]=f_StateVar(Y,n,m);

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
phi=rA1-rA2;
end