
function [r] = f_cg(q , body)
x=q(1+3*(body-1));
y=q(2+3*(body-1));
r=[x;y];
end