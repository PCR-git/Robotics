% Picks out the x and y components from the q vector for each body

function [r] = f_cg(q , body)
x=q(1+3*(body-1));
y=q(2+3*(body-1));
r=[x;y];
end