function J = f_J(body, ib, Ji)

J = Ji;                                         
J(1 + 3*(body - 1), 1 + 3*(body - 1)) = ib(1);     
J(2 + 3*(body - 1), 2 + 3*(body - 1)) = ib(2);     
J(3 + 3*(body - 1), 3 + 3*(body - 1)) = ib(3);   

end
