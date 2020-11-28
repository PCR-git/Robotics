function M = f_M(body, mb, Mi)

M = Mi;                                         
M(1 + 3*(body - 1), 1 + 3*(body - 1)) = mb;     
M(2 + 3*(body - 1), 2 + 3*(body - 1)) = mb;
M(3 + 3*(body - 1), 3 + 3*(body - 1)) = mb;    

end
