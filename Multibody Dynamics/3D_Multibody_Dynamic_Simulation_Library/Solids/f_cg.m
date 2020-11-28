function cg = f_cg(r , body)

x = r(1 + 3*(body - 1));  
y = r(2 + 3*(body - 1));        
z = r(3 + 3*(body - 1));       

cg = [x; y; z];

end
