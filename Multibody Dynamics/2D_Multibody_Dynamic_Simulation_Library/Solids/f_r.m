% Returns the global coordinates of a constrained point
function rA = f_r(q,body,sA)
if body==0
     [rA]=[0;0];
else
rA= f_cg(q,body)+f_RM(f_angle(q,body))*sA;
end

end