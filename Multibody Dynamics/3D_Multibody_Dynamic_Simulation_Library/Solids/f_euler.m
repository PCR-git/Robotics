function euler = f_euler(u, chi)

e0 = cos(chi/2);
e1 = u(1)*sin(chi*0.5);
e2 = u(2)*sin(chi*0.5);
e3 = u(3)*sin(chi*0.5);

euler = [e0; e1; e2; e3];

end