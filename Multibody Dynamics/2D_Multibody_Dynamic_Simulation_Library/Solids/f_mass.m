% Writes the mass matrix for a system

function Mass = f_mass(body,mass,Inertia,M)

Mass=M;
Mass(3*(body-1)+1,3*(body-1)+1)=mass;
Mass(3*(body-1)+2,3*(body-1)+2)=mass;
Mass(3*(body-1)+3,3*(body-1)+3)=Inertia;

end