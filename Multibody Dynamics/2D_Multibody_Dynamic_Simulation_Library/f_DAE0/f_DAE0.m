% Solves for the derivative of the state vector

function [Ypn] = f_DAE0(t,Yn,M,PHIq,Gamma,Qa,n,m,s)

if t>0
[M,PHIq,Gamma,Qa]=BuildMechanism(Yn,n,m,M,s);
end

qp=Yn(1:n); % velocity
%q=Yn((n+m+1):end); %position

% Zero elements of the LHS matrix
Zr13=zeros(n,n);
Zr22=zeros(m,m);
Zr23=zeros(m,n);
Zr31=zeros(n,n);
Zr32=zeros(n,m);
Id=eye(n);

% Builds and solves the Matrix DEA for the system
LHS=[  M  , transpose(PHIq),  Zr13;...
      PHIq,     Zr22,         Zr23;...
      Zr31,     Zr32          Id];

RHS=[Qa; Gamma ; qp];

Ypn=LHS\RHS;
      
      
end

