% Constructs the constraint vector (each row corresponds to a constraint)

function [PHI] = Evalconstraints(Y,n,m,s)

body1 = 1;
body2 = 2;
body3 = 3;
body4 = 4;
body5 = 5;
body6 = 6;
body7 = 7;
body8 = 8;

sA1 = s(:,1);
sA2 = s(:,2);
sB2 = s(:,3);
sB3 = s(:,4);
sC3 = s(:,5);
sC4 = s(:,6);
sD4 = s(:,7);
sD5 = s(:,8);
sE5 = s(:,9);
sE6 = s(:,10);
sF6 = s(:,11);
sF1 = s(:,12);
sB7 = s(:,13);
%sG7 = s(:,14);
sE8 = s(:,15);
%sG8 = s(:,16);

% Constraint vector
PHI= [f_Revolute(Y,body1,sA1,body2,sA2,n,m);...
      f_Revolute(Y,body2,sB2,body3,sB3,n,m);...
      f_Revolute(Y,body3,sC3,body4,sC4,n,m);...
      f_Revolute(Y,body4,sD4,body5,sD5,n,m);...
      f_Revolute(Y,body5,sE5,body6,sE6,n,m);...
      f_Revolute(Y,body6,sF6,body1,sF1,n,m);...
      f_Revolute(Y,body2,sB2,body7,sB7,n,m);...
      f_Revolute(Y,body6,sE6,body8,sE8,n,m)];

end

