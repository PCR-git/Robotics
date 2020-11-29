% Set Global Error

function setGlobalError(i,Errori)

error1 = Errori(1);
error2 = Errori(2);

global Error;

Error{i,1} = error1;
Error{i,2} = error2;

end