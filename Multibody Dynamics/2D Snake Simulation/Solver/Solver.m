
% Solver
function [Tx,Yx,Tex,Yex,PHI_Tx,qppx] = Solver(t_f,Te,Yi,M,LVec,n,m,s,g,time2steps,Control,Fr,PF,rwi,rwj,Ck,Disturb,Noise,WPtol,Go,WPang,WPNum)

    % Prints time to Way Pt n
    if WPNum > 0
        TimeWn = sprintf('Time to Way Pt %s: %s seconds',num2str(WPNum),num2str(Te));
        disp(TimeWn);
    end
    
    % Build Mechanism. Constructs the Jacobian, Gamma, and the global force vector, Qa
    [M,PHIq,Gamma] = BuildMechanism(Yi,n,m,M,s);

    % f_DAE0. Builds and solves the algebraic-differential equation to give the
    % derivative of the state-vector for any given time.
    f_DAE = @(t,Yn)f_DAE0(t,Yn,M,LVec,PHIq,Gamma,n,m,s,g,time2steps,Control,Fr,PF,rwi,rwj,Ck,Disturb,Noise,Go,WPang,WPNum);

    % Evalconstraints. Evaluates the norm of the constraint vector (to monitor accuracy of simulation).
    f_Evalconstraints = @(t,Yn)Evalconstraints(Yn,n,m,s);

    % NumericalAnalysis. Solves for position, velocity, and acceleration for each unit of time.
    [Tx,Yx,Tex,Yex,PHI_Tx,qppx] = NumericalAnalysis(f_DAE,f_Evalconstraints,Yi,t_f,n,m,rwj,WPtol,Go);
    
end
