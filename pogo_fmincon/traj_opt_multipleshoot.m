function [MinSoln,Problem,Guess,FVAL,LAMBDA,GRAD,HESSIAN,EXITFLAG,OUTPUT]=traj_opt_multipleshoot(nNodesF1,nNodesS,nNodesF2,pogo)
    nNodes=nNodesF1+nNodesS+nNodesF2;
    %% Get initial guess
    Guess=getGuess(nNodesF1,nNodesS,nNodesF2,pogo);
    %% Set up optimization problem
    Problem.solver = 'fmincon';
    Problem.x0 = Guess;
    %% Bounds
    
    %time bounds
    LB_t = [0.01 0.01 0.01];
    UB_t = [3 2 3];
    
    %stance lower bounds
    LBS_q = [pogo.parameters.lmin];
    LBS_dq = -50;
    LBS_u = -4000;
    
    %stance upper bounds
    UBS_q = [pogo.parameters.lmax];
    UBS_dq = 50;
    UBS_u = 4000;
    
    %flight lower bounds
    LBF_q = [pogo.parameters.lmin pogo.parameters.lmin];
    LBF_dq = [-50 -50];
    LBF_u = -4000;
    
    %flight upper bounds
    UBF_q = [2*pogo.parameters.init_q(1) pogo.parameters.lmax];
    UBF_dq = [50 50];
    UBF_u = 4000;
    %% remap bounds for the problem (must match the guess order)
    Problem.lb=[LB_t,repmat([LBF_q,LBF_dq,LBF_u],1,nNodesF1),repmat([LBS_q,LBS_dq,LBS_u],1,nNodesS),repmat([LBF_q,LBF_dq,LBF_u],1,nNodesF2)];
    Problem.ub=[UB_t,repmat([UBF_q,UBF_dq,UBF_u],1,nNodesF1),repmat([UBS_q,UBS_dq,UBS_u],1,nNodesS),repmat([UBF_q,UBF_dq,UBF_u],1,nNodesF2)];
    %% Linear constraints 
% %     tempA=zeros(nNodes+nNodesS,length(Guess));%constraints for stance phase are handled in the lb and ub
% %     tempb=zeros(nNodes+nNodesS,1);
% %     
% %     k=0;
% %     for i=0:1:nNodesF1-1
% %         tempA(i+1,k+4)=-1;
% %         tempA(i+1,k+5)=1;%-z+l<0
% %         tempb(i+1)=0;
% %         k=k+5;
% %     end  
% %     temp=k;
% %     for i=nNodesF1:1:nNodesF1+nNodesS-1
% %         tempA(i+1,k+4)=-1;
% %         tempb(i+1)=-pogo.parameters.lmin;%-z<-l
% %         k=k+3;
% %     end
% %     k=temp;
% %     for i=nNodesF1+nNodesS:1:nNodesF1+nNodesS+nNodesS-1
% %         tempA(i+1,k+4)=1;
% %         tempb(i+1)=pogo.parameters.lmax;%z<lmax
% %         k=k+3;
% %     end
% %     for i=nNodesF1+nNodesS+nNodesS:1:nNodesF1+nNodesS+nNodesS+nNodesF2-1
% %         tempA(i+1,k+4)=-1;
% %         tempA(i+1,k+5)=1;%-z+l<0
% %         tempb(i+1)=0;
% %         k=k+5;
% %     end
    
    %no linear constraints, all handled in the nonlincon function
    Problem.Aineq = [];%tempA;%Aineq x<=bineq
    Problem.bineq = [];%tempb;
    
    Problem.Aeq = [];%Aeq x=beq
    Problem.beq = [];  
    %% Optimization options
    Problem.options = optimset(...
        'Display','iter',...
        'MaxFunEvals',15000,...
        'MaxIter',7000,...
        'Algorithm','interior-point',...
        'TolCon',1e-3,...
        'TolFun',1e-4,...
        'TolX',1e-4,...
        'FinDiffRelStep',5e-4,...
        'diffminchange',5e-4);
    %% user-defined functions
    %set the initial conditions
    q0=pogo.parameters.init_q;
    dq0=pogo.parameters.init_dq;
    Problem.objective = @(DecVar)pogo_objective(DecVar,nNodesF1,nNodesS,nNodesF2);
    Problem.nonlcon = @(DecVar)pogo_nonlcon(DecVar,nNodesF1,nNodesS,nNodesF2,q0,dq0,pogo.parameters.lmin,pogo.parameters.lmax);
    %% SOLVE PROBLEM
    [MinSoln,FVAL,EXITFLAG,OUTPUT,LAMBDA,GRAD,HESSIAN] = fmincon(Problem);
    %% check the exit flag:
    if (EXITFLAG==1)
        disp('First order optimality conditions satisfied');
    elseif (EXITFLAG==0)
        disp('Too many function evaluations or iterations')
    elseif (EXITFLAG==-1)
        disp('Stopped by output or plot function');
    elseif (EXITFLAG==2)
        disp('Change in X too small')
    elseif (EXITFLAG==3)
        disp('Change in objective function too small');
    elseif (EXITFLAG==4)
        disp('Computed search direction too small');
    elseif (EXITFLAG==5)
        disp('Predicted change in objective function too small');
    elseif (EXITFLAG==-3)
        disp('Problem seems unbounded');
    end
    
    %% display optimisation info:
    OUTPUT 
end