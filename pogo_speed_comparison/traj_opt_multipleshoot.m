function traj_opt_multipleshoot(nNodesF1,nNodesS,nNodesF2,pogo)
    nNodes=nNodesF1+nNodesS+nNodesF2;
    %% Get initial guess
    Guess=getGuess(nNodesF1,nNodesS,nNodesF2,pogo);
    Problem.x0=Guess;
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
    %%
    %no linear constraints, all handled in the nonlincon function
    Problem.Aineq = [];%tempA;%Aineq x<=bineq
    Problem.bineq = [];%tempb;
    
    Problem.Aeq = [];%Aeq x=beq
    Problem.beq = [];  
    %% Optimization options
    options = optimset(...
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
    
    Problem.options=options;
    
    
    global auxdata
    auxdata.nNodesF1=nNodesF1;
    auxdata.nNodesS=nNodesS;
    auxdata.nNodesF2=nNodesF2;
    auxdata.q_0=q0;
    auxdata.dq_0=dq0;
    auxdata.lmin=pogo.parameters.lmin;
    auxdata.lmax=pogo.parameters.lmax;
    auxdata.pogo=pogo;
    
    %% fmincon
    Problem.solver = 'fmincon';
    Problem.objective = @(DecVar)pogo_objective(DecVar,auxdata);
    Problem.nonlcon = @(DecVar)pogo_nonlcon(DecVar,auxdata);
    
    tic 
    [MinSoln,FVAL,EXITFLAG,OUTPUT,LAMBDA,GRAD,HESSIAN] = fmincon(Problem);
    toc
    
    path='results/';
    name='fmincon_';
    opt_num=0;
    numText=sprintf('%.5d',opt_num);
    while(exist([path name numText '.mat'],'file')==2)
        opt_num = opt_num+1;
        numText = sprintf('%.5d',opt_num);
    end
    filename = [path name numText];

    save(filename,'MinSoln','FVAL','EXITFLAG','OUTPUT','LAMBDA','GRAD','HESSIAN'); % save all the data
    %% ADIGATOR
    Problem.solver = 'fmincon';
    options=optimset(options,'GradObj','on','GradConstr','on');%,'Hessian','user-supplied','HessFcn',@(x,lambda)pogo_objective_Hes(x,auxdata,lambda));
    Problem.options=options;
    
    Problem.objective = @(x)pogo_objective_Grd(x,auxdata);
    Problem.nonlcon = @(x)pogo_nonlcon_Grd(x,auxdata);
    tic
    [MinSoln,FVAL,EXITFLAG,OUTPUT,LAMBDA,GRAD,HESSIAN] = fmincon(Problem);
    toc
    
    path='results/';
    name='adigator_';
    opt_num=0;
    numText=sprintf('%.5d',opt_num);
    while(exist([path name numText '.mat'],'file')==2)
        opt_num = opt_num+1;
        numText = sprintf('%.5d',opt_num);
    end
    filename = [path name numText];

    save(filename,'MinSoln','FVAL','EXITFLAG','OUTPUT','LAMBDA','GRAD','HESSIAN'); % save all the data
    %% SOLVE PROBLEM SNOPT
    Problem.obj=@objective;
    Problem.con=@constraint;
    
    snscreen on;
    snprint('traj_opt_multipleshoot.out');  % By default, screen output is off;
%    sn.spc = which('sn.spc');
%    snspec (sn.spc);
    snseti ('Major Iteration limit', 250);
    
    tic
    [x,fval,exitflag,lambda,states] = snsolve( Problem.obj, Problem.x0, Problem.Aineq, Problem.bineq, Problem.Aeq, Problem.beq, Problem.lb', Problem.ub', Problem.con);
    toc
    
    snprint off;
    snend;
    path='results/';
    name='snopt_';
    opt_num=0;
    numText=sprintf('%.5d',opt_num);
    while(exist([path name numText '.mat'],'file')==2)
        opt_num = opt_num+1;
        numText = sprintf('%.5d',opt_num);
    end
    filename = [path name numText];

    save(filename,'x','fval','exitflag','lambda','states'); % save all the data
end