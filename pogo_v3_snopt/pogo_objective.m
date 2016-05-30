function cost = pogo_objective(DecVar,auxdata)
nNodesF1=auxdata.nNodesF1;
nNodesS=auxdata.nNodesS;
nNodesF2=auxdata.nNodesF2;
%minimize applied force
%% reshape the DecVar to find the applied force
i=nNodesF1*5+3;
traj1=reshape(DecVar(4:i),[5,nNodesF1]);
j=i+nNodesS*3;
traj2=reshape(DecVar(i+1:j),[3,nNodesS]);
traj3=reshape(DecVar(j+1:end),[5,nNodesF2]);
%% calculate the time for each node
dtF1=DecVar(1)/nNodesF1;
dtS=DecVar(2)/nNodesS;
dtF2=DecVar(3)/nNodesF2;
%% calculate the cost function
cost=sum(traj1(5,:).^2)*dtF1+sum(traj2(3,:).^2)*dtS+sum(traj3(5,:).^2)*dtF2;
end
