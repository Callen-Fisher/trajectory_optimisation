close all 
%% get the required parameters from the optimal solution
time_F1=x(1);
time_S=x(2);
time_F2=x(3);

traj_sim=[];
i=nNodesF1*5+3;
traj1=reshape(x(4:i),[5,nNodesF1]);
j=i+nNodesS*3;
traj2=reshape(x(i+1:j),[3,nNodesS]);
traj3=reshape(x(j+1:end),[5,nNodesF2]);
%% re run the simulation
%% FLIGHT 1
Tsample = x(1)/nNodesF1;
for i = 0:1:nNodesF1-1
    % get the start nodes
    q0 = traj1(1:2,i+1)';
    dq0 = traj1(3:4,i+1)';
    u = traj1(5,i+1);
    %run the model 
    odefun  = @(t,q)[q(3);q(4);get_ddq(q,u)];
    y0=[q0,dq0]';
    tspan=linspace(0,Tsample,100);
    temp=ode4(odefun,tspan,y0);
    % save the trajectory
    traj_sim=[traj_sim,temp'];
end
%% STANCE
Tsample = x(2)/nNodesS;
traj_S=[];
for i = 0:1:nNodesS-1
    %get the start nodes
    q0 = traj2(1,i+1);
    dq0 = traj2(2,i+1);
    u = traj2(3,i+1);
    %run the model 
    odefun  = @(t,q)[q(2);get_ddqs(q,u)];
    y0=[q0,dq0]';
    tspan=linspace(0,Tsample,100);
    temp=ode4(odefun,tspan,y0);
    % save the trajectory
    traj_sim=[traj_sim,[temp(:,1)';temp(:,1)';temp(:,2)';temp(:,2)']];
    traj_S(i+1,:,:)=temp;
end
%% calculate the GRF
grf=[];
for i=1:1:nNodesS
    for j=1:1:100
        grf=[grf,get_grfs([traj_S(i,j,1),traj_S(i,j,2)]',traj2(3,i))];% Stance GRF
    end
end
%% FLIGHT 2
Tsample = x(3)/nNodesF2;
for i = 0:1:nNodesF2-1
    % get the start nodes
    q0 = traj3(1:2,i+1)';
    dq0 = traj3(3:4,i+1)';
    u = traj3(5,i+1);
    %run the model 
    odefun  = @(t,q)[q(3);q(4);get_ddq(q,u)];
    y0=[q0,dq0]';
    tspan=linspace(0,Tsample,100);
    temp=ode4(odefun,tspan,y0);
    %save the trajectory
    traj_sim=[traj_sim,temp'];
end
time_sim=[linspace(0,x(1),nNodesF1*100),linspace(x(1),x(1)+x(2),nNodesS*100),linspace(x(1)+x(2),x(1)+x(2)+x(3),nNodesF2*100)];

%% The node points
traj=[[traj1(1,:),traj2(1,:),traj3(1,:)];
      [traj1(2,:),traj2(1,:),traj3(2,:)];
      [traj1(3,:),traj2(2,:),traj3(3,:)];
      [traj1(4,:),traj2(2,:),traj3(4,:)];
      [traj1(5,:),traj2(3,:),traj3(5,:)]];

time=[linspace(0,x(1),nNodesF1+1),linspace(x(1),x(1)+x(2),nNodesS+1),linspace(x(1)+x(2),x(1)+x(2)+x(3),nNodesF2+1)];
time(nNodesF1+1)=[];
time(nNodesF1+nNodesS+1)=[];
time(nNodesF1+nNodesS+nNodesF2+1)=[];
%% plot the results
subplot(3,2,1)
plot(time(1:16),traj(1,1:16),'b');
hold on
plot(time(16:21),traj(1,16:21),'r');
plot(time(21:35),traj(1,21:35),'b');
plot(time,traj(1,:)-traj(2,:),'k');
plot(time_sim,traj_sim(1,:),'g');
plot(time_sim,traj_sim(1,:)-traj_sim(2,:),'c');
legend('aerial 1 nodes','stance nodes','aerial 2 nodes','z-l nodes','rerun sim','z-l rerun sim')
plot([time_F1,time_F1],[0,10],'m');
plot([time_F1+time_S,time_F1+time_S],[0,10],'m');
title('z vs time (also z-l versus time)');

subplot(3,2,2)
plot(time(1:16),traj(2,1:16),'b');
hold on
plot(time(16:21),traj(2,16:21),'r');
plot(time(21:35),traj(2,21:35),'b');
plot(time,pogo.parameters.lmax*ones(1,length(time)),'k')
plot(time,pogo.parameters.lmin*ones(1,length(time)),'k')
plot(time_sim,traj_sim(2,:),'g');
title('l versus time')

subplot(3,2,3)
plot(time(1:16),traj(3,1:16),'b');
hold on
plot(time(16:21),traj(3,16:21),'r');
plot(time(21:35),traj(3,21:35),'b');
plot(time_sim,traj_sim(3,:),'g');
title('dz versus time')

subplot(3,2,4)
plot(time(1:16),traj(4,1:16),'b');
hold on
plot(time(16:21),traj(4,16:21),'r');
plot(time(21:35),traj(4,21:35),'b');
plot(time_sim,traj_sim(4,:),'g');
title('dl versus time')

subplot(3,2,5)
stairs(time(1:16),traj(5,1:16),'b')
hold on
stairs(time(16:21),traj(5,16:21),'r')
stairs(time(21:35),traj(5,21:35),'b')
title('applied force versus time')

subplot(3,2,6)
plot(grf)
title('the GRF, for the stance phase only, versus time')