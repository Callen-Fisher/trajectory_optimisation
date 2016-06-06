function [c,ceq] = pogo_nonlcon(DecVar,auxdata)
nNodesF1=auxdata.nNodesF1;
nNodesS=auxdata.nNodesS;
nNodesF2=auxdata.nNodesF2;
q_0=auxdata.q_0;
dq_0=auxdata.dq_0;
lmin=auxdata.lmin;
lmax=auxdata.lmax;
%% reshape the DecVar into the three trajectories (flight, stance, flight) with [q,dq,u] order
i=nNodesF1*5+3;
traj1=reshape(DecVar(4:i),[5,nNodesF1]);
j=i+nNodesS*3;
traj2=reshape(DecVar(i+1:j),[3,nNodesS]);
traj3=reshape(DecVar(j+1:end),[5,nNodesF2]);
%% define variables 
start_states_F1 = ones(nNodesF1,4);
final_states_F1 = start_states_F1;
start_states_S = ones(nNodesS,2);
final_states_S = start_states_S;
start_states_F2 = ones(nNodesF2,4);
final_states_F2 = start_states_F2;
defects =[];
traj_F1=[];
traj_S=[];
traj_F2=[];
Tsample_F1 = DecVar(1)/nNodesF1;
Tsample_S = DecVar(2)/nNodesS;
Tsample_F2 = DecVar(3)/nNodesF2;
%% FLIGHT 1 and 2
for i = 0:1:nNodesF1-1
    % FLIGHT 1
    u = traj1(5,i+1);
    start_states_F1(i+1,:) = [traj1(1:2,i+1)',traj1(3:4,i+1)'];
    %run the model 
    %odefun  = @(t,q)[q(3);q(4);get_ddq(q,u)];
    tspan=linspace(0,Tsample_F1,100);
    %temp=ode4(odefun,tspan,start_states_F1(i+1,:)');
    
    hi = tspan(2)-tspan(1);
    neq = length(start_states_F1(i+1,:)');
    N = length(tspan);
    Y = zeros(neq,N);
    F = zeros(neq,4);
    Y(:,1) = start_states_F1(i+1,:)';
    for j = 2:N
      yi = Y(:,j-1);

      tempVar=get_ddq(yi,u);
      F(1,1) = yi(3);
      F(2,1) = yi(4);
      F(3,1) = tempVar(1);%u.*(-1.0./1.0e1)+yi(2).*5.0e1-8.231e1;
      F(4,1) = tempVar(2);%u.*(-1.1e1./1.0e1)+yi(2).*5.5e2-7.975e2;
      
      tempVar=get_ddq((yi+0.5*hi*F(:,1)),u);
      F(1,2) = yi(3)+0.5*hi*F(3,1);
      F(2,2) = yi(4)+0.5*hi*F(4,1);
      F(3,2) = tempVar(1);%u.*(-1.0./1.0e1)+(yi(2)+0.5*hi*F(2,1)).*5.0e1-8.231e1;
      F(4,2) = tempVar(2);%u.*(-1.1e1./1.0e1)+(yi(2)+0.5*hi*F(2,1)).*5.5e2-7.975e2;
      
      tempVar=get_ddq(yi+0.5*hi*F(:,2),u);
      F(1,3) = yi(3)+0.5*hi*F(3,2);
      F(2,3) = yi(4)+0.5*hi*F(4,2);
      F(3,3) = tempVar(1);%u.*(-1.0./1.0e1)+(yi(2)+0.5*hi*F(2,2)).*5.0e1-8.231e1;
      F(4,3) = tempVar(2);%u.*(-1.1e1./1.0e1)+(yi(2)+0.5*hi*F(2,2)).*5.5e2-7.975e2;
      
      tempVar=get_ddq(yi+hi*F(:,3),u);
      F(1,4) = yi(3)+hi*F(3,3);
      F(2,4) = yi(4)+hi*F(4,3);
      F(3,4) = tempVar(1);%u.*(-1.0./1.0e1)+(yi(2)+hi*F(2,3)).*5.0e1-8.231e1;
      F(4,4) = tempVar(2);%u.*(-1.1e1./1.0e1)+(yi(2)+hi*F(2,3)).*5.5e2-7.975e2;
      
      Y(1,j) = yi(1) + (hi/6)*(F(1,1) + 2*F(1,2) + 2*F(1,3) + F(1,4));
      Y(2,j) = yi(2) + (hi/6)*(F(2,1) + 2*F(2,2) + 2*F(2,3) + F(2,4));
      Y(3,j) = yi(3) + (hi/6)*(F(3,1) + 2*F(3,2) + 2*F(3,3) + F(3,4));
      Y(4,j) = yi(4) + (hi/6)*(F(4,1) + 2*F(4,2) + 2*F(4,3) + F(4,4));
    end
    temp(:,:) =Y(:,:).';
    
    %save the results
    traj_F1=[traj_F1;temp];
    final_states_F1(i+1,:) = temp(end,:);
    %%
    %FLIGHT 2
    u =traj3(5,i+1);
    start_states_F2(i+1,:) = [traj3(1:2,i+1)',traj3(3:4,i+1)'];
    %run the model 
    %odefun  = @(t,q)[q(3);q(4);get_ddq(q,u)];
    tspan=linspace(0,Tsample_F2,100);
    %temp=ode4(odefun,tspan,start_states_F2(i+1,:)');
    hi = tspan(2)-tspan(1);
    neq = length(start_states_F2(i+1,:)');
    N = length(tspan);
    Y = zeros(neq,N);
    F = zeros(neq,4);
    Y(:,1) = start_states_F2(i+1,:)';
    for j = 2:N
      yi = Y(:,j-1);

      tempVar=get_ddq(yi,u);
      F(1,1) = yi(3);
      F(2,1) = yi(4);
      F(3,1) = tempVar(1);%u.*(-1.0./1.0e1)+yi(2).*5.0e1-8.231e1;
      F(4,1) = tempVar(2);%u.*(-1.1e1./1.0e1)+yi(2).*5.5e2-7.975e2;
      
      %yi+0.5*hi*F(:,1)
      tempVar=get_ddq(yi+0.5*hi*F(:,1),u);
      F(1,2) = yi(3)+0.5*hi*F(3,1);
      F(2,2) = yi(4)+0.5*hi*F(4,1);
      F(3,2) = tempVar(1);%u.*(-1.0./1.0e1)+(yi(2)+0.5*hi*F(2,1)).*5.0e1-8.231e1;
      F(4,2) = tempVar(2);%u.*(-1.1e1./1.0e1)+(yi(2)+0.5*hi*F(2,1)).*5.5e2-7.975e2;
      
      %yi+0.5*hi*F(:,2)
      tempVar=get_ddq(yi+0.5*hi*F(:,2),u);
      F(1,3) = yi(3)+0.5*hi*F(3,2);
      F(2,3) = yi(4)+0.5*hi*F(4,2);
      F(3,3) = tempVar(1);%u.*(-1.0./1.0e1)+(yi(2)+0.5*hi*F(2,2)).*5.0e1-8.231e1;
      F(4,3) = tempVar(2);%u.*(-1.1e1./1.0e1)+(yi(2)+0.5*hi*F(2,2)).*5.5e2-7.975e2;
      
      %yi+hi*F(:,3)
      tempVar=get_ddq(yi+hi*F(:,3),u);
      F(1,4) = yi(3)+hi*F(3,3);
      F(2,4) = yi(4)+hi*F(4,3);
      F(3,4) = tempVar(1);%u.*(-1.0./1.0e1)+(yi(2)+hi*F(2,3)).*5.0e1-8.231e1;
      F(4,4) = tempVar(2);%u.*(-1.1e1./1.0e1)+(yi(2)+hi*F(2,3)).*5.5e2-7.975e2;
      
      Y(1,j) = yi(1) + (hi/6)*(F(1,1) + 2*F(1,2) + 2*F(1,3) + F(1,4));
      Y(2,j) = yi(2) + (hi/6)*(F(2,1) + 2*F(2,2) + 2*F(2,3) + F(2,4));
      Y(3,j) = yi(3) + (hi/6)*(F(3,1) + 2*F(3,2) + 2*F(3,3) + F(3,4));
      Y(4,j) = yi(4) + (hi/6)*(F(4,1) + 2*F(4,2) + 2*F(4,3) + F(4,4));
    end
    temp = Y.';
    
    %save the results
    traj_F2=[traj_F2;temp];
    final_states_F2(i+1,:) = temp(end,:);
end
%% STANCE
for i = 0:1:nNodesS-1
    u = traj2(3,i+1);
    start_states_S(i+1,:) = [traj2(1,i+1),traj2(2,i+1)];
    %run the model 
    %odefun  = @(t,q)[q(2);get_ddqs(q,u)];
    tspan=linspace(0,Tsample_S,100);
    %temp=ode4(odefun,tspan,start_states_S(i+1,:)');
    hi = tspan(2)-tspan(1);
    neq = length(start_states_S(i+1,:)');
    N = length(tspan);
    Y = zeros(neq,N);
    F = zeros(neq,4);
    Y(:,1) = start_states_S(i+1,:)';
    for j = 2:N
      yi = Y(:,j-1);

      tempVar=get_ddqs(yi,u);
      F(1,1) = yi(2);
      F(2,1) = tempVar;%u.*(1.0./1.0e1)-yi(1).*5.0e1+6.269e1;
      
      %yi+0.5*hi*F(:,1)
      tempVar=get_ddqs(yi+0.5*hi*F(:,1),u);
      F(1,2) = yi(2)+0.5*hi*F(2,1);
      F(2,2) = tempVar;%u.*(1.0./1.0e1)-(yi(1)+0.5*hi*F(1,1)).*5.0e1+6.269e1;
      
      %yi+0.5*hi*F(:,2)
      tempVar=get_ddqs(yi+0.5*hi*F(:,2),u);
      F(1,3) = yi(2)+0.5*hi*F(2,2);
      F(2,3) = tempVar;%u.*(1.0./1.0e1)-(yi(1)+0.5*hi*F(1,2)).*5.0e1+6.269e1;
      
      %yi+hi*F(:,3)
      tempVar=get_ddqs(yi+hi*F(:,3),u);
      F(1,4) = yi(2)+hi*F(2,3);
      F(2,4) = tempVar;%u.*(1.0./1.0e1)-(yi(1)+hi*F(1,3)).*5.0e1+6.269e1;
      
      Y(1,j) = yi(1) + (hi/6)*(F(1,1) + 2*F(1,2) + 2*F(1,3) + F(1,4));
      Y(2,j) = yi(2) + (hi/6)*(F(2,1) + 2*F(2,2) + 2*F(2,3) + F(2,4));
    end
    temp = Y.';
    %save the results
    traj_S=[traj_S;temp];
    final_states_S(i+1,:) = temp(end,:);
end
%% calculate the defects 
for i = [2:1:nNodesF1]
    defects = [defects, start_states_F1(i,:)-final_states_F1(i-1,:), start_states_F2(i,:)-final_states_F2(i-1,:)];
end
defects = [defects,start_states_S(1,1)-final_states_F1(end,1),start_states_S(1,1)-final_states_F1(end,2)];% only the pos at switching surface 1
defects = [defects,start_states_S(1,2)-final_states_F1(end,3),start_states_S(1,2)-final_states_F1(end,4)];% velocity defects at switching surface 1
for i = [2:1:nNodesS]
    defects = [defects, start_states_S(i,:)-final_states_S(i-1,:)];
end
defects = [defects,start_states_F2(1,1)-final_states_S(end,1),start_states_F2(1,2)-final_states_S(end,1)];% only the pos at switching surface 2
defects = [defects,final_states_S(end,2)-start_states_F2(1,3),final_states_S(end,2)-start_states_F2(1,4)];% velocity defects at switching surface 2
%% length constraints 
temp1=[];
temp1=[temp1,-traj_F1(:,1)'+traj_F1(:,2)',-traj_F2(:,1)'+traj_F2(:,2)'];
temp1=[temp1,-traj_S(:,1)'+lmin,traj_S(:,1)'-lmax];
% for i=1:1:nNodesF1
%     temp1=[temp1,-traj_F1(i,:,1)+traj_F1(i,:,2),-traj_F2(i,:,1)+traj_F2(i,:,2)];%q-l>0, m1 and m2 must be above the ground
% end
% for i=1:1:nNodesS
%     temp1=[temp1,-traj_S(i,:,1)+lmin,traj_S(i,:,1)-lmax];%z>lmin%z<lmax, z is bounded between lmin and lmax when on the ground
% end
%% GRF
grf=[];
for i=1:1:nNodesS
    for j=1:1:100
        grf=[grf,traj2(3,i)-traj_S((i-1)*100+j,1).*5.0e2+6.269e2];
        
        
        %grf=[grf,get_grfs([traj_S(i,j,1),traj_S(i,j,2)]',traj2(3,i))];% Stance GRF, when on the ground the ground must exert a force on the foot (+ force, must never go neg)
    end
end
%% initial and final conditions
pogo_Init = [q_0,dq_0]-start_states_F1(1,:);%the pogo stick must start at the initial conditions
pogo_Final = [q_0,dq_0]-final_states_F2(end,:); %the pogo stick must end at the initial conditions (periodic motion constraint)
%% switching surface 1
touch_down_pos=final_states_F1(end,1)-final_states_F1(end,2);%z-l=0, The pogo stick must touch the ground at the end of the first flight stage
%% switching surface 2
lift_off_pos=start_states_F2(1,1)-start_states_F2(1,2);%z-l=0, the pogo stick must start on the ground at the start of the second flight stage
%% ground equalities and inequalities
c = [temp1,-grf(1:end-1)];%<=0 
ceq = [defects,pogo_Init,pogo_Final,touch_down_pos,lift_off_pos,grf(end)];%=0  ,