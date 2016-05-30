% This code was generated using ADiGator version 1.2
% ©2010-2014 Matthew J. Weinstein and Anil V. Rao
% ADiGator may be obtained at https://sourceforge.net/projects/adigator/ 
% Contact: mweinstein@ufl.edu
% Bugs/suggestions may be reported to the sourceforge forums
%                    DISCLAIMER
% ADiGator is a general-purpose software distributed under the GNU General
% Public License version 3.0. While the software is distributed with the
% hope that it will be useful, both the software and generated code are
% provided 'AS IS' with NO WARRANTIES OF ANY KIND and no merchantability
% or fitness for any purpose or application.

function cost = pogo_objective_ADiGatorGrd(DecVar,auxdata)
global ADiGator_pogo_objective_ADiGatorGrd
if isempty(ADiGator_pogo_objective_ADiGatorGrd); ADiGator_LoadData(); end
Gator1Data = ADiGator_pogo_objective_ADiGatorGrd.pogo_objective_ADiGatorGrd.Gator1Data;
% ADiGator Start Derivative Computations
nNodesF1 = auxdata.nNodesF1;
%User Line: nNodesF1=auxdata.nNodesF1;
nNodesS = auxdata.nNodesS;
%User Line: nNodesS=auxdata.nNodesS;
nNodesF2 = auxdata.nNodesF2;
%User Line: nNodesF2=auxdata.nNodesF2;
%User Line: %minimize applied force
%User Line: %% reshape the DecVar to find the applied force
cada1f1 = nNodesF1*5;
i.f = cada1f1 + 3;
%User Line: i=nNodesF1*5+3;
cada1f1 = 4:i.f;
cada1f2dx = DecVar.dx(Gator1Data.Index1);
cada1f2 = DecVar.f(cada1f1);
cada1f3 = [5 nNodesF1];
traj1.dx = cada1f2dx;
traj1.f = reshape(cada1f2,cada1f3);
%User Line: traj1=reshape(DecVar(4:i),[5,nNodesF1]);
cada1f1 = nNodesS*3;
j.f = i.f + cada1f1;
%User Line: j=i+nNodesS*3;
cada1f1 = i.f + 1;
cada1f2 = cada1f1:j.f;
cada1f3dx = DecVar.dx(Gator1Data.Index2);
cada1f3 = DecVar.f(cada1f2);
cada1f4 = [3 nNodesS];
traj2.dx = cada1f3dx;
traj2.f = reshape(cada1f3,cada1f4);
%User Line: traj2=reshape(DecVar(i+1:j),[3,nNodesS]);
cada1f1 = j.f + 1;
cada1f2 = length(DecVar.f);
cada1f3 = cada1f1:cada1f2;
cada1f4dx = DecVar.dx(Gator1Data.Index3);
cada1f4 = DecVar.f(cada1f3);
cada1f5 = [5 nNodesF2];
traj3.dx = cada1f4dx;
traj3.f = reshape(cada1f4,cada1f5);
%User Line: traj3=reshape(DecVar(j+1:end),[5,nNodesF2]);
%User Line: %% calculate the time for each node
cada1f1dx = DecVar.dx(1);
cada1f1 = DecVar.f(1);
dtF1.dx = cada1f1dx./nNodesF1;
dtF1.f = cada1f1/nNodesF1;
%User Line: dtF1=DecVar(1)/nNodesF1;
cada1f1dx = DecVar.dx(2);
cada1f1 = DecVar.f(2);
dtS.dx = cada1f1dx./nNodesS;
dtS.f = cada1f1/nNodesS;
%User Line: dtS=DecVar(2)/nNodesS;
cada1f1dx = DecVar.dx(3);
cada1f1 = DecVar.f(3);
dtF2.dx = cada1f1dx./nNodesF2;
dtF2.f = cada1f1/nNodesF2;
%User Line: dtF2=DecVar(3)/nNodesF2;
%User Line: %% calculate the cost function
cada1f1dx = traj1.dx(Gator1Data.Index4);
cada1f1 = traj1.f(5,:);
cada1f2dx = 2.*cada1f1(:).^(2-1).*cada1f1dx;
cada1f2 = cada1f1.^2;
cada1f3dx = cada1f2dx;
cada1f3 = sum(cada1f2);
cada1td1 = zeros(16,1);
cada1td1(Gator1Data.Index5) = dtF1.f.*cada1f3dx;
cada1td1(1) = cada1td1(1) + cada1f3.*dtF1.dx;
cada1f4dx = cada1td1;
cada1f4 = cada1f3*dtF1.f;
cada1f5dx = traj2.dx(Gator1Data.Index6);
cada1f5 = traj2.f(3,:);
cada1f6dx = 2.*cada1f5(:).^(2-1).*cada1f5dx;
cada1f6 = cada1f5.^2;
cada1f7dx = cada1f6dx;
cada1f7 = sum(cada1f6);
cada1td1 = zeros(6,1);
cada1td1(Gator1Data.Index7) = dtS.f.*cada1f7dx;
cada1td1(1) = cada1td1(1) + cada1f7.*dtS.dx;
cada1f8dx = cada1td1;
cada1f8 = cada1f7*dtS.f;
cada1td1 = zeros(22,1);
cada1td1(Gator1Data.Index8) = cada1f4dx;
cada1td1(Gator1Data.Index9) = cada1td1(Gator1Data.Index9) + cada1f8dx;
cada1f9dx = cada1td1;
cada1f9 = cada1f4 + cada1f8;
cada1f10dx = traj3.dx(Gator1Data.Index10);
cada1f10 = traj3.f(5,:);
cada1f11dx = 2.*cada1f10(:).^(2-1).*cada1f10dx;
cada1f11 = cada1f10.^2;
cada1f12dx = cada1f11dx;
cada1f12 = sum(cada1f11);
cada1td1 = zeros(16,1);
cada1td1(Gator1Data.Index11) = dtF2.f.*cada1f12dx;
cada1td1(1) = cada1td1(1) + cada1f12.*dtF2.dx;
cada1f13dx = cada1td1;
cada1f13 = cada1f12*dtF2.f;
cada1td1 = zeros(38,1);
cada1td1(Gator1Data.Index12) = cada1f9dx;
cada1td1(Gator1Data.Index13) = cada1td1(Gator1Data.Index13) + cada1f13dx;
cost.dx = cada1td1;
cost.f = cada1f9 + cada1f13;
%User Line: cost=sum(traj1(5,:).^2)*dtF1+sum(traj2(3,:).^2)*dtS+sum(traj3(5,:).^2)*dtF2;
cost.dx_size = 168;
cost.dx_location = Gator1Data.Index14;
end


function ADiGator_LoadData()
global ADiGator_pogo_objective_ADiGatorGrd
ADiGator_pogo_objective_ADiGatorGrd = load('pogo_objective_ADiGatorGrd.mat');
return
end