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

function cost = pogo_objective_ADiGatorHes(DecVar,auxdata)
global ADiGator_pogo_objective_ADiGatorHes
if isempty(ADiGator_pogo_objective_ADiGatorHes); ADiGator_LoadData(); end
Gator1Data = ADiGator_pogo_objective_ADiGatorHes.pogo_objective_ADiGatorHes.Gator1Data;
Gator2Data = ADiGator_pogo_objective_ADiGatorHes.pogo_objective_ADiGatorHes.Gator2Data;
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
dtF1.dx = cada1f1dx/nNodesF1;
dtF1.f = cada1f1/nNodesF1;
%User Line: dtF1=DecVar(1)/nNodesF1;
cada1f1dx = DecVar.dx(2);
cada1f1 = DecVar.f(2);
dtS.dx = cada1f1dx/nNodesS;
dtS.f = cada1f1/nNodesS;
%User Line: dtS=DecVar(2)/nNodesS;
cada1f1dx = DecVar.dx(3);
cada1f1 = DecVar.f(3);
dtF2.dx = cada1f1dx/nNodesF2;
dtF2.f = cada1f1/nNodesF2;
%User Line: dtF2=DecVar(3)/nNodesF2;
%User Line: %% calculate the cost function
cada1f1dx = traj1.dx(Gator1Data.Index4);
cada1f1 = traj1.f(5,:);
cada2f1dx = cada1f1dx;
cada2f1 = cada1f1(:);
cada2f2dx = 1.*cada2f1(:).^(1-1).*cada2f1dx;
cada2f2 = cada2f1.^1;
cada2f3dx = 2.*cada2f2dx;
cada2f3 = 2*cada2f2;
cada1f2dxdx = cada1f1dx(:).*cada2f3dx;
cada1f2dx = cada2f3.*cada1f1dx;
cada1f2 = cada1f1.^2;
cada1f3dxdx = cada1f2dxdx; cada1f3dx = cada1f2dx;
cada1f3 = sum(cada1f2);
cada1td1 =  zeros(16,1);
cada2tempdx = dtF1.dx(Gator2Data.Index1);
cada2td1 = zeros(30,1);
cada2td1(Gator2Data.Index2) = cada1f3dx(:).*cada2tempdx;
cada2td1(Gator2Data.Index3) = cada2td1(Gator2Data.Index3) + dtF1.f.*cada1f3dxdx;
cada2f1dx = cada2td1;
cada2f1 = dtF1.f*cada1f3dx;
cada1td1dx = cada2f1dx;
cada1td1(Gator1Data.Index5) = cada2f1;
cada2f1 = cada1td1(1);
cada2f2dx = dtF1.dx.*cada1f3dx;
cada2f2 = cada1f3*dtF1.dx;
cada2f3dx = cada2f2dx;
cada2f3 = cada2f1 + cada2f2;
cada2td1 = zeros(45,1);
cada2td1(Gator2Data.Index4) = cada2f3dx;
cada2td1(Gator2Data.Index5) = cada1td1dx(Gator2Data.Index6);
cada1td1dx = cada2td1;
cada1td1(1) = cada2f3;
cada1f4dxdx = cada1td1dx; cada1f4dx = cada1td1;
cada1f4 = cada1f3*dtF1.f;
cada1f5dx = traj2.dx(Gator1Data.Index6);
cada1f5 = traj2.f(3,:);
cada2f1dx = cada1f5dx;
cada2f1 = cada1f5(:);
cada2f2dx = 1.*cada2f1(:).^(1-1).*cada2f1dx;
cada2f2 = cada2f1.^1;
cada2f3dx = 2.*cada2f2dx;
cada2f3 = 2*cada2f2;
cada1f6dxdx = cada1f5dx(:).*cada2f3dx;
cada1f6dx = cada2f3.*cada1f5dx;
cada1f6 = cada1f5.^2;
cada1f7dxdx = cada1f6dxdx; cada1f7dx = cada1f6dx;
cada1f7 = sum(cada1f6);
cada1td1 =  zeros(6,1);
cada2tempdx = dtS.dx(Gator2Data.Index7);
cada2td1 = zeros(10,1);
cada2td1(Gator2Data.Index8) = cada1f7dx(:).*cada2tempdx;
cada2td1(Gator2Data.Index9) = cada2td1(Gator2Data.Index9) + dtS.f.*cada1f7dxdx;
cada2f1dx = cada2td1;
cada2f1 = dtS.f*cada1f7dx;
cada1td1dx = cada2f1dx;
cada1td1(Gator1Data.Index7) = cada2f1;
cada2f1 = cada1td1(1);
cada2f2dx = dtS.dx.*cada1f7dx;
cada2f2 = cada1f7*dtS.dx;
cada2f3dx = cada2f2dx;
cada2f3 = cada2f1 + cada2f2;
cada2td1 = zeros(15,1);
cada2td1(Gator2Data.Index10) = cada2f3dx;
cada2td1(Gator2Data.Index11) = cada1td1dx(Gator2Data.Index12);
cada1td1dx = cada2td1;
cada1td1(1) = cada2f3;
cada1f8dxdx = cada1td1dx; cada1f8dx = cada1td1;
cada1f8 = cada1f7*dtS.f;
cada1td1 =  zeros(22,1);
cada1td1dx = cada1f4dxdx;
cada1td1(Gator1Data.Index8) = cada1f4dx;
cada2f1 = cada1td1(Gator1Data.Index9);
cada2f2dx = cada1f8dxdx;
cada2f2 = cada2f1 + cada1f8dx;
cada2td1 = zeros(60,1);
cada2td1(Gator2Data.Index13) = cada2f2dx;
cada2td1(Gator2Data.Index14) = cada1td1dx(Gator2Data.Index15);
cada1td1dx = cada2td1;
cada1td1(Gator1Data.Index9) = cada2f2;
cada1f9dxdx = cada1td1dx; cada1f9dx = cada1td1;
cada1f9 = cada1f4 + cada1f8;
cada1f10dx = traj3.dx(Gator1Data.Index10);
cada1f10 = traj3.f(5,:);
cada2f1dx = cada1f10dx;
cada2f1 = cada1f10(:);
cada2f2dx = 1.*cada2f1(:).^(1-1).*cada2f1dx;
cada2f2 = cada2f1.^1;
cada2f3dx = 2.*cada2f2dx;
cada2f3 = 2*cada2f2;
cada1f11dxdx = cada1f10dx(:).*cada2f3dx;
cada1f11dx = cada2f3.*cada1f10dx;
cada1f11 = cada1f10.^2;
cada1f12dxdx = cada1f11dxdx; cada1f12dx = cada1f11dx;
cada1f12 = sum(cada1f11);
cada1td1 =  zeros(16,1);
cada2tempdx = dtF2.dx(Gator2Data.Index16);
cada2td1 = zeros(30,1);
cada2td1(Gator2Data.Index17) = cada1f12dx(:).*cada2tempdx;
cada2td1(Gator2Data.Index18) = cada2td1(Gator2Data.Index18) + dtF2.f.*cada1f12dxdx;
cada2f1dx = cada2td1;
cada2f1 = dtF2.f*cada1f12dx;
cada1td1dx = cada2f1dx;
cada1td1(Gator1Data.Index11) = cada2f1;
cada2f1 = cada1td1(1);
cada2f2dx = dtF2.dx.*cada1f12dx;
cada2f2 = cada1f12*dtF2.dx;
cada2f3dx = cada2f2dx;
cada2f3 = cada2f1 + cada2f2;
cada2td1 = zeros(45,1);
cada2td1(Gator2Data.Index19) = cada2f3dx;
cada2td1(Gator2Data.Index20) = cada1td1dx(Gator2Data.Index21);
cada1td1dx = cada2td1;
cada1td1(1) = cada2f3;
cada1f13dxdx = cada1td1dx; cada1f13dx = cada1td1;
cada1f13 = cada1f12*dtF2.f;
cada1td1 =  zeros(38,1);
cada1td1dx = cada1f9dxdx;
cada1td1(Gator1Data.Index12) = cada1f9dx;
cada2f1 = cada1td1(Gator1Data.Index13);
cada2f2dx = cada1f13dxdx;
cada2f2 = cada2f1 + cada1f13dx;
cada2td1 = zeros(105,1);
cada2td1(Gator2Data.Index22) = cada2f2dx;
cada2td1(Gator2Data.Index23) = cada1td1dx(Gator2Data.Index24);
cada1td1dx = cada2td1;
cada1td1(Gator1Data.Index13) = cada2f2;
cost.dxdx = cada1td1dx; cost.dx = cada1td1;
cost.f = cada1f9 + cada1f13;
%User Line: cost=sum(traj1(5,:).^2)*dtF1+sum(traj2(3,:).^2)*dtS+sum(traj3(5,:).^2)*dtF2;
cost.dx_size = 168;
cost.dx_location = Gator1Data.Index14;
cost.dxdx_size = [cost.dx_size,168];
cost.dxdx_location = [cost.dx_location(Gator2Data.Index25,:), Gator2Data.Index26];
end


function ADiGator_LoadData()
global ADiGator_pogo_objective_ADiGatorHes
ADiGator_pogo_objective_ADiGatorHes = load('pogo_objective_ADiGatorHes.mat');
return
end