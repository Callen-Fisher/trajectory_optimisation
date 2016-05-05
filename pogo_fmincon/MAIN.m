%% Written by Callen Fisher, UCT, 2016.
% please read the readme file

clc;clear
close all
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Add paths %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('ODE_Solvers'); %add the path for the ODE4 solver
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%   GENERATE EOM   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('generating the EOM')
run('setup'); % create the symbolic variables
run('parameters'); % assign parameters
disp('calculating ground EOM') 
run('EoM_ground_phase'); % calculate the ground phase EOM
disp('calculating aerial EOM') 
run('EoM_aerial_phase'); % calculate the aerial EOM
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% TRAJECTORY OPTIMISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('multipleshoot');
nNodesF1=15; % 15 nodes for the first aerial phase
nNodesS=5; % 5 nodes for the stance phase
nNodesF2=nNodesF1; % another 15 nodes for the second aerial phase
tic % start the timer
[MinSoln,Problem,Guess,FVAL,LAMBDA,GRAD,HESSIAN,EXITFLAG,OUTPUT]=traj_opt_multipleshoot(nNodesF1,nNodesS,nNodesF2,pogo); % runt he multipleshoot setup function
toc % end the timer and display how long the optimisation took
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% RE-RUN THE SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('re_running the simulation');
run('test'); % take the optimised node points and re-integrate to get a finer resolution and then plot the data
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%  SAVE THE DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% this code determines what file has been saved, increments and saves the
% results, therefore the previous results are not overwritten
path='results/';
name='aerial_stance_pogo_';
opt_num=0;
numText=sprintf('%.5d',opt_num);
while(exist([path name numText '.mat'],'file')==2)
    opt_num = opt_num+1;
    numText = sprintf('%.5d',opt_num);
end
filename = [path name numText];
save(filename,'traj','traj_sim','time_F1','time_S','time_F2','nNodesF1','nNodesS','nNodesF2','MinSoln','Guess','pogo','Problem','FVAL','LAMBDA','GRAD','HESSIAN','EXITFLAG','OUTPUT'); % save all the data
clear all