
%% Written by Callen Fisher, UCT, 2016.
% please read the readme file

clc;clear
close all
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Add paths %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('ODE_Solvers');
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% load the pogo %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('results/aerial_stance_pogo_00000.mat')
clear adifuncs EXITFLAG FVAL GRAD Guess HESSIAN LAMBDA MinSoln nNodesF1 nNodesF2 nNodesS OUTPUT Problem time_F1 time_F2 time_S traj traj_sim
clc
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% TRAJECTORY OPTIMISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('multipleshoot');
nNodesF1=15; % 15 nodes for the first aerial phase
nNodesS=5; % 5 nodes for the stance phase
nNodesF2=nNodesF1; % another 15 nodes for the second aerial phase
tic % start the timer
[Problem,Guess,x,fval,exitflag,lambda,states]=traj_opt_multipleshoot(nNodesF1,nNodesS,nNodesF2,pogo); % run the multipleshoot setup function
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

save(filename,'traj','traj_sim','time_F1','time_S','time_F2','nNodesF1','nNodesS','nNodesF2','Guess','pogo','Problem','x','fval','exitflag','lambda','states'); % save all the data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% ADD TO THE GIT REPOSITORY %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
git add *.*
git commit -m filename *.*
%%
clear all