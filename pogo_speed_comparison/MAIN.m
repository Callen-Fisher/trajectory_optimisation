
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
load('results/aerial_stance_pogo_00000.mat')%load the pogo structure
clear adifuncs EXITFLAG FVAL GRAD Guess HESSIAN LAMBDA MinSoln nNodesF1 nNodesF2 nNodesS OUTPUT Problem time_F1 time_F2 time_S traj traj_sim
clc
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% TRAJECTORY OPTIMISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('multipleshoot');
nNodesF1=15; % 15 nodes for the first aerial phase
nNodesS=5; % 5 nodes for the stance phase
nNodesF2=nNodesF1; % another 15 nodes for the second aerial phase

traj_opt_multipleshoot(nNodesF1,nNodesS,nNodesF2,pogo); % run the multipleshoot setup function