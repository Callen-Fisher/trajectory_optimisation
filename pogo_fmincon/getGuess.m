function [ Guess ] = getGuess( nNodesF1,nNodesS,nNodesF2,pogo)
%getGuess Get the initial guess for the optimisation
%   two types of guesses are available, eithe a hand calculated guess which
%   is not ideal, or a previous result can be used as the initial guess.
%% Written by Callen Fisher, UCT, 2016.
% please read the readme file
%% use a previous result as the guess
load('results/aerial_stance_pogo_00009.mat')
Guess=MinSoln;

%% uncomment this for the hand calculated guess
% Guess=zeros(1,3+nNodesF1*5+nNodesS*3+nNodesF2*5);
% l=pogo.parameters.lrest;
% z_f1=linspace(pogo.parameters.init_q(1),l,nNodesF1);
% z_f2=linspace(l,pogo.parameters.init_q(1),nNodesF2);
% 
% k=1;
% for i=5:5:5*nNodesF1
%     Guess(i+3)=inf;
%     Guess(i-4+3)=z_f1(k);
%     Guess(i-3+3)=l;
%     %Guess(i-2+3)=0;
%     %Guess(i-1+3)=0;
%     k=k+1;
% end
% for i=5*nNodesF1+3:3:5*nNodesF1+3*nNodesS
%     Guess(i+3)=inf;
%     Guess(i-2+3)=l;
%     %Guess(i-1+3)=0;
% end
% k=1;
% for i=5*nNodesF1+3*nNodesS+5:5:5*nNodesF1+3*nNodesS+5*nNodesF2
%     Guess(i+3)=inf;
%     Guess(i-4+3)=z_f2(k);
%     Guess(i-3+3)=l;
%     %Guess(i-2+3)=0;
%     %Guess(i-1+3)=0;
%     k=k+1;
% end
% 
% Guess(1)=3;
% Guess(2)=2;
% Guess(3)=3;

end

