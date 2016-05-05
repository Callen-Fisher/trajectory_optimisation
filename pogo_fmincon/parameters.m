%% Written by Callen Fisher, UCT, 2016.
% please read the readme file

% set the parameters for the model
pogo.parameters.m1=10;
pogo.parameters.m2=1;
pogo.parameters.k=0.5e3;
pogo.parameters.c=1;
pogo.parameters.g=9.81;
pogo.parameters.lmin=0.1;
pogo.parameters.lmax=3;
pogo.parameters.lrest=(pogo.parameters.lmax-pogo.parameters.lmin)/2;
pogo.parameters.time=4;
pogo.parameters.sampleTime=0.001;
pogo.parameters.init_q=[10 pogo.parameters.lrest];
pogo.parameters.init_dq=[0 0];
pogo.parameters.guessForce=inf;