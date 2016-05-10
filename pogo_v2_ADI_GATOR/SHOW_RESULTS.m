clc;clear
close all
%% load the results
path='results/';
name='aerial_stance_pogo_';
opt_num=0;
numText=sprintf('%.5d',opt_num);
while(exist([path name numText '.mat'],'file')==2)
    opt_num = opt_num+1;
    numText = sprintf('%.5d',opt_num);
end
numText = sprintf('%.5d',opt_num-1);
filename = [path name numText];
load(filename);
%% run the test script to display the results
run('test');
figure(2)
for i=1:1:3
    i % display the iteration number
    animate(traj_sim,time_sim(end),30,false); % run animation to animate the results
end
i=i+1
animate(traj_sim,time_sim(end),30,true);% save the animation video