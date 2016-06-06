function [F,G]=objective(x)
global auxdata
[F,G_temp]=pogo_objective_Grd(x,auxdata);
[~,~,G]=find(G_temp);%get the sparce element values