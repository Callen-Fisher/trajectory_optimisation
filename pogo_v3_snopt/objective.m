function [F,G]=objective(x)
global auxdata
[F,G_temp]=pogo_objective_Grd(x,auxdata);
G=G_temp';