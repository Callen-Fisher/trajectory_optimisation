function[c,ceq,dc,dceq]=constraint(x)
global auxdata
[c_temp,ceq_temp,dc_temp,dceq_temp]=pogo_nonlcon_Grd(x,auxdata);
c=c_temp';
ceq=ceq_temp';
dc=full(dc_temp)';
dceq=full(dceq_temp)';