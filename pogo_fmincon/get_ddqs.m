function ddq = get_ddqs(in1,F)
%GET_DDQS
%    DDQ = GET_DDQS(IN1,F)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    04-May-2016 15:30:03

z = in1(1,:);
ddq = F.*(1.0./1.0e1)-z.*5.0e1+6.269e1;
