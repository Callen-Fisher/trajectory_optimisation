function ddq = get_ddq(in1,F)
%GET_DDQ
%    DDQ = GET_DDQ(IN1,F)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    10-May-2016 06:57:48

l = in1(2,:);
ddq = [F.*(-1.0./1.0e1)+l.*5.0e1-8.231e1;F.*(-1.1e1./1.0e1)+l.*5.5e2-7.975e2];
