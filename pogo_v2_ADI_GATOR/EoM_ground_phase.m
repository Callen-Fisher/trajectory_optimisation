%% Written by Callen Fisher, UCT, 2016.
% please read the readme file

pogo.ground.genCo=z;
pogo.ground.dgenCo=dz;
%%
q=pogo.ground.genCo;
dq=pogo.ground.dgenCo;
%% define the kinetic energy
T=1/2*pogo.m1*dq(1)^2;
pogo.ground.T=T;
%% define the potential energy
V1=pogo.m1*pogo.g*q(1);
V2=1/2*pogo.k*(pogo.lrest-q).^2;
V=V1+V2;
pogo.ground.V=V;
%% Mass Matrix
M  = sym('M', length(q));
M=jacobian(jacobian(T,dq).',dq);
M = simplify(M);
%% Coriolis and Centripetal Terms
C  = sym(zeros(length(q)));     
for i = 1:length(q(:))
    for j = 1:length(q(:))
        for k = 1:length(q(:))
            C(i,j) = C(i,j)+ 0.5*(diff(M(i,j),q(k)) + diff(M(i,k),q(j)) - diff(M(j,k),q(i)))*dq(k);
        end
    end
end
C = simplify(C);
%% Gravitational Terms
G  = sym('G',[length(q) 1]);  
for i = 1:length(q(:))
    G(i) = diff(V,q(i));
end
G = simplify(G);
%% simplifications
m1  = pogo.parameters.m1;
m2  = pogo.parameters.m2;
k   = pogo.parameters.k;
g   = pogo.parameters.g;
lrest=pogo.parameters.lrest;

CG = subs(C*dq +G);
Minv = inv(M);
%% Contact force 
J    = jacobian(z,q);
%% EOM
ddq  = simplify(Minv*(-CG +pogo.F));
pogo.ground.EOM=ddq;
%% grf
grf = inv(J*Minv*J.')*(J*Minv*(-CG +pogo.F));
%% substitutes 
ddq = subs(ddq);grf=subs(grf);
%% Write Functions ----------------------------
matlabFunction(ddq, 'File',  'get_ddqs.m', 'Vars', {[q;dq],pogo.F});
matlabFunction(grf, 'File',  'get_grfs.m', 'Vars', {[q;dq],pogo.F});
%% add all the matrices to the pogo structure
pogo.ground.T=T;
pogo.ground.V=V;
pogo.ground.M=M;
pogo.ground.C=C;
pogo.ground.G=G;
%% clear the variables
clear T V M C G Q B u A ans ans c lret Minv J grf dgrf ddqf CG lambda lrest