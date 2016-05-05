%% Written by Callen Fisher, UCT, 2016.
% please read the readme file

pogo.aerial.genCo=[z;l];
pogo.aerial.dgenCo=[dz;dl];
%%
q=pogo.aerial.genCo;
dq=pogo.aerial.dgenCo;
%% define the kinetic energy
T1=1/2*pogo.m1*dq(1)^2;
T2=1/2*pogo.m2*(dq(1)-dq(2)).^2;
T=T1+T2;
pogo.aerial.T=T;
%% define the potential energy
V1=m1*pogo.g*q(1);
V2=m2*pogo.g*(q(1)-q(2));
V3=-1/2*pogo.k*(pogo.lrest-q(2)).^2;%spring force
V=V1+V2+V3;
pogo.aerial.V=V;
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
%% Simplifications
m1  = pogo.parameters.m1;
m2  = pogo.parameters.m2;
lrest  = pogo.parameters.lrest;
k   = pogo.parameters.k;
c   = pogo.parameters.c;
g   = pogo.parameters.g;

CG = subs(C*dq +G);
Minv = inv(M);
%% Damping and contact force
J    = jacobian(z-l,q);%contact jacobian
ddq  = simplify(Minv*(-CG  + [0;-pogo.F]));
pogo.aerial.EOM=ddq;
%% after contact velocity (dq+)=A(dq-)
A = (eye(size(M)) -Minv*J.'*inv(J*Minv*J.')*J);
%% substitutes
ddq = subs(ddq); A=subs(A);
%% Write Functions
matlabFunction(ddq, 'File',  'get_ddq.m',   'Vars', {[q;dq],pogo.F});
matlabFunction(A,   'File',  'get_A.m',     'Vars', {[q;dq]});
%% add all the matrices to the pogo structure
pogo.aerial.T=T;
pogo.aerial.V=V;
pogo.aerial.M=M;
pogo.aerial.C=C;
pogo.aerial.G=G;
%% clear unwanted the variables
clear T1 T2 T V1 V2 V3 V M C Q counter i j k tempCount z r q m l g f F EoM dz dr dq dl ddz ddq ddl G lmax lmin m1 m2 B u A ans c lret Minv lambda J grf dgrf ddqf CG lrest q dq 