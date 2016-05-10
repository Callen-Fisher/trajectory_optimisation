%% Written by Callen Fisher, UCT, 2016.
% please read the readme file

% define all the symbolic variables for the model
l=sym('l');
z=sym('z');
F=sym('F');
lmin=sym('lmin');
lmax=sym('lmax');
lrest=sym('lrest');
g=sym('g');
m1=sym('m1');
m2=sym('m2');
dz=sym('dz');
ddz=sym('ddz');
dl=sym('dl');
ddl=sym('ddl');
k=sym('k');
c=sym('c');
lambda=sym('lambda');

%% save the variables in the pogo stick structure
pogo.m1=m1;
pogo.m2=m2;
pogo.lmax=lmax;
pogo.lmin=lmin;
pogo.g=g;
pogo.lrest=lrest;
pogo.k=k;
pogo.c=c;
pogo.lambda=lambda;
pogo.F=F;