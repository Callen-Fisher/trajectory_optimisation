function funcs = adigatorGenFiles4Ipopt(setup)
% funcs = adigatorGenFiles4Ipopt(setup)
%
% ADiGator Gradient/Jacobian/Hessian File Generation Function: this
% function is used when you wish to generate derivative files for IPOPT.
% The user must specify their objective and (optionally) constraint
% functions, and whether they want to generate 1st or 2nd derivatives. This
% function calls adigator on the user's functions and creates wrapper files
% for the gradient/jacobian/hessian, of the form required by IPOPT. The
% output of this function is the 'funcs' input to the ipopt call. Note: all
% auxiliary data must be both known and fixed when passed to this function.
%
% IPOPT is an open source, primal-dual interior point NLP solver. For more
% information on IPOPT and to obtain a MEX file, visit:
%    http://www.cs.ubc.ca/~pcarbo/ipopt-for-matlab
%    http://projects.coin-or.org/Ipopt
%
%
% --------------------------- User Function Requirements ---------------- %
% The objective function must be of the form:
%                   obj = objective(x) OR obj = objective(x,auxdata)
% The constraint function must be of the form:
%                   con = constraints(x) OR con = constraints(x,auxdata)
% Where x corresponds to the NLP decision variable, and auxdata is a fixed
% structure/cell/array.
%
% NOTE: both the objective and constraint functions must be written in
% their own M-file.
%
% If auxdata is an input to the objective, it must also be an input to the
% constraint function, and vice verse. Note, the same auxdata structure
% should be given to both functions. Moreover, if auxdata is an input, then
% the auxdata must be given to adigatorGenFiles4Ipopt.
%
% --------------------------- Input Structure --------------------------- %
%
% setup should be a structure with the following fields:
%
% setup.order:      derivative order - 1 generates objective gradient and
%                   (if constrained) constraint Jacobian, 2 also generates
%                   Lagrangian Hessian
%
% setup.numvar:     length of the NLP variable of differentiation (i.e. x)
%
% setup.objective:  string name of the user's objective function
%
% setup.constraint: (optional) if solving a constrained optimization
%                   problem, then the string name of the user's constraint
%                   function should be included here
%
% setup.options:    (optional) options structure generated by
%                   adigatorOptions
%
% setup.auxdata:    (optional) if the objective/constraint function have an
%                   auxdata input, then the auxdata should be included here
%
%
% --------------------------- Output Structure -------------------------- %
%
% The output is then the funcs structure which goes to the ipopt call.
% 
% funcs.objective: objective function handle
%
% funcs.gradient: objective gradient function handle
%
% funcs.constraints: constraint function handle (if constrained)
%
% funcs.jacobian: constraint Jacobian function handle (if constrained)
%
% funcs.jacobianstructre: constraint Jacobian sparsity pattern function
% handle
%
% funcs.hessian: hessian function handle (if order = 2)
%
% funcs.hessianstructure: hessian sparsity pattern (if order = 2)
%
% Copyright 2011-2014 Matthew J. Weinstein and Anil V. Rao
% Distributed under the GNU General Public License version 3.0
%
% see also adigator, adigatorCreateDerivInput, adigatorOptions,
% adigatorGenJacFile, adigatorGenHesFile, ipopt

% ---------------------------- Parse Inputs ----------------------------- %

if isfield(setup,'order') && (setup.order==1 || setup.order==2)
  order = setup.order;
else
  error('must specify setup.order = 1 or setup.order = 2')
end
if isfield(setup,'numvar') && length(setup.numvar) == 1
  n = setup.numvar;
  if n < 2
    error('adigatorGenFiles4Ipopt not coded for scalar decision variable')
  end
else
  error('must specify decision vector length')
end
if isfield(setup,'objective') && ischar(setup.objective) && ...
    exist(setup.objective,'file')
  ObjFunName = setup.objective;
  ObjFun     = str2func(ObjFunName);
else
  error('must specify objective function')
end
consflag = 0;
if isfield(setup,'constraint') && ~isempty(setup.constraint)
  if ischar(setup.constraint) && exist(setup.constraint,'file')
    ConFunName = setup.constraint;
    ConFun     = str2func(ConFunName);
    consflag   = 1;
  else
    error('invalid constraint function name')
  end
end
if isfield(setup,'options')
  opts = setup.options;
  if ~isfield(opts,'overwrite')
    opts.overwrite = 1;
  end
else
  opts.overwrite = 1;
end
if isfield(setup,'auxdata')
  if nargin(ObjFun) ~= 2 || (consflag == 1 && nargin(ConFun) ~= 2)
    error('if auxdata specified, objective and constraint functions must have 2 inputs')
  end
  auxflag = 1;
  auxdata = setup.auxdata;
else
  if nargin(ObjFun) ~= 1 || (consflag == 1 && nargin(ConFun)~=1)
    error('objective and constraint function should have single input')
  end
  auxflag = 0;
end
if nargout(ObjFun)~=1 || (consflag == 1 && nargout(ConFun)~=1)
  error('objctive and constraint functions should have single output')
end

% --------------------------- Set Up File Names ------------------------- %
GrdFileName    = [ObjFunName,'_Grd'];            % Gradient Wrapper
ObjD1FileName  = [ObjFunName,'_ADiGatorGrd'];    % Obj 1st derivs
if order == 2
  % 2nd derivs
  HesFileName   = [ObjFunName,'_Hes'];           % Hessian Wrapper
  ObjD2FileName = [ObjFunName,'_ADiGatorHes'];   % Obj 2nd derivs
  if consflag
    % constrained
    JacFileName   = [ConFunName,'_Jac'];           % Jacobian Wrapper
    ConD1FileName = [ConFunName,'_ADiGatorJac'];   % Cons 1st derivs
    ConD2FileName = [ConFunName,'_ADiGatorHes'];   % Cons 2nd derivs
    AllFileNames = {GrdFileName, ObjD1FileName, HesFileName, ObjD2FileName,...
      JacFileName, ConD1FileName, ConD2FileName};
  else
    % unconstrained
    AllFileNames = {GrdFileName, ObjD1FileName, HesFileName, ObjD2FileName};
  end
elseif consflag
  % constrained 1st derivs
  JacFileName   = [ConFunName,'_Jac'];           % Jacobian Wrapper
  ConD1FileName = [ConFunName,'_ADiGatorJac'];   % Cons 1st derivs
  AllFileNames = {GrdFileName, ObjD1FileName, JacFileName, ConD1FileName};
else
  % unconstrained 1st derivs
  AllFileNames = {GrdFileName, ObjD1FileName};
end

% ---------------------- Check/Delete All Files ------------------------- %
CallingDir = cd;
for I = 1:length(AllFileNames)
  FileNamei = AllFileNames{I};
  if exist([CallingDir,filesep,FileNamei,'.m'],'file');
    if opts.overwrite
      delete([CallingDir,filesep,FileNamei,'.m']);
      rehash
    else
      error(['The file ',CallingDir,filesep,FileNamei,'.m already exists, ',...
        'quitting transformation. To set manual overwrite of file use ',...
        '''''adigatorOptions(''OVERWRITE'',1);''''. Alternatively, delete the ',...
        'existing file and any associated .mat file.']);
    end
  end
end

% ------------------------ Differentiate Objective File ----------------- %
x = adigatorCreateDerivInput([n 1],'x');
if auxflag
  Inputs = {x, auxdata};
else
  Inputs = {x};
end
objout1 = adigator(ObjFunName,Inputs,ObjD1FileName,opts);
objout1 = objout1{1};
if prod(objout1.func.size) ~= 1
  error('objective appears to be non-scalar')
end
if order == 2
  % Second deriv of objective
  x2 = struct('f',x,'dx',ones(n,1));
  Inputs2 = Inputs;  Inputs2{1} = x2;
  objout2 = adigator(ObjD1FileName,Inputs2,ObjD2FileName,opts);
  objout2 = objout2{1};
end

% ------------------------ Differentiate Constraint File ---------------- %
if consflag
  [conout1,ConFunInfo] = adigator(ConFunName,Inputs,ConD1FileName,opts);
  conout1 = conout1{1};
  conD1 = conout1.deriv.nzlocs;
  if order == 2
    conout2 = adigator(ConD1FileName,Inputs2,ConD2FileName,opts);
    conout2 = conout2{1};
  end
end

% -------------------------- Create Necessary Files --------------------- %
if auxflag
  InVarStr  = 'x,auxdata';
  dInVarStr = 'gx,auxdata';
else
  InVarStr  = 'x';
  dInVarStr = 'gx';
end
Gfid = fopen([GrdFileName,'.m'],'w+');
fprintf(Gfid,['function Grd = ',GrdFileName,'(',InVarStr,')\n']);
if order == 2
  Hfid = fopen([HesFileName,'.m'],'w+');
  fprintf(Hfid,['function Hes = ',HesFileName,'(',InVarStr,',sigma,lambda)\n']);
end
if consflag
  Jfid = fopen([JacFileName,'.m'],'w+');
  fprintf(Jfid,['function Jac = ',JacFileName,'(',InVarStr,')\n']);
  if order == 2
    allFid = [Gfid Jfid Hfid];
  else
    allFid = [Gfid Jfid];
  end
elseif order == 2
  allFid = [Gfid Hfid];
else
  allFid = Gfid;
end

% -------------------------- Print Common File Headers ------------------ %
for fid = allFid
  % Print Function Header
  fprintf(fid,'%% \n');
  fprintf(fid,'%% Wrapper file generated by ADiGator\n');
  fprintf(fid,['%% ',char(169),'2010-2014 Matthew J. Weinstein and Anil V. Rao\n']);
  fprintf(fid,'%% ADiGator may be obtained at https://sourceforge.net/projects/adigator/ \n');
  fprintf(fid,'%% Contact: mweinstein@ufl.edu\n');
  fprintf(fid,'%% Bugs/suggestions may be reported to the sourceforge forums\n');
  fprintf(fid,'%%                    DISCLAIMER\n');
  fprintf(fid,'%% ADiGator is a general-purpose software distributed under the GNU General\n');
  fprintf(fid,'%% Public License version 3.0. While the software is distributed with the\n');
  fprintf(fid,'%% hope that it will be useful, both the software and generated code are\n');
  fprintf(fid,'%% provided ''AS IS'' with NO WARRANTIES OF ANY KIND and no merchantability\n');
  fprintf(fid,'%% or fitness for any purpose or application.\n\n');
  
  % Change Derivative Inputs
  fprintf(fid,'gx.f = x;\n');
  fprintf(fid,'gx.dx = ones(%1.0d,1);\n',n);
end

% --------------------------- Print Gradient File ----------------------- %
fprintf(Gfid,['obj = ',ObjD1FileName,'(',dInVarStr,');\n']);
% Gradient should be a row vector - check number of non-zeros in gradient
% to see if need to project or simply reshape.
objD1 = objout1.deriv.nzlocs;
Gnnz  = size(objD1,1);
if Gnnz == n
  % Gradient is completely full
  fprintf(Gfid,'Grd = reshape(obj.dx,1,%1.0d);',n);
else
  % Gradient has some known zeros
  fprintf(Gfid,'Grd = zeros(1,%1.0d);\n',n);
  fprintf(Gfid,'Grd(obj.dx_location) = obj.dx;\n');
end

% -------------------------- Print Jacobian File ------------------------ %
if consflag
  % let m = number of cons.
  m = prod(conout1.func.size);
  fprintf(Jfid,['con = ',ConD1FileName,'(',dInVarStr,');\n']);
  if m > 1
    fprintf(Jfid,['Jac = sparse(con.dx_location(:,1),con.dx_location(:,2),',...
      'con.dx,%1.0d,%1.0d);\n'],m,n);
  else
    fprintf(Jfid,['Jac = sparse(ones(%1.0f,1),con.dx_location(:,1),',...
      'con.dx,%1.0d,%1.0d);\n'],size(conD1,1),m,n);
  end
else
  m = 0;
end

% --------------------------- Print Hessian File ------------------------ %
if order == 2
  if consflag
    conD2 = conout2.dx.deriv.nzlocs;
    compressHes = 0;
    if m > 1 && ~isempty(conD2)
      iCxx = conD1(conD2(:,1),1);
      jCxx = conD1(conD2(:,1),2);
      kCxx = conD2(:,2);
      jkCxx = sub2ind([n n],jCxx,kCxx);
      jkCxxc = unique(jkCxx);
      if length(jkCxxc) <= 3/4*n*n
        compressHes = 1;
        fprintf(Hfid,['global ADiGator_',ConD2FileName,'\n']);
      end
    end
  end
  
  objD1 = objout2.f.deriv.nzlocs;
  objD2 = objout2.dx.deriv.nzlocs;
  fprintf(Hfid,['obj = ',ObjD2FileName,'(',dInVarStr,');\n']);
  if ~isempty(objD2)
    fprintf(Hfid,['objHes = sparse(obj.dxdx_location(:,1),obj.dxdx_location(:,2),',...
      'sigma*obj.dxdx,%1.0d,%1.0d);\n'],n,n);
  else
    fprintf(Hfid,'objHes = sparse([],[],[],%1.0d,%1.0d);\n',n,n);
  end
  if consflag
    conD2 = conout2.dx.deriv.nzlocs;
    
    if isempty(conD2)
      printf(Hfid,'conHes = sparse([],[],[],%1.0d,%1.0d);\n',n,n);
    else
      fprintf(Hfid,['con = ',ConD2FileName,'(',dInVarStr,');\n']);
      % Project constraint 2nd derivs into m x n*n matrix to multiply through
      % by lambda
      if compressHes
        % Want to use compression on hessian when multiplying by lambda..
        indMap = zeros(1,n*n);
        indMap(jkCxxc) = 1:length(jkCxxc);
        jkCxx_comp = indMap(jkCxx);
        fprintf(Hfid,['HesData1 = ADiGator_',ConD2FileName,'.',ConD2FileName,'.Gator2Data.HesData1;\n']);
        fprintf(Hfid,'conHesnz = lambda.''*sparse(con.dxdx_location(:,1),HesData1,con.dxdx,%1.0d,%1.0d);\n',m,length(jkCxxc));
        % conHesnz is the non-zeros of the hessian of lambda.'*dcdc
        ijHxx = unique([jCxx,kCxx],'rows');
        fprintf(Hfid,['HesData2 = ADiGator_',ConD2FileName,'.',ConD2FileName,'.Gator2Data.HesData2;\n']);
        fprintf(Hfid,'conHes = sparse(HesData2(:,1),HesData2(:,2),conHesnz,%1.0d,%1.0d);\n',n,n);
        setGlobalHesData(ConD2FileName,jkCxx_comp,ijHxx);
      elseif m > 1
        fprintf(Hfid,'conHesLocs = (con.dxdx_location(:,3)-1)*%1.0d + con.dxdx_location(:,2);\n',n);
        fprintf(Hfid,'conHesReshaped = sparse(con.dxdx_location(:,1),conHesLocs,con.dxdx,%1.0d,%1.0d);\n',m,n*n);
        fprintf(Hfid,'conHes = reshape(sparse(lambda).''*conHesReshaped,%1.0f,%1.0f);\n',n,n);
      else
        fprintf(Hfid,['conHes = sparse(con.dxdx_location(:,1),con.dxdx_location(:,2),',...
          'lambda*con.dxdx,%1.0d,%1.0d);\n']);
      end
    end
    % IPOPT likes lower triangular - take average of it as well
    fprintf(Hfid,'Hes = objHes + conHes;\n');
    fprintf(Hfid,'Hes = tril((Hes+Hes.''))/2;\n');
  else
    % IPOPT likes lower triangular - take average of it as well
    fprintf(Hfid,'Hes = tril((objHes+objHes.''))/2;\n');
  end
end

% --------------------------- Close All Files --------------------------- %
for fid = allFid
  fclose(fid);
  rehash
end

% -------------------------- Create Function Calls ---------------------- %
if auxflag
  funcs.objective = eval(['@(x)',ObjFunName,'(x,auxdata)']);
  funcs.gradient  = eval(['@(x)',GrdFileName,'(x,auxdata)']);
  if consflag
    funcs.constraints = eval(['@(x)',ConFunName,'(x,auxdata)']);
    funcs.jacobian    = eval(['@(x)',JacFileName,'(x,auxdata)']);
  end
  if order == 2
    funcs.hessian = eval(['@(x,sigma,lambda)',HesFileName,'(x,auxdata,sigma,lambda)']);
  end
else
  funcs.objective = str2func(ObjFunName);
  funcs.gradient  = str2func(GrdFileName);
  if consflag
    funcs.constraints = str2func(ConFunName);
    funcs.jacobian    = str2func(JacFileName);
  end
  if order == 2
    funcs.hessian = str2func(HesFileName);
  end
end

% -------------------- Determine Sparsity Patterns ---------------------- %
if consflag
  % Jacobian Sparsity Pattern
  conD1  = conout1.deriv.nzlocs; conD1nnz = size(conD1,1);
  JacPat = sparse(conD1(:,1),conD1(:,2),ones(conD1nnz,1),m,n);
  funcs.jacobianstructure = @()JacPat;
end
if order == 2
  if ~isempty(objD2)
    objHesRows = objD1(objD2(:,1),2);
    objHesCols = objD2(:,2);
    objHesnnz  = length(objHesRows);
    objHesPat  = sparse(objHesRows,objHesCols,ones(objHesnnz,1),n,n);
  else
    objHesPat  = sparse([],[],[],n,n);
  end
  if consflag
    conD2 = conout2.dx.deriv.nzlocs; conD2nnz = size(conD2,1);
    if ~isempty(conD2)
      conHesLocs = [conD1(conD2(:,1),:) conD2(:,2)];
      conHesReshapeRows = conHesLocs(:,1);
      conHesReshapeCols = (conHesLocs(:,3)-1)*n + conHesLocs(:,2);
      conHesReshape     = sparse(conHesReshapeRows,conHesReshapeCols,ones(conD2nnz,1),m,n*n);
      conHes            = sparse(ones(m,1),1:m,ones(m,1),1,m)*conHesReshape;
      conHesPat         = reshape(conHes,n,n);
      HesPat = spones(tril(conHesPat + objHesPat));
    else
      HesPat = tril(objHesPat);
    end
  else
    HesPat = tril(objHesPat);
  end
  funcs.hessianstructure = @()HesPat;
end


fprintf(['\n<strong>adigatorGenFiles4Ipopt</strong> successfully generated IPOPT wrapper files;\n\n']);
end
function setGlobalHesData(ConD2FileName,HesData1,HesData2) %#ok<INUSD>
eval(['global ADiGator_',ConD2FileName]);
eval(['ADiGator_',ConD2FileName,'.',ConD2FileName,'.Gator2Data.HesData1 = HesData1;']);
eval(['ADiGator_',ConD2FileName,'.',ConD2FileName,'.Gator2Data.HesData2 = HesData2;']);
end