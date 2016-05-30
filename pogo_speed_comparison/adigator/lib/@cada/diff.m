function R = diff(S,varargin)
%DIFF   Differentiate.
%   DIFF(S) differentiates a symbolic expression S with respect to its
%   free variable as determined by SYMVAR.
%   DIFF(S,'v') or DIFF(S,sym('v')) differentiates S with respect to v.
%   DIFF(S,n), for a positive integer n, differentiates S n times.
%   DIFF(S,'v',n) and DIFF(S,n,'v') are also acceptable.
%   DIFF(S,'v1','v2',...) or DIFF(S,sym('v1'),sym('v2'),...) differentiates 
%   S with respect to v1, v2, ...                                            
%
%   Examples;
%      syms x t
%      diff(sin(x^2)) is 2*x*cos(x^2)
%      diff(t^6,6) is 720.
%      diff(sin(x^2+t),x,t,t) is -2*x*cos(x^2 + t)
%
%   See also SYM/INT, SYM/JACOBIAN, SYM/SYMVAR.

%   Copyright 1993-2012 The MathWorks, Inc.


argout = varargin;
n = nargin-1;
for k=1:n
    arg = varargin{k};
    if isa(arg,'sym') && builtin('numel',arg) ~= 1
        argout{k} = normalizesym(arg);
    elseif ~isa(arg,'sym')
        argout{k} = sym(arg);
    end
end
args=argout;
args{1}=S;
S = args{1};


for k = 1:nargin-1 
    if isa(varargin{k},'symfun')
        error(message('symbolic:sym:diff:NoSYMFUNs')) 
    end
end

n = sym([]);

if nargin >= 3  
   v1 = args{2};
   v2 = args{3};
   if isempty(symvar(v1)) && isempty(symvar(v2)) 
       error(message('symbolic:sym:diff:OrderOrVariable'))    
   end
   a = makeInteger(v1);
   if ~isnan(a)
       n = a;
   else
       a = makeInteger(v2);
       if ~isnan(a)
           n = a;
       end    
   end
elseif nargin == 2 
   a = makeInteger(args{2});
   if ~isnan(a)
       n = a;
   end
end

if nargin > 3 && ~isempty(n)
    error(message('symbolic:sym:diff:OrderNotSupported'));
end

vars = cell(1,nargin-1);

for j = 2:nargin 
    a = args{j};
    isIdent = logical(feval(symengine, 'testtype', a, 'Type::Indeterminate'));
    if j > 3 && ~isIdent
        error(message('symbolic:sym:diff:ArgumentsMustBeVariables'));
    elseif j <= 3 && ~isIdent 
        a = makeInteger(a);
        if isnan(a)            
            if nargin == 2
                error(message('symbolic:sym:diff:OrderOrVariableAsSecondArgument'));
            elseif nargin == 3 
                error(message('symbolic:sym:diff:OrderOrVariable'));
            else
                error(message('symbolic:sym:diff:ArgumentsMustBeVariables'));
            end
        end    
    end
    if isIdent
        vars{j-1} = a; 
    else 
        vars{j-1} = sym(0);
    end
end

x = sym(vars);

if isempty(n) 
    n = sym(1);
end

if isempty(symvar(x)) 
    x = symvar(S);
end

if n == 0
    R = S;
elseif isempty(x)
    R = 0*S;
else
    R = mupadmex('symobj::diff', S.s, x.s, n.s);
end

if isa(S,'symfun')
    oldR = R;
    R = symfun(oldR, argnames(S));
end


% local function makeInteger(a)
% converts a into a nonnegative integer (DOM_INT) 
% or returns NaN if no conversion is possible
function N = makeInteger(a)
N = feval(symengine, 'specfunc::makeInteger', a);
if ~feval(symengine, 'testtype', N, 'Type::NonNegInt')
    N = NaN;
end    

    
