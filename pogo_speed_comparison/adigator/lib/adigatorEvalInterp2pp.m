function ZI = adigatorEvalInterp2pp(pp,XI,YI)
% function ZI = adigatorEvalInterp2pp(pp,XI,YI)
% NON-OVERLOADED VERSION
% This function is used to evaluate the 2-D piecewise polynomials generated
% using the adigatorGenInterp2pp function. Since MATLAB does not have a
% built in 2-D polynomial evaluation command, this file will need to be in
% the user's path if they create a derivative file by using the adigator
% command on a function file which contains an interp2 command. The inputs
% to this are the pp generated by pp = adigatorGenInterp2pp(X,Y,Z,method),
% as well as the inputs XI, YI. 
%
% The following two lines of code:
%
%     pp = adigatorGenInterp2pp(X,Y,Z,method);
%     ZI = adigatorEvalInterp2pp(pp,XI,YI);
%
% Are (roughly) equivalent to the single line of code:
%
%     ZI = interp2(X,Y,Z,XI,YI,method,'extrap');
%
% Where the difference is due to rounding errors since when using the 
% adigatorGenInterp2pp and adigatorEvalInterp2pp commands, the 2-D
% coefficients are calculated, whereas MATLAB interp2 essentially performs
% interp1 twice, thus generating 1-D coefficients twice.
%
% Copyright 2011-214 Matthew J. Weinstein and Anil V. Rao
% Distributed under the GNU General Public License version 3.0
%
% See also interp2, interp1, ppval, adigatorGenInterp2pp


% -------------------------- Get pp Data -------------------------------- %
xorder = pp.xorder;
yorder = pp.yorder;
X = pp.xbreaks;
Y = pp.ybreaks.';
D = pp.coefs;
% Info on D:
% 1st index corresponds to y location
% 2nd index corresponds to x location
% 3rd index corresponds to y order
% 4th index corresponds to x order

M = length(Y);
N = length(X);

% Find where XI and YI lie
[~,xindex] = histc(XI,[-inf,X(2:N-1),inf]);
[~,yindex] = histc(YI,[-inf;Y(2:M-1);inf]);

if size(XI) == size(YI)
  % ZI will be the size of XI and YI - will need to use a linear index for
  % this
  [ZIrow,ZIcol] = size(XI);
  
  % Switch to Linear Indexing on first to dimensions of D
  %D = reshape(D,[(M-1)*(N-1),yorder,xorder]);
  D = cellfun(@(Di)reshape(Di,[(M-1)*(N-1),1]),D,'UniformOutput',0);
  xyindex = sub2ind([M-1,N-1],yindex(:),xindex(:));
  
  % Switch to XI-X, YI-Y
  XD = XI(:) - X(xindex(:)).';
  YD = YI(:) - Y(yindex(:));
  
  % Build ZI
  ZI = zeros(ZIrow*ZIcol,1);
  for I=1:yorder
    for J=1:xorder
      ZI = ZI + D{I,J}(xyindex).*YD.^(yorder-I).*XD.^(xorder-J);
      %ZI = ZI + D(xyindex,I,J).*YD.^(yorder-I).*XD.^(xorder-J);
    end
  end
  ZI = reshape(ZI,[ZIrow, ZIcol]);
elseif (size(XI,1) == 1 && size(YI,2) == 1) || ...
    (size(XI,2) == 1 && size(YI,1) == 1)
  % ZI will have row dimension of length of YI and column dimension of
  % length of XI
  ZIrow = length(YI); ZIcol = length(XI);
  
  % Switch to XI-X, YI-Y
  XD = XI(:) - X(xindex(:)).';
  YD = YI(:) - Y(yindex(:));
  
  XD = repmat(XD.',[ZIrow 1]);
  YD = repmat(YD,[1 ZIcol]);
  
  ZI = zeros(ZIrow,ZIcol);
  for I = 1:yorder
    for J = 1:xorder
      ZI = ZI + D{I,J}(yindex,xindex).*YD.^(yorder-I).*XD.^(xorder-J);
      %ZI = ZI + D(yindex,xindex,I,J).*YD.^(yorder-I).*XD.^(xorder-J);
    end
  end
else
  error('XI and YI must be the same size or vectors of different orientations.')
end