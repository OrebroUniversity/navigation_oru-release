function varargout = conic_in(varargin)
    %CONIC_IN Get QP solver input scheme name by index.
    %
    %  {char} = CONIC_IN()
    %  char = CONIC_IN(int ind)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  CONIC_IN()
    %
    %
    %
    %Get input scheme of QP solvers.
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONIC_IN(int ind)
    %
    %
    %
    %Get QP solver input scheme name by index.
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(790, varargin{:});
end
