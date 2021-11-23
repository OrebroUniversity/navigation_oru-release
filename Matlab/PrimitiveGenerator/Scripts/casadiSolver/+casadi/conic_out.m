function varargout = conic_out(varargin)
    %CONIC_OUT Get output scheme name by index.
    %
    %  {char} = CONIC_OUT()
    %  char = CONIC_OUT(int ind)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  CONIC_OUT()
    %
    %
    %
    %Get QP solver output scheme of QP solvers.
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
    %  CONIC_OUT(int ind)
    %
    %
    %
    %Get output scheme name by index.
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(791, varargin{:});
end
