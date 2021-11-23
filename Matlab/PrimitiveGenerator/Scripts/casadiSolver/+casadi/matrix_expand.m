function varargout = matrix_expand(varargin)
    %MATRIX_EXPAND Expand MX graph to SXFunction call.
    %
    %  MX = MATRIX_EXPAND(MX e, {MX} boundary, struct options)
    %  {MX} = MATRIX_EXPAND({MX} e, {MX} boundary, struct options)
    %
    %
    %Expand the given expression e, optionally supplying expressions contained in
    %it at which expansion should stop.
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(871, varargin{:});
end
