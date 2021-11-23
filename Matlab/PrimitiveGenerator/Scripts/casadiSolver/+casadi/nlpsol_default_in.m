function varargout = nlpsol_default_in(varargin)
    %NLPSOL_DEFAULT_IN Default input for an NLP solver.
    %
    %  [double] = NLPSOL_DEFAULT_IN()
    %  double = NLPSOL_DEFAULT_IN(int ind)
    %
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(806, varargin{:});
end
