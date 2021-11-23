function varargout = collocation_points(varargin)
    %COLLOCATION_POINTS Obtain collocation points of specific order and scheme.
    %
    %  [double] = COLLOCATION_POINTS(int order, char scheme)
    %
    %
    %Parameters:
    %-----------
    %
    %order:  Which order (1 to 9 supported)
    %
    %scheme:  'radau' or 'legendre'
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(940, varargin{:});
end
