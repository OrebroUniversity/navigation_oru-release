function varargout = conic_debug(varargin)
    %CONIC_DEBUG Generate native code in the interfaced language for debugging
    %
    %  CONIC_DEBUG(Function f, std::ostream & file)
    %  CONIC_DEBUG(Function f, char filename)
    %
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(800, varargin{:});
end
