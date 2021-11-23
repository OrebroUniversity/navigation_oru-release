function varargout = integrator_out(varargin)
    %INTEGRATOR_OUT Get output scheme name by index.
    %
    %  {char} = INTEGRATOR_OUT()
    %  char = INTEGRATOR_OUT(int ind)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  INTEGRATOR_OUT()
    %
    %
    %
    %Get integrator output scheme of integrators.
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
    %  INTEGRATOR_OUT(int ind)
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
  [varargout{1:nargout}] = casadiMEX(785, varargin{:});
end
