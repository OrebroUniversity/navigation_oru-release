function varargout = simpleIntegrator(varargin)
    %SIMPLEINTEGRATOR Simplified wrapper for the Integrator class Constructs an integrator using
    %
    %  Function = SIMPLEINTEGRATOR(Function f, char integrator, struct integrator_options)
    %
    %the same syntax as simpleRK and simpleIRK. The constructed function has
    %three inputs, corresponding to initial state (x0), parameter (p) and
    %integration time (h) and one output, corresponding to final state (xf).
    %
    %Parameters:
    %-----------
    %
    %f:  ODE function with two inputs (x and p) and one output (xdot)
    %
    %N:  Number of integrator steps
    %
    %order:  Order of interpolating polynomials
    %
    %scheme:   Collocation scheme, as excepted by collocationPoints function.
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(945, varargin{:});
end
