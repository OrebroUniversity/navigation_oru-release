function varargout = simpleRK(varargin)
    %SIMPLERK Construct an explicit Runge-Kutta integrator The constructed function has
    %
    %  Function = SIMPLERK(Function f, int N, int order)
    %
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
    %
    %
  [varargout{1:nargout}] = casadiMEX(943, varargin{:});
end
