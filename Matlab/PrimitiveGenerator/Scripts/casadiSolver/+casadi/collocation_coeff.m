function varargout = collocation_coeff(varargin)
    %COLLOCATION_COEFF Obtain collocation interpolating matrices.
    %
    %  [DM OUTPUT, DM OUTPUT, DM OUTPUT] = COLLOCATION_COEFF([double] tau)
    %
    %
    %A collocation method poses a polynomial Pi that interpolates exactly through
    %an initial state (0,X_0) and helper states at collocation points
    %(tau_j,Xc_j) with j=1..degree.
    %
    %This function computes the linear mapping between dPi/dt and coefficients
    %Z=[X_0 Xc].
    %
    %Parameters:
    %-----------
    %
    %tau:  location of collocation points (length: degree), as obtained from
    %collocation_points
    %
    %C:  interpolating coefficients to obtain derivatives. Size: (degree+1)-by-
    %degree
    %
    %You may find the slopes of Pi at the collocation points as
    %
    %::
    %
    %dPi/dt @ Xc = (1/h) Z*C,
    %
    %
    %
    %with h the length of the integration interval.
    %
    %Parameters:
    %-----------
    %
    %D:  interpolating coefficients to obtain end state. Size: (degree+1)-by-1
    %
    %You may find the end point of Pi as
    %
    %::
    %
    %Pi @X_f = Z*D
    %
    %
    %
    %Parameters:
    %-----------
    %
    %B:  quadrature coefficients Size: degree-by-1
    %
    %Given quadrature righ-hand-sides 'quad' evaluated at the collocation points,
    %you may find the integrated quadratures as
    %
    %::
    %
    %q = quad*B*h
    %
    %
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(942, varargin{:});
end
