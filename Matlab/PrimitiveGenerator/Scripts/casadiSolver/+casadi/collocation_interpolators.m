function varargout = collocation_interpolators(varargin)
    %COLLOCATION_INTERPOLATORS Obtain collocation interpolating matrices.
    %
    %  [{[double]} OUTPUT, [double] OUTPUT] = COLLOCATION_INTERPOLATORS([double] tau)
    %
    %
    %A collocation method poses a polynomial Pi that interpolates exactly through
    %an initial state (0,X_0) and helper states at collocation points
    %(tau_j,X(j)).
    %
    %This function computes the linear mapping between dPi/dt and coefficients
    %Z=[X_0 X].
    %
    %Parameters:
    %-----------
    %
    %tau:  location of collocation points, as obtained from collocation_points
    %
    %output_C:  interpolating coefficients to obtain derivatives. Length:
    %order+1, order+1
    %
    %
    %
    %::
    %
    %dPi/dt @Z_j = (1/h) Sum_i C[j][i]*Z_i,
    %
    %
    %
    %with h the length of the integration interval.
    %
    %Parameters:
    %-----------
    %
    %output_D:  interpolating coefficients to obtain end state. Length: order+1
    %
    %
    %
    %::
    %
    %Pi @X_f = Sum_i D[i]*Z_i
    %
    %
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(941, varargin{:});
end
