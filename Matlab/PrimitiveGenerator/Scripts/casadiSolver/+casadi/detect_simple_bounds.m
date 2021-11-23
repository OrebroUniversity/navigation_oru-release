function varargout = detect_simple_bounds(varargin)
    %DETECT_SIMPLE_BOUNDS Detect simple bounds from general constraints.
    %
    %  [[int] OUTPUT, SX OUTPUT, SX OUTPUT, Function OUTPUT, Function OUTPUT] = DETECT_SIMPLE_BOUNDS(SX xX, SX p, SX g, SX lbg, SX ubg)
    %  [[int] OUTPUT, MX OUTPUT, MX OUTPUT, Function OUTPUT, Function OUTPUT] = DETECT_SIMPLE_BOUNDS(MX xX, MX p, MX g, MX lbg, MX ubg)
    %
    %
    %Given parametric constraints:
    %
    %::
    %
    %  *   subject to lbg(p) <= g(x,p) <= ubg(p)
    %  * 
    %
    %
    %
    %Returns an equivalent set
    %
    %::
    %
    %  *   subject to  lbg(p)(gi) <= g(x,p)(gi) <= ubg(p)(gi)
    %  *               lbx(p) <= x                 <= ubx(p)
    %  * 
    %
    %
    %
    %Parameters:
    %-----------
    %
    %lam_forward:  (lam_g,p)->(lam_sg,lam_x)
    %
    %lam_backward:  (lam_sg,lam_x,p)->(lam_g)
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(946, varargin{:});
end
