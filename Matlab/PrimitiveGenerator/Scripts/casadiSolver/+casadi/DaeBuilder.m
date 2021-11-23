classdef  DaeBuilder < casadi.PrintableCommon
    %DAEBUILDER An initial-value problem in differential-algebraic equations.
    %
    %
    %
    %Independent variables:
    %======================
    %
    %
    %
    %
    %
    %::
    %
    %  t:      time
    %  
    %
    %
    %
    %Time-continuous variables:
    %==========================
    %
    %
    %
    %
    %
    %::
    %
    %  x:      states defined by ODE
    %  s:      implicitly defined states
    %  z:      algebraic variables
    %  u:      control signals
    %  q:      quadrature states
    %  y:      outputs
    %  
    %
    %
    %
    %Time-constant variables:
    %========================
    %
    %
    %
    %
    %
    %::
    %
    %  p:      free parameters
    %  d:      dependent parameters
    %  
    %
    %
    %
    %Dynamic constraints (imposed everywhere):
    %=========================================
    %
    %
    %
    %
    %
    %::
    %
    %  ODE                    \\dot{x} ==  ode(t, x, s, z, u, p, d)
    %  DAE or implicit ODE:         0 ==  dae(t, x, s, z, u, p, d, sdot)
    %  algebraic equations:         0 ==  alg(t, x, s, z, u, p, d)
    %  quadrature equations:  \\dot{q} == quad(t, x, s, z, u, p, d)
    %  dependent parameters:        d == ddef(t, x, s, z, u, p, d)
    %  output equations:            y == ydef(t, x, s, z, u, p, d)
    %  
    %
    %
    %
    %Point constraints (imposed pointwise):
    %======================================
    %
    %
    %
    %
    %
    %::
    %
    %  Initial equations:           0 == init(t, x, s, z, u, p, d, sdot)
    %  
    %
    %
    %
    %Joel Andersson
    %
    %C++ includes: dae_builder.hpp 
    %
  methods
    function v = t(self)
      v = casadiMEX(986, self);
    end
    function v = x(self)
      v = casadiMEX(987, self);
    end
    function v = ode(self)
      v = casadiMEX(988, self);
    end
    function v = lam_ode(self)
      v = casadiMEX(989, self);
    end
    function v = s(self)
      v = casadiMEX(990, self);
    end
    function v = sdot(self)
      v = casadiMEX(991, self);
    end
    function v = dae(self)
      v = casadiMEX(992, self);
    end
    function v = lam_dae(self)
      v = casadiMEX(993, self);
    end
    function v = z(self)
      v = casadiMEX(994, self);
    end
    function v = alg(self)
      v = casadiMEX(995, self);
    end
    function v = lam_alg(self)
      v = casadiMEX(996, self);
    end
    function v = q(self)
      v = casadiMEX(997, self);
    end
    function v = quad(self)
      v = casadiMEX(998, self);
    end
    function v = lam_quad(self)
      v = casadiMEX(999, self);
    end
    function v = w(self)
      v = casadiMEX(1000, self);
    end
    function v = wdef(self)
      v = casadiMEX(1001, self);
    end
    function v = lam_wdef(self)
      v = casadiMEX(1002, self);
    end
    function v = y(self)
      v = casadiMEX(1003, self);
    end
    function v = ydef(self)
      v = casadiMEX(1004, self);
    end
    function v = lam_ydef(self)
      v = casadiMEX(1005, self);
    end
    function v = u(self)
      v = casadiMEX(1006, self);
    end
    function v = p(self)
      v = casadiMEX(1007, self);
    end
    function v = c(self)
      v = casadiMEX(1008, self);
    end
    function v = cdef(self)
      v = casadiMEX(1009, self);
    end
    function v = d(self)
      v = casadiMEX(1010, self);
    end
    function v = ddef(self)
      v = casadiMEX(1011, self);
    end
    function v = lam_ddef(self)
      v = casadiMEX(1012, self);
    end
    function v = aux(self)
      v = casadiMEX(1013, self);
    end
    function v = init(self)
      v = casadiMEX(1014, self);
    end
    function varargout = add_p(self,varargin)
    %ADD_P Add a new parameter
    %
    %  MX = ADD_P(self, char name, int n)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1015, self, varargin{:});
    end
    function varargout = add_u(self,varargin)
    %ADD_U Add a new control.
    %
    %  MX = ADD_U(self, char name, int n)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1016, self, varargin{:});
    end
    function varargout = add_x(self,varargin)
    %ADD_X Add a new differential state.
    %
    %  MX = ADD_X(self, char name, int n)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1017, self, varargin{:});
    end
    function varargout = add_s(self,varargin)
    %ADD_S Add a implicit state.
    %
    %  (MX,MX) = ADD_S(self, char name, int n)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1018, self, varargin{:});
    end
    function varargout = add_z(self,varargin)
    %ADD_Z Add a new algebraic variable.
    %
    %  MX = ADD_Z(self, char name, int n)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1019, self, varargin{:});
    end
    function varargout = add_q(self,varargin)
    %ADD_Q Add a new quadrature state.
    %
    %  MX = ADD_Q(self, char name, int n)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1020, self, varargin{:});
    end
    function varargout = add_d(self,varargin)
    %ADD_D Add a new dependent parameter.
    %
    %  MX = ADD_D(self, char name, MX new_ddef)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1021, self, varargin{:});
    end
    function varargout = add_y(self,varargin)
    %ADD_Y Add a new output.
    %
    %  MX = ADD_Y(self, char name, MX new_ydef)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1022, self, varargin{:});
    end
    function varargout = add_ode(self,varargin)
    %ADD_ODE Add an ordinary differential equation.
    %
    %  ADD_ODE(self, char name, MX new_ode)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1023, self, varargin{:});
    end
    function varargout = add_dae(self,varargin)
    %ADD_DAE Add a differential-algebraic equation.
    %
    %  ADD_DAE(self, char name, MX new_dae)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1024, self, varargin{:});
    end
    function varargout = add_alg(self,varargin)
    %ADD_ALG Add an algebraic equation.
    %
    %  ADD_ALG(self, char name, MX new_alg)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1025, self, varargin{:});
    end
    function varargout = add_quad(self,varargin)
    %ADD_QUAD Add a quadrature equation.
    %
    %  ADD_QUAD(self, char name, MX new_quad)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1026, self, varargin{:});
    end
    function varargout = add_aux(self,varargin)
    %ADD_AUX Add an auxiliary variable.
    %
    %  MX = ADD_AUX(self, char name, int n)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1027, self, varargin{:});
    end
    function varargout = sanity_check(self,varargin)
    %SANITY_CHECK Check if dimensions match.
    %
    %  SANITY_CHECK(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1028, self, varargin{:});
    end
    function varargout = split_dae(self,varargin)
    %SPLIT_DAE Identify and separate the algebraic variables and equations in the DAE.
    %
    %  SPLIT_DAE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1029, self, varargin{:});
    end
    function varargout = eliminate_alg(self,varargin)
    %ELIMINATE_ALG Eliminate algebraic variables and equations transforming them into outputs.
    %
    %  ELIMINATE_ALG(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1030, self, varargin{:});
    end
    function varargout = make_semi_explicit(self,varargin)
    %MAKE_SEMI_EXPLICIT Transform the implicit DAE to a semi-explicit DAE.
    %
    %  MAKE_SEMI_EXPLICIT(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1031, self, varargin{:});
    end
    function varargout = make_explicit(self,varargin)
    %MAKE_EXPLICIT Transform the implicit DAE or semi-explicit DAE into an explicit ODE.
    %
    %  MAKE_EXPLICIT(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1032, self, varargin{:});
    end
    function varargout = sort_d(self,varargin)
    %SORT_D Sort dependent parameters.
    %
    %  SORT_D(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1033, self, varargin{:});
    end
    function varargout = split_d(self,varargin)
    %SPLIT_D Eliminate interdependencies amongst dependent parameters.
    %
    %  SPLIT_D(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1034, self, varargin{:});
    end
    function varargout = eliminate_d(self,varargin)
    %ELIMINATE_D Eliminate dependent parameters.
    %
    %  ELIMINATE_D(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1035, self, varargin{:});
    end
    function varargout = eliminate_quad(self,varargin)
    %ELIMINATE_QUAD Eliminate quadrature states and turn them into ODE states.
    %
    %  ELIMINATE_QUAD(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1036, self, varargin{:});
    end
    function varargout = sort_dae(self,varargin)
    %SORT_DAE Sort the DAE and implicitly defined states.
    %
    %  SORT_DAE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1037, self, varargin{:});
    end
    function varargout = sort_alg(self,varargin)
    %SORT_ALG Sort the algebraic equations and algebraic states.
    %
    %  SORT_ALG(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1038, self, varargin{:});
    end
    function varargout = scale_variables(self,varargin)
    %SCALE_VARIABLES Scale the variables.
    %
    %  SCALE_VARIABLES(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1039, self, varargin{:});
    end
    function varargout = scale_equations(self,varargin)
    %SCALE_EQUATIONS Scale the implicit equations.
    %
    %  SCALE_EQUATIONS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1040, self, varargin{:});
    end
    function varargout = add_fun(self,varargin)
    %ADD_FUN Add an external function.
    %
    %  Function = ADD_FUN(self, Function f)
    %  Function = ADD_FUN(self, char name, Importer compiler, struct opts)
    %  Function = ADD_FUN(self, char name, {char} arg, {char} res, struct opts)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  ADD_FUN(self, Function f)
    %
    %
    %
    %Add an already existing function.
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
    %  ADD_FUN(self, char name, Importer compiler, struct opts)
    %
    %
    %
    %Add an external function.
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
    %  ADD_FUN(self, char name, {char} arg, {char} res, struct opts)
    %
    %
    %
    %Add a function from loaded expressions.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1041, self, varargin{:});
    end
    function varargout = has_fun(self,varargin)
    %HAS_FUN Does a particular function already exist?
    %
    %  bool = HAS_FUN(self, char name)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1042, self, varargin{:});
    end
    function varargout = fun(self,varargin)
    %FUN Get function by name.
    %
    %  Function = FUN(self, char name)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1043, self, varargin{:});
    end
    function varargout = parse_fmi(self,varargin)
    %PARSE_FMI Import existing problem from FMI/XML
    %
    %  PARSE_FMI(self, char filename)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1044, self, varargin{:});
    end
    function varargout = add_lc(self,varargin)
    %ADD_LC Add a named linear combination of output expressions.
    %
    %  MX = ADD_LC(self, char name, {char} f_out)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1045, self, varargin{:});
    end
    function varargout = create(self,varargin)
    %CREATE Construct a function object.
    %
    %  Function = CREATE(self, char fname, {char} s_in, {char} s_out)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1046, self, varargin{:});
    end
    function varargout = var(self,varargin)
    %VAR Get variable expression by name.
    %
    %  MX = VAR(self, char name)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1047, self, varargin{:});
    end
    function varargout = paren(self,varargin)
    %PAREN 
    %
    %  MX = PAREN(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1048, self, varargin{:});
    end
    function varargout = der(self,varargin)
    %DER Get a derivative expression by non-differentiated expression.
    %
    %  MX = DER(self, MX var)
    %  MX = DER(self, char name)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  DER(self, char name)
    %
    %
    %
    %Get a derivative expression by name.
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
    %  DER(self, MX var)
    %
    %
    %
    %Get a derivative expression by non-differentiated expression.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1049, self, varargin{:});
    end
    function varargout = nominal(self,varargin)
    %NOMINAL Get the nominal value(s) by expression.
    %
    %  [double] = NOMINAL(self, MX var)
    %  double = NOMINAL(self, char name)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  NOMINAL(self, MX var)
    %
    %
    %
    %Get the nominal value(s) by expression.
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
    %  NOMINAL(self, char name)
    %
    %
    %
    %Get the nominal value by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1050, self, varargin{:});
    end
    function varargout = set_nominal(self,varargin)
    %SET_NOMINAL Set the nominal value(s) by expression.
    %
    %  SET_NOMINAL(self, MX var, [double] val)
    %  SET_NOMINAL(self, char name, double val)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  SET_NOMINAL(self, MX var, [double] val)
    %
    %
    %
    %Set the nominal value(s) by expression.
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
    %  SET_NOMINAL(self, char name, double val)
    %
    %
    %
    %Set the nominal value by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1051, self, varargin{:});
    end
    function varargout = min(self,varargin)
    %MIN Get the lower bound(s) by expression.
    %
    %  [double] = MIN(self, MX var, bool normalized)
    %  double = MIN(self, char name, bool normalized)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  MIN(self, MX var, bool normalized)
    %
    %
    %
    %Get the lower bound(s) by expression.
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
    %  MIN(self, char name, bool normalized)
    %
    %
    %
    %Get the lower bound by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1052, self, varargin{:});
    end
    function varargout = set_min(self,varargin)
    %SET_MIN Set the lower bound(s) by expression.
    %
    %  SET_MIN(self, MX var, [double] val, bool normalized)
    %  SET_MIN(self, char name, double val, bool normalized)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  SET_MIN(self, MX var, [double] val, bool normalized)
    %
    %
    %
    %Set the lower bound(s) by expression.
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
    %  SET_MIN(self, char name, double val, bool normalized)
    %
    %
    %
    %Set the lower bound by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1053, self, varargin{:});
    end
    function varargout = max(self,varargin)
    %MAX Get the upper bound(s) by expression.
    %
    %  [double] = MAX(self, MX var, bool normalized)
    %  double = MAX(self, char name, bool normalized)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  MAX(self, MX var, bool normalized)
    %
    %
    %
    %Get the upper bound(s) by expression.
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
    %  MAX(self, char name, bool normalized)
    %
    %
    %
    %Get the upper bound by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1054, self, varargin{:});
    end
    function varargout = set_max(self,varargin)
    %SET_MAX Set the upper bound(s) by expression.
    %
    %  SET_MAX(self, MX var, [double] val, bool normalized)
    %  SET_MAX(self, char name, double val, bool normalized)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  SET_MAX(self, MX var, [double] val, bool normalized)
    %
    %
    %
    %Set the upper bound(s) by expression.
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
    %  SET_MAX(self, char name, double val, bool normalized)
    %
    %
    %
    %Set the upper bound by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1055, self, varargin{:});
    end
    function varargout = guess(self,varargin)
    %GUESS Get the initial guess(es) by expression.
    %
    %  [double] = GUESS(self, MX var, bool normalized)
    %  double = GUESS(self, char name, bool normalized)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  GUESS(self, MX var, bool normalized)
    %
    %
    %
    %Get the initial guess(es) by expression.
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
    %  GUESS(self, char name, bool normalized)
    %
    %
    %
    %Get the initial guess by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1056, self, varargin{:});
    end
    function varargout = set_guess(self,varargin)
    %SET_GUESS Set the initial guess(es) by expression.
    %
    %  SET_GUESS(self, MX var, [double] val, bool normalized)
    %  SET_GUESS(self, char name, double val, bool normalized)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  SET_GUESS(self, MX var, [double] val, bool normalized)
    %
    %
    %
    %Set the initial guess(es) by expression.
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
    %  SET_GUESS(self, char name, double val, bool normalized)
    %
    %
    %
    %Set the initial guess by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1057, self, varargin{:});
    end
    function varargout = start(self,varargin)
    %START Get the (optionally normalized) value(s) at time 0 by expression.
    %
    %  [double] = START(self, MX var, bool normalized)
    %  double = START(self, char name, bool normalized)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  START(self, MX var, bool normalized)
    %
    %
    %
    %Get the (optionally normalized) value(s) at time 0 by expression.
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
    %  START(self, char name, bool normalized)
    %
    %
    %
    %Get the (optionally normalized) value at time 0 by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1058, self, varargin{:});
    end
    function varargout = set_start(self,varargin)
    %SET_START Set the (optionally normalized) value(s) at time 0 by expression.
    %
    %  SET_START(self, MX var, [double] val, bool normalized)
    %  SET_START(self, char name, double val, bool normalized)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  SET_START(self, MX var, [double] val, bool normalized)
    %
    %
    %
    %Set the (optionally normalized) value(s) at time 0 by expression.
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
    %  SET_START(self, char name, double val, bool normalized)
    %
    %
    %
    %Set the (optionally normalized) value at time 0 by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1059, self, varargin{:});
    end
    function varargout = derivative_start(self,varargin)
    %DERIVATIVE_START Get the (optionally normalized) derivative value(s) at time 0 by expression.
    %
    %  [double] = DERIVATIVE_START(self, MX var, bool normalized)
    %  double = DERIVATIVE_START(self, char name, bool normalized)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  DERIVATIVE_START(self, MX var, bool normalized)
    %
    %
    %
    %Get the (optionally normalized) derivative value(s) at time 0 by expression.
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
    %  DERIVATIVE_START(self, char name, bool normalized)
    %
    %
    %
    %Get the (optionally normalized) derivative value at time 0 by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1060, self, varargin{:});
    end
    function varargout = set_derivative_start(self,varargin)
    %SET_DERIVATIVE_START Set the (optionally normalized) derivative value(s) at time 0 by expression.
    %
    %  SET_DERIVATIVE_START(self, MX var, [double] val, bool normalized)
    %  SET_DERIVATIVE_START(self, char name, double val, bool normalized)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  SET_DERIVATIVE_START(self, MX var, [double] val, bool normalized)
    %
    %
    %
    %Set the (optionally normalized) derivative value(s) at time 0 by expression.
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
    %  SET_DERIVATIVE_START(self, char name, double val, bool normalized)
    %
    %
    %
    %Set the (optionally normalized) derivative value at time 0 by name.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1061, self, varargin{:});
    end
    function varargout = unit(self,varargin)
    %UNIT Get the unit given a vector of symbolic variables (all units must be
    %
    %  char = UNIT(self, MX var)
    %  char = UNIT(self, char name)
    %
    %identical)
    %
    %
    %
    %.......
    %
    %::
    %
    %  UNIT(self, MX var)
    %
    %
    %
    %Get the unit given a vector of symbolic variables (all units must be
    %identical)
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
    %  UNIT(self, char name)
    %
    %
    %
    %Get the unit for a component.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1062, self, varargin{:});
    end
    function varargout = set_unit(self,varargin)
    %SET_UNIT Set the unit for a component.
    %
    %  SET_UNIT(self, char name, char val)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1063, self, varargin{:});
    end
    function varargout = type_name(self,varargin)
    %TYPE_NAME Readable name of the class.
    %
    %  char = TYPE_NAME(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1064, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP Print representation.
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1065, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR Get string representation.
    %
    %  char = STR(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1066, self, varargin{:});
    end
    function varargout = add_variable(self,varargin)
    %ADD_VARIABLE Add a new variable: returns corresponding symbolic expression.
    %
    %  MX = ADD_VARIABLE(self, char name, int n)
    %  ADD_VARIABLE(self, char name, Variable var)
    %  MX = ADD_VARIABLE(self, char name, Sparsity sp)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  ADD_VARIABLE(self, char name, Variable var)
    %
    %
    %
    %Add a variable.
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
    %  ADD_VARIABLE(self, char name, int n)
    %  ADD_VARIABLE(self, char name, Sparsity sp)
    %
    %
    %
    %Add a new variable: returns corresponding symbolic expression.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1067, self, varargin{:});
    end
    function varargout = variable(self,varargin)
    %VARIABLE Access a variable by name
    %
    %  Variable = VARIABLE(self, char name)
    %  Variable = VARIABLE(self, char name)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1068, self, varargin{:});
    end
    function self = DaeBuilder(varargin)
    %DAEBUILDER 
    %
    %  new_obj = DAEBUILDER()
    %
    %
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1069, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1070, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
