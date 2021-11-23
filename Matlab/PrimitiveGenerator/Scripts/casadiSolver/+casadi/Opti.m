classdef  Opti < casadi.PrintableCommon & casadi.SharedObject
    %OPTI A simplified interface for NLP modeling/solving.
    %
    %
    %
    %This class offers a view with model description facilities The API is
    %guaranteed to be stable.
    %
    %Example NLP:
    %
    %::
    %
    %    opti = casadi.Opti();
    %  
    %    x = opti.variable();
    %    y = opti.variable();
    %  
    %    opti.minimize(  (y-x^2)^2   );
    %    opti.subject_to( x^2+y^2==1 );
    %    opti.subject_to(     x+y>=1 );
    %  
    %    opti.solver('ipopt');
    %    sol = opti.solve();
    %  
    %    sol.value(x)
    %    sol.value(y)
    %
    %
    %
    %Example parametric NLP:
    %
    %::
    %
    %    opti = casadi.Opti();
    %  
    %    x = opti.variable(2,1);
    %    p = opti.parameter();
    %  
    %    opti.minimize(  (p*x(2)-x(1)^2)^2   );
    %    opti.subject_to( 1<=sum(x)<=2 );
    %  
    %    opti.solver('ipopt');
    %  
    %    opti.set_value(p, 3);
    %    sol = opti.solve();
    %    sol.value(x)
    %  
    %    opti.set_value(p, 5);
    %    sol = opti.solve();
    %    sol.value(x)
    %
    %
    %
    %Joris Gillis, Erik Lambrechts, Joel Andersson
    %
    %C++ includes: optistack.hpp 
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = internal_variable(self,varargin)
    %INTERNAL_VARIABLE Create a decision variable (symbol)
    %
    %  MX = INTERNAL_VARIABLE(self, int n, int m, char attribute)
    %
    %
    %The order of creation matters. The order will be reflected in the
    %optimization problem. It is not required for decision variables to actualy
    %appear in the optimization problem.
    %
    %Parameters:
    %-----------
    %
    %n:  number of rows (default 1)
    %
    %m:  number of columnss (default 1)
    %
    %attribute:  'full' (default) or 'symmetric'
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1135, self, varargin{:});
    end
    function varargout = internal_parameter(self,varargin)
    %INTERNAL_PARAMETER Create a parameter (symbol); fixed during optimization.
    %
    %  MX = INTERNAL_PARAMETER(self, int n, int m, char attribute)
    %
    %
    %The order of creation does not matter. It is not required for parameter to
    %actualy appear in the optimization problem. Parameters that do appear, must
    %be given a value before the problem can be solved.
    %
    %Parameters:
    %-----------
    %
    %n:  number of rows (default 1)
    %
    %m:  number of columnss (default 1)
    %
    %attribute:  'full' (default) or 'symmetric'
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1136, self, varargin{:});
    end
    function varargout = minimize(self,varargin)
    %MINIMIZE Set objective.
    %
    %  MINIMIZE(self, MX f)
    %
    %
    %Objective must be a scalar. Default objective: 0 When method is called
    %multiple times, the last call takes effect
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1137, self, varargin{:});
    end
    function varargout = internal_subject_to(self,varargin)
    %INTERNAL_SUBJECT_TO Clear constraints.
    %
    %  INTERNAL_SUBJECT_TO(self)
    %  INTERNAL_SUBJECT_TO(self, MX g)
    %  INTERNAL_SUBJECT_TO(self, {MX} g)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  INTERNAL_SUBJECT_TO(self)
    %
    %
    %
    %Clear constraints.
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
    %  INTERNAL_SUBJECT_TO(self, MX g)
    %  INTERNAL_SUBJECT_TO(self, {MX} g)
    %
    %
    %
    %Add constraints.
    %
    %Examples:
    %
    %::
    %
    %  * \\begin{itemize}
    %  * opti.subject_to( sqrt(x+y) >= 1);
    %  * opti.subject_to( sqrt(x+y) > 1)}: same as above
    %  * opti.subject_to( 1<= sqrt(x+y) )}: same as above
    %  * opti.subject_to( 5*x+y==1 )}: equality
    %  *
    %  * Python
    %  * opti.subject_to([x*y>=1,x==3])
    %  * opti.subject_to(opti.bounded(0,x,1))
    %  *
    %  * MATLAB
    %  * opti.subject_to({x*y>=1,x==3})
    %  * opti.subject_to( 0<=x<=1 )
    %  * 
    %
    %
    %
    %Related functionalities: opti.lbg,opti.g,opti.ubg represent the vector of
    %flattened constraints
    %
    %opti.debug.show_infeasibilities() may be used to inspect which constraints
    %are violated
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1138, self, varargin{:});
    end
    function varargout = solver(self,varargin)
    %SOLVER Set a solver.
    %
    %  SOLVER(self, char solver, struct plugin_options, struct solver_options)
    %
    %
    %Parameters:
    %-----------
    %
    %solver:  any of the nlpsol plugins can be used here In practice, not all
    %nlpsol plugins may be supported yet
    %
    %options:  passed on to nlpsol plugin No stability can be guaranteed about
    %this part of the API
    %
    %options:  to be passed to nlpsol solver No stability can be guaranteed about
    %this part of the API
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1139, self, varargin{:});
    end
    function varargout = set_initial(self,varargin)
    %SET_INITIAL Set initial guess for decision variables
    %
    %  SET_INITIAL(self, {MX} assignments)
    %  SET_INITIAL(self, MX x, DM v)
    %
    %
    %::
    %
    %  * opti.set_initial(x, 2)
    %  * opti.set_initial(10*x(1), 2)
    %  * 
    %
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1140, self, varargin{:});
    end
    function varargout = set_value(self,varargin)
    %SET_VALUE Set value of parameter.
    %
    %  SET_VALUE(self, {MX} assignments)
    %  SET_VALUE(self, MX x, DM v)
    %
    %
    %Each parameter must be given a value before 'solve' can be called
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1141, self, varargin{:});
    end
    function varargout = solve(self,varargin)
    %SOLVE Crunch the numbers; solve the problem.
    %
    %  OptiSol = SOLVE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1142, self, varargin{:});
    end
    function varargout = solve_limited(self,varargin)
    %SOLVE_LIMITED Crunch the numbers; solve the problem.
    %
    %  OptiSol = SOLVE_LIMITED(self)
    %
    %
    %Allows the solver to return without error when an iteration or time limit is
    %reached
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1143, self, varargin{:});
    end
    function varargout = value(self,varargin)
    %VALUE Obtain value of expression at the current value
    %
    %  double = VALUE(self, DM x, {MX} values)
    %  double = VALUE(self, SX x, {MX} values)
    %  double = VALUE(self, MX x, {MX} values)
    %
    %
    %In regular mode, teh current value is the converged solution In debug mode,
    %the value can be non-converged
    %
    %Parameters:
    %-----------
    %
    %values:  Optional assignment expressions (e.g. x==3) to overrule the current
    %value
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1144, self, varargin{:});
    end
    function varargout = stats(self,varargin)
    %STATS Get statistics.
    %
    %  struct = STATS(self)
    %
    %
    %nlpsol stats are passed as-is. No stability can be guaranteed about this
    %part of the API
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1145, self, varargin{:});
    end
    function varargout = return_status(self,varargin)
    %RETURN_STATUS Get return status of solver passed as-is from nlpsol No stability can be
    %
    %  char = RETURN_STATUS(self)
    %
    %guaranteed about this part of the API.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1146, self, varargin{:});
    end
    function varargout = initial(self,varargin)
    %INITIAL get assignment expressions for initial values
    %
    %  {MX} = INITIAL(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1147, self, varargin{:});
    end
    function varargout = value_variables(self,varargin)
    %VALUE_VARIABLES get assignment expressions for latest values
    %
    %  {MX} = VALUE_VARIABLES(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1148, self, varargin{:});
    end
    function varargout = value_parameters(self,varargin)
    %VALUE_PARAMETERS 
    %
    %  {MX} = VALUE_PARAMETERS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1149, self, varargin{:});
    end
    function varargout = dual(self,varargin)
    %DUAL get the dual variable
    %
    %  MX = DUAL(self, MX m)
    %
    %
    %m must be a constraint expression. The returned value is still a symbolic
    %expression. Use value on it to obtain the numerical value.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1150, self, varargin{:});
    end
    function varargout = nx(self,varargin)
    %NX Number of (scalarised) decision variables.
    %
    %  int = NX(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1151, self, varargin{:});
    end
    function varargout = np(self,varargin)
    %NP Number of (scalarised) parameters.
    %
    %  int = NP(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1152, self, varargin{:});
    end
    function varargout = ng(self,varargin)
    %NG Number of (scalarised) constraints.
    %
    %  int = NG(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1153, self, varargin{:});
    end
    function varargout = x(self,varargin)
    %X Get all (scalarised) decision variables as a symbolic column vector.
    %
    %  MX = X(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1154, self, varargin{:});
    end
    function varargout = p(self,varargin)
    %P Get all (scalarised) parameters as a symbolic column vector.
    %
    %  MX = P(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1155, self, varargin{:});
    end
    function varargout = g(self,varargin)
    %G Get all (scalarised) constraint expressions as a column vector.
    %
    %  MX = G(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1156, self, varargin{:});
    end
    function varargout = f(self,varargin)
    %F Get objective expression.
    %
    %  MX = F(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1157, self, varargin{:});
    end
    function varargout = lbg(self,varargin)
    %LBG Get all (scalarised) bounds on constraints as a column vector.
    %
    %  MX = LBG(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1158, self, varargin{:});
    end
    function varargout = ubg(self,varargin)
    %UBG 
    %
    %  MX = UBG(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1159, self, varargin{:});
    end
    function varargout = lam_g(self,varargin)
    %LAM_G Get all (scalarised) dual variables as a symbolic column vector.
    %
    %  MX = LAM_G(self)
    %
    %
    %Useful for obtaining the Lagrange Hessian:
    %
    %::
    %
    %  * sol.value(hessian(opti.f+opti.lam_g'*opti.g,opti.x)) % MATLAB
    %  * sol.value(hessian(opti.f+dot(opti.lam_g,opti.g),opti.x)[0]) # Python
    %  * 
    %
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1160, self, varargin{:});
    end
    function varargout = to_function(self,varargin)
    %TO_FUNCTION 
    %
    %  Function = TO_FUNCTION(self, char name, {MX} args, {MX} res, struct opts)
    %  Function = TO_FUNCTION(self, char name, struct:MX dict, {char} name_in, {char} name_out, struct opts)
    %  Function = TO_FUNCTION(self, char name, {MX} args, {MX} res, {char} name_in, {char} name_out, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  TO_FUNCTION(self, char name, struct:MX dict, {char} name_in, {char} name_out, struct opts)
    %  TO_FUNCTION(self, char name, {MX} args, {MX} res, {char} name_in, {char} name_out, struct opts)
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
    %  TO_FUNCTION(self, char name, {MX} args, {MX} res, struct opts)
    %
    %
    %
    %Create a CasADi Function from the Opti solver.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1161, self, varargin{:});
    end
    function varargout = debug(self,varargin)
    %DEBUG Get a copy with advanced functionality.
    %
    %  OptiAdvanced = DEBUG(self)
    %
    %
    %You get access to more methods, but you have no guarantees about API
    %stability
    %
    %The copy is effectively a deep copy: Updating the state of the copy does not
    %update the original.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1162, self, varargin{:});
    end
    function varargout = advanced(self,varargin)
    %ADVANCED Get a copy with advanced functionality.
    %
    %  OptiAdvanced = ADVANCED(self)
    %
    %
    %You get access to more methods, but you have no guarantees about API
    %stability
    %
    %The copy is effectively a deep copy: Updating the state of the copy does not
    %update the original.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1163, self, varargin{:});
    end
    function varargout = copy(self,varargin)
    %COPY Get a copy of the.
    %
    %  Opti = COPY(self)
    %
    %
    %The copy is effectively a deep copy: Updating the state of the copy does not
    %update the original.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1164, self, varargin{:});
    end
    function varargout = update_user_dict(self,varargin)
    %UPDATE_USER_DICT 
    %
    %  UPDATE_USER_DICT(self, MX m, struct meta)
    %  UPDATE_USER_DICT(self, {MX} m, struct meta)
    %
    %
    %.......
    %
    %::
    %
    %  UPDATE_USER_DICT(self, {MX} m, struct meta)
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
    %  UPDATE_USER_DICT(self, MX m, struct meta)
    %
    %
    %
    %add user data Add arbitrary data in the form of a dictionary to symbols or
    %constraints
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1165, self, varargin{:});
    end
    function varargout = user_dict(self,varargin)
    %USER_DICT Get user data.
    %
    %  struct = USER_DICT(self, MX m)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1166, self, varargin{:});
    end
    function varargout = type_name(self,varargin)
    %TYPE_NAME Readable name of the class.
    %
    %  char = TYPE_NAME(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1167, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP Print representation.
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1168, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR Get string representation.
    %
    %  char = STR(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1169, self, varargin{:});
    end
    function varargout = callback_class(self,varargin)
    %CALLBACK_CLASS Helper methods for callback()
    %
    %  CALLBACK_CLASS(self)
    %  CALLBACK_CLASS(self, OptiCallback callback)
    %
    %
    %Do not use directly.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1170, self, varargin{:});
    end

      function out = variable(self, varargin)
        st = dbstack('-completenames',1);
        if length(st)>0
          meta = struct('stacktrace', st(1));
        else
          meta = struct;
        end
        out = self.internal_variable(varargin{:});
        self.update_user_dict(out, meta);
      end
      function out = parameter(self, varargin)
        st = dbstack('-completenames',1);
        if length(st)>0
          meta = struct('stacktrace', st(1));
        else
          meta = struct;
        end
        out = self.internal_parameter(varargin{:});
        self.update_user_dict(out, meta);
      end
      function [] = subject_to(self, varargin)
        if length(varargin)==0
          self.internal_subject_to();
          return;
        end
        st = dbstack('-completenames',1);
        if length(st)>0
          meta = struct('stacktrace', st(1));
        else
          meta = struct;
        end
        self.internal_subject_to(varargin{:});
        self.update_user_dict(varargin{1}, meta);
      end
    
    function [] = callback(self, varargin)
      casadi.OptiCallbackHelper.callback_setup(self, varargin{:})
    end
      function self = Opti(varargin)
    %OPTI 
    %
    %  new_obj = OPTI(char problem_type)
    %
    %
      self@casadi.PrintableCommon(SwigRef.Null);
      self@casadi.SharedObject(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1171, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1172, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
