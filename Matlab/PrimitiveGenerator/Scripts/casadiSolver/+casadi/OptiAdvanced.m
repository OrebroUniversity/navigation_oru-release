classdef  OptiAdvanced < casadi.Opti
    %OPTIADVANCED C++ includes: optistack.hpp 
    %
    %
    %
  methods
    function delete(self)
        if self.swigPtr
          casadiMEX(1202, self);
          self.SwigClear();
        end
    end
    function varargout = solver(self,varargin)
    %SOLVER Get the underlying CasADi solver of the Opti stack.
    %
    %  Function = SOLVER(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1203, self, varargin{:});
    end
    function varargout = is_parametric(self,varargin)
    %IS_PARAMETRIC return true if expression is only dependant on Opti parameters, not
    %
    %  bool = IS_PARAMETRIC(self, MX expr)
    %
    %variables
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1204, self, varargin{:});
    end
    function varargout = symvar(self,varargin)
    %SYMVAR Get symbols present in expression.
    %
    %  {MX} = SYMVAR(self)
    %  {MX} = SYMVAR(self, MX expr)
    %  {MX} = SYMVAR(self, MX expr, casadi::VariableType type)
    %
    %
    %Returned vector is ordered according to the order of variable()/parameter()
    %calls used to create the variables
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1205, self, varargin{:});
    end
    function varargout = canon_expr(self,varargin)
    %CANON_EXPR Interpret an expression (for internal use only)
    %
    %  MetaCon = CANON_EXPR(self, MX expr)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1206, self, varargin{:});
    end
    function varargout = get_meta(self,varargin)
    %GET_META Get meta-data of symbol (for internal use only)
    %
    %  MetaVar = GET_META(self, MX m)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1207, self, varargin{:});
    end
    function varargout = get_meta_con(self,varargin)
    %GET_META_CON Get meta-data of symbol (for internal use only)
    %
    %  MetaCon = GET_META_CON(self, MX m)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1208, self, varargin{:});
    end
    function varargout = set_meta(self,varargin)
    %SET_META Set meta-data of an expression.
    %
    %  SET_META(self, MX m, MetaVar meta)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1209, self, varargin{:});
    end
    function varargout = set_meta_con(self,varargin)
    %SET_META_CON Set meta-data of an expression.
    %
    %  SET_META_CON(self, MX m, MetaCon meta)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1210, self, varargin{:});
    end
    function varargout = assert_active_symbol(self,varargin)
    %ASSERT_ACTIVE_SYMBOL 
    %
    %  ASSERT_ACTIVE_SYMBOL(self, MX m)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1211, self, varargin{:});
    end
    function varargout = active_symvar(self,varargin)
    %ACTIVE_SYMVAR 
    %
    %  {MX} = ACTIVE_SYMVAR(self, casadi::VariableType type)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1212, self, varargin{:});
    end
    function varargout = active_values(self,varargin)
    %ACTIVE_VALUES 
    %
    %  {DM} = ACTIVE_VALUES(self, casadi::VariableType type)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1213, self, varargin{:});
    end
    function varargout = x_lookup(self,varargin)
    %X_LOOKUP 
    %
    %  MX = X_LOOKUP(self, index i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1214, self, varargin{:});
    end
    function varargout = g_lookup(self,varargin)
    %G_LOOKUP 
    %
    %  MX = G_LOOKUP(self, index i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1215, self, varargin{:});
    end
    function varargout = x_describe(self,varargin)
    %X_DESCRIBE 
    %
    %  char = X_DESCRIBE(self, index i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1216, self, varargin{:});
    end
    function varargout = g_describe(self,varargin)
    %G_DESCRIBE 
    %
    %  char = G_DESCRIBE(self, index i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1217, self, varargin{:});
    end
    function varargout = describe(self,varargin)
    %DESCRIBE 
    %
    %  char = DESCRIBE(self, MX x, index indent)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1218, self, varargin{:});
    end
    function varargout = show_infeasibilities(self,varargin)
    %SHOW_INFEASIBILITIES 
    %
    %  SHOW_INFEASIBILITIES(self, double tol)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1219, self, varargin{:});
    end
    function varargout = solve_prepare(self,varargin)
    %SOLVE_PREPARE 
    %
    %  SOLVE_PREPARE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1220, self, varargin{:});
    end
    function varargout = solve_actual(self,varargin)
    %SOLVE_ACTUAL 
    %
    %  struct:DM = SOLVE_ACTUAL(self, struct:DM args)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1221, self, varargin{:});
    end
    function varargout = arg(self,varargin)
    %ARG 
    %
    %  struct:DM = ARG(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1222, self, varargin{:});
    end
    function varargout = res(self,varargin)
    %RES 
    %
    %  struct:DM = RES(self)
    %  RES(self, struct:DM res)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1223, self, varargin{:});
    end
    function varargout = constraints(self,varargin)
    %CONSTRAINTS 
    %
    %  {MX} = CONSTRAINTS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1224, self, varargin{:});
    end
    function varargout = objective(self,varargin)
    %OBJECTIVE 
    %
    %  MX = OBJECTIVE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1225, self, varargin{:});
    end
    function varargout = baked_copy(self,varargin)
    %BAKED_COPY 
    %
    %  OptiAdvanced = BAKED_COPY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1226, self, varargin{:});
    end
    function varargout = assert_empty(self,varargin)
    %ASSERT_EMPTY 
    %
    %  ASSERT_EMPTY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1227, self, varargin{:});
    end
    function varargout = bake(self,varargin)
    %BAKE Fix the structure of the optimization problem.
    %
    %  BAKE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1228, self, varargin{:});
    end
    function v = problem_dirty_(self)
      v = casadiMEX(1229, self);
    end
    function varargout = mark_problem_dirty(self,varargin)
    %MARK_PROBLEM_DIRTY 
    %
    %  MARK_PROBLEM_DIRTY(self, bool flag)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1230, self, varargin{:});
    end
    function varargout = problem_dirty(self,varargin)
    %PROBLEM_DIRTY 
    %
    %  bool = PROBLEM_DIRTY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1231, self, varargin{:});
    end
    function v = solver_dirty_(self)
      v = casadiMEX(1232, self);
    end
    function varargout = mark_solver_dirty(self,varargin)
    %MARK_SOLVER_DIRTY 
    %
    %  MARK_SOLVER_DIRTY(self, bool flag)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1233, self, varargin{:});
    end
    function varargout = solver_dirty(self,varargin)
    %SOLVER_DIRTY 
    %
    %  bool = SOLVER_DIRTY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1234, self, varargin{:});
    end
    function v = solved_(self)
      v = casadiMEX(1235, self);
    end
    function varargout = mark_solved(self,varargin)
    %MARK_SOLVED 
    %
    %  MARK_SOLVED(self, bool flag)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1236, self, varargin{:});
    end
    function varargout = solved(self,varargin)
    %SOLVED 
    %
    %  bool = SOLVED(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1237, self, varargin{:});
    end
    function varargout = assert_solved(self,varargin)
    %ASSERT_SOLVED 
    %
    %  ASSERT_SOLVED(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1238, self, varargin{:});
    end
    function varargout = assert_baked(self,varargin)
    %ASSERT_BAKED 
    %
    %  ASSERT_BAKED(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1239, self, varargin{:});
    end
    function varargout = instance_number(self,varargin)
    %INSTANCE_NUMBER 
    %
    %  int = INSTANCE_NUMBER(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1240, self, varargin{:});
    end
    function self = OptiAdvanced(varargin)
    %OPTIADVANCED 
    %
    %  new_obj = OPTIADVANCED(Opti x)
    %
    %
      self@casadi.Opti(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1241, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
