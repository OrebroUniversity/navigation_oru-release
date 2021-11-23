classdef  OptiSol < casadi.PrintableCommon
    %OPTISOL A simplified interface for NLP modeling/solving.
    %
    %
    %
    %This class offers a view with solution retrieval facilities The API is
    %guaranteed to be stable.
    %
    %Joris Gillis, Erik Lambrechts
    %
    %C++ includes: optistack.hpp 
    %
  methods
    function varargout = type_name(self,varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1242, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP 
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1243, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR 
    %
    %  char = STR(self, bool more)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1244, self, varargin{:});
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
      [varargout{1:nargout}] = casadiMEX(1245, self, varargin{:});
    end
    function varargout = value_variables(self,varargin)
    %VALUE_VARIABLES get assignment expressions for the optimal solution
    %
    %  {MX} = VALUE_VARIABLES(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1246, self, varargin{:});
    end
    function varargout = value_parameters(self,varargin)
    %VALUE_PARAMETERS 
    %
    %  {MX} = VALUE_PARAMETERS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1247, self, varargin{:});
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
      [varargout{1:nargout}] = casadiMEX(1248, self, varargin{:});
    end
    function varargout = opti(self,varargin)
    %OPTI 
    %
    %  Opti = OPTI(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1249, self, varargin{:});
    end
    function self = OptiSol(varargin)
    %OPTISOL 
    %
    %  new_obj = OPTISOL()
    %
    %
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1250, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1251, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
