classdef  Variable < casadi.PrintableCommon
    %VARIABLE 
    %
    %   = VARIABLE()
    %
    %
  methods
    function varargout = name(self,varargin)
    %NAME 
    %
    %  char = NAME(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(963, self, varargin{:});
    end
    function v = v(self)
      v = casadiMEX(964, self);
    end
    function v = d(self)
      v = casadiMEX(965, self);
    end
    function v = nominal(self)
      v = casadiMEX(966, self);
    end
    function v = start(self)
      v = casadiMEX(967, self);
    end
    function v = min(self)
      v = casadiMEX(968, self);
    end
    function v = max(self)
      v = casadiMEX(969, self);
    end
    function v = guess(self)
      v = casadiMEX(970, self);
    end
    function v = derivative_start(self)
      v = casadiMEX(971, self);
    end
    function v = variability(self)
      v = casadiMEX(972, self);
    end
    function v = causality(self)
      v = casadiMEX(973, self);
    end
    function v = category(self)
      v = casadiMEX(974, self);
    end
    function v = alias(self)
      v = casadiMEX(975, self);
    end
    function v = description(self)
      v = casadiMEX(976, self);
    end
    function v = valueReference(self)
      v = casadiMEX(977, self);
    end
    function v = unit(self)
      v = casadiMEX(978, self);
    end
    function v = display_unit(self)
      v = casadiMEX(979, self);
    end
    function v = free(self)
      v = casadiMEX(980, self);
    end
    function varargout = type_name(self,varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(981, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP 
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %
      [varargout{1:nargout}] = casadiMEX(982, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR 
    %
    %  char = STR(self, bool more)
    %
    %
      [varargout{1:nargout}] = casadiMEX(983, self, varargin{:});
    end
    function self = Variable(varargin)
    %VARIABLE 
    %
    %  new_obj = VARIABLE()
    %  new_obj = VARIABLE(char name, Sparsity sp)
    %
    %
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(984, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(985, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
