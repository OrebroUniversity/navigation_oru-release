classdef  MetaCon < casadi.IndexAbstraction
    %METACON 
    %
    %   = METACON()
    %
    %
  methods
    function v = original(self)
      v = casadiMEX(1177, self);
    end
    function v = canon(self)
      v = casadiMEX(1178, self);
    end
    function v = type(self)
      v = casadiMEX(1179, self);
    end
    function v = lb(self)
      v = casadiMEX(1180, self);
    end
    function v = ub(self)
      v = casadiMEX(1181, self);
    end
    function v = n(self)
      v = casadiMEX(1182, self);
    end
    function v = flipped(self)
      v = casadiMEX(1183, self);
    end
    function v = dual_canon(self)
      v = casadiMEX(1184, self);
    end
    function v = dual(self)
      v = casadiMEX(1185, self);
    end
    function v = extra(self)
      v = casadiMEX(1186, self);
    end
    function self = MetaCon(varargin)
    %METACON 
    %
    %  new_obj = METACON()
    %
    %
      self@casadi.IndexAbstraction(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1187, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1188, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
