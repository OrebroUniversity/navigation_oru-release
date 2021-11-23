classdef  NlpBuilder < casadi.PrintableCommon
    %NLPBUILDER A symbolic NLP representation.
    %
    %
    %
    %Joel Andersson
    %
    %C++ includes: nlp_builder.hpp 
    %
  methods
    function v = x(self)
      v = casadiMEX(947, self);
    end
    function v = f(self)
      v = casadiMEX(948, self);
    end
    function v = g(self)
      v = casadiMEX(949, self);
    end
    function v = x_lb(self)
      v = casadiMEX(950, self);
    end
    function v = x_ub(self)
      v = casadiMEX(951, self);
    end
    function v = g_lb(self)
      v = casadiMEX(952, self);
    end
    function v = g_ub(self)
      v = casadiMEX(953, self);
    end
    function v = x_init(self)
      v = casadiMEX(954, self);
    end
    function v = lambda_init(self)
      v = casadiMEX(955, self);
    end
    function v = discrete(self)
      v = casadiMEX(956, self);
    end
    function varargout = import_nl(self,varargin)
    %IMPORT_NL Import an .nl file.
    %
    %  IMPORT_NL(self, char filename, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(957, self, varargin{:});
    end
    function varargout = type_name(self,varargin)
    %TYPE_NAME Readable name of the class.
    %
    %  char = TYPE_NAME(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(958, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP Print a description of the object.
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(959, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR Get string representation.
    %
    %  char = STR(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(960, self, varargin{:});
    end
    function self = NlpBuilder(varargin)
    %NLPBUILDER 
    %
    %  new_obj = NLPBUILDER()
    %
    %
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(961, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(962, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
