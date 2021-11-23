classdef  CasadiMeta < SwigRef
    %CASADIMETA Collects global CasADi meta information.
    %
    %
    %
    %Joris Gillis
    %
    %C++ includes: casadi_meta.hpp 
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function self = CasadiMeta(varargin)
    %CASADIMETA 
    %
    %  new_obj = CASADIMETA()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(938, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(939, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
    function varargout = version(varargin)
    %VERSION 
    %
    %  char const * = VERSION()
    %
    %
     [varargout{1:nargout}] = casadiMEX(927, varargin{:});
    end
    function varargout = git_revision(varargin)
    %GIT_REVISION 
    %
    %  char const * = GIT_REVISION()
    %
    %
     [varargout{1:nargout}] = casadiMEX(928, varargin{:});
    end
    function varargout = git_describe(varargin)
    %GIT_DESCRIBE 
    %
    %  char const * = GIT_DESCRIBE()
    %
    %
     [varargout{1:nargout}] = casadiMEX(929, varargin{:});
    end
    function varargout = feature_list(varargin)
    %FEATURE_LIST 
    %
    %  char const * = FEATURE_LIST()
    %
    %
     [varargout{1:nargout}] = casadiMEX(930, varargin{:});
    end
    function varargout = build_type(varargin)
    %BUILD_TYPE 
    %
    %  char const * = BUILD_TYPE()
    %
    %
     [varargout{1:nargout}] = casadiMEX(931, varargin{:});
    end
    function varargout = compiler_id(varargin)
    %COMPILER_ID 
    %
    %  char const * = COMPILER_ID()
    %
    %
     [varargout{1:nargout}] = casadiMEX(932, varargin{:});
    end
    function varargout = compiler(varargin)
    %COMPILER 
    %
    %  char const * = COMPILER()
    %
    %
     [varargout{1:nargout}] = casadiMEX(933, varargin{:});
    end
    function varargout = compiler_flags(varargin)
    %COMPILER_FLAGS 
    %
    %  char const * = COMPILER_FLAGS()
    %
    %
     [varargout{1:nargout}] = casadiMEX(934, varargin{:});
    end
    function varargout = modules(varargin)
    %MODULES 
    %
    %  char const * = MODULES()
    %
    %
     [varargout{1:nargout}] = casadiMEX(935, varargin{:});
    end
    function varargout = plugins(varargin)
    %PLUGINS 
    %
    %  char const * = PLUGINS()
    %
    %
     [varargout{1:nargout}] = casadiMEX(936, varargin{:});
    end
    function varargout = install_prefix(varargin)
    %INSTALL_PREFIX 
    %
    %  char const * = INSTALL_PREFIX()
    %
    %
     [varargout{1:nargout}] = casadiMEX(937, varargin{:});
    end
  end
end
