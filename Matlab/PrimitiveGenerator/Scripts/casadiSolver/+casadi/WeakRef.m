classdef  WeakRef < casadi.SharedObject
    %WEAKREF Weak reference type A weak reference to a SharedObject.
    %
    %
    %
    %Joel Andersson
    %
    %C++ includes: shared_object.hpp 
    %
  methods
    function varargout = shared(self,varargin)
    %SHARED Get a shared (owning) reference.
    %
    %  SharedObject = SHARED(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(31, self, varargin{:});
    end
    function varargout = alive(self,varargin)
    %ALIVE Check if alive.
    %
    %  bool = ALIVE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(32, self, varargin{:});
    end
    function self = WeakRef(varargin)
    %WEAKREF 
    %
    %  new_obj = WEAKREF(int dummy)
    %  new_obj = WEAKREF(SharedObject shared)
    %
    %
    %.......
    %
    %::
    %
    %  WEAKREF(SharedObject shared)
    %
    %
    %
    %Construct from a shared object (also implicit type conversion)
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
    %  WEAKREF(int dummy)
    %
    %
    %
    %Default constructor.
    %
    %
    %
    %.............
    %
    %
      self@casadi.SharedObject(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(33, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(34, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
