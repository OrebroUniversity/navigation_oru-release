classdef  StringSerializer < casadi.SerializerBase
    %STRINGSERIALIZER C++ includes: serializer.hpp 
    %
    %
    %
  methods
    function self = StringSerializer(varargin)
    %STRINGSERIALIZER 
    %
    %  new_obj = STRINGSERIALIZER()
    %
    %
      self@casadi.SerializerBase(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1125, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1126, self);
          self.SwigClear();
        end
    end
    function varargout = encode(self,varargin)
    %ENCODE Returns a string that holds the serialized objects.
    %
    %  char = ENCODE(self)
    %
    %
    %As a side effect, this method clears the internal buffer
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1127, self, varargin{:});
    end
  end
  methods(Static)
  end
end
