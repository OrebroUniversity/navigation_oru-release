classdef  StringDeserializer < casadi.DeserializerBase
    %STRINGDESERIALIZER C++ includes: serializer.hpp 
    %
    %
    %
  methods
    function self = StringDeserializer(varargin)
    %STRINGDESERIALIZER Advanced deserialization of CasADi objects.
    %
    %  new_obj = STRINGDESERIALIZER()
    %
    %
    %StringDeserializer
    %
    %
    %
      self@casadi.DeserializerBase(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1130, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1131, self);
          self.SwigClear();
        end
    end
    function varargout = decode(self,varargin)
    %DECODE Sets the string to deserialize objects from.
    %
    %  DECODE(self, char string)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1132, self, varargin{:});
    end
  end
  methods(Static)
  end
end
