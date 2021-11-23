classdef  FileDeserializer < casadi.DeserializerBase
    %FILEDESERIALIZER C++ includes: serializer.hpp 
    %
    %
    %
  methods
    function self = FileDeserializer(varargin)
    %FILEDESERIALIZER Advanced deserialization of CasADi objects.
    %
    %  new_obj = FILEDESERIALIZER()
    %
    %
    %FileSerializer
    %
    %
    %
      self@casadi.DeserializerBase(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1133, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1134, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
