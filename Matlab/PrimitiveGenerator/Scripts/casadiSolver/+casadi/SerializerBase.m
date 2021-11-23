classdef  SerializerBase < SwigRef
    %SERIALIZERBASE C++ includes: serializer.hpp 
    %
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1076, self);
          self.SwigClear();
        end
    end
    function varargout = pack(self,varargin)
    %PACK 
    %
    %  PACK(self, Linsol e)
    %  PACK(self, std::vector< casadi::Linsol,std::allocator< casadi::Linsol > > const & e)
    %  PACK(self, int e)
    %  PACK(self, double e)
    %  PACK(self, {Sparsity} e)
    %  PACK(self, Sparsity e)
    %  PACK(self, [double] e)
    %  PACK(self, [int] e)
    %  PACK(self, {char} e)
    %  PACK(self, DM e)
    %  PACK(self, {DM} e)
    %  PACK(self, SX e)
    %  PACK(self, {SX} e)
    %  PACK(self, MX e)
    %  PACK(self, {MX} e)
    %  PACK(self, char e)
    %  PACK(self, {Function} e)
    %  PACK(self, Function e)
    %  PACK(self, {GenericType} e)
    %  PACK(self, GenericType e)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1077, self, varargin{:});
    end
    function varargout = connect(self,varargin)
    %CONNECT 
    %
    %  CONNECT(self, DeserializerBase s)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1079, self, varargin{:});
    end
    function varargout = reset(self,varargin)
    %RESET 
    %
    %  RESET(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1080, self, varargin{:});
    end
    function self = SerializerBase(varargin)
    %SERIALIZERBASE C++ includes: serializer.hpp 
    %
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        error('No matching constructor');
      end
    end
  end
  methods(Static)
    function v = internal_SERIALIZED_SPARSITY()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 134);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_MX()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 135);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_DM()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 136);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_SX()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 137);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_LINSOL()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 138);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_FUNCTION()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 139);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_GENERICTYPE()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 140);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_INT()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 141);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_DOUBLE()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 142);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_STRING()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 143);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_SPARSITY_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 144);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_MX_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 145);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_DM_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 146);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_SX_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 147);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_LINSOL_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 148);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_FUNCTION_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 149);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_GENERICTYPE_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 150);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_INT_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 151);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_DOUBLE_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 152);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_STRING_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 153);
      end
      v = vInitialized;
    end
    function varargout = type_to_string(varargin)
    %TYPE_TO_STRING 
    %
    %  char = TYPE_TO_STRING(casadi::SerializerBase::SerializationType type)
    %
    %
     [varargout{1:nargout}] = casadiMEX(1078, varargin{:});
    end
  end
end
