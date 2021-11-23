classdef  DeserializerBase < SwigRef
    %DESERIALIZERBASE C++ includes: serializer.hpp 
    %
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1081, self);
          self.SwigClear();
        end
    end
    function varargout = internal_pop_type(self,varargin)
    %INTERNAL_POP_TYPE 
    %
    %  casadi::SerializerBase::SerializationType = INTERNAL_POP_TYPE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1082, self, varargin{:});
    end
    function varargout = blind_unpack_sparsity(self,varargin)
    %BLIND_UNPACK_SPARSITY 
    %
    %  Sparsity = BLIND_UNPACK_SPARSITY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1083, self, varargin{:});
    end
    function varargout = blind_unpack_mx(self,varargin)
    %BLIND_UNPACK_MX 
    %
    %  MX = BLIND_UNPACK_MX(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1084, self, varargin{:});
    end
    function varargout = blind_unpack_dm(self,varargin)
    %BLIND_UNPACK_DM 
    %
    %  DM = BLIND_UNPACK_DM(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1085, self, varargin{:});
    end
    function varargout = blind_unpack_sx(self,varargin)
    %BLIND_UNPACK_SX 
    %
    %  SX = BLIND_UNPACK_SX(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1086, self, varargin{:});
    end
    function varargout = blind_unpack_linsol(self,varargin)
    %BLIND_UNPACK_LINSOL 
    %
    %  Linsol = BLIND_UNPACK_LINSOL(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1087, self, varargin{:});
    end
    function varargout = blind_unpack_function(self,varargin)
    %BLIND_UNPACK_FUNCTION 
    %
    %  Function = BLIND_UNPACK_FUNCTION(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1088, self, varargin{:});
    end
    function varargout = blind_unpack_generictype(self,varargin)
    %BLIND_UNPACK_GENERICTYPE 
    %
    %  GenericType = BLIND_UNPACK_GENERICTYPE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1089, self, varargin{:});
    end
    function varargout = blind_unpack_int(self,varargin)
    %BLIND_UNPACK_INT 
    %
    %  int = BLIND_UNPACK_INT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1090, self, varargin{:});
    end
    function varargout = blind_unpack_double(self,varargin)
    %BLIND_UNPACK_DOUBLE 
    %
    %  double = BLIND_UNPACK_DOUBLE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1091, self, varargin{:});
    end
    function varargout = blind_unpack_string(self,varargin)
    %BLIND_UNPACK_STRING 
    %
    %  char = BLIND_UNPACK_STRING(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1092, self, varargin{:});
    end
    function varargout = blind_unpack_sparsity_vector(self,varargin)
    %BLIND_UNPACK_SPARSITY_VECTOR 
    %
    %  {Sparsity} = BLIND_UNPACK_SPARSITY_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1093, self, varargin{:});
    end
    function varargout = blind_unpack_mx_vector(self,varargin)
    %BLIND_UNPACK_MX_VECTOR 
    %
    %  {MX} = BLIND_UNPACK_MX_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1094, self, varargin{:});
    end
    function varargout = blind_unpack_dm_vector(self,varargin)
    %BLIND_UNPACK_DM_VECTOR 
    %
    %  {DM} = BLIND_UNPACK_DM_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1095, self, varargin{:});
    end
    function varargout = blind_unpack_sx_vector(self,varargin)
    %BLIND_UNPACK_SX_VECTOR 
    %
    %  {SX} = BLIND_UNPACK_SX_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1096, self, varargin{:});
    end
    function varargout = blind_unpack_linsol_vector(self,varargin)
    %BLIND_UNPACK_LINSOL_VECTOR 
    %
    %  std::vector< casadi::Linsol,std::allocator< casadi::Linsol > > = BLIND_UNPACK_LINSOL_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1097, self, varargin{:});
    end
    function varargout = blind_unpack_function_vector(self,varargin)
    %BLIND_UNPACK_FUNCTION_VECTOR 
    %
    %  {Function} = BLIND_UNPACK_FUNCTION_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1098, self, varargin{:});
    end
    function varargout = blind_unpack_generictype_vector(self,varargin)
    %BLIND_UNPACK_GENERICTYPE_VECTOR 
    %
    %  {GenericType} = BLIND_UNPACK_GENERICTYPE_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1099, self, varargin{:});
    end
    function varargout = blind_unpack_int_vector(self,varargin)
    %BLIND_UNPACK_INT_VECTOR 
    %
    %  [int] = BLIND_UNPACK_INT_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1100, self, varargin{:});
    end
    function varargout = blind_unpack_double_vector(self,varargin)
    %BLIND_UNPACK_DOUBLE_VECTOR 
    %
    %  [double] = BLIND_UNPACK_DOUBLE_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1101, self, varargin{:});
    end
    function varargout = blind_unpack_string_vector(self,varargin)
    %BLIND_UNPACK_STRING_VECTOR 
    %
    %  {char} = BLIND_UNPACK_STRING_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1102, self, varargin{:});
    end
    function varargout = unpack_sparsity(self,varargin)
    %UNPACK_SPARSITY 
    %
    %  Sparsity = UNPACK_SPARSITY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1103, self, varargin{:});
    end
    function varargout = unpack_mx(self,varargin)
    %UNPACK_MX 
    %
    %  MX = UNPACK_MX(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1104, self, varargin{:});
    end
    function varargout = unpack_dm(self,varargin)
    %UNPACK_DM 
    %
    %  DM = UNPACK_DM(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1105, self, varargin{:});
    end
    function varargout = unpack_sx(self,varargin)
    %UNPACK_SX 
    %
    %  SX = UNPACK_SX(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1106, self, varargin{:});
    end
    function varargout = unpack_linsol(self,varargin)
    %UNPACK_LINSOL 
    %
    %  Linsol = UNPACK_LINSOL(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1107, self, varargin{:});
    end
    function varargout = unpack_function(self,varargin)
    %UNPACK_FUNCTION 
    %
    %  Function = UNPACK_FUNCTION(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1108, self, varargin{:});
    end
    function varargout = unpack_generictype(self,varargin)
    %UNPACK_GENERICTYPE 
    %
    %  GenericType = UNPACK_GENERICTYPE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1109, self, varargin{:});
    end
    function varargout = unpack_int(self,varargin)
    %UNPACK_INT 
    %
    %  int = UNPACK_INT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1110, self, varargin{:});
    end
    function varargout = unpack_double(self,varargin)
    %UNPACK_DOUBLE 
    %
    %  double = UNPACK_DOUBLE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1111, self, varargin{:});
    end
    function varargout = unpack_string(self,varargin)
    %UNPACK_STRING 
    %
    %  char = UNPACK_STRING(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1112, self, varargin{:});
    end
    function varargout = unpack_sparsity_vector(self,varargin)
    %UNPACK_SPARSITY_VECTOR 
    %
    %  {Sparsity} = UNPACK_SPARSITY_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1113, self, varargin{:});
    end
    function varargout = unpack_mx_vector(self,varargin)
    %UNPACK_MX_VECTOR 
    %
    %  {MX} = UNPACK_MX_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1114, self, varargin{:});
    end
    function varargout = unpack_dm_vector(self,varargin)
    %UNPACK_DM_VECTOR 
    %
    %  {DM} = UNPACK_DM_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1115, self, varargin{:});
    end
    function varargout = unpack_sx_vector(self,varargin)
    %UNPACK_SX_VECTOR 
    %
    %  {SX} = UNPACK_SX_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1116, self, varargin{:});
    end
    function varargout = unpack_linsol_vector(self,varargin)
    %UNPACK_LINSOL_VECTOR 
    %
    %  std::vector< casadi::Linsol,std::allocator< casadi::Linsol > > = UNPACK_LINSOL_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1117, self, varargin{:});
    end
    function varargout = unpack_function_vector(self,varargin)
    %UNPACK_FUNCTION_VECTOR 
    %
    %  {Function} = UNPACK_FUNCTION_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1118, self, varargin{:});
    end
    function varargout = unpack_generictype_vector(self,varargin)
    %UNPACK_GENERICTYPE_VECTOR 
    %
    %  {GenericType} = UNPACK_GENERICTYPE_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1119, self, varargin{:});
    end
    function varargout = unpack_int_vector(self,varargin)
    %UNPACK_INT_VECTOR 
    %
    %  [int] = UNPACK_INT_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1120, self, varargin{:});
    end
    function varargout = unpack_double_vector(self,varargin)
    %UNPACK_DOUBLE_VECTOR 
    %
    %  [double] = UNPACK_DOUBLE_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1121, self, varargin{:});
    end
    function varargout = unpack_string_vector(self,varargin)
    %UNPACK_STRING_VECTOR 
    %
    %  {char} = UNPACK_STRING_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1122, self, varargin{:});
    end
    function varargout = connect(self,varargin)
    %CONNECT 
    %
    %  CONNECT(self, SerializerBase s)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1123, self, varargin{:});
    end
    function varargout = reset(self,varargin)
    %RESET 
    %
    %  RESET(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1124, self, varargin{:});
    end

    function out = unpack(self)
      type = casadi.SerializerBase.type_to_string(self.internal_pop_type);
      out = self.(['blind_unpack_' type]);
    end
      function self = DeserializerBase(varargin)
    %DESERIALIZERBASE C++ includes: serializer.hpp 
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
  end
end
