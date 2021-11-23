classdef  SparsityInterfaceCommon < SwigRef
    %SPARSITYINTERFACECOMMON 
    %
    %   = SPARSITYINTERFACECOMMON()
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = horzcat(varargin)
    %HORZCAT 
    %
    %  Sparsity = HORZCAT({Sparsity} v)
    %  DM = HORZCAT({DM} v)
    %  SX = HORZCAT({SX} v)
    %  MX = HORZCAT({MX} v)
    %
    %
     [varargout{1:nargout}] = casadiMEX(45, varargin);
    end
    function varargout = vertcat(varargin)
    %VERTCAT 
    %
    %  Sparsity = VERTCAT({Sparsity} v)
    %  DM = VERTCAT({DM} v)
    %  SX = VERTCAT({SX} v)
    %  MX = VERTCAT({MX} v)
    %
    %
     [varargout{1:nargout}] = casadiMEX(46, varargin);
    end
    function varargout = horzsplit(varargin)
    %HORZSPLIT 
    %
    %  {Sparsity} = HORZSPLIT(Sparsity v, int incr)
    %  {DM} = HORZSPLIT(DM v, int incr)
    %  {SX} = HORZSPLIT(SX v, int incr)
    %  {MX} = HORZSPLIT(MX v, int incr)
    %  {Sparsity} = HORZSPLIT(Sparsity v, [int] offset)
    %  {DM} = HORZSPLIT(DM v, [int] offset)
    %  {SX} = HORZSPLIT(SX v, [int] offset)
    %  {MX} = HORZSPLIT(MX v, [int] offset)
    %
    %
     [varargout{1:nargout}] = casadiMEX(47, varargin{:});
    end
    function varargout = offset(varargin)
    %OFFSET 
    %
    %  [int] = OFFSET({Sparsity} v, bool vert)
    %  [int] = OFFSET({DM} v, bool vert)
    %  [int] = OFFSET({SX} v, bool vert)
    %  [int] = OFFSET({MX} v, bool vert)
    %
    %
     [varargout{1:nargout}] = casadiMEX(48, varargin{:});
    end
    function varargout = vertsplit(varargin)
    %VERTSPLIT 
    %
    %  {Sparsity} = VERTSPLIT(Sparsity v, int incr)
    %  {DM} = VERTSPLIT(DM v, int incr)
    %  {SX} = VERTSPLIT(SX v, int incr)
    %  {MX} = VERTSPLIT(MX v, int incr)
    %  {Sparsity} = VERTSPLIT(Sparsity v, [int] offset)
    %  {DM} = VERTSPLIT(DM v, [int] offset)
    %  {SX} = VERTSPLIT(SX v, [int] offset)
    %  {MX} = VERTSPLIT(MX v, [int] offset)
    %
    %
     [varargout{1:nargout}] = casadiMEX(49, varargin{:});
    end
    function varargout = blockcat(varargin)
    %BLOCKCAT 
    %
    %  Sparsity = BLOCKCAT(Sparsity A, Sparsity B, Sparsity C, Sparsity D)
    %  DM = BLOCKCAT(DM A, DM B, DM C, DM D)
    %  SX = BLOCKCAT(SX A, SX B, SX C, SX D)
    %  MX = BLOCKCAT(MX A, MX B, MX C, MX D)
    %
    %
     [varargout{1:nargout}] = casadiMEX(50, varargin{:});
    end
    function varargout = blocksplit(varargin)
    %BLOCKSPLIT 
    %
    %  {{Sparsity}} = BLOCKSPLIT(Sparsity x, int vert_incr, int horz_incr)
    %  {{DM}} = BLOCKSPLIT(DM x, int vert_incr, int horz_incr)
    %  {{SX}} = BLOCKSPLIT(SX x, int vert_incr, int horz_incr)
    %  {{MX}} = BLOCKSPLIT(MX x, int vert_incr, int horz_incr)
    %  {{Sparsity}} = BLOCKSPLIT(Sparsity x, [int] vert_offset, [int] horz_offset)
    %  {{DM}} = BLOCKSPLIT(DM x, [int] vert_offset, [int] horz_offset)
    %  {{SX}} = BLOCKSPLIT(SX x, [int] vert_offset, [int] horz_offset)
    %  {{MX}} = BLOCKSPLIT(MX x, [int] vert_offset, [int] horz_offset)
    %
    %
     [varargout{1:nargout}] = casadiMEX(51, varargin{:});
    end
    function varargout = diagcat(varargin)
    %DIAGCAT 
    %
    %  Sparsity = DIAGCAT({Sparsity} A)
    %  DM = DIAGCAT({DM} A)
    %  SX = DIAGCAT({SX} A)
    %  MX = DIAGCAT({MX} A)
    %
    %
     [varargout{1:nargout}] = casadiMEX(52, varargin);
    end
    function varargout = diagsplit(varargin)
    %DIAGSPLIT 
    %
    %  {Sparsity} = DIAGSPLIT(Sparsity x, int incr)
    %  {DM} = DIAGSPLIT(DM x, int incr)
    %  {SX} = DIAGSPLIT(SX x, int incr)
    %  {MX} = DIAGSPLIT(MX x, int incr)
    %  {Sparsity} = DIAGSPLIT(Sparsity x, [int] output_offset)
    %  {DM} = DIAGSPLIT(DM x, [int] output_offset)
    %  {SX} = DIAGSPLIT(SX x, [int] output_offset)
    %  {MX} = DIAGSPLIT(MX x, [int] output_offset)
    %  {Sparsity} = DIAGSPLIT(Sparsity x, int incr1, int incr2)
    %  {Sparsity} = DIAGSPLIT(Sparsity x, [int] output_offset1, [int] output_offset2)
    %  {DM} = DIAGSPLIT(DM x, int incr1, int incr2)
    %  {DM} = DIAGSPLIT(DM x, [int] output_offset1, [int] output_offset2)
    %  {SX} = DIAGSPLIT(SX x, int incr1, int incr2)
    %  {SX} = DIAGSPLIT(SX x, [int] output_offset1, [int] output_offset2)
    %  {MX} = DIAGSPLIT(MX x, int incr1, int incr2)
    %  {MX} = DIAGSPLIT(MX x, [int] output_offset1, [int] output_offset2)
    %
    %
     [varargout{1:nargout}] = casadiMEX(53, varargin{:});
    end
    function varargout = veccat(varargin)
    %VECCAT 
    %
    %  Sparsity = VECCAT({Sparsity} x)
    %  DM = VECCAT({DM} x)
    %  SX = VECCAT({SX} x)
    %  MX = VECCAT({MX} x)
    %
    %
     [varargout{1:nargout}] = casadiMEX(54, varargin);
    end
    function varargout = mtimes(varargin)
    %MTIMES 
    %
    %  Sparsity = MTIMES({Sparsity} args)
    %  DM = MTIMES({DM} args)
    %  SX = MTIMES({SX} args)
    %  MX = MTIMES({MX} args)
    %  Sparsity = MTIMES(Sparsity x, Sparsity y)
    %  DM = MTIMES(DM x, DM y)
    %  SX = MTIMES(SX x, SX y)
    %  MX = MTIMES(MX x, MX y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(55, varargin{:});
    end
    function varargout = mac(varargin)
    %MAC 
    %
    %  Sparsity = MAC(Sparsity X, Sparsity Y, Sparsity Z)
    %  DM = MAC(DM X, DM Y, DM Z)
    %  SX = MAC(SX X, SX Y, SX Z)
    %  MX = MAC(MX X, MX Y, MX Z)
    %
    %
     [varargout{1:nargout}] = casadiMEX(56, varargin{:});
    end
    function varargout = transpose(varargin)
    %TRANSPOSE 
    %
    %  Sparsity = TRANSPOSE(Sparsity X)
    %  DM = TRANSPOSE(DM X)
    %  SX = TRANSPOSE(SX X)
    %  MX = TRANSPOSE(MX X)
    %
    %
     [varargout{1:nargout}] = casadiMEX(57, varargin{:});
    end
    function varargout = vec(varargin)
    %VEC 
    %
    %  Sparsity = VEC(Sparsity a)
    %  DM = VEC(DM a)
    %  SX = VEC(SX a)
    %  MX = VEC(MX a)
    %
    %
     [varargout{1:nargout}] = casadiMEX(58, varargin{:});
    end
    function varargout = reshape(varargin)
    %RESHAPE 
    %
    %  Sparsity = RESHAPE(Sparsity a, [int,int] rc)
    %  Sparsity = RESHAPE(Sparsity a, Sparsity sp)
    %  DM = RESHAPE(DM a, [int,int] rc)
    %  DM = RESHAPE(DM a, Sparsity sp)
    %  SX = RESHAPE(SX a, [int,int] rc)
    %  SX = RESHAPE(SX a, Sparsity sp)
    %  MX = RESHAPE(MX a, [int,int] rc)
    %  MX = RESHAPE(MX a, Sparsity sp)
    %  Sparsity = RESHAPE(Sparsity a, int nrow, int ncol)
    %  DM = RESHAPE(DM a, int nrow, int ncol)
    %  SX = RESHAPE(SX a, int nrow, int ncol)
    %  MX = RESHAPE(MX a, int nrow, int ncol)
    %
    %
     [varargout{1:nargout}] = casadiMEX(59, varargin{:});
    end
    function varargout = sprank(varargin)
    %SPRANK 
    %
    %  int = SPRANK(Sparsity A)
    %  int = SPRANK(DM A)
    %  int = SPRANK(SX A)
    %  int = SPRANK(MX A)
    %
    %
     [varargout{1:nargout}] = casadiMEX(60, varargin{:});
    end
    function varargout = norm_0_mul(varargin)
    %NORM_0_MUL 
    %
    %  int = NORM_0_MUL(Sparsity x, Sparsity y)
    %  int = NORM_0_MUL(DM x, DM y)
    %  int = NORM_0_MUL(SX x, SX y)
    %  int = NORM_0_MUL(MX x, MX y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(61, varargin{:});
    end
    function varargout = triu(varargin)
    %TRIU 
    %
    %  Sparsity = TRIU(Sparsity a, bool includeDiagonal)
    %  DM = TRIU(DM a, bool includeDiagonal)
    %  SX = TRIU(SX a, bool includeDiagonal)
    %  MX = TRIU(MX a, bool includeDiagonal)
    %
    %
     [varargout{1:nargout}] = casadiMEX(62, varargin{:});
    end
    function varargout = tril(varargin)
    %TRIL 
    %
    %  Sparsity = TRIL(Sparsity a, bool includeDiagonal)
    %  DM = TRIL(DM a, bool includeDiagonal)
    %  SX = TRIL(SX a, bool includeDiagonal)
    %  MX = TRIL(MX a, bool includeDiagonal)
    %
    %
     [varargout{1:nargout}] = casadiMEX(63, varargin{:});
    end
    function varargout = kron(varargin)
    %KRON 
    %
    %  Sparsity = KRON(Sparsity a, Sparsity b)
    %  DM = KRON(DM a, DM b)
    %  SX = KRON(SX a, SX b)
    %  MX = KRON(MX a, MX b)
    %
    %
     [varargout{1:nargout}] = casadiMEX(64, varargin{:});
    end
    function varargout = repmat(varargin)
    %REPMAT 
    %
    %  Sparsity = REPMAT(Sparsity A, int n, int m)
    %  Sparsity = REPMAT(Sparsity A, [int,int] rc)
    %  DM = REPMAT(DM A, int n, int m)
    %  DM = REPMAT(DM A, [int,int] rc)
    %  SX = REPMAT(SX A, int n, int m)
    %  SX = REPMAT(SX A, [int,int] rc)
    %  MX = REPMAT(MX A, int n, int m)
    %  MX = REPMAT(MX A, [int,int] rc)
    %
    %
     [varargout{1:nargout}] = casadiMEX(65, varargin{:});
    end
    function varargout = sum2(varargin)
    %SUM2 
    %
    %  Sparsity = SUM2(Sparsity x)
    %  DM = SUM2(DM x)
    %  SX = SUM2(SX x)
    %  MX = SUM2(MX x)
    %
    %
     [varargout{1:nargout}] = casadiMEX(66, varargin{:});
    end
    function varargout = sum1(varargin)
    %SUM1 
    %
    %  Sparsity = SUM1(Sparsity x)
    %  DM = SUM1(DM x)
    %  SX = SUM1(SX x)
    %  MX = SUM1(MX x)
    %
    %
     [varargout{1:nargout}] = casadiMEX(67, varargin{:});
    end
    function varargout = length(varargin)
    %LENGTH 
    %
    %  int = LENGTH(Sparsity v)
    %  int = LENGTH(DM v)
    %  int = LENGTH(SX v)
    %  int = LENGTH(MX v)
    %
    %
     [varargout{1:nargout}] = casadiMEX(68, varargin{:});
    end
    function self = SparsityInterfaceCommon(varargin)
    %SPARSITYINTERFACECOMMON 
    %
    %  new_obj = SPARSITYINTERFACECOMMON()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(69, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(70, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
