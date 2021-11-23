classdef  GenSX < casadi.GenericMatrixCommon & casadi.SparsityInterfaceCommon
    %GENSX 
    %
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = nnz(self,varargin)
    %NNZ Get the number of (structural) non-zero elements.
    %
    %  int = NNZ(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(293, self, varargin{:});
    end
    function varargout = nnz_lower(self,varargin)
    %NNZ_LOWER Get the number of non-zeros in the lower triangular half.
    %
    %  int = NNZ_LOWER(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(294, self, varargin{:});
    end
    function varargout = nnz_upper(self,varargin)
    %NNZ_UPPER Get the number of non-zeros in the upper triangular half.
    %
    %  int = NNZ_UPPER(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(295, self, varargin{:});
    end
    function varargout = nnz_diag(self,varargin)
    %NNZ_DIAG Get get the number of non-zeros on the diagonal.
    %
    %  int = NNZ_DIAG(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(296, self, varargin{:});
    end
    function varargout = numel(self,varargin)
    %NUMEL Get the number of elements.
    %
    %  int = NUMEL(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(297, self, varargin{:});
    end
    function varargout = size1(self,varargin)
    %SIZE1 Get the first dimension (i.e. number of rows)
    %
    %  int = SIZE1(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(298, self, varargin{:});
    end
    function varargout = rows(self,varargin)
    %ROWS Get the number of rows, Octave-style syntax.
    %
    %  int = ROWS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(299, self, varargin{:});
    end
    function varargout = size2(self,varargin)
    %SIZE2 Get the second dimension (i.e. number of columns)
    %
    %  int = SIZE2(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(300, self, varargin{:});
    end
    function varargout = columns(self,varargin)
    %COLUMNS Get the number of columns, Octave-style syntax.
    %
    %  int = COLUMNS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(301, self, varargin{:});
    end
    function varargout = dim(self,varargin)
    %DIM Get string representation of dimensions. The representation is e.g. "4x5"
    %
    %  char = DIM(self, bool with_nz)
    %
    %or "4x5,10nz".
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(302, self, varargin{:});
    end
    function varargout = size(self,varargin)
    %SIZE Get the size along a particular dimensions.
    %
    %  [int,int] = SIZE(self)
    %  int = SIZE(self, int axis)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  SIZE(self)
    %
    %
    %
    %Get the shape.
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
    %  SIZE(self, int axis)
    %
    %
    %
    %Get the size along a particular dimensions.
    %
    %
    %
    %.............
    %
    %
      out = casadiMEX(303, self, varargin{:});
      if nargout<=1
        varargout{1}=out;
      else
        nargoutchk(length(out),length(out))
        for i=1:nargout
          varargout{i} = out(i);
        end
      end
    end
    function varargout = is_empty(self,varargin)
    %IS_EMPTY Check if the sparsity is empty, i.e. if one of the dimensions is zero (or
    %
    %  bool = IS_EMPTY(self, bool both)
    %
    %optionally both dimensions)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(304, self, varargin{:});
    end
    function varargout = is_dense(self,varargin)
    %IS_DENSE Check if the matrix expression is dense.
    %
    %  bool = IS_DENSE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(305, self, varargin{:});
    end
    function varargout = is_scalar(self,varargin)
    %IS_SCALAR Check if the matrix expression is scalar.
    %
    %  bool = IS_SCALAR(self, bool scalar_and_dense)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(306, self, varargin{:});
    end
    function varargout = is_square(self,varargin)
    %IS_SQUARE Check if the matrix expression is square.
    %
    %  bool = IS_SQUARE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(307, self, varargin{:});
    end
    function varargout = is_vector(self,varargin)
    %IS_VECTOR Check if the matrix is a row or column vector.
    %
    %  bool = IS_VECTOR(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(308, self, varargin{:});
    end
    function varargout = is_row(self,varargin)
    %IS_ROW Check if the matrix is a row vector (i.e. size1()==1)
    %
    %  bool = IS_ROW(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(309, self, varargin{:});
    end
    function varargout = is_column(self,varargin)
    %IS_COLUMN Check if the matrix is a column vector (i.e. size2()==1)
    %
    %  bool = IS_COLUMN(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(310, self, varargin{:});
    end
    function varargout = is_triu(self,varargin)
    %IS_TRIU Check if the matrix is upper triangular.
    %
    %  bool = IS_TRIU(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(311, self, varargin{:});
    end
    function varargout = is_tril(self,varargin)
    %IS_TRIL Check if the matrix is lower triangular.
    %
    %  bool = IS_TRIL(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(312, self, varargin{:});
    end
    function varargout = row(self,varargin)
    %ROW Get the sparsity pattern. See the Sparsity class for details.
    %
    %  [int] = ROW(self)
    %  int = ROW(self, int el)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(313, self, varargin{:});
    end
    function varargout = colind(self,varargin)
    %COLIND Get the sparsity pattern. See the Sparsity class for details.
    %
    %  [int] = COLIND(self)
    %  int = COLIND(self, int col)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(314, self, varargin{:});
    end
    function varargout = sparsity(self,varargin)
    %SPARSITY Get the sparsity pattern.
    %
    %  Sparsity = SPARSITY(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(315, self, varargin{:});
    end
    function self = GenSX(varargin)
    %GENSX 
    %
    %  new_obj = GENSX()
    %
    %
      self@casadi.GenericMatrixCommon(SwigRef.Null);
      self@casadi.SparsityInterfaceCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(319, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(320, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
    function varargout = sym(varargin)
    %SYM Create a vector of length r of vectors of length p with nrow-by-ncol
    %
    %  SX = SYM(char name, int nrow, int ncol)
    %  SX = SYM(char name, [int,int] rc)
    %  SX = SYM(char name, Sparsity sp)
    %  {SX} = SYM(char name, Sparsity sp, int p)
    %  {SX} = SYM(char name, int nrow, int ncol, int p)
    %  {{SX}} = SYM(char name, Sparsity sp, int p, int r)
    %  {{SX}} = SYM(char name, int nrow, int ncol, int p, int r)
    %
    %symbolic primitives.
    %
    %
    %
    %.......
    %
    %::
    %
    %  SYM(char name, [int,int] rc)
    %
    %
    %
    %Construct a symbolic primitive with given dimensions.
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
    %  SYM(char name, int nrow, int ncol, int p)
    %
    %
    %
    %Create a vector of length p with nrow-by-ncol symbolic primitives.
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
    %  SYM(char name, Sparsity sp, int p, int r)
    %
    %
    %
    %Create a vector of length r of vectors of length p with symbolic primitives
    %with given sparsity.
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
    %  SYM(char name, int nrow, int ncol, int p, int r)
    %
    %
    %
    %Create a vector of length r of vectors of length p with nrow-by-ncol
    %symbolic primitives.
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
    %  SYM(char name, Sparsity sp)
    %
    %
    %
    %Create symbolic primitive with a given sparsity pattern.
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
    %  SYM(char name, int nrow, int ncol)
    %
    %
    %
    %Create an nrow-by-ncol symbolic primitive.
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
    %  SYM(char name, Sparsity sp, int p)
    %
    %
    %
    %Create a vector of length p with with matrices with symbolic primitives of
    %given sparsity.
    %
    %
    %
    %.............
    %
    %
     [varargout{1:nargout}] = casadiMEX(316, varargin{:});
    end
    function varargout = zeros(varargin)
    %ZEROS Create a dense matrix or a matrix with specified sparsity with all entries
    %
    %  SX = ZEROS(int nrow, int ncol)
    %  SX = ZEROS([int,int] rc)
    %  SX = ZEROS(Sparsity sp)
    %
    %zero.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(317, varargin{:});
    end
    function varargout = ones(varargin)
    %ONES Create a dense matrix or a matrix with specified sparsity with all entries
    %
    %  SX = ONES(int nrow, int ncol)
    %  SX = ONES([int,int] rc)
    %  SX = ONES(Sparsity sp)
    %
    %one.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(318, varargin{:});
    end
  end
end
