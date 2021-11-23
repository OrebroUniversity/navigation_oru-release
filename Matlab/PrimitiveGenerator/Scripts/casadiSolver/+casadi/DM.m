classdef  DM < casadi.MatrixCommon & casadi.GenericExpressionCommon & casadi.GenDM & casadi.PrintableCommon
    %DM 
    %
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = sanity_check(self,varargin)
    %SANITY_CHECK [DEPRECATED] Correctness is checked during construction
    %
    %  SANITY_CHECK(self, bool complete)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(425, self, varargin{:});
    end
    function varargout = has_nz(self,varargin)
    %HAS_NZ Returns true if the matrix has a non-zero at location rr, cc.
    %
    %  bool = HAS_NZ(self, int rr, int cc)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(426, self, varargin{:});
    end
    function varargout = nonzero(self,varargin)
    %NONZERO Returns the truth value of a Matrix.
    %
    %  bool = NONZERO(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(427, self, varargin{:});
    end
    function varargout = get(self,varargin)
    %GET 
    %
    %  DM = GET(self, bool ind1, Sparsity sp)
    %  DM = GET(self, bool ind1, Slice rr)
    %  DM = GET(self, bool ind1, IM rr)
    %  DM = GET(self, bool ind1, Slice rr, Slice cc)
    %  DM = GET(self, bool ind1, Slice rr, IM cc)
    %  DM = GET(self, bool ind1, IM rr, Slice cc)
    %  DM = GET(self, bool ind1, IM rr, IM cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(428, self, varargin{:});
    end
    function varargout = set(self,varargin)
    %SET 
    %
    %  SET(self, DM m, bool ind1, Sparsity sp)
    %  SET(self, DM m, bool ind1, Slice rr)
    %  SET(self, DM m, bool ind1, IM rr)
    %  SET(self, DM m, bool ind1, Slice rr, Slice cc)
    %  SET(self, DM m, bool ind1, Slice rr, IM cc)
    %  SET(self, DM m, bool ind1, IM rr, Slice cc)
    %  SET(self, DM m, bool ind1, IM rr, IM cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(429, self, varargin{:});
    end
    function varargout = get_nz(self,varargin)
    %GET_NZ 
    %
    %  DM = GET_NZ(self, bool ind1, Slice k)
    %  DM = GET_NZ(self, bool ind1, IM k)
    %
    %
      [varargout{1:nargout}] = casadiMEX(430, self, varargin{:});
    end
    function varargout = set_nz(self,varargin)
    %SET_NZ 
    %
    %  SET_NZ(self, DM m, bool ind1, Slice k)
    %  SET_NZ(self, DM m, bool ind1, IM k)
    %
    %
      [varargout{1:nargout}] = casadiMEX(431, self, varargin{:});
    end
    function varargout = uplus(self,varargin)
    %UPLUS 
    %
    %  DM = UPLUS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(432, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
    %UMINUS 
    %
    %  DM = UMINUS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(433, self, varargin{:});
    end
    function varargout = printme(self,varargin)
    %PRINTME 
    %
    %  DM = PRINTME(self, DM y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(439, self, varargin{:});
    end
    function varargout = T(self,varargin)
    %T Transpose the matrix.
    %
    %  DM = T(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(440, self, varargin{:});
    end
    function varargout = print_split(self,varargin)
    %PRINT_SPLIT Get strings corresponding to the nonzeros and the interdependencies.
    %
    %  [{char} OUTPUT, {char} OUTPUT] = PRINT_SPLIT(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(446, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP Print a representation of the object.
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(447, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR Get string representation.
    %
    %  char = STR(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(448, self, varargin{:});
    end
    function varargout = print_scalar(self,varargin)
    %PRINT_SCALAR Print scalar.
    %
    %  std::ostream & = PRINT_SCALAR(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(449, self, varargin{:});
    end
    function varargout = print_vector(self,varargin)
    %PRINT_VECTOR Print vector-style.
    %
    %  std::ostream & = PRINT_VECTOR(self, bool truncate)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(450, self, varargin{:});
    end
    function varargout = print_dense(self,varargin)
    %PRINT_DENSE Print dense matrix-stype.
    %
    %  std::ostream & = PRINT_DENSE(self, bool truncate)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(451, self, varargin{:});
    end
    function varargout = print_sparse(self,varargin)
    %PRINT_SPARSE Print sparse matrix style.
    %
    %  std::ostream & = PRINT_SPARSE(self, bool truncate)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(452, self, varargin{:});
    end
    function varargout = clear(self,varargin)
    %CLEAR 
    %
    %  CLEAR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(453, self, varargin{:});
    end
    function varargout = resize(self,varargin)
    %RESIZE 
    %
    %  RESIZE(self, int nrow, int ncol)
    %
    %
      [varargout{1:nargout}] = casadiMEX(454, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
    %RESERVE 
    %
    %  RESERVE(self, int nnz)
    %  RESERVE(self, int nnz, int ncol)
    %
    %
      [varargout{1:nargout}] = casadiMEX(455, self, varargin{:});
    end
    function varargout = erase(self,varargin)
    %ERASE Erase a submatrix (leaving structural zeros in its place) Erase elements of
    %
    %  ERASE(self, [int] rr, bool ind1)
    %  ERASE(self, [int] rr, [int] cc, bool ind1)
    %
    %a matrix.
    %
    %
    %
    %.......
    %
    %::
    %
    %  ERASE(self, [int] rr, bool ind1)
    %
    %
    %
    %Erase a submatrix (leaving structural zeros in its place) Erase elements of
    %a matrix.
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
    %  ERASE(self, [int] rr, [int] cc, bool ind1)
    %
    %
    %
    %Erase a submatrix (leaving structural zeros in its place) Erase rows and/or
    %columns of a matrix.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(456, self, varargin{:});
    end
    function varargout = remove(self,varargin)
    %REMOVE Remove columns and rows Remove/delete rows and/or columns of a matrix.
    %
    %  REMOVE(self, [int] rr, [int] cc)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(457, self, varargin{:});
    end
    function varargout = enlarge(self,varargin)
    %ENLARGE Enlarge matrix Make the matrix larger by inserting empty rows and columns,
    %
    %  ENLARGE(self, int nrow, int ncol, [int] rr, [int] cc, bool ind1)
    %
    %keeping the existing non-zeros.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(458, self, varargin{:});
    end
    function varargout = sparsity(self,varargin)
    %SPARSITY Get an owning reference to the sparsity pattern.
    %
    %  Sparsity = SPARSITY(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(459, self, varargin{:});
    end
    function varargout = element_hash(self,varargin)
    %ELEMENT_HASH 
    %
    %  int = ELEMENT_HASH(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(464, self, varargin{:});
    end
    function varargout = is_regular(self,varargin)
    %IS_REGULAR 
    %
    %  bool = IS_REGULAR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(465, self, varargin{:});
    end
    function varargout = is_smooth(self,varargin)
    %IS_SMOOTH 
    %
    %  bool = IS_SMOOTH(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(466, self, varargin{:});
    end
    function varargout = is_leaf(self,varargin)
    %IS_LEAF 
    %
    %  bool = IS_LEAF(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(467, self, varargin{:});
    end
    function varargout = is_commutative(self,varargin)
    %IS_COMMUTATIVE 
    %
    %  bool = IS_COMMUTATIVE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(468, self, varargin{:});
    end
    function varargout = is_symbolic(self,varargin)
    %IS_SYMBOLIC 
    %
    %  bool = IS_SYMBOLIC(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(469, self, varargin{:});
    end
    function varargout = is_valid_input(self,varargin)
    %IS_VALID_INPUT 
    %
    %  bool = IS_VALID_INPUT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(470, self, varargin{:});
    end
    function varargout = has_duplicates(self,varargin)
    %HAS_DUPLICATES 
    %
    %  bool = HAS_DUPLICATES(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(471, self, varargin{:});
    end
    function varargout = reset_input(self,varargin)
    %RESET_INPUT 
    %
    %  RESET_INPUT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(472, self, varargin{:});
    end
    function varargout = is_constant(self,varargin)
    %IS_CONSTANT Check if the matrix is constant (note that false negative answers are
    %
    %  bool = IS_CONSTANT(self)
    %
    %possible)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(473, self, varargin{:});
    end
    function varargout = is_integer(self,varargin)
    %IS_INTEGER Check if the matrix is integer-valued (note that false negative answers are
    %
    %  bool = IS_INTEGER(self)
    %
    %possible)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(474, self, varargin{:});
    end
    function varargout = is_zero(self,varargin)
    %IS_ZERO check if the matrix is 0 (note that false negative answers are possible)
    %
    %  bool = IS_ZERO(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(475, self, varargin{:});
    end
    function varargout = is_one(self,varargin)
    %IS_ONE check if the matrix is 1 (note that false negative answers are possible)
    %
    %  bool = IS_ONE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(476, self, varargin{:});
    end
    function varargout = is_minus_one(self,varargin)
    %IS_MINUS_ONE check if the matrix is -1 (note that false negative answers are possible)
    %
    %  bool = IS_MINUS_ONE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(477, self, varargin{:});
    end
    function varargout = is_eye(self,varargin)
    %IS_EYE check if the matrix is an identity matrix (note that false negative answers
    %
    %  bool = IS_EYE(self)
    %
    %are possible)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(478, self, varargin{:});
    end
    function varargout = op(self,varargin)
    %OP 
    %
    %  int = OP(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(479, self, varargin{:});
    end
    function varargout = is_op(self,varargin)
    %IS_OP 
    %
    %  bool = IS_OP(self, int op)
    %
    %
      [varargout{1:nargout}] = casadiMEX(480, self, varargin{:});
    end
    function varargout = has_zeros(self,varargin)
    %HAS_ZEROS Check if the matrix has any zero entries which are not structural zeros.
    %
    %  bool = HAS_ZEROS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(481, self, varargin{:});
    end
    function varargout = nonzeros(self,varargin)
    %NONZEROS Get all nonzeros.
    %
    %  [double] = NONZEROS(self)
    %
    %
    %Implementation of Matrix::get_nonzeros (in public API)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(482, self, varargin{:});
    end
    function varargout = elements(self,varargin)
    %ELEMENTS Get all elements.
    %
    %  [double] = ELEMENTS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(483, self, varargin{:});
    end
    function varargout = to_double(self,varargin)
    %TO_DOUBLE 
    %
    %  double = TO_DOUBLE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(484, self, varargin{:});
    end
    function varargout = to_int(self,varargin)
    %TO_INT 
    %
    %  int = TO_INT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(485, self, varargin{:});
    end
    function varargout = name(self,varargin)
    %NAME 
    %
    %  char = NAME(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(486, self, varargin{:});
    end
    function varargout = dep(self,varargin)
    %DEP 
    %
    %  DM = DEP(self, int ch)
    %
    %
      [varargout{1:nargout}] = casadiMEX(487, self, varargin{:});
    end
    function varargout = n_dep(self,varargin)
    %N_DEP 
    %
    %  int = N_DEP(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(488, self, varargin{:});
    end
    function varargout = export_code(self,varargin)
    %EXPORT_CODE Export matrix in specific language.
    %
    %  std::ostream & = EXPORT_CODE(self, char lang, struct options)
    %
    %
    %lang: only 'matlab' supported for now
    %
    %::
    %
    %  * options:
    %  *   inline: Indicates if you want everything on a single line (default: False)
    %  *   name: Name of exported variable (default: 'm')
    %  *   indent_level: Level of indentation (default: 0)
    %  *   spoof_zero: Replace numerical zero by a 1e-200 (default: false)
    %  *               might be needed for matlab sparse construct,
    %  *               which doesn't allow numerical zero
    %  * 
    %
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(494, self, varargin{:});
    end
    function varargout = info(self,varargin)
    %INFO 
    %
    %  struct = INFO(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(495, self, varargin{:});
    end
    function varargout = serialize(self,varargin)
    %SERIALIZE Serialize an object.
    %
    %  char = SERIALIZE(self)
    %  SERIALIZE(self, casadi::SerializingStream & s)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  SERIALIZE(self, casadi::SerializingStream & s)
    %
    %
    %
    %Serialize an object.
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
    %  SERIALIZE(self)
    %
    %
    %
    %Serialize.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(496, self, varargin{:});
    end
    function varargout = to_file(self,varargin)
    %TO_FILE Export numerical matrix to file
    %
    %  TO_FILE(self, char filename, char format)
    %
    %
    %Supported formats:
    %
    %
    %
    %::
    %
    %  *   - .mtx   Matrix Market (sparse)
    %  *   - .txt   Ascii full precision representation (sparse)
    %  *            Whitespace separated, aligned.
    %  *            Comments with # % or /
    %  *            Uses C locale
    %  *            Structural zeros represented by 00
    %  *            Does not scale well for large sparse matrices
    %  * 
    %
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(498, self, varargin{:});
    end
    function self = DM(varargin)
    %DM 
    %
    %  new_obj = DM()
    %  new_obj = DM(Sparsity sp)
    %  new_obj = DM(double val)
    %  new_obj = DM(DM m)
    %  new_obj = DM(int nrow, int ncol)
    %  new_obj = DM(Sparsity sp, DM d)
    %
    %
      self@casadi.MatrixCommon(SwigRef.Null);
      self@casadi.GenericExpressionCommon(SwigRef.Null);
      self@casadi.GenDM(SwigRef.Null);
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(500, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = assign(self,varargin)
    %ASSIGN 
    %
    %  ASSIGN(self, DM rhs)
    %
    %
      [varargout{1:nargout}] = casadiMEX(501, self, varargin{:});
    end
    function varargout = paren(self,varargin)
    %PAREN 
    %
    %  DM = PAREN(self, Sparsity sp)
    %  DM = PAREN(self, IM rr)
    %  DM = PAREN(self, char rr)
    %  DM = PAREN(self, IM rr, IM cc)
    %  DM = PAREN(self, IM rr, char cc)
    %  DM = PAREN(self, char rr, IM cc)
    %  DM = PAREN(self, char rr, char cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(502, self, varargin{:});
    end
    function varargout = paren_asgn(self,varargin)
    %PAREN_ASGN 
    %
    %  PAREN_ASGN(self, DM m, Sparsity sp)
    %  PAREN_ASGN(self, DM m, IM rr)
    %  PAREN_ASGN(self, DM m, char rr)
    %  PAREN_ASGN(self, DM m, IM rr, IM cc)
    %  PAREN_ASGN(self, DM m, IM rr, char cc)
    %  PAREN_ASGN(self, DM m, char rr, IM cc)
    %  PAREN_ASGN(self, DM m, char rr, char cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(503, self, varargin{:});
    end
    function varargout = brace(self,varargin)
    %BRACE 
    %
    %  DM = BRACE(self, IM rr)
    %  DM = BRACE(self, char rr)
    %
    %
      [varargout{1:nargout}] = casadiMEX(504, self, varargin{:});
    end
    function varargout = setbrace(self,varargin)
    %SETBRACE 
    %
    %  SETBRACE(self, DM m, IM rr)
    %  SETBRACE(self, DM m, char rr)
    %
    %
      [varargout{1:nargout}] = casadiMEX(505, self, varargin{:});
    end
    function varargout = end(self,varargin)
    %END 
    %
    %  int = END(self, int i, int n)
    %
    %
      [varargout{1:nargout}] = casadiMEX(506, self, varargin{:});
    end
    function varargout = numel(self,varargin)
    %NUMEL Get the number of elements.
    %
    %  int = NUMEL(self)
    %  int = NUMEL(self, int k)
    %  int = NUMEL(self, [int] k)
    %  int = NUMEL(self, char rr)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  NUMEL(self)
    %
    %
    %
    %Get the number of elements.
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
    %  NUMEL(self, int k)
    %  NUMEL(self, [int] k)
    %  NUMEL(self, char rr)
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(507, self, varargin{:});
    end
    function varargout = ctranspose(self,varargin)
    %CTRANSPOSE 
    %
    %  DM = CTRANSPOSE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(508, self, varargin{:});
    end
    function varargout = full(self,varargin)
    %FULL 
    %
    %  mxArray * = FULL(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(509, self, varargin{:});
    end
    function varargout = sparse(self,varargin)
    %SPARSE 
    %
    %  mxArray * = SPARSE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(510, self, varargin{:});
    end

     function s = saveobj(obj)
        try
            s.serialization = obj.serialize();
        catch exception
            warning(['Serializing of CasADi DM failed:' getReport(exception) ]);
            s = struct;
        end
     end
      function delete(self)
        if self.swigPtr
          casadiMEX(511, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
    function varargout = binary(varargin)
    %BINARY 
    %
    %  DM = BINARY(int op, DM x, DM y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(434, varargin{:});
    end
    function varargout = unary(varargin)
    %UNARY 
    %
    %  DM = UNARY(int op, DM x)
    %
    %
     [varargout{1:nargout}] = casadiMEX(435, varargin{:});
    end
    function varargout = scalar_matrix(varargin)
    %SCALAR_MATRIX 
    %
    %  DM = SCALAR_MATRIX(int op, DM x, DM y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(436, varargin{:});
    end
    function varargout = matrix_scalar(varargin)
    %MATRIX_SCALAR 
    %
    %  DM = MATRIX_SCALAR(int op, DM x, DM y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(437, varargin{:});
    end
    function varargout = matrix_matrix(varargin)
    %MATRIX_MATRIX 
    %
    %  DM = MATRIX_MATRIX(int op, DM x, DM y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(438, varargin{:});
    end
    function varargout = set_max_depth(varargin)
    %SET_MAX_DEPTH 
    %
    %  SET_MAX_DEPTH(int eq_depth)
    %
    %
     [varargout{1:nargout}] = casadiMEX(441, varargin{:});
    end
    function varargout = get_max_depth(varargin)
    %GET_MAX_DEPTH 
    %
    %  int = GET_MAX_DEPTH()
    %
    %
     [varargout{1:nargout}] = casadiMEX(442, varargin{:});
    end
    function varargout = get_input(varargin)
    %GET_INPUT 
    %
    %  {DM} = GET_INPUT(Function f)
    %
    %
     [varargout{1:nargout}] = casadiMEX(443, varargin{:});
    end
    function varargout = get_free(varargin)
    %GET_FREE 
    %
    %  {DM} = GET_FREE(Function f)
    %
    %
     [varargout{1:nargout}] = casadiMEX(444, varargin{:});
    end
    function varargout = type_name(varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME()
    %
    %
     [varargout{1:nargout}] = casadiMEX(445, varargin{:});
    end
    function varargout = triplet(varargin)
    %TRIPLET 
    %
    %  DM = TRIPLET([int] row, [int] col, DM d)
    %  DM = TRIPLET([int] row, [int] col, DM d, [int,int] rc)
    %  DM = TRIPLET([int] row, [int] col, DM d, int nrow, int ncol)
    %
    %
     [varargout{1:nargout}] = casadiMEX(460, varargin{:});
    end
    function varargout = inf(varargin)
    %INF create a matrix with all inf
    %
    %  DM = INF(int nrow, int ncol)
    %  DM = INF([int,int] rc)
    %  DM = INF(Sparsity sp)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(461, varargin{:});
    end
    function varargout = nan(varargin)
    %NAN create a matrix with all nan
    %
    %  DM = NAN(int nrow, int ncol)
    %  DM = NAN([int,int] rc)
    %  DM = NAN(Sparsity sp)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(462, varargin{:});
    end
    function varargout = eye(varargin)
    %EYE 
    %
    %  DM = EYE(int n)
    %
    %
     [varargout{1:nargout}] = casadiMEX(463, varargin{:});
    end
    function varargout = set_precision(varargin)
    %SET_PRECISION Set the 'precision, width & scientific' used in printing and serializing to
    %
    %  SET_PRECISION(int precision)
    %
    %streams.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(489, varargin{:});
    end
    function varargout = set_width(varargin)
    %SET_WIDTH Set the 'precision, width & scientific' used in printing and serializing to
    %
    %  SET_WIDTH(int width)
    %
    %streams.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(490, varargin{:});
    end
    function varargout = set_scientific(varargin)
    %SET_SCIENTIFIC Set the 'precision, width & scientific' used in printing and serializing to
    %
    %  SET_SCIENTIFIC(bool scientific)
    %
    %streams.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(491, varargin{:});
    end
    function varargout = rng(varargin)
    %RNG 
    %
    %  RNG(int seed)
    %
    %
     [varargout{1:nargout}] = casadiMEX(492, varargin{:});
    end
    function varargout = rand(varargin)
    %RAND Create a matrix with uniformly distributed random numbers.
    %
    %  DM = RAND(int nrow, int ncol)
    %  DM = RAND([int,int] rc)
    %  DM = RAND(Sparsity sp)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(493, varargin{:});
    end
    function varargout = deserialize(varargin)
    %DESERIALIZE 
    %
    %  DM = DESERIALIZE(std::istream & stream)
    %  DM = DESERIALIZE(casadi::DeserializingStream & s)
    %  DM = DESERIALIZE(char s)
    %
    %
     [varargout{1:nargout}] = casadiMEX(497, varargin{:});
    end
    function varargout = from_file(varargin)
    %FROM_FILE Export numerical matrix to file
    %
    %  DM = FROM_FILE(char filename, char format_hint)
    %
    %
    %Supported formats:
    %
    %
    %
    %::
    %
    %  *   - .mtx   Matrix Market (sparse)
    %  *   - .txt   Ascii full precision representation (sparse)
    %  *            Whitespace separated, aligned.
    %  *            Comments with # % or /
    %  *            Uses C locale
    %  *            Structural zeros represented by 00
    %  *            Does not scale well for large sparse matrices
    %  * 
    %
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(499, varargin{:});
    end

     function obj = loadobj(s)
        try
          if isstruct(s)
             obj = casadi.DM.deserialize(s.serialization);
          else
             obj = s;
          end
        catch exception
            warning(['Serializing of CasADi DM failed:' getReport(exception) ]);
            s = struct;
        end
     end
    end
end
