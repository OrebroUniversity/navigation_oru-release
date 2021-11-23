classdef  SX < casadi.MatrixCommon & casadi.GenericExpressionCommon & casadi.GenSX & casadi.PrintableCommon
    %SX 
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
      [varargout{1:nargout}] = casadiMEX(514, self, varargin{:});
    end
    function varargout = has_nz(self,varargin)
    %HAS_NZ Returns true if the matrix has a non-zero at location rr, cc.
    %
    %  bool = HAS_NZ(self, int rr, int cc)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(515, self, varargin{:});
    end
    function varargout = nonzero(self,varargin)
    %NONZERO Returns the truth value of a Matrix.
    %
    %  bool = NONZERO(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(516, self, varargin{:});
    end
    function varargout = get(self,varargin)
    %GET 
    %
    %  SX = GET(self, bool ind1, Sparsity sp)
    %  SX = GET(self, bool ind1, Slice rr)
    %  SX = GET(self, bool ind1, IM rr)
    %  SX = GET(self, bool ind1, Slice rr, Slice cc)
    %  SX = GET(self, bool ind1, Slice rr, IM cc)
    %  SX = GET(self, bool ind1, IM rr, Slice cc)
    %  SX = GET(self, bool ind1, IM rr, IM cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(517, self, varargin{:});
    end
    function varargout = set(self,varargin)
    %SET 
    %
    %  SET(self, SX m, bool ind1, Sparsity sp)
    %  SET(self, SX m, bool ind1, Slice rr)
    %  SET(self, SX m, bool ind1, IM rr)
    %  SET(self, SX m, bool ind1, Slice rr, Slice cc)
    %  SET(self, SX m, bool ind1, Slice rr, IM cc)
    %  SET(self, SX m, bool ind1, IM rr, Slice cc)
    %  SET(self, SX m, bool ind1, IM rr, IM cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(518, self, varargin{:});
    end
    function varargout = get_nz(self,varargin)
    %GET_NZ 
    %
    %  SX = GET_NZ(self, bool ind1, Slice k)
    %  SX = GET_NZ(self, bool ind1, IM k)
    %
    %
      [varargout{1:nargout}] = casadiMEX(519, self, varargin{:});
    end
    function varargout = set_nz(self,varargin)
    %SET_NZ 
    %
    %  SET_NZ(self, SX m, bool ind1, Slice k)
    %  SET_NZ(self, SX m, bool ind1, IM k)
    %
    %
      [varargout{1:nargout}] = casadiMEX(520, self, varargin{:});
    end
    function varargout = uplus(self,varargin)
    %UPLUS 
    %
    %  SX = UPLUS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(521, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
    %UMINUS 
    %
    %  SX = UMINUS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(522, self, varargin{:});
    end
    function varargout = printme(self,varargin)
    %PRINTME 
    %
    %  SX = PRINTME(self, SX y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(528, self, varargin{:});
    end
    function varargout = T(self,varargin)
    %T Transpose the matrix.
    %
    %  SX = T(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(529, self, varargin{:});
    end
    function varargout = print_split(self,varargin)
    %PRINT_SPLIT Get strings corresponding to the nonzeros and the interdependencies.
    %
    %  [{char} OUTPUT, {char} OUTPUT] = PRINT_SPLIT(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(535, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP Print a representation of the object.
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(536, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR Get string representation.
    %
    %  char = STR(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(537, self, varargin{:});
    end
    function varargout = print_scalar(self,varargin)
    %PRINT_SCALAR Print scalar.
    %
    %  std::ostream & = PRINT_SCALAR(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(538, self, varargin{:});
    end
    function varargout = print_vector(self,varargin)
    %PRINT_VECTOR Print vector-style.
    %
    %  std::ostream & = PRINT_VECTOR(self, bool truncate)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(539, self, varargin{:});
    end
    function varargout = print_dense(self,varargin)
    %PRINT_DENSE Print dense matrix-stype.
    %
    %  std::ostream & = PRINT_DENSE(self, bool truncate)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(540, self, varargin{:});
    end
    function varargout = print_sparse(self,varargin)
    %PRINT_SPARSE Print sparse matrix style.
    %
    %  std::ostream & = PRINT_SPARSE(self, bool truncate)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(541, self, varargin{:});
    end
    function varargout = clear(self,varargin)
    %CLEAR 
    %
    %  CLEAR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(542, self, varargin{:});
    end
    function varargout = resize(self,varargin)
    %RESIZE 
    %
    %  RESIZE(self, int nrow, int ncol)
    %
    %
      [varargout{1:nargout}] = casadiMEX(543, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
    %RESERVE 
    %
    %  RESERVE(self, int nnz)
    %  RESERVE(self, int nnz, int ncol)
    %
    %
      [varargout{1:nargout}] = casadiMEX(544, self, varargin{:});
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
      [varargout{1:nargout}] = casadiMEX(545, self, varargin{:});
    end
    function varargout = remove(self,varargin)
    %REMOVE Remove columns and rows Remove/delete rows and/or columns of a matrix.
    %
    %  REMOVE(self, [int] rr, [int] cc)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(546, self, varargin{:});
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
      [varargout{1:nargout}] = casadiMEX(547, self, varargin{:});
    end
    function varargout = sparsity(self,varargin)
    %SPARSITY Get an owning reference to the sparsity pattern.
    %
    %  Sparsity = SPARSITY(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(548, self, varargin{:});
    end
    function varargout = element_hash(self,varargin)
    %ELEMENT_HASH 
    %
    %  int = ELEMENT_HASH(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(553, self, varargin{:});
    end
    function varargout = is_regular(self,varargin)
    %IS_REGULAR 
    %
    %  bool = IS_REGULAR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(554, self, varargin{:});
    end
    function varargout = is_smooth(self,varargin)
    %IS_SMOOTH 
    %
    %  bool = IS_SMOOTH(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(555, self, varargin{:});
    end
    function varargout = is_leaf(self,varargin)
    %IS_LEAF 
    %
    %  bool = IS_LEAF(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(556, self, varargin{:});
    end
    function varargout = is_commutative(self,varargin)
    %IS_COMMUTATIVE 
    %
    %  bool = IS_COMMUTATIVE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(557, self, varargin{:});
    end
    function varargout = is_symbolic(self,varargin)
    %IS_SYMBOLIC 
    %
    %  bool = IS_SYMBOLIC(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(558, self, varargin{:});
    end
    function varargout = is_valid_input(self,varargin)
    %IS_VALID_INPUT 
    %
    %  bool = IS_VALID_INPUT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(559, self, varargin{:});
    end
    function varargout = has_duplicates(self,varargin)
    %HAS_DUPLICATES 
    %
    %  bool = HAS_DUPLICATES(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(560, self, varargin{:});
    end
    function varargout = reset_input(self,varargin)
    %RESET_INPUT 
    %
    %  RESET_INPUT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(561, self, varargin{:});
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
      [varargout{1:nargout}] = casadiMEX(562, self, varargin{:});
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
      [varargout{1:nargout}] = casadiMEX(563, self, varargin{:});
    end
    function varargout = is_zero(self,varargin)
    %IS_ZERO check if the matrix is 0 (note that false negative answers are possible)
    %
    %  bool = IS_ZERO(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(564, self, varargin{:});
    end
    function varargout = is_one(self,varargin)
    %IS_ONE check if the matrix is 1 (note that false negative answers are possible)
    %
    %  bool = IS_ONE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(565, self, varargin{:});
    end
    function varargout = is_minus_one(self,varargin)
    %IS_MINUS_ONE check if the matrix is -1 (note that false negative answers are possible)
    %
    %  bool = IS_MINUS_ONE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(566, self, varargin{:});
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
      [varargout{1:nargout}] = casadiMEX(567, self, varargin{:});
    end
    function varargout = op(self,varargin)
    %OP 
    %
    %  int = OP(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(568, self, varargin{:});
    end
    function varargout = is_op(self,varargin)
    %IS_OP 
    %
    %  bool = IS_OP(self, int op)
    %
    %
      [varargout{1:nargout}] = casadiMEX(569, self, varargin{:});
    end
    function varargout = has_zeros(self,varargin)
    %HAS_ZEROS Check if the matrix has any zero entries which are not structural zeros.
    %
    %  bool = HAS_ZEROS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(570, self, varargin{:});
    end
    function varargout = nonzeros(self,varargin)
    %NONZEROS Get all nonzeros.
    %
    %  {SXElem} = NONZEROS(self)
    %
    %
    %Implementation of Matrix::get_nonzeros (in public API)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(571, self, varargin{:});
    end
    function varargout = elements(self,varargin)
    %ELEMENTS Get all elements.
    %
    %  {SXElem} = ELEMENTS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(572, self, varargin{:});
    end
    function varargout = to_double(self,varargin)
    %TO_DOUBLE 
    %
    %  double = TO_DOUBLE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(573, self, varargin{:});
    end
    function varargout = to_int(self,varargin)
    %TO_INT 
    %
    %  int = TO_INT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(574, self, varargin{:});
    end
    function varargout = name(self,varargin)
    %NAME 
    %
    %  char = NAME(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(575, self, varargin{:});
    end
    function varargout = dep(self,varargin)
    %DEP 
    %
    %  SX = DEP(self, int ch)
    %
    %
      [varargout{1:nargout}] = casadiMEX(576, self, varargin{:});
    end
    function varargout = n_dep(self,varargin)
    %N_DEP 
    %
    %  int = N_DEP(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(577, self, varargin{:});
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
      [varargout{1:nargout}] = casadiMEX(583, self, varargin{:});
    end
    function varargout = info(self,varargin)
    %INFO 
    %
    %  struct = INFO(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(584, self, varargin{:});
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
      [varargout{1:nargout}] = casadiMEX(585, self, varargin{:});
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
      [varargout{1:nargout}] = casadiMEX(587, self, varargin{:});
    end
    function varargout = paren(self,varargin)
    %PAREN 
    %
    %  SX = PAREN(self, Sparsity sp)
    %  SX = PAREN(self, IM rr)
    %  SX = PAREN(self, char rr)
    %  SX = PAREN(self, IM rr, IM cc)
    %  SX = PAREN(self, IM rr, char cc)
    %  SX = PAREN(self, char rr, IM cc)
    %  SX = PAREN(self, char rr, char cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(589, self, varargin{:});
    end
    function varargout = paren_asgn(self,varargin)
    %PAREN_ASGN 
    %
    %  PAREN_ASGN(self, SX m, Sparsity sp)
    %  PAREN_ASGN(self, SX m, IM rr)
    %  PAREN_ASGN(self, SX m, char rr)
    %  PAREN_ASGN(self, SX m, IM rr, IM cc)
    %  PAREN_ASGN(self, SX m, IM rr, char cc)
    %  PAREN_ASGN(self, SX m, char rr, IM cc)
    %  PAREN_ASGN(self, SX m, char rr, char cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(590, self, varargin{:});
    end
    function varargout = brace(self,varargin)
    %BRACE 
    %
    %  SX = BRACE(self, IM rr)
    %  SX = BRACE(self, char rr)
    %
    %
      [varargout{1:nargout}] = casadiMEX(591, self, varargin{:});
    end
    function varargout = setbrace(self,varargin)
    %SETBRACE 
    %
    %  SETBRACE(self, SX m, IM rr)
    %  SETBRACE(self, SX m, char rr)
    %
    %
      [varargout{1:nargout}] = casadiMEX(592, self, varargin{:});
    end
    function varargout = end(self,varargin)
    %END 
    %
    %  int = END(self, int i, int n)
    %
    %
      [varargout{1:nargout}] = casadiMEX(593, self, varargin{:});
    end
    function varargout = numel(self,varargin)
    %NUMEL 
    %
    %  int = NUMEL(self)
    %  int = NUMEL(self, int k)
    %  int = NUMEL(self, [int] k)
    %  int = NUMEL(self, char rr)
    %
    %
      [varargout{1:nargout}] = casadiMEX(594, self, varargin{:});
    end
    function varargout = ctranspose(self,varargin)
    %CTRANSPOSE 
    %
    %  SX = CTRANSPOSE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(595, self, varargin{:});
    end
    function self = SX(varargin)
    %SX 
    %
    %  new_obj = SX()
    %  new_obj = SX(Sparsity sp)
    %  new_obj = SX(double val)
    %  new_obj = SX(SX m)
    %  new_obj = SX(int nrow, int ncol)
    %  new_obj = SX(Sparsity sp, SX d)
    %
    %
      self@casadi.MatrixCommon(SwigRef.Null);
      self@casadi.GenericExpressionCommon(SwigRef.Null);
      self@casadi.GenSX(SwigRef.Null);
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(596, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(597, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
    function varargout = binary(varargin)
    %BINARY 
    %
    %  SX = BINARY(int op, SX x, SX y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(523, varargin{:});
    end
    function varargout = unary(varargin)
    %UNARY 
    %
    %  SX = UNARY(int op, SX x)
    %
    %
     [varargout{1:nargout}] = casadiMEX(524, varargin{:});
    end
    function varargout = scalar_matrix(varargin)
    %SCALAR_MATRIX 
    %
    %  SX = SCALAR_MATRIX(int op, SX x, SX y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(525, varargin{:});
    end
    function varargout = matrix_scalar(varargin)
    %MATRIX_SCALAR 
    %
    %  SX = MATRIX_SCALAR(int op, SX x, SX y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(526, varargin{:});
    end
    function varargout = matrix_matrix(varargin)
    %MATRIX_MATRIX 
    %
    %  SX = MATRIX_MATRIX(int op, SX x, SX y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(527, varargin{:});
    end
    function varargout = set_max_depth(varargin)
    %SET_MAX_DEPTH 
    %
    %  SET_MAX_DEPTH(int eq_depth)
    %
    %
     [varargout{1:nargout}] = casadiMEX(530, varargin{:});
    end
    function varargout = get_max_depth(varargin)
    %GET_MAX_DEPTH 
    %
    %  int = GET_MAX_DEPTH()
    %
    %
     [varargout{1:nargout}] = casadiMEX(531, varargin{:});
    end
    function varargout = get_input(varargin)
    %GET_INPUT 
    %
    %  {SX} = GET_INPUT(Function f)
    %
    %
     [varargout{1:nargout}] = casadiMEX(532, varargin{:});
    end
    function varargout = get_free(varargin)
    %GET_FREE 
    %
    %  {SX} = GET_FREE(Function f)
    %
    %
     [varargout{1:nargout}] = casadiMEX(533, varargin{:});
    end
    function varargout = type_name(varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME()
    %
    %
     [varargout{1:nargout}] = casadiMEX(534, varargin{:});
    end
    function varargout = triplet(varargin)
    %TRIPLET 
    %
    %  SX = TRIPLET([int] row, [int] col, SX d)
    %  SX = TRIPLET([int] row, [int] col, SX d, [int,int] rc)
    %  SX = TRIPLET([int] row, [int] col, SX d, int nrow, int ncol)
    %
    %
     [varargout{1:nargout}] = casadiMEX(549, varargin{:});
    end
    function varargout = inf(varargin)
    %INF create a matrix with all inf
    %
    %  SX = INF(int nrow, int ncol)
    %  SX = INF([int,int] rc)
    %  SX = INF(Sparsity sp)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(550, varargin{:});
    end
    function varargout = nan(varargin)
    %NAN create a matrix with all nan
    %
    %  SX = NAN(int nrow, int ncol)
    %  SX = NAN([int,int] rc)
    %  SX = NAN(Sparsity sp)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(551, varargin{:});
    end
    function varargout = eye(varargin)
    %EYE 
    %
    %  SX = EYE(int n)
    %
    %
     [varargout{1:nargout}] = casadiMEX(552, varargin{:});
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
     [varargout{1:nargout}] = casadiMEX(578, varargin{:});
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
     [varargout{1:nargout}] = casadiMEX(579, varargin{:});
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
     [varargout{1:nargout}] = casadiMEX(580, varargin{:});
    end
    function varargout = rng(varargin)
    %RNG 
    %
    %  RNG(int seed)
    %
    %
     [varargout{1:nargout}] = casadiMEX(581, varargin{:});
    end
    function varargout = rand(varargin)
    %RAND Create a matrix with uniformly distributed random numbers.
    %
    %  SX = RAND(int nrow, int ncol)
    %  SX = RAND([int,int] rc)
    %  SX = RAND(Sparsity sp)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(582, varargin{:});
    end
    function varargout = deserialize(varargin)
    %DESERIALIZE 
    %
    %  SX = DESERIALIZE(std::istream & stream)
    %  SX = DESERIALIZE(casadi::DeserializingStream & s)
    %  SX = DESERIALIZE(char s)
    %
    %
     [varargout{1:nargout}] = casadiMEX(586, varargin{:});
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
     [varargout{1:nargout}] = casadiMEX(588, varargin{:});
    end
  end
end
