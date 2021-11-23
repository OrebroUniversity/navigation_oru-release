classdef  Sparsity < casadi.SharedObject & casadi.SparsityInterfaceCommon & casadi.PrintableCommon
    %SPARSITY General sparsity class.
    %
    %
    %
    %The storage format is a compressed column storage (CCS) format.  In this
    %format, the structural non-zero elements are stored in column-major order,
    %starting from the upper left corner of the matrix and ending in the lower
    %right corner.
    %
    %In addition to the dimension ( size1(), size2()), (i.e. the number of rows
    %and the number of columns respectively), there are also two vectors of
    %integers:
    %
    %"colind" [length size2()+1], which contains the index to the first non-
    %zero element on or after the corresponding column. All the non-zero elements
    %of a particular i are thus the elements with index el that fulfills:
    %colind[i] <= el < colind[i+1].
    %
    %"row" [same length as the number of non-zero elements, nnz()] The rows for
    %each of the structural non-zeros.
    %
    %Note that with this format, it is cheap to loop over all the non-zero
    %elements of a particular column, at constant time per element, but expensive
    %to jump to access a location (i, j).
    %
    %If the matrix is dense, i.e. length(row) == size1()*size2(), the format
    %reduces to standard dense column major format, which allows access to an
    %arbitrary element in constant time.
    %
    %Since the object is reference counted (it inherits from SharedObject),
    %several matrices are allowed to share the same sparsity pattern.
    %
    %The implementations of methods marked as such in this class has been taken
    %from the CSparse package and modified to fit CasADi data structures and
    %separation of sparsity pattern calculation and numerical evaluation. These
    %functions are Copyright(c) Timothy A. Davis, 2006-2009 and licensed as a
    %derivative work under the GNU LGPL
    %
    %See:   Matrix
    %
    %Joel Andersson
    %
    %C++ includes: sparsity.hpp 
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = sanity_check(self,varargin)
    %SANITY_CHECK [DEPRECATED] Correctness of sparsity patterns are checked during
    %
    %  SANITY_CHECK(self, bool complete)
    %
    %construction
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(83, self, varargin{:});
    end
    function varargout = get_diag(self,varargin)
    %GET_DIAG Get the diagonal of the matrix/create a diagonal matrix (mapping will
    %
    %  [Sparsity , [int] OUTPUT] = GET_DIAG(self)
    %
    %contain the nonzero mapping) When the input is square, the diagonal elements
    %are returned. If the input is vector-like, a diagonal matrix is constructed
    %with it.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(84, self, varargin{:});
    end
    function varargout = compress(self,varargin)
    %COMPRESS Compress a sparsity pattern.
    %
    %  [int] = COMPRESS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(85, self, varargin{:});
    end
    function varargout = is_equal(self,varargin)
    %IS_EQUAL 
    %
    %  bool = IS_EQUAL(self, Sparsity y)
    %  bool = IS_EQUAL(self, int nrow, int ncol, [int] colind, [int] row)
    %
    %
      [varargout{1:nargout}] = casadiMEX(86, self, varargin{:});
    end
    function varargout = eq(self,varargin)
    %EQ 
    %
    %  bool = EQ(self, Sparsity y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(87, self, varargin{:});
    end
    function varargout = ne(self,varargin)
    %NE 
    %
    %  bool = NE(self, Sparsity y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(88, self, varargin{:});
    end
    function varargout = is_stacked(self,varargin)
    %IS_STACKED Check if pattern is horizontal repeat of another.
    %
    %  bool = IS_STACKED(self, Sparsity y, int n)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(89, self, varargin{:});
    end
    function varargout = size1(self,varargin)
    %SIZE1 Get the number of rows.
    %
    %  int = SIZE1(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(90, self, varargin{:});
    end
    function varargout = rows(self,varargin)
    %ROWS Get the number of rows, Octave-style syntax.
    %
    %  int = ROWS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(91, self, varargin{:});
    end
    function varargout = size2(self,varargin)
    %SIZE2 Get the number of columns.
    %
    %  int = SIZE2(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(92, self, varargin{:});
    end
    function varargout = columns(self,varargin)
    %COLUMNS Get the number of columns, Octave-style syntax.
    %
    %  int = COLUMNS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(93, self, varargin{:});
    end
    function varargout = numel(self,varargin)
    %NUMEL The total number of elements, including structural zeros, i.e.
    %
    %  int = NUMEL(self)
    %
    %size2()*size1() Beware of overflow.
    %
    %See:   nnz()
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(94, self, varargin{:});
    end
    function varargout = density(self,varargin)
    %DENSITY The percentage of nonzero Equivalent to (100.0 * nnz())/numel(), but avoids
    %
    %  double = DENSITY(self)
    %
    %overflow.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(95, self, varargin{:});
    end
    function varargout = is_empty(self,varargin)
    %IS_EMPTY Check if the sparsity is empty.
    %
    %  bool = IS_EMPTY(self, bool both)
    %
    %
    %A sparsity is considered empty if one of the dimensions is zero (or
    %optionally both dimensions)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(96, self, varargin{:});
    end
    function varargout = nnz(self,varargin)
    %NNZ Get the number of (structural) non-zeros.
    %
    %  int = NNZ(self)
    %
    %
    %See:   numel()
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(97, self, varargin{:});
    end
    function varargout = nnz_upper(self,varargin)
    %NNZ_UPPER Number of non-zeros in the upper triangular half, i.e. the number of
    %
    %  int = NNZ_UPPER(self, bool strictly)
    %
    %elements (i, j) with j>=i.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(98, self, varargin{:});
    end
    function varargout = nnz_lower(self,varargin)
    %NNZ_LOWER Number of non-zeros in the lower triangular half, i.e. the number of
    %
    %  int = NNZ_LOWER(self, bool strictly)
    %
    %elements (i, j) with j<=i.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(99, self, varargin{:});
    end
    function varargout = nnz_diag(self,varargin)
    %NNZ_DIAG Number of non-zeros on the diagonal, i.e. the number of elements (i, j) with
    %
    %  int = NNZ_DIAG(self)
    %
    %j==i.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(100, self, varargin{:});
    end
    function varargout = bw_upper(self,varargin)
    %BW_UPPER Upper half-bandwidth.
    %
    %  int = BW_UPPER(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(101, self, varargin{:});
    end
    function varargout = bw_lower(self,varargin)
    %BW_LOWER Lower half-bandwidth.
    %
    %  int = BW_LOWER(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(102, self, varargin{:});
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
      out = casadiMEX(103, self, varargin{:});
      if nargout<=1
        varargout{1}=out;
      else
        nargoutchk(length(out),length(out))
        for i=1:nargout
          varargout{i} = out(i);
        end
      end
    end
    function varargout = info(self,varargin)
    %INFO Obtain information about sparsity
    %
    %  struct = INFO(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(104, self, varargin{:});
    end
    function varargout = to_file(self,varargin)
    %TO_FILE Export sparsity pattern to file
    %
    %  TO_FILE(self, char filename, char format_hint)
    %
    %
    %Supported formats: .mtx Matrix Market
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(105, self, varargin{:});
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
      [varargout{1:nargout}] = casadiMEX(107, self, varargin{:});
    end
    function varargout = colind(self,varargin)
    %COLIND Get a reference to the colindex of column cc (see class description)
    %
    %  [int] = COLIND(self)
    %  int = COLIND(self, int cc)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  COLIND(self)
    %
    %
    %
    %Get the column index for each column Together with the row-vector, one
    %obtains the sparsity pattern in the column compressed format.
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
    %  COLIND(self, int cc)
    %
    %
    %
    %Get a reference to the colindex of column cc (see class description)
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(109, self, varargin{:});
    end
    function varargout = row(self,varargin)
    %ROW Get the row of a non-zero element.
    %
    %  [int] = ROW(self)
    %  int = ROW(self, int el)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  ROW(self, int el)
    %
    %
    %
    %Get the row of a non-zero element.
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
    %  ROW(self)
    %
    %
    %
    %Get the row for each non-zero entry Together with the column-vector, this
    %vector gives the sparsity of the matrix in sparse triplet format, and
    %together with the colind vector, one obtains the sparsity in column
    %compressed format.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(110, self, varargin{:});
    end
    function varargout = get_col(self,varargin)
    %GET_COL Get the column for each non-zero entry Together with the row-vector, this
    %
    %  [int] = GET_COL(self)
    %
    %vector gives the sparsity of the matrix in sparse triplet format, i.e. the
    %column and row for each non-zero elements.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(111, self, varargin{:});
    end
    function varargout = resize(self,varargin)
    %RESIZE Resize.
    %
    %  RESIZE(self, int nrow, int ncol)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(112, self, varargin{:});
    end
    function varargout = add_nz(self,varargin)
    %ADD_NZ Get the index of a non-zero element Add the element if it does not exist and
    %
    %  int = ADD_NZ(self, int rr, int cc)
    %
    %copy object if it's not unique.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(113, self, varargin{:});
    end
    function varargout = has_nz(self,varargin)
    %HAS_NZ Returns true if the pattern has a non-zero at location rr, cc.
    %
    %  bool = HAS_NZ(self, int rr, int cc)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(114, self, varargin{:});
    end
    function varargout = get_nz(self,varargin)
    %GET_NZ Get the nonzero index for a set of elements The index vector is used both
    %
    %  [int] = GET_NZ(self)
    %  int = GET_NZ(self, int rr, int cc)
    %  [int] = GET_NZ(self, [int] rr, [int] cc)
    %
    %for input and outputs and must be sorted by increasing nonzero index, i.e.
    %column-wise. Elements not found in the sparsity pattern are set to -1.
    %
    %
    %
    %.......
    %
    %::
    %
    %  GET_NZ(self, int rr, int cc)
    %
    %
    %
    %Get the index of an existing non-zero element return -1 if the element does
    %not exist.
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
    %  GET_NZ(self)
    %
    %
    %
    %Get the nonzero index for a set of elements The index vector is used both
    %for input and outputs and must be sorted by increasing nonzero index, i.e.
    %column-wise. Elements not found in the sparsity pattern are set to -1.
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
    %  GET_NZ(self, [int] rr, [int] cc)
    %
    %
    %
    %Get a set of non-zero element return -1 if the element does not exist.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(115, self, varargin{:});
    end
    function varargout = get_lower(self,varargin)
    %GET_LOWER Get nonzeros in lower triangular part.
    %
    %  [int] = GET_LOWER(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(116, self, varargin{:});
    end
    function varargout = get_upper(self,varargin)
    %GET_UPPER Get nonzeros in upper triangular part.
    %
    %  [int] = GET_UPPER(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(117, self, varargin{:});
    end
    function varargout = get_ccs(self,varargin)
    %GET_CCS Get the sparsity in compressed column storage (CCS) format.
    %
    %  [[int] OUTPUT, [int] OUTPUT] = GET_CCS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(118, self, varargin{:});
    end
    function varargout = get_crs(self,varargin)
    %GET_CRS Get the sparsity in compressed row storage (CRS) format.
    %
    %  [[int] OUTPUT, [int] OUTPUT] = GET_CRS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(119, self, varargin{:});
    end
    function varargout = get_triplet(self,varargin)
    %GET_TRIPLET Get the sparsity in sparse triplet format.
    %
    %  [[int] OUTPUT, [int] OUTPUT] = GET_TRIPLET(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(120, self, varargin{:});
    end
    function varargout = sub(self,varargin)
    %SUB Get a set of elements.
    %
    %  [Sparsity , [int] OUTPUT] = SUB(self, [int] rr, Sparsity sp, bool ind1)
    %  [Sparsity , [int] OUTPUT] = SUB(self, [int] rr, [int] cc, bool ind1)
    %
    %
    %Returns the sparsity of the corresponding elements, with a mapping such that
    %submatrix[k] = originalmatrix[mapping[k]]
    %
    %
    %
    %.......
    %
    %::
    %
    %  SUB(self, [int] rr, Sparsity sp, bool ind1)
    %
    %
    %
    %Get a set of elements.
    %
    %Returns the sparsity of the corresponding elements, with a mapping such that
    %submatrix[k] = originalmatrix[mapping[k]]
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
    %  SUB(self, [int] rr, [int] cc, bool ind1)
    %
    %
    %
    %Get a submatrix.
    %
    %Returns the sparsity of the submatrix, with a mapping such that submatrix[k]
    %= originalmatrix[mapping[k]]
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(121, self, varargin{:});
    end
    function varargout = T(self,varargin)
    %T Transpose the matrix.
    %
    %  Sparsity = T(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(122, self, varargin{:});
    end
    function varargout = transpose(self,varargin)
    %TRANSPOSE Transpose the matrix and get the reordering of the non-zero entries.
    %
    %  [Sparsity , [int] OUTPUT] = TRANSPOSE(self, bool invert_mapping)
    %
    %
    %Parameters:
    %-----------
    %
    %mapping:  the non-zeros of the original matrix for each non-zero of the new
    %matrix
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(123, self, varargin{:});
    end
    function varargout = is_transpose(self,varargin)
    %IS_TRANSPOSE Check if the sparsity is the transpose of another.
    %
    %  bool = IS_TRANSPOSE(self, Sparsity y)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(124, self, varargin{:});
    end
    function varargout = is_reshape(self,varargin)
    %IS_RESHAPE Check if the sparsity is a reshape of another.
    %
    %  bool = IS_RESHAPE(self, Sparsity y)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(125, self, varargin{:});
    end
    function varargout = combine(self,varargin)
    %COMBINE Combine two sparsity patterns Returns the new sparsity pattern as well as a
    %
    %  Sparsity = COMBINE(self, Sparsity y, bool f0x_is_zero, bool function0_is_zero)
    %
    %mapping with the same length as the number of non-zero elements The mapping
    %matrix contains the arguments for each nonzero, the first bit indicates if
    %the first argument is nonzero, the second bit indicates if the second
    %argument is nonzero (note that none of, one of or both of the arguments can
    %be nonzero)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(126, self, varargin{:});
    end
    function varargout = unite(self,varargin)
    %UNITE Union of two sparsity patterns.
    %
    %  Sparsity = UNITE(self, Sparsity y)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(127, self, varargin{:});
    end
    function varargout = plus(self,varargin)
    %PLUS 
    %
    %  Sparsity = PLUS(self, Sparsity b)
    %
    %
      [varargout{1:nargout}] = casadiMEX(128, self, varargin{:});
    end
    function varargout = intersect(self,varargin)
    %INTERSECT Intersection of two sparsity patterns Returns the new sparsity pattern as
    %
    %  Sparsity = INTERSECT(self, Sparsity y)
    %
    %well as a mapping with the same length as the number of non-zero elements
    %The value is 1 if the non-zero comes from the first (i.e. this) object, 2 if
    %it is from the second and 3 (i.e. 1 | 2) if from both.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(129, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
    %MTIMES 
    %
    %  Sparsity = MTIMES(self, Sparsity b)
    %
    %
      [varargout{1:nargout}] = casadiMEX(130, self, varargin{:});
    end
    function varargout = is_subset(self,varargin)
    %IS_SUBSET Is subset?
    %
    %  bool = IS_SUBSET(self, Sparsity rhs)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(131, self, varargin{:});
    end
    function varargout = pattern_inverse(self,varargin)
    %PATTERN_INVERSE Take the inverse of a sparsity pattern; flip zeros and non-zeros.
    %
    %  Sparsity = PATTERN_INVERSE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(132, self, varargin{:});
    end
    function varargout = enlarge(self,varargin)
    %ENLARGE Enlarge matrix Make the matrix larger by inserting empty rows and columns,
    %
    %  ENLARGE(self, int nrow, int ncol, [int] rr, [int] cc, bool ind1)
    %
    %keeping the existing non-zeros.
    %
    %For the matrices A to B A(m, n) length(jj)=m , length(ii)=n B(nrow, ncol)
    %
    %A=enlarge(m, n, ii, jj) makes sure that
    %
    %B[jj, ii] == A
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(133, self, varargin{:});
    end
    function varargout = enlargeRows(self,varargin)
    %ENLARGEROWS Enlarge the matrix along the first dimension (i.e. insert rows)
    %
    %  ENLARGEROWS(self, int nrow, [int] rr, bool ind1)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(134, self, varargin{:});
    end
    function varargout = enlargeColumns(self,varargin)
    %ENLARGECOLUMNS Enlarge the matrix along the second dimension (i.e. insert columns)
    %
    %  ENLARGECOLUMNS(self, int ncol, [int] cc, bool ind1)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(135, self, varargin{:});
    end
    function varargout = makeDense(self,varargin)
    %MAKEDENSE Make a patten dense.
    %
    %  [Sparsity , [int] OUTPUT] = MAKEDENSE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(136, self, varargin{:});
    end
    function varargout = erase(self,varargin)
    %ERASE Erase elements of a matrix.
    %
    %  [int] = ERASE(self, [int] rr, bool ind1)
    %  [int] = ERASE(self, [int] rr, [int] cc, bool ind1)
    %
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
    %Erase elements of a matrix.
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
    %Erase rows and/or columns of a matrix.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(137, self, varargin{:});
    end
    function varargout = append(self,varargin)
    %APPEND Append another sparsity patten vertically (NOTE: only efficient if vector)
    %
    %  APPEND(self, Sparsity sp)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(138, self, varargin{:});
    end
    function varargout = appendColumns(self,varargin)
    %APPENDCOLUMNS Append another sparsity patten horizontally.
    %
    %  APPENDCOLUMNS(self, Sparsity sp)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(139, self, varargin{:});
    end
    function varargout = is_scalar(self,varargin)
    %IS_SCALAR Is scalar?
    %
    %  bool = IS_SCALAR(self, bool scalar_and_dense)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(140, self, varargin{:});
    end
    function varargout = is_dense(self,varargin)
    %IS_DENSE Is dense?
    %
    %  bool = IS_DENSE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(141, self, varargin{:});
    end
    function varargout = is_row(self,varargin)
    %IS_ROW Check if the pattern is a row vector (i.e. size1()==1)
    %
    %  bool = IS_ROW(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(142, self, varargin{:});
    end
    function varargout = is_column(self,varargin)
    %IS_COLUMN Check if the pattern is a column vector (i.e. size2()==1)
    %
    %  bool = IS_COLUMN(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(143, self, varargin{:});
    end
    function varargout = is_vector(self,varargin)
    %IS_VECTOR Check if the pattern is a row or column vector.
    %
    %  bool = IS_VECTOR(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(144, self, varargin{:});
    end
    function varargout = is_diag(self,varargin)
    %IS_DIAG Is diagonal?
    %
    %  bool = IS_DIAG(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(145, self, varargin{:});
    end
    function varargout = is_square(self,varargin)
    %IS_SQUARE Is square?
    %
    %  bool = IS_SQUARE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(146, self, varargin{:});
    end
    function varargout = is_symmetric(self,varargin)
    %IS_SYMMETRIC Is symmetric?
    %
    %  bool = IS_SYMMETRIC(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(147, self, varargin{:});
    end
    function varargout = is_triu(self,varargin)
    %IS_TRIU Is upper triangular?
    %
    %  bool = IS_TRIU(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(148, self, varargin{:});
    end
    function varargout = is_tril(self,varargin)
    %IS_TRIL Is lower triangular?
    %
    %  bool = IS_TRIL(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(149, self, varargin{:});
    end
    function varargout = is_singular(self,varargin)
    %IS_SINGULAR Check whether the sparsity-pattern indicates structural singularity.
    %
    %  bool = IS_SINGULAR(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(150, self, varargin{:});
    end
    function varargout = rowsSequential(self,varargin)
    %ROWSSEQUENTIAL Do the rows appear sequentially on each column.
    %
    %  bool = ROWSSEQUENTIAL(self, bool strictly)
    %
    %
    %Parameters:
    %-----------
    %
    %strictly:  if true, then do not allow multiple entries
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(151, self, varargin{:});
    end
    function varargout = removeDuplicates(self,varargin)
    %REMOVEDUPLICATES Remove duplicate entries.
    %
    %  [int] = REMOVEDUPLICATES(self)
    %
    %
    %The same indices will be removed from the mapping vector, which must have
    %the same length as the number of nonzeros
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(152, self, varargin{:});
    end
    function varargout = etree(self,varargin)
    %ETREE Calculate the elimination tree See Direct Methods for Sparse Linear Systems
    %
    %  [int] = ETREE(self, bool ata)
    %
    %by Davis (2006). If the parameter ata is false, the algorithm is equivalent
    %to MATLAB's etree(A), except that the indices are zero- based. If ata is
    %true, the algorithm is equivalent to MATLAB's etree(A, 'col').
    %
    %The implementation is a modified version of cs_etree in CSparse Copyright(c)
    %Timothy A. Davis, 2006-2009 Licensed as a derivative work under the GNU LGPL
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(153, self, varargin{:});
    end
    function varargout = ldl(self,varargin)
    %LDL Symbolic LDL factorization Returns the sparsity pattern of L^T.
    %
    %  [Sparsity , [int] OUTPUT] = LDL(self, bool amd)
    %
    %
    %The implementation is a modified version of LDL Copyright(c) Timothy A.
    %Davis, 2005-2013 Licensed as a derivative work under the GNU LGPL
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(154, self, varargin{:});
    end
    function varargout = qr_sparse(self,varargin)
    %QR_SPARSE Symbolic QR factorization Returns the sparsity pattern of V (compact
    %
    %  [Sparsity OUTPUT, Sparsity OUTPUT, [int] OUTPUT, [int] OUTPUT] = QR_SPARSE(self, bool amd)
    %
    %representation of Q) and R as well as vectors needed for the numerical
    %factorization and solution. The implementation is a modified version of
    %CSparse Copyright(c) Timothy A. Davis, 2006-2009 Licensed as a derivative
    %work under the GNU LGPL.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(155, self, varargin{:});
    end
    function varargout = dfs(self,varargin)
    %DFS Depth-first search on the adjacency graph of the sparsity See Direct Methods
    %
    %  [int , [int] INOUT, [int] INOUT, [bool] INOUT] = DFS(self, int j, int top, [int] pinv)
    %
    %for Sparse Linear Systems by Davis (2006).
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(156, self, varargin{:});
    end
    function varargout = scc(self,varargin)
    %SCC Find the strongly connected components of the bigraph defined by the
    %
    %  [int , [int] OUTPUT, [int] OUTPUT] = SCC(self)
    %
    %sparsity pattern of a square matrix.
    %
    %See Direct Methods for Sparse Linear Systems by Davis (2006). Returns:
    %Number of components
    %
    %Offset for each components (length: 1 + number of components)
    %
    %Indices for each components, component i has indices index[offset[i]], ...,
    %index[offset[i+1]]
    %
    %In the case that the matrix is symmetric, the result has a particular
    %interpretation: Given a symmetric matrix A and n = A.scc(p, r)
    %
    %=> A[p, p] will appear block-diagonal with n blocks and with the indices of
    %the block boundaries to be found in r.
    %
    %The implementation is a modified version of cs_scc in CSparse Copyright(c)
    %Timothy A. Davis, 2006-2009 Licensed as a derivative work under the GNU LGPL
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(157, self, varargin{:});
    end
    function varargout = btf(self,varargin)
    %BTF Calculate the block triangular form (BTF) See Direct Methods for Sparse
    %
    %  [int , [int] OUTPUT, [int] OUTPUT, [int] OUTPUT, [int] OUTPUT, [int] OUTPUT, [int] OUTPUT] = BTF(self)
    %
    %Linear Systems by Davis (2006).
    %
    %The function computes the Dulmage-Mendelsohn decomposition, which allows you
    %to reorder the rows and columns of a matrix to bring it into block
    %triangular form (BTF).
    %
    %It will not consider the distance of off-diagonal elements to the diagonal:
    %there is no guarantee you will get a block-diagonal matrix if you supply a
    %randomly permuted block-diagonal matrix.
    %
    %If your matrix is symmetrical, this method is of limited use; permutation
    %can make it non-symmetric.
    %
    %See:   scc  The implementation is a modified version of cs_dmperm in CSparse
    %Copyright(c) Timothy A. Davis, 2006-2009 Licensed as a derivative work under
    %the GNU LGPL
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(158, self, varargin{:});
    end
    function varargout = amd(self,varargin)
    %AMD Approximate minimal degree preordering Fill-reducing ordering applied to the
    %
    %  [int] = AMD(self)
    %
    %sparsity pattern of a linear system prior to factorization. The system must
    %be symmetric, for an unsymmetric matrix A, first form the square of the
    %pattern, A'*A.
    %
    %The implementation is a modified version of cs_amd in CSparse Copyright(c)
    %Timothy A. Davis, 2006-2009 Licensed as a derivative work under the GNU LGPL
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(159, self, varargin{:});
    end
    function varargout = find(self,varargin)
    %FIND Get the location of all non-zero elements as they would appear in a Dense
    %
    %  [int] = FIND(self, bool ind1)
    %
    %matrix A : DenseMatrix 4 x 3 B : SparseMatrix 4 x 3 , 5 structural non-
    %zeros.
    %
    %k = A.find() A[k] will contain the elements of A that are non-zero in B
    %
    %Inverse of nonzeros.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(160, self, varargin{:});
    end
    function varargout = uni_coloring(self,varargin)
    %UNI_COLORING Perform a unidirectional coloring: A greedy distance-2 coloring algorithm
    %
    %  Sparsity = UNI_COLORING(self, Sparsity AT, int cutoff)
    %
    %(Algorithm 3.1 in A. H. GEBREMEDHIN, F. MANNE, A. POTHEN)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(161, self, varargin{:});
    end
    function varargout = star_coloring(self,varargin)
    %STAR_COLORING Perform a star coloring of a symmetric matrix: A greedy distance-2 coloring
    %
    %  Sparsity = STAR_COLORING(self, int ordering, int cutoff)
    %
    %algorithm Algorithm 4.1 in What Color Is Your Jacobian? Graph Coloring for
    %Computing Derivatives A. H. GEBREMEDHIN, F. MANNE, A. POTHEN SIAM Rev.,
    %47(4), 629705 (2006)
    %
    %Ordering options: None (0), largest first (1)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(162, self, varargin{:});
    end
    function varargout = star_coloring2(self,varargin)
    %STAR_COLORING2 Perform a star coloring of a symmetric matrix: A new greedy distance-2
    %
    %  Sparsity = STAR_COLORING2(self, int ordering, int cutoff)
    %
    %coloring algorithm Algorithm 4.1 in NEW ACYCLIC AND STAR COLORING ALGORITHMS
    %WITH APPLICATION TO COMPUTING HESSIANS A. H. GEBREMEDHIN, A. TARAFDAR, F.
    %MANNE, A. POTHEN SIAM J. SCI. COMPUT. Vol. 29, No. 3, pp. 10421072 (2007)
    %
    %Ordering options: None (0), largest first (1)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(163, self, varargin{:});
    end
    function varargout = largest_first(self,varargin)
    %LARGEST_FIRST Order the columns by decreasing degree.
    %
    %  [int] = LARGEST_FIRST(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(164, self, varargin{:});
    end
    function varargout = pmult(self,varargin)
    %PMULT Permute rows and/or columns Multiply the sparsity with a permutation matrix
    %
    %  Sparsity = PMULT(self, [int] p, bool permute_rows, bool permute_columns, bool invert_permutation)
    %
    %from the left and/or from the right P * A * trans(P), A * trans(P) or A *
    %trans(P) with P defined by an index vector containing the row for each col.
    %As an alternative, P can be transposed (inverted).
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(165, self, varargin{:});
    end
    function varargout = dim(self,varargin)
    %DIM Get the dimension as a string.
    %
    %  char = DIM(self, bool with_nz)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(166, self, varargin{:});
    end
    function varargout = postfix_dim(self,varargin)
    %POSTFIX_DIM Dimension string as a postfix to a name Rules:
    %
    %  char = POSTFIX_DIM(self)
    %
    %
    %Dense and scalar: ""
    %
    %0-by-0: "[]"
    %
    %Dense column vector: "[5]"
    %
    %Dense matrix: "[5x10]"
    %
    %Otherwise: "[5x10,3nz]"
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(167, self, varargin{:});
    end
    function varargout = repr_el(self,varargin)
    %REPR_EL Describe the nonzero location k as a string.
    %
    %  char = REPR_EL(self, int k)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(168, self, varargin{:});
    end
    function varargout = spy(self,varargin)
    %SPY Print a textual representation of sparsity.
    %
    %  std::ostream & = SPY(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(169, self, varargin{:});
    end
    function varargout = spy_matlab(self,varargin)
    %SPY_MATLAB Generate a script for Matlab or Octave which visualizes the sparsity using
    %
    %  SPY_MATLAB(self, char mfile)
    %
    %the spy command.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(170, self, varargin{:});
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
    %   * options:
    %   *   inline: Indicates if you want everything on a single line (default: False)
    %   *   name: Name of exported variable (default: 'sp')
    %   *   as_matrix: Matlab does not have a sparsity object. (default: false)
    %  *               With this option true, a numeric matrix will be constructed
    %   * 
    %
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(171, self, varargin{:});
    end
    function varargout = hash(self,varargin)
    %HASH 
    %
    %  std::size_t = HASH(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(173, self, varargin{:});
    end

     function s = saveobj(obj)
        try
            s.serialization = obj.serialize();
        catch exception
            warning(['Serializing of CasADi Sparsity failed:' getReport(exception) ]);
            s = struct;
        end
     end
      function self = Sparsity(varargin)
    %SPARSITY 
    %
    %  new_obj = SPARSITY(int dummy)
    %  new_obj = SPARSITY([int,int] rc)
    %  new_obj = SPARSITY(int nrow, int ncol)
    %  new_obj = SPARSITY(int nrow, int ncol, [int] colind, [int] row, bool order_rows)
    %
    %
    %.......
    %
    %::
    %
    %  SPARSITY(int nrow, int ncol)
    %
    %
    %
    %Pattern with all structural zeros.
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
    %  SPARSITY(int nrow, int ncol, [int] colind, [int] row, bool order_rows)
    %
    %
    %
    %Construct from sparsity pattern vectors given in compressed column storage
    %format.
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
    %  SPARSITY(int dummy)
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
    %.......
    %
    %::
    %
    %  SPARSITY([int,int] rc)
    %
    %
    %
    %Create a sparse matrix with all structural zeros.
    %
    %
    %
    %.............
    %
    %
      self@casadi.SharedObject(SwigRef.Null);
      self@casadi.SparsityInterfaceCommon(SwigRef.Null);
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(176, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(177, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
    function varargout = scalar(varargin)
    %SCALAR Create a scalar sparsity pattern.
    %
    %  Sparsity = SCALAR(bool dense_scalar)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(71, varargin{:});
    end
    function varargout = dense(varargin)
    %DENSE Create a dense rectangular sparsity pattern.
    %
    %  Sparsity = DENSE(int nrow, int ncol)
    %  Sparsity = DENSE([int,int] rc)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(72, varargin{:});
    end
    function varargout = unit(varargin)
    %UNIT Create the sparsity pattern for a unit vector of length n and a nonzero on
    %
    %  Sparsity = UNIT(int n, int el)
    %
    %position el.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(73, varargin{:});
    end
    function varargout = upper(varargin)
    %UPPER 
    %
    %  Sparsity = UPPER(int n)
    %
    %
     [varargout{1:nargout}] = casadiMEX(74, varargin{:});
    end
    function varargout = lower(varargin)
    %LOWER 
    %
    %  Sparsity = LOWER(int n)
    %
    %
     [varargout{1:nargout}] = casadiMEX(75, varargin{:});
    end
    function varargout = diag(varargin)
    %DIAG Create diagonal sparsity pattern.
    %
    %  Sparsity = DIAG(int nrow)
    %  Sparsity = DIAG([int,int] rc)
    %  Sparsity = DIAG(int nrow, int ncol)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(76, varargin{:});
    end
    function varargout = band(varargin)
    %BAND 
    %
    %  Sparsity = BAND(int n, int p)
    %
    %
     [varargout{1:nargout}] = casadiMEX(77, varargin{:});
    end
    function varargout = banded(varargin)
    %BANDED 
    %
    %  Sparsity = BANDED(int n, int p)
    %
    %
     [varargout{1:nargout}] = casadiMEX(78, varargin{:});
    end
    function varargout = rowcol(varargin)
    %ROWCOL 
    %
    %  Sparsity = ROWCOL([int] row, [int] col, int nrow, int ncol)
    %
    %
     [varargout{1:nargout}] = casadiMEX(79, varargin{:});
    end
    function varargout = triplet(varargin)
    %TRIPLET 
    %
    %  Sparsity = TRIPLET(int nrow, int ncol, [int] row, [int] col)
    %  [Sparsity , [int] OUTPUT] = TRIPLET(int nrow, int ncol, [int] row, [int] col, bool invert_mapping)
    %
    %
     [varargout{1:nargout}] = casadiMEX(80, varargin{:});
    end
    function varargout = nonzeros(varargin)
    %NONZEROS 
    %
    %  Sparsity = NONZEROS(int nrow, int ncol, [int] nz, bool ind1)
    %
    %
     [varargout{1:nargout}] = casadiMEX(81, varargin{:});
    end
    function varargout = compressed(varargin)
    %COMPRESSED Create from a single vector containing the pattern in compressed column
    %
    %  Sparsity = COMPRESSED([int] v, bool order_rows)
    %
    %storage format: The format: The first two entries are the number of rows
    %(nrow) and columns (ncol) The next ncol+1 entries are the column offsets
    %(colind). Note that the last element, colind[ncol], gives the number of
    %nonzeros The last colind[ncol] entries are the row indices
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(82, varargin{:});
    end
    function varargout = from_file(varargin)
    %FROM_FILE 
    %
    %  Sparsity = FROM_FILE(char filename, char format_hint)
    %
    %
     [varargout{1:nargout}] = casadiMEX(106, varargin{:});
    end
    function varargout = deserialize(varargin)
    %DESERIALIZE 
    %
    %  Sparsity = DESERIALIZE(std::istream & stream)
    %  Sparsity = DESERIALIZE(casadi::DeserializingStream & s)
    %  Sparsity = DESERIALIZE(char s)
    %
    %
     [varargout{1:nargout}] = casadiMEX(108, varargin{:});
    end
    function varargout = type_name(varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME()
    %
    %
     [varargout{1:nargout}] = casadiMEX(172, varargin{:});
    end
    function varargout = test_cast(varargin)
    %TEST_CAST 
    %
    %  bool = TEST_CAST(casadi::SharedObjectInternal const * ptr)
    %
    %
     [varargout{1:nargout}] = casadiMEX(174, varargin{:});
    end
    function varargout = kkt(varargin)
    %KKT 
    %
    %  Sparsity = KKT(Sparsity H, Sparsity J, bool with_x_diag, bool with_lam_g_diag)
    %
    %
     [varargout{1:nargout}] = casadiMEX(175, varargin{:});
    end

     function obj = loadobj(s)
        try
          if isstruct(s)
             obj = casadi.Sparsity.deserialize(s.serialization);
          else
             obj = s;
          end
        catch exception
            warning(['Serializing of CasADi Sparsity failed:' getReport(exception) ]);
            s = struct;
        end
     end
    end
end
