classdef  GenericMatrixCommon < SwigRef
    %GENERICMATRIXCOMMON Matrix base class.
    %
    %   = GENERICMATRIXCOMMON()
    %
    %
    %This is a common base class for MX and Matrix<>, introducing a uniform
    %syntax and implementing common functionality using the curiously recurring
    %template pattern (CRTP) idiom.  The class is designed with the idea that
    %"everything is a matrix", that is, also scalars and vectors. This
    %philosophy makes it easy to use and to interface in particularly with Python
    %and Matlab/Octave.  The syntax tries to stay as close as possible to the
    %ublas syntax when it comes to vector/matrix operations.  Index starts with
    %0. Index vec happens as follows: (rr, cc) -> k = rr+cc*size1() Vectors are
    %column vectors.  The storage format is Compressed Column Storage (CCS),
    %similar to that used for sparse matrices in Matlab, but unlike this format,
    %we do allow for elements to be structurally non-zero but numerically zero.
    %The sparsity pattern, which is reference counted and cached, can be accessed
    %with Sparsity& sparsity() Joel Andersson
    %
    %C++ includes: generic_matrix.hpp 
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end

    function varargout = spy(self,varargin)
      [varargout{1:nargout}] = spy(sparse(casadi.DM(self.sparsity(),1)),varargin{:});
    end
    function varargout = subsref(self,s)
      if numel(s)==1 && strcmp(s.type,'()')
        [varargout{1}] = paren(self, s.subs{:});
      elseif numel(s)==1 && strcmp(s.type,'{}')
        [varargout{1}] = brace(self, s.subs{:});
      else
        [varargout{1:nargout}] = builtin('subsref',self,s);
      end
    end
    function self = subsasgn(self,s,v)
      if numel(s)==1 && strcmp(s.type,'()')
        paren_asgn(self, v, s.subs{:});
      elseif numel(s)==1 && strcmp(s.type,'{}')
        brace_asgn(self, v, s.subs{:});
      else
        self = builtin('subsasgn',self,s,v);
      end
    end
    function out = sum(self,varargin)
      narginchk(1,3);
      if nargin==1
        if is_vector(self)
          if is_column(self)
            out = sum1(self);
          else
            out = sum2(self);
          end
        else
          out = sum1(self);
        end
      else
        i = varargin{1};
        if i==1
          out = sum1(self);
        elseif i==2
          out = sum2(self);
        else
          error('sum argument (if present) must be 1 or 2');
        end
      end
    end
    function out = norm(self,varargin)
      narginchk(1,2);
      % 2-norm by default
      if nargin==1
        ind = 2;
      else
        ind = varargin{1};
      end
      % Typecheck
      assert((isnumeric(ind) && isscalar(ind)) || ischar(ind))
      % Pick the right norm
      if isnumeric(ind)
        switch ind
          case 1
            out = norm_1(self);
          case 2
            out = norm_2(self);
          case inf
            out = norm_inf(self);
          otherwise
            error(sprintf('Unknown norm argument: %g', ind))
        end
      else
        switch ind
          case 'fro'
            out = norm_fro(self);
          case 'inf'
            out = norm_inf(self);
          otherwise
            error(sprintf('Unknown norm argument: ''%s''', ind))
        end
      end
    end
    function out = min(varargin)
      narginchk(1,2);
      if nargin==1
        out = mmin(varargin{1});
      else
        out = fmin(varargin{1}, varargin{2});
      end
    end
    function out = max(varargin)
      narginchk(1,2);
      if nargin==1
        out = mmax(varargin{1});
      else
        out = fmax(varargin{1}, varargin{2});
      end
    end
    function b = isrow(self)
      b = is_row(self);
    end
    function b = iscolumn(self)
      b = is_column(self);
    end
    function b = isvector(self)
      b = is_vector(self);
    end
    function b = isscalar(self)
      b = is_scalar(self);
    end
      function varargout = mpower(varargin)
    %MPOWER Matrix power x^n.
    %
    %  DM = MPOWER(DM x, DM n)
    %  SX = MPOWER(SX x, SX n)
    %  MX = MPOWER(MX x, MX n)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(205, varargin{:});
    end
    function varargout = mrdivide(varargin)
    %MRDIVIDE Matrix divide (cf. slash '/' in MATLAB)
    %
    %  DM = MRDIVIDE(DM x, DM y)
    %  SX = MRDIVIDE(SX x, SX y)
    %  MX = MRDIVIDE(MX x, MX y)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(206, varargin{:});
    end
    function varargout = mldivide(varargin)
    %MLDIVIDE Matrix divide (cf. backslash '\\' in MATLAB)
    %
    %  DM = MLDIVIDE(DM x, DM y)
    %  SX = MLDIVIDE(SX x, SX y)
    %  MX = MLDIVIDE(MX x, MX y)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(207, varargin{:});
    end
    function varargout = symvar(varargin)
    %SYMVAR Get all symbols contained in the supplied expression Get all symbols on
    %
    %  {DM} = SYMVAR(DM x)
    %  {SX} = SYMVAR(SX x)
    %  {MX} = SYMVAR(MX x)
    %
    %which the supplied expression depends.
    %
    %See:  SXFunction::getFree(), MXFunction::getFree()
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(208, varargin{:});
    end
    function varargout = bilin(varargin)
    %BILIN Calculate bilinear form x^T A y.
    %
    %  DM = BILIN(DM A, DM x, DM y)
    %  SX = BILIN(SX A, SX x, SX y)
    %  MX = BILIN(MX A, MX x, MX y)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(209, varargin{:});
    end
    function varargout = rank1(varargin)
    %RANK1 Make a rank-1 update to a matrix A Calculates A + 1/2 * alpha * x*y'.
    %
    %  DM = RANK1(DM A, DM alpha, DM x, DM y)
    %  SX = RANK1(SX A, SX alpha, SX x, SX y)
    %  MX = RANK1(MX A, MX alpha, MX x, MX y)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(210, varargin{:});
    end
    function varargout = sumsqr(varargin)
    %SUMSQR Calculate sum of squares: sum_ij X_ij^2.
    %
    %  DM = SUMSQR(DM X)
    %  SX = SUMSQR(SX X)
    %  MX = SUMSQR(MX X)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(211, varargin{:});
    end
    function varargout = linspace(varargin)
    %LINSPACE Matlab's linspace command.
    %
    %  DM = LINSPACE(DM a, DM b, int nsteps)
    %  SX = LINSPACE(SX a, SX b, int nsteps)
    %  MX = LINSPACE(MX a, MX b, int nsteps)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(212, varargin{:});
    end
    function varargout = interp1d(varargin)
    %INTERP1D Performs 1d linear interpolation.
    %
    %  DM = INTERP1D([double] x, DM v, [double] xq, char mode, bool equidistant)
    %  SX = INTERP1D([double] x, SX v, [double] xq, char mode, bool equidistant)
    %  MX = INTERP1D([double] x, MX v, [double] xq, char mode, bool equidistant)
    %
    %
    %The data-points to be interpolated are given as (x[i], v[i]). xq[j] is used
    %as interplating value
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(213, varargin{:});
    end
    function varargout = soc(varargin)
    %SOC Construct second-order-convex.
    %
    %  DM = SOC(DM x, DM y)
    %  SX = SOC(SX x, SX y)
    %  MX = SOC(MX x, MX y)
    %
    %
    %Parameters:
    %-----------
    %
    %x:  vector expression of size n
    %
    %y:  scalar expression
    %
    %soc(x,y) computes [y*eye(n) x; x' y]
    %
    %soc(x,y) positive semi definite <=> || x ||_2 <= y
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(214, varargin{:});
    end
    function varargout = cross(varargin)
    %CROSS Matlab's cross command.
    %
    %  DM = CROSS(DM a, DM b, int dim)
    %  SX = CROSS(SX a, SX b, int dim)
    %  MX = CROSS(MX a, MX b, int dim)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(215, varargin{:});
    end
    function varargout = skew(varargin)
    %SKEW Generate a skew symmetric matrix from a 3-vector.
    %
    %  DM = SKEW(DM a)
    %  SX = SKEW(SX a)
    %  MX = SKEW(MX a)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(216, varargin{:});
    end
    function varargout = inv_skew(varargin)
    %INV_SKEW Generate the 3-vector progenitor of a skew symmetric matrix.
    %
    %  DM = INV_SKEW(DM a)
    %  SX = INV_SKEW(SX a)
    %  MX = INV_SKEW(MX a)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(217, varargin{:});
    end
    function varargout = det(varargin)
    %DET Matrix determinant (experimental)
    %
    %  DM = DET(DM A)
    %  SX = DET(SX A)
    %  MX = DET(MX A)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(218, varargin{:});
    end
    function varargout = inv_minor(varargin)
    %INV_MINOR Matrix inverse (experimental)
    %
    %  DM = INV_MINOR(DM A)
    %  SX = INV_MINOR(SX A)
    %  MX = INV_MINOR(MX A)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(219, varargin{:});
    end
    function varargout = inv(varargin)
    %INV Matrix inverse.
    %
    %  DM = INV(DM A)
    %  SX = INV(SX A)
    %  MX = INV(MX A)
    %  DM = INV(DM A, char lsolver, struct opts)
    %  SX = INV(SX A, char lsolver, struct opts)
    %  MX = INV(MX A, char lsolver, struct opts)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(220, varargin{:});
    end
    function varargout = trace(varargin)
    %TRACE Matrix trace.
    %
    %  DM = TRACE(DM a)
    %  SX = TRACE(SX a)
    %  MX = TRACE(MX a)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(221, varargin{:});
    end
    function varargout = tril2symm(varargin)
    %TRIL2SYMM Convert a lower triangular matrix to a symmetric one.
    %
    %  DM = TRIL2SYMM(DM a)
    %  SX = TRIL2SYMM(SX a)
    %  MX = TRIL2SYMM(MX a)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(222, varargin{:});
    end
    function varargout = triu2symm(varargin)
    %TRIU2SYMM Convert a upper triangular matrix to a symmetric one.
    %
    %  DM = TRIU2SYMM(DM a)
    %  SX = TRIU2SYMM(SX a)
    %  MX = TRIU2SYMM(MX a)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(223, varargin{:});
    end
    function varargout = norm_fro(varargin)
    %NORM_FRO Frobenius norm.
    %
    %  DM = NORM_FRO(DM x)
    %  SX = NORM_FRO(SX x)
    %  MX = NORM_FRO(MX x)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(224, varargin{:});
    end
    function varargout = norm_2(varargin)
    %NORM_2 2-norm
    %
    %  DM = NORM_2(DM x)
    %  SX = NORM_2(SX x)
    %  MX = NORM_2(MX x)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(225, varargin{:});
    end
    function varargout = norm_1(varargin)
    %NORM_1 1-norm
    %
    %  DM = NORM_1(DM x)
    %  SX = NORM_1(SX x)
    %  MX = NORM_1(MX x)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(226, varargin{:});
    end
    function varargout = norm_inf(varargin)
    %NORM_INF Infinity-norm.
    %
    %  DM = NORM_INF(DM x)
    %  SX = NORM_INF(SX x)
    %  MX = NORM_INF(MX x)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(227, varargin{:});
    end
    function varargout = dot(varargin)
    %DOT Inner product of two matrices with x and y matrices of the same dimension.
    %
    %  DM = DOT(DM x, DM y)
    %  SX = DOT(SX x, SX y)
    %  MX = DOT(MX x, MX y)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(228, varargin{:});
    end
    function varargout = nullspace(varargin)
    %NULLSPACE Computes the nullspace of a matrix A.
    %
    %  DM = NULLSPACE(DM A)
    %  SX = NULLSPACE(SX A)
    %  MX = NULLSPACE(MX A)
    %
    %
    %Finds Z m-by-(m-n) such that AZ = 0 with A n-by-m with m > n
    %
    %Assumes A is full rank
    %
    %Inspired by Numerical Methods in Scientific Computing by Ake Bjorck
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(229, varargin{:});
    end
    function varargout = polyval(varargin)
    %POLYVAL Evaluate a polynomial with coefficients p in x.
    %
    %  DM = POLYVAL(DM p, DM x)
    %  SX = POLYVAL(SX p, SX x)
    %  MX = POLYVAL(MX p, MX x)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(230, varargin{:});
    end
    function varargout = diag(varargin)
    %DIAG Get the diagonal of a matrix or construct a diagonal When the input is
    %
    %  DM = DIAG(DM A)
    %  SX = DIAG(SX A)
    %  MX = DIAG(MX A)
    %
    %square, the diagonal elements are returned. If the input is vector- like, a
    %diagonal matrix is constructed with it.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(231, varargin{:});
    end
    function varargout = unite(varargin)
    %UNITE Unite two matrices no overlapping sparsity.
    %
    %  DM = UNITE(DM A, DM B)
    %  SX = UNITE(SX A, SX B)
    %  MX = UNITE(MX A, MX B)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(232, varargin{:});
    end
    function varargout = densify(varargin)
    %DENSIFY Make the matrix dense and assign nonzeros to a value.
    %
    %  DM = DENSIFY(DM x)
    %  SX = DENSIFY(SX x)
    %  MX = DENSIFY(MX x)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(233, varargin{:});
    end
    function varargout = project(varargin)
    %PROJECT Create a new matrix with a given sparsity pattern but with the nonzeros
    %
    %  DM = PROJECT(DM A, Sparsity sp, bool intersect)
    %  SX = PROJECT(SX A, Sparsity sp, bool intersect)
    %  MX = PROJECT(MX A, Sparsity sp, bool intersect)
    %
    %taken from an existing matrix.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(234, varargin{:});
    end
    function varargout = if_else(varargin)
    %IF_ELSE Branching on MX nodes Ternary operator, "cond ? if_true : if_false".
    %
    %  DM = IF_ELSE(DM cond, DM if_true, DM if_false, bool short_circuit)
    %  SX = IF_ELSE(SX cond, SX if_true, SX if_false, bool short_circuit)
    %  MX = IF_ELSE(MX cond, MX if_true, MX if_false, bool short_circuit)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(235, varargin{:});
    end
    function varargout = conditional(varargin)
    %CONDITIONAL Create a switch.
    %
    %  DM = CONDITIONAL(DM ind, {DM} x, DM x_default, bool short_circuit)
    %  SX = CONDITIONAL(SX ind, {SX} x, SX x_default, bool short_circuit)
    %  MX = CONDITIONAL(MX ind, {MX} x, MX x_default, bool short_circuit)
    %
    %
    %If the condition
    %
    %Parameters:
    %-----------
    %
    %ind:  evaluates to the integer k, where 0<=k<f.size(), then x[k] will be
    %returned, otherwise
    %
    %x_default:  will be returned.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(236, varargin{:});
    end
    function varargout = depends_on(varargin)
    %DEPENDS_ON Check if expression depends on the argument The argument must be symbolic.
    %
    %  bool = DEPENDS_ON(DM f, DM arg)
    %  bool = DEPENDS_ON(SX f, SX arg)
    %  bool = DEPENDS_ON(MX f, MX arg)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(237, varargin{:});
    end
    function varargout = solve(varargin)
    %SOLVE Solve a system of equations: A*x = b.
    %
    %  DM = SOLVE(DM A, DM b)
    %  SX = SOLVE(SX A, SX b)
    %  MX = SOLVE(MX A, MX b)
    %  DM = SOLVE(DM A, DM b, char lsolver, struct opts)
    %  SX = SOLVE(SX A, SX b, char lsolver, struct opts)
    %  MX = SOLVE(MX A, MX b, char lsolver, struct opts)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(238, varargin{:});
    end
    function varargout = pinv(varargin)
    %PINV Computes the Moore-Penrose pseudo-inverse.
    %
    %  DM = PINV(DM A)
    %  SX = PINV(SX A)
    %  MX = PINV(MX A)
    %  DM = PINV(DM A, char lsolver, struct opts)
    %  SX = PINV(SX A, char lsolver, struct opts)
    %  MX = PINV(MX A, char lsolver, struct opts)
    %
    %
    %If the matrix A is fat (size1>size2), mul(A, pinv(A)) is unity. If the
    %matrix A is slender (size2<size1), mul(pinv(A), A) is unity.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(239, varargin{:});
    end
    function varargout = expm_const(varargin)
    %EXPM_CONST 
    %
    %  DM = EXPM_CONST(DM A, DM t)
    %  SX = EXPM_CONST(SX A, SX t)
    %  MX = EXPM_CONST(MX A, MX t)
    %
    %
     [varargout{1:nargout}] = casadiMEX(240, varargin{:});
    end
    function varargout = expm(varargin)
    %EXPM 
    %
    %  DM = EXPM(DM A)
    %  SX = EXPM(SX A)
    %  MX = EXPM(MX A)
    %
    %
     [varargout{1:nargout}] = casadiMEX(241, varargin{:});
    end
    function varargout = jacobian(varargin)
    %JACOBIAN Calculate Jacobian.
    %
    %  DM = JACOBIAN(DM ex, DM arg, struct opts)
    %  SX = JACOBIAN(SX ex, SX arg, struct opts)
    %  MX = JACOBIAN(MX ex, MX arg, struct opts)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(242, varargin{:});
    end
    function varargout = jtimes(varargin)
    %JTIMES Calculate the Jacobian and multiply by a vector from the right This is
    %
    %  DM = JTIMES(DM ex, DM arg, DM v, bool tr)
    %  SX = JTIMES(SX ex, SX arg, SX v, bool tr)
    %  MX = JTIMES(MX ex, MX arg, MX v, bool tr)
    %
    %equivalent to mul(jacobian(ex, arg), v) or mul(jacobian(ex, arg).T, v) for
    %tr set to false and true respectively. If contrast to these expressions, it
    %will use directional derivatives which is typically (but not necessarily)
    %more efficient if the complete Jacobian is not needed and v has few rows.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(243, varargin{:});
    end
    function varargout = linearize(varargin)
    %LINEARIZE Linearize an expression.
    %
    %  DM = LINEARIZE(DM f, DM x, DM x0)
    %  SX = LINEARIZE(SX f, SX x, SX x0)
    %  MX = LINEARIZE(MX f, MX x, MX x0)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(244, varargin{:});
    end
    function varargout = which_depends(varargin)
    %WHICH_DEPENDS Find out which variables enter with some order.
    %
    %  [bool] = WHICH_DEPENDS(DM expr, DM var, int order, bool tr)
    %  [bool] = WHICH_DEPENDS(SX expr, SX var, int order, bool tr)
    %  [bool] = WHICH_DEPENDS(MX expr, MX var, int order, bool tr)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(245, varargin{:});
    end
    function varargout = is_linear(varargin)
    %IS_LINEAR Is expr linear in var?
    %
    %  bool = IS_LINEAR(DM expr, DM var)
    %  bool = IS_LINEAR(SX expr, SX var)
    %  bool = IS_LINEAR(MX expr, MX var)
    %
    %
    %False negatives are possible (an expression may not be recognised as linear
    %while it really is), false positives not.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(246, varargin{:});
    end
    function varargout = is_quadratic(varargin)
    %IS_QUADRATIC Is expr quadratic in var?
    %
    %  bool = IS_QUADRATIC(DM expr, DM var)
    %  bool = IS_QUADRATIC(SX expr, SX var)
    %  bool = IS_QUADRATIC(MX expr, MX var)
    %
    %
    %False negatives are possible (an expression may not be recognised as
    %quadratic while it really is), false positives not.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(247, varargin{:});
    end
    function varargout = gradient(varargin)
    %GRADIENT Calculate Jacobian.
    %
    %  DM = GRADIENT(DM ex, DM arg)
    %  SX = GRADIENT(SX ex, SX arg)
    %  MX = GRADIENT(MX ex, MX arg)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(248, varargin{:});
    end
    function varargout = tangent(varargin)
    %TANGENT Calculate Jacobian.
    %
    %  DM = TANGENT(DM ex, DM arg)
    %  SX = TANGENT(SX ex, SX arg)
    %  MX = TANGENT(MX ex, MX arg)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(249, varargin{:});
    end
    function varargout = hessian(varargin)
    %HESSIAN 
    %
    %  [DM , DM OUTPUT1] = HESSIAN(DM ex, DM arg)
    %  [SX , SX OUTPUT1] = HESSIAN(SX ex, SX arg)
    %  [MX , MX OUTPUT1] = HESSIAN(MX ex, MX arg)
    %
    %
     [varargout{1:nargout}] = casadiMEX(250, varargin{:});
    end
    function varargout = quadratic_coeff(varargin)
    %QUADRATIC_COEFF Recognizes quadratic form in scalar expression.
    %
    %  [DM OUTPUT1, DM OUTPUT2, DM OUTPUT3] = QUADRATIC_COEFF(DM ex, DM arg, bool check)
    %  [SX OUTPUT1, SX OUTPUT2, SX OUTPUT3] = QUADRATIC_COEFF(SX ex, SX arg, bool check)
    %  [MX OUTPUT1, MX OUTPUT2, MX OUTPUT3] = QUADRATIC_COEFF(MX ex, MX arg, bool check)
    %
    %
    %1/2*x' A x + b' x + c
    %
    %e = 0.5*bilin(A,x,x)+dot(b,x)+c
    %
    %Parameters:
    %-----------
    %
    %check[in]:  When true (default), A is checked to be independent of x.
    %Provided to deal with false positive dependency checks.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(251, varargin{:});
    end
    function varargout = linear_coeff(varargin)
    %LINEAR_COEFF Recognizes linear form in vector expression.
    %
    %  [DM OUTPUT1, DM OUTPUT2] = LINEAR_COEFF(DM ex, DM arg, bool check)
    %  [SX OUTPUT1, SX OUTPUT2] = LINEAR_COEFF(SX ex, SX arg, bool check)
    %  [MX OUTPUT1, MX OUTPUT2] = LINEAR_COEFF(MX ex, MX arg, bool check)
    %
    %
    %A x + b
    %
    %Parameters:
    %-----------
    %
    %check[in]:  When true (default)m, A is checked to be independent of x.
    %Provided to deal with false positive dependency checks.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(252, varargin{:});
    end
    function varargout = n_nodes(varargin)
    %N_NODES 
    %
    %  int = N_NODES(DM A)
    %  int = N_NODES(SX A)
    %  int = N_NODES(MX A)
    %
    %
     [varargout{1:nargout}] = casadiMEX(253, varargin{:});
    end
    function varargout = print_operator(varargin)
    %PRINT_OPERATOR Get a string representation for a binary MatType, using custom arguments.
    %
    %  char = PRINT_OPERATOR(DM xb, {char} args)
    %  char = PRINT_OPERATOR(SX xb, {char} args)
    %  char = PRINT_OPERATOR(MX xb, {char} args)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(254, varargin{:});
    end
    function varargout = repsum(varargin)
    %REPSUM Given a repeated matrix, computes the sum of repeated parts.
    %
    %  DM = REPSUM(DM A, int n, int m)
    %  SX = REPSUM(SX A, int n, int m)
    %  MX = REPSUM(MX A, int n, int m)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(255, varargin{:});
    end
    function varargout = diff(varargin)
    %DIFF Returns difference (n-th order) along given axis (MATLAB convention)
    %
    %  DM = DIFF(DM A, int n, index axis)
    %  SX = DIFF(SX A, int n, index axis)
    %  MX = DIFF(MX A, int n, index axis)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(256, varargin{:});
    end
    function varargout = cumsum(varargin)
    %CUMSUM Returns cumulative sum along given axis (MATLAB convention)
    %
    %  DM = CUMSUM(DM A, index axis)
    %  SX = CUMSUM(SX A, index axis)
    %  MX = CUMSUM(MX A, index axis)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(257, varargin{:});
    end
    function varargout = einstein(varargin)
    %EINSTEIN Performs 1d linear interpolation.
    %
    %  DM = EINSTEIN(DM A, DM B, [int] dim_a, [int] dim_b, [int] dim_c, [int] a, [int] b, [int] c)
    %  SX = EINSTEIN(SX A, SX B, [int] dim_a, [int] dim_b, [int] dim_c, [int] a, [int] b, [int] c)
    %  MX = EINSTEIN(MX A, MX B, [int] dim_a, [int] dim_b, [int] dim_c, [int] a, [int] b, [int] c)
    %  DM = EINSTEIN(DM A, DM B, DM C, [int] dim_a, [int] dim_b, [int] dim_c, [int] a, [int] b, [int] c)
    %  SX = EINSTEIN(SX A, SX B, SX C, [int] dim_a, [int] dim_b, [int] dim_c, [int] a, [int] b, [int] c)
    %  MX = EINSTEIN(MX A, MX B, MX C, [int] dim_a, [int] dim_b, [int] dim_c, [int] a, [int] b, [int] c)
    %
    %
    %The data-points to be interpolated are given as (x[i], v[i]). xq[j] is used
    %as interplating value
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(258, varargin{:});
    end
    function varargout = mmin(varargin)
    %MMIN Smallest element in a matrix.
    %
    %  DM = MMIN(DM x)
    %  SX = MMIN(SX x)
    %  MX = MMIN(MX x)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(259, varargin{:});
    end
    function varargout = mmax(varargin)
    %MMAX Largest element in a matrix.
    %
    %  DM = MMAX(DM x)
    %  SX = MMAX(SX x)
    %  MX = MMAX(MX x)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(260, varargin{:});
    end
    function varargout = evalf(varargin)
    %EVALF 
    %
    %  DM = EVALF(DM x)
    %  DM = EVALF(SX x)
    %  DM = EVALF(MX x)
    %
    %
     [varargout{1:nargout}] = casadiMEX(261, varargin{:});
    end
    function self = GenericMatrixCommon(varargin)
    %GENERICMATRIXCOMMON 
    %
    %  new_obj = GENERICMATRIXCOMMON()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(262, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(263, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
