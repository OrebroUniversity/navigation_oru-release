classdef  MatrixCommon < SwigRef
    %MATRIXCOMMON Sparse matrix class. SX and DM are specializations.
    %
    %   = MATRIXCOMMON()
    %
    %
    %General sparse matrix class that is designed with the idea that "everything
    %is a matrix", that is, also scalars and vectors. This philosophy makes it
    %easy to use and to interface in particularly with Python and Matlab/Octave.
    %Index starts with 0. Index vec happens as follows: (rr, cc) -> k =
    %rr+cc*size1() Vectors are column vectors.  The storage format is Compressed
    %Column Storage (CCS), similar to that used for sparse matrices in Matlab,
    %but unlike this format, we do allow for elements to be structurally non-zero
    %but numerically zero.  Matrix<Scalar> is polymorphic with a
    %std::vector<Scalar> that contain all non-identical-zero elements. The
    %sparsity can be accessed with Sparsity& sparsity() Joel Andersson
    %
    %C++ includes: casadi_common.hpp 
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = all(varargin)
    %ALL Returns true only if every element in the matrix is true.
    %
    %  DM = ALL(DM x)
    %  SX = ALL(SX x)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(397, varargin{:});
    end
    function varargout = any(varargin)
    %ANY Returns true only if any element in the matrix is true.
    %
    %  DM = ANY(DM x)
    %  SX = ANY(SX x)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(398, varargin{:});
    end
    function varargout = adj(varargin)
    %ADJ Matrix adjoint.
    %
    %  DM = ADJ(DM A)
    %  SX = ADJ(SX A)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(399, varargin{:});
    end
    function varargout = minor(varargin)
    %MINOR Get the (i,j) minor matrix.
    %
    %  DM = MINOR(DM x, int i, int j)
    %  SX = MINOR(SX x, int i, int j)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(400, varargin{:});
    end
    function varargout = cofactor(varargin)
    %COFACTOR Get the (i,j) cofactor matrix.
    %
    %  DM = COFACTOR(DM x, int i, int j)
    %  SX = COFACTOR(SX x, int i, int j)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(401, varargin{:});
    end
    function varargout = qr(varargin)
    %QR QR factorization using the modified Gram-Schmidt algorithm More stable than
    %
    %  [DM OUTPUT1, DM OUTPUT2] = QR(DM A)
    %  [SX OUTPUT1, SX OUTPUT2] = QR(SX A)
    %
    %the classical Gram-Schmidt, but may break down if the rows of A are nearly
    %linearly dependent See J. Demmel: Applied Numerical Linear Algebra
    %(algorithm 3.1.). Note that in SWIG, Q and R are returned by value.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(402, varargin{:});
    end
    function varargout = qr_sparse(varargin)
    %QR_SPARSE Sparse direct QR factorization See T. Davis: Direct Methods for Sparse
    %
    %  [DM OUTPUT1, DM OUTPUT2, DM OUTPUT3, [int] OUTPUT4, [int] OUTPUT5] = QR_SPARSE(DM A, bool amd)
    %  [SX OUTPUT1, SX OUTPUT2, SX OUTPUT3, [int] OUTPUT4, [int] OUTPUT5] = QR_SPARSE(SX A, bool amd)
    %
    %Linear Systems.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(403, varargin{:});
    end
    function varargout = qr_solve(varargin)
    %QR_SOLVE Solve using a sparse QR factorization.
    %
    %  DM = QR_SOLVE(DM b, DM v, DM r, DM beta, [int] prinv, [int] pc, bool tr)
    %  SX = QR_SOLVE(SX b, SX v, SX r, SX beta, [int] prinv, [int] pc, bool tr)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(404, varargin{:});
    end
    function varargout = ldl(varargin)
    %LDL Sparse LDL^T factorization Returns D and the strictly upper triangular
    %
    %  [DM OUTPUT1, DM OUTPUT2, [int] OUTPUT3] = LDL(DM A, bool amd)
    %  [SX OUTPUT1, SX OUTPUT2, [int] OUTPUT3] = LDL(SX A, bool amd)
    %
    %entries of L^T I.e. ones on the diagonal are ignored. Only guarenteed to
    %work for positive definite matrices.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(405, varargin{:});
    end
    function varargout = ldl_solve(varargin)
    %LDL_SOLVE Solve using a sparse LDL^T factorization.
    %
    %  DM = LDL_SOLVE(DM b, DM D, DM LT, [int] p)
    %  SX = LDL_SOLVE(SX b, SX D, SX LT, [int] p)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(406, varargin{:});
    end
    function varargout = chol(varargin)
    %CHOL Obtain a Cholesky factorisation of a matrix Performs and LDL transformation
    %
    %  DM = CHOL(DM A)
    %  SX = CHOL(SX A)
    %
    %[L,D] = ldl(A) and returns diag(sqrt(D))*L'.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(407, varargin{:});
    end
    function varargout = norm_inf_mul(varargin)
    %NORM_INF_MUL Inf-norm of a Matrix-Matrix product.
    %
    %  DM = NORM_INF_MUL(DM x, DM y)
    %  SX = NORM_INF_MUL(SX x, SX y)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(408, varargin{:});
    end
    function varargout = sparsify(varargin)
    %SPARSIFY Make a matrix sparse by removing numerical zeros.
    %
    %  DM = SPARSIFY(DM A, double tol)
    %  SX = SPARSIFY(SX A, double tol)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(409, varargin{:});
    end
    function varargout = expand(varargin)
    %EXPAND Expand the expression as a weighted sum (with constant weights)
    %
    %  [DM OUTPUT1, DM OUTPUT2] = EXPAND(DM ex)
    %  [SX OUTPUT1, SX OUTPUT2] = EXPAND(SX ex)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(410, varargin{:});
    end
    function varargout = pw_const(varargin)
    %PW_CONST Create a piecewise constant function Create a piecewise constant function
    %
    %  DM = PW_CONST(DM t, DM tval, DM val)
    %  SX = PW_CONST(SX t, SX tval, SX val)
    %
    %with n=val.size() intervals.
    %
    %Inputs:
    %
    %Parameters:
    %-----------
    %
    %t:  a scalar variable (e.g. time)
    %
    %tval:  vector with the discrete values of t at the interval transitions
    %(length n-1)
    %
    %val:  vector with the value of the function for each interval (length n)
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(411, varargin{:});
    end
    function varargout = pw_lin(varargin)
    %PW_LIN t a scalar variable (e.g. time)
    %
    %  DM = PW_LIN(DM t, DM tval, DM val)
    %  SX = PW_LIN(SX t, SX tval, SX val)
    %
    %
    %Create a piecewise linear function Create a piecewise linear function:
    %
    %Inputs: tval vector with the the discrete values of t (monotonically
    %increasing) val vector with the corresponding function values (same length
    %as tval)
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(412, varargin{:});
    end
    function varargout = heaviside(varargin)
    %HEAVISIDE Heaviside function.
    %
    %  DM = HEAVISIDE(DM x)
    %  SX = HEAVISIDE(SX x)
    %
    %
    %\\[ \\begin {cases} H(x) = 0 & x<0 \\\\ H(x) = 1/2 & x=0 \\\\
    %H(x) = 1 & x>0 \\\\ \\end {cases} \\]
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(413, varargin{:});
    end
    function varargout = rectangle(varargin)
    %RECTANGLE rectangle function
    %
    %  DM = RECTANGLE(DM x)
    %  SX = RECTANGLE(SX x)
    %
    %
    %\\[ \\begin {cases} \\Pi(x) = 1 & |x| < 1/2 \\\\ \\Pi(x) = 1/2 &
    %|x| = 1/2 \\\\ \\Pi(x) = 0 & |x| > 1/2 \\\\ \\end {cases} \\]
    %
    %Also called: gate function, block function, band function, pulse function,
    %window function
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(414, varargin{:});
    end
    function varargout = triangle(varargin)
    %TRIANGLE triangle function
    %
    %  DM = TRIANGLE(DM x)
    %  SX = TRIANGLE(SX x)
    %
    %
    %\\[ \\begin {cases} \\Lambda(x) = 0 & |x| >= 1 \\\\ \\Lambda(x)
    %= 1-|x| & |x| < 1 \\end {cases} \\]
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(415, varargin{:});
    end
    function varargout = ramp(varargin)
    %RAMP ramp function
    %
    %  DM = RAMP(DM x)
    %  SX = RAMP(SX x)
    %
    %
    %\\[ \\begin {cases} R(x) = 0 & x <= 1 \\\\ R(x) = x & x > 1 \\\\
    %\\end {cases} \\]
    %
    %Also called: slope function
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(416, varargin{:});
    end
    function varargout = gauss_quadrature(varargin)
    %GAUSS_QUADRATURE Matrix adjoint.
    %
    %  DM = GAUSS_QUADRATURE(DM f, DM x, DM a, DM b, int order)
    %  SX = GAUSS_QUADRATURE(SX f, SX x, SX a, SX b, int order)
    %  DM = GAUSS_QUADRATURE(DM f, DM x, DM a, DM b, int order, DM w)
    %  SX = GAUSS_QUADRATURE(SX f, SX x, SX a, SX b, int order, SX w)
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(417, varargin{:});
    end
    function varargout = taylor(varargin)
    %TAYLOR univariate Taylor series expansion
    %
    %  DM = TAYLOR(DM ex, DM x, DM a, int order)
    %  SX = TAYLOR(SX ex, SX x, SX a, int order)
    %
    %
    %Calculate the Taylor expansion of expression 'ex' up to order 'order' with
    %respect to variable 'x' around the point 'a'
    %
    %$(x)=f(a)+f'(a)(x-a)+f''(a)\\frac
    %{(x-a)^2}{2!}+f'''(a)\\frac{(x-a)^3}{3!}+\\ldots$
    %
    %Example usage:
    %
    %::
    %
    %>>   x
    %
    %
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(418, varargin{:});
    end
    function varargout = mtaylor(varargin)
    %MTAYLOR multivariate Taylor series expansion
    %
    %  DM = MTAYLOR(DM ex, DM x, DM a, int order)
    %  SX = MTAYLOR(SX ex, SX x, SX a, int order)
    %  DM = MTAYLOR(DM ex, DM x, DM a, int order, [int] order_contributions)
    %  SX = MTAYLOR(SX ex, SX x, SX a, int order, [int] order_contributions)
    %
    %
    %Do Taylor expansions until the aggregated order of a term is equal to
    %'order'. The aggregated order of $x^n y^m$ equals $n+m$.
    %
    %The argument order_contributions can denote how match each variable
    %contributes to the aggregated order. If x=[x, y] and order_contributions=[1,
    %2], then the aggregated order of $x^n y^m$ equals $1n+2m$.
    %
    %Example usage
    %
    %$ \\sin(b+a)+\\cos(b+a)(x-a)+\\cos(b+a)(y-b) $ $ y+x-(x^3+3y x^2+3 y^2
    %x+y^3)/6 $ $ (-3 x^2 y-x^3)/6+y+x $
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(419, varargin{:});
    end
    function varargout = poly_coeff(varargin)
    %POLY_COEFF extracts polynomial coefficients from an expression
    %
    %  DM = POLY_COEFF(DM ex, DM x)
    %  SX = POLY_COEFF(SX ex, SX x)
    %
    %
    %Parameters:
    %-----------
    %
    %ex:  Scalar expression that represents a polynomial
    %
    %x:  Scalar symbol that the polynomial is build up with
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(420, varargin{:});
    end
    function varargout = poly_roots(varargin)
    %POLY_ROOTS Attempts to find the roots of a polynomial.
    %
    %  DM = POLY_ROOTS(DM p)
    %  SX = POLY_ROOTS(SX p)
    %
    %
    %This will only work for polynomials up to order 3 It is assumed that the
    %roots are real.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(421, varargin{:});
    end
    function varargout = eig_symbolic(varargin)
    %EIG_SYMBOLIC Attempts to find the eigenvalues of a symbolic matrix This will only work
    %
    %  DM = EIG_SYMBOLIC(DM m)
    %  SX = EIG_SYMBOLIC(SX m)
    %
    %for up to 3x3 matrices.
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(422, varargin{:});
    end
    function self = MatrixCommon(varargin)
    %MATRIXCOMMON 
    %
    %  new_obj = MATRIXCOMMON()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(423, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(424, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
