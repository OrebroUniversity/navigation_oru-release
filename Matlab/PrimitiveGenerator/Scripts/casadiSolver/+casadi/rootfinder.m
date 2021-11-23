function varargout = rootfinder(varargin)
    %ROOTFINDER 
    %
    %  Function = ROOTFINDER(char name, char solver, struct:SX rfp, struct opts)
    %  Function = ROOTFINDER(char name, char solver, struct:MX rfp, struct opts)
    %  Function = ROOTFINDER(char name, char solver, Function f, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  ROOTFINDER(char name, char solver, struct:SX rfp, struct opts)
    %
    %
    %
    %Create a solver for rootfinding problems Takes a function where one of the
    %inputs is unknown and one of the outputs is a residual function that is
    %always zero, defines a new function where the the unknown input has been
    %replaced by a guess for the unknown and the residual output has been
    %replaced by the calculated value for the input.
    %
    %For a function [y0, y1, ...,yi, .., yn] = F(x0, x1, ..., xj, ..., xm), where
    %xj is unknown and yi=0, defines a new function [y0, y1, ...,xj, .., yn] =
    %G(x0, x1, ..., xj_guess, ..., xm),
    %
    %xj and yi must have the same dimension and d(yi)/d(xj) must be invertable.
    %
    %By default, the first input is unknown and the first output is the residual.
    %
    %General information
    %===================
    %
    %
    %
    %>List of available options
    %
    %+------------------+-----------------+------------------+------------------+
    %|        Id        |      Type       |   Description    |     Used in      |
    %+==================+=================+==================+==================+
    %| common_options   | OT_DICT         | Options for      | casadi::OracleFu |
    %|                  |                 | auto-generated   | nction           |
    %|                  |                 | functions        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| constraints      | OT_INTVECTOR    | Constrain the    | casadi::Rootfind |
    %|                  |                 | unknowns. 0      | er               |
    %|                  |                 | (default): no    |                  |
    %|                  |                 | constraint on    |                  |
    %|                  |                 | ui, 1: ui >=     |                  |
    %|                  |                 | 0.0, -1: ui <=   |                  |
    %|                  |                 | 0.0, 2: ui >     |                  |
    %|                  |                 | 0.0, -2: ui <    |                  |
    %|                  |                 | 0.0.             |                  |
    %+------------------+-----------------+------------------+------------------+
    %| error_on_fail    | OT_BOOL         | When the         | casadi::Rootfind |
    %|                  |                 | numerical        | er               |
    %|                  |                 | process returns  |                  |
    %|                  |                 | unsuccessfully,  |                  |
    %|                  |                 | raise an error   |                  |
    %|                  |                 | (default false). |                  |
    %+------------------+-----------------+------------------+------------------+
    %| expand           | OT_BOOL         | Replace MX with  | casadi::OracleFu |
    %|                  |                 | SX expressions   | nction           |
    %|                  |                 | in problem       |                  |
    %|                  |                 | formulation      |                  |
    %|                  |                 | [false]          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| implicit_input   | OT_INT          | Index of the     | casadi::Rootfind |
    %|                  |                 | input that       | er               |
    %|                  |                 | corresponds to   |                  |
    %|                  |                 | the actual root- |                  |
    %|                  |                 | finding          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| implicit_output  | OT_INT          | Index of the     | casadi::Rootfind |
    %|                  |                 | output that      | er               |
    %|                  |                 | corresponds to   |                  |
    %|                  |                 | the actual root- |                  |
    %|                  |                 | finding          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jacobian_functio | OT_FUNCTION     | Function object  | casadi::Rootfind |
    %| n                |                 | for calculating  | er               |
    %|                  |                 | the Jacobian     |                  |
    %|                  |                 | (autogenerated   |                  |
    %|                  |                 | by default)      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| linear_solver    | OT_STRING       | User-defined     | casadi::Rootfind |
    %|                  |                 | linear solver    | er               |
    %|                  |                 | class. Needed    |                  |
    %|                  |                 | for              |                  |
    %|                  |                 | sensitivities.   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| linear_solver_op | OT_DICT         | Options to be    | casadi::Rootfind |
    %| tions            |                 | passed to the    | er               |
    %|                  |                 | linear solver.   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| monitor          | OT_STRINGVECTOR | Set of user      | casadi::OracleFu |
    %|                  |                 | problem          | nction           |
    %|                  |                 | functions to be  |                  |
    %|                  |                 | monitored        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| show_eval_warnin | OT_BOOL         | Show warnings    | casadi::OracleFu |
    %| gs               |                 | generated from   | nction           |
    %|                  |                 | function         |                  |
    %|                  |                 | evaluations      |                  |
    %|                  |                 | [true]           |                  |
    %+------------------+-----------------+------------------+------------------+
    %| specific_options | OT_DICT         | Options for      | casadi::OracleFu |
    %|                  |                 | specific auto-   | nction           |
    %|                  |                 | generated        |                  |
    %|                  |                 | functions,       |                  |
    %|                  |                 | overwriting the  |                  |
    %|                  |                 | defaults from    |                  |
    %|                  |                 | common_options.  |                  |
    %|                  |                 | Nested           |                  |
    %|                  |                 | dictionary.      |                  |
    %+------------------+-----------------+------------------+------------------+
    %
    %>Input scheme: casadi::RootfinderInput (ROOTFINDER_NUM_IN = 2)
    %
    %+---------------+-------+---------------------------------+
    %|   Full name   | Short |           Description           |
    %+===============+=======+=================================+
    %| ROOTFINDER_X0 | x0    | Initial guess for the solution. |
    %+---------------+-------+---------------------------------+
    %| ROOTFINDER_P  | p     | Parameters.                     |
    %+---------------+-------+---------------------------------+
    %
    %>Output scheme: casadi::RootfinderOutput (ROOTFINDER_NUM_OUT = 1)
    %
    %+--------------+-------+--------------------------------------+
    %|  Full name   | Short |             Description              |
    %+==============+=======+======================================+
    %| ROOTFINDER_X | x     | Solution to the system of equations. |
    %+--------------+-------+--------------------------------------+
    %
    %List of plugins
    %===============
    %
    %
    %
    %- kinsol
    %
    %- fast_newton
    %
    %- nlpsol
    %
    %- newton
    %
    %Note: some of the plugins in this list might not be available on your
    %system. Also, there might be extra plugins available to you that are not
    %listed here. You can obtain their documentation with
    %Rootfinder.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %kinsol
    %------
    %
    %
    %
    %KINSOL interface from the Sundials suite
    %
    %>List of available options
    %
    %+---------------------------+-----------------+----------------------------+
    %|            Id             |      Type       |        Description         |
    %+===========================+=================+============================+
    %| abstol                    | OT_DOUBLE       | Stopping criterion         |
    %|                           |                 | tolerance                  |
    %+---------------------------+-----------------+----------------------------+
    %| disable_internal_warnings | OT_BOOL         | Disable KINSOL internal    |
    %|                           |                 | warning messages           |
    %+---------------------------+-----------------+----------------------------+
    %| exact_jacobian            | OT_BOOL         | Use exact Jacobian         |
    %|                           |                 | information                |
    %+---------------------------+-----------------+----------------------------+
    %| f_scale                   | OT_DOUBLEVECTOR | Equation scaling factors   |
    %+---------------------------+-----------------+----------------------------+
    %| iterative_solver          | OT_STRING       | gmres|bcgstab|tfqmr        |
    %+---------------------------+-----------------+----------------------------+
    %| linear_solver_type        | OT_STRING       | dense|banded|iterative|use |
    %|                           |                 | r_defined                  |
    %+---------------------------+-----------------+----------------------------+
    %| lower_bandwidth           | OT_INT          | Lower bandwidth for banded |
    %|                           |                 | linear solvers             |
    %+---------------------------+-----------------+----------------------------+
    %| max_iter                  | OT_INT          | Maximum number of Newton   |
    %|                           |                 | iterations. Putting 0 sets |
    %|                           |                 | the default value of       |
    %|                           |                 | KinSol.                    |
    %+---------------------------+-----------------+----------------------------+
    %| max_krylov                | OT_INT          | Maximum Krylov space       |
    %|                           |                 | dimension                  |
    %+---------------------------+-----------------+----------------------------+
    %| pretype                   | OT_STRING       | Type of preconditioner     |
    %+---------------------------+-----------------+----------------------------+
    %| strategy                  | OT_STRING       | Globalization strategy     |
    %+---------------------------+-----------------+----------------------------+
    %| u_scale                   | OT_DOUBLEVECTOR | Variable scaling factors   |
    %+---------------------------+-----------------+----------------------------+
    %| upper_bandwidth           | OT_INT          | Upper bandwidth for banded |
    %|                           |                 | linear solvers             |
    %+---------------------------+-----------------+----------------------------+
    %| use_preconditioner        | OT_BOOL         | Precondition an iterative  |
    %|                           |                 | solver                     |
    %+---------------------------+-----------------+----------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %fast_newton
    %-----------
    %
    %
    %
    %Implements simple newton iterations to solve an implicit function.
    %
    %>List of available options
    %
    %+------------+-----------+-------------------------------------------------+
    %|     Id     |   Type    |                   Description                   |
    %+============+===========+=================================================+
    %| abstol     | OT_DOUBLE | Stopping criterion tolerance on ||g||__inf)     |
    %+------------+-----------+-------------------------------------------------+
    %| abstolStep | OT_DOUBLE | Stopping criterion tolerance on step size       |
    %+------------+-----------+-------------------------------------------------+
    %| max_iter   | OT_INT    | Maximum number of Newton iterations to perform  |
    %|            |           | before returning.                               |
    %+------------+-----------+-------------------------------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %nlpsol
    %------
    %
    %
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %newton
    %------
    %
    %
    %
    %Implements simple newton iterations to solve an implicit function.
    %
    %>List of available options
    %
    %+-----------------+-----------+--------------------------------------------+
    %|       Id        |   Type    |                Description                 |
    %+=================+===========+============================================+
    %| abstol          | OT_DOUBLE | Stopping criterion tolerance on max(|F|)   |
    %+-----------------+-----------+--------------------------------------------+
    %| abstolStep      | OT_DOUBLE | Stopping criterion tolerance on step size  |
    %+-----------------+-----------+--------------------------------------------+
    %| line_search     | OT_BOOL   | Enable line-search (default: true)         |
    %+-----------------+-----------+--------------------------------------------+
    %| max_iter        | OT_INT    | Maximum number of Newton iterations to     |
    %|                 |           | perform before returning.                  |
    %+-----------------+-----------+--------------------------------------------+
    %| print_iteration | OT_BOOL   | Print information about each iteration     |
    %+-----------------+-----------+--------------------------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %Joel Andersson
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
    %  ROOTFINDER(char name, char solver, struct:MX rfp, struct opts)
    %  ROOTFINDER(char name, char solver, Function f, struct opts)
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(813, varargin{:});
end
