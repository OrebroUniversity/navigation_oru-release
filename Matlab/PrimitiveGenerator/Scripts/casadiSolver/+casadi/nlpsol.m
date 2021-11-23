function varargout = nlpsol(varargin)
    %NLPSOL 
    %
    %  Function = NLPSOL(char name, char solver, Importer compiler, struct opts)
    %  Function = NLPSOL(char name, char solver, NlpBuilder nl, struct opts)
    %  Function = NLPSOL(char name, char solver, struct:SX nlp, struct opts)
    %  Function = NLPSOL(char name, char solver, struct:MX nlp, struct opts)
    %  Function = NLPSOL(char name, char solver, char fname, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  NLPSOL(char name, char solver, Importer compiler, struct opts)
    %  NLPSOL(char name, char solver, NlpBuilder nl, struct opts)
    %  NLPSOL(char name, char solver, struct:MX nlp, struct opts)
    %  NLPSOL(char name, char solver, char fname, struct opts)
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
    %  NLPSOL(char name, char solver, struct:SX nlp, struct opts)
    %
    %
    %
    %Create an NLP solver Creates a solver for the following parametric nonlinear
    %program (NLP):
    %
    %::
    %
    %  min          F(x, p)
    %  x
    %  
    %  subject to
    %  LBX <=   x    <= UBX
    %  LBG <= G(x, p) <= UBG
    %  p  == P
    %  
    %  nx: number of decision variables
    %  ng: number of constraints
    %  np: number of parameters
    %
    %
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
    %| bound_consistenc | OT_BOOL         | Ensure that      | casadi::Nlpsol   |
    %| y                |                 | primal-dual      |                  |
    %|                  |                 | solution is      |                  |
    %|                  |                 | consistent with  |                  |
    %|                  |                 | the bounds       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| calc_f           | OT_BOOL         | Calculate 'f' in | casadi::Nlpsol   |
    %|                  |                 | the Nlpsol base  |                  |
    %|                  |                 | class            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| calc_g           | OT_BOOL         | Calculate 'g' in | casadi::Nlpsol   |
    %|                  |                 | the Nlpsol base  |                  |
    %|                  |                 | class            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| calc_lam_p       | OT_BOOL         | Calculate        | casadi::Nlpsol   |
    %|                  |                 | 'lam_p' in the   |                  |
    %|                  |                 | Nlpsol base      |                  |
    %|                  |                 | class            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| calc_lam_x       | OT_BOOL         | Calculate        | casadi::Nlpsol   |
    %|                  |                 | 'lam_x' in the   |                  |
    %|                  |                 | Nlpsol base      |                  |
    %|                  |                 | class            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| calc_multipliers | OT_BOOL         | Calculate        | casadi::Nlpsol   |
    %|                  |                 | Lagrange         |                  |
    %|                  |                 | multipliers in   |                  |
    %|                  |                 | the Nlpsol base  |                  |
    %|                  |                 | class            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| common_options   | OT_DICT         | Options for      | casadi::OracleFu |
    %|                  |                 | auto-generated   | nction           |
    %|                  |                 | functions        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| discrete         | OT_BOOLVECTOR   | Indicates which  | casadi::Nlpsol   |
    %|                  |                 | of the variables |                  |
    %|                  |                 | are discrete,    |                  |
    %|                  |                 | i.e. integer-    |                  |
    %|                  |                 | valued           |                  |
    %+------------------+-----------------+------------------+------------------+
    %| error_on_fail    | OT_BOOL         | When the         | casadi::Nlpsol   |
    %|                  |                 | numerical        |                  |
    %|                  |                 | process returns  |                  |
    %|                  |                 | unsuccessfully,  |                  |
    %|                  |                 | raise an error   |                  |
    %|                  |                 | (default false). |                  |
    %+------------------+-----------------+------------------+------------------+
    %| eval_errors_fata | OT_BOOL         | When errors      | casadi::Nlpsol   |
    %| l                |                 | occur during     |                  |
    %|                  |                 | evaluation of    |                  |
    %|                  |                 | f,g,...,stop the |                  |
    %|                  |                 | iterations       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| expand           | OT_BOOL         | Replace MX with  | casadi::OracleFu |
    %|                  |                 | SX expressions   | nction           |
    %|                  |                 | in problem       |                  |
    %|                  |                 | formulation      |                  |
    %|                  |                 | [false]          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| ignore_check_vec | OT_BOOL         | If set to true,  | casadi::Nlpsol   |
    %|                  |                 | the input shape  |                  |
    %|                  |                 | of F will not be |                  |
    %|                  |                 | checked.         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| iteration_callba | OT_FUNCTION     | A function that  | casadi::Nlpsol   |
    %| ck               |                 | will be called   |                  |
    %|                  |                 | at each          |                  |
    %|                  |                 | iteration with   |                  |
    %|                  |                 | the solver as    |                  |
    %|                  |                 | input. Check     |                  |
    %|                  |                 | documentation of |                  |
    %|                  |                 | Callback .       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| iteration_callba | OT_BOOL         | If set to true,  | casadi::Nlpsol   |
    %| ck_ignore_errors |                 | errors thrown by |                  |
    %|                  |                 | iteration_callba |                  |
    %|                  |                 | ck will be       |                  |
    %|                  |                 | ignored.         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| iteration_callba | OT_INT          | Only call the    | casadi::Nlpsol   |
    %| ck_step          |                 | callback         |                  |
    %|                  |                 | function every   |                  |
    %|                  |                 | few iterations.  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| min_lam          | OT_DOUBLE       | Minimum allowed  | casadi::Nlpsol   |
    %|                  |                 | multiplier value |                  |
    %+------------------+-----------------+------------------+------------------+
    %| monitor          | OT_STRINGVECTOR | Set of user      | casadi::OracleFu |
    %|                  |                 | problem          | nction           |
    %|                  |                 | functions to be  |                  |
    %|                  |                 | monitored        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| no_nlp_grad      | OT_BOOL         | Prevent the      | casadi::Nlpsol   |
    %|                  |                 | creation of the  |                  |
    %|                  |                 | 'nlp_grad'       |                  |
    %|                  |                 | function         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| oracle_options   | OT_DICT         | Options to be    | casadi::Nlpsol   |
    %|                  |                 | passed to the    |                  |
    %|                  |                 | oracle function  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| sens_linsol      | OT_STRING       | Linear solver    | casadi::Nlpsol   |
    %|                  |                 | used for         |                  |
    %|                  |                 | parametric       |                  |
    %|                  |                 | sensitivities    |                  |
    %|                  |                 | (default 'qr').  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| sens_linsol_opti | OT_DICT         | Linear solver    | casadi::Nlpsol   |
    %| ons              |                 | options used for |                  |
    %|                  |                 | parametric       |                  |
    %|                  |                 | sensitivities.   |                  |
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
    %| verbose_init     | OT_BOOL         | Print out timing | casadi::Nlpsol   |
    %|                  |                 | information      |                  |
    %|                  |                 | about the        |                  |
    %|                  |                 | different stages |                  |
    %|                  |                 | of               |                  |
    %|                  |                 | initialization   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| warn_initial_bou | OT_BOOL         | Warn if the      | casadi::Nlpsol   |
    %| nds              |                 | initial guess    |                  |
    %|                  |                 | does not satisfy |                  |
    %|                  |                 | LBX and UBX      |                  |
    %+------------------+-----------------+------------------+------------------+
    %
    %>Input scheme: casadi::NlpsolInput (NLPSOL_NUM_IN = 8)
    %
    %+---------------+--------+-------------------------------------------------+
    %|   Full name   | Short  |                   Description                   |
    %+===============+========+=================================================+
    %| NLPSOL_X0     | x0     | Decision variables, initial guess (nx x 1)      |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_P      | p      | Value of fixed parameters (np x 1)              |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_LBX    | lbx    | Decision variables lower bound (nx x 1),        |
    %|               |        | default -inf.                                   |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_UBX    | ubx    | Decision variables upper bound (nx x 1),        |
    %|               |        | default +inf.                                   |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_LBG    | lbg    | Constraints lower bound (ng x 1), default -inf. |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_UBG    | ubg    | Constraints upper bound (ng x 1), default +inf. |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_LAM_X0 | lam_x0 | Lagrange multipliers for bounds on X, initial   |
    %|               |        | guess (nx x 1)                                  |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_LAM_G0 | lam_g0 | Lagrange multipliers for bounds on G, initial   |
    %|               |        | guess (ng x 1)                                  |
    %+---------------+--------+-------------------------------------------------+
    %
    %>Output scheme: casadi::NlpsolOutput (NLPSOL_NUM_OUT = 6)
    %
    %+--------------+-------+---------------------------------------------------+
    %|  Full name   | Short |                    Description                    |
    %+==============+=======+===================================================+
    %| NLPSOL_X     | x     | Decision variables at the optimal solution (nx x  |
    %|              |       | 1)                                                |
    %+--------------+-------+---------------------------------------------------+
    %| NLPSOL_F     | f     | Cost function value at the optimal solution (1 x  |
    %|              |       | 1)                                                |
    %+--------------+-------+---------------------------------------------------+
    %| NLPSOL_G     | g     | Constraints function at the optimal solution (ng  |
    %|              |       | x 1)                                              |
    %+--------------+-------+---------------------------------------------------+
    %| NLPSOL_LAM_X | lam_x | Lagrange multipliers for bounds on X at the       |
    %|              |       | solution (nx x 1)                                 |
    %+--------------+-------+---------------------------------------------------+
    %| NLPSOL_LAM_G | lam_g | Lagrange multipliers for bounds on G at the       |
    %|              |       | solution (ng x 1)                                 |
    %+--------------+-------+---------------------------------------------------+
    %| NLPSOL_LAM_P | lam_p | Lagrange multipliers for bounds on P at the       |
    %|              |       | solution (np x 1)                                 |
    %+--------------+-------+---------------------------------------------------+
    %
    %List of plugins
    %===============
    %
    %
    %
    %- AmplInterface
    %
    %- blocksqp
    %
    %- bonmin
    %
    %- ipopt
    %
    %- knitro
    %
    %- snopt
    %
    %- worhp
    %
    %- qrsqp
    %
    %- scpgen
    %
    %- sqpmethod
    %
    %Note: some of the plugins in this list might not be available on your
    %system. Also, there might be extra plugins available to you that are not
    %listed here. You can obtain their documentation with
    %Nlpsol.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %AmplInterface
    %-------------
    %
    %
    %
    %>List of available options
    %
    %+--------+-----------+--------------------+
    %|   Id   |   Type    |    Description     |
    %+========+===========+====================+
    %| solver | OT_STRING | AMPL solver binary |
    %+--------+-----------+--------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %blocksqp
    %--------
    %
    %
    %
    %This is a modified version of blockSQP by Janka et al.
    %
    %Dennis Janka, Joel Andersson
    %
    %>List of available options
    %
    %+----------------------------+-----------+---------------------------------+
    %|             Id             |   Type    |           Description           |
    %+============================+===========+=================================+
    %| block_hess                 | OT_INT    | Blockwise Hessian               |
    %|                            |           | approximation?                  |
    %+----------------------------+-----------+---------------------------------+
    %| col_eps                    | OT_DOUBLE | Epsilon for COL scaling         |
    %|                            |           | strategy                        |
    %+----------------------------+-----------+---------------------------------+
    %| col_tau1                   | OT_DOUBLE | tau1 for COL scaling strategy   |
    %+----------------------------+-----------+---------------------------------+
    %| col_tau2                   | OT_DOUBLE | tau2 for COL scaling strategy   |
    %+----------------------------+-----------+---------------------------------+
    %| conv_strategy              | OT_INT    | Convexification strategy        |
    %+----------------------------+-----------+---------------------------------+
    %| delta                      | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| delta_h0                   | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| eps                        | OT_DOUBLE | Values smaller than this are    |
    %|                            |           | regarded as numerically zero    |
    %+----------------------------+-----------+---------------------------------+
    %| eta                        | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| fallback_scaling           | OT_INT    | If indefinite update is used,   |
    %|                            |           | the type of fallback strategy   |
    %+----------------------------+-----------+---------------------------------+
    %| fallback_update            | OT_INT    | If indefinite update is used,   |
    %|                            |           | the type of fallback strategy   |
    %+----------------------------+-----------+---------------------------------+
    %| gamma_f                    | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| gamma_theta                | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| globalization              | OT_BOOL   | Enable globalization            |
    %+----------------------------+-----------+---------------------------------+
    %| hess_damp                  | OT_INT    | Activate Powell damping for     |
    %|                            |           | BFGS                            |
    %+----------------------------+-----------+---------------------------------+
    %| hess_damp_fac              | OT_DOUBLE | Damping factor for BFGS Powell  |
    %|                            |           | modification                    |
    %+----------------------------+-----------+---------------------------------+
    %| hess_lim_mem               | OT_INT    | Full or limited memory          |
    %+----------------------------+-----------+---------------------------------+
    %| hess_memsize               | OT_INT    | Memory size for L-BFGS updates  |
    %+----------------------------+-----------+---------------------------------+
    %| hess_scaling               | OT_INT    | Scaling strategy for Hessian    |
    %|                            |           | approximation                   |
    %+----------------------------+-----------+---------------------------------+
    %| hess_update                | OT_INT    | Type of Hessian approximation   |
    %+----------------------------+-----------+---------------------------------+
    %| ini_hess_diag              | OT_DOUBLE | Initial Hessian guess: diagonal |
    %|                            |           | matrix diag(iniHessDiag)        |
    %+----------------------------+-----------+---------------------------------+
    %| kappa_f                    | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| kappa_minus                | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| kappa_plus                 | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| kappa_plus_max             | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| kappa_soc                  | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| linsol                     | OT_STRING | The linear solver to be used by |
    %|                            |           | the QP method                   |
    %+----------------------------+-----------+---------------------------------+
    %| max_consec_reduced_steps   | OT_INT    | Maximum number of consecutive   |
    %|                            |           | reduced steps                   |
    %+----------------------------+-----------+---------------------------------+
    %| max_consec_skipped_updates | OT_INT    | Maximum number of consecutive   |
    %|                            |           | skipped updates                 |
    %+----------------------------+-----------+---------------------------------+
    %| max_conv_qp                | OT_INT    | How many additional QPs may be  |
    %|                            |           | solved for convexification per  |
    %|                            |           | iteration?                      |
    %+----------------------------+-----------+---------------------------------+
    %| max_it_qp                  | OT_INT    | Maximum number of QP iterations |
    %|                            |           | per SQP iteration               |
    %+----------------------------+-----------+---------------------------------+
    %| max_iter                   | OT_INT    | Maximum number of SQP           |
    %|                            |           | iterations                      |
    %+----------------------------+-----------+---------------------------------+
    %| max_line_search            | OT_INT    | Maximum number of steps in line |
    %|                            |           | search                          |
    %+----------------------------+-----------+---------------------------------+
    %| max_soc_iter               | OT_INT    | Maximum number of SOC line      |
    %|                            |           | search iterations               |
    %+----------------------------+-----------+---------------------------------+
    %| max_time_qp                | OT_DOUBLE | Maximum number of time in       |
    %|                            |           | seconds per QP solve per SQP    |
    %|                            |           | iteration                       |
    %+----------------------------+-----------+---------------------------------+
    %| nlinfeastol                | OT_DOUBLE | Nonlinear feasibility tolerance |
    %+----------------------------+-----------+---------------------------------+
    %| obj_lo                     | OT_DOUBLE | Lower bound on objective        |
    %|                            |           | function [-inf]                 |
    %+----------------------------+-----------+---------------------------------+
    %| obj_up                     | OT_DOUBLE | Upper bound on objective        |
    %|                            |           | function [inf]                  |
    %+----------------------------+-----------+---------------------------------+
    %| opttol                     | OT_DOUBLE | Optimality tolerance            |
    %+----------------------------+-----------+---------------------------------+
    %| print_header               | OT_BOOL   | Print solver header at startup  |
    %+----------------------------+-----------+---------------------------------+
    %| print_iteration            | OT_BOOL   | Print SQP iterations            |
    %+----------------------------+-----------+---------------------------------+
    %| print_maxit_reached        | OT_BOOL   | Print error when maximum number |
    %|                            |           | of SQP iterations reached       |
    %+----------------------------+-----------+---------------------------------+
    %| qp_init                    | OT_BOOL   | Use warmstarting                |
    %+----------------------------+-----------+---------------------------------+
    %| qpsol                      | OT_STRING | The QP solver to be used by the |
    %|                            |           | SQP method                      |
    %+----------------------------+-----------+---------------------------------+
    %| qpsol_options              | OT_DICT   | Options to be passed to the QP  |
    %|                            |           | solver                          |
    %+----------------------------+-----------+---------------------------------+
    %| restore_feas               | OT_BOOL   | Use feasibility restoration     |
    %|                            |           | phase                           |
    %+----------------------------+-----------+---------------------------------+
    %| rho                        | OT_DOUBLE | Feasibility restoration phase   |
    %|                            |           | parameter                       |
    %+----------------------------+-----------+---------------------------------+
    %| s_f                        | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| s_theta                    | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| schur                      | OT_BOOL   | Use qpOASES Schur compliment    |
    %|                            |           | approach                        |
    %+----------------------------+-----------+---------------------------------+
    %| skip_first_globalization   | OT_BOOL   | No globalization strategy in    |
    %|                            |           | first iteration                 |
    %+----------------------------+-----------+---------------------------------+
    %| theta_max                  | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| theta_min                  | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| warmstart                  | OT_BOOL   | Use warmstarting                |
    %+----------------------------+-----------+---------------------------------+
    %| which_second_derv          | OT_INT    | For which block should second   |
    %|                            |           | derivatives be provided by the  |
    %|                            |           | user                            |
    %+----------------------------+-----------+---------------------------------+
    %| zeta                       | OT_DOUBLE | Feasibility restoration phase   |
    %|                            |           | parameter                       |
    %+----------------------------+-----------+---------------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %bonmin
    %------
    %
    %
    %
    %When in warmstart mode, output NLPSOL_LAM_X may be used as input
    %
    %NOTE: Even when max_iter == 0, it is not guaranteed that input(NLPSOL_X0) ==
    %output(NLPSOL_X). Indeed if bounds on X or constraints are unmet, they will
    %differ.
    %
    %For a good tutorial on BONMIN,
    %seehttp://drops.dagstuhl.de/volltexte/2009/2089/pdf/09061.WaechterAndreas.Paper.2089.pdf
    %
    %A good resource about the algorithms in BONMIN is: Wachter and L. T.
    %Biegler, On the Implementation of an Interior-Point Filter Line-Search
    %Algorithm for Large-Scale Nonlinear Programming, Mathematical Programming
    %106(1), pp. 25-57, 2006 (As Research Report RC 23149, IBM T. J. Watson
    %Research Center, Yorktown, USA
    %
    %Caveats: with default options, multipliers for the decision variables are
    %wrong for equality constraints. Change the 'fixed_variable_treatment' to
    %'make_constraint' or 'relax_bounds' to obtain correct results.
    %
    %>List of available options
    %
    %+-------------------------+-----------------------+------------------------+
    %|           Id            |         Type          |      Description       |
    %+=========================+=======================+========================+
    %| bonmin                  | OT_DICT               | Options to be passed   |
    %|                         |                       | to BONMIN              |
    %+-------------------------+-----------------------+------------------------+
    %| con_integer_md          | OT_DICT               | Integer metadata (a    |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of integers) about     |
    %|                         |                       | constraints to be      |
    %|                         |                       | passed to BONMIN       |
    %+-------------------------+-----------------------+------------------------+
    %| con_numeric_md          | OT_DICT               | Numeric metadata (a    |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of reals) about        |
    %|                         |                       | constraints to be      |
    %|                         |                       | passed to BONMIN       |
    %+-------------------------+-----------------------+------------------------+
    %| con_string_md           | OT_DICT               | String metadata (a     |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of strings) about      |
    %|                         |                       | constraints to be      |
    %|                         |                       | passed to BONMIN       |
    %+-------------------------+-----------------------+------------------------+
    %| grad_f                  | OT_FUNCTION           | Function for           |
    %|                         |                       | calculating the        |
    %|                         |                       | gradient of the        |
    %|                         |                       | objective (column,     |
    %|                         |                       | autogenerated by       |
    %|                         |                       | default)               |
    %+-------------------------+-----------------------+------------------------+
    %| grad_f_options          | OT_DICT               | Options for the        |
    %|                         |                       | autogenerated gradient |
    %|                         |                       | of the objective.      |
    %+-------------------------+-----------------------+------------------------+
    %| hess_lag                | OT_FUNCTION           | Function for           |
    %|                         |                       | calculating the        |
    %|                         |                       | Hessian of the         |
    %|                         |                       | Lagrangian             |
    %|                         |                       | (autogenerated by      |
    %|                         |                       | default)               |
    %+-------------------------+-----------------------+------------------------+
    %| hess_lag_options        | OT_DICT               | Options for the        |
    %|                         |                       | autogenerated Hessian  |
    %|                         |                       | of the Lagrangian.     |
    %+-------------------------+-----------------------+------------------------+
    %| jac_g                   | OT_FUNCTION           | Function for           |
    %|                         |                       | calculating the        |
    %|                         |                       | Jacobian of the        |
    %|                         |                       | constraints            |
    %|                         |                       | (autogenerated by      |
    %|                         |                       | default)               |
    %+-------------------------+-----------------------+------------------------+
    %| jac_g_options           | OT_DICT               | Options for the        |
    %|                         |                       | autogenerated Jacobian |
    %|                         |                       | of the constraints.    |
    %+-------------------------+-----------------------+------------------------+
    %| pass_nonlinear_constrai | OT_BOOL               | Pass list of           |
    %| nts                     |                       | constraints entering   |
    %|                         |                       | nonlinearly to BONMIN  |
    %+-------------------------+-----------------------+------------------------+
    %| pass_nonlinear_variable | OT_BOOL               | Pass list of variables |
    %| s                       |                       | entering nonlinearly   |
    %|                         |                       | to BONMIN              |
    %+-------------------------+-----------------------+------------------------+
    %| sos1_groups             | OT_INTVECTORVECTOR    | Options for the        |
    %|                         |                       | autogenerated gradient |
    %|                         |                       | of the objective.      |
    %+-------------------------+-----------------------+------------------------+
    %| sos1_priorities         | OT_INTVECTOR          | Options for the        |
    %|                         |                       | autogenerated gradient |
    %|                         |                       | of the objective.      |
    %+-------------------------+-----------------------+------------------------+
    %| sos1_weights            | OT_DOUBLEVECTORVECTOR | Options for the        |
    %|                         |                       | autogenerated gradient |
    %|                         |                       | of the objective.      |
    %+-------------------------+-----------------------+------------------------+
    %| var_integer_md          | OT_DICT               | Integer metadata (a    |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of integers) about     |
    %|                         |                       | variables to be passed |
    %|                         |                       | to BONMIN              |
    %+-------------------------+-----------------------+------------------------+
    %| var_numeric_md          | OT_DICT               | Numeric metadata (a    |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of reals) about        |
    %|                         |                       | variables to be passed |
    %|                         |                       | to BONMIN              |
    %+-------------------------+-----------------------+------------------------+
    %| var_string_md           | OT_DICT               | String metadata (a     |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of strings) about      |
    %|                         |                       | variables to be passed |
    %|                         |                       | to BONMIN              |
    %+-------------------------+-----------------------+------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %ipopt
    %-----
    %
    %
    %
    %When in warmstart mode, output NLPSOL_LAM_X may be used as input
    %
    %NOTE: Even when max_iter == 0, it is not guaranteed that input(NLPSOL_X0) ==
    %output(NLPSOL_X). Indeed if bounds on X or constraints are unmet, they will
    %differ.
    %
    %For a good tutorial on IPOPT,
    %seehttp://drops.dagstuhl.de/volltexte/2009/2089/pdf/09061.WaechterAndreas.Paper.2089.pdf
    %
    %A good resource about the algorithms in IPOPT is: Wachter and L. T. Biegler,
    %On the Implementation of an Interior-Point Filter Line-Search Algorithm for
    %Large-Scale Nonlinear Programming, Mathematical Programming 106(1), pp.
    %25-57, 2006 (As Research Report RC 23149, IBM T. J. Watson Research Center,
    %Yorktown, USA
    %
    %Caveats: with default options, multipliers for the decision variables are
    %wrong for equality constraints. Change the 'fixed_variable_treatment' to
    %'make_constraint' or 'relax_bounds' to obtain correct results.
    %
    %>List of available options
    %
    %+--------------------------+-------------+---------------------------------+
    %|            Id            |    Type     |           Description           |
    %+==========================+=============+=================================+
    %| clip_inactive_lam        | OT_BOOL     | Explicitly set Lagrange         |
    %|                          |             | multipliers to 0 when bound is  |
    %|                          |             | deemed inactive (default:       |
    %|                          |             | false).                         |
    %+--------------------------+-------------+---------------------------------+
    %| con_integer_md           | OT_DICT     | Integer metadata (a dictionary  |
    %|                          |             | with lists of integers) about   |
    %|                          |             | constraints to be passed to     |
    %|                          |             | IPOPT                           |
    %+--------------------------+-------------+---------------------------------+
    %| con_numeric_md           | OT_DICT     | Numeric metadata (a dictionary  |
    %|                          |             | with lists of reals) about      |
    %|                          |             | constraints to be passed to     |
    %|                          |             | IPOPT                           |
    %+--------------------------+-------------+---------------------------------+
    %| con_string_md            | OT_DICT     | String metadata (a dictionary   |
    %|                          |             | with lists of strings) about    |
    %|                          |             | constraints to be passed to     |
    %|                          |             | IPOPT                           |
    %+--------------------------+-------------+---------------------------------+
    %| convexify_margin         | OT_DOUBLE   | When using a convexification    |
    %|                          |             | strategy, make sure that the    |
    %|                          |             | smallest eigenvalue is at least |
    %|                          |             | this (default: 1e-7).           |
    %+--------------------------+-------------+---------------------------------+
    %| convexify_strategy       | OT_STRING   | NONE|regularize|eigen-reflect   |
    %|                          |             | |eigen-clip. Strategy to        |
    %|                          |             | convexify the Lagrange Hessian  |
    %|                          |             | before passing it to the        |
    %|                          |             | solver.                         |
    %+--------------------------+-------------+---------------------------------+
    %| grad_f                   | OT_FUNCTION | Function for calculating the    |
    %|                          |             | gradient of the objective       |
    %|                          |             | (column, autogenerated by       |
    %|                          |             | default)                        |
    %+--------------------------+-------------+---------------------------------+
    %| hess_lag                 | OT_FUNCTION | Function for calculating the    |
    %|                          |             | Hessian of the Lagrangian       |
    %|                          |             | (autogenerated by default)      |
    %+--------------------------+-------------+---------------------------------+
    %| inactive_lam_strategy    | OT_STRING   | Strategy to detect if a bound   |
    %|                          |             | is inactive. RELTOL: use        |
    %|                          |             | solver-defined constraint       |
    %|                          |             | tolerance *                     |
    %|                          |             | inactive_lam_value|abstol: use  |
    %|                          |             | inactive_lam_value              |
    %+--------------------------+-------------+---------------------------------+
    %| inactive_lam_value       | OT_DOUBLE   | Value used in                   |
    %|                          |             | inactive_lam_strategy (default: |
    %|                          |             | 10).                            |
    %+--------------------------+-------------+---------------------------------+
    %| ipopt                    | OT_DICT     | Options to be passed to IPOPT   |
    %+--------------------------+-------------+---------------------------------+
    %| jac_g                    | OT_FUNCTION | Function for calculating the    |
    %|                          |             | Jacobian of the constraints     |
    %|                          |             | (autogenerated by default)      |
    %+--------------------------+-------------+---------------------------------+
    %| max_iter_eig             | OT_DOUBLE   | Maximum number of iterations to |
    %|                          |             | compute an eigenvalue           |
    %|                          |             | decomposition (default: 50).    |
    %+--------------------------+-------------+---------------------------------+
    %| pass_nonlinear_variables | OT_BOOL     | Pass list of variables entering |
    %|                          |             | nonlinearly to IPOPT            |
    %+--------------------------+-------------+---------------------------------+
    %| var_integer_md           | OT_DICT     | Integer metadata (a dictionary  |
    %|                          |             | with lists of integers) about   |
    %|                          |             | variables to be passed to IPOPT |
    %+--------------------------+-------------+---------------------------------+
    %| var_numeric_md           | OT_DICT     | Numeric metadata (a dictionary  |
    %|                          |             | with lists of reals) about      |
    %|                          |             | variables to be passed to IPOPT |
    %+--------------------------+-------------+---------------------------------+
    %| var_string_md            | OT_DICT     | String metadata (a dictionary   |
    %|                          |             | with lists of strings) about    |
    %|                          |             | variables to be passed to IPOPT |
    %+--------------------------+-------------+---------------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %knitro
    %------
    %
    %
    %
    %KNITRO interface
    %
    %>List of available options
    %
    %+--------------------------+--------------------+--------------------------+
    %|            Id            |        Type        |       Description        |
    %+==========================+====================+==========================+
    %| complem_variables        | OT_INTVECTORVECTOR | List of complementary    |
    %|                          |                    | constraints on simple    |
    %|                          |                    | bounds. Pair (i, j)      |
    %|                          |                    | encodes complementarity  |
    %|                          |                    | between the bounds on    |
    %|                          |                    | variable i and variable  |
    %|                          |                    | j.                       |
    %+--------------------------+--------------------+--------------------------+
    %| contype                  | OT_INTVECTOR       | Type of constraint       |
    %+--------------------------+--------------------+--------------------------+
    %| detect_linear_constraint | OT_BOOL            | Detect type of           |
    %| s                        |                    | constraints              |
    %+--------------------------+--------------------+--------------------------+
    %| knitro                   | OT_DICT            | Options to be passed to  |
    %|                          |                    | KNITRO                   |
    %+--------------------------+--------------------+--------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %snopt
    %-----
    %
    %
    %
    %SNOPT interface
    %
    %>List of available options
    %
    %+-------+-----------+---------------------------------------------+
    %|  Id   |   Type    |                 Description                 |
    %+=======+===========+=============================================+
    %| snopt | OT_DICT   | Options to be passed to SNOPT               |
    %+-------+-----------+---------------------------------------------+
    %| start | OT_STRING | Warm-start options for Worhp: cold|warm|hot |
    %+-------+-----------+---------------------------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %worhp
    %-----
    %
    %
    %
    %WORHP interface
    %
    %Designed for Worhp 1.12
    %
    %>List of available options
    %
    %+-------+---------+-------------------------------+
    %|  Id   |  Type   |          Description          |
    %+=======+=========+===============================+
    %| worhp | OT_DICT | Options to be passed to WORHP |
    %+-------+---------+-------------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %qrsqp
    %-----
    %
    %
    %
    %A textbook SQPMethod
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %scpgen
    %------
    %
    %
    %
    %A structure-exploiting sequential quadratic programming (to be come
    %sequential convex programming) method for nonlinear programming.
    %
    %>List of available options
    %
    %+-----------------------+-----------------+--------------------------------+
    %|          Id           |      Type       |          Description           |
    %+=======================+=================+================================+
    %| beta                  | OT_DOUBLE       | Line-search parameter,         |
    %|                       |                 | restoration factor of stepsize |
    %+-----------------------+-----------------+--------------------------------+
    %| c1                    | OT_DOUBLE       | Armijo condition, coefficient  |
    %|                       |                 | of decrease in merit           |
    %+-----------------------+-----------------+--------------------------------+
    %| codegen               | OT_BOOL         | C-code generation              |
    %+-----------------------+-----------------+--------------------------------+
    %| hessian_approximation | OT_STRING       | gauss-newton|exact             |
    %+-----------------------+-----------------+--------------------------------+
    %| lbfgs_memory          | OT_INT          | Size of L-BFGS memory.         |
    %+-----------------------+-----------------+--------------------------------+
    %| max_iter              | OT_INT          | Maximum number of SQP          |
    %|                       |                 | iterations                     |
    %+-----------------------+-----------------+--------------------------------+
    %| max_iter_ls           | OT_INT          | Maximum number of linesearch   |
    %|                       |                 | iterations                     |
    %+-----------------------+-----------------+--------------------------------+
    %| merit_memsize         | OT_INT          | Size of memory to store        |
    %|                       |                 | history of merit function      |
    %|                       |                 | values                         |
    %+-----------------------+-----------------+--------------------------------+
    %| merit_start           | OT_DOUBLE       | Lower bound for the merit      |
    %|                       |                 | function parameter             |
    %+-----------------------+-----------------+--------------------------------+
    %| name_x                | OT_STRINGVECTOR | Names of the variables.        |
    %+-----------------------+-----------------+--------------------------------+
    %| print_header          | OT_BOOL         | Print the header with problem  |
    %|                       |                 | statistics                     |
    %+-----------------------+-----------------+--------------------------------+
    %| print_x               | OT_INTVECTOR    | Which variables to print.      |
    %+-----------------------+-----------------+--------------------------------+
    %| qpsol                 | OT_STRING       | The QP solver to be used by    |
    %|                       |                 | the SQP method                 |
    %+-----------------------+-----------------+--------------------------------+
    %| qpsol_options         | OT_DICT         | Options to be passed to the QP |
    %|                       |                 | solver                         |
    %+-----------------------+-----------------+--------------------------------+
    %| reg_threshold         | OT_DOUBLE       | Threshold for the              |
    %|                       |                 | regularization.                |
    %+-----------------------+-----------------+--------------------------------+
    %| regularize            | OT_BOOL         | Automatic regularization of    |
    %|                       |                 | Lagrange Hessian.              |
    %+-----------------------+-----------------+--------------------------------+
    %| tol_du                | OT_DOUBLE       | Stopping criterion for dual    |
    %|                       |                 | infeasability                  |
    %+-----------------------+-----------------+--------------------------------+
    %| tol_pr                | OT_DOUBLE       | Stopping criterion for primal  |
    %|                       |                 | infeasibility                  |
    %+-----------------------+-----------------+--------------------------------+
    %| tol_pr_step           | OT_DOUBLE       | Stopping criterion for the     |
    %|                       |                 | step size                      |
    %+-----------------------+-----------------+--------------------------------+
    %| tol_reg               | OT_DOUBLE       | Stopping criterion for         |
    %|                       |                 | regularization                 |
    %+-----------------------+-----------------+--------------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %sqpmethod
    %---------
    %
    %
    %
    %A textbook SQPMethod
    %
    %>List of available options
    %
    %+-----------------------+-------------+------------------------------------+
    %|          Id           |    Type     |            Description             |
    %+=======================+=============+====================================+
    %| beta                  | OT_DOUBLE   | Line-search parameter, restoration |
    %|                       |             | factor of stepsize                 |
    %+-----------------------+-------------+------------------------------------+
    %| c1                    | OT_DOUBLE   | Armijo condition, coefficient of   |
    %|                       |             | decrease in merit                  |
    %+-----------------------+-------------+------------------------------------+
    %| convexify_margin      | OT_DOUBLE   | When using a convexification       |
    %|                       |             | strategy, make sure that the       |
    %|                       |             | smallest eigenvalue is at least    |
    %|                       |             | this (default: 1e-7).              |
    %+-----------------------+-------------+------------------------------------+
    %| convexify_strategy    | OT_STRING   | NONE|regularize|eigen-reflect      |
    %|                       |             | |eigen-clip. Strategy to convexify |
    %|                       |             | the Lagrange Hessian before        |
    %|                       |             | passing it to the solver.          |
    %+-----------------------+-------------+------------------------------------+
    %| hess_lag              | OT_FUNCTION | Function for calculating the       |
    %|                       |             | Hessian of the Lagrangian          |
    %|                       |             | (autogenerated by default)         |
    %+-----------------------+-------------+------------------------------------+
    %| hessian_approximation | OT_STRING   | limited-memory|exact               |
    %+-----------------------+-------------+------------------------------------+
    %| jac_fg                | OT_FUNCTION | Function for calculating the       |
    %|                       |             | gradient of the objective and      |
    %|                       |             | Jacobian of the constraints        |
    %|                       |             | (autogenerated by default)         |
    %+-----------------------+-------------+------------------------------------+
    %| lbfgs_memory          | OT_INT      | Size of L-BFGS memory.             |
    %+-----------------------+-------------+------------------------------------+
    %| max_iter              | OT_INT      | Maximum number of SQP iterations   |
    %+-----------------------+-------------+------------------------------------+
    %| max_iter_eig          | OT_DOUBLE   | Maximum number of iterations to    |
    %|                       |             | compute an eigenvalue              |
    %|                       |             | decomposition (default: 50).       |
    %+-----------------------+-------------+------------------------------------+
    %| max_iter_ls           | OT_INT      | Maximum number of linesearch       |
    %|                       |             | iterations                         |
    %+-----------------------+-------------+------------------------------------+
    %| merit_memory          | OT_INT      | Size of memory to store history of |
    %|                       |             | merit function values              |
    %+-----------------------+-------------+------------------------------------+
    %| min_iter              | OT_INT      | Minimum number of SQP iterations   |
    %+-----------------------+-------------+------------------------------------+
    %| min_step_size         | OT_DOUBLE   | The size (inf-norm) of the step    |
    %|                       |             | size should not become smaller     |
    %|                       |             | than this.                         |
    %+-----------------------+-------------+------------------------------------+
    %| print_header          | OT_BOOL     | Print the header with problem      |
    %|                       |             | statistics                         |
    %+-----------------------+-------------+------------------------------------+
    %| print_iteration       | OT_BOOL     | Print the iterations               |
    %+-----------------------+-------------+------------------------------------+
    %| print_status          | OT_BOOL     | Print a status message after       |
    %|                       |             | solving                            |
    %+-----------------------+-------------+------------------------------------+
    %| qpsol                 | OT_STRING   | The QP solver to be used by the    |
    %|                       |             | SQP method [qpoases]               |
    %+-----------------------+-------------+------------------------------------+
    %| qpsol_options         | OT_DICT     | Options to be passed to the QP     |
    %|                       |             | solver                             |
    %+-----------------------+-------------+------------------------------------+
    %| tol_du                | OT_DOUBLE   | Stopping criterion for dual        |
    %|                       |             | infeasability                      |
    %+-----------------------+-------------+------------------------------------+
    %| tol_pr                | OT_DOUBLE   | Stopping criterion for primal      |
    %|                       |             | infeasibility                      |
    %+-----------------------+-------------+------------------------------------+
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
  [varargout{1:nargout}] = casadiMEX(801, varargin{:});
end
