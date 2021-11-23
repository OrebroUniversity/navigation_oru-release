function varargout = nlpsol_out(varargin)
    %NLPSOL_OUT Get output scheme name by index.
    %
    %  {char} = NLPSOL_OUT()
    %  char = NLPSOL_OUT(int ind)
    %
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
    %
    %
    %.......
    %
    %::
    %
    %  NLPSOL_OUT(int ind)
    %
    %
    %
    %Get output scheme name by index.
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
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NLPSOL_OUT()
    %
    %
    %
    %Get NLP solver output scheme of NLP solvers.
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
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(803, varargin{:});
end
